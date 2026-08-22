package simulator

import (
	"fmt"
	"math"
	"math/rand"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/propulsion"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Двигательная установка ступени.
//
// Связывает баки, двигатели и полётную модель. Тяга и удельный импульс больше
// не задаются таблицей: они получаются из давления в камере, коэффициента тяги
// и площади критического сечения, которые в свою очередь зависят от оборотов
// турбонасоса, давления в баках и температуры компонентов.
//
// Масса ракеты разложена на четыре составляющие — конструкция, полезная
// нагрузка, горючее и окислитель. Остатки компонентов не хранятся отдельно,
// а сводятся с начальной заправкой через накопленный расход и утечку.
// -----------------------------------------------------------------------------

// PropulsionSystem — баки и двигатели одной ступени.
type PropulsionSystem struct {
	FuelTank *propulsion.Tank
	OxTank   *propulsion.Tank

	Engines []*propulsion.Engine

	// StructureMass — сухая масса конструкции ступени со всем, что она везёт,
	// кроме полезной нагрузки и компонентов топлива, кг.
	StructureMass float64

	// PayloadMass — масса полезной нагрузки, кг.
	PayloadMass float64

	// Суммарные показатели по всем работающим двигателям.
	TotalThrust     float64 // Н
	TotalFuelFlow   float64 // кг/с
	TotalOxFlow     float64 // кг/с
	SpecificImpulse float64 // с

	// ThrottleDemand — требуемый уровень тяги ступени, доля номинала.
	ThrottleDemand float64

	// Commissioned — сколько камер положено работать по плану полёта.
	//
	// Ноль означает «все установленные». Отличие от числа работающих в том,
	// что намеренно погашенная камера отказом не считается: на посадке корабль
	// садится тремя камерами из шести, и добирать «потерю» оставшимся не нужно.
	Commissioned int

	// EngineThrottle — уровень тяги, поданный на каждый работающий двигатель.
	//
	// При отказе одного двигателя он выше требуемого: оставшиеся пытаются
	// добрать тягу за выбывшего в пределах аттестованного запаса.
	EngineThrottle float64

	// EnginesOut — число двигателей, не создающих тяги.
	EnginesOut int

	// ThrustDeficit — недобор тяги против требуемого, доля.
	//
	// Ноль означает, что потеря полностью скомпенсирована. При отказе одного
	// двигателя из девяти дефицит остаётся: восемь двигателей с восьмипроцентным
	// запасом дают 8.64 номинала вместо девяти.
	ThrustDeficit float64

	// CoolingValve и MixtureTrim позволяют вручную нарушить штатную работу,
	// чтобы проследить причинные связи. Единица — штатное положение.
	CoolingValve float64
	MixtureTrim  float64

	rng *rand.Rand
}

// StageProps — параметры двигательной установки ступени.
type StageProps struct {
	// StructureMass — сухая масса конструкции, кг.
	StructureMass float64

	// PayloadMass — масса полезной нагрузки, кг.
	PayloadMass float64

	// PropellantMass — суммарная масса компонентов, кг.
	PropellantMass float64

	// MixtureRatio — проектное соотношение компонентов при заправке.
	MixtureRatio float64

	// EngineCount — число двигателей основного типа.
	EngineCount int

	// EngineGroups — дополнительные группы двигателей с другой конфигурацией.
	//
	// На ступени не обязательно стоят одинаковые двигатели: у второй ступени
	// сверхтяжёлого носителя часть камер с атмосферными соплами, чтобы можно
	// было садиться, а часть — с вакуумными, ради удельного импульса.
	// Пустой список означает, что все двигатели одного типа.
	EngineGroups []vehicle.EngineGroup

	// StagePrefix — обозначение ступени для идентификаторов двигателей,
	// например "S1". Идентификаторы обязаны быть уникальными в пределах
	// всего полёта, иначе двигатель второй ступени переиспользовал бы метку
	// первой, и на графиках это выглядело бы как повторный запуск.
	StagePrefix string

	// EngineConfig — конфигурация двигателя.
	EngineConfig propulsion.EngineConfig

	// FuelTankVolumeMargin — запас объёма бака сверх объёма заправки.
	FuelTankVolumeMargin float64

	// TankHeight — высота бака, м. Задаёт гидростатический напор.
	FuelTankHeight float64
	OxTankHeight   float64

	// TankPressure — поддерживаемое давление наддува, Па.
	FuelTankPressure float64
	OxTankPressure   float64

	// PressurantMass — запас газа наддува на бак, кг.
	FuelPressurantMass float64
	OxPressurantMass   float64

	// LeakArea — площадь неплотности бака, м². Ноль означает герметичность.
	FuelLeakArea float64
	OxLeakArea   float64
}

// fuelHeatInflow возвращает приток тепла в бак горючего, Вт.
//
// Криогенное горючее кипит в баке так же, как кислород, и требует наддува
// заметно интенсивнее, чем керосин при комнатной температуре.
func fuelHeatInflow(fuel propulsion.Propellant) float64 {
	if fuel.Cryogenic {
		return 7000
	}
	return 1500
}

// NewPropulsionSystem собирает двигательную установку ступени.
func NewPropulsionSystem(p StageProps, rng *rand.Rand, ambientTemperature float64) *PropulsionSystem {
	// Компоненты берутся из конфигурации двигателя, а не задаются здесь:
	// на метане и на керосине это разные жидкости с разной плотностью,
	// температурой кипения и давлением насыщенных паров, и бак обязан
	// об этом знать.
	fuel := p.EngineConfig.Pair.Fuel
	ox := p.EngineConfig.Pair.Oxidizer
	if fuel.Name == "" {
		fuel = propulsion.RP1()
	}
	if ox.Name == "" {
		ox = propulsion.LOX()
	}

	// Криогенное горючее заправляется у температуры кипения, некриогенное —
	// при температуре окружающего воздуха.
	fuelFillTemp := 288.0
	if fuel.Cryogenic {
		fuelFillTemp = fuel.BoilingPoint - 5
	}

	// Заправка делится по тому соотношению, с которым двигатель реально
	// отбирает компоненты из баков, а не по соотношению в камере: разницу
	// создаёт газогенератор, работающий почти на чистом горючем.
	mr := p.EngineConfig.OverallMixtureRatio()
	if mr <= 0 {
		mr = p.MixtureRatio
	}
	if mr <= 0 {
		mr = 2.34
	}
	fuelMass := p.PropellantMass / (1 + mr)
	oxMass := p.PropellantMass - fuelMass

	margin := p.FuelTankVolumeMargin
	if margin <= 0 {
		margin = 1.04
	}

	fuelVolume := fuelMass / fuel.Density(fuelFillTemp) * margin
	oxVolume := oxMass / ox.Density(85) * margin

	fuelHeight := p.FuelTankHeight
	if fuelHeight <= 0 {
		fuelHeight = 12
	}
	oxHeight := p.OxTankHeight
	if oxHeight <= 0 {
		oxHeight = 22
	}

	// Сопротивление магистралей подбирается под расход этого носителя.
	//
	// Задавать его числом нельзя: у сверхтяжёлой ракеты секундный расход
	// впятеро больше, а потери растут как квадрат расхода. Значение, дававшее
	// на Falcon 9 несколько килопаскалей, здесь съедало бы весь наддув,
	// давление на входе насосов обращалось в ноль и двигатели не запускались.
	//
	// Поэтому задаётся не сопротивление, а потери на номинале — восемь
	// килопаскалей, — и уже из них считается коэффициент:
	//
	//	R = Δp·ρ / ṁ²
	const nominalLineDrop = 8000.0

	engineFlow := p.EngineConfig.NominalMassFlow()
	totalCount := p.EngineCount
	for _, g := range p.EngineGroups {
		totalCount += g.Count
	}
	designFlow := engineFlow * float64(totalCount)
	designFuelFlow := designFlow / (1 + mr)
	designOxFlow := designFlow - designFuelFlow

	// Расход наддува должен покрывать освобождающийся объём с запасом:
	// газ занимает место ушедшей жидкости, иначе давление в баке падает,
	// вслед за ним падает давление на входе насосов, и двигатели гаснут.
	pressurantFlow := func(flow, density float64) float64 {
		if flow <= 0 || density <= 0 {
			return 0.35
		}
		const heliumR, gasTemp, margin = 2077.0, 200.0, 1.6
		gasDensity := p.FuelTankPressure / (heliumR * gasTemp)
		if gasDensity <= 0 {
			gasDensity = 0.8
		}
		return flow / density * gasDensity * margin
	}

	lineResistance := func(flow, density float64) float64 {
		if flow <= 0 || density <= 0 {
			return 9
		}
		return nominalLineDrop * density / (flow * flow)
	}

	sys := &PropulsionSystem{
		StructureMass: p.StructureMass,
		PayloadMass:   p.PayloadMass,
		CoolingValve:  1,
		MixtureTrim:   1,
		rng:           rng,
	}

	sys.FuelTank = propulsion.NewTank(propulsion.TankConfig{
		Name:                  "fuel",
		Volume:                fuelVolume,
		Height:                fuelHeight,
		CrossSection:          fuelVolume / fuelHeight,
		InitialMass:           fuelMass,
		InitialTemperature:    fuelFillTemp,
		TargetPressure:        p.FuelTankPressure,
		LineResistance:        lineResistance(designFuelFlow, fuel.Density(fuelFillTemp)),
		LeakArea:              p.FuelLeakArea,
		HeatInflow:            fuelHeatInflow(fuel),
		SloshDamping:          0.03,
		PressurantMass:        p.FuelPressurantMass,
		MaxPressurantFlow:     pressurantFlow(designFuelFlow, fuel.Density(fuelFillTemp)),
		PressurantPressure:    2.1e7,
		PressurantTemperature: 200,
	}, fuel)

	sys.OxTank = propulsion.NewTank(propulsion.TankConfig{
		Name:         "ox",
		Volume:       oxVolume,
		Height:       oxHeight,
		CrossSection: oxVolume / oxHeight,
		InitialMass:  oxMass,
		// Кислород заправляется переохлаждённым: при температуре кипения
		// насос немедленно сорвался бы в кавитацию.
		InitialTemperature:    85,
		TargetPressure:        p.OxTankPressure,
		LineResistance:        lineResistance(designOxFlow, ox.Density(85)),
		LeakArea:              p.OxLeakArea,
		HeatInflow:            9000,
		SloshDamping:          0.025,
		PressurantMass:        p.OxPressurantMass,
		MaxPressurantFlow:     pressurantFlow(designOxFlow, ox.Density(85)),
		PressurantPressure:    2.1e7,
		PressurantTemperature: 200,
	}, ox)

	groups := []vehicle.EngineGroup{{Count: p.EngineCount, Config: p.EngineConfig}}
	groups = append(groups, p.EngineGroups...)

	prefix := p.StagePrefix
	if prefix == "" {
		prefix = "S1"
	}

	for _, g := range groups {
		sys.addEngineGroup(g, prefix, rng, ambientTemperature)
	}

	return sys
}

// addEngineGroup добавляет в ступень группу однотипных двигателей.
func (sys *PropulsionSystem) addEngineGroup(g vehicle.EngineGroup, prefix string,
	rng *rand.Rand, ambientTemperature float64) {

	for i := 0; i < g.Count; i++ {
		// Каждый двигатель — отдельное изделие. Приёмочные испытания
		// разбраковывают их в допуске, но не делают одинаковыми: полнота
		// сгорания, КПД насосов и остаточный дисбаланс ротора у каждого свои.
		// Без этого разброса все агрегаты блока вели бы себя побитово
		// одинаково, и отказ одного было бы не отличить от отказа всех.
		cfg := g.Config
		cfg.Chamber.BaselineCStarEfficiency *= 1 + rng.NormFloat64()*0.004
		cfg.Turbopump.FuelPump.Efficiency *= 1 + rng.NormFloat64()*0.01
		cfg.Turbopump.OxPump.Efficiency *= 1 + rng.NormFloat64()*0.01
		cfg.Turbopump.RotorImbalance *= 1 + rng.NormFloat64()*0.15
		cfg.Turbopump.GasGeneratorFlow *= 1 + rng.NormFloat64()*0.006

		e := propulsion.NewEngine(cfg, rng, ambientTemperature)
		e.ID = fmt.Sprintf("%s-%s%d", prefix, g.Suffix, i+1)

		// Подробный спектральный анализ ведётся по одному двигателю ступени:
		// у остальных картина та же, а преобразование Фурье стоит дорого.
		e.Detailed = len(sys.Engines) == 0
		sys.Engines = append(sys.Engines, e)
	}
}

// utilizationTrim возвращает поправку системы управления расходованием.
//
// Соотношение компонентов в камере никогда не совпадает с соотношением
// заправки в точности: оно зависит от давлений подачи, а те меняются по мере
// опустошения баков. Разница в считанные проценты за время работы ступени
// приводит к тому, что один компонент кончается, когда второго остаётся
// заметный запас, — и этот запас становится мёртвым грузом. У Starship так
// оставалось сорок тонн окислителя при пустом баке горючего.
//
// Поэтому на реальных ступенях стоит система управления расходованием: она
// следит за уровнями в баках и подтравливает соотношение так, чтобы баки
// опустели одновременно. Здесь она же: сравниваются доли остатка, разница
// отрабатывается клапаном горючего с ограниченной властью в десять процентов.
//
// Ручное вмешательство оператора эту поправку не отменяет, а складывается
// с ней — ровно как на изделии. Прямая подмена соотношения через реестр
// параметров, наоборот, отключает автоматику: это уже отладочный режим.
func (s *PropulsionSystem) utilizationTrim() float64 {
	if s.FuelTank == nil || s.OxTank == nil {
		return 1
	}

	fuel, ox := s.FuelTank.FillFraction(), s.OxTank.FillFraction()

	// Пока баки почти полны, разница остатков ничего не значит: она тонет
	// в погрешности измерения уровня при плещущейся жидкости.
	if fuel > 0.97 && ox > 0.97 {
		return 1
	}

	// Усиление подобрано по тому, что контур обязан вытянуть.
	//
	// Расхождение соотношения за работу ступени доходит до нескольких
	// процентов расхода, а разница долей остатка при этом измеряется долями
	// процента. С прежним усилием в полтора на такую разницу приходилась
	// поправка в один процент — вчетверо меньше нужной, и корабль всё равно
	// приходил к посадке с пустым баком горючего при сорока тоннах
	// окислителя. Ниже по мере опустошения баков усиление растёт: одна и та же
	// разница долей в конце работы означает куда меньший запас в тоннах,
	// и парировать её надо решительнее.
	const (
		gain      = 12.0
		authority = 0.10
	)

	scale := 1.0
	if remaining := math.Min(fuel, ox); remaining < 0.25 && remaining > 0 {
		scale = 0.25 / remaining
		if scale > 4 {
			scale = 4
		}
	}

	// Горючего осталось относительно больше — открыть его клапан шире.
	trim := 1 + gain*scale*(fuel-ox)
	return math.Max(1-authority, math.Min(1+authority, trim))
}

// FillFraction возвращает долю оставшихся компонентов от заправки.
//
// Нужна для положения центра масс: полные баки смещают его к носу, пустые
// оставляют массу конструкции у двигателей, и запас устойчивости по ходу
// работы ступени заметно меняется.
func (s *PropulsionSystem) FillFraction() float64 {
	initial := s.FuelTank.Config.InitialMass + s.OxTank.Config.InitialMass
	if initial <= 0 {
		return 0
	}
	f := s.PropellantMass() / initial
	return math.Max(0, math.Min(1, f))
}

// PropellantMass возвращает суммарную массу компонентов, кг.
func (s *PropulsionSystem) PropellantMass() float64 {
	return s.FuelTank.Mass + s.OxTank.Mass
}

// DryMass возвращает массу без компонентов топлива, кг.
func (s *PropulsionSystem) DryMass() float64 {
	return s.StructureMass + s.PayloadMass
}

// TotalMass возвращает полную массу, кг.
func (s *PropulsionSystem) TotalMass() float64 {
	return s.DryMass() + s.PropellantMass()
}

// vehicle.RunningEngines возвращает число работающих двигателей.
func (s *PropulsionSystem) RunningEngines() int {
	n := 0
	for _, e := range s.Engines {
		if e.Running {
			n++
		}
	}
	return n
}

// StartAll запускает все двигатели.
func (s *PropulsionSystem) StartAll() {
	for _, e := range s.Engines {
		e.Start()
	}
	s.Commissioned = 0
}

// StopAll выключает все двигатели.
func (s *PropulsionSystem) StopAll() {
	for _, e := range s.Engines {
		e.Stop()
	}
}

// PropulsionEnvironment — условия полёта, влияющие на работу установки.
type PropulsionEnvironment struct {
	// AmbientPressure — забортное давление, Па.
	AmbientPressure float64

	// StructureTemperature — температура окружающей конструкции, К.
	//
	// Именно конструкции, а не разрежённого газа: на орбитальных высотах
	// кинетическая температура газа около 1000 К, но теплообмена с ним нет,
	// и бак к ней не стремится.
	StructureTemperature float64

	// AxialAcceleration — продольная перегрузка, м/с². Создаёт гидростатический
	// напор на входе насосов.
	AxialAcceleration float64

	// LateralAcceleration — поперечное ускорение, м/с². Возбуждает колебания
	// жидкости в баках.
	LateralAcceleration float64

	// AngularAcceleration — угловое ускорение корпуса, рад/с².
	AngularAcceleration float64

	// Gravity — местное ускорение свободного падения, м/с².
	Gravity float64

	// Overrides — ручные воздействия оператора на эту ступень.
	Overrides control.Overrides
}

// Update продвигает состояние установки на dt секунд.
//
// Порядок расчёта следует причинной цепочке: сначала условия на входе насосов
// определяются состоянием баков, затем работают двигатели, и только потом
// из фактического расхода вычитается масса компонентов.
func (s *PropulsionSystem) Update(dt, throttle float64, env PropulsionEnvironment) {
	if dt <= 0 {
		return
	}

	fuel := s.FuelTank
	ox := s.OxTank
	ov := env.Overrides

	// Давление на входе насосов: наддув плюс столб жидкости при перегрузке.
	fuelInlet := fuel.OutletPressure(env.AxialAcceleration)
	oxInlet := ox.OutletPressure(env.AxialAcceleration)

	// Разрыв магистрали отсекает подачу до насоса.
	if ov.FuelTank.LineRuptured {
		fuelInlet = 0
	}
	if ov.OxTank.LineRuptured {
		oxInlet = 0
	}

	in := propulsion.EngineInput{
		Throttle:          throttle,
		AmbientPressure:   env.AmbientPressure,
		FuelInletPressure: fuelInlet,
		OxInletPressure:   oxInlet,
		FuelDensity:       fuel.Density(),
		OxDensity:         ox.Density(),
		FuelTemperature:   fuel.Temperature,
		OxTemperature:     ox.Temperature,
		FuelVaporPressure: fuel.Propellant.VaporPressure(fuel.Temperature),
		OxVaporPressure:   ox.Propellant.VaporPressure(ox.Temperature),
		FuelAvailable:     !fuel.Empty() && !ov.FuelTank.LineRuptured,
		OxAvailable:       !ox.Empty() && !ov.OxTank.LineRuptured,
		Gravity:           env.Gravity,
		MixtureTrim:       s.MixtureTrim * s.utilizationTrim(),
		CoolingValve:      s.CoolingValve,
	}

	// Компенсация отказа двигателя.
	//
	// Бортовой контур управляет тягой ступени, а не отдельного двигателя.
	// Если часть двигателей не работает, оставшиеся получают повышенную
	// команду, чтобы выдать ту же суммарную тягу. Возможности форсирования
	// ограничены аттестованным запасом, поэтому потеря компенсируется лишь
	// частично, и это видно по дефициту тяги.
	//
	// Ручная уставка газа на всю ступень перебивает команду наведения
	// на каждой камере (см. engine.go, ov.ThrottleOverride.Or(in.Throttle)) —
	// и демонстрационные величины обязаны считаться от неё же. Раньше здесь
	// бралась голая команда наведения: с ручным газом на 100 % при столкновении
	// наведение продолжало тихо требовать 93,5 %, и «Газ на двигатель» в
	// панели показателей врал оператору, что реально подаётся на камеры.
	demand := ov.Engine.ThrottleOverride.Or(throttle)
	s.ThrottleDemand = demand

	running := 0
	for _, e := range s.Engines {
		if e.Running {
			running++
		}
	}
	// Сколько камер вообще должно сейчас работать.
	//
	// Это не то же самое, что число установленных: на посадке корабль намеренно
	// садится не всеми камерами, и погашенные вакуумные — не отказ. Считать их
	// отказом означало бы вдвое задрать команду оставшимся: контур честно
	// пытался бы добрать «потерянную» тягу, которой никто не просил.
	intended := len(s.Engines)
	if s.Commissioned > 0 && s.Commissioned <= len(s.Engines) {
		intended = s.Commissioned
	}
	s.EnginesOut = intended - running
	if s.EnginesOut < 0 {
		s.EnginesOut = 0
	}

	maxThrottle := 1.0
	if len(s.Engines) > 0 {
		if m := s.Engines[0].Config.MaxThrottle; m > 1 {
			maxThrottle = m
		}
	}

	perEngine := demand
	if running > 0 && demand > 0 {
		perEngine = demand * float64(intended) / float64(running)
		if perEngine > maxThrottle {
			perEngine = maxThrottle
		}
	}
	s.EngineThrottle = perEngine

	// Недобор тяги против требуемого. Считается по команде, а не по факту:
	// фактическая тяга зависит ещё и от высоты, и смешивать эти два эффекта
	// было бы неверно.
	demanded := demand * float64(intended)
	delivered := perEngine * float64(running)
	s.ThrustDeficit = 0
	if demanded > 0 && delivered < demanded {
		s.ThrustDeficit = 1 - delivered/demanded
	}

	in.Throttle = perEngine

	s.TotalThrust = 0
	s.TotalFuelFlow = 0
	s.TotalOxFlow = 0

	coolantReturn := 0.0
	coolantTemp := fuel.Temperature

	for _, e := range s.Engines {
		// Каждый двигатель получает свои поправки: команда может быть
		// адресована одному агрегату из девяти, и остальные обязаны
		// продолжать работать штатно.
		in.Overrides = ov.ForEngine(e.ID)

		e.Update(dt, in)
		s.TotalThrust += e.Thrust
		s.TotalFuelFlow += e.FuelFlow
		s.TotalOxFlow += e.OxFlow

		if e.Running && e.Nozzle.CoolingFlow > 0 {
			coolantReturn += e.Nozzle.CoolingFlow
			coolantTemp = e.Nozzle.CoolantOutletTemp
		}
	}

	totalFlow := s.TotalFuelFlow + s.TotalOxFlow
	if totalFlow > 0 {
		s.SpecificImpulse = s.TotalThrust / (totalFlow * physics.G0)
	} else {
		s.SpecificImpulse = 0
	}

	// Расход списывается с баков. Утечка добавляется внутри модели бака,
	// поэтому масса убывает быстрее расхода при негерметичности.
	tankEnv := propulsion.TankEnvironment{
		AxialAcceleration:   env.AxialAcceleration,
		LateralAcceleration: env.LateralAcceleration,
		AngularAcceleration: env.AngularAcceleration,
		AmbientPressure:     env.AmbientPressure,
		AmbientTemperature:  env.StructureTemperature,
	}

	fuelEnv := tankEnv
	// Горючее возвращается из охлаждающего тракта нагретым — это заметный
	// источник прогрева бака к концу работы ступени.
	fuelEnv.ReturnFlow = coolantReturn * 0.02
	fuelEnv.ReturnTemperature = coolantTemp
	fuelEnv.Overrides = ov.FuelTank

	oxEnv := tankEnv
	oxEnv.Overrides = ov.OxTank

	fuel.Update(dt, s.TotalFuelFlow, fuelEnv)
	ox.Update(dt, s.TotalOxFlow, oxEnv)
}

// PrimaryEngine возвращает двигатель, по которому публикуется подробная
// телеметрия. Публиковать спектр и все давления по каждому из девяти
// двигателей избыточно.
func (s *PropulsionSystem) PrimaryEngine() *propulsion.Engine {
	if len(s.Engines) == 0 {
		return nil
	}
	for _, e := range s.Engines {
		if e.Running {
			return e
		}
	}
	return s.Engines[0]
}

// syncEngineTelemetry переносит результаты расчёта установки в прежние поля
// структуры vehicle.Engine.
//
// Существующие панели Grafana и REST API продолжают работать без изменений:
// тяга, удельный импульс и температуры остаются на своих местах, но теперь
// это не таблично заданные значения, а результат расчёта давления в камере,
// коэффициента тяги и теплового баланса стенки.
func (s *Simulation) syncEngineTelemetry(atm physics.AtmosphereState) {
	sys := s.propulsion
	if sys == nil {
		return
	}

	// Сопоставление по идентификатору, а не по индексу.
	//
	// Списки не всегда идут в одном порядке. У методановой второй ступени
	// vehicle.Engine нумерует камеры подряд — "S2-1"…"S2-6", а в
	// propulsion.Engine последними стоят вакуумные, с другим именем —
	// "S2-1"…"S2-3", "S2-V1"…"S2-V3". Индекс i=3 указывал бы на "S2-4" в одном
	// списке и на физическую "S2-V1" в другом: телеметрия половины камер второй
	// ступени (тяга, УИ, температуры) уходила под чужим именем, а под настоящим
	// "S2-V*" не появлялась вовсе.
	byID := make(map[string]*propulsion.Engine, len(sys.Engines))
	for _, p := range sys.Engines {
		byID[p.ID] = p
	}

	for i := range s.engines {
		e := &s.engines[i]
		p, ok := byID[e.ID]
		if !ok {
			e.Thrust = 0
			continue
		}

		e.Running = p.Running
		e.Thrust = p.Thrust
		e.ISP = p.SpecificImpulse
		e.MaxThrust = p.Config.NominalThrustVacuum
		e.MinThrust = p.Config.NominalThrustVacuum * 0.4

		// Температуры берутся из подробной модели: камера, срез сопла,
		// самый горячий участок стенки и вход турбины.
		e.ChamberTempK = p.Chamber.Temperature
		e.NozzleTempK = p.Chamber.Temperature *
			math.Pow(math.Max(p.Nozzle.ExitPressure, 1)/math.Max(p.Chamber.MeanPressure, 1),
				(p.Config.Nozzle.SpecificHeatRatio-1)/p.Config.Nozzle.SpecificHeatRatio)
		e.WallTempK = p.Nozzle.HottestStation().Temperature
		e.TurbineTempK = p.Turbopump.TurbineTemperature

		if !p.Running {
			// Заглушенный двигатель остывает к температуре среды.
			ambient := physics.EffectiveEnvironmentTemperature(
				s.state.Altitude(), s.state.AirRelativeVelocity().Norm())
			e.ChamberTempK = physics.RelaxTo(e.ChamberTempK, ambient, s.Time.Step, 8)
			e.NozzleTempK = physics.RelaxTo(e.NozzleTempK, ambient, s.Time.Step, 6)
			e.TurbineTempK = physics.RelaxTo(e.TurbineTempK, ambient, s.Time.Step, 20)
		}
	}
	_ = atm
}

// MassBalanceError возвращает суммарную невязку баланса масс по бакам, кг.
func (s *PropulsionSystem) MassBalanceError() float64 {
	return math.Abs(s.FuelTank.MassBalanceError()) +
		math.Abs(s.OxTank.MassBalanceError())
}

// updateEngineThermal обновляет наружный нагрев двигателей набегающим потоком.
//
// Внутренние температуры камеры, сопла и турбины берутся из подробной модели;
// здесь досчитывается только влияние внешней среды на выключенные двигатели.
func (s *Simulation) updateEngineThermal(dt float64, nav orbit.NavState, atm physics.AtmosphereState) {
	env := vehicle.EnvironmentState{
		Altitude:      nav.Altitude,
		Density:       atm.Density,
		AmbientTempK:  atm.Temperature,
		RelativeSpeed: nav.AirRelativeVelocity.Norm(),
		Mach:          machNumber(nav.AirRelativeVelocity.Norm(), atm.SoundSpeed),
	}
	for i := range s.engines {
		if s.engines[i].Running {
			continue
		}
		s.engines[i].UpdateThermalState(dt, env)
	}
}

// appliedThrottle возвращает уровень, фактически поданный в камеры.
//
// Уставка наведения и то, что происходит в камере, — разные вещи: между ними
// стоит и воздействие оператора, и нижний предел дросселирования. На пульте
// нужно второе, иначе ползунок показывает одно, а двигатель работает иначе.
// Вызывается под удержанным mu.
func (s *Simulation) appliedThrottle() float64 {
	if s.propulsion == nil {
		return s.throttleCmd
	}

	sum, running := 0.0, 0
	for _, e := range s.propulsion.Engines {
		if e.Running {
			sum += e.AppliedThrottle
			running++
		}
	}
	if running == 0 {
		return 0
	}
	return sum / float64(running)
}

// anyThrottleLimited сообщает, что хотя бы одна камера удержана на нижнем
// пределе дросселирования. Вызывается под удержанным mu.
func (s *Simulation) anyThrottleLimited() bool {
	if s.propulsion == nil {
		return false
	}
	for _, e := range s.propulsion.Engines {
		if e.Running && e.ThrottleLimited {
			return true
		}
	}
	return false
}
