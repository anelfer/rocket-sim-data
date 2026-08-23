package propulsion

import (
	"log"
	"math"
	"math/rand"

	"rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Двигательная установка целиком.
//
// Порядок расчёта на шаге повторяет причинную цепочку, а не удобство
// программирования:
//
//	клапаны → расход через турбину → мощность турбины → обороты вала
//	→ напор насосов → давление подачи → расход через форсунки
//	→ давление в камере → характеристическая скорость и коэффициент тяги → тяга
//
// Обратные связи замыкаются через состояние: давление в камере предыдущего шага
// участвует в расчёте перепада на форсунках, температура стенки — в эрозии
// горловины, эрозия — в давлении камеры следующего шага.
// -----------------------------------------------------------------------------

// EngineConfig — конструктивные параметры двигателя.
type EngineConfig struct {
	Name string

	Pair      PropellantPair
	Turbopump TurbopumpConfig
	Chamber   ChamberConfig
	Nozzle    NozzleConfig

	// NominalThrustVacuum — паспортная тяга в вакууме, Н. Используется только
	// для контроля и масштабирования шкал датчиков.
	NominalThrustVacuum float64

	// ClosedCycle сообщает, что двигатель работает по замкнутой схеме.
	//
	// В открытой схеме газ, отработавший на турбине, выбрасывается за борт:
	// топливо расходуется, а тяги почти не даёт, и удельный импульс от этого
	// заметно ниже. В замкнутой схеме весь расход проходит через камеру,
	// турбина лишь отбирает по пути работу. Поэтому у замкнутых двигателей
	// нет ни отдельного выхлопа, ни смещения соотношения компонентов
	// при заправке: баки заправляются ровно по камерному соотношению.
	ClosedCycle bool

	// MinThrottle — нижний предел дросселирования, доля номинала.
	//
	// Это не пожелание, а аттестованный диапазон камеры. Ниже него турбонасос
	// не держит давление, форсунки перестают распылять как надо, и горение
	// срывается. У Raptor диапазон — от сорока процентов до номинала; у Merlin
	// примерно тот же. Команда ниже предела не гасит камеру тихо: контур
	// удерживает предел, а гасят двигатель отдельной командой.
	MinThrottle float64

	// MaxThrottle — предельный уровень форсирования, доля номинала.
	//
	// Единица означает, что двигатель работает только на паспортном режиме.
	// У реального изделия есть аттестованный запас: он и позволяет при отказе
	// одного двигателя частично добрать тягу оставшимися. Запас невелик,
	// поэтому полностью компенсировать потерю девятой части тяги нельзя —
	// ступень просто работает дольше.
	MaxThrottle float64

	// StartupGasGenRamp — время выхода клапана газогенератора на режим, с.
	StartupGasGenRamp float64

	// MinChamberPressureFraction — доля номинального давления, ниже которой
	// двигатель считается погасшим.
	MinChamberPressureFraction float64

	// GasGeneratorExhaustVelocity — эффективная скорость истечения выхлопа
	// газогенератора, м/с.
	//
	// Отработавший на турбине газ не пропадает даром. У двигателя первой
	// ступени он выбрасывается через отдельный патрубок и даёт небольшую
	// тягу, у вакуумного — вводится в насадок сопла, где успевает расшириться
	// и добавляет заметно больше. Без учёта этого вклада удельный импульс
	// получается заниженным на несколько секунд.
	GasGeneratorExhaustVelocity float64
}

// GasGeneratorMixtureRatio — соотношение компонентов в газогенераторе.
//
// Газогенератор работает с большим избытком горючего, чтобы удержать
// температуру на входе турбины в пределах жаропрочности лопаток.
const GasGeneratorMixtureRatio = 0.35

// OverallMixtureRatio возвращает соотношение компонентов, с которым двигатель
// отбирает компоненты из баков.
//
// Оно заметно ниже соотношения в камере: газогенератор потребляет почти чистое
// горючее. Заправлять баки надо именно по этому соотношению, иначе горючее
// кончится раньше окислителя — при заправке по камерному соотношению 2.34
// первая ступень оставалась с двадцатью тоннами невыработанного окислителя.
func (c EngineConfig) OverallMixtureRatio() float64 {
	chamberMR := c.Pair.OptimalMixtureRatio
	if chamberMR <= 0 {
		chamberMR = 2.34
	}

	// В замкнутой схеме весь расход идёт через камеру, отдельного потребителя
	// с другим соотношением нет, и заправлять баки надо ровно по камерному
	// соотношению.
	if c.ClosedCycle {
		return chamberMR
	}

	// Оценка расхода камеры на номинале из условия запирания горловины.
	cstar := c.Pair.CharacteristicVelocity(chamberMR) * c.Chamber.BaselineCStarEfficiency
	if cstar <= 0 || c.Nozzle.InitialThroatArea <= 0 {
		return chamberMR
	}
	chamberFlow := c.Chamber.NominalPressure * c.Nozzle.InitialThroatArea / cstar
	ggFlow := c.Turbopump.GasGeneratorFlow

	fuel := chamberFlow/(1+chamberMR) + ggFlow/(1+GasGeneratorMixtureRatio)
	ox := chamberFlow*chamberMR/(1+chamberMR) +
		ggFlow*GasGeneratorMixtureRatio/(1+GasGeneratorMixtureRatio)

	if fuel <= 0 {
		return chamberMR
	}
	return ox / fuel
}

// NominalMassFlow возвращает расчётный расход двигателя на номинале, кг/с.
//
// Следует из условия запирания горловины: ṁ = p_к·A_гор / c*. Нужен, чтобы
// подбирать сопротивление магистралей под конкретный носитель, а не
// подставлять число, подходящее одной ракете и бессмысленное для другой.
func (c EngineConfig) NominalMassFlow() float64 {
	cstar := c.Pair.CharacteristicVelocity(c.Pair.OptimalMixtureRatio) *
		c.Chamber.BaselineCStarEfficiency
	if cstar <= 0 || c.Nozzle.InitialThroatArea <= 0 {
		return 0
	}
	return c.Chamber.NominalPressure * c.Nozzle.InitialThroatArea / cstar
}

// EngineInput — внешние условия и команды на шаге.
type EngineInput struct {
	// Throttle — команда на уровень тяги, 0…1. Ноль означает выключение.
	Throttle float64

	// AmbientPressure — забортное давление, Па.
	AmbientPressure float64

	// FuelInletPressure — давление горючего перед насосом, Па.
	FuelInletPressure float64

	// OxInletPressure — давление окислителя перед насосом, Па.
	OxInletPressure float64

	// FuelDensity, OxDensity — плотности компонентов, кг/м³.
	FuelDensity float64
	OxDensity   float64

	// FuelTemperature, OxTemperature — температуры компонентов, К.
	FuelTemperature float64
	OxTemperature   float64

	// FuelVaporPressure, OxVaporPressure — давления насыщенных паров, Па.
	FuelVaporPressure float64
	OxVaporPressure   float64

	// FuelAvailable, OxAvailable — есть ли компоненты в баках.
	FuelAvailable bool
	OxAvailable   bool

	// FuelGasFraction, OxGasFraction — доля газа наддува на входе насоса,
	// а не жидкости (0…1). Источник — неосевшее в невесомости топливо
	// (Tank.Settled): заборник вместо жидкости захватывает газ. Ноль
	// означает, что топливо у заборника, как на активном участке.
	FuelGasFraction float64
	OxGasFraction   float64

	// Gravity — местное ускорение свободного падения, м/с².
	Gravity float64

	// MixtureTrim — принудительная поправка соотношения компонентов.
	// Единица означает штатное положение клапанов, значения меньше единицы
	// прикрывают клапан горючего и повышают O/F.
	MixtureTrim float64

	// CoolingValve — положение клапана охлаждения, 0…1. Единица штатная.
	CoolingValve float64

	// Overrides — ручные воздействия оператора.
	//
	// Поправки входят в расчёт как условия работы, а не как подмена
	// результата: уставка оборотов отрабатывается клапаном газогенератора,
	// засорение форсунок уменьшает проходное сечение, утечка списывает массу.
	// Всё, что следует дальше по цепочке, модель считает сама.
	Overrides control.EngineOverrides
}

// Engine — состояние двигательной установки.
type Engine struct {
	Config EngineConfig

	// ID — устойчивый идентификатор экземпляра вида "S1-3".
	//
	// Каждый двигатель блока — отдельное изделие со своим турбонасосом,
	// своей камерой и своим соплом. Идентификатор нужен, чтобы адресовать
	// воздействие конкретному агрегату, а не всей ступени сразу.
	ID string

	Turbopump *Turbopump
	Chamber   *Chamber
	Nozzle    *Nozzle
	Sensors   *SensorSuite

	// Running сообщает, работает ли двигатель.
	Running bool

	// AppliedThrottle — уровень, фактически поданный в камеру, доля номинала.
	//
	// Отличается от команды ступени: сюда уже вошли и воздействие оператора,
	// и удержание нижнего предела. Показывать оператору надо именно его —
	// иначе на пульте стоит одно число, а в камере происходит другое.
	AppliedThrottle float64

	// ThrottleLimited сообщает, что команда была ниже предела дросселирования
	// и контур удержал предел.
	ThrottleLimited bool

	// Thrust — текущая тяга, Н.
	Thrust float64

	// SpecificImpulse — удельный импульс, с.
	SpecificImpulse float64

	// FuelFlow, OxFlow — полные расходы компонентов из баков, кг/с.
	// Включают и камеру, и газогенератор.
	FuelFlow float64
	OxFlow   float64

	// ChamberFuelFlow, ChamberOxFlow — расходы, поступающие в камеру, кг/с.
	ChamberFuelFlow float64
	ChamberOxFlow   float64

	// GasGeneratorThrust — вклад выхлопа газогенератора в тягу, Н.
	GasGeneratorThrust float64

	// GasGeneratorFlow — расход через газогенератор, кг/с.
	GasGeneratorFlow     float64
	GasGeneratorFuelFlow float64
	GasGeneratorOxFlow   float64

	// ElapsedBurn — суммарная наработка двигателя, с.
	ElapsedBurn float64

	// sinceIgnition — время с последнего запуска, с. Отсчитывается заново
	// при каждом включении: иначе детектор погасания срабатывает сразу же
	// после повторного запуска, когда давление ещё не успело подняться.
	sinceIgnition float64

	// FuelSupplyPressure, OxSupplyPressure — давления перед форсунками, Па.
	FuelSupplyPressure float64
	OxSupplyPressure   float64

	rng *rand.Rand

	// sensorRng — отдельный генератор для шума измерительных каналов
	// (SensorSuite), не общий с rng.
	//
	// rng кормит физическую случайность (пульсации давления в камере —
	// Chamber.rng), а показания датчиков — уже отчётность поверх физики.
	// Если бы оба брали из одного потока, изменение характеристик датчика
	// (частота обновления, добавление нового канала) сдвигало бы все
	// последующие броски физического ГСЧ и меняло бы траекторию полёта —
	// то есть шум измерения перепутывался бы с самой физикой.
	sensorRng *rand.Rand

	// oscillationStep — шаг интегрирования пульсаций давления, с.
	oscillationStep float64

	// sensorOverrides — воздействия на измерительный тракт. Хранятся на
	// двигателе, чтобы телеметрия строилась с их учётом, не меняя сигнатур.
	sensorOverrides control.SensorOverrides

	// sensedShaftSpeed — показание датчика оборотов вала (об/мин), снятое
	// один раз за такт, в начале Update.
	//
	// Раньше показание считалось только в Telemetry() — для отчёта, а
	// регулятор оборотов читал истинные обороты напрямую. Теперь то же
	// самое измерение нужно ещё и регулятору: он не должен «подсматривать»
	// в истинное состояние модели. Считать его дважды за такт нельзя —
	// второй вызов Sensor.UpdateWith возьмёт ещё один independent бросок
	// общего ГСЧ и собьёт внутреннее состояние датчика (задержку, дрейф,
	// пропуск связи), а с ним и воспроизводимость прогона по сиду. Поэтому
	// показание считается один раз здесь, а Telemetry() его лишь читает.
	sensedShaftSpeed Measurement

	// Detailed включает подробный анализ пульсаций со спектром.
	// Достаточно одного двигателя на ступень: спектры остальных совпадают
	// по существу, а стоят дорого.
	Detailed bool
}

// NewEngine создаёт двигатель в выключенном состоянии.
//
// rng — генератор физической случайности (пульсации давления в камере);
// sensorRng — отдельный генератор для шума измерительных каналов. Их нельзя
// путать местами: см. комментарий у поля Engine.sensorRng.
func NewEngine(cfg EngineConfig, rng, sensorRng *rand.Rand, ambientTemperature float64) *Engine {
	const oscillationStep = 0.002 // 500 Гц — хватает для пульсаций до 250 Гц

	e := &Engine{
		Config:          cfg,
		Turbopump:       NewTurbopump(cfg.Turbopump, ambientTemperature),
		Chamber:         NewChamber(cfg.Chamber, cfg.Pair, rng, 1/oscillationStep),
		Nozzle:          NewNozzle(cfg.Nozzle, ambientTemperature),
		Sensors:         NewSensorSuite(cfg.Chamber.NominalPressure),
		rng:             rng,
		sensorRng:       sensorRng,
		oscillationStep: oscillationStep,
	}
	return e
}

// Update продвигает состояние двигателя на dt секунд.
func (e *Engine) Update(dt float64, in EngineInput) {
	if dt <= 0 {
		return
	}

	// --- 1. Команды на клапаны -----------------------------------------------
	//
	// Начало причинной цепочки. Клапан газогенератора задаёт мощность турбины,
	// клапаны компонентов — площадь проходного сечения форсунок.
	ov := in.Overrides
	e.sensorOverrides = ov.Sensors

	// Показание датчика оборотов — до команд клапанам: регулятор ниже
	// в этой же функции обязан работать с тем, что «видит» борт, а не
	// с истинными оборотами вала предыдущего такта.
	if e.Sensors != nil {
		v, valid := e.Sensors.ShaftSpeed.UpdateWith(e.Turbopump.RPM(), dt, e.sensorRng, e.sensorOverrides)
		e.sensedShaftSpeed = Measurement{Value: v, Valid: valid}
	} else {
		e.sensedShaftSpeed = Measurement{}
	}

	if ov.ForceIgnition && !e.Running {
		e.Start()
	}
	if ov.ForceShutdown {
		e.Running = false
	}

	// Форсирование выше номинала отрабатывается клапаном газогенератора:
	// главные клапаны компонентов на номинале уже открыты полностью, и
	// прибавка тяги берётся из роста оборотов и давления за насосами.
	maxThrottle := e.Config.MaxThrottle
	if maxThrottle < 1 {
		maxThrottle = 1
	}

	throttle := clampTo(ov.ThrottleOverride.Or(in.Throttle), maxThrottle)
	if !e.Running {
		throttle = 0
	}

	// Ниже аттестованного предела штатного режима не существует, и наведение
	// само туда никогда не просится — если его команда всё же оказалась
	// там (округление, переходный процесс), это не значит «погасить»,
	// а автоматика лишь хотела малый газ. Поэтому для неё предел держится
	// жёстко, как и раньше.
	//
	// Прямая же команда оператора (ov.ThrottleOverride) — это не наведение:
	// оператор явно просит именно это значение, в том числе для того, чтобы
	// проверить, удержится ли камера ниже аттестованного предела, или
	// сорвётся в погасание. Держать её на полу означало бы врать оператору,
	// что камера справилась, когда на деле ей просто не дали попробовать.
	if throttle > 0 && throttle < e.Config.MinThrottle && !ov.ThrottleOverride.Active {
		throttle = e.Config.MinThrottle
		e.ThrottleLimited = true
	} else {
		e.ThrottleLimited = false
	}

	// Возврат из простоя — это заново раскрутка турбонасоса, даже если
	// Running всё это время оставался true.
	//
	// Раньше отсчёт грации погасания (sinceIgnition) шёл только от Start():
	// если оператор увёл клапан в ноль командой газа, а не остановкой камеры,
	// Running не менялся, и таймер продолжал копиться с настоящего зажигания
	// минуты назад. Стоило вернуть газ — давление в камере в этот же такт
	// ещё нулевое, турбонасос ещё не раскрутился, а грация уже давно истекла:
	// детектор погасания срабатывал мгновенно, на первом же шаге после
	// команды, и гасил камеру, которая только начала выходить на режим.
	// Физически это тот же самый переходный процесс, что при первом запуске,
	// и грация ему нужна та же самая.
	if e.AppliedThrottle <= 0 && throttle > 0 {
		e.sinceIgnition = 0
	}
	e.AppliedThrottle = throttle

	trim := ov.MixtureTrimOverride.Or(in.MixtureTrim)
	if trim <= 0 {
		trim = 1
	}

	cmd := ValveCommand{
		Fuel:   throttle * trim,
		Ox:     throttle,
		GasGen: throttle,
		Bypass: 0,
	}

	// Уставка оборотов.
	//
	// Оператор задаёт не сами обороты, а режим, на который выходит регулятор.
	// Регулятор действует единственным доступным ему способом — положением
	// клапана газогенератора. Поэтому команда «поднять обороты на десять
	// процентов» проходит через мощность турбины, момент на валу и инерцию
	// ротора, а не появляется в телеметрии мгновенно.
	if ov.Turbopump.SpeedTarget.Active && e.Running {
		// Регулятор получает то же показание, что и оператор на пульте
		// (об/мин), переведённое в рад/с — единицы самого регулятора и
		// цели SpeedTarget. Не истинные обороты вала: без датчика в этой
		// цепочке отказ измерительного канала было бы нечем показать.
		measured := e.sensedShaftSpeed
		measured.Value *= math.Pi / 30
		cmd.GasGen = e.Turbopump.GovernorCommand(measured, ov.Turbopump.SpeedTarget.V, dt)
	} else {
		e.Turbopump.ReleaseGovernor()
	}
	if ov.Turbopump.Unpowered {
		cmd.GasGen = 0
	}

	// Ограничение оборотов перепуском мимо турбины.
	if limit := 1.05 * maxThrottle; e.Turbopump.SpeedFraction() > limit {
		cmd.Bypass = math.Min(1, (e.Turbopump.SpeedFraction()-limit)*8)
	}
	e.Turbopump.TrackValves(cmd, dt, ov.Turbopump.Valves)

	// --- 2. Вал: баланс мощности ---------------------------------------------
	turbineInlet := e.turbineInletTemperature()
	e.Turbopump.UpdateShaft(dt, turbineInlet, ov.Turbopump)

	// --- 3. Насосы -----------------------------------------------------------
	//
	// Расход предыдущего шага используется для расчёта потребляемой мощности:
	// это единственная развязка в цепочке, и она физически оправдана —
	// инерция жидкости в тракте не даёт расходу измениться мгновенно.
	e.Turbopump.FuelPump.MassFlow = e.FuelFlow
	e.Turbopump.OxPump.MassFlow = e.OxFlow

	e.Turbopump.FuelPump.Update(e.Turbopump.Speed, in.FuelInletPressure,
		in.FuelDensity, in.FuelVaporPressure, in.Gravity, in.FuelGasFraction,
		ov.Turbopump.FuelPump)
	e.Turbopump.OxPump.Update(e.Turbopump.Speed, in.OxInletPressure,
		in.OxDensity, in.OxVaporPressure, in.Gravity, in.OxGasFraction,
		ov.Turbopump.OxPump)

	// --- 4. Охлаждающий тракт ------------------------------------------------
	//
	// Горючее проходит рубашку охлаждения до форсунок, поэтому потери давления
	// в тракте вычитаются из давления подачи, а нагрев охладителя возвращается
	// в бак.
	fuel := e.Config.Pair.Fuel
	e.Nozzle.UpdateCooling(dt, CoolingInput{
		FuelFlow:            e.ChamberFuelFlow,
		CoolantDensity:      in.FuelDensity,
		CoolantSpecificHeat: fuel.SpecificHeat,
		CoolantViscosity:    fuel.Viscosity,
		CoolantConductivity: fuel.ThermalConductivity,
		InletTemperature:    in.FuelTemperature,

		// Стороне газа передаётся коэффициент теплоотдачи, а не готовый
		// поток: сколько тепла уйдёт в стенку, зависит от того, насколько
		// эта стенка успела нагреться.
		GasCoefficient:     e.Chamber.GasSideCoefficient(e.Nozzle.ThroatDiameter(), ov.Chamber),
		ChamberTemperature: e.Chamber.Temperature,
		SpecificHeatRatio:  e.Config.Pair.SpecificHeatRatio,
		Prandtl:            e.Config.Pair.GasPrandtl,

		CoolingValve: ov.Turbopump.Valves.Cooling.Command.Or(in.CoolingValve),
		Overrides:    ov.Nozzle,
	})
	e.Chamber.SetAppliedWallHeatFlux(e.Nozzle.HottestStation().HeatFlux)

	e.FuelSupplyPressure = e.Turbopump.FuelPump.OutletPressure - e.Nozzle.CoolingPressureDrop
	e.OxSupplyPressure = e.Turbopump.OxPump.OutletPressure

	// --- 5. Камера сгорания --------------------------------------------------
	e.Chamber.Solve(ChamberInput{
		FuelSupplyPressure: e.FuelSupplyPressure,
		OxSupplyPressure:   e.OxSupplyPressure,
		FuelDensity:        in.FuelDensity,
		OxDensity:          in.OxDensity,
		FuelValve:          e.Turbopump.FuelValve,
		OxValve:            e.Turbopump.OxValve,
		ThroatArea:         e.Nozzle.ThroatArea,
		FlowRipple:         e.Turbopump.CavitationFlowRipple(),
		FuelAvailable:      in.FuelAvailable,
		OxAvailable:        in.OxAvailable,
		Overrides:          ov.Chamber,
	})

	ripple := e.Turbopump.CavitationFlowRipple()
	e.Chamber.UpdateQuality(ripple, ov.Chamber)

	// Пульсации интегрируются мелким шагом — и это не тонкость точности,
	// а условие устойчивости счёта.
	//
	// Осциллятор явным методом Эйлера устойчив только при ω·dt порядка
	// единицы и меньше. При частоте чуга в десятки герц и шаге симуляции
	// 0.05 с ω·dt получается около десяти: решение расходится экспоненциально
	// на каждом шаге и за 20-25 секунд непрерывной работы камеры улетает
	// в ±Inf, а следующим шагом — в NaN. Раньше мелкий шаг включался только
	// для «детального» двигателя ступени — того единственного, по которому
	// ведётся спектральный анализ, — из соображений «а то спектр вырождается».
	// На деле от мелкого шага зависело не разрешение спектра, а сама
	// конечность давления: все остальные камеры блока (32 из 33 на первой
	// ступени, 5 из 6 на второй) через полминуты работы отдавали NaN
	// в Chamber.Pressure и во всё, что от него берётся в телеметрии.
	// Тяга при этом не страдала — MeanPressure считается заново на каждом
	// шаге бисекцией, а не накоплением, — но давление в камере, среднеквадратичные
	// пульсации и всё, что показывают графики Grafana по этим двигателям,
	// необратимо замирало на NaN.
	steps := int(math.Max(1, math.Round(dt/e.oscillationStep)))
	subStep := dt / float64(steps)
	for i := 0; i < steps; i++ {
		e.Chamber.UpdateOscillations(subStep, ov.Chamber)
	}
	e.Chamber.UpdateStatistics()

	// Пульсации расхода от кавитации модулируют фактический расход.
	modulation := 1.0
	if ripple > 0 {
		modulation = 1 + ripple*math.Sin(2*math.Pi*e.Chamber.DominantFrequency*e.ElapsedBurn)
	}
	e.ChamberFuelFlow = e.Chamber.FuelFlow * modulation
	e.ChamberOxFlow = e.Chamber.OxFlow * modulation

	// Газогенератор питается из тех же баков, и его расход тоже должен
	// списываться. Схема открытая: отработавший на турбине газ выбрасывается
	// за борт и заметной тяги не создаёт, но топливо расходует. Без этого
	// баланс масс не сходится, а удельный импульс получается завышенным.
	e.GasGeneratorFlow = e.Turbopump.TurbineFlow
	if e.Config.ClosedCycle {
		// Замкнутая схема: турбина стоит на пути в камеру и лишь отбирает
		// работу. Отдельного расхода за борт нет, поэтому из баков уходит
		// ровно то, что проходит через камеру.
		e.GasGeneratorFuelFlow, e.GasGeneratorOxFlow = 0, 0
	} else {
		e.GasGeneratorFuelFlow = e.GasGeneratorFlow / (1 + GasGeneratorMixtureRatio)
		e.GasGeneratorOxFlow = e.GasGeneratorFlow - e.GasGeneratorFuelFlow
	}

	e.FuelFlow = e.ChamberFuelFlow + e.GasGeneratorFuelFlow
	e.OxFlow = e.ChamberOxFlow + e.GasGeneratorOxFlow

	// --- 6. Сопло и тяга -----------------------------------------------------
	e.Nozzle.UpdateFlow(e.Chamber.MeanPressure, e.Chamber.CharacteristicVelocity,
		in.AmbientPressure)

	e.Thrust = e.Nozzle.ThrustCoefficient * e.Chamber.MeanPressure * e.Nozzle.ThroatArea
	// Выхлоп турбины даёт тягу только в открытой схеме: в замкнутой он
	// возвращается в камеру и уже учтён в её расходе.
	e.GasGeneratorThrust = 0
	if !e.Config.ClosedCycle {
		e.GasGeneratorThrust = e.GasGeneratorFlow * e.Config.GasGeneratorExhaustVelocity
		e.Thrust += e.GasGeneratorThrust
	}
	if e.Thrust < 0 {
		e.Thrust = 0
	}

	// Удельный импульс считается по полному расходу из баков, включая
	// газогенератор: именно так он определяется для двигателя открытой схемы.
	total := e.FuelFlow + e.OxFlow
	if total > 0 {
		e.SpecificImpulse = e.Thrust / (total * 9.80665)
	} else {
		e.SpecificImpulse = 0
	}

	// --- 7. Механика агрегата ------------------------------------------------
	e.Turbopump.UpdateMechanical(dt, in.FuelTemperature, ov.Turbopump)

	if e.Running {
		e.ElapsedBurn += dt
		e.sinceIgnition += dt
	}

	// Отказ подачи: в баке нет компонента вовсе.
	//
	// Это не то же самое, что погасание ниже. Той проверке нужна отсрочка —
	// турбонасосу требуются секунды на раскрутку, и без неё повторный запуск
	// на довыведении гасился бы мгновенно, ещё не выйдя на режим. Здесь
	// отсрочка не нужна и вредна: если компонента в баке нет, давление
	// в камере не поднимется никогда, сколько ни жди. Раньше в этом случае
	// Running оставался true до истечения тех же четырёх секунд ignitionGrace,
	// и приборная доска показывала «двигатель работает» там, где расход был
	// нулевым с первого же такта, — на индикаторе камера «горела» ещё долю
	// секунды после того, как ракета уже коснулась земли.
	if e.Running && throttle > 0 && (!in.FuelAvailable || !in.OxAvailable) {
		log.Printf("🕯 Камера %s остановлена: в баке нет компонента подачи", e.ID)
		e.Running = false
	}

	// Погасание: давление упало ниже допустимого.
	//
	// Проверка включается только после выхода на режим. Отсчёт ведётся от
	// момента запуска, а не от суммарной наработки: повторный запуск на
	// довыведении иначе гасился бы мгновенно, ещё до раскрутки турбонасоса.
	const ignitionGrace = 4.0
	minPressure := e.Config.Chamber.NominalPressure * e.Config.MinChamberPressureFraction
	if e.Running && throttle > 0 && e.Chamber.MeanPressure < minPressure &&
		e.sinceIgnition > ignitionGrace {

		// Погасание — событие, а не мелочь: оно меняет тягу ступени и должно
		// быть видно в журнале. Молчаливое исчезновение камеры разбирать
		// по телеметрии потом невозможно.
		log.Printf("🕯 Камера %s погасла: давление %.1f МПа при пределе %.1f МПа, "+
			"уставка %.2f", e.ID, e.Chamber.MeanPressure/1e6, minPressure/1e6,
			throttle)
		e.Running = false
	}
	if ov.Chamber.Extinguish {
		e.Running = false
	}

	// Прогоревшая стенка — это конец двигателя, а не строчка в журнале.
	// Через прогар уходит давление, камера гаснет, и запустить её больше
	// нельзя. Раньше перегрев стенки не имел последствий вовсе.
	if e.Nozzle.BurnedThrough {
		e.Running = false
	}
}

// turbineInletTemperature возвращает температуру газа перед турбиной, К.
//
// Газогенератор работает с большим избытком горючего именно для того, чтобы
// удержать температуру на входе турбины в пределах жаропрочности лопаток.
func (e *Engine) turbineInletTemperature() float64 {
	if e.Turbopump.GasGenValve <= 0 {
		return 288
	}
	base := e.Config.Pair.FlameTemperature(GasGeneratorMixtureRatio)
	return math.Min(base, 1100)
}

// Start запускает двигатель.
func (e *Engine) Start() {
	if !e.Running {
		e.sinceIgnition = 0
	}
	e.Running = true
}

// Stop выключает двигатель.
func (e *Engine) Stop() { e.Running = false }

// BurnedThrough сообщает, что стенка прогорела.
func (e *Engine) BurnedThrough() bool { return e.Nozzle.BurnedThrough }

// TotalFlow возвращает суммарный расход компонентов, кг/с.
func (e *Engine) TotalFlow() float64 { return e.FuelFlow + e.OxFlow }

// MixtureRatio возвращает фактическое соотношение компонентов.
func (e *Engine) MixtureRatio() float64 { return e.Chamber.MixtureRatio }

// ChamberPressure возвращает мгновенное давление в камере, Па.
func (e *Engine) ChamberPressure() float64 { return e.Chamber.Pressure }

// -----------------------------------------------------------------------------
// Конфигурация двигателя класса Merlin 1D
// -----------------------------------------------------------------------------

// Merlin1DConfig возвращает параметры двигателя первой ступени.
//
// Числа подобраны так, чтобы модель воспроизводила паспортные характеристики:
// тяга 845 кН у земли и 934 кН в вакууме, удельный импульс 282 и 311 с,
// давление в камере 9.7 МПа, соотношение компонентов 2.34.
func Merlin1DConfig() EngineConfig {
	const (
		nominalPressure = 9.7e6
		throatRadius    = 0.1310
		expansionRatio  = 16.0
	)
	throatArea := math.Pi * throatRadius * throatRadius

	return EngineConfig{
		Name: "Merlin 1D",
		Pair: KeroloxPair(),

		// Баланс мощности сведён на номинале: турбина 5.8 МВт против
		// 4.8 МВт насосов и 0.9 МВт механических потерь. Небольшой избыток
		// нужен, иначе агрегат не вышел бы на расчётные обороты.
		Turbopump: TurbopumpConfig{
			RotorInertia:              0.42,
			DesignSpeed:               3350, // рад/с, около 32000 об/мин
			MaxSpeed:                  4200,
			GasGeneratorFlow:          9.2,
			TurbineEnthalpyDrop:       9.5e5,
			TurbineEfficiency:         0.66,
			MechanicalLossCoefficient: 2.4e-5,
			BearingHeatCoefficient:    9.0e4,
			BearingHeatCapacity:       26000,
			BearingCooling:            760,
			RotorImbalance:            3.2,
			AxialStiffness:            9.0e7,
			GasGenStroke:              1.4,
			ValveRate:                 1.4,

			FuelPump: PumpConfig{
				Name:                 "fuel",
				DesignHead:           1417,
				DesignSpeed:          3350,
				DesignFlow:           0.118,
				Efficiency:           0.72,
				SuctionSpecificSpeed: 15,
				ImpellerArea:         0.021,
			},
			OxPump: PumpConfig{
				Name:                 "ox",
				DesignHead:           995,
				DesignSpeed:          3350,
				DesignFlow:           0.184,
				Efficiency:           0.74,
				SuctionSpecificSpeed: 12,
				ImpellerArea:         0.026,
			},
		},

		// Площади форсунок подобраны на перепад около 18 % от давления
		// в камере — обычное значение, при котором система подачи ещё
		// развязана от камеры и горение остаётся устойчивым.
		Chamber: ChamberConfig{
			FuelInjectorArea:             0.002016,
			OxInjectorArea:               0.003591,
			InjectorDischargeCoefficient: 0.78,
			Volume:                       0.031,
			WallArea:                     1.2,
			NominalPressure:              nominalPressure,
			BaselineCStarEfficiency:      0.969,
			ChugFrequency:                34,
			AcousticFrequency:            185,
		},

		Nozzle: NozzleConfig{
			InitialThroatArea: throatArea,
			ExitArea:          throatArea * expansionRatio,
			SpecificHeatRatio: 1.22,
			CoolingFraction:   1.0,
			CoolingResistance: 1.1e4,
			CoolingDesignFlow: 88,

			// Геометрия охлаждающего тракта: по ней считается теплоотдача
			// в охладитель. Сто двадцать фрезерованных каналов сечением
			// 3 × 6 мм, огневая стенка миллиметровая, медный сплав.
			ChannelCount:     120,
			ChannelWidth:     3.0e-3,
			ChannelHeight:    6.0e-3,
			WallThickness:    1.0e-3,
			WallConductivity: 350,
			ErosionThreshold: 980,
			ErosionRate:      2.2e-13,
			ThroatRadius:     throatRadius,
		},

		NominalThrustVacuum: 934000,

		// Аттестованный запас форсирования — восемь процентов. Его хватает,
		// чтобы восемь двигателей выдали 8.64 номинала вместо девяти:
		// потеря одного двигателя из девяти полностью не компенсируется
		// в принципе, и ступень вынуждена работать дольше.
		MinThrottle: 0.39,
		MaxThrottle: 1.08,

		StartupGasGenRamp: 1.6,
		// Порог погасания обязан лежать ниже того давления, которое камера
		// держит на своём же нижнем режиме. У модели на сорока процентах
		// дросселя давление садится примерно до одной пятой номинала —
		// расход через форсунки идёт как корень из перепада, и линейного
		// падения не выходит. Порог в четверть номинала при этом означал,
		// что камера гаснет на аттестованном минимуме: ровно это и убивало
		// двигатели на посадке.
		MinChamberPressureFraction:  0.15,
		GasGeneratorExhaustVelocity: 1200,
	}
}

// MerlinVacuumConfig возвращает параметры двигателя второй ступени.
//
// Отличается степенью расширения сопла: 165 против 16. В вакууме это даёт
// удельный импульс 348 с вместо 311.
func MerlinVacuumConfig() EngineConfig {
	cfg := Merlin1DConfig()
	cfg.Name = "Merlin 1D Vacuum"

	const (
		throatRadius   = 0.1345
		expansionRatio = 165.0
	)
	throatArea := math.Pi * throatRadius * throatRadius

	cfg.Nozzle.InitialThroatArea = throatArea
	cfg.Nozzle.ExitArea = throatArea * expansionRatio
	cfg.Nozzle.ThroatRadius = throatRadius
	// Сопло с радиационным охлаждением насадка: регенеративно охлаждается
	// только часть тракта.
	cfg.Nozzle.CoolingFraction = 0.85
	cfg.Nozzle.CoolingDesignFlow = 75

	cfg.Chamber.FuelInjectorArea = 0.001958
	cfg.Chamber.OxInjectorArea = 0.003485

	cfg.NominalThrustVacuum = 981000
	// Выхлоп турбины вводится в насадок сопла и успевает расшириться.
	cfg.GasGeneratorExhaustVelocity = 2500
	return cfg
}

// -----------------------------------------------------------------------------
// Конфигурация двигателя класса Raptor
// -----------------------------------------------------------------------------

// RaptorConfig возвращает параметры двигателя на метане и кислороде.
//
// Отличия от керосинового двигателя открытой схемы принципиальные.
//
// Схема замкнутая: рабочее тело турбины возвращается в камеру, а не уходит
// за борт. Из-за этого удельный импульс выше при том же топливе, а заправка
// баков идёт ровно по камерному соотношению — отдельного потребителя,
// работающего почти на чистом горючем, здесь нет.
//
// Давление в камере втрое выше — около тридцати мегапаскалей против десяти.
// Именно оно и требует замкнутой схемы: открытая на таких давлениях теряла бы
// на выхлопе неприемлемо много.
//
// Метан не коксуется в охлаждающем тракте, поэтому температурный порог эрозии
// выше, а сам двигатель рассчитан на многократное использование.
//
// Модель упрощает главное: настоящий двигатель имеет два дожигательных
// газогенератора, окислительный и восстановительный, каждый со своим
// турбонасосом. Здесь по-прежнему один агрегат с общим валом, и отбор
// рабочего тела задаётся так же, как в открытой схеме, — только возвращается
// в камеру. Баланс мощности от этого остаётся правдоподобным, но раздельного
// управления двумя контурами модель не даёт.
func RaptorConfig() EngineConfig {
	// Калибровка по паспортным данным третьего поколения двигателя: тяга
	// 267 тс у земли и 280 тс в вакууме, удельный импульс 333.5 с и 350 с,
	// давление в камере 35.2 МПа, расход 800.5 кг/с. Диаметр горловины
	// 237 мм и диаметр среза 1302 мм измерены отдельно и дают отношения
	// Ac/At = 2.85, Ae/At = 30.4 — величины ниже подобраны так, чтобы
	// геометрия горловины сошлась с этим диаметром точно, а не была
	// пересчитана заново из давления и расхода.
	//
	// Горловина: At = π·(0.237/2)² = 0.04412 м², откуда расход при этом
	// давлении и c* = 1940 м/с (см. MethaloxPair) даёт ровно 800.5 кг/с —
	// паспортные давление, расход и диаметр горловины сведены без зазора.
	//
	// Степень расширения 30.4 — тоже измеренная величина, а не подгон под
	// диаметр среза 1.35 м, как было в предыдущей калибровке: у диаметра
	// среза 1302 мм при этой горловине выходит ε = 30.2, разница с 30.4 —
	// в пределах точности замера диаметров.
	//
	// Коэффициент тяги идеального изоэнтропического сопла при этой ε
	// систематически выше того, что нужно для заданных Isp, — реальное
	// сопло теряет на трении и двумерном расширении около 5%. Эта разница
	// вынесена в Nozzle.NozzleEfficiency, а не размазана по давлению
	// или расходу, которые остаются паспортными как есть.
	const (
		nominalPressure = 35.2e6
		throatRadius    = 0.1185
		expansionRatio  = 30.4
	)
	throatArea := math.Pi * throatRadius * throatRadius

	return EngineConfig{
		Name:        "Raptor",
		Pair:        MethaloxPair(),
		ClosedCycle: true,

		Turbopump: TurbopumpConfig{
			// Ротор тяжелее и мощнее: при давлении в камере в три с половиной
			// раза выше керосинового двигателя насосам нужно поднять давление
			// подачи до сорока шести мегапаскалей.
			//
			// Отбор на турбину подобран по установившемуся режиму, а не по
			// расчёту на бумаге: равновесие вала должно приходиться ровно
			// на паспортное давление в камере. При семидесяти килограммах
			// в секунду агрегат выходит на 34.9 МПа, тридцать тысяч оборотов
			// и расход 800 кг/с — то есть на те самые числа, из которых
			// считались горловина и форсунки. Турбина при этом отдаёт
			// 50.4 МВт, насосы забирают 49.3 МВт.
			//
			// Больший отбор смещает равновесие вверх: при восьмидесяти восьми
			// килограммах двигатель устойчиво работал на 39 МПа, то есть
			// на режиме, для которого не рассчитаны ни горловина, ни тракт.
			RotorInertia:              0.85,
			DesignSpeed:               3550, // рад/с, около 34000 об/мин
			MaxSpeed:                  4500,
			GasGeneratorFlow:          70,
			TurbineEnthalpyDrop:       1.05e6,
			TurbineEfficiency:         0.72,
			MechanicalLossCoefficient: 3.0e-5,
			BearingHeatCoefficient:    1.3e5,
			BearingHeatCapacity:       34000,
			BearingCooling:            900,
			RotorImbalance:            3.6,
			// Упорный подшипник рассчитан на осевое усилие своего насоса.
			// При давлении подачи сорок мегапаскалей оно вчетверо больше
			// керосинового, и жёсткость опоры соответствующая: с прежней
			// ротор смещался на три миллиметра при допуске в два, и все
			// тридцать три двигателя постоянно числились неисправными.
			AxialStiffness: 5.0e8,
			GasGenStroke:   1.4,
			ValveRate:      1.6,

			FuelPump: PumpConfig{
				Name: "fuel",
				// Метан вдвое легче керосина, поэтому на то же давление
				// подачи требуется вдвое больший напор в метрах столба:
				// 11200 м столба метана — это 46 МПа.
				DesignHead:           11200,
				DesignSpeed:          3550,
				DesignFlow:           0.357,
				Efficiency:           0.74,
				SuctionSpecificSpeed: 14,
				ImpellerArea:         0.034,
			},
			OxPump: PumpConfig{
				Name:                 "ox",
				DesignHead:           4150,
				DesignSpeed:          3550,
				DesignFlow:           0.492,
				Efficiency:           0.76,
				SuctionSpecificSpeed: 11,
				ImpellerArea:         0.042,
			},
		},

		// Площади форсунок посчитаны на перепад 18 % от давления в камере
		// (6.3 МПа) при расходах 174 кг/с горючего и 626 кг/с окислителя:
		//
		//	A = ṁ / (Cd·√(2ρ·Δp))
		//
		// Форсунка горючего затем расширена с 0.00298 до 0.00345 м².
		// Причина физическая: горючее идёт к форсункам через рубашку
		// охлаждения и теряет там давление, а окислитель подаётся напрямую.
		// При одинаковых расчётных площадях двигатель работал на O/F 4.0
		// вместо 3.6, и окислитель кончался, когда в баке оставалось ещё
		// двадцать восемь тонн метана. Реальную камеру разводят так же:
		// сечение форсунок подбирают под фактические давления подачи.
		Chamber: ChamberConfig{
			FuelInjectorArea:             0.00345,
			OxInjectorArea:               0.00653,
			InjectorDischargeCoefficient: 0.80,
			Volume:                       0.024,
			WallArea:                     1.05,
			NominalPressure:              nominalPressure,
			// 0.995, а не прежние 0.975: при пике c* в 1950 м/с (см.
			// MethaloxPair) именно эта полнота сгорания даёт паспортные
			// 800.5 кг/с при 35.2 МПа на измеренной горловине — не запас
			// «на всякий случай», а обратный счёт от паспортных чисел.
			BaselineCStarEfficiency: 0.995,
			ChugFrequency:           42,
			AcousticFrequency:       240,
		},

		Nozzle: NozzleConfig{
			InitialThroatArea: throatArea,
			ExitArea:          throatArea * expansionRatio,
			SpecificHeatRatio: 1.20,
			// Идеальный изоэнтропический Cf при ε=30.4 даёт Isp примерно
			// на 5% больше паспортных 333.5/350 с. Разница — трение о стенку
			// и двумерное расширение на конце колокола, которых одномерная
			// формула не знает; для реального сопла такой потери нормальны.
			NozzleEfficiency:  0.951,
			CoolingFraction:   1.0,
			CoolingResistance: 9.0e3,
			CoolingDesignFlow: 174,

			// Тракт напряжённее керосинового: каналов больше, они уже,
			// стенка тоньше. Метан идёт по ним сверхкритическим, с малой
			// вязкостью и высокой скоростью.
			ChannelCount:     260,
			ChannelWidth:     1.8e-3,
			ChannelHeight:    5.5e-3,
			WallThickness:    0.9e-3,
			WallConductivity: 350,
			// Метан не коксуется, и стенка держит более высокую температуру.
			ErosionThreshold: 1120,
			ErosionRate:      1.4e-13,
			ThroatRadius:     throatRadius,
		},

		// 280 тс — паспортная тяга в вакууме.
		NominalThrustVacuum: 2.746e6,
		MinThrottle:         0.40,
		MaxThrottle:         1.08,

		StartupGasGenRamp:          1.8,
		MinChamberPressureFraction: 0.15,

		// Выхлопа за борт нет: скорость истечения газогенератора не участвует
		// в расчёте, но поле оставлено нулевым осознанно, а не забыто.
		GasGeneratorExhaustVelocity: 0,
	}
}

// RaptorVacuumConfig возвращает параметры двигателя с вакуумным соплом.
//
// Калибровка по паспортным данным: тяга 255.2 тс у земли и 295.9 тс
// в вакууме, удельный импульс 318.8 с и 369.7 с, давление в камере те же
// 35.2 МПа и расход 800.5 кг/с, что и у атмосферной версии — камера,
// форсунки и турбонасос общие, меняется только насадок сопла. Диаметры
// среза камеры, горловины и сопла — 360 / 237 / 2292 мм — дают
// Ac/At = 2.31 и Ae/At = 93.5; горловина 237 мм совпадает с атмосферной
// версией, так что расширяется именно длина сопла, а не горловина.
func RaptorVacuumConfig() EngineConfig {
	cfg := RaptorConfig()
	cfg.Name = "Raptor Vacuum"

	// Горловина — та же камера и тот же турбонасос, что у атмосферной
	// версии: меняется только насадок сопла. Степень расширения 93.5 —
	// измеренная величина (диаметр среза 2292 мм при горловине 237 мм даёт
	// ε = 93.49, разница в пределах точности замера).
	const (
		throatRadius   = 0.1185
		expansionRatio = 93.5
	)
	throatArea := math.Pi * throatRadius * throatRadius

	cfg.Nozzle.InitialThroatArea = throatArea
	cfg.Nozzle.ExitArea = throatArea * expansionRatio
	cfg.Nozzle.ThroatRadius = throatRadius
	// Расход через рубашку остаётся полным: насадок охлаждается излучением,
	// но камера и горловина — тем же горючим, что и у атмосферной версии.
	// С долей 0.55 стенка горловины выходила на 1009 К при пределе 1150 —
	// запаса не оставалось, и на переходных режимах она копила повреждение.
	cfg.Nozzle.CoolingFraction = 1.0

	// Идеальный изоэнтропический Cf при этой степени расширения даёт Isp
	// примерно на 4% больше паспортных 318.8/369.7 с — те же потери на
	// трении и двумерном расширении, что и у атмосферного сопла, но другое
	// число: сопло длиннее, и доля потери от него другая. Посчитано обратно
	// от паспортных чисел, не подогнано: коэффициент 0.96 одновременно
	// сводит и вакуумную, и посадочную (у земли) тягу и Isp почти без
	// остатка — 295.9/255.2 тс и 369.7/318.8 с получаются с точностью
	// до второго знака.
	cfg.Nozzle.NozzleEfficiency = 0.96

	cfg.Chamber.FuelInjectorArea *= 1.03
	cfg.Chamber.OxInjectorArea *= 1.03

	// 295.9 тс — паспортная тяга в вакууме (Isp 369.7 с × расход 800.5 кг/с).
	cfg.NominalThrustVacuum = 2.902e6
	return cfg
}
