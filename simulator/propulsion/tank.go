package propulsion

import (
	"math"

	"rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Бак с системой наддува.
//
// Остаток компонента не является независимой величиной: он равен начальной
// заправке за вычетом всего, что прошло через двигатель, и всего, что утекло.
// Давление на входе насоса складывается из давления наддува и гидростатического
// напора столба жидкости при осевой перегрузке — поэтому по мере опустошения
// бака напор падает, и кавитационный запас насоса ухудшается сам собой.
// -----------------------------------------------------------------------------

// TankConfig — конструктивные параметры бака.
type TankConfig struct {
	Name string

	// Volume — объём бака, м³.
	Volume float64

	// Height — высота бака, м. Задаёт гидростатический напор.
	Height float64

	// CrossSection — площадь поперечного сечения, м².
	CrossSection float64

	// InitialMass — начальная заправка, кг.
	InitialMass float64

	// InitialTemperature — начальная температура компонента, К.
	InitialTemperature float64

	// TargetPressure — давление наддува, которое поддерживает регулятор, Па.
	TargetPressure float64

	// LineResistance — гидравлическое сопротивление магистрали.
	// Потери считаются как k·ṁ²/ρ.
	LineResistance float64

	// LeakArea — эффективная площадь неплотности, м². Ноль означает
	// герметичный бак.
	LeakArea float64

	// HeatInflow — приток тепла к компоненту от окружающей среды, Вт.
	// Для криогенного бака определяет скорость прогрева.
	HeatInflow float64

	// SloshDamping — относительное демпфирование колебаний жидкости.
	SloshDamping float64

	// PressurantMass — начальный запас газа наддува, кг.
	PressurantMass float64

	// PressurantPressure — начальное давление в баллоне наддува, Па.
	PressurantPressure float64

	// MaxPressurantFlow — предельный расход газа наддува через регулятор, кг/с.
	//
	// Величина обязана соответствовать расходу компонента: газ должен
	// заполнять освобождающийся объём. У сверхтяжёлого носителя секундный
	// расход на порядок больше, и предел, достаточный для среднего класса,
	// оставил бы баки без наддува уже на первых секундах.
	MaxPressurantFlow float64

	// PressurantTemperature — температура газа наддува, К.
	PressurantTemperature float64
}

// Tank — состояние бака.
type Tank struct {
	Config     TankConfig
	Propellant Propellant

	// Mass — текущая масса компонента, кг.
	Mass float64

	// Temperature — температура компонента, К.
	Temperature float64

	// Pressure — давление газа в свободном объёме, Па.
	Pressure float64

	// PressurantRemaining — остаток газа наддува, кг.
	PressurantRemaining float64

	// PressurantPressure — давление в баллоне наддува, Па.
	PressurantPressure float64

	// PressurantFlow — текущий расход газа наддува, кг/с.
	PressurantFlow float64

	// PressurantValve — положение регулирующего клапана наддува, 0…1.
	PressurantValve float64

	// LeakRate — текущая утечка компонента, кг/с.
	LeakRate float64

	// LineDrop — потери давления в магистрали, Па.
	LineDrop float64

	// LineTemperature — температура трубопровода, К.
	LineTemperature float64

	// DrainRate — скорость уменьшения массы в баке, кг/с. Включает и расход
	// через двигатель, и утечку, поэтому при негерметичности превышает расход.
	DrainRate float64

	// HeaderFeed сообщает, что двигатели питаются из расходного бака.
	//
	// Такой бак невелик и наддут, поэтому подача не зависит ни от осевой
	// перегрузки, ни от того, куда плещется жидкость в основном баке.
	// Включается на возвращении, когда корабль летит боком.
	HeaderFeed bool

	// SloshAmplitude — амплитуда колебаний жидкости, м.
	SloshAmplitude float64

	// SloshVelocity — скорость колебаний, м/с.
	SloshVelocity float64

	// CenterOfMassOffset — смещение центра масс жидкости от оси бака, м.
	CenterOfMassOffset float64

	// consumed — накопленный расход через двигатель, кг.
	consumed float64

	// leaked — накопленная утечка, кг.
	leaked float64
}

// NewTank создаёт заправленный бак.
func NewTank(cfg TankConfig, p Propellant) *Tank {
	return &Tank{
		Config:              cfg,
		Propellant:          p,
		Mass:                cfg.InitialMass,
		Temperature:         cfg.InitialTemperature,
		Pressure:            cfg.TargetPressure,
		PressurantRemaining: cfg.PressurantMass,
		PressurantPressure:  cfg.PressurantPressure,
		LineTemperature:     cfg.InitialTemperature,
		PressurantValve:     0,
	}
}

// Density возвращает плотность компонента при текущей температуре, кг/м³.
func (t *Tank) Density() float64 {
	return t.Propellant.Density(t.Temperature)
}

// LiquidVolume возвращает объём жидкости, м³.
func (t *Tank) LiquidVolume() float64 {
	d := t.Density()
	if d <= 0 {
		return 0
	}
	return t.Mass / d
}

// UllageVolume возвращает свободный объём, м³.
func (t *Tank) UllageVolume() float64 {
	u := t.Config.Volume - t.LiquidVolume()
	if u < 0 {
		return 0
	}
	return u
}

// FillFraction возвращает объёмную заполненность бака, 0…1.
//
// Считается по объёму, а не по массе: при прогреве компонент расширяется,
// и та же масса занимает больший объём.
func (t *Tank) FillFraction() float64 {
	if t.Config.Volume <= 0 {
		return 0
	}
	f := t.LiquidVolume() / t.Config.Volume
	return math.Max(0, math.Min(1, f))
}

// LiquidLevel возвращает уровень жидкости от днища, м.
func (t *Tank) LiquidLevel() float64 {
	if t.Config.CrossSection <= 0 {
		return 0
	}
	return t.LiquidVolume() / t.Config.CrossSection
}

// Empty сообщает, что компонент выработан.
func (t *Tank) Empty() bool { return t.Mass <= 0 }

// OutletPressure возвращает давление на входе насоса, Па.
//
//	p = p_наддува + ρ·a·h − Δp_магистрали
//
// Гидростатический член зависит от осевой перегрузки: на активном участке
// столб жидкости прижат к днищу и добавляет напор, в невесомости этого вклада
// нет. По мере опустошения бака уровень падает и вклад уменьшается.
func (t *Tank) OutletPressure(axialAcceleration float64) float64 {
	// Расходный бак кормит двигатели независимо от того, куда прижата
	// жидкость в основном баке.
	//
	// Это не поблажка модели, а причина, по которой такие баки существуют.
	// Корабль возвращается боком, и в основных баках компоненты прижаты
	// к борту, а не к днищу: питать оттуда посадочные камеры нельзя. Поэтому
	// на возвращении их питает отдельный небольшой бак под наддувом — там
	// столб жидкости мал, а давление держится вытеснением.
	if t.HeaderFeed {
		p := t.Pressure + headerTankHead - t.LineDrop
		if p < 0 {
			return 0
		}
		return p
	}

	head := t.Density() * math.Max(0, axialAcceleration) * t.LiquidLevel()
	p := t.Pressure + head - t.LineDrop

	// Колебания жидкости качают давление на входе насоса: гребень волны
	// у заборного устройства повышает напор, впадина понижает.
	p += t.Density() * math.Max(0, axialAcceleration) * t.SloshAmplitude * 0.5

	if p < 0 {
		return 0
	}
	return p
}

// headerTankHead — прибавка давления от наддува расходного бака, Па.
//
// Четыре атмосферы: столько даёт вытеснительная подача небольшого бака,
// и этого достаточно, чтобы насос не срывался в кавитацию при любой
// ориентации корпуса.
const headerTankHead = 4.0e5

// DepletionTime возвращает время до полной выработки компонента, с.
// Учитывает и расход через двигатель, и утечку.
func (t *Tank) DepletionTime() float64 {
	if t.DrainRate <= 1e-6 {
		return math.Inf(1)
	}
	return t.Mass / t.DrainRate
}

// TankEnvironment — внешние условия для бака на текущем шаге.
type TankEnvironment struct {
	// AxialAcceleration — продольная перегрузка, м/с².
	AxialAcceleration float64

	// LateralAcceleration — поперечное ускорение, м/с². Возбуждает колебания.
	LateralAcceleration float64

	// AngularAcceleration — угловое ускорение корпуса, рад/с².
	AngularAcceleration float64

	// AmbientPressure — забортное давление, Па. Определяет истечение
	// через неплотность.
	AmbientPressure float64

	// AmbientTemperature — температура окружающей конструкции, К.
	AmbientTemperature float64

	// ReturnFlow — расход компонента, возвращаемого в бак из охлаждающего
	// тракта, кг/с. Ноль, если возврата нет.
	ReturnFlow float64

	// ReturnTemperature — температура возвращаемого компонента, К.
	ReturnTemperature float64

	// Overrides — ручные воздействия на бак и магистраль.
	Overrides control.TankOverrides
}

// Update продвигает состояние бака на dt секунд.
//
// consumption — расход компонента через двигатель, кг/с.
func (t *Tank) Update(dt, consumption float64, env TankEnvironment) {
	if dt <= 0 {
		return
	}

	ov := env.Overrides

	// Разрыв магистрали прекращает подачу: расход через двигатель обнуляется,
	// а компонент уходит за борт. Двигатель гаснет не по команде, а потому,
	// что ему нечем питаться.
	if ov.LineRuptured {
		consumption = 0
	}

	t.updateLeak(dt, env)
	t.updateMass(dt, consumption)
	t.updatePressurant(dt, consumption, ov)
	t.updateTemperature(dt, env)
	t.updateLine(dt, consumption, env)
	t.updateSlosh(dt, env)

	// Прямые подмены состояния. Баланс масс при этом перестаёт сходиться,
	// поэтому режим выделен в интерфейсе как отладочный.
	if ov.MassDirect.Active {
		t.Mass = math.Max(0, ov.MassDirect.V)
	}
	if ov.PressureDirect.Active {
		t.Pressure = math.Max(0, ov.PressureDirect.V)
	}
	if ov.TemperatureDirect.Active {
		t.Temperature = ov.TemperatureDirect.V
	}
}

// updateLeak считает утечку через неплотность.
//
// Истечение жидкости через отверстие подчиняется формуле Торричелли:
//
//	ṁ = Cd·A·√(2ρ·Δp)
//
// Утечка — это дополнительная потеря массы: бак пустеет быстрее, чем того
// требует расход двигателя.
func (t *Tank) updateLeak(dt float64, env TankEnvironment) {
	// Заданная оператором утечка складывается со штатной неплотностью
	// и списывается из бака наравне с расходом двигателя.
	manual := math.Max(0, env.Overrides.LeakRate.Or(0))
	if env.Overrides.LineRuptured {
		// Разрыв магистрали: компонент уходит с полным расходом подачи.
		manual = math.Max(manual, t.DrainRate)
	}

	if t.Config.LeakArea <= 0 || t.Mass <= 0 {
		t.LeakRate = manual
		if t.Mass <= 0 {
			t.LeakRate = 0
		}
		return
	}
	const dischargeCoefficient = 0.62

	dp := t.Pressure - env.AmbientPressure
	if dp <= 0 {
		t.LeakRate = manual
		return
	}
	rho := t.Density()
	t.LeakRate = dischargeCoefficient*t.Config.LeakArea*math.Sqrt(2*rho*dp) + manual
}

func (t *Tank) updateMass(dt, consumption float64) {
	drain := consumption + t.LeakRate
	if drain < 0 {
		drain = 0
	}
	t.DrainRate = drain

	removed := drain * dt
	if removed > t.Mass {
		removed = t.Mass
		// Пересчитываем фактическую скорость на неполном шаге.
		if dt > 0 {
			t.DrainRate = removed / dt
		}
	}

	// Расход и утечка учитываются раздельно, чтобы остаток всегда можно было
	// свести с начальной заправкой.
	total := consumption + t.LeakRate
	if total > 0 {
		t.consumed += removed * consumption / total
		t.leaked += removed * t.LeakRate / total
	}
	t.Mass -= removed
	if t.Mass < 0 {
		t.Mass = 0
	}
}

// updatePressurant поддерживает давление в баке, расходуя газ наддува.
//
// По мере опустошения свободный объём растёт, и без подачи газа давление
// падало бы как p·V = const. Регулятор открывает клапан, компенсируя падение.
// Когда запас газа исчерпан, давление в баке начинает снижаться — и это
// немедленно отражается на давлении перед насосом и кавитационном запасе.
func (t *Tank) updatePressurant(dt, consumption float64, ov control.TankOverrides) {
	ullage := t.UllageVolume()
	if ullage <= 0 {
		t.PressurantFlow = 0
		return
	}

	// Скорость роста свободного объёма.
	rho := t.Density()
	volumeRate := 0.0
	if rho > 0 {
		volumeRate = t.DrainRate / rho
	}

	// Расширение газа в увеличивающемся объёме снижает давление.
	if ullage > 0 {
		t.Pressure -= t.Pressure * volumeRate / ullage * dt
	}

	target := t.Config.TargetPressure
	deficit := target - t.Pressure

	// Пропорциональный регулятор наддува с ограничением хода клапана.
	demand := 0.0
	if deficit > 0 {
		demand = deficit / (0.05 * target)
	}
	t.PressurantValve = math.Max(0, math.Min(1, demand))

	// Ручное положение клапана наддува заменяет команду регулятора.
	// Закрытый клапан означает потерю наддува: давление в баке дальше падает
	// само, по мере роста свободного объёма.
	t.PressurantValve = math.Max(0, math.Min(1,
		ov.PressurantValve.Or(t.PressurantValve)))

	if t.PressurantRemaining <= 0 || t.PressurantPressure <= t.Pressure {
		// Газ кончился или его давление сравнялось с баком — поддерживать
		// давление больше нечем.
		t.PressurantFlow = 0
		t.PressurantRemaining = math.Max(0, t.PressurantRemaining)
		return
	}

	// Максимальный расход через регулятор, кг/с.
	maxPressurantFlow := t.Config.MaxPressurantFlow
	if maxPressurantFlow <= 0 {
		maxPressurantFlow = 0.35
	}
	flow := t.PressurantValve * maxPressurantFlow * ov.PressurantFlowFactor.Or(1)
	if flow < 0 {
		flow = 0
	}

	// Удельная газовая постоянная гелия.
	const heliumR = 2077.0
	gasTemp := t.Config.PressurantTemperature
	if gasTemp <= 0 {
		gasTemp = 200
	}

	// Приращение давления от поданной массы газа.
	added := flow * dt
	if added > t.PressurantRemaining {
		added = t.PressurantRemaining
		if dt > 0 {
			flow = added / dt
		}
	}
	t.Pressure += added * heliumR * gasTemp / ullage
	t.PressurantRemaining -= added
	t.PressurantFlow = flow

	// Давление в баллоне падает пропорционально израсходованной массе.
	if t.Config.PressurantMass > 0 {
		t.PressurantPressure = t.Config.PressurantPressure *
			t.PressurantRemaining / t.Config.PressurantMass
	}

	if t.Pressure > target*1.15 {
		t.Pressure = target * 1.15
	}
}

// updateTemperature считает прогрев компонента.
//
// Криогенный компонент постоянно подогревается через стенки бака. Горючее
// дополнительно нагревается возвратом из охлаждающего тракта: часть горючего
// проходит рубашку охлаждения и возвращается заметно более горячей.
func (t *Tank) updateTemperature(dt float64, env TankEnvironment) {
	if t.Mass <= 0 {
		t.Temperature = env.AmbientTemperature
		return
	}

	heatCapacity := t.Mass * t.Propellant.SpecificHeat
	if heatCapacity <= 0 {
		return
	}

	// Приток тепла извне.
	t.Temperature += t.Config.HeatInflow * dt / heatCapacity

	// Смешение с возвращаемым горячим компонентом.
	if env.ReturnFlow > 0 && env.ReturnTemperature > 0 {
		mixed := env.ReturnFlow * dt
		if mixed > 0 {
			t.Temperature += (env.ReturnTemperature - t.Temperature) * mixed / t.Mass
		}
	}

	// Криогенный компонент не может нагреться выше точки кипения при текущем
	// давлении: избыток тепла уходит на испарение, а не на нагрев.
	if t.Propellant.Cryogenic {
		limit := t.boilingPointAt(t.Pressure)
		if t.Temperature > limit {
			t.Temperature = limit
		}
	}
}

// boilingPointAt возвращает температуру кипения при заданном давлении, К.
func (t *Tank) boilingPointAt(pressure float64) float64 {
	if pressure <= 0 {
		return t.Propellant.BoilingPoint
	}
	const (
		standardPressure = 101325.0
		universalR       = 8.314
	)
	molarMass := 0.032
	if !t.Propellant.Cryogenic {
		molarMass = 0.170
	}
	specificR := universalR / molarMass

	// Обращение уравнения Клаузиуса — Клапейрона.
	inv := 1/t.Propellant.BoilingPoint -
		specificR/t.Propellant.VaporizationEnthalpy*math.Log(pressure/standardPressure)
	if inv <= 0 {
		return t.Propellant.BoilingPoint
	}
	return 1 / inv
}

// updateLine считает потери давления и прогрев трубопровода.
func (t *Tank) updateLine(dt, consumption float64, env TankEnvironment) {
	rho := t.Density()
	if rho > 0 && consumption > 0 {
		// Засорение сужает проходное сечение. Потери растут как квадрат
		// обратной площади, поэтому даже частичное перекрытие резко
		// снижает давление на входе насоса.
		resistance := t.Config.LineResistance
		if blockage := clamp01(env.Overrides.LineBlockage.Or(0)); blockage > 0 {
			open := math.Max(0.02, 1-blockage)
			resistance /= open * open
		}
		t.LineDrop = resistance * consumption * consumption / rho
	} else {
		t.LineDrop = 0
	}

	// Трубопровод приходит к температуре протекающего компонента.
	target := t.Temperature
	if consumption <= 0 {
		target = env.AmbientTemperature
	}
	tau := 20.0
	t.LineTemperature = target + (t.LineTemperature-target)*math.Exp(-dt/tau)
}

// -----------------------------------------------------------------------------
// Колебания жидкости
// -----------------------------------------------------------------------------

// updateSlosh интегрирует уравнение колебаний жидкости.
//
// Жидкость в частично заполненном баке ведёт себя как маятник:
//
//	ẍ + 2ζω·ẋ + ω²·x = a_поперечное + r·ε
//
// Собственная частота зависит от уровня заполнения и продольной перегрузки:
//
//	ω = √(1.84·g·tanh(1.84·h/R) / R)
//
// В отсутствие возбуждения колебания затухают, но медленно: демпфирование
// в гладком баке мало, и именно поэтому в баки ставят перегородки.
func (t *Tank) updateSlosh(dt float64, env TankEnvironment) {
	fill := t.FillFraction()
	if fill < 0.02 || fill > 0.995 {
		// Почти пустой или полностью залитый бак не плещется.
		t.SloshAmplitude *= math.Exp(-dt / 2)
		t.SloshVelocity *= math.Exp(-dt / 2)
		t.CenterOfMassOffset = 0
		return
	}

	radius := math.Sqrt(t.Config.CrossSection / math.Pi)
	if radius <= 0 {
		return
	}

	g := math.Max(env.AxialAcceleration, 0.1)
	level := t.LiquidLevel()

	omega := math.Sqrt(1.84 * g * math.Tanh(1.84*level/radius) / radius)
	if omega <= 0 || math.IsNaN(omega) {
		return
	}

	zeta := t.Config.SloshDamping
	if zeta <= 0 {
		zeta = 0.02
	}
	zeta *= env.Overrides.SloshDampingFactor.Or(1)

	// Возбуждение: поперечное ускорение плюс вклад углового ускорения,
	// приложенный на плече, равном расстоянию от центра масс жидкости.
	forcing := (env.LateralAcceleration + env.AngularAcceleration*level*0.5) *
		env.Overrides.SloshExcitationFactor.Or(1)

	// Полуявная схема: устойчива при любом шаге.
	accel := forcing - 2*zeta*omega*t.SloshVelocity - omega*omega*t.SloshAmplitude
	t.SloshVelocity += accel * dt
	t.SloshAmplitude += t.SloshVelocity * dt

	// Амплитуда физически ограничена радиусом бака.
	limit := radius * 0.8
	if t.SloshAmplitude > limit {
		t.SloshAmplitude = limit
		t.SloshVelocity = 0
	} else if t.SloshAmplitude < -limit {
		t.SloshAmplitude = -limit
		t.SloshVelocity = 0
	}

	// Смещение центра масс жидкости пропорционально амплитуде и доле
	// подвижной массы: в почти полном баке плещется лишь тонкий слой.
	mobile := 4 * fill * (1 - fill)
	t.CenterOfMassOffset = t.SloshAmplitude * mobile * 0.5
}

// Consumed возвращает накопленный расход через двигатель, кг.
func (t *Tank) Consumed() float64 { return t.consumed }

// Leaked возвращает накопленную утечку, кг.
func (t *Tank) Leaked() float64 { return t.leaked }

// MassBalanceError возвращает невязку баланса масс, кг.
//
// Начальная заправка обязана равняться сумме остатка, израсходованного
// и утекшего. Величина существует ради проверки: остаток не считается
// независимо, а выводится из этого баланса.
func (t *Tank) MassBalanceError() float64 {
	return t.Config.InitialMass - (t.Mass + t.consumed + t.leaked)
}
