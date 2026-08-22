package propulsion

import (
	"math"
	"math/rand"

	"rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Камера сгорания.
//
// Давление в камере не задаётся, а получается из условия запирания
// критического сечения:
//
//	p_к = ṁ · c* / A_гор
//
// Расход через форсунки, в свою очередь, зависит от перепада между давлением
// за насосом и давлением в камере:
//
//	ṁ = Cd · A_ф · √(2ρ · (p_нас − p_к))
//
// Это замкнутая алгебраическая связь: давление зависит от расхода, расход —
// от давления. Она решается итерационно на каждом шаге, поэтому прикрытие
// клапана автоматически снижает и расход, и давление, и тягу.
// -----------------------------------------------------------------------------

// CombustionState — состояние процесса горения.
type CombustionState int

const (
	CombustionStable CombustionState = iota
	CombustionMarginal
	CombustionUnstable
)

// String возвращает название состояния.
func (s CombustionState) String() string {
	switch s {
	case CombustionStable:
		return "Stable"
	case CombustionMarginal:
		return "Marginal"
	case CombustionUnstable:
		return "Unstable"
	default:
		return "Unknown"
	}
}

// ChamberConfig — параметры камеры сгорания.
type ChamberConfig struct {
	// FuelInjectorArea — суммарная площадь форсунок горючего, м².
	FuelInjectorArea float64

	// OxInjectorArea — суммарная площадь форсунок окислителя, м².
	OxInjectorArea float64

	// InjectorDischargeCoefficient — коэффициент расхода форсунки.
	InjectorDischargeCoefficient float64

	// Volume — объём камеры до критического сечения, м³.
	Volume float64

	// WallArea — площадь поверхности стенок камеры, м².
	WallArea float64

	// NominalPressure — расчётное давление в камере, Па.
	NominalPressure float64

	// BaselineCStarEfficiency — полнота сгорания при идеальном соотношении
	// компонентов и штатном перепаде на форсунках.
	BaselineCStarEfficiency float64

	// ChugFrequency — характерная частота низкочастотных пульсаций, Гц.
	// Определяется временем пребывания газа в камере и податливостью
	// системы подачи.
	ChugFrequency float64

	// AcousticFrequency — первая акустическая мода камеры, Гц.
	AcousticFrequency float64
}

// Chamber — состояние камеры сгорания.
type Chamber struct {
	Config ChamberConfig
	Pair   PropellantPair

	// MeanPressure — среднее давление, Па.
	MeanPressure float64

	// Pressure — мгновенное давление с пульсациями, Па.
	Pressure float64

	// Temperature — температура продуктов сгорания, К.
	Temperature float64

	// FuelFlow, OxFlow — расходы компонентов, кг/с.
	FuelFlow float64
	OxFlow   float64

	// MixtureRatio — фактическое соотношение компонентов.
	MixtureRatio float64

	// MixtureDeviation — относительное отклонение от оптимума.
	MixtureDeviation float64

	// CharacteristicVelocity — фактическая характеристическая скорость, м/с.
	CharacteristicVelocity float64

	// CStarEfficiency — полнота сгорания.
	CStarEfficiency float64

	// CombustionEfficiency — эффективность горения.
	CombustionEfficiency float64

	// TemperatureNonuniformity — неоднородность поля температур.
	TemperatureNonuniformity float64

	// FuelInjectorDrop, OxInjectorDrop — перепады на форсунках, Па.
	FuelInjectorDrop float64
	OxInjectorDrop   float64

	// StabilityMargin — запас устойчивости горения, 0…1.
	StabilityMargin float64

	// State — состояние горения.
	State CombustionState

	// Пульсации давления.
	oscillation     float64
	oscillationRate float64
	dominantFreq    float64

	// PressureRMS — среднеквадратичное отклонение давления, Па.
	PressureRMS float64

	// PressurePeak — максимальное отклонение от среднего в окне, Па.
	PressurePeak float64

	// DominantFrequency — доминирующая частота спектра, Гц.
	DominantFrequency float64

	// appliedWallFlux — тепловой поток, поданный на стенки на последнем шаге.
	appliedWallFlux float64

	spectrum *SpectrumAnalyzer
	rng      *rand.Rand
}

// NewChamber создаёт камеру в неработающем состоянии.
func NewChamber(cfg ChamberConfig, pair PropellantPair, rng *rand.Rand, sampleRate float64) *Chamber {
	return &Chamber{
		Config:          cfg,
		Pair:            pair,
		Temperature:     288,
		StabilityMargin: 1,
		CStarEfficiency: cfg.BaselineCStarEfficiency,
		dominantFreq:    cfg.ChugFrequency,
		spectrum:        NewSpectrumAnalyzer(spectrumSize, sampleRate),
		rng:             rng,
	}
}

// ChamberInput — входные данные для расчёта камеры на шаге.
type ChamberInput struct {
	// FuelSupplyPressure — давление горючего перед форсунками, Па.
	// Уже уменьшено на потери в охлаждающем тракте.
	FuelSupplyPressure float64

	// OxSupplyPressure — давление окислителя перед форсунками, Па.
	OxSupplyPressure float64

	// FuelDensity, OxDensity — плотности компонентов, кг/м³.
	FuelDensity float64
	OxDensity   float64

	// FuelValve, OxValve — положения клапанов, 0…1. Прикрытие клапана
	// уменьшает эффективную площадь проходного сечения.
	FuelValve float64
	OxValve   float64

	// ThroatArea — текущая площадь критического сечения, м².
	// Меняется по мере эрозии.
	ThroatArea float64

	// FlowRipple — относительная амплитуда пульсаций расхода от кавитации.
	FlowRipple float64

	// FuelAvailable, OxAvailable — есть ли компонент в баках.
	FuelAvailable bool
	OxAvailable   bool

	// Overrides — ручные воздействия на камеру.
	Overrides control.ChamberOverrides
}

// Solve находит согласованные расходы и давление в камере.
//
// Уравнения расхода через форсунки и запирания горловины решаются совместно
// методом деления отрезка пополам: функция невязки монотонна по давлению,
// поэтому сходимость гарантирована.
func (c *Chamber) Solve(in ChamberInput) {
	if in.ThroatArea <= 0 {
		c.reset()
		return
	}

	ov := in.Overrides

	// Засорение форсунок — это уменьшение проходного сечения. Дальше всё
	// следует само: перепад растёт, расход падает, соотношение компонентов
	// уходит от оптимума, меняются температура, полнота сгорания и тяга.
	fuelArea := c.Config.FuelInjectorArea * clamp01(in.FuelValve) *
		ov.InjectorFuelAreaFactor.Or(1)
	oxArea := c.Config.OxInjectorArea * clamp01(in.OxValve) *
		ov.InjectorOxAreaFactor.Or(1)

	if ov.Extinguish {
		c.reset()
		return
	}

	if !in.FuelAvailable {
		fuelArea = 0
	}
	if !in.OxAvailable {
		oxArea = 0
	}

	if fuelArea <= 0 || oxArea <= 0 {
		c.reset()
		return
	}

	cd := c.Config.InjectorDischargeCoefficient

	// Расход через форсунки при заданном давлении в камере.
	flows := func(pc float64) (fuel, ox float64) {
		fuel = injectorFlow(cd, fuelArea, in.FuelDensity, in.FuelSupplyPressure-pc)
		ox = injectorFlow(cd, oxArea, in.OxDensity, in.OxSupplyPressure-pc)
		return
	}

	// Невязка: разность между расходом, который пропускают форсунки, и
	// расходом, который способна пропустить горловина при том же давлении.
	residual := func(pc float64) float64 {
		fuel, ox := flows(pc)
		total := fuel + ox
		if total <= 0 {
			return -pc
		}
		mr := 0.0
		if fuel > 0 {
			mr = ox / fuel
		}
		cstar := c.Pair.CharacteristicVelocity(mr) * c.CStarEfficiency *
			ov.CStarEfficiencyFactor.Or(1)
		if cstar <= 0 {
			return -pc
		}
		return total*cstar/in.ThroatArea - pc
	}

	upper := math.Max(in.FuelSupplyPressure, in.OxSupplyPressure)
	if upper <= 0 {
		c.reset()
		return
	}

	// При нулевом давлении невязка положительна, при давлении подачи —
	// отрицательна: корень заведомо внутри отрезка.
	lo, hi := 0.0, upper
	for i := 0; i < 60; i++ {
		mid := (lo + hi) / 2
		if residual(mid) > 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	pc := (lo + hi) / 2

	fuel, ox := flows(pc)
	c.FuelFlow = fuel
	c.OxFlow = ox
	c.MeanPressure = pc

	if fuel > 0 {
		c.MixtureRatio = ox / fuel
	} else {
		c.MixtureRatio = 0
	}
	c.MixtureDeviation = c.Pair.MixtureRatioDeviation(c.MixtureRatio)

	c.FuelInjectorDrop = math.Max(0, in.FuelSupplyPressure-pc)
	c.OxInjectorDrop = math.Max(0, in.OxSupplyPressure-pc)

	c.CharacteristicVelocity = c.Pair.CharacteristicVelocity(c.MixtureRatio) *
		c.CStarEfficiency * ov.CStarEfficiencyFactor.Or(1)
	c.Temperature = c.Pair.FlameTemperature(c.MixtureRatio) * c.CombustionEfficiencyFactor()

	// Прямые подмены состояния. Причинность на этих величинах нарушается,
	// поэтому режим выделен в интерфейсе отдельно.
	if ov.PressureDirect.Active {
		c.MeanPressure = math.Max(0, ov.PressureDirect.V)
	}
	if ov.TemperatureDirect.Active {
		c.Temperature = ov.TemperatureDirect.V
	}
}

// injectorFlow возвращает расход через форсунку при заданном перепаде.
func injectorFlow(cd, area, density, deltaP float64) float64 {
	if area <= 0 || density <= 0 || deltaP <= 0 {
		return 0
	}
	return cd * area * math.Sqrt(2*density*deltaP)
}

// CombustionEfficiencyFactor возвращает множитель, которым отклонение
// соотношения компонентов снижает температуру продуктов сгорания.
func (c *Chamber) CombustionEfficiencyFactor() float64 {
	f := c.CombustionEfficiency
	if f <= 0 {
		return 1
	}
	return 0.9 + 0.1*f
}

func (c *Chamber) reset() {
	c.FuelFlow, c.OxFlow = 0, 0
	c.MeanPressure = 0
	c.Pressure = 0
	c.MixtureRatio = 0
	c.CharacteristicVelocity = 0
	c.FuelInjectorDrop, c.OxInjectorDrop = 0, 0
}

// TotalFlow возвращает суммарный расход, кг/с.
func (c *Chamber) TotalFlow() float64 { return c.FuelFlow + c.OxFlow }

// -----------------------------------------------------------------------------
// Качество и устойчивость горения
// -----------------------------------------------------------------------------

// UpdateQuality пересчитывает полноту сгорания и запас устойчивости.
//
// Полнота сгорания падает по трём причинам:
//   - отклонение соотношения компонентов от оптимума ухудшает смешение;
//   - малый перепад на форсунках даёт грубый распыл;
//   - развившиеся пульсации сами по себе срывают процесс.
//
// Запас устойчивости определяется прежде всего отношением перепада на
// форсунках к давлению в камере. Классический критерий: ниже 0.15 система
// подачи перестаёт развязывать камеру от магистрали, и возникает автоколебание.
func (c *Chamber) UpdateQuality(cavitationRipple float64, ov control.ChamberOverrides) {
	if c.MeanPressure <= 0 {
		c.CStarEfficiency = c.Config.BaselineCStarEfficiency
		c.CombustionEfficiency = 0
		c.StabilityMargin = 1
		c.State = CombustionStable
		c.TemperatureNonuniformity = 0
		return
	}

	// Отношение перепада на форсунках к давлению камеры.
	dropRatio := math.Min(c.FuelInjectorDrop, c.OxInjectorDrop) / c.MeanPressure

	// Потеря полноты сгорания от отклонения соотношения компонентов.
	mixturePenalty := 0.35 * c.MixtureDeviation * c.MixtureDeviation

	// Потеря от грубого распыла при малом перепаде.
	atomizationPenalty := 0.0
	if dropRatio < 0.2 {
		deficit := (0.2 - dropRatio) / 0.2
		atomizationPenalty = 0.12 * deficit * deficit
	}

	// Потеря от пульсаций.
	oscillationPenalty := 0.0
	if c.MeanPressure > 0 {
		rel := c.PressureRMS / c.MeanPressure
		oscillationPenalty = 0.5 * rel * rel
	}

	c.CStarEfficiency = c.Config.BaselineCStarEfficiency -
		mixturePenalty - atomizationPenalty - oscillationPenalty
	c.CStarEfficiency = math.Max(0.6, math.Min(1, c.CStarEfficiency))

	c.CombustionEfficiency = math.Max(0.6, math.Min(1,
		0.995-0.5*math.Abs(c.MixtureDeviation)-atomizationPenalty))

	// Неоднородность поля температур.
	c.TemperatureNonuniformity = math.Min(1.5,
		0.02+0.4*math.Abs(c.MixtureDeviation)+0.5*atomizationPenalty)
	if bias := ov.TemperatureSpreadBias.Or(0); bias > 0 && c.Temperature > 0 {
		c.TemperatureNonuniformity = math.Min(1.5,
			c.TemperatureNonuniformity+bias/c.Temperature)
	}

	// Запас устойчивости.
	//
	// Каждое слагаемое отражает свой физический механизм и вычитается
	// из единичного запаса.
	margin := 1.0

	// Недостаточная развязка через форсунки.
	if dropRatio < 0.25 {
		margin -= (0.25 - dropRatio) / 0.25
	}

	// Отклонение соотношения компонентов: смещает зону горения по длине
	// камеры и сближает её с акустическими модами.
	margin -= 1.5 * math.Abs(c.MixtureDeviation)

	// Пульсации расхода от кавитации напрямую раскачивают камеру.
	margin -= 2.0 * cavitationRipple

	// Ручное смещение запаса устойчивости. Оно не подменяет пульсации:
	// амплитуда, частота и спектр всё равно получаются интегрированием
	// осциллятора с изменившимся демпфированием.
	margin += ov.StabilityMarginBias.Or(0)

	c.StabilityMargin = math.Max(0, math.Min(1, margin))

	switch {
	case c.StabilityMargin >= 0.35:
		c.State = CombustionStable
	case c.StabilityMargin >= 0.12:
		c.State = CombustionMarginal
	default:
		c.State = CombustionUnstable
	}
}

// UpdateOscillations интегрирует пульсации давления в камере.
//
// Пульсации моделируются осциллятором второго порядка, возбуждаемым шумом
// горения:
//
//	ẍ + 2ζω·ẋ + ω²·x = w(t)
//
// Демпфирование ζ падает вместе с запасом устойчивости. При запасе около нуля
// ζ обращается в ноль, и колебание перестаёт затухать: амплитуда нарастает,
// а в спектре появляется острый пик на собственной частоте. При штатном запасе
// ζ велико, и пульсации остаются широкополосным шумом небольшой амплитуды.
//
// Метод вызывается с малым шагом (порядка миллисекунд), чтобы разрешить
// частоты в сотни герц.
func (c *Chamber) UpdateOscillations(dt float64, ov control.ChamberOverrides) {
	if c.MeanPressure <= 0 {
		c.oscillation, c.oscillationRate = 0, 0
		c.Pressure = 0
		return
	}

	// Частота: при потере устойчивости процесс перескакивает с низкочастотного
	// «чуга» на акустическую моду камеры.
	freq := c.Config.ChugFrequency
	if f := ov.OscillationFrequency.Or(0); f > 0 {
		c.dominantFreq = f
		c.integrateOscillator(dt, f, ov)
		return
	}
	if c.StabilityMargin < 0.12 {
		freq = c.Config.AcousticFrequency
	} else if c.StabilityMargin < 0.35 {
		t := (0.35 - c.StabilityMargin) / 0.23
		freq = c.Config.ChugFrequency +
			t*(c.Config.AcousticFrequency-c.Config.ChugFrequency)*0.35
	}
	c.dominantFreq = freq
	c.integrateOscillator(dt, freq, ov)
}

// integrateOscillator продвигает осциллятор пульсаций на один шаг.
func (c *Chamber) integrateOscillator(dt, freq float64, ov control.ChamberOverrides) {
	omega := 2 * math.Pi * freq

	// Демпфирование обращается в ноль при исчерпании запаса устойчивости.
	zeta := 0.02 + 0.55*c.StabilityMargin
	if c.StabilityMargin < 0.08 {
		// Отрицательное демпфирование: автоколебание нарастает.
		zeta = -0.02 * (0.08 - c.StabilityMargin) / 0.08
	}

	// Возбуждение — широкополосный шум горения, пропорциональный давлению.
	forcing := c.rng.NormFloat64() * c.MeanPressure * 0.35 * omega *
		ov.OscillationFactor.Or(1)

	accel := forcing - 2*zeta*omega*c.oscillationRate - omega*omega*c.oscillation
	c.oscillationRate += accel * dt
	c.oscillation += c.oscillationRate * dt

	// Амплитуда физически ограничена: при слишком сильных колебаниях
	// нелинейности процесса ограничивают рост.
	limit := 0.45 * c.MeanPressure
	if c.oscillation > limit {
		c.oscillation = limit
		c.oscillationRate *= 0.5
	} else if c.oscillation < -limit {
		c.oscillation = -limit
		c.oscillationRate *= 0.5
	}

	c.Pressure = c.MeanPressure + c.oscillation
	if c.Pressure < 0 {
		c.Pressure = 0
	}

	c.spectrum.Push(c.oscillation)
}

// UpdateStatistics пересчитывает среднеквадратичное и пиковое значение
// пульсаций. Операция линейна по размеру окна и выполняется каждый шаг.
func (c *Chamber) UpdateStatistics() {
	c.PressureRMS = c.spectrum.RMS()
	c.PressurePeak = c.spectrum.Peak()
}

// UpdateSpectrum пересчитывает спектр пульсаций и доминирующую частоту.
//
// Преобразование Фурье на порядок дороже статистики, поэтому вызывается
// только при публикации телеметрии и только для одного двигателя: считать
// спектр для всех девяти на каждом шаге бессмысленно дорого.
func (c *Chamber) UpdateSpectrum() {
	if f, ok := c.spectrum.DominantFrequency(); ok {
		c.DominantFrequency = f
	} else {
		c.DominantFrequency = 0
	}
}

// Spectrum возвращает спектр пульсаций давления.
func (c *Chamber) Spectrum() []SpectrumBin { return c.spectrum.Spectrum() }

// GasSideCoefficient возвращает коэффициент теплоотдачи от газа к стенке,
// Вт/(м²·К), по соотношению Бартца.
//
//	h = (0.026/D^0.2)·(μ^0.2·cp/Pr^0.6)·(p_к/c*)^0.8·σ
//
// Соотношение выведено не из ниоткуда: это критериальная зависимость
// Дитуса–Бёльтера для турбулентного пограничного слоя, Nu = 0.026·Re^0.8·Pr^0.4,
// приложенная к течению в сопле. Показатель 0.8 берётся именно из числа
// Рейнольдса, а связь с давлением возникает потому, что горловина заперта:
// массовый поток через неё равен p_к/c*, то есть пропорционален давлению.
// Отсюда и знаменитое h ∝ p^0.8.
//
// Эмпирика здесь — постоянная 0.026 и сами показатели степени: они подобраны
// по опытам с пограничным слоем. Считать это место «из первых принципов»
// означало бы решать уравнения Навье–Стокса с моделью турбулентности,
// то есть заниматься вычислительной газодинамикой, а не полётом в реальном
// времени. Бартцем пользуются при проектном расчёте камеры, и здесь он же.
func (c *Chamber) GasSideCoefficient(throatDiameter float64, ov control.ChamberOverrides) float64 {
	if c.MeanPressure <= 0 || throatDiameter <= 0 || c.CharacteristicVelocity <= 0 {
		return 0
	}

	pair := c.Pair
	mu, cp, pr := pair.GasViscosity, pair.GasSpecificHeat, pair.GasPrandtl
	if mu <= 0 || cp <= 0 || pr <= 0 {
		return 0
	}

	// Массовый поток через горловину: следствие запирания.
	massFlux := c.MeanPressure / c.CharacteristicVelocity

	// σ — поправка на изменение свойств газа поперёк пограничного слоя.
	// У холодной стенки газ у неё плотнее, и теплоотдача выше, чем даёт
	// расчёт по параметрам ядра потока.
	const sigma = 0.8

	h := 0.026 / math.Pow(throatDiameter, 0.2) *
		(math.Pow(mu, 0.2) * cp / math.Pow(pr, 0.6)) *
		math.Pow(massFlux, 0.8) * sigma

	return h * ov.HeatFluxFactor.Or(1) * (1 + c.TemperatureNonuniformity)
}

// RecoveryTemperature возвращает температуру восстановления, К.
//
// Стенку греет не температура торможения и не статическая температура газа,
// а промежуточная: часть кинетической энергии в пограничном слое переходит
// в тепло, часть уносится теплопроводностью. Доля определяется коэффициентом
// восстановления r = Pr^(1/3).
func (c *Chamber) RecoveryTemperature(machNumber float64) float64 {
	pair := c.Pair
	gamma := pair.SpecificHeatRatio
	if gamma <= 1 {
		gamma = 1.2
	}
	pr := pair.GasPrandtl
	if pr <= 0 {
		pr = 0.5
	}

	r := math.Cbrt(pr)
	k := (gamma - 1) / 2 * machNumber * machNumber

	// Температура ядра потока падает по мере разгона, а восстановление
	// возвращает стенке часть скоростного напора.
	return c.Temperature * (1 + r*k) / (1 + k)
}

// WallHeatFlux возвращает удельный тепловой поток в стенку камеры, Вт/м².
//
// Оставлен для телеметрии и как опорная величина: поток считается при
// характерной температуре стенки. Фактический поток на каждом участке
// считает сопло — он зависит от того, насколько эта стенка горяча.
func (c *Chamber) WallHeatFlux(ov control.ChamberOverrides) float64 {
	return c.appliedWallFlux
}

// SetAppliedWallHeatFlux запоминает поток, который фактически сняли стенки.
func (c *Chamber) SetAppliedWallHeatFlux(q float64) { c.appliedWallFlux = q }

// AppliedWallHeatFlux возвращает поток, фактически поданный на стенки
// на последнем шаге, Вт/м².
//
// В телеметрию идёт именно он, а не пересчёт: иначе показание расходилось бы
// с тем, что видела тепловая модель.
func (c *Chamber) AppliedWallHeatFlux() float64 { return c.appliedWallFlux }
