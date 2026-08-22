package propulsion

import (
	"math"

	"rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Сопло и регенеративное охлаждение.
//
// Стенка разбита на пять участков, потому что тепловая нагрузка вдоль тракта
// различается на порядок: максимум приходится на район критического сечения,
// где скорость газа переходит через звуковую, а площадь сечения минимальна.
//
// Унос материала в горловине постепенно увеличивает её площадь. Это меняет
// давление в камере, коэффициент тяги и удельный импульс даже при полностью
// неизменном положении клапанов — двигатель «стареет» прямо по ходу работы.
// -----------------------------------------------------------------------------

// WallStation — участок стенки со своим тепловым режимом.
type WallStation struct {
	Name string

	// FluxFactor — доля теплового потока камеры, приходящаяся на участок.
	// В районе критического сечения поток максимален.
	FluxFactor float64

	// Area — площадь участка, м².
	Area float64

	// HeatCapacity — теплоёмкость участка, Дж/К.
	HeatCapacity float64

	// MachNumber — местное число Маха. Нужно для температуры восстановления:
	// чем быстрее газ, тем сильнее его статическая температура отличается
	// от температуры торможения.
	MachNumber float64

	// ChannelAreaFactor — сужение канала охлаждения на участке.
	//
	// У горловины каналы делают уже: скорость охладителя там нужна выше,
	// потому что и поток в стенку максимален. Это конструктивный приём,
	// а не подгонка: то же делают на всех регенеративно охлаждаемых камерах.
	ChannelAreaFactor float64

	// CoolantCoefficient — коэффициент теплоотдачи в охладитель на участке,
	// Вт/(м²·К). Считается на каждом шаге по числу Рейнольдса в канале.
	CoolantCoefficient float64

	// MaterialLimit — предельная температура материала, К.
	MaterialLimit float64

	// Temperature — текущая температура стенки, К.
	Temperature float64

	// HeatFlux — текущий удельный поток, Вт/м².
	HeatFlux float64
}

// Margin возвращает запас до предельной температуры, К.
func (s WallStation) Margin() float64 { return s.MaterialLimit - s.Temperature }

// NozzleConfig — параметры сопла и охлаждающего тракта.
type NozzleConfig struct {
	// InitialThroatArea — начальная площадь критического сечения, м².
	InitialThroatArea float64

	// ExitArea — площадь среза сопла, м².
	ExitArea float64

	// SpecificHeatRatio — показатель адиабаты продуктов сгорания.
	SpecificHeatRatio float64

	// NozzleEfficiency — отношение действительного коэффициента тяги
	// к идеальному изоэнтропическому, 0…1. Ноль трактуется как единица —
	// так нулевое значение структуры остаётся осмысленным для двигателей,
	// у которых эта поправка не откалибрована.
	//
	// Идеальная формула Cf исходит из одномерного изоэнтропического течения:
	// ни трения о стенку, ни двумерного расширения на конце сопла, ни потерь
	// на пограничный слой она не знает. Реальное сопло всегда отдаёт немного
	// меньше — единицы процентов у хорошо спрофилированного колокола.
	// Без этой поправки паспортные давление в камере, расход и диаметр
	// горловины реального двигателя нельзя свести с его же паспортным
	// удельным импульсом: по одним только этим трём числам обратным счётом
	// получается c*, которого не бывает ни у одного метанового топлива,
	// — избыток на самом деле сидит в Cf, а не в топливе.
	NozzleEfficiency float64

	// CoolingFraction — доля расхода горючего, идущая в рубашку охлаждения.
	CoolingFraction float64

	// CoolingResistance — гидравлическое сопротивление охлаждающего тракта.
	CoolingResistance float64

	// CoolingDesignFlow — расчётный расход охладителя, кг/с.
	CoolingDesignFlow float64

	// Геометрия охлаждающего тракта. По ней считается теплоотдача
	// в охладитель, а не задаётся числом: проводимость участка следует
	// из числа каналов, их сечения, толщины и материала огневой стенки.
	//
	// ChannelCount — число каналов по окружности.
	ChannelCount int

	// ChannelWidth, ChannelHeight — сечение канала в цилиндрической части, м.
	ChannelWidth  float64
	ChannelHeight float64

	// WallThickness — толщина огневой стенки, м.
	WallThickness float64

	// WallConductivity — теплопроводность стенки, Вт/(м·К).
	// У медных сплавов, из которых делают огневое днище, около 350.
	WallConductivity float64

	// ErosionThreshold — температура стенки, выше которой начинается
	// заметный унос материала, К.
	ErosionThreshold float64

	// ErosionRate — коэффициент скорости уноса, м/(с·К).
	ErosionRate float64

	// ThroatRadius — начальный радиус критического сечения, м.
	ThroatRadius float64
}

// Nozzle — состояние сопла и охлаждения.
type Nozzle struct {
	Config NozzleConfig

	Stations []WallStation

	// ThroatArea — текущая площадь критического сечения, м².
	ThroatArea float64

	// WallDamage — накопленное повреждение стенки, 0…1.
	//
	// Пока стенка холоднее предела, повреждение не растёт. Выше предела
	// материал теряет прочность тем быстрее, чем больше перегрев, и при
	// единице прогорает насквозь.
	WallDamage float64

	// BurnedThrough — стенка прогорела. Двигатель после этого не работает.
	BurnedThrough bool

	// ThroatErosion — накопленный унос по радиусу, м.
	ThroatErosion float64

	// ExpansionRatio — текущая степень расширения.
	ExpansionRatio float64

	// ExitPressure — давление на срезе сопла, Па.
	ExitPressure float64

	// ThrustCoefficient — коэффициент тяги.
	ThrustCoefficient float64

	// ExhaustVelocity — эффективная скорость истечения, м/с.
	ExhaustVelocity float64

	// ExpansionRegime — отношение давления на срезе к забортному.
	ExpansionRegime float64

	// Охлаждающий тракт.
	CoolingFlow          float64 // кг/с
	CoolantInletTemp     float64 // К
	CoolantOutletTemp    float64 // К
	CoolingPressureDrop  float64 // Па
	CoolingEffectiveness float64 // 0…1
	AbsorbedHeat         float64 // Вт
}

// NewNozzle создаёт сопло в холодном состоянии.
func NewNozzle(cfg NozzleConfig, ambientTemperature float64) *Nozzle {
	n := &Nozzle{
		Config:               cfg,
		ThroatArea:           cfg.InitialThroatArea,
		CoolantInletTemp:     ambientTemperature,
		CoolantOutletTemp:    ambientTemperature,
		CoolingEffectiveness: 1,
	}
	n.Stations = defaultStations(ambientTemperature)
	n.updateGeometry()
	return n
}

// defaultStations возвращает разбиение тракта на участки.
//
// Задаются только обводы и материал: площадь участка, теплоёмкость, местное
// число Маха, сужение каналов охлаждения и предел материала. Теплоотвод здесь
// не задаётся вовсе — он считается на каждом шаге из числа Рейнольдса
// в канале, а приход тепла — из соотношения Бартца.
//
// FluxFactor — форма распределения теплоотдачи вдоль тракта. В камере газ
// плотный, но медленный; в горловине массовый поток максимален, и коэффициент
// теплоотдачи достигает пика; дальше по расширяющейся части газ разрежается,
// и теплоотдача падает на порядок.
func defaultStations(ambient float64) []WallStation {
	return []WallStation{
		{
			Name: "chamber", FluxFactor: 0.35, Area: 0.45,
			HeatCapacity: 34000, MachNumber: 0.2, ChannelAreaFactor: 1.0,
			MaterialLimit: 1000, Temperature: ambient,
		},
		{
			// Критическое сечение: максимальный поток при наименьшей площади.
			// Самый нагруженный участок, именно он определяет эрозию.
			// Каналы здесь сужают, чтобы поднять скорость охладителя.
			Name: "throat", FluxFactor: 1.0, Area: 0.10,
			HeatCapacity: 9000, MachNumber: 1.0, ChannelAreaFactor: 0.55,
			MaterialLimit: 1150, Temperature: ambient,
		},
		{
			Name: "divergent_start", FluxFactor: 0.50, Area: 0.25,
			HeatCapacity: 16000, MachNumber: 1.8, ChannelAreaFactor: 0.75,
			MaterialLimit: 1050, Temperature: ambient,
		},
		{
			Name: "divergent_mid", FluxFactor: 0.15, Area: 0.80,
			HeatCapacity: 30000, MachNumber: 2.6, ChannelAreaFactor: 1.1,
			MaterialLimit: 1000, Temperature: ambient,
		},
		{
			// Насадок охлаждается излучением, каналы здесь широкие,
			// но и поток на порядок ниже.
			Name: "exit", FluxFactor: 0.04, Area: 1.50,
			HeatCapacity: 42000, MachNumber: 3.4, ChannelAreaFactor: 1.6,
			MaterialLimit: 1200, Temperature: ambient,
		},
	}
}

// recoveryTemperature возвращает температуру восстановления, К.
//
// Стенку греет не температура торможения и не статическая температура газа,
// а промежуточная: часть кинетической энергии в пограничном слое переходит
// в тепло, часть уносится теплопроводностью. Доля определяется коэффициентом
// восстановления r = Pr^(1/3).
func recoveryTemperature(chamberTemp, gamma, prandtl, mach float64) float64 {
	if gamma <= 1 {
		gamma = 1.2
	}
	if prandtl <= 0 {
		prandtl = 0.5
	}

	r := math.Cbrt(prandtl)
	k := (gamma - 1) / 2 * mach * mach
	return chamberTemp * (1 + r*k) / (1 + k)
}

// coolantCoefficient возвращает коэффициент теплоотдачи в охладитель
// на участке, Вт/(м²·К).
//
// Считается по критериальной зависимости Дитуса–Бёльтера для турбулентного
// течения в канале:
//
//	Nu = 0.023·Re^0.8·Pr^0.4,  h = Nu·λ/D_г
//
// Число Рейнольдса берётся по фактическому массовому потоку в канале, поэтому
// падение расхода само по себе роняет теплоотдачу — отдельного «коэффициента
// эффективности охлаждения» для этого больше не нужно.
func (n *Nozzle) coolantCoefficient(s *WallStation, flow float64, in CoolingInput) float64 {
	cfg := n.Config

	area, hydraulic := n.channelGeometry(s)
	if area <= 0 || hydraulic <= 0 || flow <= 0 ||
		in.CoolantViscosity <= 0 || in.CoolantConductivity <= 0 {

		// Без прокачки остаётся только теплопроводность застойного
		// охладителя и конструкции. Это мало, но не ноль: иначе стенка
		// у выключенного двигателя грелась бы до бесконечности.
		return stagnantCoolantCoefficient
	}

	massFlux := flow / area
	reynolds := massFlux * hydraulic / in.CoolantViscosity

	prandtl := in.CoolantViscosity * in.CoolantSpecificHeat / in.CoolantConductivity
	if prandtl <= 0 {
		prandtl = 5
	}

	nusselt := 0.023 * math.Pow(reynolds, 0.8) * math.Pow(prandtl, 0.4)
	h := nusselt * in.CoolantConductivity / hydraulic

	_ = cfg
	return math.Max(h, stagnantCoolantCoefficient)
}

// stagnantCoolantCoefficient — теплоотдача в неподвижный охладитель,
// Вт/(м²·К).
const stagnantCoolantCoefficient = 150

// channelGeometry возвращает суммарное проходное сечение каналов на участке
// и гидравлический диаметр канала, м.
func (n *Nozzle) channelGeometry(s *WallStation) (area, hydraulic float64) {
	cfg := n.Config
	if cfg.ChannelCount <= 0 || cfg.ChannelWidth <= 0 || cfg.ChannelHeight <= 0 {
		return 0, 0
	}

	factor := s.ChannelAreaFactor
	if factor <= 0 {
		factor = 1
	}

	// Сужается ширина канала: глубину ограничивает толщина рубашки.
	width := cfg.ChannelWidth * factor
	height := cfg.ChannelHeight

	area = float64(cfg.ChannelCount) * width * height
	hydraulic = 4 * width * height / (2 * (width + height))
	return area, hydraulic
}

func (n *Nozzle) updateGeometry() {
	if n.ThroatArea > 0 {
		n.ExpansionRatio = n.Config.ExitArea / n.ThroatArea
	}
}

// HottestStation возвращает самый горячий участок.
func (n *Nozzle) HottestStation() WallStation {
	hottest := n.Stations[0]
	for _, s := range n.Stations[1:] {
		if s.Temperature > hottest.Temperature {
			hottest = s
		}
	}
	return hottest
}

// MinimumMargin возвращает наименьший запас до предельной температуры, К.
func (n *Nozzle) MinimumMargin() float64 {
	min := math.Inf(1)
	for _, s := range n.Stations {
		if m := s.Margin(); m < min {
			min = m
		}
	}
	return min
}

// -----------------------------------------------------------------------------
// Газодинамика сопла
// -----------------------------------------------------------------------------

// UpdateFlow пересчитывает давление на срезе, коэффициент тяги и скорость
// истечения.
//
// Давление на срезе находится из уравнения изоэнтропического течения, которое
// связывает степень расширения с числом Маха на срезе. Уравнение решается
// делением отрезка пополам по отношению давлений.
func (n *Nozzle) UpdateFlow(chamberPressure, characteristicVelocity, ambientPressure float64) {
	n.updateGeometry()

	if chamberPressure <= 0 || n.ThroatArea <= 0 {
		n.ExitPressure = ambientPressure
		n.ThrustCoefficient = 0
		n.ExhaustVelocity = 0
		n.ExpansionRegime = 1
		return
	}

	gamma := n.Config.SpecificHeatRatio
	n.ExitPressure = exitPressure(chamberPressure, n.ExpansionRatio, gamma)

	n.ThrustCoefficient = thrustCoefficient(
		chamberPressure, n.ExitPressure, ambientPressure, n.ExpansionRatio, gamma)

	efficiency := n.Config.NozzleEfficiency
	if efficiency <= 0 {
		efficiency = 1
	}
	n.ThrustCoefficient *= efficiency

	n.ExhaustVelocity = characteristicVelocity * n.ThrustCoefficient

	if ambientPressure > 1 {
		n.ExpansionRegime = n.ExitPressure / ambientPressure
	} else {
		// В вакууме отношение теряет смысл: любое сопло недорасширено.
		n.ExpansionRegime = math.Inf(1)
	}
}

// exitPressure решает уравнение площади сопла относительно давления на срезе.
//
// Связь степени расширения с отношением давлений:
//
//	A_e/A_t = (1/M)·[ (2/(γ+1))·(1 + (γ−1)/2·M²) ]^((γ+1)/(2(γ−1)))
//
// в форме, разрешённой через отношение давлений p_e/p_c.
func exitPressure(chamberPressure, expansionRatio, gamma float64) float64 {
	if expansionRatio <= 1 || chamberPressure <= 0 {
		return chamberPressure
	}

	// Отношение площадей как функция отношения давлений p_e/p_к.
	//
	// Обратная величина выражается в замкнутом виде:
	//
	//	A_t/A_e = ((γ+1)/2)^(1/(γ−1)) · pr^(1/γ) · √( (γ+1)/(γ−1)·(1 − pr^((γ−1)/γ)) )
	//
	// На сверхзвуковой ветви функция монотонно убывает по pr: чем ниже
	// давление на срезе, тем длиннее должно быть сопло.
	areaRatio := func(pr float64) float64 {
		if pr <= 0 || pr >= 1 {
			return 1
		}
		g := gamma
		inverse := math.Pow((g+1)/2, 1/(g-1)) *
			math.Pow(pr, 1/g) *
			math.Sqrt((g+1)/(g-1)*(1-math.Pow(pr, (g-1)/g)))
		if inverse <= 0 {
			return math.Inf(1)
		}
		return 1 / inverse
	}

	// Соотношение площади и давления двузначно: одному отношению площадей
	// отвечают дозвуковое и сверхзвуковое решения. Функция имеет минимум
	// в критическом сечении, где отношение площадей равно единице.
	// Нас интересует сверхзвуковая ветвь, поэтому поиск ограничивается
	// давлениями ниже критического — там функция монотонна.
	critical := math.Pow(2/(gamma+1), gamma/(gamma-1))

	lo, hi := 1e-9, critical*0.999999
	for i := 0; i < 100; i++ {
		mid := (lo + hi) / 2
		if areaRatio(mid) > expansionRatio {
			// Давление слишком низкое: такому отношению площадей отвечает
			// более длинное сопло, чем есть.
			lo = mid
		} else {
			hi = mid
		}
	}
	return chamberPressure * (lo + hi) / 2
}

// thrustCoefficient возвращает коэффициент тяги сопла.
//
//	Cf = √( 2γ²/(γ−1) · (2/(γ+1))^((γ+1)/(γ−1)) · (1 − (p_e/p_c)^((γ−1)/γ)) )
//	     + (p_e − p_a)/p_c · A_e/A_t
//
// Первое слагаемое — импульс расширяющегося газа, второе — вклад разности
// давлений на срезе. В вакууме второе слагаемое положительно и добавляет тягу,
// у земли при перерасширении оно отрицательно и тягу отнимает.
func thrustCoefficient(chamberPressure, exitPressure, ambientPressure, expansionRatio, gamma float64) float64 {
	if chamberPressure <= 0 {
		return 0
	}
	g := gamma
	pr := exitPressure / chamberPressure
	if pr >= 1 {
		return 0
	}

	momentum := math.Sqrt(
		2 * g * g / (g - 1) *
			math.Pow(2/(g+1), (g+1)/(g-1)) *
			(1 - math.Pow(pr, (g-1)/g)))

	pressureTerm := (exitPressure - ambientPressure) / chamberPressure * expansionRatio

	cf := momentum + pressureTerm
	if cf < 0 {
		return 0
	}
	return cf
}

// -----------------------------------------------------------------------------
// Охлаждение и эрозия
// -----------------------------------------------------------------------------

// CoolingInput — входные данные охлаждающего тракта.
type CoolingInput struct {
	// FuelFlow — расход горючего через двигатель, кг/с.
	FuelFlow float64

	// CoolantDensity — плотность охладителя, кг/м³.
	CoolantDensity float64

	// CoolantSpecificHeat — теплоёмкость охладителя, Дж/(кг·К).
	CoolantSpecificHeat float64

	// InletTemperature — температура охладителя на входе, К.
	InletTemperature float64

	// GasCoefficient — коэффициент теплоотдачи от газа к стенке, Вт/(м²·К),
	// посчитанный по Бартцу. Поток из него получается уже здесь: он зависит
	// от того, насколько нагрелась сама стенка.
	GasCoefficient float64

	// ChamberTemperature — температура продуктов сгорания, К.
	ChamberTemperature float64

	// SpecificHeatRatio, Prandtl — свойства газа для температуры
	// восстановления.
	SpecificHeatRatio float64
	Prandtl           float64

	// CoolantViscosity — динамическая вязкость охладителя, Па·с.
	CoolantViscosity float64

	// CoolantConductivity — теплопроводность охладителя, Вт/(м·К).
	CoolantConductivity float64

	// CoolingValve — положение клапана охлаждения, 0…1. Позволяет искусственно
	// уменьшить расход охладителя и проследить, как растёт температура стенки
	// и ускоряется эрозия.
	CoolingValve float64

	// Overrides — ручные воздействия на сопло и охлаждающий тракт.
	Overrides control.NozzleOverrides
}

// UpdateCooling считает тепловое состояние стенок и охлаждающего тракта.
func (n *Nozzle) UpdateCooling(dt float64, in CoolingInput) {
	cfg := n.Config

	ov := in.Overrides

	valve := in.CoolingValve
	if valve <= 0 {
		valve = 1
	}
	n.CoolingFlow = in.FuelFlow * cfg.CoolingFraction * clamp01(valve) *
		ov.CoolingFlowFactor.Or(1)
	n.CoolantInletTemp = ov.CoolantInletTemp.Or(in.InletTemperature)

	// Потери давления в тракте: квадратичны по расходу. Рост сопротивления
	// снижает давление подачи горючего и через него — расход и давление
	// в камере.
	if in.CoolantDensity > 0 {
		n.CoolingPressureDrop = cfg.CoolingResistance *
			n.CoolingFlow * n.CoolingFlow / in.CoolantDensity *
			ov.CoolingDropFactor.Or(1)
	} else {
		n.CoolingPressureDrop = 0
	}

	// Тепловое состояние каждого участка.
	//
	// Цепочка сопротивлений последовательная: газ отдаёт тепло стенке,
	// стенка проводит его сквозь себя, охладитель уносит. Полный поток
	// определяется суммой трёх сопротивлений, а температура стенки —
	// тем, сколько на ней падает.
	//
	//	q = (T_восст − T_охл) / (1/h_газ + δ/λ + 1/h_охл)
	//
	// Отсюда и обратная связь: раскалившаяся стенка принимает меньше тепла,
	// потому что разность температур с газом сокращается.
	absorbed := 0.0
	nominalCoolant := 0.0
	actualCoolant := 0.0

	for i := range n.Stations {
		s := &n.Stations[i]

		// Сторона газа: коэффициент по Бартцу, распределённый вдоль тракта.
		hGas := in.GasCoefficient * s.FluxFactor
		recovery := recoveryTemperature(in.ChamberTemperature,
			in.SpecificHeatRatio, in.Prandtl, s.MachNumber)

		// Сторона охладителя: критериальная зависимость Дитуса–Бёльтера
		// по фактическому числу Рейнольдса в канале.
		hCool := n.coolantCoefficient(s, n.CoolingFlow, in)
		hCool *= ov.CoolingConductanceFactor.Or(1)

		// Местное ухудшение охлаждения: засорился один канал, остальные
		// работают штатно. Прогорает при этом именно этот участок.
		if ov.HotSpotStation == s.Name {
			hCool *= ov.HotSpotFactor.Or(1)
		}
		s.CoolantCoefficient = hCool

		nominalCoolant += n.coolantCoefficient(s, cfg.CoolingDesignFlow, in) * s.Area
		actualCoolant += hCool * s.Area

		// Сопротивление стенки по толщине.
		wallResistance := 0.0
		if cfg.WallConductivity > 0 {
			wallResistance = cfg.WallThickness / cfg.WallConductivity
		}

		// Баланс энергии узла стенки:
		//
		//	C·dT/dt = h_газ·A·(T_восст − T) − U_охл·A·(T − T_охл)
		//
		// Равновесие лежит между температурой восстановления и температурой
		// охладителя, ближе к той стороне, которая проводит тепло лучше.
		// Постоянная времени определяется суммой проводимостей: если
		// охлаждение пропало, стенку разгоняет уже сторона газа, и разгоняет
		// быстро — за секунды, а не за минуты.
		hCool = math.Max(hCool, 1e-3)
		coolantConductance := 1 / (wallResistance + 1/hCool)

		total := hGas + coolantConductance
		if total <= 0 {
			continue
		}

		equilibrium := (hGas*recovery + coolantConductance*n.CoolantInletTemp) / total

		capacity := s.HeatCapacity
		if capacity <= 0 {
			capacity = 1000
		}
		tau := capacity / (s.Area * total)
		s.Temperature = equilibrium + (s.Temperature-equilibrium)*math.Exp(-dt/tau)

		// В телеметрию идёт фактический поток со стороны газа: он зависит
		// от того, насколько стенка успела нагреться.
		s.HeatFlux = hGas * math.Max(0, recovery-s.Temperature)

		// Охладитель уносит ровно то, что снял со стенки.
		absorbed += coolantConductance * math.Max(0, s.Temperature-n.CoolantInletTemp) * s.Area
	}
	n.AbsorbedHeat = absorbed

	// Эффективность охлаждения теперь не задаётся, а измеряется: отношение
	// фактической теплоотдачи в охладитель к расчётной. При недоборе расхода
	// она падает сама, потому что падает число Рейнольдса в канале.
	n.CoolingEffectiveness = 1
	if nominalCoolant > 0 {
		n.CoolingEffectiveness = actualCoolant / nominalCoolant
	}

	// Нагрев охладителя в тракте.
	if n.CoolingFlow > 0 && in.CoolantSpecificHeat > 0 {
		n.CoolantOutletTemp = n.CoolantInletTemp +
			absorbed/(n.CoolingFlow*in.CoolantSpecificHeat)
	} else {
		n.CoolantOutletTemp = n.CoolantInletTemp
	}
	// Охладитель не может нагреться выше стенки, которую он охлаждает.
	if hottest := n.HottestStation().Temperature; n.CoolantOutletTemp > hottest {
		n.CoolantOutletTemp = hottest
	}

	n.updateErosion(dt, ov)

	if ov.ThroatAreaDirect.Active {
		n.ThroatArea = math.Max(1e-6, ov.ThroatAreaDirect.V)
		n.updateGeometry()
	}
}

// updateErosion интегрирует унос материала в критическом сечении.
//
// Скорость уноса резко растёт с температурой стенки: пока стенка ниже порога,
// эрозии практически нет, выше — унос идёт тем быстрее, чем горячее материал.
// Зависимость взята степенной, что качественно соответствует испарению
// и химическому взаимодействию с продуктами сгорания.
func (n *Nozzle) updateErosion(dt float64, ov control.NozzleOverrides) {
	var throat *WallStation
	for i := range n.Stations {
		if n.Stations[i].Name == "throat" {
			throat = &n.Stations[i]
			break
		}
	}
	if throat == nil {
		return
	}

	// Прогар считается по самому горячему участку и по его собственному
	// пределу: прогорает то место, где стенке хуже всего, а не обязательно
	// критическое сечение.
	n.updateDamage(dt)

	excess := throat.Temperature - n.Config.ErosionThreshold
	if excess <= 0 {
		return
	}

	// Унос по радиусу за шаг, м.
	rate := n.Config.ErosionRate * math.Pow(excess, 1.6) * ov.ErosionRateFactor.Or(1)
	n.ThroatErosion += rate * dt

	// Пересчёт площади: A = π(r + Δr)².
	r := n.Config.ThroatRadius + n.ThroatErosion
	n.ThroatArea = math.Pi * r * r
	n.updateGeometry()
}

// updateDamage накапливает повреждение перегретой стенки.
//
// Раньше выход за предел не имел последствий вообще: запас мог уйти в минус
// на семьсот градусов, а двигатель работал как ни в чём не бывало. Унос
// материала при таком перегреве составлял доли микрона за весь полёт —
// то есть был чистой декорацией.
//
// Теперь перегрев копится. Скорость взята квадратичной по превышению:
// сотня градусов сверх предела даёт полторы минуты до прогара, три сотни —
// восемь секунд, а семь сотен — полторы. Это соответствует порядку величин
// для медного огневого днища: небольшой перегрев оно терпит долго,
// значительный — секунды.
func (n *Nozzle) updateDamage(dt float64) {
	if n.BurnedThrough {
		return
	}

	worst := 0.0
	for _, s := range n.Stations {
		if excess := s.Temperature - s.MaterialLimit; excess > worst {
			worst = excess
		}
	}
	if worst <= 0 {
		return
	}

	const scale = 300.0 // градусов перегрева, дающих прогар за восемь секунд
	n.WallDamage += (worst / scale) * (worst / scale) / 8 * dt
	if n.WallDamage >= 1 {
		n.WallDamage = 1
		n.BurnedThrough = true
	}
}

// ThroatDiameter возвращает текущий диаметр критического сечения, м.
//
// Именно он входит в соотношение Бартца, и именно поэтому эрозия горловины
// сама по себе меняет теплоотдачу: расточенная горловина греется слабее.
func (n *Nozzle) ThroatDiameter() float64 {
	if n.ThroatArea <= 0 {
		return 0
	}
	return 2 * math.Sqrt(n.ThroatArea/math.Pi)
}

// ThroatErosionMillimeters возвращает накопленный унос в миллиметрах.
func (n *Nozzle) ThroatErosionMillimeters() float64 { return n.ThroatErosion * 1000 }
