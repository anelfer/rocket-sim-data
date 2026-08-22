package vehicle

import (
	"fmt"
	"math"
	"strconv"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Двигатель и его тепловое состояние.
//
// Все температуры внутри модели хранятся в КЕЛЬВИНАХ. Наружу (в Prometheus и
// REST) они отдаются в градусах Цельсия через явные конвертеры. Прежняя модель
// смешивала шкалы: температура в °C умножалась на безразмерный коэффициент
// экранирования, что физически бессмысленно (у шкалы Цельсия произвольный ноль),
// а знак члена охлаждения стенки был перепутан, из-за чего «охлаждение»
// разогревало стенку до 1071 °C даже у заглушенного двигателя.
// -----------------------------------------------------------------------------

// EngineThermalConfig — параметры тепловой модели двигателя.
type EngineThermalConfig struct {
	// FlameTemperature — температура продуктов сгорания при номинальном режиме, К.
	// Для пары РГ-1/жидкий кислород — около 3500 К.
	FlameTemperature float64

	// ChamberTimeConstant — постоянная времени выхода камеры на режим, с.
	ChamberTimeConstant float64

	// WallHeatCapacity — теплоёмкость стенки камеры целиком, Дж/К.
	WallHeatCapacity float64

	// GasSideConductance — теплопередача от газа к стенке, Вт/К.
	GasSideConductance float64

	// CoolantConductance — теплоотвод в регенеративный тракт, Вт/К.
	// Работает только пока идёт расход топлива.
	CoolantConductance float64

	// CoolantTemperature — температура горючего в охлаждающем тракте, К.
	CoolantTemperature float64

	// ExternalArea — площадь наружной поверхности для излучения и обдува, м².
	ExternalArea float64

	// ExitPressureRatio — отношение давления на срезе сопла к давлению
	// в камере. Определяет температуру газа на срезе по адиабате.
	ExitPressureRatio float64

	// GasGamma — показатель адиабаты продуктов сгорания.
	GasGamma float64

	// TurbineInletTemperature — температура на входе турбины ТНА при
	// номинальном режиме, К. У газогенераторной схемы с избытком горючего
	// она намного ниже температуры в камере.
	TurbineInletTemperature float64

	// NoseRadius — эффективный радиус затупления для расчёта
	// конвективного нагрева, м.
	NoseRadius float64

	// ShieldingFactor — доля наружной поверхности, открытой набегающему потоку.
	// 0 — полностью экранирована, 1 — полностью открыта.
	ShieldingFactor float64
}

// DefaultEngineThermalConfig возвращает параметры, соответствующие
// керосин-кислородному двигателю класса Merlin 1D.
func DefaultEngineThermalConfig() EngineThermalConfig {
	return EngineThermalConfig{
		FlameTemperature:        3500.0,
		ChamberTimeConstant:     0.5,
		WallHeatCapacity:        50000.0, // ≈100 кг инконеля
		GasSideConductance:      2000.0,
		CoolantConductance:      10800.0,
		CoolantTemperature:      300.0,
		ExternalArea:            4.0,
		ExitPressureRatio:       0.0072, // Merlin 1D, уровень моря
		GasGamma:                1.20,
		TurbineInletTemperature: 1000.0,
		NoseRadius:              0.5,
		ShieldingFactor:         0.25,
	}
}

// Engine — жидкостный ракетный двигатель.
type Engine struct {
	// ID — устойчивый идентификатор вида "S1-3" или "S2-1".
	//
	// Идентификатор обязан быть уникальным в пределах всего полёта. Раньше
	// нумерация начиналась с единицы в каждой ступени, поэтому двигатель
	// второй ступени переиспользовал метку engine_id="1", и на графиках это
	// выглядело как повторный запуск первого двигателя первой ступени.
	ID string `json:"id"`

	Thrust    float64 `json:"thrust"`    // текущая тяга, Н
	MaxThrust float64 `json:"maxThrust"` // тяга на номинале, Н
	MinThrust float64 `json:"minThrust"` // тяга на минимальном дросселе, Н

	ISP    float64 `json:"isp"`    // текущий удельный импульс, с
	VacIsp float64 `json:"vacIsp"` // удельный импульс в вакууме, с
	SeaIsp float64 `json:"seaIsp"` // удельный импульс на уровне моря, с

	Running bool `json:"running"`

	// AngleRad — угловое положение двигателя в блоке (радианы).
	// Хранится именно угол, а не декартовы координаты: раньше «соседство»
	// определялось через atan2 разности координат, что для кольцевой
	// компоновки давало неверную классификацию.
	AngleRad float64 `json:"-"`

	// Radius — расстояние от оси блока, м. 0 — центральный двигатель.
	Radius float64 `json:"-"`

	// Температуры хранятся в Кельвинах.
	ChamberTempK float64 `json:"-"`
	NozzleTempK  float64 `json:"-"`
	WallTempK    float64 `json:"-"`
	TurbineTempK float64 `json:"-"`

	Thermal EngineThermalConfig `json:"-"`
}

// ChamberTempC и остальные — представление температур в градусах Цельсия
// для телеметрии.
func (e Engine) ChamberTempC() float64 { return physics.KelvinToCelsius(e.ChamberTempK) }
func (e Engine) NozzleTempC() float64  { return physics.KelvinToCelsius(e.NozzleTempK) }
func (e Engine) WallTempC() float64    { return physics.KelvinToCelsius(e.WallTempK) }
func (e Engine) TurbineTempC() float64 { return physics.KelvinToCelsius(e.TurbineTempK) }

// ThrottleFraction возвращает текущий уровень дросселирования (0…1)
// относительно номинальной тяги.
func (e Engine) ThrottleFraction() float64 {
	if e.MaxThrust <= 0 {
		return 0
	}
	return physics.Clamp(e.Thrust/e.MaxThrust, 0, 1)
}

// SetTemperatures устанавливает все температуры двигателя.
func (e *Engine) SetTemperatures(tempK float64) {
	e.ChamberTempK = tempK
	e.NozzleTempK = tempK
	e.WallTempK = tempK
	e.TurbineTempK = tempK
}

// EnvironmentState — условия снаружи двигателя на текущем шаге.
type EnvironmentState struct {
	Altitude      float64 // м
	Density       float64 // кг/м³
	AmbientTempK  float64 // кинетическая температура газа, К
	RelativeSpeed float64 // скорость относительно атмосферы, м/с
	Mach          float64
}

// UpdateThermalState продвигает тепловое состояние двигателя на dt секунд.
//
// Учитываются четыре механизма:
//  1. тепловыделение сгорания (пока двигатель работает);
//  2. регенеративное охлаждение горючим (только при наличии расхода);
//  3. конвективный аэродинамический нагрев набегающим потоком;
//  4. излучение с наружной поверхности.
//
// Уравнение стенки решается полунеявно: линейная часть берётся аналитически
// через экспоненту, поэтому схема устойчива при любом шаге, а не только при
// dt < 2τ, как явный Эйлер.
func (e *Engine) UpdateThermalState(dt float64, env EnvironmentState) {
	if dt <= 0 {
		return
	}
	cfg := e.Thermal

	// --- 1. Камера сгорания -------------------------------------------------
	//
	// Температура продуктов сгорания задаётся химией топливной пары и почти
	// не зависит от режима: дросселирование до 65 % снижает её примерно на 1 %,
	// а не на треть, как получалось при линейной зависимости от тяги.
	if e.Running && e.Thrust > 0 {
		throttle := e.ThrottleFraction()
		target := cfg.FlameTemperature * (0.96 + 0.04*throttle)
		e.ChamberTempK = physics.RelaxTo(e.ChamberTempK, target, dt, cfg.ChamberTimeConstant)
	} else {
		// Двигатель заглушен: камера остывает к температуре стенки.
		e.ChamberTempK = physics.RelaxTo(e.ChamberTempK, e.WallTempK, dt, 4*cfg.ChamberTimeConstant)
	}

	// --- 2. Срез сопла ------------------------------------------------------
	//
	// Адиабатическое расширение: T_e / T_c = (p_e / p_c)^((γ−1)/γ).
	// Ранее вместо этого стоял эмпирический множитель 0.93, из-за которого
	// газ на срезе оказывался почти таким же горячим, как в камере.
	if e.Running && e.Thrust > 0 {
		exponent := (cfg.GasGamma - 1) / cfg.GasGamma
		e.NozzleTempK = e.ChamberTempK * math.Pow(cfg.ExitPressureRatio, exponent)
	} else {
		e.NozzleTempK = physics.RelaxTo(e.NozzleTempK, e.WallTempK, dt, 2*cfg.ChamberTimeConstant)
	}

	// --- 3. Стенка камеры ---------------------------------------------------
	//
	// dTw/dt = [ h_gas·(T_c − Tw) − h_cool·(Tw − T_ох) + q_аэро + q_изл ] / C
	//
	// Линейные по Tw члены объединяются в постоянную времени, нелинейное
	// излучение линеаризуется вокруг текущей температуры.
	var conductance, source float64

	if e.Running && e.Thrust > 0 {
		conductance += cfg.GasSideConductance
		source += cfg.GasSideConductance * e.ChamberTempK

		// Охлаждение работает, только пока через тракт идёт горючее.
		conductance += cfg.CoolantConductance
		source += cfg.CoolantConductance * cfg.CoolantTemperature
	}

	// Аэродинамический нагрев наружной поверхности.
	hAero := physics.ConvectiveHeatTransferCoefficient(
		env.Density, env.RelativeSpeed, cfg.NoseRadius,
		env.AmbientTempK, env.Mach, cfg.ShieldingFactor,
	) * cfg.ExternalArea
	if hAero > 0 {
		recovery := physics.RecoveryTemperature(env.AmbientTempK, env.Mach)
		conductance += hAero
		source += hAero * recovery
	}

	// Излучение. Линеаризация: σ(T⁴−Te⁴) ≈ 4σT³·(T−Te).
	radEq := physics.RadiativeEquilibriumTemperature(env.Altitude)
	hRad := 4 * 0.85 * physics.StefanBoltzmann * math.Pow(e.WallTempK, 3) * cfg.ExternalArea
	conductance += hRad
	source += hRad * radEq

	if conductance > 0 {
		equilibrium := source / conductance
		tau := cfg.WallHeatCapacity / conductance
		e.WallTempK = physics.RelaxTo(e.WallTempK, equilibrium, dt, tau)
	}

	// --- 4. Турбина ТНА -----------------------------------------------------
	//
	// Газогенератор работает с большим избытком горючего, поэтому температура
	// на входе турбины втрое ниже, чем в камере, и слабо зависит от режима.
	if e.Running && e.Thrust > 0 {
		target := cfg.TurbineInletTemperature * (0.85 + 0.15*e.ThrottleFraction())
		e.TurbineTempK = physics.RelaxTo(e.TurbineTempK, target, dt, cfg.ChamberTimeConstant)
	} else {
		e.TurbineTempK = physics.RelaxTo(e.TurbineTempK, e.WallTempK, dt, 8*cfg.ChamberTimeConstant)
	}

	// Страховка от нефизичных значений.
	e.ChamberTempK = math.Max(0, e.ChamberTempK)
	e.NozzleTempK = math.Max(0, e.NozzleTempK)
	e.WallTempK = math.Max(0, e.WallTempK)
	e.TurbineTempK = math.Max(0, e.TurbineTempK)
}

// -----------------------------------------------------------------------------
// Удельный импульс и расход
// -----------------------------------------------------------------------------

// UpdateISP пересчитывает удельный импульс по внешнему давлению.
//
// Интерполяция ведётся по отношению давлений, а не линейно по высоте:
// удельный импульс зависит именно от противодавления на срезе сопла.
// Для вакуумного двигателя, у которого понятие «Isp на уровне моря»
// неприменимо, SeaIsp просто задаётся равным VacIsp.
func (e *Engine) UpdateISP(ambientPressure float64) {
	const seaLevelPressure = 101325.0
	ratio := physics.Clamp(ambientPressure/seaLevelPressure, 0, 1)
	e.ISP = e.VacIsp - (e.VacIsp-e.SeaIsp)*ratio
}

// MassFlow возвращает секундный расход топлива, кг/с.
func (e Engine) MassFlow() float64 {
	if !e.Running || e.ISP <= 0 || e.Thrust <= 0 {
		return 0
	}
	return e.Thrust / (e.ISP * physics.G0)
}

// -----------------------------------------------------------------------------
// Компоновка блока двигателей
// -----------------------------------------------------------------------------

// AngularSeparation возвращает угол между двумя двигателями в блоке (радианы,
// от 0 до π). Считается как разность полярных углов, а не через атан2
// разности координат.
func AngularSeparation(a, b Engine) float64 {
	d := math.Abs(a.AngleRad - b.AngleRad)
	d = math.Mod(d, 2*math.Pi)
	if d > math.Pi {
		d = 2*math.Pi - d
	}
	return d
}

// IsOpposite сообщает, что двигатель расположен диаметрально противоположно.
func IsOpposite(a, b Engine) bool {
	if a.Radius == 0 || b.Radius == 0 {
		return false // центральный двигатель не имеет «противоположного»
	}
	return AngularSeparation(a, b) > 135*math.Pi/180
}

// IsNeighbor сообщает, что двигатель соседний по кольцу.
func IsNeighbor(a, b Engine) bool {
	if a.Radius == 0 || b.Radius == 0 {
		return false
	}
	sep := AngularSeparation(a, b)
	return sep > 0 && sep <= 60*math.Pi/180
}

// RunningEngines возвращает число работающих двигателей.
func RunningEngines(engines []Engine) int {
	count := 0
	for i := range engines {
		if engines[i].Running {
			count++
		}
	}
	return count
}

// TotalThrust возвращает суммарную тягу работающих двигателей.
func TotalThrust(engines []Engine) float64 {
	total := 0.0
	for i := range engines {
		if engines[i].Running {
			total += engines[i].Thrust
		}
	}
	return total
}

// TotalMassFlow возвращает суммарный расход топлива, кг/с.
func TotalMassFlow(engines []Engine) float64 {
	total := 0.0
	for i := range engines {
		total += engines[i].MassFlow()
	}
	return total
}

// UpdateEngine изменяет параметры двигателя по его идентификатору.
// Принимается как полный идентификатор ("S1-3"), так и порядковый номер
// в текущей ступени ("3") — для совместимости с прежним REST API.
func UpdateEngine(id string, thrust float64, running bool, engines []Engine) error {
	idx := findEngine(id, engines)
	if idx < 0 {
		return fmt.Errorf("unknown engine %q", id)
	}
	e := &engines[idx]
	e.Thrust = physics.Clamp(thrust, 0, e.MaxThrust)
	e.Running = running
	return nil
}

// findEngine ищет двигатель по идентификатору или порядковому номеру.
func findEngine(id string, engines []Engine) int {
	for i := range engines {
		if engines[i].ID == id {
			return i
		}
	}
	if n, err := strconv.Atoi(id); err == nil && n >= 1 && n <= len(engines) {
		return n - 1
	}
	return -1
}

// BalanceEngines перераспределяет тягу после отказа двигателя, чтобы
// скомпенсировать возникший момент: противоположный двигатель добавляет тягу,
// соседние — снижают.
func BalanceEngines(engines []Engine, failedIdx int, jitter func() float64) {
	if failedIdx < 0 || failedIdx >= len(engines) {
		return
	}
	failed := engines[failedIdx]
	if failed.Radius == 0 {
		return // отказ центрального двигателя момента не создаёт
	}

	const (
		oppositeShare = 0.30
		neighborShare = 0.15
	)

	for i := range engines {
		if !engines[i].Running || i == failedIdx {
			continue
		}
		switch {
		case IsOpposite(engines[i], failed):
			factor := oppositeShare + jitter()*0.1 - 0.05
			delta := math.Min(failed.Thrust*factor, engines[i].MaxThrust-engines[i].Thrust)
			engines[i].Thrust += delta
		case IsNeighbor(engines[i], failed):
			factor := neighborShare + jitter()*0.05 - 0.025
			delta := math.Min(failed.Thrust*factor, engines[i].Thrust-engines[i].MinThrust)
			if delta > 0 {
				engines[i].Thrust -= delta
			}
		}
	}
}
