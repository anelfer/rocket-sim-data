package env

import (
	"math"
	"math/rand"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Отработавшая ступень: неуправляемый вход в атмосферу.
//
// После разделения ступень лишена системы стабилизации, а аэродинамический
// момент относительно центра масс не скомпенсирован. Пустая ступень имеет центр
// масс в двигательном отсеке, а центр давления при поперечном обтекании —
// примерно посередине корпуса, поэтому положение «носом вперёд» статически
// неустойчиво: корпус разворачивает, и он начинает кувыркаться.
//
// Здесь интегрируются одновременно поступательное и вращательное движение,
// считается раздельный нагрев носка, боковой поверхности и донной части,
// а также состояние ударного слоя вплоть до ионизации.
// -----------------------------------------------------------------------------

// StageOutcome — состояние первой ступени.
type StageOutcome int

const (
	// StageAttached — ступень ещё в составе ракеты, до разделения.
	//
	// Это состояние нужно, чтобы телеметрия ступени существовала с момента
	// старта. Иначе панели «высота и скорость ступени», «кувыркание»,
	// «нагрузки» пустуют первые две с половиной минуты полёта, хотя ступень
	// всё это время летит — просто в составе носителя.
	StageAttached StageOutcome = iota
	StageFlying                // отделилась и летит самостоятельно
	StageBrokenUp              // разрушилась в атмосфере
	StageImpacted              // столкнулась с поверхностью
)

// String возвращает название состояния.
func (o StageOutcome) String() string {
	switch o {
	case StageAttached:
		return "Attached"
	case StageFlying:
		return "Flying"
	case StageBrokenUp:
		return "Broken Up"
	case StageImpacted:
		return "Impacted"
	default:
		return "Unknown"
	}
}

// -----------------------------------------------------------------------------
// Тепловые узлы
// -----------------------------------------------------------------------------

// ThermalNode — участок конструкции с собственной температурой.
//
// Разделение на узлы нужно потому, что нагрев по корпусу распределён крайне
// неравномерно: в критической точке поток на порядок выше, чем на боковой
// поверхности, и на два порядка выше, чем в донной области, укрытой следом.
type ThermalNode struct {
	Name string

	// Normal — внешняя нормаль участка в связанных осях.
	Normal physics.Vec3

	// Area — площадь участка, м².
	Area float64

	// HeatCapacity — теплоёмкость участка целиком, Дж/К.
	HeatCapacity float64

	// EffectiveRadius — радиус затупления для формулы Саттона — Грейвса, м.
	EffectiveRadius float64

	// MeltingPoint — температура потери прочности, К.
	MeltingPoint float64

	Temperature float64 // текущая температура, К
	PeakFlux    float64 // максимальный поток за полёт, Вт/м²
	PeakTemp    float64 // максимальная температура за полёт, К
}

// StructuralLimits — пределы прочности конструкции при входе в атмосферу.
type StructuralLimits struct {
	// MaxDynamicPressure — предельный скоростной напор, Па. Пустая ступень
	// без наддува баков теряет устойчивость обечайки около 90 кПа.
	MaxDynamicPressure float64

	// MaxLateralLoad — предельная поперечная нагрузка при кувыркании, Па.
	// Корпус рассчитан на осевое сжатие и разрушается от поперечного напора
	// заметно раньше, чем от осевого.
	MaxLateralLoad float64

	// MaxSpinRate — предельная скорость вращения, рад/с. Выше неё центробежные
	// нагрузки разрывают конструкцию.
	MaxSpinRate float64
}

// DefaultStructuralLimits возвращает пределы для пустой ступени носителя.
func DefaultStructuralLimits() StructuralLimits {
	return StructuralLimits{
		MaxDynamicPressure: 90000,
		MaxLateralLoad:     45000,
		MaxSpinRate:        3.5,
	}
}

// -----------------------------------------------------------------------------
// Состояние
// -----------------------------------------------------------------------------

// RigidState — полное состояние твёрдого тела.
type RigidState struct {
	Position        physics.Vec3
	Velocity        physics.Vec3
	Orientation     physics.Quaternion
	AngularVelocity physics.Vec3 // в связанных осях, рад/с
}

// SpentStage — отработавшая ступень в свободном полёте.
type SpentStage struct {
	Name string

	State RigidState
	Mass  float64
	Shape physics.AeroShape

	Inertia physics.InertiaTensor
	Limits  StructuralLimits

	Nodes []ThermalNode
	Cabin ThermalNode // усреднённое состояние конструкции для сводки

	Plasma physics.PlasmaState

	Outcome        StageOutcome
	SeparationTime float64
	OutcomeTime    float64

	PeakHeatFlux float64
	PeakDynamicQ float64
	PeakSpinRate float64
	MaxAltitude  float64

	OutcomeLatitude  float64
	OutcomeLongitude float64
	OutcomeAltitude  float64
	OutcomeReason    string

	// Debris — оценка зоны падения обломков, заполняется при разрушении.
	Debris *DebrisField

	wind WindModel
}

// NewSpentStage создаёт отработавшую ступень в момент разделения.
//
// tipOffRate задаёт начальную угловую скорость от несимметричного срабатывания
// узлов разделения: толкатели никогда не срабатывают строго одновременно,
// поэтому ступень всегда получает небольшую закрутку.
func NewSpentStage(
	name string,
	position, velocity physics.Vec3,
	mass, length, radius, skinTempK, t float64,
	tipOff physics.Vec3,
	wind WindModel,
) *SpentStage {
	shape := physics.DefaultStageShape(length, radius)

	// Ступень отделяется ориентированной по потоку: до разделения она летела
	// под управлением второй ступени.
	forward := velocity.Unit()
	if forward.Norm() < 0.5 {
		forward = physics.Vec3{X: 1}
	}
	right := forward.Cross(position.Unit()).Unit()
	if right.Norm() < 0.5 {
		right = physics.Vec3{Y: 1}
	}
	up := forward.Cross(right).Unit()

	s := &SpentStage{
		Name: name,
		State: RigidState{
			Position:        position,
			Velocity:        velocity,
			Orientation:     physics.QuaternionFromBasis(forward, right, up),
			AngularVelocity: tipOff,
		},
		Mass:           mass,
		Shape:          shape,
		Inertia:        physics.CylinderInertia(mass, radius, length),
		Limits:         DefaultStructuralLimits(),
		Outcome:        StageFlying,
		SeparationTime: t,
		MaxAltitude:    position.Norm() - physics.EarthRadius,
		wind:           wind,
	}
	s.initNodes(skinTempK)
	return s
}

// initNodes создаёт тепловые узлы конструкции.
func (s *SpentStage) initNodes(skinTempK float64) {
	shape := s.Shape
	sideArea := 2 * math.Pi * shape.Radius * shape.Length
	capArea := math.Pi * shape.Radius * shape.Radius

	// Массы распределены по площади, теплоёмкость алюминиевого сплава
	// около 900 Дж/(кг·К).
	const specificHeat = 900.0
	totalArea := sideArea + 2*capArea

	node := func(name string, normal physics.Vec3, area, radius, melting float64) ThermalNode {
		return ThermalNode{
			Name:            name,
			Normal:          normal.Unit(),
			Area:            area,
			HeatCapacity:    s.Mass * (area / totalArea) * specificHeat,
			EffectiveRadius: radius,
			MeltingPoint:    melting,
			Temperature:     skinTempK,
		}
	}

	s.Nodes = []ThermalNode{
		// Носовая часть: межступенчатый переходник, острая кромка,
		// поэтому радиус затупления мал и поток максимален.
		node("nose", physics.Vec3{X: 1}, capArea, 0.3, 850),
		// Боковая поверхность бака.
		node("side", physics.Vec3{Y: 1}, sideArea, shape.Radius, 800),
		// Донная часть с двигателями: массивная, укрыта следом,
		// сделана из жаропрочных сплавов.
		node("base", physics.Vec3{X: -1}, capArea, shape.Radius, 1200),
	}
	s.Cabin = s.Nodes[1]
}

// Altitude возвращает текущую высоту ступени.
func (s *SpentStage) Altitude() float64 {
	return s.State.Position.Norm() - physics.EarthRadius
}

// Alive сообщает, продолжается ли полёт ступени.
func (s *SpentStage) Alive() bool { return s.Outcome == StageFlying }

// SpinRate возвращает модуль угловой скорости, рад/с.
func (s *SpentStage) SpinRate() float64 { return s.State.AngularVelocity.Norm() }

// AngleOfAttack возвращает угол между продольной осью и потоком, градусы.
func (s *SpentStage) AngleOfAttack(t float64) float64 {
	flow := s.airflow(s.State, t)
	if flow.Norm() < 1e-6 {
		return 0
	}
	forward := s.State.Orientation.Rotate(physics.Vec3{X: 1})
	return physics.AngleOfAttack(forward, flow.Scale(-1))
}

// airflow возвращает скорость набегающего потока в инерциальных осях:
// скорость тела относительно воздуха с учётом ветра.
func (s *SpentStage) airflow(st RigidState, t float64) physics.Vec3 {
	air := physics.CorotatingVelocity(st.Position)
	if s.wind != nil {
		altitude := st.Position.Norm() - physics.EarthRadius
		air = air.Add(s.wind.Velocity(st.Position, altitude, t))
	}
	return st.Velocity.Sub(air)
}

// -----------------------------------------------------------------------------
// Интегрирование
// -----------------------------------------------------------------------------

// rigidDerivative — производные состояния твёрдого тела.
type rigidDerivative struct {
	dPosition    physics.Vec3
	dVelocity    physics.Vec3
	dOrientation physics.Quaternion
	dOmega       physics.Vec3
}

// derivative вычисляет производные состояния.
func (s *SpentStage) derivative(st RigidState, t float64) rigidDerivative {
	altitude := st.Position.Norm() - physics.EarthRadius
	atm := physics.Atmosphere(altitude)

	vRel := s.airflow(st, t)
	speed := vRel.Norm()

	acc := physics.GravityAcceleration(st.Position)
	torque := physics.Vec3{}

	if speed > 1e-3 && atm.Density > 0 && s.Mass > 0 {
		// Поток в связанных осях: направлен навстречу движению.
		flowBody := st.Orientation.RotateInverse(vRel.Scale(-1))

		mach := 0.0
		if atm.SoundSpeed > 0 {
			mach = speed / atm.SoundSpeed
		}

		// Угол между продольной осью и набегающим потоком.
		alpha := math.Acos(physics.Clamp(-flowBody.Unit().X, -1, 1))

		// Площадь и коэффициент зависят от ориентации: кувыркающийся корпус
		// подставляет потоку то мидель, то всю боковую проекцию.
		area := s.Shape.EffectiveArea(alpha)
		cd := s.Shape.DragCoefficientAtAngle(mach, alpha)

		q := physics.DynamicPressure(atm.Density, speed)
		drag := q * cd * area

		acc = acc.Add(vRel.Unit().Scale(-drag / s.Mass))

		torque = s.Shape.AerodynamicTorque(q, flowBody, st.AngularVelocity, mach)
	}

	return rigidDerivative{
		dPosition:    st.Velocity,
		dVelocity:    acc,
		dOrientation: st.Orientation.Derivative(st.AngularVelocity),
		dOmega:       s.Inertia.AngularAcceleration(st.AngularVelocity, torque),
	}
}

func advanceRigid(st RigidState, d rigidDerivative, dt float64) RigidState {
	return RigidState{
		Position:        st.Position.Add(d.dPosition.Scale(dt)),
		Velocity:        st.Velocity.Add(d.dVelocity.Scale(dt)),
		Orientation:     st.Orientation.Add(d.dOrientation.Scale(dt)).Normalized(),
		AngularVelocity: st.AngularVelocity.Add(d.dOmega.Scale(dt)),
	}
}

// Step продвигает состояние ступени на dt секунд.
func (s *SpentStage) Step(dt, t float64) {
	if !s.Alive() {
		return
	}

	// Рунге — Кутта 4-го порядка одновременно для поступательного
	// и вращательного движения: они связаны через ориентацию, которая
	// определяет площадь сопротивления.
	k1 := s.derivative(s.State, t)
	k2 := s.derivative(advanceRigid(s.State, k1, dt/2), t+dt/2)
	k3 := s.derivative(advanceRigid(s.State, k2, dt/2), t+dt/2)
	k4 := s.derivative(advanceRigid(s.State, k3, dt), t+dt)

	weighted := rigidDerivative{
		dPosition: k1.dPosition.Add(k2.dPosition.Scale(2)).
			Add(k3.dPosition.Scale(2)).Add(k4.dPosition).Scale(1.0 / 6.0),
		dVelocity: k1.dVelocity.Add(k2.dVelocity.Scale(2)).
			Add(k3.dVelocity.Scale(2)).Add(k4.dVelocity).Scale(1.0 / 6.0),
		dOrientation: k1.dOrientation.Add(k2.dOrientation.Scale(2)).
			Add(k3.dOrientation.Scale(2)).Add(k4.dOrientation).Scale(1.0 / 6.0),
		dOmega: k1.dOmega.Add(k2.dOmega.Scale(2)).
			Add(k3.dOmega.Scale(2)).Add(k4.dOmega).Scale(1.0 / 6.0),
	}

	s.State = advanceRigid(s.State, weighted, dt)

	altitude := s.Altitude()
	if altitude > s.MaxAltitude {
		s.MaxAltitude = altitude
	}
	if spin := s.SpinRate(); spin > s.PeakSpinRate {
		s.PeakSpinRate = spin
	}

	s.updateEnvironment(dt, t)
	s.checkDestruction(t)
}

// updateEnvironment пересчитывает нагрев узлов и состояние ударного слоя.
func (s *SpentStage) updateEnvironment(dt, t float64) {
	altitude := s.Altitude()
	atm := physics.Atmosphere(altitude)

	vRel := s.airflow(s.State, t)
	speed := vRel.Norm()

	mach := 0.0
	if atm.SoundSpeed > 0 {
		mach = speed / atm.SoundSpeed
	}

	q := physics.DynamicPressure(atm.Density, speed)
	if q > s.PeakDynamicQ {
		s.PeakDynamicQ = q
	}

	// Состояние ударного слоя.
	s.Plasma = physics.AnalyzePlasma(atm.Temperature, atm.Density, speed,
		physics.DefaultTelemetryFrequency)

	// Направление потока в связанных осях.
	flowBody := physics.Vec3{}
	if speed > 1e-6 {
		flowBody = s.State.Orientation.RotateInverse(vRel.Scale(-1)).Unit()
	}

	recovery := physics.RecoveryTemperature(atm.Temperature, mach)
	radEq := physics.RadiativeEquilibriumTemperature(altitude)

	for i := range s.Nodes {
		n := &s.Nodes[i]

		// Экспозиция узла: косинус угла между его нормалью и потоком.
		// Узлы, отвёрнутые от потока, попадают в след и почти не греются.
		exposure := 0.0
		if speed > 1e-6 {
			exposure = math.Max(0, n.Normal.Dot(flowBody))
		}
		// Донная область в следе получает около 5 % потока критической точки,
		// но не ноль: рециркуляция в отрывной зоне всё же греет конструкцию.
		exposure = 0.05 + 0.95*exposure

		stagnation := physics.StagnationHeatFlux(atm.Density, speed, n.EffectiveRadius)
		flux := stagnation * exposure
		if flux > n.PeakFlux {
			n.PeakFlux = flux
		}
		if flux > s.PeakHeatFlux {
			s.PeakHeatFlux = flux
		}

		// Тепловой баланс узла: конвекция плюс излучение.
		var conductance, source float64

		if stagnation > 0 && recovery > physics.CelsiusToKelvin(0) {
			h := flux / math.Max(recovery-300, 1) * n.Area
			if h > 0 {
				conductance += h
				source += h * recovery
			}
		}

		hRad := 4 * 0.85 * physics.StefanBoltzmann *
			math.Pow(n.Temperature, 3) * n.Area
		conductance += hRad
		source += hRad * radEq

		if conductance > 0 {
			equilibrium := source / conductance
			tau := n.HeatCapacity / conductance
			n.Temperature = physics.RelaxTo(n.Temperature, equilibrium, dt, tau)
		}

		if n.Temperature > n.PeakTemp {
			n.PeakTemp = n.Temperature
		}
	}

	// Сводная температура — самая горячая точка конструкции.
	hottest := 0
	for i := range s.Nodes {
		if s.Nodes[i].Temperature > s.Nodes[hottest].Temperature {
			hottest = i
		}
	}
	s.Cabin = s.Nodes[hottest]
}

// checkDestruction проверяет условия разрушения или падения.
func (s *SpentStage) checkDestruction(t float64) {
	altitude := s.Altitude()
	atm := physics.Atmosphere(altitude)
	vRel := s.airflow(s.State, t)
	speed := vRel.Norm()
	q := physics.DynamicPressure(atm.Density, speed)

	alpha := s.AngleOfAttack(t) * math.Pi / 180

	// Поперечная составляющая напора: при кувыркании корпус подставляет потоку
	// боковую поверхность, для которой он рассчитан гораздо хуже.
	lateral := q * math.Abs(math.Sin(alpha))

	switch {
	case q > s.Limits.MaxDynamicPressure:
		s.finish(StageBrokenUp, t, "осевой напор")
	case lateral > s.Limits.MaxLateralLoad:
		s.finish(StageBrokenUp, t, "поперечная нагрузка при кувыркании")
	case s.SpinRate() > s.Limits.MaxSpinRate:
		s.finish(StageBrokenUp, t, "центробежные нагрузки")
	case s.hottestOverLimit():
		s.finish(StageBrokenUp, t, "прогорание конструкции")
	case altitude <= 0:
		s.State.Position = s.State.Position.Unit().Scale(physics.EarthRadius)
		s.State.Velocity = physics.CorotatingVelocity(s.State.Position)
		s.finish(StageImpacted, t, "удар о поверхность")
	}
}

func (s *SpentStage) hottestOverLimit() bool {
	for i := range s.Nodes {
		if s.Nodes[i].Temperature > s.Nodes[i].MeltingPoint {
			return true
		}
	}
	return false
}

// Telemetry собирает снимок состояния ступени.
func (s *SpentStage) Telemetry(t float64) *SpentStageTelemetry {
	altitude := s.Altitude()
	atm := physics.Atmosphere(altitude)

	vRel := s.airflow(s.State, t)
	speed := vRel.Norm()

	mach := 0.0
	if atm.SoundSpeed > 0 {
		mach = speed / atm.SoundSpeed
	}

	geo := physics.ECIToGeodetic(s.State.Position, t)
	if !s.Alive() {
		geo.Latitude = s.OutcomeLatitude
		geo.Longitude = s.OutcomeLongitude
	}

	attitude := s.State.Orientation.AttitudeIn(physics.NewLocalFrame(s.State.Position))
	spinDeg := s.SpinRate() * 180 / math.Pi

	tel := &SpentStageTelemetry{
		Name:      s.Name,
		Outcome:   s.Outcome.String(),
		Reason:    s.OutcomeReason,
		Altitude:  altitude,
		Latitude:  geo.Latitude,
		Longitude: geo.Longitude,
		Speed:     speed,
		Mach:      mach,
		HeatFlux:  physics.StagnationHeatFlux(atm.Density, speed, s.Shape.Radius),
		DynamicQ:  physics.DynamicPressure(atm.Density, speed),

		AngleOfAttack: s.AngleOfAttack(t),
		SpinRate:      spinDeg,
		Pitch:         attitude.Pitch,
		Yaw:           attitude.Yaw,
		Roll:          attitude.Roll,
		// Кувырканием считается вращение быстрее градуса в секунду:
		// ниже этого корпус можно считать ориентированным.
		Tumbling: spinDeg > 1.0,

		ShockTemp:          physics.KelvinToCelsius(s.Plasma.ShockTemperature),
		IonizationFraction: s.Plasma.IonizationFraction,
		PlasmaFrequency:    s.Plasma.PlasmaFrequency,
		RadioBlackout:      s.Plasma.RadioBlackout,
		Luminous:           s.Plasma.Luminous,

		MaxAltitude:  s.MaxAltitude,
		PeakHeatFlux: s.PeakHeatFlux,
		PeakDynamicQ: s.PeakDynamicQ,
		PeakSpinRate: s.PeakSpinRate * 180 / math.Pi,
		OutcomeTime:  s.OutcomeTime,
	}

	tel.Nodes = make([]ThermalNodeTelemetry, len(s.Nodes))
	hottest := 0.0
	for i, n := range s.Nodes {
		tel.Nodes[i] = ThermalNodeTelemetry{
			Name:        n.Name,
			Temperature: physics.KelvinToCelsius(n.Temperature),
			PeakTemp:    physics.KelvinToCelsius(n.PeakTemp),
			HeatFlux:    n.PeakFlux,
			MeltingsAt:  physics.KelvinToCelsius(n.MeltingPoint),
		}
		if n.Temperature > hottest {
			hottest = n.Temperature
		}
	}
	tel.HottestNode = physics.KelvinToCelsius(hottest)

	if s.Debris != nil {
		tel.Debris = s.Debris.telemetry()
	}

	return tel
}

// telemetry собирает сводку по зоне падения обломков.
func (d *DebrisField) telemetry() *DebrisTelemetry {
	out := &DebrisTelemetry{
		BreakupLatitude:       d.BreakupLatitude,
		BreakupLongitude:      d.BreakupLongitude,
		BreakupAltitude:       d.BreakupAltitude,
		CenterLatitude:        d.CenterLatitude,
		CenterLongitude:       d.CenterLongitude,
		SemiMajorAxis:         d.SemiMajorAxis,
		SemiMinorAxis:         d.SemiMinorAxis,
		BearingDegrees:        d.BearingDegrees,
		SurvivingMassFraction: d.SurvivingMassFraction,
	}
	for _, imp := range d.Impacts {
		out.Fragments = append(out.Fragments, FragmentTelemetry{
			Name:        imp.Class.Name,
			Mass:        imp.Class.Mass,
			Ballistic:   imp.Class.BallisticCoefficient(),
			Reached:     imp.Reached,
			Latitude:    imp.Latitude,
			Longitude:   imp.Longitude,
			Downrange:   imp.Downrange,
			FlightTime:  imp.FlightTime,
			ImpactSpeed: imp.ImpactSpeed,
		})
	}
	return out
}

func (s *SpentStage) finish(outcome StageOutcome, t float64, reason string) {
	s.Outcome = outcome
	s.OutcomeTime = t
	s.OutcomeReason = reason

	geo := physics.ECIToGeodetic(s.State.Position, t)
	s.OutcomeLatitude = geo.Latitude
	s.OutcomeLongitude = geo.Longitude
	s.OutcomeAltitude = geo.Altitude

	if outcome == StageBrokenUp {
		s.Debris = EstimateDebrisField(s.State.Position, s.State.Velocity, s.wind, t)
	}
}

// RandomTipOff возвращает начальную закрутку от несимметричного срабатывания
// узлов разделения, рад/с.
func RandomTipOff(rng *rand.Rand) physics.Vec3 {
	// Разброс порядка нескольких градусов в секунду по поперечным осям
	// и заметно меньше по продольной.
	return physics.Vec3{
		X: rng.NormFloat64() * 0.01,
		Y: rng.NormFloat64() * 0.03,
		Z: rng.NormFloat64() * 0.03,
	}
}
