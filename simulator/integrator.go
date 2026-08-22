package simulator

import (
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Интегратор уравнений движения.
//
// Состояние ракеты — вектор положения и скорости в инерциальной геоцентрической
// системе (ECI) плюс масса топлива. Такая постановка убирает целый класс прежних
// ошибок:
//
//   - центробежная сила больше не вычитается вручную из гравитации: в
//     инерциальной системе её попросту нет, орбитальное движение получается
//     само собой из интегрирования;
//   - наклонение орбиты становится результатом динамики, а не отдельно
//     «нарисованной» величиной;
//   - наземный трек получается преобразованием ECI → ECEF → широта/долгота,
//     поэтому вращение Земли учитывается ровно один раз.
//
// Интегрирование — классический Рунге — Кутта 4-го порядка. Управление
// (направление тяги, дросселирование) фиксируется на шаге — обычный
// зероордерный удержатель, как в реальной цифровой системе управления.
// -----------------------------------------------------------------------------

// VehicleState — мгновенное состояние ракеты.
type VehicleState struct {
	Position physics.Vec3 // ECI, м
	Velocity physics.Vec3 // ECI, м/с
	FuelMass float64      // кг
}

// Altitude возвращает высоту над сферой радиуса EarthRadius.
func (s VehicleState) Altitude() float64 {
	return s.Position.Norm() - physics.EarthRadius
}

// LocalFrame возвращает локальный базис ENU в текущей точке.
func (s VehicleState) LocalFrame() physics.LocalFrame {
	return physics.NewLocalFrame(s.Position)
}

// AirRelativeVelocity возвращает скорость относительно атмосферы без учёта
// ветра. Атмосфера вращается вместе с Землёй, поэтому именно эта скорость
// определяет сопротивление, число Маха и аэродинамический нагрев.
func (s VehicleState) AirRelativeVelocity() physics.Vec3 {
	return s.Velocity.Sub(physics.CorotatingVelocity(s.Position))
}

// AirRelativeVelocityWithWind возвращает скорость относительно воздуха
// с учётом ветра.
func (s VehicleState) AirRelativeVelocityWithWind(wind physics.Vec3) physics.Vec3 {
	return s.AirRelativeVelocity().Sub(wind)
}

// RadialVelocity — вертикальная (радиальная) составляющая скорости, м/с.
func (s VehicleState) RadialVelocity() float64 {
	return s.Velocity.Dot(s.Position.Unit())
}

// TransverseVelocity — горизонтальная составляющая инерциальной скорости, м/с.
func (s VehicleState) TransverseVelocity() float64 {
	up := s.Position.Unit()
	radial := up.Scale(s.Velocity.Dot(up))
	return s.Velocity.Sub(radial).Norm()
}

// GroundSpeed — горизонтальная скорость относительно поверхности, м/с.
func (s VehicleState) GroundSpeed() float64 {
	rel := s.AirRelativeVelocity()
	up := s.Position.Unit()
	return rel.Sub(up.Scale(rel.Dot(up))).Norm()
}

// ForceModel описывает силы, действующие на ракету в течение одного шага.
// Значения удерживаются постоянными на всём шаге интегрирования.
type ForceModel struct {
	DryMass   float64      // сухая масса, кг
	Thrust    float64      // суммарная тяга, Н
	ThrustDir physics.Vec3 // единичный вектор направления тяги в ECI
	MassFlow  float64      // секундный расход топлива, кг/с
	Area      float64      // площадь миделя, м²

	// SideArea — площадь борта, м². Корпус, поставленный поперёк потока,
	// тормозит именно ею.
	SideArea float64

	// AngleOfAttack — угол между продольной осью и потоком, градусы.
	AngleOfAttack float64

	// Wind — скорость воздуха относительно вращающейся Земли, в осях ECI.
	Wind physics.Vec3

	// DensityFactor — множитель плотности атмосферы для этого пуска.
	// Ноль трактуется как единица, чтобы нулевое значение структуры
	// оставалось осмысленным.
	DensityFactor float64
}

// densityFactor возвращает множитель плотности, приводя ноль к единице.
func (fm ForceModel) densityFactor() float64 {
	if fm.DensityFactor <= 0 {
		return 1
	}
	return fm.DensityFactor
}

// Accelerations — разложение ускорения по источникам (для телеметрии).
type Accelerations struct {
	Gravity physics.Vec3
	Thrust  physics.Vec3
	Drag    physics.Vec3

	// AngleOfAttack — угол между продольной осью и потоком, градусы.
	AngleOfAttack float64

	// Area — площадь, фактически подставленная потоку, м².
	Area  float64
	Total physics.Vec3

	DragForce float64 // модуль силы сопротивления, Н
	Mach      float64
	Density   float64
	DynamicQ  float64 // скоростной напор, Па
}

// Evaluate вычисляет ускорения для заданного состояния.
func (fm ForceModel) Evaluate(s VehicleState) Accelerations {
	mass := fm.DryMass + math.Max(0, s.FuelMass)
	if mass <= 0 {
		mass = 1 // защита от деления на ноль при вырожденной конфигурации
	}

	acc := Accelerations{}
	acc.Gravity = physics.GravityAcceleration(s.Position)

	// Тяга есть, только пока есть топливо.
	if fm.Thrust > 0 && s.FuelMass > 0 {
		acc.Thrust = fm.ThrustDir.Scale(fm.Thrust / mass)
	}

	// Сопротивление считается по скорости относительно воздуха: вращающаяся
	// вместе с Землёй атмосфера плюс ветер.
	altitude := s.Altitude()
	atm := physics.Atmosphere(altitude)
	atm.Density *= fm.densityFactor()

	vRel := s.AirRelativeVelocityWithWind(fm.Wind)
	speed := vRel.Norm()

	acc.Density = atm.Density
	acc.DynamicQ = physics.DynamicPressure(atm.Density, speed)

	if speed > 1e-3 && atm.Density > 0 {
		if atm.SoundSpeed > 0 {
			acc.Mach = speed / atm.SoundSpeed
		}
		// Сопротивление раскладывается по двум направлениям: вдоль корпуса
		// работает обтекаемая форма с малым коэффициентом, поперёк — цилиндр
		// с коэффициентом около полутора. Носом вперёд первое слагаемое
		// главное, брюхом вперёд — второе, и оно на два порядка больше.
		//
		// Без этого разложения вход брюхом ничем не отличался бы от входа
		// носом: корабль влетал бы в плотные слои на первой космической
		// и разбирался бы там от напора в мегапаскаль.
		dragArea := fm.dragArea(acc.Mach, fm.AngleOfAttack)
		dragForce := acc.DynamicQ * dragArea
		acc.DragForce = dragForce
		acc.Drag = vRel.Unit().Scale(-dragForce / mass)
	}

	acc.AngleOfAttack = fm.AngleOfAttack
	acc.Area = fm.effectiveArea(fm.AngleOfAttack)
	acc.Total = acc.Gravity.Add(acc.Thrust).Add(acc.Drag)
	return acc
}

// derivative — производные компонентов состояния по времени.
type derivative struct {
	dPosition physics.Vec3
	dVelocity physics.Vec3
	dFuel     float64
}

func (fm ForceModel) derivativeAt(s VehicleState) derivative {
	acc := fm.Evaluate(s)

	dFuel := 0.0
	if fm.Thrust > 0 && s.FuelMass > 0 {
		dFuel = -fm.MassFlow
	}

	return derivative{
		dPosition: s.Velocity,
		dVelocity: acc.Total,
		dFuel:     dFuel,
	}
}

// advance возвращает состояние, смещённое на dt вдоль производной d.
func advance(s VehicleState, d derivative, dt float64) VehicleState {
	return VehicleState{
		Position: s.Position.Add(d.dPosition.Scale(dt)),
		Velocity: s.Velocity.Add(d.dVelocity.Scale(dt)),
		FuelMass: math.Max(0, s.FuelMass+d.dFuel*dt),
	}
}

// RK4Step делает один шаг интегрирования методом Рунге — Кутты 4-го порядка.
//
// По сравнению с прежним явным Эйлером с шагом 1 с ошибка на активном участке
// падает примерно на четыре порядка, а на орбите перестаёт накапливаться
// систематический дрейф энергии.
func RK4Step(s VehicleState, fm ForceModel, dt float64) VehicleState {
	k1 := fm.derivativeAt(s)
	k2 := fm.derivativeAt(advance(s, k1, dt/2))
	k3 := fm.derivativeAt(advance(s, k2, dt/2))
	k4 := fm.derivativeAt(advance(s, k3, dt))

	weighted := derivative{
		dPosition: k1.dPosition.
			Add(k2.dPosition.Scale(2)).
			Add(k3.dPosition.Scale(2)).
			Add(k4.dPosition).Scale(1.0 / 6.0),
		dVelocity: k1.dVelocity.
			Add(k2.dVelocity.Scale(2)).
			Add(k3.dVelocity.Scale(2)).
			Add(k4.dVelocity).Scale(1.0 / 6.0),
		dFuel: (k1.dFuel + 2*k2.dFuel + 2*k3.dFuel + k4.dFuel) / 6.0,
	}

	return advance(s, weighted, dt)
}

// SpecificEnergy возвращает удельную механическую энергию (Дж/кг).
// Используется в тестах как контроль сохранения энергии на пассивном участке.
func SpecificEnergy(s VehicleState) float64 {
	v := s.Velocity.Norm()
	return v*v/2 - physics.GM/s.Position.Norm()
}

// dragArea возвращает произведение коэффициента сопротивления на площадь, м².
//
// Продольная и поперечная составляющие считаются раздельно: у цилиндра,
// поставленного поперёк потока, коэффициент около полутора против трёх
// десятых у него же носом вперёд.
func (fm ForceModel) dragArea(mach, angleOfAttack float64) float64 {
	const crossflowCd = 1.4

	axial := physics.DragCoefficient(mach)
	if fm.SideArea <= 0 {
		return axial * fm.Area
	}

	a := math.Abs(angleOfAttack) * physics.DegToRad
	return axial*fm.Area*math.Abs(math.Cos(a)) +
		crossflowCd*fm.SideArea*math.Abs(math.Sin(a))
}

// effectiveArea возвращает площадь, подставленную потоку, м².
//
// Цилиндр под углом атаки подставляет потоку проекцию: мидель по косинусу
// угла и борт по синусу. Именно на этом построен вход брюхом — корпус
// разворачивают поперёк, баллистический коэффициент падает в разы,
// торможение начинается высоко и идёт мягче.
func (fm ForceModel) effectiveArea(angleOfAttack float64) float64 {
	side := fm.SideArea
	if side <= 0 {
		return fm.Area
	}

	a := math.Abs(angleOfAttack) * physics.DegToRad
	return fm.Area*math.Abs(math.Cos(a)) + side*math.Abs(math.Sin(a))
}
