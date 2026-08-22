package physics

import "math"

// -----------------------------------------------------------------------------
// Динамика вращения твёрдого тела.
//
// Неуправляемая ступень после разделения кувыркается: аэродинамический момент
// относительно центра масс не скомпенсирован, а системы стабилизации нет.
// Здесь интегрируются уравнения Эйлера для вращения вокруг центра масс.
// -----------------------------------------------------------------------------

// InertiaTensor — главные моменты инерции в связанных осях, кг·м².
//
// Для тела вращения оси совпадают с главными: X — продольная,
// Y и Z — поперечные и равны между собой.
type InertiaTensor struct {
	Ixx float64 // относительно продольной оси
	Iyy float64 // относительно поперечной
	Izz float64 // относительно поперечной
}

// CylinderInertia возвращает тензор инерции однородного тонкостенного цилиндра.
//
//	I_прод = m·r²
//	I_попер = m·(6r² + L²)/12
//
// Тонкостенная модель ближе к реальной ступени, чем сплошной цилиндр:
// масса сосредоточена в обечайке баков, а не распределена по объёму.
func CylinderInertia(mass, radius, length float64) InertiaTensor {
	longitudinal := mass * radius * radius
	transverse := mass * (6*radius*radius + length*length) / 12
	return InertiaTensor{
		Ixx: longitudinal,
		Iyy: transverse,
		Izz: transverse,
	}
}

// AngularAcceleration решает уравнения Эйлера относительно углового ускорения.
//
//	I·dω/dt + ω × (I·ω) = M
//
// Член ω × (I·ω) — гироскопический. Именно он отвечает за то, что свободное
// вращение вытянутого тела вокруг поперечной оси устойчиво, а вокруг оси
// с промежуточным моментом инерции — нет.
func (i InertiaTensor) AngularAcceleration(omega, torque Vec3) Vec3 {
	if i.Ixx <= 0 || i.Iyy <= 0 || i.Izz <= 0 {
		return Vec3{}
	}

	momentum := Vec3{X: i.Ixx * omega.X, Y: i.Iyy * omega.Y, Z: i.Izz * omega.Z}
	gyroscopic := omega.Cross(momentum)

	return Vec3{
		X: (torque.X - gyroscopic.X) / i.Ixx,
		Y: (torque.Y - gyroscopic.Y) / i.Iyy,
		Z: (torque.Z - gyroscopic.Z) / i.Izz,
	}
}

// AngularMomentum возвращает вектор момента импульса в связанных осях.
func (i InertiaTensor) AngularMomentum(omega Vec3) Vec3 {
	return Vec3{X: i.Ixx * omega.X, Y: i.Iyy * omega.Y, Z: i.Izz * omega.Z}
}

// RotationalEnergy возвращает кинетическую энергию вращения, Дж.
func (i InertiaTensor) RotationalEnergy(omega Vec3) float64 {
	return 0.5 * (i.Ixx*omega.X*omega.X +
		i.Iyy*omega.Y*omega.Y +
		i.Izz*omega.Z*omega.Z)
}

// -----------------------------------------------------------------------------
// Аэродинамика тела вращения при произвольном угле атаки
// -----------------------------------------------------------------------------

// AeroShape — геометрия цилиндрического корпуса для расчёта сил и моментов.
type AeroShape struct {
	Length float64 // длина корпуса, м
	Radius float64 // радиус корпуса, м

	// CenterOfMass — положение центра масс от носа, доля длины (0…1).
	CenterOfMass float64

	// CenterOfPressureAxial — положение центра давления при осевом обтекании,
	// доля длины от носа.
	CenterOfPressureAxial float64

	// CenterOfPressureNormal — положение центра давления при поперечном
	// обтекании, доля длины. Для цилиндра близко к геометрической середине.
	CenterOfPressureNormal float64
}

// DefaultStageShape возвращает геометрию ступени носителя.
//
// Пустая ступень имеет центр масс ближе к двигательному отсеку (там
// сосредоточена масса девяти двигателей), а центр давления при поперечном
// обтекании — примерно посередине. Такое взаимное расположение делает полёт
// двигателями вперёд статически устойчивым, а носом вперёд — неустойчивым.
func DefaultStageShape(length, radius float64) AeroShape {
	return AeroShape{
		Length:                 length,
		Radius:                 radius,
		CenterOfMass:           0.65,
		CenterOfPressureAxial:  0.30,
		CenterOfPressureNormal: 0.50,
	}
}

// ReferenceArea возвращает площадь миделя, м².
func (s AeroShape) ReferenceArea() float64 {
	return math.Pi * s.Radius * s.Radius
}

// SideArea возвращает площадь боковой проекции, м².
func (s AeroShape) SideArea() float64 {
	return 2 * s.Radius * s.Length
}

// EffectiveArea возвращает эффективную площадь сопротивления при угле alpha
// между продольной осью и потоком (радианы).
//
// Проекция цилиндра на плоскость, перпендикулярную потоку:
//
//	A(α) = A_мидель·|cos α| + A_бок·|sin α|
//
// Для ступени с удлинением 12 боковая проекция вчетверо больше миделя, поэтому
// кувыркающееся тело тормозится существенно сильнее ориентированного по потоку.
// Раньше это учитывалось единым множителем 2.2, теперь получается из геометрии.
func (s AeroShape) EffectiveArea(alpha float64) float64 {
	return s.ReferenceArea()*math.Abs(math.Cos(alpha)) +
		s.SideArea()*math.Abs(math.Sin(alpha))
}

// DragCoefficientAtAngle возвращает коэффициент сопротивления с учётом угла
// атаки и числа Маха.
func (s AeroShape) DragCoefficientAtAngle(mach, alpha float64) float64 {
	axial := DragCoefficient(mach)
	// Поперечное обтекание цилиндра: коэффициент около 1.2 на дозвуке
	// и растёт до 1.7 на сверхзвуке.
	crossflow := 1.2
	if mach > 1 {
		crossflow = 1.2 + 0.5*math.Min((mach-1)/2, 1)
	}
	sa := math.Abs(math.Sin(alpha))
	return axial*(1-sa) + crossflow*sa
}

// AerodynamicTorque возвращает аэродинамический момент в связанных осях, Н·м.
//
// Момент возникает из-за того, что равнодействующая аэродинамических сил
// приложена в центре давления, а вращение происходит вокруг центра масс.
// Плечо равно расстоянию между ними.
//
//	M = r_цд→цм × F_аэро
//
// Дополнительно учитывается демпфирующий момент от вращения: при повороте
// корпуса набегающий поток создаёт сопротивление вращению, пропорциональное
// угловой скорости. Без него кувыркание никогда бы не затухало.
func (s AeroShape) AerodynamicTorque(
	dynamicPressure float64,
	airflowBody Vec3,
	omega Vec3,
	mach float64,
) Vec3 {
	speed := airflowBody.Norm()
	if speed < 1e-6 || dynamicPressure <= 0 {
		return Vec3{}
	}

	flow := airflowBody.Unit()

	// Угол между продольной осью и потоком.
	alpha := math.Acos(clamp(flow.X, -1, 1))

	// Поперечная составляющая потока в связанных осях.
	normal := Vec3{X: 0, Y: flow.Y, Z: flow.Z}
	normalMag := normal.Norm()

	var torque Vec3

	if normalMag > 1e-9 {
		// Нормальная сила пропорциональна квадрату синуса угла атаки:
		// так ведёт себя поперечное обтекание цилиндра.
		cn := 1.2 * math.Sin(alpha) * math.Abs(math.Sin(alpha))
		normalForce := dynamicPressure * s.SideArea() * cn

		// Плечо: расстояние от центра масс до центра давления вдоль корпуса.
		// Положительное значение означает, что центр давления впереди центра
		// масс — конфигурация статически неустойчивая.
		arm := (s.CenterOfMass - s.CenterOfPressureNormal) * s.Length

		// Направление нормальной силы.
		//
		// Аргумент airflowBody — это скорость корпуса относительно воздуха,
		// а не скорость набегающего потока: воздух движется мимо корпуса
		// в противоположную сторону. Поэтому нормальная сила направлена
		// ПРОТИВ поперечной составляющей скорости, как и сила сопротивления
		// направлена против скорости целиком.
		//
		// Со знаком «по потоку» получалось прямо обратное: корпус с центром
		// давления впереди центра масс возвращался к потоку вместо того,
		// чтобы уводиться от него. Ракета без оперения оказывалась устойчивой,
		// отказ управления ничем не грозил, а отработавшая ступень должна была
		// бы лететь носом вперёд вместо разворота двигателями назад.
		forceVec := normal.Unit().Scale(-normalForce)
		leverVec := Vec3{X: arm}
		torque = leverVec.Cross(forceVec)
	}

	// Демпфирование вращения. Коэффициент оценивается из интегрирования
	// местной скорости по длине корпуса: элемент на расстоянии x от центра масс
	// движется со скоростью ω·x и создаёт момент, пропорциональный ω·x².
	dampingCoeff := 0.5 * AtmosphericDensityFromQ(dynamicPressure, speed) *
		speed * s.SideArea() * s.Length * s.Length / 12

	torque = torque.Sub(Vec3{
		// Вокруг продольной оси демпфирование слабое: обтекается только
		// боковая поверхность, плечо равно радиусу.
		X: dampingCoeff * omega.X * (s.Radius / s.Length) * 0.1,
		Y: dampingCoeff * omega.Y,
		Z: dampingCoeff * omega.Z,
	})

	return torque
}

// AtmosphericDensityFromQ восстанавливает плотность из скоростного напора.
// Нужна, чтобы не передавать плотность отдельным параметром там, где уже
// известен напор.
func AtmosphericDensityFromQ(dynamicPressure, speed float64) float64 {
	if speed < 1e-6 {
		return 0
	}
	return 2 * dynamicPressure / (speed * speed)
}
