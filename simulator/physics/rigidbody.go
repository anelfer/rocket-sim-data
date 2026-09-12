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

// CompositeInertia возвращает тензор инерции ступени как трёх тел,
// приведённых к общему центру масс теоремой Гюйгенса-Штейнера:
//
//   - собственная конструкция ступени с её топливом (ownMass) — тонкостенный
//     цилиндр по длине ступени, центр на геометрической середине;
//   - двигатели (engineMass) — точечная масса у среза сопел (x=length);
//   - всё, что ступень везёт сверх этого (carriedMass) — следующая ступень,
//     обтекатель, полезная нагрузка — точечная масса у носа (x=0).
//
// Размазывать двигатели и несомую массу по длине ступени физически неверно:
// это тяжёлые массы, сосредоточенные на концах, а не распределённые вдоль
// корпуса — у связки ступеней разное распределение массы в зависимости от
// того, пристыкована ли следующая ступень, а вклад далёкой массы в момент
// инерции определяется не тем, что она где-то есть, а плечом до общего
// центра масс (∝ m·d²).
//
// centerOfMass — общий центр масс системы, доля длины от носа (0…1).
func CompositeInertia(ownMass, engineMass, carriedMass, radius, length, centerOfMass float64) InertiaTensor {
	own := CylinderInertia(ownMass, radius, length)

	xcm := centerOfMass * length
	d1 := xcm - length/2 // плечо собственного тела до общего ЦМ
	d2 := xcm - length   // плечо двигателей (хвост, x=length) до общего ЦМ
	d3 := xcm            // плечо несомой массы (нос, x=0) до общего ЦМ

	shift := ownMass*d1*d1 + engineMass*d2*d2 + carriedMass*d3*d3
	return InertiaTensor{
		Ixx: own.Ixx, // точечные массы на оси не дают вклада в крен
		Iyy: own.Iyy + shift,
		Izz: own.Izz + shift,
	}
}

// StageInertia возвращает тензор инерции ступени относительно её центра масс,
// считая топливо ОТДЕЛЬНЫМ телом в своём собственном положении.
//
// Отличие от CompositeInertia в том, где находится топливо. Там оно входило в
// массу собственной конструкции и потому размазывалось по всей длине ступени
// вместе с ней; здесь у него своё положение (propX) и своя длина (propLength —
// высота залитого столба), приходящие из геометрии баков и уровня их
// заполнения. Для почти пустой ступени разница мала, для полной — велика:
// топливо составляет три четверти её массы и сидит не там, где корпус.
//
// Тела и их положения (x отсчитывается от носа):
//
//   - конструкция без камер — тонкостенный цилиндр по длине, центр на L/2;
//   - камеры — сосредоточенная масса у среза сопел, x = L;
//   - топливо — сплошной цилиндр высотой propLength с центром в propX;
//   - несомая масса (следующая ступень, нагрузка) — у носа, x = 0.
//
// Продольная (крен) составляющая складывается из собственных вкладов тел:
// точечные массы на оси в неё не дают ничего, а топливо даёт — оно занимает
// весь диаметр бака, и при вращении вокруг продольной оси его приходится
// раскручивать.
func StageInertia(structMass, engineMass, propMass, carriedMass,
	propX, propLength, radius, length, centerOfMass float64) InertiaTensor {

	structure := CylinderInertia(structMass, radius, length)
	propellant := CylinderInertia(propMass, radius, propLength)

	d := func(x float64) float64 {
		v := x - centerOfMass
		return v * v
	}
	shift := structMass*d(length/2) + engineMass*d(length) +
		propMass*d(propX) + carriedMass*d(0)

	return InertiaTensor{
		Ixx: structure.Ixx + propellant.Ixx,
		Iyy: structure.Iyy + propellant.Iyy + shift,
		Izz: structure.Izz + propellant.Izz + shift,
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
}

// DefaultStageShape возвращает геометрию ступени носителя.
//
// Пустая ступень имеет центр масс ближе к двигательному отсеку (там
// сосредоточена масса девяти двигателей). Центр давления при поперечном
// обтекании — не свойство формы, а зависит от числа Маха, см.
// CenterOfPressureNormal.
func DefaultStageShape(length, radius float64) AeroShape {
	return AeroShape{
		Length:       length,
		Radius:       radius,
		CenterOfMass: 0.65,
	}
}

// CenterOfPressureNormal возвращает положение центра давления при
// поперечном обтекании, доля длины корпуса от носа, в зависимости от числа
// Маха.
//
// У тела вращения без оперения ЦД на дозвуке стоит у геометрической
// середины боковой проекции, к трансзвуку (M≈1–1.6) резко уходит назад —
// это и есть классическая просадка запаса статической устойчивости на
// участке Max-Q, — а на сверхзвуке и гиперзвуке снова смещается вперёд
// и стабилизируется.
func CenterOfPressureNormal(mach float64) float64 {
	if math.IsNaN(mach) || mach < 0 {
		return 0.46
	}
	switch {
	case mach < 0.8:
		return 0.46
	case mach < 1.2:
		t := (mach - 0.8) / 0.4
		return 0.46 + t*(0.50-0.46)
	case mach < 1.6:
		t := (mach - 1.2) / 0.4
		return 0.50 - t*(0.50-0.47)
	case mach < 4.0:
		t := (mach - 1.6) / 2.4
		return 0.47 - t*(0.47-0.44)
	default:
		return 0.44
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
		cn := CrossflowDragCoefficient * math.Sin(alpha) * math.Abs(math.Sin(alpha))
		normalForce := dynamicPressure * s.SideArea() * cn

		// Плечо: расстояние от центра масс до центра давления вдоль корпуса.
		// Положительное значение означает, что центр давления впереди центра
		// масс — конфигурация статически неустойчивая.
		arm := (s.CenterOfMass - CenterOfPressureNormal(mach)) * s.Length

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
