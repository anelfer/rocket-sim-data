package physics

import "math"

// -----------------------------------------------------------------------------
// Угловая ориентация корпуса.
//
// Ориентация задаётся тремя углами относительно локального базиса ENU:
//
//	pitch — угол продольной оси над местным горизонтом;
//	yaw   — азимут проекции продольной оси, отсчитываемый от севера к востоку;
//	roll  — поворот корпуса вокруг собственной продольной оси.
//
// Из этих углов строится правая связанная система координат:
//
//	Forward — продольная ось, направление тяги;
//	Right   — правое крыло (правый борт);
//	Up      — «спина» корпуса.
//
// Roll поворачивает Right и Up вокруг Forward и на направление тяги не влияет.
// Физический смысл он приобретает через ролл-программу: носитель разворачивается
// вокруг оси так, чтобы плоскость тангажа совпала с плоскостью выведения.
// -----------------------------------------------------------------------------

// Attitude — угловая ориентация корпуса в градусах.
type Attitude struct {
	Pitch float64 // угол над местным горизонтом, −90…+90
	Yaw   float64 // азимут от севера, 0…360
	Roll  float64 // крен вокруг продольной оси, −180…180
}

// BodyFrame — связанная система координат, выраженная в осях ECI.
//
// Тройка (вперёд, вправо, вниз) — стандартная связанная система: она правая,
// то есть Forward × Right = Down, и потому векторные произведения в расчёте
// моментов дают верные знаки. Третья ось направлена именно вниз, а не вверх:
// «вперёд векторно вправо» — это направление под днище, и называть его «Up»
// значило бы врать о его направлении.
type BodyFrame struct {
	Forward Vec3 // продольная ось (направление тяги)
	Right   Vec3 // правый борт
	Down    Vec3 // к днищу; «вверх» — это Down со знаком минус
}

// BodyFrame строит связанную систему координат по углам ориентации
// и локальному базису.
func (a Attitude) BodyFrame(local LocalFrame) BodyFrame {
	forward := local.Direction(a.Pitch, a.Yaw)

	// Опорное направление «вправо» при нулевом крене: горизонтальный вектор,
	// перпендикулярный проекции продольной оси. Берётся поворотом азимута
	// на 90°, поэтому не вырождается при вертикальном положении корпуса,
	// когда векторное произведение с местной вертикалью обращается в ноль.
	rightRef := local.Direction(0, a.Yaw+90)
	downRef := forward.Cross(rightRef).Unit()

	// Поворот вокруг продольной оси на угол крена.
	rollRad := a.Roll * math.Pi / 180
	cr, sr := math.Cos(rollRad), math.Sin(rollRad)

	right := rightRef.Scale(cr).Add(downRef.Scale(sr)).Unit()
	down := forward.Cross(right).Unit()

	return BodyFrame{Forward: forward, Right: right, Down: down}
}

// AngleOfAttack возвращает полный угол атаки (градусы) — угол между продольной
// осью корпуса и вектором скорости относительно воздуха.
//
// На гравитационном развороте он должен быть близок к нулю: любое заметное
// отклонение создаёт поперечную нагрузку на корпус. Это один из главных
// показателей качества выведения.
func AngleOfAttack(forward, airVelocity Vec3) float64 {
	speed := airVelocity.Norm()
	if speed < 1e-3 {
		return 0
	}
	cosAlpha := clamp(forward.Unit().Dot(airVelocity.Unit()), -1, 1)
	return math.Acos(cosAlpha) * 180 / math.Pi
}

// AeroAngles раскладывает полный угол атаки на две составляющие в связанных
// осях: угол атаки в плоскости тангажа и угол скольжения в плоскости рыскания.
//
// Знаки соответствуют авиационному соглашению: положительный угол атаки —
// поток приходит снизу, положительный угол скольжения — поток приходит справа.
//
// Поток приходит снизу тогда, когда нос поднят над вектором скорости, то есть
// когда у скорости есть составляющая «вниз» в связанных осях. Прежде здесь
// стоял минус перед арктангенсом, и знак угла атаки получался обратным:
// на графике он расходился с полным углом, который неотрицателен по
// определению.
func AeroAngles(body BodyFrame, airVelocity Vec3) (alpha, beta float64) {
	speed := airVelocity.Norm()
	if speed < 1e-3 {
		return 0, 0
	}
	v := airVelocity.Unit()

	forward := v.Dot(body.Forward)
	right := v.Dot(body.Right)
	down := v.Dot(body.Down)

	alpha = math.Atan2(down, forward) * 180 / math.Pi
	beta = math.Asin(clamp(right, -1, 1)) * 180 / math.Pi
	return alpha, beta
}

// -----------------------------------------------------------------------------
// Ограничение угловых скоростей
// -----------------------------------------------------------------------------

// RateLimits — предельные угловые скорости по осям, градусы в секунду.
type RateLimits struct {
	Pitch float64
	Yaw   float64
	Roll  float64
}

// Approach возвращает ориентацию, приближённую к target за время dt
// с учётом предельных угловых скоростей.
//
// Углы рыскания и крена цикличны, поэтому доворот всегда идёт кратчайшим путём:
// с 350° на 10° система поворачивается на 20°, а не на 340°.
func (a Attitude) Approach(target Attitude, limits RateLimits, dt float64) Attitude {
	return Attitude{
		Pitch: approachLinear(a.Pitch, target.Pitch, limits.Pitch*dt),
		Yaw:   NormalizeAzimuth(approachCyclic(a.Yaw, target.Yaw, limits.Yaw*dt)),
		Roll:  normalizeSigned(approachCyclic(a.Roll, target.Roll, limits.Roll*dt)),
	}
}

// AngularRates возвращает угловые скорости (градусы в секунду) перехода
// из предыдущей ориентации в текущую.
func (a Attitude) AngularRates(previous Attitude, dt float64) (pitchRate, yawRate, rollRate float64) {
	if dt <= 0 {
		return 0, 0, 0
	}
	return (a.Pitch - previous.Pitch) / dt,
		shortestAngle(a.Yaw-previous.Yaw) / dt,
		shortestAngle(a.Roll-previous.Roll) / dt
}

func approachLinear(current, target, maxChange float64) float64 {
	diff := target - current
	if math.Abs(diff) <= maxChange {
		return target
	}
	if diff > 0 {
		return current + maxChange
	}
	return current - maxChange
}

func approachCyclic(current, target, maxChange float64) float64 {
	diff := shortestAngle(target - current)
	if math.Abs(diff) <= maxChange {
		return target
	}
	if diff > 0 {
		return current + maxChange
	}
	return current - maxChange
}

// shortestAngle приводит разность углов к диапазону (−180, 180].
func shortestAngle(deg float64) float64 {
	return math.Mod(math.Mod(deg, 360)+540, 360) - 180
}

// ShortestAngle приводит разность углов к диапазону (−180, 180].
func ShortestAngle(deg float64) float64 { return shortestAngle(deg) }

// normalizeSigned приводит угол к диапазону (−180, 180].
func normalizeSigned(deg float64) float64 {
	return shortestAngle(deg)
}
