package physics

import "math"

// -----------------------------------------------------------------------------
// Кватернионы для описания ориентации твёрдого тела.
//
// Углы Эйлера для свободно кувыркающегося тела не годятся: при тангаже около
// ±90° две оси вырождаются в одну (шарнирный замок), и производные уходят
// в бесконечность. У неуправляемой ступени, которая беспорядочно вращается,
// такие положения проходятся постоянно, поэтому ориентация хранится
// кватернионом, а углы вычисляются только для вывода в телеметрию.
// -----------------------------------------------------------------------------

// Quaternion — кватернион поворота из связанных осей в инерциальные.
type Quaternion struct {
	W, X, Y, Z float64
}

// IdentityQuaternion возвращает единичный кватернион (нулевой поворот).
func IdentityQuaternion() Quaternion {
	return Quaternion{W: 1}
}

// QuaternionFromAxisAngle строит кватернион поворота вокруг оси на угол
// (радианы).
func QuaternionFromAxisAngle(axis Vec3, angle float64) Quaternion {
	a := axis.Unit()
	half := angle / 2
	s := math.Sin(half)
	return Quaternion{
		W: math.Cos(half),
		X: a.X * s,
		Y: a.Y * s,
		Z: a.Z * s,
	}
}

// QuaternionFromBasis строит кватернион по ортонормированному базису
// связанных осей, заданному в инерциальной системе.
//
// Используется алгоритм Шеперда: выбирается наибольшая по модулю компонента,
// что исключает деление на малое число.
func QuaternionFromBasis(forward, right, down Vec3) Quaternion {
	// Матрица поворота по столбцам: связанные оси в инерциальных координатах.
	m00, m01, m02 := forward.X, right.X, down.X
	m10, m11, m12 := forward.Y, right.Y, down.Y
	m20, m21, m22 := forward.Z, right.Z, down.Z

	trace := m00 + m11 + m22

	var q Quaternion
	switch {
	case trace > 0:
		s := math.Sqrt(trace+1) * 2
		q = Quaternion{
			W: 0.25 * s,
			X: (m21 - m12) / s,
			Y: (m02 - m20) / s,
			Z: (m10 - m01) / s,
		}
	case m00 > m11 && m00 > m22:
		s := math.Sqrt(1+m00-m11-m22) * 2
		q = Quaternion{
			W: (m21 - m12) / s,
			X: 0.25 * s,
			Y: (m01 + m10) / s,
			Z: (m02 + m20) / s,
		}
	case m11 > m22:
		s := math.Sqrt(1+m11-m00-m22) * 2
		q = Quaternion{
			W: (m02 - m20) / s,
			X: (m01 + m10) / s,
			Y: 0.25 * s,
			Z: (m12 + m21) / s,
		}
	default:
		s := math.Sqrt(1+m22-m00-m11) * 2
		q = Quaternion{
			W: (m10 - m01) / s,
			X: (m02 + m20) / s,
			Y: (m12 + m21) / s,
			Z: 0.25 * s,
		}
	}
	return q.Normalized()
}

// Norm возвращает модуль кватерниона.
func (q Quaternion) Norm() float64 {
	return math.Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)
}

// Normalized возвращает единичный кватернион.
//
// Нормировка после каждого шага интегрирования обязательна: численная ошибка
// накапливается, и без неё кватернион перестаёт описывать поворот.
func (q Quaternion) Normalized() Quaternion {
	n := q.Norm()
	if n < 1e-12 {
		return IdentityQuaternion()
	}
	return Quaternion{W: q.W / n, X: q.X / n, Y: q.Y / n, Z: q.Z / n}
}

// Conjugate возвращает сопряжённый кватернион (обратный поворот).
func (q Quaternion) Conjugate() Quaternion {
	return Quaternion{W: q.W, X: -q.X, Y: -q.Y, Z: -q.Z}
}

// Multiply возвращает композицию поворотов q ∘ r.
func (q Quaternion) Multiply(r Quaternion) Quaternion {
	return Quaternion{
		W: q.W*r.W - q.X*r.X - q.Y*r.Y - q.Z*r.Z,
		X: q.W*r.X + q.X*r.W + q.Y*r.Z - q.Z*r.Y,
		Y: q.W*r.Y - q.X*r.Z + q.Y*r.W + q.Z*r.X,
		Z: q.W*r.Z + q.X*r.Y - q.Y*r.X + q.Z*r.W,
	}
}

// Rotate поворачивает вектор из связанных осей в инерциальные.
func (q Quaternion) Rotate(v Vec3) Vec3 {
	// Формула Родрига в кватернионной записи: v' = v + 2·w·(u×v) + 2·u×(u×v),
	// где u — векторная часть. Быстрее полного умножения кватернионов.
	u := Vec3{X: q.X, Y: q.Y, Z: q.Z}
	t := u.Cross(v).Scale(2)
	return v.Add(t.Scale(q.W)).Add(u.Cross(t))
}

// RotateInverse поворачивает вектор из инерциальных осей в связанные.
func (q Quaternion) RotateInverse(v Vec3) Vec3 {
	return q.Conjugate().Rotate(v)
}

// Derivative возвращает производную кватерниона при угловой скорости omega,
// заданной в СВЯЗАННЫХ осях (рад/с).
//
//	dq/dt = ½ · q ⊗ (0, ω)
func (q Quaternion) Derivative(omega Vec3) Quaternion {
	w := Quaternion{W: 0, X: omega.X, Y: omega.Y, Z: omega.Z}
	p := q.Multiply(w)
	return Quaternion{W: 0.5 * p.W, X: 0.5 * p.X, Y: 0.5 * p.Y, Z: 0.5 * p.Z}
}

// Add складывает кватернионы покомпонентно (для шагов интегрирования).
func (q Quaternion) Add(r Quaternion) Quaternion {
	return Quaternion{W: q.W + r.W, X: q.X + r.X, Y: q.Y + r.Y, Z: q.Z + r.Z}
}

// Scale умножает кватернион на скаляр.
func (q Quaternion) Scale(k float64) Quaternion {
	return Quaternion{W: q.W * k, X: q.X * k, Y: q.Y * k, Z: q.Z * k}
}

// Body возвращает связанный базис в инерциальных осях.
func (q Quaternion) Body() BodyFrame {
	return BodyFrame{
		Forward: q.Rotate(Vec3{X: 1}),
		Right:   q.Rotate(Vec3{Y: 1}),
		Down:    q.Rotate(Vec3{Z: 1}),
	}
}

// AimQuaternion строит ориентацию-цель: КРАТЧАЙШИЙ поворот от текущей
// ориентации, приводящий продольную ось к направлению want.
//
// Зачем это нужно вместо привычной пары «тангаж + азимут». Наведение всегда
// знает потребное НАПРАВЛЕНИЕ продольной оси — вектор. Чтобы передать его
// автопилоту через Attitude, вектор раскладывают на тангаж и азимут
// (LocalFrame.PitchAzimuth), а автопилот собирает из них базис обратно
// (Attitude.BodyFrame). Продольная ось этот круг переживает без потерь, а
// вот опорное направление крена — нет: BodyFrame строит его как
// local.Direction(0, yaw+90), то есть ИЗ АЗИМУТА. У вертикально стоящего
// корпуса азимут продольной оси не определён, и возмущение в доли
// сантиметра в секунду разворачивает его на сто восемьдесят градусов —
// вместе с опорой крена и, значит, с командой по крену.
//
// Бустер стоит почти вертикально весь возврат (тангаж 85-90°), то есть
// живёт ровно в этой особой точке. Кратчайший поворот особых точек не
// имеет: он определён через векторное произведение текущего направления с
// потребным и обращается в тождественный поворот ровно тогда, когда они
// совпадают. Крен при такой цели не назначается вовсе — ошибка по крену
// выходит нулевой по построению, и канал крена остаётся демпфированием
// угловой скорости. Для бустера это и есть верное поведение: миссии крен
// возврата безразличен, а погоня за абсолютным нулём крена разгоняла ωroll
// до −64°/с. Наведение бустера, впрочем, пользуется не этой функцией, а
// AimBasis: крен ему всё же нужен управляемым (решётчатые рули стоят по
// кольцу на фиксированных азимутах), просто с непрерывной опорой.
func AimQuaternion(current Quaternion, want Vec3) Quaternion {
	forward := current.Rotate(Vec3{X: 1})
	w := want.Unit()

	cos := clamp(forward.Dot(w), -1, 1)
	axis := forward.Cross(w)

	if n := axis.Norm(); n > 1e-12 {
		// Поворот задан в ИНЕРЦИАЛЬНЫХ осях, поэтому умножается слева:
		// кватернион здесь переводит связанные оси в инерциальные.
		return QuaternionFromAxisAngle(axis.Scale(1/n), math.Acos(cos)).
			Multiply(current).Normalized()
	}

	if cos > 0 {
		// Уже смотрим туда — доворачивать нечего.
		return current
	}

	// Ровно назад: ось поворота вырождена, годится любая перпендикулярная
	// продольной. Берём связанную «вправо» — она перпендикулярна по
	// построению.
	return QuaternionFromAxisAngle(current.Rotate(Vec3{Y: 1}), math.Pi).
		Multiply(current).Normalized()
}

// AimBasis строит ориентацию-цель по потребному направлению продольной оси
// и ОПОРЕ КРЕНА, перенося опору параллельно вдоль изменения этой оси.
// Возвращает цель и перенесённую опору, которую вызывающая сторона обязана
// сохранить до следующего такта.
//
// Чем это отличается от AimQuaternion. Кратчайший поворот особых точек не
// имеет, но и крена не назначает вовсе: ошибка по крену выходит нулевой по
// построению, и канал крена вырождается в демпфирование угловой скорости.
// Для корпуса, у которого органы управления расставлены по кольцу на
// фиксированных азимутах — решётчатые рули бустера, — это не безразлично:
// достижимый момент по тангажу и рысканию зависит от того, каким боком
// корпус повёрнут к потоку, и свободный дрейф крена этот момент вращает
// вместе с корпусом.
//
// Опора переносится, а не берётся заново из локального базиса: взятая
// заново (через азимут, как это делает Attitude.BodyFrame) она вырождается
// у вертикально стоящего корпуса — см. AimQuaternion. Перенос особых точек
// не имеет: опора лишь проецируется на плоскость, перпендикулярную новой
// продольной оси, и нормируется.
func AimBasis(want, rightRef Vec3) (Quaternion, Vec3) {
	f := want.Unit()

	r := rightRef.Sub(f.Scale(f.Dot(rightRef)))
	if r.Norm() < 1e-9 {
		// Опора совпала с продольной осью (или ещё не задана) — годится
		// любая перпендикулярная ей.
		r = f.Cross(Vec3{X: 1})
		if r.Norm() < 1e-9 {
			r = f.Cross(Vec3{Y: 1})
		}
	}
	r = r.Unit()

	return QuaternionFromBasis(f, r, f.Cross(r)).Normalized(), r
}

// AttitudeIn возвращает углы ориентации относительно локального базиса.
func (q Quaternion) AttitudeIn(local LocalFrame) Attitude {
	body := q.Body()

	pitch, yaw := local.PitchAzimuth(body.Forward)

	// Крен: угол между связанной осью «вправо» и горизонтальным направлением,
	// перпендикулярным проекции продольной оси.
	rightRef := local.Direction(0, yaw+90)
	downRef := body.Forward.Cross(rightRef).Unit()

	roll := math.Atan2(body.Right.Dot(downRef), body.Right.Dot(rightRef)) * 180 / math.Pi

	return Attitude{Pitch: pitch, Yaw: yaw, Roll: normalizeSigned(roll)}
}
