package physics

import "math"

// GravityAcceleration возвращает вектор ускорения свободного падения в точке r
// (ECI, метры) с учётом сплюснутости Земли (гармоника J2).
//
// Ньютоновский член:  -GM/|r|³ · r
// Член J2 отвечает за прецессию восходящего узла и аргумента перигея —
// для LEO это примерно −5°/сутки при наклонении 51.6°, величина заметная.
func GravityAcceleration(r Vec3) Vec3 {
	rMag := r.Norm()
	if rMag < 1e-6 {
		return Vec3{}
	}

	// Ньютоновская часть.
	k := -GM / (rMag * rMag * rMag)
	acc := r.Scale(k)

	// Возмущение J2.
	re := EarthEquatorialRadius
	zr := r.Z / rMag
	factor := 1.5 * J2 * GM * re * re / (rMag * rMag * rMag * rMag * rMag)

	acc.X += -factor * r.X * (1 - 5*zr*zr)
	acc.Y += -factor * r.Y * (1 - 5*zr*zr)
	acc.Z += -factor * r.Z * (3 - 5*zr*zr)

	return acc
}

// GravityMagnitudeAtAltitude возвращает скалярную величину ускорения свободного
// падения на заданной высоте по сферической модели.
//
// Это справочная величина (для телеметрии и расчёта TWR). В уравнения движения
// она не подставляется — там работает векторная GravityAcceleration.
//
// Важно: это ИМЕННО гравитация. Центростремительное ускорение сюда не входит:
// в инерциальной системе координат центробежной силы не существует, она
// появляется автоматически из интегрирования движения.
func GravityMagnitudeAtAltitude(altitude float64) float64 {
	r := EarthRadius + altitude
	return GM / (r * r)
}

// CorotatingVelocity возвращает скорость точки, жёстко связанной с вращающейся
// Землёй, в инерциальной системе координат: v = ω × r.
//
// Используется дважды:
//   - для задания начальной скорости ракеты на стартовом столе;
//   - для вычисления скорости относительно атмосферы (атмосфера вращается
//     вместе с Землёй), от которой зависит сопротивление и нагрев.
func CorotatingVelocity(r Vec3) Vec3 {
	return EarthRotationVector.Cross(r)
}

// SpeedOfSound возвращает скорость звука в м/с при температуре в КЕЛЬВИНАХ.
//
// Функция намеренно принимает только Кельвины: передача сюда градусов Цельсия
// (как это было раньше) даёт sqrt(отрицательного) = NaN, после чего число Маха
// становится NaN и все сравнения в таблице Cd молча проваливаются в default.
func SpeedOfSound(tempK float64) float64 {
	if tempK <= 0 {
		return 0
	}
	return math.Sqrt(GammaAir * RSpecificAir * tempK)
}
