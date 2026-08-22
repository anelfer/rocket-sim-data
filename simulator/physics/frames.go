package physics

import "math"

// -----------------------------------------------------------------------------
// Системы координат.
//
// Единое соглашение по времени для всего симулятора: t — секунды от начала
// симуляции, и при t = 0 ось X системы ECI совпадает с гринвичским меридианом.
// Угол поворота Земли θ = ω·t используется одинаково во всех преобразованиях
// в обе стороны. Раньше прямое и обратное преобразования брали разные отсчёты
// времени, из-за чего в момент выхода на орбиту координаты скакали на ω·t₀.
// -----------------------------------------------------------------------------

// Geodetic — географические координаты (сферическая модель Земли).
type Geodetic struct {
	Latitude  float64 // градусы, положительная — север
	Longitude float64 // градусы, положительная — восток
	Altitude  float64 // метры над сферой радиуса EarthRadius
}

// EarthRotationAngle возвращает угол поворота Земли (радианы) к моменту t.
func EarthRotationAngle(t float64) float64 {
	return EarthAngularVelocity * t
}

// ECIToECEF переводит вектор из инерциальной системы во вращающуюся.
func ECIToECEF(v Vec3, t float64) Vec3 {
	theta := EarthRotationAngle(t)
	c, s := math.Cos(theta), math.Sin(theta)
	return Vec3{
		X: c*v.X + s*v.Y,
		Y: -s*v.X + c*v.Y,
		Z: v.Z,
	}
}

// ECEFToECI переводит вектор из вращающейся системы в инерциальную.
// Строго обратно ECIToECEF при том же t.
func ECEFToECI(v Vec3, t float64) Vec3 {
	theta := EarthRotationAngle(t)
	c, s := math.Cos(theta), math.Sin(theta)
	return Vec3{
		X: c*v.X - s*v.Y,
		Y: s*v.X + c*v.Y,
		Z: v.Z,
	}
}

// ECEFToGeodetic переводит декартовы координаты во вращающейся системе
// в широту/долготу/высоту (сферическая модель).
func ECEFToGeodetic(r Vec3) Geodetic {
	rMag := r.Norm()
	if rMag < 1e-6 {
		return Geodetic{}
	}
	return Geodetic{
		Latitude:  math.Asin(clamp(r.Z/rMag, -1, 1)) * 180 / math.Pi,
		Longitude: normalizeLongitude(math.Atan2(r.Y, r.X) * 180 / math.Pi),
		Altitude:  rMag - EarthRadius,
	}
}

// GeodeticToECEF — обратное преобразование.
func GeodeticToECEF(g Geodetic) Vec3 {
	latRad := g.Latitude * math.Pi / 180
	lonRad := g.Longitude * math.Pi / 180
	r := EarthRadius + g.Altitude
	return Vec3{
		X: r * math.Cos(latRad) * math.Cos(lonRad),
		Y: r * math.Cos(latRad) * math.Sin(lonRad),
		Z: r * math.Sin(latRad),
	}
}

// ECIToGeodetic — сквозное преобразование ECI → широта/долгота/высота.
func ECIToGeodetic(r Vec3, t float64) Geodetic {
	return ECEFToGeodetic(ECIToECEF(r, t))
}

// LocalFrame — локальный топоцентрический базис ENU (восток, север, вверх),
// выраженный в осях ECI.
type LocalFrame struct {
	East  Vec3
	North Vec3
	Up    Vec3
}

// NewLocalFrame строит базис ENU в точке r (ECI).
//
// Ось вращения Земли совпадает с осью Z и в ECI, и в ECEF, поэтому базис
// строится напрямую из вектора положения — переводить в широту/долготу
// и обратно не требуется.
func NewLocalFrame(r Vec3) LocalFrame {
	up := r.Unit()
	east := Vec3{0, 0, 1}.Cross(up)
	if east.Norm() < 1e-9 {
		// Полюс: восточное направление вырождается, берём произвольное
		// перпендикулярное оси.
		east = Vec3{1, 0, 0}
	}
	east = east.Unit()
	north := up.Cross(east).Unit()
	return LocalFrame{East: east, North: north, Up: up}
}

// Direction возвращает единичный вектор в ECI по углу места (pitch, градусы
// над местным горизонтом) и азимуту (градусы от направления на север,
// по часовой стрелке: 0° — север, 90° — восток).
func (f LocalFrame) Direction(pitchDeg, azimuthDeg float64) Vec3 {
	p := pitchDeg * math.Pi / 180
	a := azimuthDeg * math.Pi / 180
	cp := math.Cos(p)
	return f.North.Scale(cp * math.Cos(a)).
		Add(f.East.Scale(cp * math.Sin(a))).
		Add(f.Up.Scale(math.Sin(p)))
}

// Decompose раскладывает вектор на компоненты восток/север/вверх.
func (f LocalFrame) Decompose(v Vec3) (east, north, up float64) {
	return v.Dot(f.East), v.Dot(f.North), v.Dot(f.Up)
}

// PitchAzimuth возвращает угол места (градусы над горизонтом) и азимут
// (градусы от севера) для вектора v.
func (f LocalFrame) PitchAzimuth(v Vec3) (pitchDeg, azimuthDeg float64) {
	e, n, u := f.Decompose(v)
	horizontal := math.Hypot(e, n)
	pitchDeg = math.Atan2(u, horizontal) * 180 / math.Pi
	azimuthDeg = normalizeAzimuth(math.Atan2(e, n) * 180 / math.Pi)
	return
}

// -----------------------------------------------------------------------------
// Вспомогательные функции
// -----------------------------------------------------------------------------

func clamp(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}

// Clamp зажимает значение в диапазон [lo, hi].
func Clamp(x, lo, hi float64) float64 { return clamp(x, lo, hi) }

// normalizeLongitude приводит долготу к диапазону (−180, 180].
func normalizeLongitude(deg float64) float64 {
	for deg > 180 {
		deg -= 360
	}
	for deg <= -180 {
		deg += 360
	}
	return deg
}

// NormalizeLongitude приводит долготу к диапазону (−180, 180].
func NormalizeLongitude(deg float64) float64 { return normalizeLongitude(deg) }

// normalizeAzimuth приводит азимут к диапазону [0, 360).
func normalizeAzimuth(deg float64) float64 {
	deg = math.Mod(deg, 360)
	if deg < 0 {
		deg += 360
	}
	return deg
}

// NormalizeAzimuth приводит азимут к диапазону [0, 360).
func NormalizeAzimuth(deg float64) float64 { return normalizeAzimuth(deg) }

// RelaxTo возвращает значение, приближённое к target за время dt с постоянной
// времени tau по точному решению линейного уравнения релаксации.
//
// Устойчиво при любом шаге, в отличие от явного Эйлера: экспонента никогда
// не выходит за пределы отрезка между текущим значением и целью, тогда как
// явная схема при dt больше постоянной времени даёт перелёт и раскачку.
func RelaxTo(current, target, dt, tau float64) float64 {
	if tau <= 0 {
		return target
	}
	decay := math.Exp(-dt / tau)
	return target + (current-target)*decay
}
