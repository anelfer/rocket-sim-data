package physics

import "math"

// -----------------------------------------------------------------------------
// Кеплеровы элементы орбиты.
//
// Элементы вычисляются аналитически из вектора состояния на каждом шаге.
// Никаких «сглаживаний», «обновлений только в апсидах» и обвязки из проверок,
// возвращающих текущую высоту при неудобном результате: если орбита
// гиперболическая или суборбитальная, об этом сообщается флагами, а не
// подменой значения.
// -----------------------------------------------------------------------------

// OrbitalElements — полный набор кеплеровых элементов плюс производные величины.
type OrbitalElements struct {
	SemiMajorAxis       float64 // большая полуось, м (отрицательна для гиперболы)
	Eccentricity        float64 // эксцентриситет
	Inclination         float64 // наклонение, радианы
	RAAN                float64 // долгота восходящего узла, радианы
	ArgumentOfPeriapsis float64 // аргумент перицентра, радианы
	TrueAnomaly         float64 // истинная аномалия, радианы
	Epoch               float64 // время, к которому относятся элементы, с

	// Производные величины (высоты над поверхностью, м).
	ApoapsisAltitude  float64
	PeriapsisAltitude float64

	// Elliptical сообщает, что орбита замкнута (0 ≤ e < 1 и a > 0).
	// Для гиперболической и параболической траектории апоцентра не существует,
	// и ApoapsisAltitude равен +Inf.
	Elliptical bool
}

// Period возвращает период обращения (с). Для незамкнутой орбиты — 0.
func (e OrbitalElements) Period() float64 {
	if !e.Elliptical {
		return 0
	}
	a := e.SemiMajorAxis
	return 2 * math.Pi * math.Sqrt(a*a*a/GM)
}

// InclinationDegrees возвращает наклонение в градусах.
func (e OrbitalElements) InclinationDegrees() float64 {
	return e.Inclination * 180 / math.Pi
}

// ElementsFromState вычисляет кеплеровы элементы по вектору состояния в ECI.
func ElementsFromState(r, v Vec3, t float64) OrbitalElements {
	rMag := r.Norm()
	vMag := v.Norm()
	if rMag < 1e-6 {
		return OrbitalElements{Epoch: t}
	}

	// Удельный момент импульса h = r × v.
	h := r.Cross(v)
	hMag := h.Norm()

	// Вектор восходящего узла n = ẑ × h.
	n := Vec3{0, 0, 1}.Cross(h)
	nMag := n.Norm()

	// Вектор эксцентриситета e = (v × h)/GM − r̂.
	eVec := v.Cross(h).Scale(1 / GM).Sub(r.Unit())
	ecc := eVec.Norm()

	// Удельная орбитальная энергия и большая полуось.
	energy := vMag*vMag/2 - GM/rMag
	a := math.Inf(1)
	if math.Abs(energy) > 1e-9 {
		a = -GM / (2 * energy)
	}

	el := OrbitalElements{
		SemiMajorAxis: a,
		Eccentricity:  ecc,
		Inclination:   math.Acos(clamp(h.Z/hMag, -1, 1)),
		Epoch:         t,
	}

	// Долгота восходящего узла.
	if nMag > 1e-9 {
		el.RAAN = math.Acos(clamp(n.X/nMag, -1, 1))
		if n.Y < 0 {
			el.RAAN = 2*math.Pi - el.RAAN
		}
	}

	// Аргумент перицентра.
	if nMag > 1e-9 && ecc > 1e-9 {
		el.ArgumentOfPeriapsis = math.Acos(clamp(n.Dot(eVec)/(nMag*ecc), -1, 1))
		if eVec.Z < 0 {
			el.ArgumentOfPeriapsis = 2*math.Pi - el.ArgumentOfPeriapsis
		}
	}

	// Истинная аномалия.
	switch {
	case ecc > 1e-9:
		el.TrueAnomaly = math.Acos(clamp(eVec.Dot(r)/(ecc*rMag), -1, 1))
		if r.Dot(v) < 0 {
			el.TrueAnomaly = 2*math.Pi - el.TrueAnomaly
		}
	case nMag > 1e-9:
		// Круговая наклонная орбита: отсчёт от восходящего узла.
		el.TrueAnomaly = math.Acos(clamp(n.Dot(r)/(nMag*rMag), -1, 1))
		if r.Z < 0 {
			el.TrueAnomaly = 2*math.Pi - el.TrueAnomaly
		}
	default:
		// Круговая экваториальная орбита.
		el.TrueAnomaly = math.Atan2(r.Y, r.X)
	}

	el.Elliptical = ecc < 1 && a > 0 && !math.IsInf(a, 0)
	if el.Elliptical {
		el.ApoapsisAltitude = a*(1+ecc) - EarthRadius
		el.PeriapsisAltitude = a*(1-ecc) - EarthRadius
	} else {
		el.ApoapsisAltitude = math.Inf(1)
		// Перицентр существует и у гиперболы: r_p = h²/GM · 1/(1+e).
		if ecc > 0 {
			el.PeriapsisAltitude = hMag*hMag/GM/(1+ecc) - EarthRadius
		}
	}

	return el
}

// MeanAnomaly возвращает среднюю аномалию (радианы) для текущих элементов.
func (e OrbitalElements) MeanAnomaly() float64 {
	if !e.Elliptical {
		return 0
	}
	ecc := e.Eccentricity
	nu := e.TrueAnomaly
	// E = 2·atan2(√(1−e)·sin(ν/2), √(1+e)·cos(ν/2)) — форма, сохраняющая квадрант.
	ea := 2 * math.Atan2(
		math.Sqrt(1-ecc)*math.Sin(nu/2),
		math.Sqrt(1+ecc)*math.Cos(nu/2),
	)
	m := ea - ecc*math.Sin(ea)
	return normalizeAngle(m)
}

// TimeToApoapsis возвращает время до прохождения апоцентра (с).
// Для незамкнутой орбиты возвращает +Inf.
func (e OrbitalElements) TimeToApoapsis() float64 {
	if !e.Elliptical {
		return math.Inf(1)
	}
	period := e.Period()
	if period <= 0 {
		return math.Inf(1)
	}
	n := 2 * math.Pi / period
	// В апоцентре средняя аномалия равна π.
	delta := normalizeAngle(math.Pi - e.MeanAnomaly())
	return delta / n
}

// TimeToPeriapsis возвращает время до прохождения перицентра (с).
func (e OrbitalElements) TimeToPeriapsis() float64 {
	if !e.Elliptical {
		return math.Inf(1)
	}
	period := e.Period()
	if period <= 0 {
		return math.Inf(1)
	}
	n := 2 * math.Pi / period
	return normalizeAngle(-e.MeanAnomaly()) / n
}

// normalizeAngle приводит угол к диапазону [0, 2π).
func normalizeAngle(a float64) float64 {
	a = math.Mod(a, 2*math.Pi)
	if a < 0 {
		a += 2 * math.Pi
	}
	return a
}

// CircularOrbitVelocity возвращает скорость круговой орбиты на данной высоте.
func CircularOrbitVelocity(altitude float64) float64 {
	return math.Sqrt(GM / (EarthRadius + altitude))
}

// VisVivaSpeed возвращает орбитальную скорость на расстоянии r от центра
// для орбиты с большой полуосью a: v = √(GM·(2/r − 1/a)).
func VisVivaSpeed(r, a float64) float64 {
	x := GM * (2/r - 1/a)
	if x <= 0 {
		return 0
	}
	return math.Sqrt(x)
}

// LaunchAzimuthForInclination возвращает азимут (градусы от севера), при котором
// пуск с заданной широты выводит на орбиту с заданным наклонением.
//
//	sin(A) = cos(i) / cos(φ)
//
// Второй результат — признак достижимости: если |cos i| > |cos φ|, наклонение
// с этой широты недостижимо прямым выведением, и возвращается ближайший
// возможный азимут (строго на восток или на запад).
func LaunchAzimuthForInclination(latitudeDeg, inclinationDeg float64, ascending bool) (float64, bool) {
	latRad := latitudeDeg * math.Pi / 180
	incRad := inclinationDeg * math.Pi / 180

	cosLat := math.Cos(latRad)
	if math.Abs(cosLat) < 1e-6 {
		return 90, false
	}

	sinAz := math.Cos(incRad) / cosLat
	reachable := math.Abs(sinAz) <= 1
	sinAz = clamp(sinAz, -1, 1)

	az := math.Asin(sinAz) * 180 / math.Pi
	if !ascending {
		// Нисходящая ветвь витка: то же наклонение, движение к югу.
		az = 180 - az
	}
	return normalizeAzimuth(az), reachable
}
