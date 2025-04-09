package simulator

import (
	"math"
	"math/rand"
)

// AngleBetweenEngines возвращает угол в градусах между двумя двигателями.
func AngleBetweenEngines(e1, e2 Engine) float64 {
	return math.Atan2(e2.Y-e1.Y, e2.X-e1.X) * (180 / math.Pi)
}

// IsOpposite определяет, является ли двигатель противоположным (угол ~180°).
func IsOpposite(e1, e2 Engine) bool {
	angle := math.Abs(AngleBetweenEngines(e1, e2))
	return angle > 150 && angle <= 210
}

// IsNeighbor определяет, является ли двигатель соседним (угол <90°).
func IsNeighbor(e1, e2 Engine) bool {
	angle := math.Abs(AngleBetweenEngines(e1, e2))
	return angle <= 90
}

// RandomThrustAdjustment возвращает случайное отклонение для тяги в указанном диапазоне.
func RandomThrustAdjustment(rng *rand.Rand, min, max float64) float64 {
	value := min + rng.Float64()*(max-min)
	if rng.Intn(2) == 0 {
		return -value
	}
	return value
}

func KelvinToCelsius(kelvin float64) float64 {
	return kelvin - 273.15
}

func CdByVelocity(velocity, tempK float64) float64 {
	// Скорость звука (м/с)
	a := SpeedOfSound(tempK)

	// Число Маха
	mach := velocity / a
	return CdByMach(mach)
}

// CdByMach возвращает приближённый коэффициент лобового сопротивления по числу Маха
func CdByMach(m float64) float64 {
	switch {
	case m < 0.8:
		return 0.2
	case m >= 0.8 && m < 1.0:
		t := (m - 0.8) / (1.0 - 0.8)
		return 0.2 + t*(0.5-0.2)
	case m >= 1.0 && m <= 1.2:
		t := (m - 1.0) / (1.2 - 1.0)
		return 0.5 - t*(0.5-0.45)
	case m > 1.2 && m <= 3.0:
		t := (m - 1.2) / (3.0 - 1.2)
		return 0.45 - t*(0.45-0.35)
	case m > 3.0 && m <= 5.0:
		t := (m - 3.0) / (5.0 - 3.0)
		return 0.35 - t*(0.35-0.31)
	default:
		return 0.31
	}
}

// SpeedOfSound возвращает скорость звука (м/с) при заданной температуре (в Кельвинах)
func SpeedOfSound(tempK float64) float64 {
	const gamma = 1.4
	const R = 287.05
	return math.Sqrt(gamma * R * tempK)
}

func TotalThrust(engines []Engine) float64 {
	total := 0.0
	for _, e := range engines {
		if e.Running {
			total += e.Thrust
		}
	}
	return total
}

func MaxTotalThrust(engines []Engine) float64 {
	total := 0.0
	for _, e := range engines {
		if e.Running {
			total += e.MaxThrust
		}
	}
	return total
}
