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

// CdByMach возвращает приближённый коэффициент лобового сопротивления по числу Маха
func CdByMach(mach float64) float64 {
	switch {
	case mach < 0.8:
		return 0.2
	case mach >= 0.8 && mach <= 1.2:
		return 0.5
	case mach > 1.2 && mach <= 3.0:
		return 0.4
	case mach > 3.0 && mach <= 6.0:
		return 0.3
	default:
		return 0.25
	}
}

// SpeedOfSound возвращает скорость звука (м/с) при заданной температуре (в Кельвинах)
func SpeedOfSound(tempK float64) float64 {
	const gamma = 1.4
	const R = 287.05
	return math.Sqrt(gamma * R * tempK)
}
