package orbit

import "math"

type OrbitType int

const (
	OrbitLEO OrbitType = iota // Низкая околоземная орбита
	OrbitSSO                  // Полярная/Солнечно-синхронная
	OrbitMEO                  // Средняя орбита
	OrbitGEO                  // Геостационарная
)

// GetTargetOrbitAltitude возвращает целевую высоту орбиты (в метрах)
func GetTargetOrbitAltitude(orbit OrbitType) float64 {
	switch orbit {
	case OrbitLEO:
		return 400000.0 // 400 км
	case OrbitSSO:
		return 750000.0 // 750 км (полярные спутники)
	case OrbitMEO:
		return 20200000.0 // 20 200 км (например, GPS)
	case OrbitGEO:
		return 35786000.0 // 35 786 км (геостационарная)
	default:
		return 400000.0 // по умолчанию LEO
	}
}

// ComputePitchByAltitude — простая сигмоида для наклона тяги
func ComputePitchByAltitude(altitude float64, mission OrbitType) float64 {
	targetAlt := GetTargetOrbitAltitude(mission)

	// Настраиваемые параметры
	pitchoverStart := 0.005 * targetAlt
	pitchoverEnd := 0.125 * targetAlt
	initialPitch := 90.0

	// Минимальный допустимый pitch, пока не достигнута нужная высота
	minSafePitch := 5.0

	if altitude < pitchoverStart {
		return initialPitch
	}

	if altitude < pitchoverEnd {
		progress := (altitude - pitchoverStart) / (pitchoverEnd - pitchoverStart)
		eased := 0.5 * (1 - math.Cos(progress*math.Pi))
		pitch := initialPitch * (1 - eased)
		return math.Max(pitch, minSafePitch) // ⬅ НЕ давать упасть ниже safe pitch
	}

	// Если высота всё ещё ниже цели — держим safe pitch
	if altitude < 0.95*targetAlt {
		return minSafePitch
	}

	return 0.0
}
