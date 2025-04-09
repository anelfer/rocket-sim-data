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
func (orbit OrbitType) GetTargetOrbitAltitude() float64 {
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

func (orbit OrbitType) GetPitchTurnParams() (float64, float64, float64) {
	switch orbit {
	case OrbitLEO:
		// Старт поворота с 1.5 км, основная фаза до 60 км, потом до орбиты.
		return 1500.0, 100000.0, 47.0
	case OrbitSSO:
		// Начинаем чуть позже, т.к. SSO требует немного большего вертикального компонента.
		return 2000.0, 120000.0, 55.0
	case OrbitMEO:
		// Дальняя орбита — плавный наклон, больше фазы вертикального полёта.
		return 5000.0, 160000.0, 60.0
	case OrbitGEO:
		// GEO — стартуем выше и поворачиваем плавно, больше "вверх".
		return 8000.0, 200000.0, 65.0
	default:
		return 2000.0, 160000.0, 45.0
	}
}

// ComputePitchByAltitude — простая сигмоида для наклона тяги
// ComputePitchByAltitude — сигмоида с удержанием наклона после MECO
func ComputePitchByAltitude(altitude float64, elapsedTime float64, mission OrbitType) float64 {
	targetAlt := mission.GetTargetOrbitAltitude()
	start, end, minSafe := mission.GetPitchTurnParams()

	initial := 90.0
	//holdDuration := 20.0 // удержание в секундах

	if altitude <= start {
		return initial
	}

	// Первая фаза: от initial до minSafe
	if altitude < end {
		t := (altitude - start) / (end - start)
		eased := 0.5 * (1 - math.Cos(t*math.Pi)) // от 0 до 1
		pitch := initial*(1-eased) + minSafe*eased
		return pitch
	}

	// Вторая фаза: удержание наклона
	// Предположим, что момент входа во вторую фазу — когда altitude >= end
	// Тогда запомним время начала второй фазы
	holdEndTime := 150 + 20.0 // должен вернуть время начала второй фазы + holdDuration

	if elapsedTime < holdEndTime {
		return minSafe
	}

	// Третья фаза: плавный спад до 0
	t := (altitude - end) / (targetAlt - end)
	eased := 0.5 * (1 - math.Cos(t*math.Pi))
	pitch := minSafe * (1 - eased)
	return pitch
}
