package orbit

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
		return 1250.0, 55600.0, 23.5
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
