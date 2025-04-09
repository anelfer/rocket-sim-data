package orbit

import (
	"math"
)

const (
	earthRadius = 6371000.0
	earthOmega  = 7.2921159e-5
)

// GuidanceUpdatePosition обновляет позицию ракеты за 1 секунду
func GuidanceUpdatePosition(
	lat, lon float64, // текущие координаты (градусы)
	velocity float64, // полная скорость ракеты (м/с)
	pitch float64, // угол к горизонту (градусы), 90 = вверх
	heading float64, // азимут (градусы), 90 = восток
) (float64, float64) {
	latRad := lat * math.Pi / 180
	lonRad := lon * math.Pi / 180
	headingRad := heading * math.Pi / 180
	pitchRad := pitch * math.Pi / 180

	// Горизонтальная компонента скорости (движение по поверхности)
	horizontalVelocity := velocity * math.Cos(pitchRad)

	// Расстояние за 1 секунду (горизонтальное перемещение по сфере)
	d := horizontalVelocity // м/с

	// Обновляем широту
	newLat := math.Asin(
		math.Sin(latRad)*math.Cos(d/earthRadius) +
			math.Cos(latRad)*math.Sin(d/earthRadius)*math.Cos(headingRad),
	)

	// Обновляем долготу
	newLon := lonRad + math.Atan2(
		math.Sin(headingRad)*math.Sin(d/earthRadius)*math.Cos(latRad),
		math.Cos(d/earthRadius)-math.Sin(latRad)*math.Sin(newLat),
	)

	return newLat * 180 / math.Pi, newLon * 180 / math.Pi
}

func EarthRotationSpeedAtLatitude(latitude float64) float64 {
	const omega = 7.2921159e-5 // рад/с
	const radius = 6371000.0   // м
	latRad := latitude * math.Pi / 180
	return omega * radius * math.Cos(latRad) // в м/с
}
