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
	altitude float64, // высота (м)
	velocity float64, // полная скорость ракеты (м/с)
	heading float64, // направление (азимут) в градусах
	mission OrbitType, // тип орбиты
) (float64, float64) {

	latRad := lat * math.Pi / 180
	lonRad := lon * math.Pi / 180
	headingRad := heading * math.Pi / 180

	// Вращение Земли
	//earthRotationSpeed := EarthRotationSpeedAtLatitude(altitude)

	// Питч по высоте
	//pitchAngle := ComputePitchByAltitude(altitude, mission)
	//pitchRad := pitchAngle * math.Pi / 180

	// Делим скорость на вертикальную и горизонтальную
	//vHoriz := velocity * math.Cos(pitchRad)
	//totalHorizVelocity := vHoriz + earthRotationSpeed

	// Расстояние по сфере за 1 секунду
	d := velocity

	newLat := math.Asin(
		math.Sin(latRad)*math.Cos(d/earthRadius) +
			math.Cos(latRad)*math.Sin(d/earthRadius)*math.Cos(headingRad),
	)

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
