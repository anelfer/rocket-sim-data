package metrics

import (
	"fmt"
	"github.com/prometheus/client_golang/prometheus"
	"math"
)

var (
	altitudeGauge     = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_altitude_meters"})
	velocityGauge     = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_velocity_mps"})
	accelerationGauge = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_acceleration_mps2"})
	massGauge         = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_mass_kg"})
	dragGauge         = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_drag_newton"})
	airDensityGauge   = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_air_density_kg_per_m3"})
	engineStatusGauge = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_engine_status"})
	engineThrustGauge = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "rocket_engine_thrust_newton",
			Help: "Current thrust of each rocket engine (in Newtons)",
		},
		[]string{"engine_id"},
	)

	rocketPositionLat = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_latitude"})
	rocketPositionLon = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_longitude"})
	rocketPositionAlt = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_altitude"})
)

func init() {
	prometheus.MustRegister(
		altitudeGauge, velocityGauge, accelerationGauge,
		massGauge, dragGauge, airDensityGauge,
		engineStatusGauge, engineThrustGauge,
		rocketPositionLat, rocketPositionLon, rocketPositionAlt,
	)
}

func SendBasicMetrics(altitude, velocity, acceleration, mass, drag, airDensity float64, enginesRunning int) {
	fmt.Printf("Altitude: %.2f m | Velocity: %.2f m/s | Acceleration: %.2f m/s² | Mass: %.2f kg | Drag: %.2f N | Air Density: %.4f kg/m³ | Engine: %.d\n",
		altitude, velocity, acceleration, mass, drag, airDensity, enginesRunning)
	altitudeGauge.Set(altitude)
	velocityGauge.Set(velocity)
	accelerationGauge.Set(acceleration)
	massGauge.Set(mass)
	dragGauge.Set(drag)
	airDensityGauge.Set(airDensity)
	engineStatusGauge.Set(float64(enginesRunning))

	// Координаты места запуска (например, космодром Байконур)
	lat, lon := GetLatLon(altitude, velocity)
	alt := altitude // Altitude — текущая высота ракеты

	// Отправляем новые метрики в Prometheus
	rocketPositionLat.Set(lat)
	rocketPositionLon.Set(lon)
	rocketPositionAlt.Set(alt)
}

func GetLatLon(altitude float64, velocity float64) (float64, float64) {
	baseLat, baseLon := 45.9647, 63.3050

	h0 := 5000.0 // Высота начала поворота
	k := 2000.0  // Параметр плавности

	turnAngle := 90.0 / (1.0 + math.Exp(-(altitude-h0)/k)) // В градусах

	R := 6371000.0         // Радиус Земли
	directionAngle := 90.0 // Пусть по умолчанию ракета летит на восток

	// Вычисляем горизонтальную скорость с учётом угла атаки
	horizontalVelocity := velocity * math.Sin(turnAngle*math.Pi/180)

	earthRotationSpeed := 465 * math.Cos(baseLat*math.Pi/180) // Скорость вращения Земли в м/с
	initialHorizontalSpeed := earthRotationSpeed
	// Учитываем вращение Земли
	horizontalVelocity += initialHorizontalSpeed

	// Обновляем координаты
	deltaLat := (horizontalVelocity * math.Cos(directionAngle*math.Pi/180)) / R
	deltaLon := (horizontalVelocity * math.Sin(directionAngle*math.Pi/180)) / (R * math.Cos(baseLat*math.Pi/180))

	lat := baseLat + (deltaLat * 180 / math.Pi) // Переводим в градусы
	lon := baseLon + (deltaLon * 180 / math.Pi)
	return lat, lon
}

func SetEngineThrust(engineID string, thrust float64) {
	engineThrustGauge.WithLabelValues(engineID).Set(thrust)
}
