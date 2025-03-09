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
	baseLat, baseLon := 45.9647, 63.3050

	// Высота, на которой ракета начинает горизонтальный манёвр
	turnStartAltitude := 5000.0

	// Горизонтальный угол направления полета ракеты
	directionAngle := 45.0 // 0 - север, 90 - восток и т.д.

	// Считаем горизонтальное смещение
	var horizontalDistance float64

	if altitude < turnStartAltitude {
		// сначала почти строго вверх
		horizontalDistance = altitude * 0.01
	} else {
		// затем увеличиваем горизонтальное движение
		horizontalDistance = (altitude - turnStartAltitude) * 1.5
	}

	// Переводим horizontalDistance в реальные координаты lat/lon:
	deltaLat := (horizontalDistance * math.Cos(directionAngle*math.Pi/180)) / 111000.0
	deltaLon := (horizontalDistance * math.Sin(directionAngle*math.Pi/180)) / (111000.0 * math.Cos(baseLat*math.Pi/180))

	lat := baseLat + deltaLat
	lon := baseLon + deltaLon
	alt := altitude // Altitude — текущая высота ракеты

	// Отправляем новые метрики в Prometheus
	rocketPositionLat.Set(lat)
	rocketPositionLon.Set(lon)
	rocketPositionAlt.Set(alt)
}

func SetEngineThrust(engineID string, thrust float64) {
	engineThrustGauge.WithLabelValues(engineID).Set(thrust)
}
