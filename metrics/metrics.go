package metrics

import (
	"fmt"
	"github.com/prometheus/client_golang/prometheus"
	"rocketTelemetrySim/utils"
)

var (
	altitudeGauge               = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_altitude_meters"})
	verticalVelocityGauge       = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_v_velocity_mps"})
	horizontalVelocityGauge     = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_h_velocity_mps"})
	horizontalAccelerationGauge = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_h_acceleration_mps2"})
	verticalAccelerationGauge   = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_v_acceleration_mps2"})
	massGauge                   = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_mass_kg"})
	dragGauge                   = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_drag_newton"})
	airDensityGauge             = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_air_density_kg_per_m3"})
	engineStatusGauge           = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_engine_status"})
	effectiveAmbientTemp        = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_effective_temp"})
	engineThrustGauge           = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "rocket_engine_thrust_newton",
			Help: "Current thrust of each rocket engine (in Newtons)",
		},
		[]string{"engine_id"},
	)
	engineChamberTempGauge = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "rocket_engine_chamber_temperature_celsius",
			Help: "Current combustion chamber temperature of each rocket engine (in Celsius)",
		},
		[]string{"engine_id"},
	)
	engineNozzleTempGauge = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "rocket_engine_nozzle_temperature_celsius",
			Help: "Current nozzle temperature of each rocket engine (in Celsius)",
		},
		[]string{"engine_id"},
	)
	engineWallTempGauge = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "rocket_engine_wall_temperature_celsius",
			Help: "Current wall temperature of each rocket engine (in Celsius)",
		},
		[]string{"engine_id"},
	)
	engineTurbineTempGauge = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "rocket_engine_turbine_temperature_celsius",
			Help: "Current turbine temperature of each rocket engine (in Celsius)",
		},
		[]string{"engine_id"},
	)
	engineIspGauge = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "rocket_engine_isp",
		},
		[]string{"engine_id"},
	)
	engineFuelTempGauge = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "rocket_engine_fuel_temperature_celsius",
			Help: "Current fuel temperature for each rocket engine (in Celsius)",
		},
		[]string{"engine_id"},
	)
	engineOxidizerTempGauge = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "rocket_engine_oxidizer_temperature_celsius",
			Help: "Current oxidizer temperature for each rocket engine (in Celsius)",
		},
		[]string{"engine_id"},
	)
	rocketPositionLat = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_latitude"})
	rocketPositionLon = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_longitude"})
	rocketPositionAlt = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_altitude"})
	rocketPitch       = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_pitch"})
	rocketTwr         = prometheus.NewGauge(prometheus.GaugeOpts{Name: "rocket_twr"})
)

func init() {
	prometheus.MustRegister(
		altitudeGauge, verticalVelocityGauge, horizontalVelocityGauge,
		horizontalAccelerationGauge, verticalAccelerationGauge, effectiveAmbientTemp,
		massGauge, dragGauge, airDensityGauge,
		engineStatusGauge, engineThrustGauge,
		rocketPositionLat, rocketPositionLon, rocketPositionAlt,
		engineChamberTempGauge, engineNozzleTempGauge, engineWallTempGauge,
		engineTurbineTempGauge, engineIspGauge, engineFuelTempGauge,
		engineOxidizerTempGauge, rocketPitch, rocketTwr,
	)
	rocketPositionLat.Set(lat)
	rocketPositionLon.Set(lon)
}

var lat = 45.9647
var lon = 63.3050
var heading = 90.0 // восток

func SendBasicMetrics(altitude, verticalVelocity, horizontalVelocity, horizontalAccel, verticalAccel, mass, drag, airDensity float64, enginesRunning int, ambientTemp, pitch, gravity, totalThrust, time, latV, lonV float64) {
	fmt.Printf("Time: %.2f s | Altitude: %.2f m | V_Velocity: %.2f m/s | V_Acceleration: %.2f m/s² | Mass: %.2f kg | Drag: %.2f N | Air Density: %.4f kg/m³ | Ambient Temp: %.2f C | Engine: %.d | Pitch: %.3f | Gravity: %.3f\n",
		time, altitude, verticalVelocity, verticalAccel, mass, drag, airDensity, ambientTemp, enginesRunning, pitch, gravity)
	altitudeGauge.Set(altitude)
	verticalVelocityGauge.Set(verticalVelocity)
	horizontalVelocityGauge.Set(horizontalVelocity)
	horizontalAccelerationGauge.Set(horizontalAccel)
	verticalAccelerationGauge.Set(verticalAccel)
	massGauge.Set(mass)
	dragGauge.Set(drag)
	airDensityGauge.Set(airDensity)
	engineStatusGauge.Set(float64(enginesRunning))
	effectiveAmbientTemp.Set(ambientTemp)

	// Координаты места запуска (например, космодром Байконур)
	alt := altitude // Altitude — текущая высота ракеты

	// Отправляем новые метрики в Prometheus
	rocketPositionLat.Set(latV)
	rocketPositionLon.Set(lonV)
	rocketPositionAlt.Set(alt)
	rocketPitch.Set(pitch)
	rocketTwr.Set(utils.CalculateTWR(totalThrust, mass, gravity))
}

func SetEngineThrust(engineID string, thrust, isp, chamberTemp, nozzleTemp, wallTemp, turbineTemp float64) {
	engineThrustGauge.WithLabelValues(engineID).Set(thrust)
	engineIspGauge.WithLabelValues(engineID).Set(isp)
	engineChamberTempGauge.WithLabelValues(engineID).Set(chamberTemp)
	engineNozzleTempGauge.WithLabelValues(engineID).Set(nozzleTemp)
	engineWallTempGauge.WithLabelValues(engineID).Set(wallTemp)
	engineTurbineTempGauge.WithLabelValues(engineID).Set(turbineTemp)
}
