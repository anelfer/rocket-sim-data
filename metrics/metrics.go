// Package metrics публикует телеметрию симулятора в Prometheus.
package metrics

import (
	"math"
	"sync"
	"sync/atomic"

	"github.com/prometheus/client_golang/prometheus"
)

// -----------------------------------------------------------------------------
// Имена метрик сохранены из прежней версии, чтобы существующие дашборды
// Grafana продолжали работать. Добавлены новые: число Маха, скоростной напор,
// тепловой поток, температура торможения, наклонение и период орбиты.
// -----------------------------------------------------------------------------

var (
	altitudeGauge           = gauge("rocket_altitude_meters", "Высота над уровнем моря")
	verticalVelocityGauge   = gauge("rocket_v_velocity_mps", "Радиальная (вертикальная) скорость")
	horizontalVelocityGauge = gauge("rocket_h_velocity_mps", "Горизонтальная скорость в инерциальной системе")
	groundSpeedGauge        = gauge("rocket_ground_speed_mps", "Горизонтальная скорость относительно поверхности")

	horizontalAccelGauge = gauge("rocket_h_acceleration_mps2", "Горизонтальное ускорение")
	verticalAccelGauge   = gauge("rocket_v_acceleration_mps2", "Вертикальное ускорение")

	massGauge           = gauge("rocket_mass_kg", "Текущая масса")
	fuelMassGauge       = gauge("rocket_fuel_kg", "Остаток топлива (истинный)")
	fuelMassSensedGauge = gauge("rocket_fuel_sensed_kg", "Остаток топлива (показание датчика, на нём решается MECO)")

	dragGauge            = gauge("rocket_drag_newton", "Сила лобового сопротивления")
	airDensityGauge      = gauge("rocket_air_density_kg_per_m3", "Плотность атмосферы")
	machGauge            = gauge("rocket_mach", "Число Маха относительно атмосферы")
	dynamicPressureGauge = gauge("rocket_dynamic_pressure_pa", "Скоростной напор")
	heatFluxGauge        = gauge("rocket_heat_flux_w_per_m2", "Конвективный тепловой поток в критической точке")

	ambientTempGauge  = gauge("rocket_effective_temp", "Эффективная температура среды, °C")
	recoveryTempGauge = gauge("rocket_recovery_temp_celsius", "Температура торможения потока, °C")

	engineStatusGauge = gauge("rocket_engine_status", "Число работающих двигателей")
	throttleGauge     = gauge("rocket_throttle", "Уровень дросселирования, 0…1")
	totalThrustGauge  = gauge("rocket_total_thrust_newton", "Суммарная тяга")

	rocketPositionLat = gauge("rocket_latitude", "Широта")
	rocketPositionLon = gauge("rocket_longitude", "Долгота")
	rocketPositionAlt = gauge("rocket_altitude", "Высота")
	rocketPitch       = gauge("rocket_pitch", "Тангаж: угол места продольной оси, градусы")
	rocketYaw         = gauge("rocket_yaw", "Рыскание: азимут продольной оси, градусы от севера")
	rocketRoll        = gauge("rocket_roll", "Крен вокруг продольной оси, градусы")
	rocketAzimuth     = gauge("rocket_azimuth", "Азимут продольной оси, градусы от севера")

	rocketPitchRate = gauge("rocket_pitch_rate_dps", "Угловая скорость по тангажу, град/с")
	rocketYawRate   = gauge("rocket_yaw_rate_dps", "Угловая скорость по рысканию, град/с")
	rocketRollRate  = gauge("rocket_roll_rate_dps", "Угловая скорость по крену, град/с")

	attitudeSensorErrorGauge = gauge("rocket_attitude_sensor_error_deg",
		"Расхождение показания датчика ориентации с истинной ориентацией, град")

	// --- Рулевой тракт и устойчивость ----------------------------------------
	ctrlGimbalPitch  = gauge("rocket_gimbal_pitch_deg", "Отклонение камер по тангажу")
	ctrlGimbalYaw    = gauge("rocket_gimbal_yaw_deg", "Отклонение камер по рысканию")
	ctrlGimbalDemand = gauge("rocket_gimbal_demand_deg",
		"Потребное отклонение камер до ограничения хода")
	ctrlGimbalLimit = gauge("rocket_gimbal_limit_deg", "Предельный ход привода")
	ctrlAuthority   = gauge("rocket_control_authority",
		"Отношение потребного управляющего момента к располагаемому; выше единицы — привод на упоре")
	ctrlSaturated  = gauge("rocket_control_saturated", "1 — привод на упоре")
	ctrlRCS        = gauge("rocket_control_rcs", "1 — ориентацией управляют двигатели ориентации")
	ctrlAeroTorque = gauge("rocket_aero_torque_nm",
		"Аэродинамический момент относительно центра масс")
	ctrlTorque       = gauge("rocket_control_torque_nm", "Управляющий момент")
	ctrlStaticMargin = gauge("rocket_static_margin_calibers",
		"Запас статической устойчивости в калибрах; отрицательный — корпус уводится потоком")

	rocketAoA = gauge("rocket_angle_of_attack_deg",
		"Угол атаки в плоскости тангажа со знаком: плюс — поток снизу, минус — сверху")
	rocketSideslip = gauge("rocket_sideslip_deg", "Угол скольжения, градусы")
	rocketTotalAoA = gauge("rocket_total_aoa_deg",
		"Полный угол между продольной осью и потоком, всегда неотрицательный")

	rocketTwr     = gauge("rocket_twr", "Отношение тяги к весу")
	rocketGravity = gauge("rocket_gravity_mps2", "Ускорение свободного падения на текущей высоте")

	rocketApoapsis     = gauge("rocket_apoapsis_meters", "Высота апоцентра")
	rocketPeriapsis    = gauge("rocket_periapsis_meters", "Высота перицентра")
	rocketEccentricity = gauge("rocket_orbit_eccentricity", "Эксцентриситет орбиты")
	rocketInclination  = gauge("rocket_orbit_inclination_deg", "Наклонение орбиты")
	rocketOrbitPeriod  = gauge("rocket_orbit_period_seconds", "Период обращения")
	rocketFlightPhase  = gauge("rocket_flight_phase", "Номер фазы полёта")
	rocketOrbitClosed  = gauge("rocket_orbit_closed", "1 — траектория замкнута и перицентр выше поверхности")

	// Первая ступень. Метрики публикуются с момента старта: до разделения
	// они повторяют состояние носителя, после — состояние отделившейся ступени.
	stageAltitude  = gauge("rocket_spent_stage_altitude_meters", "Высота первой ступени")
	stageLatitude  = gauge("rocket_spent_stage_latitude", "Широта первой ступени")
	stageLongitude = gauge("rocket_spent_stage_longitude", "Долгота первой ступени")
	stageSpeed     = gauge("rocket_spent_stage_speed_mps", "Скорость первой ступени относительно воздуха")
	stageMach      = gauge("rocket_spent_stage_mach", "Число Маха первой ступени")
	stageSkinTemp  = gauge("rocket_spent_stage_skin_temperature_celsius", "Температура самой горячей точки конструкции ступени, °C")
	stageHeatFlux  = gauge("rocket_spent_stage_heat_flux_w_per_m2", "Тепловой поток к первой ступени")
	stageDynamicQ  = gauge("rocket_spent_stage_dynamic_pressure_pa", "Скоростной напор на первой ступени")
	stageOutcome   = gauge("rocket_spent_stage_outcome", "Состояние: 0 — в составе носителя, 1 — отделилась, 2 — разрушилась, 3 — упала")
	stageSpinRate  = gauge("rocket_spent_stage_spin_rate_dps", "Скорость кувыркания ступени, град/с")
	stageAoA       = gauge("rocket_spent_stage_aoa_deg", "Угол между корпусом ступени и потоком, градусы")

	// Ударный слой первой ступени.
	stageShockTemp  = gauge("rocket_spent_stage_shock_temperature_celsius", "Температура газа за ударной волной у первой ступени, °C")
	stageIonization = gauge("rocket_spent_stage_ionization_fraction", "Степень ионизации ударного слоя первой ступени")
	stagePlasmaFreq = gauge("rocket_spent_stage_plasma_frequency_hz", "Плазменная частота ударного слоя первой ступени")
	stageBlackout   = gauge("rocket_spent_stage_radio_blackout", "1 — связь с первой ступенью невозможна из-за плазменной оболочки")

	stageNodeTemp = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "rocket_spent_stage_node_temperature_celsius",
			Help: "Температура участка конструкции ступени, °C",
		},
		[]string{"node"},
	)

	// Зона падения обломков.
	debrisLatitude  = gauge("rocket_debris_center_latitude", "Широта центра зоны падения обломков")
	debrisLongitude = gauge("rocket_debris_center_longitude", "Долгота центра зоны падения обломков")
	debrisMajorAxis = gauge("rocket_debris_semi_major_axis_meters", "Полуось зоны падения вдоль трассы")
	debrisMinorAxis = gauge("rocket_debris_semi_minor_axis_meters", "Полуось зоны падения поперёк трассы")
	debrisSurviving = gauge("rocket_debris_surviving_mass_fraction", "Доля массы, достигшей поверхности")

	engineThrustGauge  = gaugeVec("rocket_engine_thrust_newton", "Тяга двигателя")
	engineIspGauge     = gaugeVec("rocket_engine_isp", "Удельный импульс двигателя")
	engineChamberGauge = gaugeVec("rocket_engine_chamber_temperature_celsius", "Температура в камере сгорания, °C")
	engineNozzleGauge  = gaugeVec("rocket_engine_nozzle_temperature_celsius", "Температура газа на срезе сопла, °C")
	engineWallGauge    = gaugeVec("rocket_engine_wall_temperature_celsius", "Температура стенки камеры, °C")
	engineTurbineGauge = gaugeVec("rocket_engine_turbine_temperature_celsius", "Температура на входе турбины, °C")

	// Возвращающийся бустер (RTLS). Появляется только на носителях с активным
	// возвратом первой ступени и только с момента отделения — до этого момента
	// и после разрушения все величины не определены (см. undefinedOnStart).
	//
	// Собственная двигательная установка бустера метрик не заводит отдельно:
	// её камеры (S1-1 … S1-N) публикуются через тот же rocket_engine_*
	// с engine_id, что и у корабля — идентификаторы не пересекаются, а вести
	// две параллельные пары метрик под одни и те же величины незачем.
	boosterAltitude   = gauge("rocket_booster_altitude_meters", "Высота возвращающегося бустера")
	boosterLatitude   = gauge("rocket_booster_latitude", "Широта возвращающегося бустера")
	boosterLongitude  = gauge("rocket_booster_longitude", "Долгота возвращающегося бустера")
	boosterVVel       = gauge("rocket_booster_vertical_velocity_mps", "Вертикальная скорость бустера")
	boosterTVel       = gauge("rocket_booster_total_velocity_mps", "Полная скорость бустера")
	boosterPitch      = gauge("rocket_booster_pitch_deg", "Тангаж бустера, градусы")
	boosterYaw        = gauge("rocket_booster_yaw_deg", "Рыскание бустера, градусы")
	boosterRoll       = gauge("rocket_booster_roll_deg", "Крен бустера, градусы")
	boosterFuelMass   = gauge("rocket_booster_fuel_mass_kg", "Остаток топлива бустера")
	boosterThrottle   = gauge("rocket_booster_throttle_percent", "Уставка тяги бустера")
	boosterEngines    = gauge("rocket_booster_engines_running", "Число работающих камер бустера")
	boosterPhase      = gauge("rocket_booster_phase", "Фаза возврата: 0 — разворотный импульс, 1 — баллистика, 2 — посадочный импульс, 3 — приводнение, 4 — разрушен")
	boosterVentGas    = gauge("rocket_booster_vent_gas_kg", "Остаток газа наддува (питает и баки, и ориентацию на пассивном участке)")
	boosterTilt       = gauge("rocket_booster_tilt_deg", "Угол завала после приводнения, градусы")
	boosterSplashV    = gauge("rocket_booster_splash_speed_mps", "Скорость в момент касания воды")
	boosterSplashdown = gauge("rocket_booster_splashdown", "1 — коснулся воды")
	boosterDestroyed  = gauge("rocket_booster_destroyed", "1 — разрушен")

	boosterFinDeflection = gaugeVec("rocket_booster_grid_fin_deflection_deg", "Угол раскрытия решётчатого руля")
)

func gauge(name, help string) prometheus.Gauge {
	return prometheus.NewGauge(prometheus.GaugeOpts{Name: name, Help: help})
}

func gaugeVec(name, help string) *prometheus.GaugeVec {
	return prometheus.NewGaugeVec(
		prometheus.GaugeOpts{Name: name, Help: help},
		[]string{"engine_id"},
	)
}

// -----------------------------------------------------------------------------
// Ленивая регистрация.
//
// Метрики появляются в /actuator/prometheus только после первой публикации
// телеметрии, то есть после старта симуляции.
//
// Зарегистрированный gauge отдаёт ноль, пока в него ничего не записали. При
// интервале сбора в одну секунду Prometheus успевал набрать сколько угодно
// точек с нулевыми координатами, и на карте Grafana появлялась отметка (0°, 0°)
// в Гвинейском заливе, от которой в момент старта шёл скачок на Байконур.
// То же самое касалось нулевых высоты, массы и тяги на графиках.
//
// Регистрация по факту наличия данных решает это для всех метрик сразу:
// до старта временного ряда просто не существует, и рисовать Grafana нечего.
// -----------------------------------------------------------------------------

var (
	registerOnce sync.Once
	registered   atomic.Bool
)

func collectors() []prometheus.Collector {
	base := []prometheus.Collector{
		altitudeGauge, verticalVelocityGauge, horizontalVelocityGauge, groundSpeedGauge,
		horizontalAccelGauge, verticalAccelGauge,
		massGauge, fuelMassGauge, fuelMassSensedGauge,
		dragGauge, airDensityGauge, machGauge, dynamicPressureGauge, heatFluxGauge,
		ambientTempGauge, recoveryTempGauge,
		engineStatusGauge, throttleGauge, totalThrustGauge,
		rocketPositionLat, rocketPositionLon, rocketPositionAlt,
		rocketPitch, rocketYaw, rocketRoll, rocketAzimuth,
		rocketPitchRate, rocketYawRate, rocketRollRate, attitudeSensorErrorGauge,
		rocketAoA, rocketSideslip, rocketTotalAoA,
		ctrlGimbalPitch, ctrlGimbalYaw, ctrlGimbalDemand, ctrlGimbalLimit,
		ctrlAuthority, ctrlSaturated, ctrlRCS,
		ctrlAeroTorque, ctrlTorque, ctrlStaticMargin,
		rocketTwr, rocketGravity,
		rocketApoapsis, rocketPeriapsis, rocketEccentricity,
		rocketInclination, rocketOrbitPeriod, rocketFlightPhase, rocketOrbitClosed,
		stageAltitude, stageLatitude, stageLongitude, stageSpeed, stageMach,
		stageSkinTemp, stageHeatFlux, stageDynamicQ, stageOutcome,
		stageSpinRate, stageAoA, stageShockTemp, stageIonization,
		stagePlasmaFreq, stageBlackout, stageNodeTemp,
		debrisLatitude, debrisLongitude, debrisMajorAxis,
		debrisMinorAxis, debrisSurviving,
		engineThrustGauge, engineIspGauge, engineChamberGauge,
		engineNozzleGauge, engineWallGauge, engineTurbineGauge,
		boosterAltitude, boosterLatitude, boosterLongitude,
		boosterVVel, boosterTVel, boosterPitch, boosterYaw, boosterRoll,
		boosterFuelMass, boosterThrottle, boosterEngines, boosterPhase,
		boosterVentGas, boosterTilt, boosterSplashV, boosterSplashdown, boosterDestroyed,
		boosterFinDeflection,
	}
	return append(base, propulsionCollectors()...)
}

// undefinedOnStart — метрики, у которых нулевое значение является осмысленной
// координатой или состоянием, но данных на старте ещё нет.
//
// Зарегистрированный gauge отдаёт ноль, пока в него не записали значение.
// Для координат это означает точку (0°, 0°) в Гвинейском заливе, а для
// признака радиосвязи — уверенное «связь есть» с ещё не существующим объектом.
// Такие метрики инициализируются как NaN: Prometheus его хранит, а Grafana
// рисует на его месте разрыв.
func undefinedOnStart() []prometheus.Gauge {
	return []prometheus.Gauge{
		debrisLatitude, debrisLongitude,
		debrisMajorAxis, debrisMinorAxis, debrisSurviving,

		// Отработавшая ступень появляется только после разделения. До этого
		// момента все её величины не определены, и нулевое значение здесь
		// откровенно лжёт: высота ноль, координаты в Гвинейском заливе,
		// а признак радиосвязи читается как уверенное «связь есть»
		// с ещё не существующим объектом.
		stageAltitude, stageLatitude, stageLongitude, stageSpeed, stageMach,
		stageSkinTemp, stageHeatFlux, stageDynamicQ, stageOutcome,
		stageSpinRate, stageAoA, stageShockTemp, stageIonization,
		stagePlasmaFreq, stageBlackout,

		// Бустер существует только на носителях с активным возвратом,
		// и только с момента отделения — та же причина, что и у отработавшей
		// ступени выше.
		boosterAltitude, boosterLatitude, boosterLongitude,
		boosterVVel, boosterTVel, boosterPitch, boosterYaw, boosterRoll,
		boosterFuelMass, boosterThrottle, boosterEngines, boosterPhase,
		boosterVentGas, boosterTilt, boosterSplashV, boosterSplashdown, boosterDestroyed,
	}
}

// ensureRegistered регистрирует метрики при первой публикации телеметрии.
func ensureRegistered() {
	registerOnce.Do(func() {
		prometheus.MustRegister(collectors()...)
		for _, g := range undefinedOnStart() {
			g.Set(math.NaN())
		}
		registered.Store(true)
	})
}

// Registered сообщает, опубликована ли уже хотя бы одна порция телеметрии.
func Registered() bool {
	return registered.Load()
}

// FlightSample — набор параметров полёта для публикации.
type FlightSample struct {
	Time                   float64
	Altitude               float64
	VerticalVelocity       float64
	HorizontalVelocity     float64
	GroundSpeed            float64
	VerticalAccel          float64
	HorizontalAccel        float64
	Mass                   float64
	FuelMass               float64
	FuelMassSensed         float64
	Drag                   float64
	AirDensity             float64
	AmbientTemp            float64
	RecoveryTemp           float64
	Mach                   float64
	DynamicPressure        float64
	HeatFlux               float64
	EnginesRunning         int
	Pitch                  float64
	Yaw                    float64
	Roll                   float64
	Azimuth                float64
	PitchRate              float64
	AttitudeSensorErrorDeg float64
	YawRate                float64
	RollRate               float64
	AngleOfAttack          float64
	SideslipAngle          float64
	TotalAoA               float64

	// Рулевой тракт и статическая устойчивость.
	GimbalPitch      float64
	GimbalYaw        float64
	GimbalDemand     float64
	GimbalLimit      float64
	ControlAuthority float64
	ControlSaturated bool
	UsingRCS         bool
	AeroTorque       float64
	ControlTorque    float64
	StaticMargin     float64

	Gravity     float64
	TWR         float64
	Throttle    float64
	TotalThrust float64
	Latitude    float64
	Longitude   float64
}

// SetFlight публикует параметры полёта.
func SetFlight(s FlightSample) {
	ensureRegistered()
	altitudeGauge.Set(s.Altitude)
	verticalVelocityGauge.Set(s.VerticalVelocity)
	horizontalVelocityGauge.Set(s.HorizontalVelocity)
	groundSpeedGauge.Set(s.GroundSpeed)
	verticalAccelGauge.Set(s.VerticalAccel)
	horizontalAccelGauge.Set(s.HorizontalAccel)
	massGauge.Set(s.Mass)
	fuelMassGauge.Set(s.FuelMass)
	fuelMassSensedGauge.Set(s.FuelMassSensed)
	dragGauge.Set(s.Drag)
	airDensityGauge.Set(s.AirDensity)
	machGauge.Set(s.Mach)
	dynamicPressureGauge.Set(s.DynamicPressure)
	heatFluxGauge.Set(s.HeatFlux)
	ambientTempGauge.Set(s.AmbientTemp)
	recoveryTempGauge.Set(s.RecoveryTemp)
	engineStatusGauge.Set(float64(s.EnginesRunning))
	throttleGauge.Set(s.Throttle)
	totalThrustGauge.Set(s.TotalThrust)
	rocketPositionLat.Set(s.Latitude)
	rocketPositionLon.Set(s.Longitude)
	rocketPositionAlt.Set(s.Altitude)
	rocketPitch.Set(s.Pitch)
	rocketYaw.Set(s.Yaw)
	rocketRoll.Set(s.Roll)
	rocketAzimuth.Set(s.Azimuth)
	rocketPitchRate.Set(s.PitchRate)
	rocketYawRate.Set(s.YawRate)
	rocketRollRate.Set(s.RollRate)
	attitudeSensorErrorGauge.Set(s.AttitudeSensorErrorDeg)
	rocketAoA.Set(s.AngleOfAttack)

	ctrlGimbalPitch.Set(s.GimbalPitch)
	ctrlGimbalYaw.Set(s.GimbalYaw)
	ctrlGimbalDemand.Set(s.GimbalDemand)
	ctrlGimbalLimit.Set(s.GimbalLimit)
	ctrlAuthority.Set(s.ControlAuthority)
	ctrlSaturated.Set(boolToFloat(s.ControlSaturated))
	ctrlRCS.Set(boolToFloat(s.UsingRCS))
	ctrlAeroTorque.Set(s.AeroTorque)
	ctrlTorque.Set(s.ControlTorque)
	ctrlStaticMargin.Set(s.StaticMargin)
	rocketSideslip.Set(s.SideslipAngle)
	rocketTotalAoA.Set(s.TotalAoA)
	rocketGravity.Set(s.Gravity)
	rocketTwr.Set(s.TWR)
}

// NodeTemp — температура одного участка конструкции.
type NodeTemp struct {
	Name        string
	Temperature float64
}

// SpentStageSample — параметры отработавшей ступени.
type SpentStageSample struct {
	Altitude  float64
	Latitude  float64
	Longitude float64
	Speed     float64
	Mach      float64
	SkinTemp  float64
	HeatFlux  float64
	DynamicQ  float64
	Outcome   int

	SpinRate      float64
	AngleOfAttack float64

	ShockTemp     float64
	Ionization    float64
	PlasmaFreq    float64
	RadioBlackout bool

	// Attached сообщает, что ступень ещё не отделилась.
	Attached bool

	NodeTemps []NodeTemp
}

// SetSpentStage публикует параметры отработавшей ступени.
func SetSpentStage(s SpentStageSample) {
	ensureRegistered()
	stageAltitude.Set(s.Altitude)
	stageLatitude.Set(s.Latitude)
	stageLongitude.Set(s.Longitude)
	stageSpeed.Set(s.Speed)
	stageMach.Set(s.Mach)
	stageSkinTemp.Set(s.SkinTemp)
	stageHeatFlux.Set(s.HeatFlux)
	stageDynamicQ.Set(s.DynamicQ)
	stageOutcome.Set(float64(s.Outcome))
	stageSpinRate.Set(s.SpinRate)
	stageAoA.Set(s.AngleOfAttack)
	stageShockTemp.Set(s.ShockTemp)
	stageIonization.Set(s.Ionization)
	stagePlasmaFreq.Set(s.PlasmaFreq)
	// Радиосвязь имеет смысл только с отделившейся ступенью: плазменная
	// оболочка возникает при входе в атмосферу. Пока ступень в составе
	// носителя, величина не определена, и Grafana рисует разрыв вместо
	// вечного «связь есть».
	if s.Attached {
		stageBlackout.Set(math.NaN())
	} else {
		stageBlackout.Set(boolToFloat(s.RadioBlackout))
	}

	for _, n := range s.NodeTemps {
		stageNodeTemp.WithLabelValues(n.Name).Set(n.Temperature)
	}
}

// GridFinSample — угол раскрытия одного решётчатого руля.
type GridFinSample struct {
	Name       string
	Deflection float64
}

// BoosterSample — параметры возвращающегося бустера (RTLS).
type BoosterSample struct {
	Phase float64

	Altitude         float64
	Latitude         float64
	Longitude        float64
	VerticalVelocity float64
	TotalVelocity    float64

	Pitch, Yaw, Roll float64

	FuelMass       float64
	Throttle       float64
	EnginesRunning float64

	// VentGasMass — остаток газа наддува в баках, кг: тот же ресурс держит
	// в них давление и питает стравливание для ориентации на пассивном
	// участке (Coast) — отдельных блоков РСУ у бустера нет.
	VentGasMass float64

	Splashdown  bool
	Destroyed   bool
	SplashSpeed float64
	Tilt        float64

	GridFins []GridFinSample
}

// SetBooster публикует параметры возвращающегося бустера.
func SetBooster(s BoosterSample) {
	ensureRegistered()
	boosterPhase.Set(s.Phase)
	boosterAltitude.Set(s.Altitude)
	boosterLatitude.Set(s.Latitude)
	boosterLongitude.Set(s.Longitude)
	boosterVVel.Set(s.VerticalVelocity)
	boosterTVel.Set(s.TotalVelocity)
	boosterPitch.Set(s.Pitch)
	boosterYaw.Set(s.Yaw)
	boosterRoll.Set(s.Roll)
	boosterFuelMass.Set(s.FuelMass)
	boosterThrottle.Set(s.Throttle)
	boosterEngines.Set(s.EnginesRunning)
	boosterVentGas.Set(s.VentGasMass)
	boosterTilt.Set(s.Tilt)
	boosterSplashdown.Set(boolToFloat(s.Splashdown))
	boosterDestroyed.Set(boolToFloat(s.Destroyed))
	// Скорость касания воды имеет смысл только после самого касания —
	// та же причина, что и у радиосвязи отработавшей ступени выше.
	if s.Splashdown {
		boosterSplashV.Set(s.SplashSpeed)
	} else {
		boosterSplashV.Set(math.NaN())
	}
	for _, f := range s.GridFins {
		boosterFinDeflection.WithLabelValues(f.Name).Set(f.Deflection)
	}
}

// DebrisSample — сводка по зоне падения обломков.
type DebrisSample struct {
	CenterLatitude  float64
	CenterLongitude float64
	SemiMajorAxis   float64
	SemiMinorAxis   float64
	SurvivingMass   float64
}

// SetDebris публикует параметры зоны падения обломков.
func SetDebris(s DebrisSample) {
	ensureRegistered()
	debrisLatitude.Set(s.CenterLatitude)
	debrisLongitude.Set(s.CenterLongitude)
	debrisMajorAxis.Set(s.SemiMajorAxis)
	debrisMinorAxis.Set(s.SemiMinorAxis)
	debrisSurviving.Set(s.SurvivingMass)
}

// OrbitSample — параметры орбиты.
type OrbitSample struct {
	Apoapsis     float64
	Periapsis    float64
	Eccentricity float64
	Inclination  float64
	Period       float64
	Phase        int
	Closed       bool
}

// SetOrbit публикует параметры орбиты.
//
// Апоцентр и перицентр приходят сюда как NaN, пока траектория не замкнута.
// Prometheus корректно хранит NaN, а Grafana рисует на его месте разрыв —
// это честнее, чем показывать перицентр в минус шесть тысяч километров,
// который формально верен для баллистической горки, но ломает масштаб графика.
func SetOrbit(s OrbitSample) {
	ensureRegistered()
	rocketApoapsis.Set(s.Apoapsis)
	rocketPeriapsis.Set(s.Periapsis)
	rocketEccentricity.Set(s.Eccentricity)
	rocketInclination.Set(s.Inclination)
	rocketOrbitPeriod.Set(s.Period)
	rocketFlightPhase.Set(float64(s.Phase))
	rocketOrbitClosed.Set(boolToFloat(s.Closed))
}

func boolToFloat(b bool) float64 {
	if b {
		return 1
	}
	return 0
}

// EngineSample — параметры одного двигателя.
type EngineSample struct {
	ID          string
	Thrust      float64
	ISP         float64
	ChamberTemp float64
	NozzleTemp  float64
	WallTemp    float64
	TurbineTemp float64
}

// SetEngine публикует параметры двигателя.
func SetEngine(s EngineSample) {
	ensureRegistered()
	engineThrustGauge.WithLabelValues(s.ID).Set(s.Thrust)
	engineIspGauge.WithLabelValues(s.ID).Set(s.ISP)
	engineChamberGauge.WithLabelValues(s.ID).Set(s.ChamberTemp)
	engineNozzleGauge.WithLabelValues(s.ID).Set(s.NozzleTemp)
	engineWallGauge.WithLabelValues(s.ID).Set(s.WallTemp)
	engineTurbineGauge.WithLabelValues(s.ID).Set(s.TurbineTemp)
}

// RetireEngines удаляет из реестра наборы меток двигателей, которых больше нет.
//
// Без этого метрики отделившейся ступени навсегда застывали на последнем
// записанном значении: Prometheus продолжал отдавать их при каждом сборе,
// и на графике температура камеры выглядела как «зависшая» линия.
func RetireEngines(ids []string) {
	for _, id := range ids {
		labels := prometheus.Labels{"engine_id": id}
		engineThrustGauge.Delete(labels)
		engineIspGauge.Delete(labels)
		engineChamberGauge.Delete(labels)
		engineNozzleGauge.Delete(labels)
		engineWallGauge.Delete(labels)
		engineTurbineGauge.Delete(labels)

		// Подробные метрики двигательной установки тоже размечены двигателем,
		// и их нужно снимать вместе с остальными: иначе после разделения
		// обороты турбонасоса улетевшей ступени навсегда останутся на графике
		// застывшей горизонтальной линией.
		retirePropulsion(labels)
	}
}
