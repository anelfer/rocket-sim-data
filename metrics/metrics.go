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
	boosterAltitude  = gauge("rocket_booster_altitude_meters", "Высота возвращающегося бустера")
	boosterLatitude  = gauge("rocket_booster_latitude", "Широта возвращающегося бустера")
	boosterLongitude = gauge("rocket_booster_longitude", "Долгота возвращающегося бустера")
	boosterVVel      = gauge("rocket_booster_vertical_velocity_mps", "Вертикальная скорость бустера")
	boosterTVel      = gauge("rocket_booster_total_velocity_mps", "Полная скорость бустера")
	// Сырые углы Эйлера: диагностические. Азимут выдаётся в 0…360, крен —
	// в −180…180, и оба рвутся на границах там, где корпус поворачивается
	// плавно. Для чтения ориентации служат непрерывные двойники ниже.
	boosterPitch = gauge("rocket_booster_pitch_deg", "Тангаж бустера, градусы (угол места, −90…90, не цикличен)")
	boosterYaw   = gauge("rocket_booster_yaw_deg", "Рыскание бустера, градусы (сырой азимут 0…360, рвётся на границе)")
	boosterRoll  = gauge("rocket_booster_roll_deg", "Крен бустера, градусы (сырой, −180…180, рвётся на границе)")

	// Развёрнутые (непрерывные) углы: те же величины с накоплением, без
	// разрывов на 360°/0° и ±180°. Выходят за пределы оборота — так и
	// задумано, обороты должны быть видны как обороты.
	// Захват башней. Промах меряется относительно ЦЕНТРА ЗОНЫ ЗАХВАТА между
	// руками, а не относительно основания башни, и снимается в момент
	// прохождения плоскости рук, а не после падения корпуса: положение
	// упавшей и завалившейся ступени — это уже другая величина, и выдавать
	// её за ошибку наведения нельзя.
	boosterCatchAlong      = gauge("rocket_booster_catch_miss_along_m", "Промах захвата вдоль рук башни, м (NaN до прохода зоны)")
	boosterCatchAcross     = gauge("rocket_booster_catch_miss_across_m", "Промах захвата поперёк просвета между руками, м (NaN до прохода зоны)")
	boosterCatchVertical   = gauge("rocket_booster_catch_miss_vertical_m", "Промах захвата по высоте относительно плоскости рук, м (NaN до прохода зоны)")
	boosterCatchHorizontal = gauge("rocket_booster_catch_miss_horizontal_m", "Горизонтальный промах захвата, м (NaN до прохода зоны)")
	boosterCatch3D         = gauge("rocket_booster_catch_miss_3d_m", "Полный трёхмерный промах захвата, м (NaN до прохода зоны)")
	boosterCatchDownrange  = gauge("rocket_booster_catch_miss_downrange_m", "Продольный промах вдоль курса отлёта в момент прохода зоны, м")
	boosterCatchCrossrange = gauge("rocket_booster_catch_miss_crossrange_m", "Боковой промах поперёк курса отлёта в момент прохода зоны, м")
	boosterCatchVVel       = gauge("rocket_booster_catch_vertical_velocity_mps", "Скорость снижения в момент прохода зоны захвата, м/с")
	boosterCatchHVel       = gauge("rocket_booster_catch_horizontal_velocity_mps", "Боковая скорость в момент прохода зоны захвата, м/с")
	boosterCatchTilt       = gauge("rocket_booster_catch_tilt_deg", "Наклон корпуса от вертикали в момент прохода зоны захвата, градусы")
	boosterCatchRate       = gauge("rocket_booster_catch_angular_rate_dps", "Модуль угловой скорости в момент прохода зоны захвата, град/с")
	boosterCatchCrossed    = gauge("rocket_booster_catch_crossed", "Плоскость захвата пройдена сверху вниз: 1 — да, 0 — нет")
	boosterCatchSuccess    = gauge("rocket_booster_catch_success", "Захват состоялся: 1 — да, 0 — нет")

	boosterYawCont  = gauge("rocket_booster_yaw_continuous_deg", "Рыскание бустера, непрерывное: 359° → 361° вместо 359° → 1°")
	boosterRollCont = gauge("rocket_booster_roll_continuous_deg", "Крен бустера, непрерывный: 179° → 181° вместо 179° → −179°")

	// Собственные угловые скорости в СВЯЗАННЫХ осях. Особых точек у них нет
	// вовсе: это не координаты ориентации, а сама угловая скорость корпуса.
	// Углы Эйлера у вертикально стоящего бустера вырождены — азимут метётся
	// сколь угодно быстро при сколь угодно медленном физическом вращении, —
	// и никакое разворачивание этого не лечит, потому что лечить тут нечего:
	// такова сама координата. Судить о вращении следует по этим трём.
	boosterRateRoll   = gauge("rocket_booster_body_roll_rate_dps", "Угловая скорость бустера вокруг продольной оси, град/с")
	boosterRatePitch  = gauge("rocket_booster_body_pitch_rate_dps", "Угловая скорость бустера по тангажу (связанная ось), град/с")
	boosterRateYaw    = gauge("rocket_booster_body_yaw_rate_dps", "Угловая скорость бустера по рысканию (связанная ось), град/с")
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

	// Наведение посадочного импульса по G-FOLD (simulator/orbit/gfold).
	// Пять величин, которыми объясняется любое поведение контура: сошлась
	// ли задача, каким вышло решение, насколько ступень от него отстала и
	// осталась ли релаксация точной.
	boosterFinTorqueReq = gauge("rocket_booster_grid_fin_torque_requested_nm",
		"Момент, запрошенный у решётчатых рулей контуром ориентации")
	boosterFinTorqueDone = gauge("rocket_booster_grid_fin_torque_delivered_nm",
		"Момент, который рули дают на назначенных углах")
	boosterFinSaturated = gauge("rocket_booster_grid_fin_saturated",
		"1 — рули на упоре: запрошенный момент не выдаётся")

	// Теневая проба терминальной задачи на пассивном участке: наведение по
	// ней не идёт, она только отвечает, достижима ли ещё площадка из
	// предсказанной точки розжига. Именно её и не хватало, чтобы видеть
	// потерю цели задолго до включения настоящего G-FOLD.
	boosterGfoldShadowStatus = gauge("rocket_booster_gfold_shadow_status",
		"Исход теневой пробы терминальной задачи: 0 — решение найдено, 1 — цель недостижима, 2 — решатель не сошёлся, NaN — проба ещё не делалась")
	boosterGfoldShadowMiss = gauge("rocket_booster_gfold_shadow_miss_meters",
		"Промах теневого решения: на сколько терминальный узел пробной траектории расходится с зоной захвата")
	boosterGfoldShadowTof = gauge("rocket_booster_gfold_shadow_time_of_flight_seconds",
		"Время полёта теневой траектории")
	boosterGfoldShadowReach = gauge("rocket_booster_gfold_shadow_reachable",
		"1 — мягкая посадка в зоне захвата из предсказанной точки розжига ещё разрешима")

	// Боковое наведение пассивного участка. Промах разомкнутый (что будет,
	// если больше не править), поэтому по нему и видно, где ошибка
	// появляется, а запрос/власть отвечают, упёрлись ли рули.
	boosterPassiveMiss = gauge("rocket_booster_passive_miss_meters",
		"Разомкнутый прогнозируемый промах пассивного участка: куда придёт ступень, если боковой коррекции больше не делать")
	boosterCoastLeanDemand = gauge("rocket_booster_coast_lean_demand_deg",
		"Отклонение от tail-first, которое запросило боковое наведение по промаху")
	boosterCoastLeanAuthority = gauge("rocket_booster_coast_lean_authority_deg",
		"Отклонение от tail-first, которое удержат решётчатые рули и газоотвод при текущем напоре")
	boosterCoastLeanApplied = gauge("rocket_booster_coast_lean_applied_deg",
		"Отклонение от tail-first, фактически поданное в цель ориентации")

	boosterGfoldStatus = gauge("rocket_booster_gfold_status",
		"Исход последнего решения задачи наведения: 0 — решение найдено, 1 — цель недостижима, 2 — решатель не сошёлся, NaN — задача ещё не решалась")
	boosterGfoldSolves = gauge("rocket_booster_gfold_solves_total",
		"Сколько раз запрошено решение задачи наведения посадочного импульса")
	boosterGfoldFailures = gauge("rocket_booster_gfold_failures_total",
		"Сколько решений не удалось получить (ступень летит по предыдущему плану)")
	boosterGfoldMiss = gauge("rocket_booster_gfold_plan_miss_meters",
		"Промах ПЛАНА: расхождение терминального узла найденной траектории с точкой посадки")
	boosterGfoldTof = gauge("rocket_booster_gfold_time_of_flight_seconds",
		"Время полёта найденной траектории")
	boosterGfoldFuel = gauge("rocket_booster_gfold_planned_fuel_kg",
		"Запланированный расход топлива на найденной траектории")
	boosterGfoldSlack = gauge("rocket_booster_gfold_slack",
		"Невязка условия ‖u‖=σ в решении: мера того, осталась ли релаксация точной (теорема о lossless convexification)")
	boosterGfoldTrackPos = gauge("rocket_booster_gfold_tracking_position_error_meters",
		"Отставание от опорной траектории по положению")
	boosterGfoldTrackVel = gauge("rocket_booster_gfold_tracking_velocity_error_mps",
		"Отставание от опорной траектории по скорости")
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
		boosterYawCont, boosterRollCont,
		boosterRateRoll, boosterRatePitch, boosterRateYaw,
		boosterFuelMass, boosterThrottle, boosterEngines, boosterPhase,
		boosterVentGas, boosterTilt, boosterSplashV, boosterSplashdown, boosterDestroyed,
		boosterFinDeflection,
		boosterGfoldStatus, boosterGfoldSolves, boosterGfoldFailures,
		boosterFinTorqueReq, boosterFinTorqueDone, boosterFinSaturated,
		boosterGfoldShadowStatus, boosterGfoldShadowMiss, boosterGfoldShadowTof,
		boosterGfoldShadowReach,
		boosterPassiveMiss, boosterCoastLeanDemand, boosterCoastLeanAuthority,
		boosterCoastLeanApplied,
		boosterGfoldMiss, boosterGfoldTof, boosterGfoldFuel, boosterGfoldSlack,
		boosterGfoldTrackPos, boosterGfoldTrackVel,
		boosterCatchAlong, boosterCatchAcross, boosterCatchVertical,
		boosterCatchHorizontal, boosterCatch3D,
		boosterCatchDownrange, boosterCatchCrossrange,
		boosterCatchVVel, boosterCatchHVel, boosterCatchTilt, boosterCatchRate,
		boosterCatchCrossed, boosterCatchSuccess,
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
		boosterYawCont, boosterRollCont,
		boosterRateRoll, boosterRatePitch, boosterRateYaw,
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

	// YawContinuous, RollContinuous — развёрнутые углы (см. метрики
	// rocket_booster_*_continuous_deg).
	YawContinuous, RollContinuous float64

	// BodyRollRate, BodyPitchRate, BodyYawRate — угловая скорость в
	// связанных осях, град/с.
	BodyRollRate, BodyPitchRate, BodyYawRate float64

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

	// Наведение посадочного импульса по G-FOLD. GfoldStatus — исход
	// последнего решения (см. gfoldStatusCode); NaN означает, что задача
	// ещё ни разу не решалась, и это осмысленно отличается от нуля («решение
	// найдено»), см. CLAUDE.md про неопределённые величины.
	GfoldStatus        float64
	GfoldSolves        float64
	GfoldFailures      float64
	GfoldMiss          float64
	GfoldTimeOfFlight  float64
	GfoldPlannedFuel   float64
	GfoldSlack         float64
	GfoldTrackPosition float64
	GfoldTrackVelocity float64

	// Захват башней (см. simulator/catch_tower.go). Все величины сняты в
	// момент прохождения плоскости рук, а не после падения корпуса, и до
	// этого момента равны NaN: ноль здесь означал бы «промах нулевой».
	FinTorqueRequested float64
	FinTorqueDelivered float64
	FinSaturated       bool

	GfoldShadowStatus    float64
	GfoldShadowMiss      float64
	GfoldShadowTof       float64
	GfoldShadowReachable bool

	PassiveMiss        float64
	CoastLeanDemand    float64
	CoastLeanAuthority float64
	CoastLeanApplied   float64

	CatchMissAlong      float64
	CatchMissAcross     float64
	CatchMissVertical   float64
	CatchMissHorizontal float64
	CatchMiss3D         float64
	CatchDownrange      float64
	CatchCrossrange     float64
	CatchVertical       float64
	CatchHorizontal     float64
	CatchTilt           float64
	CatchAngularRate    float64
	CatchCrossed        bool
	CatchSuccess        bool
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
	boosterYawCont.Set(s.YawContinuous)
	boosterRollCont.Set(s.RollContinuous)
	boosterRateRoll.Set(s.BodyRollRate)
	boosterRatePitch.Set(s.BodyPitchRate)
	boosterRateYaw.Set(s.BodyYawRate)
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
	boosterGfoldStatus.Set(s.GfoldStatus)
	boosterGfoldSolves.Set(s.GfoldSolves)
	boosterGfoldFailures.Set(s.GfoldFailures)
	boosterGfoldMiss.Set(s.GfoldMiss)
	boosterFinTorqueReq.Set(s.FinTorqueRequested)
	boosterFinTorqueDone.Set(s.FinTorqueDelivered)
	boosterFinSaturated.Set(boolToFloat(s.FinSaturated))
	boosterGfoldShadowStatus.Set(s.GfoldShadowStatus)
	boosterGfoldShadowMiss.Set(s.GfoldShadowMiss)
	boosterGfoldShadowTof.Set(s.GfoldShadowTof)
	boosterGfoldShadowReach.Set(boolToFloat(s.GfoldShadowReachable))
	boosterPassiveMiss.Set(s.PassiveMiss)
	boosterCoastLeanDemand.Set(s.CoastLeanDemand)
	boosterCoastLeanAuthority.Set(s.CoastLeanAuthority)
	boosterCoastLeanApplied.Set(s.CoastLeanApplied)
	boosterGfoldTof.Set(s.GfoldTimeOfFlight)
	boosterGfoldFuel.Set(s.GfoldPlannedFuel)
	boosterGfoldSlack.Set(s.GfoldSlack)
	boosterGfoldTrackPos.Set(s.GfoldTrackPosition)
	boosterGfoldTrackVel.Set(s.GfoldTrackVelocity)

	boosterCatchAlong.Set(s.CatchMissAlong)
	boosterCatchAcross.Set(s.CatchMissAcross)
	boosterCatchVertical.Set(s.CatchMissVertical)
	boosterCatchHorizontal.Set(s.CatchMissHorizontal)
	boosterCatch3D.Set(s.CatchMiss3D)
	boosterCatchDownrange.Set(s.CatchDownrange)
	boosterCatchCrossrange.Set(s.CatchCrossrange)
	boosterCatchVVel.Set(s.CatchVertical)
	boosterCatchHVel.Set(s.CatchHorizontal)
	boosterCatchTilt.Set(s.CatchTilt)
	boosterCatchRate.Set(s.CatchAngularRate)
	boosterCatchCrossed.Set(boolToFloat(s.CatchCrossed))
	boosterCatchSuccess.Set(boolToFloat(s.CatchSuccess))

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
