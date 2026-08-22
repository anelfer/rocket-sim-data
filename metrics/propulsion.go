package metrics

import "github.com/prometheus/client_golang/prometheus"

// -----------------------------------------------------------------------------
// Метрики двигательной установки.
//
// Разбиты на пять групп, соответствующих секциям дашборда: массы и компоненты,
// система подачи, турбонасосный агрегат, горение, сопло и охлаждение.
//
// Пороговые значения не дублируются здесь: они заданы в реестре параметров
// пакета propulsion и оттуда же попадают в панели Grafana.
//
// Величины, относящиеся к отдельному двигателю, размечены меткой engine_id.
// Двигателей на ступени девять, и у каждого свой турбонасос, своя камера
// и своё сопло: без метки в Grafana попадал бы только один из них, и отказ
// одного агрегата было бы не отличить от отказа всей ступени. Величины
// ступени — массы, баки, наддув, магистрали — остаются без метки: они общие.
// -----------------------------------------------------------------------------

var (
	// --- Массы и компоненты --------------------------------------------------
	massDry     = gauge("rocket_dry_mass_kg", "Сухая масса конструкции")
	massPayload = gauge("rocket_payload_mass_kg", "Масса полезной нагрузки")
	massFuel    = gauge("rocket_fuel_mass_kg", "Масса горючего")
	massOx      = gauge("rocket_oxidizer_mass_kg", "Масса окислителя")

	tankFuelLevel     = gauge("tank_fuel_level_percent", "Заполненность бака горючего")
	tankOxLevel       = gauge("tank_ox_level_percent", "Заполненность бака окислителя")
	tankFuelTemp      = gauge("tank_fuel_temperature_k", "Температура горючего")
	tankOxTemp        = gauge("tank_ox_temperature_k", "Температура окислителя")
	tankFuelDensity   = gauge("tank_fuel_density_kgm3", "Плотность горючего")
	tankOxDensity     = gauge("tank_ox_density_kgm3", "Плотность окислителя")
	tankFuelDepletion = gauge("tank_fuel_depletion_seconds", "Время до выработки горючего")
	tankOxDepletion   = gauge("tank_ox_depletion_seconds", "Время до выработки окислителя")

	// --- Система подачи ------------------------------------------------------
	tankFuelPressure = gauge("tank_fuel_pressure_pa", "Давление в баке горючего")
	tankOxPressure   = gauge("tank_ox_pressure_pa", "Давление в баке окислителя")
	tankFuelLevelM   = gauge("tank_fuel_level_meters", "Уровень горючего в баке")
	tankOxLevelM     = gauge("tank_ox_level_meters", "Уровень окислителя в баке")

	pumpFuelInlet  = gauge("pump_fuel_inlet_pressure_pa", "Давление перед насосом горючего")
	pumpOxInlet    = gauge("pump_ox_inlet_pressure_pa", "Давление перед насосом окислителя")
	pumpFuelOutlet = gaugeVec("pump_fuel_outlet_pressure_pa", "Давление за насосом горючего")
	pumpOxOutlet   = gaugeVec("pump_ox_outlet_pressure_pa", "Давление за насосом окислителя")

	flowFuel    = gaugeVec("engine_fuel_flow_kgps", "Расход горючего")
	flowOx      = gaugeVec("engine_ox_flow_kgps", "Расход окислителя")
	flowTotal   = gaugeVec("engine_total_flow_kgps", "Суммарный расход")
	mixtureGge  = gaugeVec("engine_mixture_ratio", "Соотношение компонентов O/F")
	flowGasGen  = gaugeVec("engine_gasgen_flow_kgps", "Расход через газогенератор")
	injFuelDrop = gaugeVec("injector_fuel_pressure_drop_pa", "Перепад на форсунках горючего")
	injOxDrop   = gaugeVec("injector_ox_pressure_drop_pa", "Перепад на форсунках окислителя")
	injFraction = gaugeVec("injector_drop_fraction", "Отношение перепада на форсунках к давлению камеры")

	pressFuelGas   = gauge("press_fuel_gas_pressure_pa", "Давление газа наддува горючего")
	pressOxGas     = gauge("press_ox_gas_pressure_pa", "Давление газа наддува окислителя")
	pressFuelMass  = gauge("press_fuel_gas_mass_kg", "Остаток газа наддува горючего")
	pressOxMass    = gauge("press_ox_gas_mass_kg", "Остаток газа наддува окислителя")
	pressFuelFlow  = gauge("press_fuel_flow_kgps", "Расход газа наддува горючего")
	pressOxFlow    = gauge("press_ox_flow_kgps", "Расход газа наддува окислителя")
	pressFuelValve = gauge("press_fuel_valve", "Положение клапана наддува горючего")
	pressOxValve   = gauge("press_ox_valve", "Положение клапана наддува окислителя")

	lineFuelDrop = gauge("line_fuel_pressure_drop_pa", "Потери давления в магистрали горючего")
	lineOxDrop   = gauge("line_ox_pressure_drop_pa", "Потери давления в магистрали окислителя")
	lineFuelTemp = gauge("line_fuel_temperature_k", "Температура магистрали горючего")
	lineOxTemp   = gauge("line_ox_temperature_k", "Температура магистрали окислителя")

	sloshFuel     = gauge("tank_fuel_slosh_amplitude_m", "Колебания жидкости в баке горючего")
	sloshOx       = gauge("tank_ox_slosh_amplitude_m", "Колебания жидкости в баке окислителя")
	sloshFuelCoM  = gauge("tank_fuel_com_offset_m", "Смещение центра масс горючего")
	sloshOxCoM    = gauge("tank_ox_com_offset_m", "Смещение центра масс окислителя")
	leakFuel      = gauge("tank_fuel_leak_kgps", "Утечка горючего")
	leakOx        = gauge("tank_ox_leak_kgps", "Утечка окислителя")
	drainRateFuel = gauge("tank_fuel_drain_kgps", "Скорость опустошения бака горючего")
	drainRateOx   = gauge("tank_ox_drain_kgps", "Скорость опустошения бака окислителя")

	// --- Компенсация отказа двигателя ----------------------------------------
	//
	// Величины ступени, а не двигателя: бортовой контур управляет тягой блока
	// целиком и перераспределяет её между исправными агрегатами.
	compDemand  = gauge("stage_throttle_demand", "Требуемый уровень тяги ступени")
	compPerEng  = gauge("stage_engine_throttle", "Уровень тяги на работающий двигатель")
	compOut     = gauge("stage_engines_out", "Число двигателей, не создающих тяги")
	compDeficit = gauge("stage_thrust_deficit", "Недобор тяги против требуемого")

	valveFuel   = gaugeVec("valve_fuel_position", "Положение клапана горючего")
	valveOx     = gaugeVec("valve_ox_position", "Положение клапана окислителя")
	valveGasGen = gaugeVec("valve_gasgen_position", "Положение клапана газогенератора")
	valveBypass = gaugeVec("valve_bypass_position", "Положение байпасного клапана")

	// --- Турбонасосный агрегат ------------------------------------------------
	tpRPM       = gaugeVec("turbopump_rpm", "Обороты вала ТНА")
	tpRPMTrue   = gaugeVec("turbopump_rpm_true", "Обороты вала ТНА (истинные, без шума датчика)")
	tpAngAccel  = gaugeVec("turbopump_angular_acceleration_radps2", "Угловое ускорение вала")
	tpTorque    = gaugeVec("turbopump_torque_nm", "Крутящий момент на валу")
	tpTurbineP  = gaugeVec("turbine_power_w", "Мощность турбины")
	tpFuelPumpP = gaugeVec("pump_fuel_power_w", "Мощность насоса горючего")
	tpOxPumpP   = gaugeVec("pump_ox_power_w", "Мощность насоса окислителя")
	tpLossP     = gaugeVec("turbopump_mechanical_loss_w", "Механические потери ТНА")
	tpMarginP   = gaugeVec("turbopump_power_margin_w", "Запас мощности турбины")

	tpFuelCav  = gaugeVec("pump_fuel_cavitation_margin_m", "Кавитационный запас насоса горючего")
	tpOxCav    = gaugeVec("pump_ox_cavitation_margin_m", "Кавитационный запас насоса окислителя")
	tpVibr     = gaugeVec("turbopump_vibration_g", "Вибрация подшипников")
	tpBearingT = gaugeVec("turbopump_bearing_temperature_k", "Температура подшипников")
	tpAxial    = gaugeVec("turbopump_axial_displacement_mm", "Осевое смещение ротора")
	tpTurbineT = gaugeVec("turbine_temperature_k", "Температура на входе турбины")

	// --- Горение --------------------------------------------------------------
	combPc         = gaugeVec("chamber_pressure_pa", "Давление в камере сгорания")
	combPcTrue     = gaugeVec("chamber_pressure_true_pa", "Давление в камере сгорания (истинное, без шума датчика)")
	combPcMean     = gaugeVec("chamber_pressure_mean_pa", "Среднее давление в камере")
	combCstar      = gaugeVec("combustion_cstar_mps", "Характеристическая скорость c*")
	combCstarEff   = gaugeVec("combustion_cstar_efficiency", "Полнота сгорания c*")
	combEff        = gaugeVec("combustion_efficiency", "Эффективность горения")
	combCf         = gaugeVec("nozzle_thrust_coefficient", "Коэффициент тяги Cf")
	combTemp       = gaugeVec("chamber_temperature_k", "Температура в камере")
	combMixDev     = gaugeVec("combustion_mixture_deviation", "Отклонение O/F от номинала")
	combRMS        = gaugeVec("combustion_pressure_rms_pa", "Среднеквадратичные пульсации давления")
	combPeak       = gaugeVec("combustion_pressure_peak_pa", "Амплитуда пульсаций давления")
	combFreq       = gaugeVec("combustion_dominant_frequency_hz", "Доминирующая частота пульсаций")
	combMargin     = gaugeVec("combustion_stability_margin", "Запас устойчивости горения")
	combState      = gaugeVec("combustion_state", "Состояние горения")
	combNonuniform = gaugeVec("chamber_temperature_nonuniformity", "Температурная неоднородность камеры")

	// Спектр пульсаций давления по частотным полосам.
	combSpectrum = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "chamber_pressure_spectrum_pa",
			Help: "Амплитудный спектр пульсаций давления в камере по частотным полосам",
		},
		[]string{"engine_id", "frequency_hz"},
	)

	// --- Сопло и охлаждение ---------------------------------------------------
	nozHeatFlux = gaugeVec("chamber_wall_heat_flux_w_m2", "Тепловой поток в стенку камеры")
	nozMargin   = gaugeVec("nozzle_wall_margin_k", "Запас до критической температуры")
	nozThroat   = gaugeVec("nozzle_throat_area_m2", "Площадь критического сечения")
	nozErosion  = gaugeVec("nozzle_throat_erosion_mm", "Эрозия критического сечения")
	nozExpRatio = gaugeVec("nozzle_expansion_ratio", "Степень расширения сопла")
	nozExitP    = gaugeVec("nozzle_exit_pressure_pa", "Давление на срезе сопла")
	nozExhaustV = gaugeVec("nozzle_exhaust_velocity_mps", "Скорость истечения")
	nozRegime   = gaugeVec("nozzle_expansion_regime", "Режим расширения струи")

	coolFlow   = gaugeVec("cooling_flow_kgps", "Расход в охлаждающем тракте")
	coolInletT = gaugeVec("cooling_inlet_temperature_k", "Температура охладителя на входе")
	coolOutT   = gaugeVec("cooling_outlet_temperature_k", "Температура охладителя на выходе")
	coolDrop   = gaugeVec("cooling_pressure_drop_pa", "Перепад давления в охлаждающем тракте")
	coolEff    = gaugeVec("cooling_effectiveness", "Эффективность охлаждения")

	// Температура и поток по участкам стенки.
	wallStationTemp = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "nozzle_wall_temperature_k",
			Help: "Температура стенки по участкам тракта",
		},
		[]string{"engine_id", "station"},
	)
	wallStationFlux = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "nozzle_wall_heat_flux_w_m2",
			Help: "Тепловой поток по участкам тракта",
		},
		[]string{"engine_id", "station"},
	)
	wallStationMargin = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Name: "nozzle_wall_station_margin_k",
			Help: "Запас до предельной температуры по участкам тракта",
		},
		[]string{"engine_id", "station"},
	)
)

// CompensationSample — состояние компенсации отказа двигателя.
type CompensationSample struct {
	ThrottleDemand float64
	EngineThrottle float64
	EnginesOut     float64
	ThrustDeficit  float64
}

// SetThrustCompensation публикует состояние компенсации отказа двигателя.
func SetThrustCompensation(s CompensationSample) {
	ensureRegistered()

	compDemand.Set(s.ThrottleDemand)
	compPerEng.Set(s.EngineThrottle)
	compOut.Set(s.EnginesOut)
	compDeficit.Set(s.ThrustDeficit)
}

// retirePropulsion снимает подробные метрики отделившегося двигателя.
func retirePropulsion(labels prometheus.Labels) {
	for _, v := range enginePropulsionVecs() {
		v.Delete(labels)
	}
	// Спектр и участки стенки размечены дополнительной меткой, поэтому
	// удаляются по частичному совпадению.
	combSpectrum.DeletePartialMatch(labels)
	wallStationTemp.DeletePartialMatch(labels)
	wallStationFlux.DeletePartialMatch(labels)
	wallStationMargin.DeletePartialMatch(labels)
}

// enginePropulsionVecs возвращает метрики, относящиеся к отдельному двигателю.
func enginePropulsionVecs() []*prometheus.GaugeVec {
	return []*prometheus.GaugeVec{
		pumpFuelOutlet, pumpOxOutlet,
		flowFuel, flowOx, flowTotal, mixtureGge, flowGasGen,
		injFuelDrop, injOxDrop, injFraction,
		valveFuel, valveOx, valveGasGen, valveBypass,
		tpRPM, tpRPMTrue, tpAngAccel, tpTorque, tpTurbineP, tpFuelPumpP, tpOxPumpP,
		tpLossP, tpMarginP, tpFuelCav, tpOxCav, tpVibr, tpBearingT,
		tpAxial, tpTurbineT,
		combPc, combPcTrue, combPcMean, combCstar, combCstarEff, combEff, combCf,
		combTemp, combMixDev, combRMS, combPeak, combFreq, combMargin,
		combState, combNonuniform,
		nozHeatFlux, nozMargin, nozThroat, nozErosion, nozExpRatio,
		nozExitP, nozExhaustV, nozRegime,
		coolFlow, coolInletT, coolOutT, coolDrop, coolEff,
	}
}

// propulsionCollectors возвращает метрики двигательной установки.
func propulsionCollectors() []prometheus.Collector {
	return []prometheus.Collector{
		massDry, massPayload, massFuel, massOx,
		tankFuelLevel, tankOxLevel, tankFuelTemp, tankOxTemp,
		tankFuelDensity, tankOxDensity, tankFuelDepletion, tankOxDepletion,

		tankFuelPressure, tankOxPressure, tankFuelLevelM, tankOxLevelM,
		pumpFuelInlet, pumpOxInlet, pumpFuelOutlet, pumpOxOutlet,
		flowFuel, flowOx, flowTotal, mixtureGge, flowGasGen,
		injFuelDrop, injOxDrop, injFraction,
		pressFuelGas, pressOxGas, pressFuelMass, pressOxMass,
		pressFuelFlow, pressOxFlow, pressFuelValve, pressOxValve,
		lineFuelDrop, lineOxDrop, lineFuelTemp, lineOxTemp,
		sloshFuel, sloshOx, sloshFuelCoM, sloshOxCoM,
		leakFuel, leakOx, drainRateFuel, drainRateOx,
		compDemand, compPerEng, compOut, compDeficit,
		valveFuel, valveOx, valveGasGen, valveBypass,

		tpRPM, tpRPMTrue, tpAngAccel, tpTorque, tpTurbineP, tpFuelPumpP, tpOxPumpP,
		tpLossP, tpMarginP, tpFuelCav, tpOxCav, tpVibr, tpBearingT,
		tpAxial, tpTurbineT,

		combPc, combPcTrue, combPcMean, combCstar, combCstarEff, combEff, combCf,
		combTemp, combMixDev, combRMS, combPeak, combFreq, combMargin,
		combState, combNonuniform, combSpectrum,

		nozHeatFlux, nozMargin, nozThroat, nozErosion, nozExpRatio,
		nozExitP, nozExhaustV, nozRegime,
		coolFlow, coolInletT, coolOutT, coolDrop, coolEff,
		wallStationTemp, wallStationFlux, wallStationMargin,
	}
}

// MassSample — состав массы ракеты.
type MassSample struct {
	Dry      float64
	Payload  float64
	Fuel     float64
	Oxidizer float64
}

// SetMassBreakdown публикует состав массы.
func SetMassBreakdown(s MassSample) {
	ensureRegistered()
	massDry.Set(s.Dry)
	massPayload.Set(s.Payload)
	massFuel.Set(s.Fuel)
	massOx.Set(s.Oxidizer)
}

// TankSample — состояние одного бака.
type TankSample struct {
	Mass           float64
	FillFraction   float64
	Pressure       float64
	Temperature    float64
	Density        float64
	Level          float64
	DrainRate      float64
	DepletionTime  float64
	OutletPressure float64

	SloshAmplitude     float64
	CenterOfMassOffset float64
	LineDrop           float64
	LineTemperature    float64
	LeakRate           float64

	PressurantMass     float64
	PressurantPressure float64
	PressurantFlow     float64
	PressurantValve    float64
}

// SetFuelTank публикует состояние бака горючего.
func SetFuelTank(s TankSample) {
	ensureRegistered()
	tankFuelLevel.Set(s.FillFraction * 100)
	tankFuelTemp.Set(s.Temperature)
	tankFuelDensity.Set(s.Density)
	tankFuelDepletion.Set(s.DepletionTime)
	tankFuelPressure.Set(s.Pressure)
	tankFuelLevelM.Set(s.Level)
	pumpFuelInlet.Set(s.OutletPressure)
	lineFuelDrop.Set(s.LineDrop)
	lineFuelTemp.Set(s.LineTemperature)
	sloshFuel.Set(s.SloshAmplitude)
	sloshFuelCoM.Set(s.CenterOfMassOffset)
	leakFuel.Set(s.LeakRate)
	drainRateFuel.Set(s.DrainRate)
	pressFuelGas.Set(s.PressurantPressure)
	pressFuelMass.Set(s.PressurantMass)
	pressFuelFlow.Set(s.PressurantFlow)
	pressFuelValve.Set(s.PressurantValve)
}

// SetOxTank публикует состояние бака окислителя.
func SetOxTank(s TankSample) {
	ensureRegistered()
	tankOxLevel.Set(s.FillFraction * 100)
	tankOxTemp.Set(s.Temperature)
	tankOxDensity.Set(s.Density)
	tankOxDepletion.Set(s.DepletionTime)
	tankOxPressure.Set(s.Pressure)
	tankOxLevelM.Set(s.Level)
	pumpOxInlet.Set(s.OutletPressure)
	lineOxDrop.Set(s.LineDrop)
	lineOxTemp.Set(s.LineTemperature)
	sloshOx.Set(s.SloshAmplitude)
	sloshOxCoM.Set(s.CenterOfMassOffset)
	leakOx.Set(s.LeakRate)
	drainRateOx.Set(s.DrainRate)
	pressOxGas.Set(s.PressurantPressure)
	pressOxMass.Set(s.PressurantMass)
	pressOxFlow.Set(s.PressurantFlow)
	pressOxValve.Set(s.PressurantValve)
}

// EngineDetailSample — подробное состояние двигателя.
type EngineDetailSample struct {
	FuelFlow             float64
	OxFlow               float64
	TotalFlow            float64
	GasGeneratorFlow     float64
	MixtureRatio         float64
	FuelPumpOutlet       float64
	OxPumpOutlet         float64
	FuelInjectorDrop     float64
	OxInjectorDrop       float64
	InjectorDropFraction float64

	ShaftRPM            float64
	ShaftRPMTrue        float64
	AngularAcceleration float64
	Torque              float64
	TurbinePower        float64
	FuelPumpPower       float64
	OxPumpPower         float64
	MechanicalLoss      float64
	PowerMargin         float64

	FuelCavitationMargin float64
	OxCavitationMargin   float64
	Vibration            float64
	BearingTemperature   float64
	AxialDisplacement    float64
	TurbineTemperature   float64

	FuelValve   float64
	OxValve     float64
	GasGenValve float64
	BypassValve float64

	ChamberPressure     float64
	ChamberPressureTrue float64
	ChamberPressureMean float64
	CharacteristicSpeed float64
	CStarEfficiency     float64
	CombustionEff       float64
	ThrustCoefficient   float64
	ChamberTemperature  float64
	MixtureDeviation    float64
	PressureRMS         float64
	PressurePeak        float64
	DominantFrequency   float64
	StabilityMargin     float64
	CombustionState     int
	TemperatureSpread   float64

	WallHeatFlux    float64
	WallMargin      float64
	ThroatArea      float64
	ThroatErosion   float64
	ExpansionRatio  float64
	ExitPressure    float64
	ExhaustVelocity float64
	ExpansionRegime float64

	CoolingFlow       float64
	CoolantInlet      float64
	CoolantOutlet     float64
	CoolingDrop       float64
	CoolingEffective  float64
	WallStationName   []string
	WallStationTemp   []float64
	WallStationFlux   []float64
	WallStationMargin []float64

	SpectrumFrequency []string
	SpectrumAmplitude []float64
}

// SetEngineDetail публикует подробное состояние двигателя.
func SetEngineDetail(engineID string, s EngineDetailSample) {
	ensureRegistered()

	if engineID == "" {
		engineID = "unknown"
	}

	flowFuel.WithLabelValues(engineID).Set(s.FuelFlow)
	flowOx.WithLabelValues(engineID).Set(s.OxFlow)
	flowTotal.WithLabelValues(engineID).Set(s.TotalFlow)
	flowGasGen.WithLabelValues(engineID).Set(s.GasGeneratorFlow)
	mixtureGge.WithLabelValues(engineID).Set(s.MixtureRatio)
	pumpFuelOutlet.WithLabelValues(engineID).Set(s.FuelPumpOutlet)
	pumpOxOutlet.WithLabelValues(engineID).Set(s.OxPumpOutlet)
	injFuelDrop.WithLabelValues(engineID).Set(s.FuelInjectorDrop)
	injOxDrop.WithLabelValues(engineID).Set(s.OxInjectorDrop)
	injFraction.WithLabelValues(engineID).Set(s.InjectorDropFraction)

	tpRPM.WithLabelValues(engineID).Set(s.ShaftRPM)
	tpRPMTrue.WithLabelValues(engineID).Set(s.ShaftRPMTrue)
	tpAngAccel.WithLabelValues(engineID).Set(s.AngularAcceleration)
	tpTorque.WithLabelValues(engineID).Set(s.Torque)
	tpTurbineP.WithLabelValues(engineID).Set(s.TurbinePower)
	tpFuelPumpP.WithLabelValues(engineID).Set(s.FuelPumpPower)
	tpOxPumpP.WithLabelValues(engineID).Set(s.OxPumpPower)
	tpLossP.WithLabelValues(engineID).Set(s.MechanicalLoss)
	tpMarginP.WithLabelValues(engineID).Set(s.PowerMargin)
	tpFuelCav.WithLabelValues(engineID).Set(s.FuelCavitationMargin)
	tpOxCav.WithLabelValues(engineID).Set(s.OxCavitationMargin)
	tpVibr.WithLabelValues(engineID).Set(s.Vibration)
	tpBearingT.WithLabelValues(engineID).Set(s.BearingTemperature)
	tpAxial.WithLabelValues(engineID).Set(s.AxialDisplacement)
	tpTurbineT.WithLabelValues(engineID).Set(s.TurbineTemperature)

	valveFuel.WithLabelValues(engineID).Set(s.FuelValve)
	valveOx.WithLabelValues(engineID).Set(s.OxValve)
	valveGasGen.WithLabelValues(engineID).Set(s.GasGenValve)
	valveBypass.WithLabelValues(engineID).Set(s.BypassValve)

	combPc.WithLabelValues(engineID).Set(s.ChamberPressure)
	combPcTrue.WithLabelValues(engineID).Set(s.ChamberPressureTrue)
	combPcMean.WithLabelValues(engineID).Set(s.ChamberPressureMean)
	combCstar.WithLabelValues(engineID).Set(s.CharacteristicSpeed)
	combCstarEff.WithLabelValues(engineID).Set(s.CStarEfficiency)
	combEff.WithLabelValues(engineID).Set(s.CombustionEff)
	combCf.WithLabelValues(engineID).Set(s.ThrustCoefficient)
	combTemp.WithLabelValues(engineID).Set(s.ChamberTemperature)
	combMixDev.WithLabelValues(engineID).Set(s.MixtureDeviation)
	combRMS.WithLabelValues(engineID).Set(s.PressureRMS)
	combPeak.WithLabelValues(engineID).Set(s.PressurePeak)
	combFreq.WithLabelValues(engineID).Set(s.DominantFrequency)
	combMargin.WithLabelValues(engineID).Set(s.StabilityMargin)
	combState.WithLabelValues(engineID).Set(float64(s.CombustionState))
	combNonuniform.WithLabelValues(engineID).Set(s.TemperatureSpread)

	nozHeatFlux.WithLabelValues(engineID).Set(s.WallHeatFlux)
	nozMargin.WithLabelValues(engineID).Set(s.WallMargin)
	nozThroat.WithLabelValues(engineID).Set(s.ThroatArea)
	nozErosion.WithLabelValues(engineID).Set(s.ThroatErosion)
	nozExpRatio.WithLabelValues(engineID).Set(s.ExpansionRatio)
	nozExitP.WithLabelValues(engineID).Set(s.ExitPressure)
	nozExhaustV.WithLabelValues(engineID).Set(s.ExhaustVelocity)
	nozRegime.WithLabelValues(engineID).Set(s.ExpansionRegime)

	coolFlow.WithLabelValues(engineID).Set(s.CoolingFlow)
	coolInletT.WithLabelValues(engineID).Set(s.CoolantInlet)
	coolOutT.WithLabelValues(engineID).Set(s.CoolantOutlet)
	coolDrop.WithLabelValues(engineID).Set(s.CoolingDrop)
	coolEff.WithLabelValues(engineID).Set(s.CoolingEffective)

	for i, name := range s.WallStationName {
		if i < len(s.WallStationTemp) {
			wallStationTemp.WithLabelValues(engineID, name).Set(s.WallStationTemp[i])
		}
		if i < len(s.WallStationFlux) {
			wallStationFlux.WithLabelValues(engineID, name).Set(s.WallStationFlux[i])
		}
		if i < len(s.WallStationMargin) {
			wallStationMargin.WithLabelValues(engineID, name).Set(s.WallStationMargin[i])
		}
	}

	for i, f := range s.SpectrumFrequency {
		if i < len(s.SpectrumAmplitude) {
			combSpectrum.WithLabelValues(engineID, f).Set(s.SpectrumAmplitude[i])
		}
	}
}
