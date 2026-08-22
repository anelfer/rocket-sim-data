package simulator

import (
	"math"
	"strconv"

	"rocketTelemetrySim/metrics"
	"rocketTelemetrySim/simulator/propulsion"
)

// -----------------------------------------------------------------------------
// Публикация телеметрии двигательной установки.
//
// Снимок собирается один раз за такт и содержит согласованный набор величин:
// расход, давление в камере, тяга и удельный импульс получены из одного
// решения, а не сгенерированы независимо.
// -----------------------------------------------------------------------------

// PropulsionTelemetry — снимок состояния двигательной установки.
type PropulsionTelemetry struct {
	// Состав массы.
	StructureMass float64 `json:"structureMass"` // кг
	PayloadMass   float64 `json:"payloadMass"`   // кг
	FuelMass      float64 `json:"fuelMass"`      // кг
	OxidizerMass  float64 `json:"oxidizerMass"`  // кг

	FuelTank propulsion.TankTelemetry `json:"fuelTank"`
	OxTank   propulsion.TankTelemetry `json:"oxTank"`

	// Суммарные показатели.
	TotalThrust     float64 `json:"totalThrust"`     // Н
	SpecificImpulse float64 `json:"specificImpulse"` // с
	TotalFuelFlow   float64 `json:"totalFuelFlow"`   // кг/с
	TotalOxFlow     float64 `json:"totalOxFlow"`     // кг/с
	RunningEngines  int     `json:"runningEngines"`

	// Подробное состояние ведущего двигателя.
	Engine propulsion.EngineTelemetry `json:"engine"`

	// Engines — состояние каждого двигателя блока.
	//
	// Двигателей девять, и каждый со своим турбонасосом: без этого списка
	// отказ одного агрегата был бы неотличим от отказа всей ступени.
	// Спектр пульсаций считается только по ведущему — преобразование Фурье
	// для девяти двигателей на каждом такте стоило бы дороже всей остальной
	// телеметрии вместе взятой.
	Engines []propulsion.EngineTelemetry `json:"engines"`

	// Компенсация отказа двигателя.
	ThrottleDemand float64 `json:"throttleDemand"` // требуется от ступени
	EngineThrottle float64 `json:"engineThrottle"` // подано на двигатель
	EnginesOut     int     `json:"enginesOut"`
	ThrustDeficit  float64 `json:"thrustDeficit"` // недобор тяги, доля

	// Невязка баланса масс, кг. Должна оставаться нулевой.
	MassBalanceError float64 `json:"massBalanceError"`
}

// buildPropulsionTelemetry собирает снимок. Вызывается под удержанным mu.
func (s *Simulation) buildPropulsionTelemetry() PropulsionTelemetry {
	sys := s.propulsion
	if sys == nil {
		return PropulsionTelemetry{}
	}

	t := PropulsionTelemetry{
		StructureMass:    sys.StructureMass,
		PayloadMass:      sys.PayloadMass,
		FuelMass:         sys.FuelTank.Mass,
		OxidizerMass:     sys.OxTank.Mass,
		TotalThrust:      sys.TotalThrust,
		SpecificImpulse:  sys.SpecificImpulse,
		TotalFuelFlow:    sys.TotalFuelFlow,
		TotalOxFlow:      sys.TotalOxFlow,
		RunningEngines:   sys.RunningEngines(),
		ThrottleDemand:   sys.ThrottleDemand,
		EngineThrottle:   sys.EngineThrottle,
		EnginesOut:       sys.EnginesOut,
		ThrustDeficit:    sys.ThrustDeficit,
		MassBalanceError: sys.MassBalanceError(),
	}

	t.FuelTank = sys.FuelTank.Telemetry()
	t.OxTank = sys.OxTank.Telemetry()
	t.FuelTank.OutletPressure = sys.FuelTank.OutletPressure(s.prevAxialAccel)
	t.OxTank.OutletPressure = sys.OxTank.OutletPressure(s.prevAxialAccel)

	// Шаг для датчиков — модельное время между сборками телеметрии.
	//
	// Датчики запаздывают, дрейфуют и шумят, и всё это отсчитывается
	// по модельному времени, а не по времени работы программы. На ускоренном
	// прогоне за такт проходит несколько секунд модели, и брать здесь период
	// такта значило бы замедлять приборы во столько же раз.
	dt := s.Time.TickInterval.Seconds() * math.Max(s.Time.Scale, 0.01)

	t.Engines = make([]propulsion.EngineTelemetry, 0, len(sys.Engines))
	for _, e := range sys.Engines {
		t.Engines = append(t.Engines, e.Telemetry(dt, e.Detailed))
	}
	if e := sys.PrimaryEngine(); e != nil {
		for i := range t.Engines {
			if t.Engines[i].ID == e.ID {
				t.Engine = t.Engines[i]
				break
			}
		}
	}

	return t
}

// Publish отправляет снимок двигательной установки в Prometheus.
func (t PropulsionTelemetry) Publish() {
	metrics.SetMassBreakdown(metrics.MassSample{
		Dry:      t.StructureMass,
		Payload:  t.PayloadMass,
		Fuel:     t.FuelMass,
		Oxidizer: t.OxidizerMass,
	})

	metrics.SetThrustCompensation(metrics.CompensationSample{
		ThrottleDemand: t.ThrottleDemand,
		EngineThrottle: t.EngineThrottle,
		EnginesOut:     float64(t.EnginesOut),
		ThrustDeficit:  t.ThrustDeficit,
	})

	metrics.SetFuelTank(tankSample(t.FuelTank))
	metrics.SetOxTank(tankSample(t.OxTank))

	// Метрики публикуются по каждому двигателю: у каждого свой турбонасос,
	// своя камера и своё сопло, и в Grafana они обязаны быть различимы.
	for _, e := range t.Engines {
		metrics.SetEngineDetail(e.ID, engineSample(e))
	}
}

// engineSample переводит телеметрию двигателя в набор метрик.
func engineSample(e propulsion.EngineTelemetry) metrics.EngineDetailSample {
	sample := metrics.EngineDetailSample{
		FuelFlow:             e.FuelFlow.Value(),
		OxFlow:               e.OxFlow.Value(),
		TotalFlow:            e.TotalFlow,
		GasGeneratorFlow:     e.GasGeneratorFlow,
		MixtureRatio:         e.MixtureRatio,
		FuelPumpOutlet:       e.FuelPumpOutlet.Value(),
		OxPumpOutlet:         e.OxPumpOutlet.Value(),
		FuelInjectorDrop:     e.FuelInjectorDrop,
		OxInjectorDrop:       e.OxInjectorDrop,
		InjectorDropFraction: e.InjectorDropFraction,

		ShaftRPM:            e.ShaftRPM.Value(),
		AngularAcceleration: e.AngularAcceleration,
		Torque:              e.ShaftTorque,
		TurbinePower:        e.TurbinePower,
		FuelPumpPower:       e.FuelPumpPower,
		OxPumpPower:         e.OxPumpPower,
		MechanicalLoss:      e.MechanicalLoss,
		PowerMargin:         e.PowerMargin,

		FuelCavitationMargin: e.FuelCavitationMargin,
		OxCavitationMargin:   e.OxCavitationMargin,
		Vibration:            e.Vibration.Value(),
		BearingTemperature:   e.BearingTemperature.Value(),
		AxialDisplacement:    e.AxialDisplacement,
		TurbineTemperature:   e.TurbineTemperature.Value(),

		FuelValve:   e.FuelValve,
		OxValve:     e.OxValve,
		GasGenValve: e.GasGenValve,
		BypassValve: e.BypassValve,

		ChamberPressure:     e.ChamberPressure.Value(),
		ChamberPressureMean: e.ChamberPressureMean,
		CharacteristicSpeed: e.CharacteristicVelocity,
		CStarEfficiency:     e.CStarEfficiency,
		CombustionEff:       e.CombustionEfficiency,
		ThrustCoefficient:   e.ThrustCoefficient,
		ChamberTemperature:  e.ChamberTemperature,
		MixtureDeviation:    e.MixtureDeviation,
		PressureRMS:         e.PressureRMS,
		PressurePeak:        e.PressurePeak,
		DominantFrequency:   e.DominantFrequency,
		StabilityMargin:     e.StabilityMargin,
		CombustionState:     e.CombustionStateCode,
		TemperatureSpread:   e.TemperatureSpread,

		WallHeatFlux:    e.WallHeatFlux,
		WallMargin:      e.WallMargin,
		ThroatArea:      e.ThroatArea,
		ThroatErosion:   e.ThroatErosion,
		ExpansionRatio:  e.ExpansionRatio,
		ExitPressure:    e.ExitPressure,
		ExhaustVelocity: e.ExhaustVelocity,
		ExpansionRegime: e.ExpansionRegime.Value(),

		CoolingFlow:      e.CoolingFlow,
		CoolantInlet:     e.CoolantInlet,
		CoolantOutlet:    e.CoolantOutlet.Value(),
		CoolingDrop:      e.CoolingDrop,
		CoolingEffective: e.CoolingEffective,
	}

	for _, w := range e.WallStations {
		sample.WallStationName = append(sample.WallStationName, w.Name)
		sample.WallStationTemp = append(sample.WallStationTemp, w.Temperature)
		sample.WallStationFlux = append(sample.WallStationFlux, w.HeatFlux)
		sample.WallStationMargin = append(sample.WallStationMargin, w.Margin)
	}

	// Спектр публикуется полосами: метка — центральная частота полосы.
	// Так спектральную панель можно построить обычным графиком по метке.
	for _, b := range e.Spectrum {
		sample.SpectrumFrequency = append(sample.SpectrumFrequency,
			strconv.FormatFloat(b.Frequency, 'f', 1, 64))
		sample.SpectrumAmplitude = append(sample.SpectrumAmplitude, b.Amplitude)
	}

	return sample
}

func tankSample(t propulsion.TankTelemetry) metrics.TankSample {
	return metrics.TankSample{
		Mass:               t.Mass,
		FillFraction:       t.FillFraction,
		Pressure:           t.Pressure,
		Temperature:        t.Temperature,
		Density:            t.Density,
		Level:              t.Level,
		DrainRate:          t.DrainRate,
		DepletionTime:      t.DepletionTime.Value(),
		OutletPressure:     t.OutletPressure,
		SloshAmplitude:     t.SloshAmplitude,
		CenterOfMassOffset: t.CenterOfMassOffset,
		LineDrop:           t.LineDrop,
		LineTemperature:    t.LineTemperature,
		LeakRate:           t.LeakRate,
		PressurantMass:     t.PressurantMass,
		PressurantPressure: t.PressurantPressure,
		PressurantFlow:     t.PressurantFlow,
		PressurantValve:    t.PressurantValve,
	}
}
