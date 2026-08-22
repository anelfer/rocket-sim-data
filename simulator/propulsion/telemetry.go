package propulsion

import (
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Телеметрия двигательной установки.
//
// Снимок собирается один раз за такт публикации. Показания проходят через
// модель датчиков — с шумом, дрейфом и запаздыванием, — но сами уравнения
// считаются по истинным величинам, поэтому связанные параметры остаются
// согласованными между собой.
// -----------------------------------------------------------------------------

// EngineTelemetry — состояние одного двигателя.
type EngineTelemetry struct {
	ID      string `json:"id"`
	Name    string `json:"name"`
	Running bool   `json:"running"`

	// Тяга и расходы.
	Thrust          float64       `json:"thrust"`          // Н
	SpecificImpulse float64       `json:"specificImpulse"` // с
	FuelFlow        physics.Float `json:"fuelFlow"`        // кг/с
	OxFlow          physics.Float `json:"oxFlow"`          // кг/с
	TotalFlow       float64       `json:"totalFlow"`       // кг/с
	MixtureRatio    float64       `json:"mixtureRatio"`

	GasGeneratorFlow   float64 `json:"gasGeneratorFlow"`   // кг/с
	GasGeneratorThrust float64 `json:"gasGeneratorThrust"` // Н

	// Давления.
	ChamberPressure      physics.Float `json:"chamberPressure"`      // Па, с пульсациями
	ChamberPressureMean  float64       `json:"chamberPressureMean"`  // Па
	FuelPumpInlet        physics.Float `json:"fuelPumpInlet"`        // Па
	FuelPumpOutlet       physics.Float `json:"fuelPumpOutlet"`       // Па
	OxPumpInlet          physics.Float `json:"oxPumpInlet"`          // Па
	OxPumpOutlet         physics.Float `json:"oxPumpOutlet"`         // Па
	FuelInjectorDrop     float64       `json:"fuelInjectorDrop"`     // Па
	OxInjectorDrop       float64       `json:"oxInjectorDrop"`       // Па
	InjectorDropFraction float64       `json:"injectorDropFraction"` // доля от Pc

	// Турбонасосный агрегат.
	ShaftRPM            physics.Float `json:"shaftRpm"`
	AngularAcceleration float64       `json:"angularAcceleration"` // рад/с²
	TurbinePower        float64       `json:"turbinePower"`        // Вт
	FuelPumpPower       float64       `json:"fuelPumpPower"`       // Вт
	OxPumpPower         float64       `json:"oxPumpPower"`         // Вт
	MechanicalLoss      float64       `json:"mechanicalLoss"`      // Вт
	ShaftTorque         float64       `json:"shaftTorque"`         // Н·м
	PowerMargin         float64       `json:"powerMargin"`         // Вт

	FuelCavitationMargin float64       `json:"fuelCavitationMargin"` // м
	OxCavitationMargin   float64       `json:"oxCavitationMargin"`   // м
	Vibration            physics.Float `json:"vibration"`            // g
	BearingTemperature   physics.Float `json:"bearingTemperature"`   // К
	AxialDisplacement    float64       `json:"axialDisplacement"`    // мм
	TurbineTemperature   physics.Float `json:"turbineTemperature"`   // К

	// Клапаны.
	FuelValve   float64 `json:"fuelValve"`
	OxValve     float64 `json:"oxValve"`
	GasGenValve float64 `json:"gasGenValve"`
	BypassValve float64 `json:"bypassValve"`

	// Горение.
	CharacteristicVelocity float64 `json:"characteristicVelocity"` // м/с
	CStarEfficiency        float64 `json:"cStarEfficiency"`
	CombustionEfficiency   float64 `json:"combustionEfficiency"`
	ThrustCoefficient      float64 `json:"thrustCoefficient"`
	ChamberTemperature     float64 `json:"chamberTemperature"` // К
	MixtureDeviation       float64 `json:"mixtureDeviation"`
	TemperatureSpread      float64 `json:"temperatureSpread"`
	PressureRMS            float64 `json:"pressureRms"`  // Па
	PressurePeak           float64 `json:"pressurePeak"` // Па
	DominantFrequency      float64 `json:"dominantFrequency"`
	StabilityMargin        float64 `json:"stabilityMargin"`
	CombustionState        string  `json:"combustionState"`
	CombustionStateCode    int     `json:"combustionStateCode"`

	// Сопло и охлаждение.
	ThroatArea       float64       `json:"throatArea"`      // м²
	ThroatErosion    float64       `json:"throatErosion"`   // мм
	ExpansionRatio   float64       `json:"expansionRatio"`  //
	ExitPressure     float64       `json:"exitPressure"`    // Па
	ExhaustVelocity  float64       `json:"exhaustVelocity"` // м/с
	ExpansionRegime  physics.Float `json:"expansionRegime"` // p_срез/p_забортное
	WallHeatFlux     float64       `json:"wallHeatFlux"`    // Вт/м²
	WallMargin       float64       `json:"wallMargin"`      // К
	WallDamage       float64       `json:"wallDamage"`      // 0…1
	BurnedThrough    bool          `json:"burnedThrough"`
	CoolingFlow      float64       `json:"coolingFlow"`      // кг/с
	CoolantInlet     float64       `json:"coolantInlet"`     // К
	CoolantOutlet    physics.Float `json:"coolantOutlet"`    // К
	CoolingDrop      float64       `json:"coolingDrop"`      // Па
	CoolingEffective float64       `json:"coolingEffective"` //

	// Стенки по участкам.
	WallStations []WallStationTelemetry `json:"wallStations"`

	// Спектр пульсаций давления. Заполняется только для двигателя,
	// по которому ведётся подробный анализ.
	Spectrum []SpectrumBin `json:"spectrum,omitempty"`
}

// WallStationTelemetry — состояние участка стенки.
type WallStationTelemetry struct {
	Name        string  `json:"name"`
	Temperature float64 `json:"temperature"` // К
	HeatFlux    float64 `json:"heatFlux"`    // Вт/м²
	Margin      float64 `json:"margin"`      // К до предела материала
}

// TankTelemetry — состояние бака.
type TankTelemetry struct {
	Name string `json:"name"`

	Mass           float64       `json:"mass"`           // кг
	FillFraction   float64       `json:"fillFraction"`   // 0…1
	Pressure       float64       `json:"pressure"`       // Па
	Temperature    float64       `json:"temperature"`    // К
	Density        float64       `json:"density"`        // кг/м³
	Level          float64       `json:"level"`          // м
	DrainRate      float64       `json:"drainRate"`      // кг/с
	DepletionTime  physics.Float `json:"depletionTime"`  // с
	OutletPressure float64       `json:"outletPressure"` // Па

	SloshAmplitude     float64 `json:"sloshAmplitude"`     // м
	CenterOfMassOffset float64 `json:"centerOfMassOffset"` // м
	LineDrop           float64 `json:"lineDrop"`           // Па
	LineTemperature    float64 `json:"lineTemperature"`    // К
	LeakRate           float64 `json:"leakRate"`           // кг/с

	PressurantMass     float64 `json:"pressurantMass"`     // кг
	PressurantPressure float64 `json:"pressurantPressure"` // Па
	PressurantFlow     float64 `json:"pressurantFlow"`     // кг/с
	PressurantValve    float64 `json:"pressurantValve"`    // 0…1
}

// Telemetry собирает снимок состояния двигателя.
//
// dt нужен для модели датчиков: она вносит запаздывание и ограничивает
// частоту обновления показаний.
func (e *Engine) Telemetry(dt float64, withSpectrum bool) EngineTelemetry {
	tp := e.Turbopump
	ch := e.Chamber
	nz := e.Nozzle

	if withSpectrum && e.Detailed {
		ch.UpdateSpectrum()
	}

	dropFraction := 0.0
	if ch.MeanPressure > 0 {
		dropFraction = math.Min(ch.FuelInjectorDrop, ch.OxInjectorDrop) / ch.MeanPressure
	}

	t := EngineTelemetry{
		ID:      e.ID,
		Name:    e.Config.Name,
		Running: e.Running,

		Thrust:          e.Thrust,
		SpecificImpulse: e.SpecificImpulse,
		FuelFlow:        physics.Float(e.FuelFlow),
		OxFlow:          physics.Float(e.OxFlow),
		TotalFlow:       e.TotalFlow(),
		MixtureRatio:    ch.MixtureRatio,

		GasGeneratorFlow:   e.GasGeneratorFlow,
		GasGeneratorThrust: e.GasGeneratorThrust,

		ChamberPressure:      physics.Float(ch.Pressure),
		ChamberPressureMean:  ch.MeanPressure,
		FuelPumpInlet:        physics.Float(tp.FuelPump.InletPressure),
		FuelPumpOutlet:       physics.Float(tp.FuelPump.OutletPressure),
		OxPumpInlet:          physics.Float(tp.OxPump.InletPressure),
		OxPumpOutlet:         physics.Float(tp.OxPump.OutletPressure),
		FuelInjectorDrop:     ch.FuelInjectorDrop,
		OxInjectorDrop:       ch.OxInjectorDrop,
		InjectorDropFraction: dropFraction,

		ShaftRPM:            physics.Float(tp.RPM()),
		AngularAcceleration: tp.AngularAcceleration,
		TurbinePower:        tp.TurbinePower,
		FuelPumpPower:       tp.FuelPump.Power,
		OxPumpPower:         tp.OxPump.Power,
		MechanicalLoss:      tp.MechanicalLoss,
		ShaftTorque:         tp.Torque,
		PowerMargin:         tp.TurbinePower - tp.PumpPower() - tp.MechanicalLoss,

		FuelCavitationMargin: tp.FuelPump.CavitationMargin,
		OxCavitationMargin:   tp.OxPump.CavitationMargin,
		Vibration:            physics.Float(tp.Vibration),
		BearingTemperature:   physics.Float(tp.BearingTemperature),
		AxialDisplacement:    tp.AxialDisplacement * 1000,
		TurbineTemperature:   physics.Float(tp.TurbineTemperature),

		FuelValve:   tp.FuelValve,
		OxValve:     tp.OxValve,
		GasGenValve: tp.GasGenValve,
		BypassValve: tp.BypassValve,

		CharacteristicVelocity: ch.CharacteristicVelocity,
		CStarEfficiency:        ch.CStarEfficiency,
		CombustionEfficiency:   ch.CombustionEfficiency,
		ThrustCoefficient:      nz.ThrustCoefficient,
		ChamberTemperature:     ch.Temperature,
		MixtureDeviation:       ch.MixtureDeviation,
		TemperatureSpread:      ch.TemperatureNonuniformity,
		PressureRMS:            ch.PressureRMS,
		PressurePeak:           ch.PressurePeak,
		DominantFrequency:      ch.DominantFrequency,
		StabilityMargin:        ch.StabilityMargin,
		CombustionState:        ch.State.String(),
		CombustionStateCode:    int(ch.State),

		ThroatArea:       nz.ThroatArea,
		ThroatErosion:    nz.ThroatErosionMillimeters(),
		ExpansionRatio:   nz.ExpansionRatio,
		ExitPressure:     nz.ExitPressure,
		ExhaustVelocity:  nz.ExhaustVelocity,
		ExpansionRegime:  physics.Float(nz.ExpansionRegime),
		WallHeatFlux:     ch.AppliedWallHeatFlux(),
		WallMargin:       nz.MinimumMargin(),
		WallDamage:       nz.WallDamage,
		BurnedThrough:    nz.BurnedThrough,
		CoolingFlow:      nz.CoolingFlow,
		CoolantInlet:     nz.CoolantInletTemp,
		CoolantOutlet:    physics.Float(nz.CoolantOutletTemp),
		CoolingDrop:      nz.CoolingPressureDrop,
		CoolingEffective: nz.CoolingEffectiveness,
	}

	for _, s := range nz.Stations {
		t.WallStations = append(t.WallStations, WallStationTelemetry{
			Name:        s.Name,
			Temperature: s.Temperature,
			HeatFlux:    s.HeatFlux,
			Margin:      s.Margin(),
		})
	}

	if withSpectrum && e.Detailed {
		t.Spectrum = ch.Spectrum()
	}

	// Показания приборов: истинные величины проходят через модель датчиков.
	//
	// Физические связи при этом не нарушаются — шум накладывается на выход.
	// Тип показаний отличается от типа истинных величин намеренно: датчик
	// может не дать значения вовсе, и «нет данных» обязано отличаться от нуля.
	if dt > 0 {
		s := e.Sensors
		so := e.sensorOverrides
		measure := func(sensor *Sensor, trueValue float64) physics.Float {
			v, valid := sensor.UpdateWith(trueValue, dt, e.rng, so)
			if !valid {
				return physics.Float(math.NaN())
			}
			return physics.Float(v)
		}
		t.ChamberPressure = measure(s.ChamberPressure, ch.Pressure)
		t.FuelPumpOutlet = measure(s.FuelPumpOutlet, tp.FuelPump.OutletPressure)
		t.OxPumpOutlet = measure(s.OxPumpOutlet, tp.OxPump.OutletPressure)
		t.FuelPumpInlet = measure(s.FuelPumpInlet, tp.FuelPump.InletPressure)
		t.OxPumpInlet = measure(s.OxPumpInlet, tp.OxPump.InletPressure)
		t.FuelFlow = measure(s.FuelFlow, e.FuelFlow)
		t.OxFlow = measure(s.OxFlow, e.OxFlow)
		t.ShaftRPM = measure(s.ShaftSpeed, tp.RPM())
		t.TurbineTemperature = measure(s.TurbineTemp, tp.TurbineTemperature)
		t.BearingTemperature = measure(s.BearingTemp, tp.BearingTemperature)
		t.CoolantOutlet = measure(s.CoolantOutlet, nz.CoolantOutletTemp)
		t.Vibration = measure(s.Vibration, tp.Vibration)
	}

	return t
}

// Telemetry собирает снимок состояния бака.
func (t *Tank) Telemetry() TankTelemetry {
	return TankTelemetry{
		Name:               t.Config.Name,
		Mass:               t.Mass,
		FillFraction:       t.FillFraction(),
		Pressure:           t.Pressure,
		Temperature:        t.Temperature,
		Density:            t.Density(),
		Level:              t.LiquidLevel(),
		DrainRate:          t.DrainRate,
		DepletionTime:      physics.Float(t.DepletionTime()),
		SloshAmplitude:     t.SloshAmplitude,
		CenterOfMassOffset: t.CenterOfMassOffset,
		LineDrop:           t.LineDrop,
		LineTemperature:    t.LineTemperature,
		LeakRate:           t.LeakRate,
		PressurantMass:     t.PressurantRemaining,
		PressurantPressure: t.PressurantPressure,
		PressurantFlow:     t.PressurantFlow,
		PressurantValve:    t.PressurantValve,
	}
}
