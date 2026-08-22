package simulator

import (
	"math"

	"rocketTelemetrySim/metrics"
	"rocketTelemetrySim/simulator/env"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Телеметрия.
//
// Telemetry — неизменяемый снимок состояния. Публикация метрик и обработка
// REST-запросов работают только с копией, поэтому гонок данных нет: раньше
// хендлеры читали срез двигателей и поля симуляции без блокировки, пока
// цикл симуляции писал в них.
// -----------------------------------------------------------------------------

// EngineTelemetry — состояние одного двигателя.
type EngineTelemetry struct {
	ID          string  `json:"id"`
	Running     bool    `json:"running"`
	Thrust      float64 `json:"thrust"`
	MaxThrust   float64 `json:"maxThrust"`
	MinThrust   float64 `json:"minThrust"`
	ISP         float64 `json:"isp"`
	ChamberTemp float64 `json:"chamberTemp"` // °C
	NozzleTemp  float64 `json:"nozzleTemp"`  // °C
	WallTemp    float64 `json:"wallTemp"`    // °C
	TurbineTemp float64 `json:"turbineTemp"` // °C
}

// Telemetry — полный снимок состояния симуляции.
type Telemetry struct {
	Time       float64 `json:"time"`
	Phase      string  `json:"phase"`
	PhaseIndex int     `json:"phaseIndex"`
	Stage      int     `json:"stage"`

	// Массы, кг.
	DryMass        float64 `json:"dryMass"`
	FuelMass       float64 `json:"fuelMass"`
	FuelMassSensed float64 `json:"fuelMassSensed"` // показание датчика остатка, на нём принимается решение о MECO
	TotalMass      float64 `json:"totalMass"`

	// Кинематика.
	Altitude           float64 `json:"altitude"`           // м
	VerticalVelocity   float64 `json:"verticalVelocity"`   // м/с, радиальная
	HorizontalVelocity float64 `json:"horizontalVelocity"` // м/с, инерциальная
	GroundSpeed        float64 `json:"groundSpeed"`        // м/с, относительно поверхности
	TotalVelocity      float64 `json:"totalVelocity"`      // м/с, инерциальная

	VerticalAccel   float64 `json:"verticalAccel"`   // м/с²
	HorizontalAccel float64 `json:"horizontalAccel"` // м/с²

	// Положение.
	Latitude  float64 `json:"lat"`
	Longitude float64 `json:"lon"`

	// Ориентация корпуса.
	Pitch   float64 `json:"pitch"`   // угол места продольной оси, градусы
	Yaw     float64 `json:"yaw"`     // азимут продольной оси, градусы от севера
	Roll    float64 `json:"roll"`    // крен вокруг продольной оси, градусы
	Azimuth float64 `json:"azimuth"` // синоним Yaw, оставлен для совместимости

	// Угловые скорости корпуса, град/с.
	PitchRate float64 `json:"pitchRate"`
	YawRate   float64 `json:"yawRate"`
	RollRate  float64 `json:"rollRate"`

	// AttitudeSensorErrorDeg — расхождение показания датчика ориентации
	// с истинной ориентацией, градусы. Это и есть то, чем в действительности
	// распоряжается автопилот при вычислении ошибки (attitudeError) — а не
	// сама ошибка наведения.
	AttitudeSensorErrorDeg float64 `json:"attitudeSensorErrorDeg"`

	// Аэродинамические углы, градусы.
	AngleOfAttack float64 `json:"angleOfAttack"`
	SideslipAngle float64 `json:"sideslipAngle"`
	TotalAoA      float64 `json:"totalAoA"`

	// Рулевой тракт.
	GimbalPitch      float64 `json:"gimbalPitch"`      // градусы
	GimbalYaw        float64 `json:"gimbalYaw"`        // градусы
	GimbalDemand     float64 `json:"gimbalDemand"`     // потребное отклонение, градусы
	GimbalLimit      float64 `json:"gimbalLimit"`      // предел хода, градусы
	ControlAuthority float64 `json:"controlAuthority"` // потребное к располагаемому
	ControlSaturated bool    `json:"controlSaturated"`
	UsingRCS         bool    `json:"usingRcs"`

	// Моменты, Н·м.
	AeroTorque    float64 `json:"aeroTorque"`
	ControlTorque float64 `json:"controlTorque"`

	// StaticMargin — запас статической устойчивости в калибрах.
	// Отрицательный означает, что корпус сам уводится от потока.
	StaticMargin float64 `json:"staticMargin"`

	// Среда.
	AirDensity      float64 `json:"airDensity"`   // кг/м³
	AmbientTemp     float64 `json:"ambientTemp"`  // °C, эффективная
	KineticTemp     float64 `json:"kineticTemp"`  // °C, кинетическая температура газа
	RecoveryTemp    float64 `json:"recoveryTemp"` // °C, температура торможения
	Mach            float64 `json:"mach"`
	DynamicPressure float64 `json:"dynamicPressure"` // Па
	DragForce       float64 `json:"drag"`            // Н
	HeatFlux        float64 `json:"heatFlux"`        // Вт/м², в критической точке

	// Силы.
	TotalThrust float64 `json:"totalThrust"` // Н
	Gravity     float64 `json:"gravity"`     // м/с², истинная гравитация
	TWR         float64 `json:"twr"`
	Throttle    float64 `json:"throttle"` // 0…1

	// ThrottleFloor — нижний предел дросселирования камеры, доля номинала.
	// Ниже него горение срывается, поэтому уставка туда не пропускается.
	ThrottleFloor float64 `json:"throttleFloor"`

	// ThrottleLimited сообщает, что уставка оператора удержана на пределе.
	ThrottleLimited bool `json:"throttleLimited"`

	// Орбита. Апоцентр и перицентр не определены, пока траектория не замкнута,
	// и в этом случае отдаются как null.
	Apoapsis     Float   `json:"apoapsis"`  // м
	Periapsis    Float   `json:"periapsis"` // м
	Eccentricity float64 `json:"eccentricity"`
	Inclination  float64 `json:"inclination"` // градусы
	OrbitPeriod  float64 `json:"orbitPeriod"` // с

	// Max-Q.
	MaxQ         float64 `json:"maxQ"`         // Па
	MaxQTime     float64 `json:"maxQTime"`     // с
	MaxQAltitude float64 `json:"maxQAltitude"` // м

	// OrbitClosed сообщает, что траектория замкнута и перицентр выше
	// поверхности. Пока это не так, перицентр как величина бессмыслен
	// для графика: у баллистической горки он лежит внутри Земли.
	OrbitClosed bool `json:"orbitClosed"`

	EnginesRunning int               `json:"enginesRunning"`
	Engines        []EngineTelemetry `json:"engines"`

	// SpentStage — состояние отработавшей первой ступени, если она отделилась.
	SpentStage *env.SpentStageTelemetry `json:"spentStage,omitempty"`

	// Propulsion — состояние двигательной установки: баки, турбонасосы,
	// камеры сгорания, сопла и охлаждение.
	Propulsion PropulsionTelemetry `json:"propulsion"`

	Crashed      bool `json:"crashed"`
	OrbitReached bool `json:"orbitReached"`

	// Flaps — состояние аэродинамических управляющих поверхностей.
	// На активном участке список пуст: поверхностей ещё нет или они прижаты.
	Flaps []FlapTelemetry `json:"flaps,omitempty"`

	// FlapsDeployed сообщает, что поверхности выпущены в поток.
	FlapsDeployed bool `json:"flapsDeployed"`

	// FlapAuthority — располагаемый момент поверхностей по тангажу, Н·м.
	FlapAuthority float64 `json:"flapAuthority"`

	// Scene — оси корпуса и направление потока в местном горизонте.
	// Нужны трёхмерной сцене, из остальных полей однозначно не выводятся.
	Scene SceneTelemetry `json:"scene"`

	// Landing — итог касания. Заполняется после посадки.
	Landing *LandingTelemetry `json:"landing,omitempty"`

	// HeatShield — состояние теплозащиты: какая сторона подставлена потоку
	// и сколько ей осталось до предела.
	HeatShield *HeatShieldTelemetry `json:"heatShield,omitempty"`
}

// buildTelemetryLocked собирает снимок состояния. Вызывается под удержанным mu.
func (s *Simulation) buildTelemetryLocked() Telemetry {
	st := s.state
	altitude := st.Altitude()
	atm := physics.Atmosphere(altitude)

	vRel := st.AirRelativeVelocity()
	relSpeed := vRel.Norm()
	mach := machNumber(relSpeed, atm.SoundSpeed)

	elements := physics.ElementsFromState(st.Position, st.Velocity, s.elapsed)
	geo := physics.ECIToGeodetic(st.Position, s.elapsed)
	frame := st.LocalFrame()

	// Ориентация берётся фактическая — из уравнений вращения, а не из команды
	// наведения. Иначе телеметрия показывала бы, куда ракета должна смотреть,
	// а не куда смотрит: при потере управления углы оставались бы штатными,
	// пока корпус кувыркается.
	attitude := s.attitude.AttitudeIn(frame)
	body := s.attitude.Orientation.Body()

	fm := ForceModel{
		DryMass:   s.dryMass,
		Thrust:    vehicle.TotalThrust(s.engines),
		ThrustDir: body.Forward,
		MassFlow:  vehicle.TotalMassFlow(s.engines),
		Area:      s.area,
	}
	acc := fm.Evaluate(st)
	east, north, verticalAccel := frame.Decompose(acc.Total)
	horizontalAccel := math.Hypot(east, north)

	// Угловые скорости и аэродинамические углы — от фактической ориентации.
	pitchRate, yawRate, rollRate := s.attitude.Rates()
	alpha, beta := physics.AeroAngles(body, vRel)

	// Расхождение показания датчика ориентации с истинной ориентацией —
	// то, что реально закладывает шум в контур автопилота. |W|, а не W:
	// кватернионы q и −q описывают один и тот же поворот.
	attitudeErrQ := s.attitude.Orientation.Conjugate().Multiply(s.lastValidSensedOrientation).Normalized()
	attitudeSensorErrorDeg := 2 * math.Acos(math.Min(1, math.Abs(attitudeErrQ.W))) * physics.RadToDeg

	mass := s.dryMass + st.FuelMass
	gravity := physics.GravityMagnitudeAtAltitude(altitude)

	twr := 0.0
	if mass > 0 && gravity > 0 {
		twr = fm.Thrust / (mass * gravity)
	}

	t := Telemetry{
		Time:       s.elapsed,
		Phase:      s.phase.String(),
		PhaseIndex: int(s.phase),
		Stage:      s.stage,

		DryMass:        s.dryMass,
		FuelMass:       st.FuelMass,
		FuelMassSensed: s.sensedFuelMass,
		TotalMass:      mass,

		Altitude:           altitude,
		VerticalVelocity:   st.RadialVelocity(),
		HorizontalVelocity: st.TransverseVelocity(),
		GroundSpeed:        st.GroundSpeed(),
		TotalVelocity:      st.Velocity.Norm(),

		VerticalAccel:   verticalAccel,
		HorizontalAccel: horizontalAccel,

		Latitude:  geo.Latitude,
		Longitude: geo.Longitude,

		Pitch:   attitude.Pitch,
		Yaw:     attitude.Yaw,
		Roll:    attitude.Roll,
		Azimuth: attitude.Yaw,

		PitchRate: pitchRate,
		YawRate:   yawRate,
		RollRate:  rollRate,

		AttitudeSensorErrorDeg: attitudeSensorErrorDeg,

		AngleOfAttack: alpha,
		SideslipAngle: beta,
		TotalAoA:      physics.AngleOfAttack(body.Forward, vRel),

		GimbalPitch:  s.attitude.GimbalPitch * physics.RadToDeg,
		GimbalYaw:    s.attitude.GimbalYaw * physics.RadToDeg,
		GimbalDemand: math.Hypot(s.attitude.CommandPitch, s.attitude.CommandYaw) * physics.RadToDeg,
		GimbalLimit:  s.attitude.EffectiveLimit * physics.RadToDeg,

		ControlAuthority: s.attitude.Authority,
		ControlSaturated: s.attitude.Saturated,
		UsingRCS:         s.attitude.UsingRCS,

		AeroTorque:    s.attitude.AeroTorque.Norm(),
		ControlTorque: s.attitude.ControlTorque.Norm(),
		StaticMargin:  s.attitude.StaticMargin,

		AirDensity: atm.Density,
		AmbientTemp: physics.KelvinToCelsius(
			physics.EffectiveEnvironmentTemperature(altitude, relSpeed),
		),
		KineticTemp:     physics.KelvinToCelsius(atm.Temperature),
		RecoveryTemp:    physics.KelvinToCelsius(physics.RecoveryTemperature(atm.Temperature, mach)),
		Mach:            mach,
		DynamicPressure: acc.DynamicQ,
		DragForce:       acc.DragForce,
		HeatFlux:        physics.StagnationHeatFlux(atm.Density, relSpeed, 1.0),

		TotalThrust: fm.Thrust,
		Gravity:     gravity,
		TWR:         twr,
		Throttle:    s.appliedThrottle(),

		ThrottleFloor:   s.throttleFloor,
		ThrottleLimited: s.throttleLimited || s.anyThrottleLimited(),

		Eccentricity: elements.Eccentricity,
		Inclination:  elements.InclinationDegrees(),
		OrbitPeriod:  elements.Period(),

		MaxQ:         s.maxQ.PeakDynamicPressure,
		MaxQTime:     s.maxQ.PeakTime,
		MaxQAltitude: s.maxQ.PeakAltitude,

		EnginesRunning: vehicle.RunningEngines(s.engines),

		Crashed:      s.crashed,
		OrbitReached: s.orbitReached,
	}

	// Апоцентр и перицентр ведут себя по-разному, и отдавать их одинаково
	// неправильно.
	//
	// Апоцентр определён с первых секунд полёта и осмыслен на всём выведении:
	// именно по нему работает наведение второй ступени, и на графике видно,
	// как он подтягивается к целевой высоте. Его отдаём всегда, пока
	// траектория замкнута.
	//
	// Перицентр на активном участке лежит внутри Земли — у баллистической
	// горки он уходит на тысячи километров вниз. Такое значение ломает масштаб
	// панели и ничего не сообщает, поэтому до выхода на орбиту вместо него
	// идёт NaN, и Grafana рисует разрыв.
	t.OrbitClosed = elements.Elliptical && elements.PeriapsisAltitude > 0

	if elements.Elliptical {
		t.Apoapsis = Float(elements.ApoapsisAltitude)
	} else {
		t.Apoapsis = Float(math.NaN())
	}

	if t.OrbitClosed {
		t.Periapsis = Float(elements.PeriapsisAltitude)
	} else {
		t.Periapsis = Float(math.NaN())
	}

	// Телеметрия первой ступени существует с момента старта. До разделения она
	// повторяет состояние носителя: ступень летит, просто в его составе.
	if s.spentStage != nil {
		t.SpentStage = s.spentStage.Telemetry(s.elapsed)
	} else {
		t.SpentStage = s.attachedStageTelemetry(altitude, atm, vRel, mach, acc, geo)
	}

	t.Propulsion = s.buildPropulsionTelemetry()

	// Аэродинамические поверхности: положение каждой, факт выпуска
	// и располагаемый момент. На активном участке список пуст.
	t.Flaps = s.buildFlapTelemetry()
	t.FlapsDeployed = s.attitude.Surfaces != nil && s.attitude.Surfaces.Deployed
	t.FlapAuthority = s.attitude.SurfaceAuthority
	t.HeatShield = s.buildHeatShieldTelemetry()

	// Оси корпуса и направление потока для трёхмерной сцены.
	t.Scene = s.buildSceneTelemetry(vRel, geo)
	t.Landing = s.buildLandingTelemetry()

	t.Engines = make([]EngineTelemetry, len(s.engines))
	for i := range s.engines {
		e := s.engines[i]
		t.Engines[i] = EngineTelemetry{
			ID:          e.ID,
			Running:     e.Running,
			Thrust:      e.Thrust,
			MaxThrust:   e.MaxThrust,
			MinThrust:   e.MinThrust,
			ISP:         e.ISP,
			ChamberTemp: e.ChamberTempC(),
			NozzleTemp:  e.NozzleTempC(),
			WallTemp:    e.WallTempC(),
			TurbineTemp: e.TurbineTempC(),
		}
	}

	return t
}

// attachedStageTelemetry собирает телеметрию первой ступени до разделения.
//
// Пока ступень в составе носителя, её кинематика совпадает с кинематикой
// ракеты, вращения нет (полёт управляемый), а нагрев конструкции определяется
// тем же набегающим потоком. Благодаря этому панели первой ступени заполнены
// с момента старта, а не появляются внезапно на T+150 с.
func (s *Simulation) attachedStageTelemetry(
	altitude float64,
	atm physics.AtmosphereState,
	vRel physics.Vec3,
	mach float64,
	acc Accelerations,
	geo physics.Geodetic,
) *env.SpentStageTelemetry {
	speed := vRel.Norm()

	plasma := physics.AnalyzePlasma(atm.Temperature, atm.Density, speed,
		physics.DefaultTelemetryFrequency)

	skin := physics.KelvinToCelsius(
		physics.EffectiveEnvironmentTemperature(altitude, speed),
	)

	return &env.SpentStageTelemetry{
		Name:      s.Config.FirstStage.Name,
		Attached:  true,
		Outcome:   env.StageAttached.String(),
		Reason:    "в составе носителя",
		Altitude:  altitude,
		Latitude:  geo.Latitude,
		Longitude: geo.Longitude,
		Speed:     speed,
		Mach:      mach,
		HeatFlux:  physics.StagnationHeatFlux(atm.Density, speed, s.Config.Diameter/2),
		DynamicQ:  acc.DynamicQ,

		// Ступень удерживается системой управления носителя: собственного
		// вращения нет, ориентация совпадает с ориентацией ракеты.
		AngleOfAttack: s.gnc.TotalAoA,
		SpinRate:      0,
		Pitch:         s.gnc.Attitude.Pitch,
		Yaw:           s.gnc.Attitude.Yaw,
		Roll:          s.gnc.Attitude.Roll,
		Tumbling:      false,

		HottestNode: skin,
		Nodes: []env.ThermalNodeTelemetry{
			{Name: "nose", Temperature: skin, PeakTemp: skin, MeltingsAt: 577},
			{Name: "side", Temperature: skin, PeakTemp: skin, MeltingsAt: 527},
			{Name: "base", Temperature: skin, PeakTemp: skin, MeltingsAt: 927},
		},

		ShockTemp:          physics.KelvinToCelsius(plasma.ShockTemperature),
		IonizationFraction: plasma.IonizationFraction,
		PlasmaFrequency:    plasma.PlasmaFrequency,
		RadioBlackout:      plasma.RadioBlackout,
		Luminous:           plasma.Luminous,

		MaxAltitude:  altitude,
		PeakHeatFlux: 0,
		PeakDynamicQ: s.maxQ.PeakDynamicPressure,
	}
}

// Publish отправляет снимок в Prometheus.
func (t Telemetry) Publish() {
	metrics.SetFlight(metrics.FlightSample{
		Time:                   t.Time,
		Altitude:               t.Altitude,
		VerticalVelocity:       t.VerticalVelocity,
		HorizontalVelocity:     t.HorizontalVelocity,
		GroundSpeed:            t.GroundSpeed,
		VerticalAccel:          t.VerticalAccel,
		HorizontalAccel:        t.HorizontalAccel,
		Mass:                   t.TotalMass,
		FuelMass:               t.FuelMass,
		FuelMassSensed:         t.FuelMassSensed,
		Drag:                   t.DragForce,
		AirDensity:             t.AirDensity,
		AmbientTemp:            t.AmbientTemp,
		RecoveryTemp:           t.RecoveryTemp,
		Mach:                   t.Mach,
		DynamicPressure:        t.DynamicPressure,
		HeatFlux:               t.HeatFlux,
		EnginesRunning:         t.EnginesRunning,
		Pitch:                  t.Pitch,
		Yaw:                    t.Yaw,
		Roll:                   t.Roll,
		Azimuth:                t.Azimuth,
		PitchRate:              t.PitchRate,
		YawRate:                t.YawRate,
		RollRate:               t.RollRate,
		AttitudeSensorErrorDeg: t.AttitudeSensorErrorDeg,
		AngleOfAttack:          t.AngleOfAttack,
		GimbalPitch:            t.GimbalPitch,
		GimbalYaw:              t.GimbalYaw,
		GimbalDemand:           t.GimbalDemand,
		GimbalLimit:            t.GimbalLimit,
		ControlAuthority:       t.ControlAuthority,
		ControlSaturated:       t.ControlSaturated,
		UsingRCS:               t.UsingRCS,
		AeroTorque:             t.AeroTorque,
		ControlTorque:          t.ControlTorque,
		StaticMargin:           t.StaticMargin,
		SideslipAngle:          t.SideslipAngle,
		TotalAoA:               t.TotalAoA,
		Gravity:                t.Gravity,
		TWR:                    t.TWR,
		Throttle:               t.Throttle,
		TotalThrust:            t.TotalThrust,
		Latitude:               t.Latitude,
		Longitude:              t.Longitude,
	})

	metrics.SetOrbit(metrics.OrbitSample{
		Apoapsis:     t.Apoapsis.Value(),
		Periapsis:    t.Periapsis.Value(),
		Eccentricity: t.Eccentricity,
		Inclination:  t.Inclination,
		Period:       t.OrbitPeriod,
		Phase:        t.PhaseIndex,
		Closed:       t.OrbitClosed,
	})

	if st := t.SpentStage; st != nil {
		outcome := int(env.StageAttached)
		switch st.Outcome {
		case env.StageFlying.String():
			outcome = int(env.StageFlying)
		case env.StageBrokenUp.String():
			outcome = int(env.StageBrokenUp)
		case env.StageImpacted.String():
			outcome = int(env.StageImpacted)
		}
		sample := metrics.SpentStageSample{
			Altitude:      st.Altitude,
			Latitude:      st.Latitude,
			Longitude:     st.Longitude,
			Speed:         st.Speed,
			Mach:          st.Mach,
			SkinTemp:      st.HottestNode,
			HeatFlux:      st.HeatFlux,
			DynamicQ:      st.DynamicQ,
			Outcome:       outcome,
			SpinRate:      st.SpinRate,
			AngleOfAttack: st.AngleOfAttack,
			ShockTemp:     st.ShockTemp,
			Ionization:    st.IonizationFraction,
			PlasmaFreq:    st.PlasmaFrequency,
			RadioBlackout: st.RadioBlackout,
			Attached:      st.Attached,
		}
		for _, n := range st.Nodes {
			sample.NodeTemps = append(sample.NodeTemps,
				metrics.NodeTemp{Name: n.Name, Temperature: n.Temperature})
		}
		metrics.SetSpentStage(sample)

		if d := st.Debris; d != nil {
			metrics.SetDebris(metrics.DebrisSample{
				CenterLatitude:  d.CenterLatitude,
				CenterLongitude: d.CenterLongitude,
				SemiMajorAxis:   d.SemiMajorAxis,
				SemiMinorAxis:   d.SemiMinorAxis,
				SurvivingMass:   d.SurvivingMassFraction,
			})
		}
	}

	t.Propulsion.Publish()

	for _, e := range t.Engines {
		metrics.SetEngine(metrics.EngineSample{
			ID:          e.ID,
			Thrust:      e.Thrust,
			ISP:         e.ISP,
			ChamberTemp: e.ChamberTemp,
			NozzleTemp:  e.NozzleTemp,
			WallTemp:    e.WallTemp,
			TurbineTemp: e.TurbineTemp,
		})
	}
}

// -----------------------------------------------------------------------------
// Потокобезопасные аксессоры
// -----------------------------------------------------------------------------

// Snapshot возвращает копию текущей телеметрии.
func (s *Simulation) Snapshot() Telemetry {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.telemetry
}

// EngineList возвращает копию состояния двигателей.
func (s *Simulation) EngineList() []EngineTelemetry {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := make([]EngineTelemetry, len(s.telemetry.Engines))
	copy(out, s.telemetry.Engines)
	return out
}

// Phase возвращает текущую фазу полёта.
func (s *Simulation) Phase() orbit.FlightPhase {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.phase
}

// SetEngineState изменяет состояние одного двигателя (для REST API).
// Идентификатор — либо полный ("S1-3"), либо порядковый номер в текущей ступени.
func (s *Simulation) SetEngineState(id string, thrust float64, running bool) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := vehicle.UpdateEngine(id, thrust, running, s.engines); err != nil {
		return err
	}
	s.telemetry = s.buildTelemetryLocked()
	return nil
}

// SetAllEnginesRunning включает или выключает все двигатели (для REST API).
func (s *Simulation) SetAllEnginesRunning(running bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for i := range s.engines {
		s.engines[i].Running = running
		if !running {
			s.engines[i].Thrust = 0
		}
	}
	s.telemetry = s.buildTelemetryLocked()
}

// ForceStageSeparation принудительно отделяет ступень (для REST API).
func (s *Simulation) ForceStageSeparation() {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.stage != 1 {
		return
	}
	s.performStageSeparation()
	s.telemetry = s.buildTelemetryLocked()
}

// FlapTelemetry — состояние одной управляющей поверхности.
type FlapTelemetry struct {
	Name string `json:"name"`

	// Deflection — фактический угол отклонения, градусы.
	Deflection float64 `json:"deflection"`

	// Command — потребный угол, градусы.
	Command float64 `json:"command"`

	// Limit — предельный угол, градусы.
	Limit float64 `json:"limit"`

	// Area — площадь панели, м².
	Area float64 `json:"area"`

	// Manual сообщает, что поверхность под ручным управлением.
	Manual bool `json:"manual"`

	// Jammed сообщает, что привод заклинило.
	Jammed bool `json:"jammed"`
}

// buildFlapTelemetry собирает состояние управляющих поверхностей.
func (s *Simulation) buildFlapTelemetry() []FlapTelemetry {
	set := s.attitude.Surfaces
	if set.Count() == 0 {
		return nil
	}

	out := make([]FlapTelemetry, 0, set.Count())
	for i, surface := range set.Surfaces {
		out = append(out, FlapTelemetry{
			Name:       surface.Name,
			Deflection: set.Deflection[i] * physics.RadToDeg,
			Command:    set.Command[i] * physics.RadToDeg,
			Limit:      surface.MaxDeflection * physics.RadToDeg,
			Area:       surface.Area,
			Manual:     set.Manual[i],
			Jammed:     set.Jammed[i],
		})
	}
	return out
}
