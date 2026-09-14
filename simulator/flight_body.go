package simulator

import (
	"log"
	"math/rand"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// flightBody owns hardware and physical state for the entire flight. Simulation
// selects the body driving the ascent integrator; changing that selection never
// constructs new hardware or transfers engine state between stages.
type flightBody struct {
	state         VehicleState
	propulsion    *PropulsionSystem
	engines       []vehicle.Engine
	attitude      VehicleAttitude
	heatShield    *HeatShield
	rcsPropellant float64

	prevAxialAccel   float64
	prevLateralAccel float64
	prevAngularAccel float64
}

// Ship exists from launch, including its tanks, engines and control surfaces.
// While attached, its kinematics follow the stack and its engines remain off.
type Ship struct{ *flightBody }

func initialVehicleAttitude(cfg vehicle.Config, position physics.Vec3) VehicleAttitude {
	c := DefaultAttitudeControl()
	if cfg.MaxGimbalDegrees > 0 {
		c.MaxGimbal = cfg.MaxGimbalDegrees * physics.DegToRad
	}
	a := VehicleAttitude{Config: c}
	azimuth, _ := physics.LaunchAzimuthForInclination(cfg.LaunchLatitude, cfg.TargetInclination, true)
	a.Init(physics.Attitude{Pitch: 90, Yaw: azimuth, Roll: cfg.InitialRoll()}, physics.NewLocalFrame(position))
	return a
}

func newStagePropulsion(cfg vehicle.Config, stageIndex int, propellantMass, structureMass float64,
	rng, sensorRng *rand.Rand, ambient float64) *PropulsionSystem {
	stage := cfg.FirstStage
	p := StageProps{StagePrefix: "S1", EngineCount: int(stage.EngineCount), EngineConfig: vehicle.FirstStageEngineConfig(cfg)}
	if stageIndex == 2 {
		stage = cfg.SecondStage
		p.StagePrefix = "S2"
		p.EngineCount = vehicle.SecondStagePrimaryCount(cfg)
		p.EngineConfig = vehicle.SecondStagePrimaryConfig(cfg)
		p.EngineGroups = vehicle.SecondStageGroups(cfg)
		p.PayloadMass = cfg.PayloadMass
	}
	p.StructureMass, p.PropellantMass, p.MixtureRatio = structureMass, propellantMass, stage.MixtureRatio
	p.FuelTankHeight, p.OxTankHeight = stage.FuelTankHeight, stage.OxTankHeight
	p.FuelTankPressure, p.OxTankPressure = stage.FuelTankPressure, stage.OxTankPressure
	p.FuelPressurantMass, p.OxPressurantMass = stage.FuelPressurantMass, stage.OxPressurantMass
	return NewPropulsionSystem(p, rng, sensorRng, ambient)
}

func (s *Simulation) initVehicles() {
	cfg := s.Config
	// A separate stream avoids changing booster manufacturing dispersions simply
	// because ship hardware is now initialized earlier.
	shipRng := rand.New(rand.NewSource(int64(splitMix64(uint64(s.seed) ^ 0x53484950))))
	shipSensorRng := rand.New(rand.NewSource(int64(splitMix64(uint64(s.seed) ^ 0x53454e53))))
	body := &flightBody{
		rcsPropellant: cfg.RCSPropellantMass,
		state:         VehicleState{Position: s.state.Position, Velocity: s.state.Velocity},
		attitude:      initialVehicleAttitude(cfg, s.state.Position),
		engines:       vehicle.BuildSecondStageEngine(cfg, stageStartTemperature(cfg)),
	}
	body.propulsion = newStagePropulsion(cfg, 2, cfg.SecondStage.FuelMass,
		cfg.SecondStage.DryMass+cfg.FairingMass, shipRng, shipSensorRng, stageStartTemperature(cfg))
	body.propulsion.StopAll()
	for i := range body.engines {
		body.engines[i].Running = false
		body.engines[i].Thrust = 0
	}
	body.state.FuelMass = body.propulsion.PropellantMass()
	if cfg.Flaps {
		length := secondStageLength(cfg)
		body.attitude.Surfaces = NewSurfaceSet(vehicle.ShipFlaps(length, cfg.Diameter/2))
		body.heatShield = NewHeatShield(length, cfg.Diameter, stageStartTemperature(cfg))
	}
	s.ship = &Ship{flightBody: body}
	if cfg.BoosterReturn {
		s.booster = newBoosterForBody(cfg, s.firstStage, 0, s.rng, s.seed)
		for i, e := range s.booster.propulsion.Engines {
			s.firstStage.engines[i].ID = e.ID
		}
	}
	s.syncAttachedShip()
}

func (s *Simulation) syncAttachedShip() {
	if s.stage != 1 || s.ship == nil {
		return
	}
	ship, stack := s.ship.flightBody, s.firstStage
	// Keep the existing common reference point for the constrained stack. This
	// lifetime refactor does not introduce separation impulses or geometric offsets.
	ship.state.Position, ship.state.Velocity = stack.state.Position, stack.state.Velocity
	ship.state.FuelMass = ship.propulsion.PropellantMass()
	ship.attitude.Orientation, ship.attitude.Omega = stack.attitude.Orientation, stack.attitude.Omega
	ship.prevAxialAccel, ship.prevLateralAccel, ship.prevAngularAccel = stack.prevAxialAccel, stack.prevLateralAccel, stack.prevAngularAccel
	s.dryMass = stack.propulsion.DryMass() + ship.propulsion.TotalMass()
	if s.booster != nil {
		s.booster.elapsed = s.elapsed
	}
}

func (s *Simulation) updateAttachedShip(dt float64) {
	ship := s.ship
	altitude := s.state.Altitude()
	axial := s.prevAxialAccel
	if !s.released {
		axial = physics.GravityMagnitudeAtAltitude(altitude)
	}
	ship.propulsion.Update(dt, 0, PropulsionEnvironment{
		AmbientPressure:      physics.Atmosphere(altitude).Pressure,
		StructureTemperature: stageStartTemperature(s.Config),
		AxialAcceleration:    axial, LateralAcceleration: s.prevLateralAccel,
		AngularAcceleration: s.prevAngularAccel, Gravity: physics.GravityMagnitudeAtAltitude(altitude),
	})
	s.dryMass = s.firstStage.propulsion.DryMass() + ship.propulsion.TotalMass()
}

// detach starts return guidance on the existing booster. Physical state,
// navigation channels, tank geometry, engine failures and actuator state survive.
func (b *Booster) detach(elapsed float64) {
	if b.phase != BoosterAttached {
		return
	}
	b.elapsed, b.boostbackStartElapsed = elapsed, elapsed
	b.phase = BoosterBoostback
	up := b.state.Position.Unit()
	ground := b.state.Velocity.Sub(physics.CorotatingVelocity(b.state.Position))
	horizontal := ground.Sub(up.Scale(ground.Dot(up)))
	b.outboundSpeed = horizontal.Norm()
	if b.outboundSpeed > 1e-3 {
		b.outboundHorizontal = horizontal.Unit()
	}
	b.attitude.Config.RCSRateCommand = true
	b.attitude.Config.RCSMoment = b.Config.RCSMoment
	b.attitude.Surfaces = b.gridFins
	b.gridFins.Deployed = true
	b.setEngineGroup(boosterBoostbackHigh)
	log.Printf("🪂 Бустер начинает возврат на T+%.1f с: высота %.1f км, скорость %.0f м/с",
		elapsed, b.state.Altitude()/1000, b.state.Velocity.Norm())
}

// detachedBooster keeps UI/trajectory publication tied to physical separation,
// rather than to the allocation time of the booster object.
func (s *Simulation) detachedBooster() *Booster {
	if s.stage != 2 {
		return nil
	}
	return s.booster
}
