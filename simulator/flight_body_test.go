package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/propulsion"
	"rocketTelemetrySim/simulator/vehicle"
)

func TestVehiclesExistBeforeLaunch(t *testing.T) {
	s := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
	if s.booster == nil || s.ship == nil {
		t.Fatal("both vehicles must exist before launch")
	}
	if s.flightBody != s.booster.flightBody || s.booster.flightBody == s.ship.flightBody {
		t.Fatal("stack must use the booster body; ship must own a distinct body")
	}
	if s.booster.phase != BoosterAttached {
		t.Fatal("return guidance started before separation")
	}
	if s.booster.propulsion == s.ship.propulsion || s.booster.propulsion.FuelTank == s.ship.propulsion.FuelTank {
		t.Fatal("vehicles share hardware")
	}
	if len(s.booster.propulsion.Engines) != 33 || len(s.ship.propulsion.Engines) != 6 {
		t.Fatal("missing engines")
	}
	for _, e := range s.ship.propulsion.Engines {
		if e.Running {
			t.Fatal("ship engine running before ignition")
		}
	}
	total := s.booster.propulsion.TotalMass() + s.ship.propulsion.TotalMass()
	if math.Abs(s.dryMass+s.state.FuelMass-total) > 1e-6 {
		t.Fatal("stack mass double-counts or omits ship")
	}
	if s.Snapshot().Booster != nil {
		t.Fatal("attached booster published as an independent returning vehicle")
	}
}

func TestSeparationPreservesVehicleHardwareAndState(t *testing.T) {
	s := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
	b, ship := s.booster, s.ship
	bsys, ssys := b.propulsion, ship.propulsion
	fuel, ox, shipFuel, shipOx := bsys.FuelTank, bsys.OxTank, ssys.FuelTank, ssys.OxTank
	engines := append([]*propulsion.Engine(nil), bsys.Engines...)
	shipEngines := append([]*propulsion.Engine(nil), ssys.Engines...)
	sensor, navigation, fins, flaps, shield := b.fuelSensor, b.positionSensor, b.gridFins, ship.attitude.Surfaces, ship.heatShield
	volumes := [4]float64{fuel.Config.Volume, ox.Config.Volume, shipFuel.Config.Volume, shipOx.Config.Volume}
	// Distinct, evolved states must survive the transition, not just pointer identity.
	fuel.Mass *= 0.11
	ox.Mass *= 0.17
	shipFuel.Mass *= 0.97
	fuel.Pressure = 234567
	fuel.PressurantRemaining = 123
	fuel.Settled = 0.42
	engines[0].Turbopump.Speed = 1234
	engines[0].Nozzle.BurnedThrough = true
	shipEngines[0].Turbopump.Speed = 321
	shipEngines[0].Nozzle.BurnedThrough = true
	b.state.FuelMass = bsys.PropellantMass()
	b.state.Position = b.state.Position.Unit().Scale(physics.EarthRadius + 65000)
	b.state.Velocity = physics.Vec3{X: 150, Y: 250, Z: 350}
	b.attitude.Omega = physics.Vec3{X: 0.1, Y: 0.2, Z: 0.3}
	b.attitude.GimbalPitch = 0.025
	b.prevAxialAccel, b.prevLateralAccel, b.prevAngularAccel = 12, 3, 0.2
	ship.rcsPropellant = 777
	s.elapsed = 130
	s.syncAttachedShip()
	before := b.state
	orientation, rate := b.attitude.Orientation, b.attitude.Omega
	total := s.dryMass + s.state.FuelMass
	s.performStageSeparation()
	if s.booster != b || s.ship != ship || s.flightBody != ship.flightBody {
		t.Fatal("vehicle replaced at separation")
	}
	if b.propulsion != bsys || s.propulsion != ssys || bsys.FuelTank != fuel || bsys.OxTank != ox || ssys.FuelTank != shipFuel || ssys.OxTank != shipOx {
		t.Fatal("tank or propulsion system replaced")
	}
	for i, e := range engines {
		if bsys.Engines[i] != e {
			t.Fatal("booster engine replaced")
		}
	}
	for i, e := range shipEngines {
		if ssys.Engines[i] != e {
			t.Fatal("ship engine replaced")
		}
	}
	if b.state != before || b.attitude.Omega != rate || b.attitude.Orientation != orientation || b.attitude.GimbalPitch != 0.025 {
		t.Fatal("booster motion/actuator state reset")
	}
	if ship.state.Position != before.Position || ship.state.Velocity != before.Velocity || ship.attitude.Omega != rate || ship.attitude.Orientation != orientation {
		t.Fatal("ship did not inherit constrained stack motion")
	}
	if [4]float64{fuel.Config.Volume, ox.Config.Volume, shipFuel.Config.Volume, shipOx.Config.Volume} != volumes {
		t.Fatal("tank geometry changed")
	}
	if fuel.Pressure != 234567 || fuel.PressurantRemaining != 123 || fuel.Settled != 0.42 || engines[0].Turbopump.Speed != 1234 || shipEngines[0].Turbopump.Speed != 321 {
		t.Fatal("hardware state reset")
	}
	if !engines[0].Nozzle.BurnedThrough || !shipEngines[0].Nozzle.BurnedThrough {
		t.Fatal("engine damage repaired by separation")
	}
	if b.fuelSensor != sensor || b.positionSensor != navigation || b.attitude.Surfaces != fins || ship.attitude.Surfaces != flaps || ship.heatShield != shield {
		t.Fatal("sensor/surface/shield replaced")
	}
	if b.prevAxialAccel != 12 || ship.prevAxialAccel != 12 || b.prevAngularAccel != 0.2 || ship.rcsPropellant != 777 {
		t.Fatal("acceleration history lost")
	}
	if math.Abs(total-bsys.TotalMass()-ssys.TotalMass()) > 1e-6 {
		t.Fatal("mass created or lost at separation")
	}
}

func TestAttachedShipEvolvesWithoutIndependentMotion(t *testing.T) {
	s := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
	b, ship := s.booster, s.ship
	initialFuel := b.propulsion.PropellantMass()
	temperature := ship.propulsion.FuelTank.Temperature
	for i := 0; i < 100; i++ {
		s.step(0.05)
	}
	if s.booster != b || s.ship != ship || s.stage != 1 {
		t.Fatal("unexpected vehicle lifecycle")
	}
	if ship.state.Position != b.state.Position || ship.state.Velocity != b.state.Velocity || ship.attitude.Omega != b.attitude.Omega {
		t.Fatal("attached bodies diverged")
	}
	if b.propulsion.PropellantMass() >= initialFuel {
		t.Fatal("booster tanks did not supply ascent engines")
	}
	if ship.propulsion.FuelTank.Temperature == temperature {
		t.Fatal("ship tank remained frozen until separation")
	}
	if ship.propulsion.TotalThrust != 0 {
		t.Fatal("dormant ship contributes thrust")
	}
}

func TestSeparationDuringFlightKeepsActualPropellant(t *testing.T) {
	s := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
	b, ship := s.booster, s.ship
	fuelVolume := b.propulsion.FuelTank.Config.Volume
	for i := 0; i < 5000 && s.stage == 1 && !s.crashed; i++ {
		s.step(0.05)
	}
	if s.stage != 2 {
		t.Fatal("flight did not reach separation")
	}
	if s.booster != b || s.ship != ship || b.propulsion.FuelTank.Config.Volume != fuelVolume {
		t.Fatal("flight rebuilt a vehicle or tank")
	}
	if b.state.FuelMass != b.propulsion.PropellantMass() {
		t.Fatal("booster mass differs from its tanks")
	}
	if b.state.FuelMass >= s.Config.FirstStage.FuelReserve {
		t.Fatal("actual propellant was refilled to configured reserve")
	}
	if b.propulsion.FuelTank.FillFraction() > 0.5 {
		t.Fatal("depleted ascent tank became a full landing tank")
	}
	t.Logf("separated at %.2f s; booster propellant %.1f t, configured reserve %.1f t", s.elapsed, b.state.FuelMass/1000, s.Config.FirstStage.FuelReserve/1000)
}

func TestVehicleResetStartsFreshReproducibleFlight(t *testing.T) {
	s := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 7)
	b, ship := s.booster, s.ship
	for i := 0; i < 100; i++ {
		s.step(0.05)
	}
	state := s.state
	shipTemperature := s.ship.propulsion.FuelTank.Temperature
	s.Reset()
	if s.booster == b || s.ship == ship || s.flightBody != s.booster.flightBody {
		t.Fatal("reset did not create a fresh attached pair")
	}
	for i := 0; i < 100; i++ {
		s.step(0.05)
	}
	if s.state != state || s.ship.propulsion.FuelTank.Temperature != shipTemperature {
		t.Fatal("reset changed the seeded flight")
	}
}

func TestShipHardwareAdvancesOnceOnSeparationTick(t *testing.T) {
	makeSim := func() *Simulation {
		s := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
		s.elapsed = 10
		s.mecoTime = 0
		s.phase = orbit.PhaseCoast1
		s.released = true
		s.state.Position = s.state.Position.Unit().Scale(physics.EarthRadius + 60000)
		s.syncAttachedShip()
		return s
	}
	during, before := makeSim(), makeSim()
	before.performStageSeparation()
	during.step(0.05)
	before.step(0.05)
	if during.stage != 2 {
		t.Fatal("did not separate within the tick")
	}
	a, b := during.ship.propulsion, before.ship.propulsion
	if *a.FuelTank != *b.FuelTank || *a.OxTank != *b.OxTank {
		t.Fatal("separation tick advanced ship tanks differently from one independent tick")
	}
}
