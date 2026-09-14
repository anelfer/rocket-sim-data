package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/vehicle"
)

// TestBoosterGuidanceConvergesAtCatchPlane is an end-to-end regression for
// the return loop. The limits deliberately remain wider than CatchTower's
// strict capture envelope: this test detects a return to the former
// kilometre-scale guidance error without declaring a near miss a successful
// catch.
func TestBoosterGuidanceConvergesAtCatchPlane(t *testing.T) {
	if testing.Short() {
		t.Skip("full booster return")
	}

	m, _ := vehicle.MissionByID("transatmospheric")
	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithSeed(m.Apply(p.Config()), DefaultTimeConfig(), 1)

	dt := sim.Time.Step
	var b *Booster
	for step := 0; step < int(800/dt); step++ {
		sim.step(dt)
		b = sim.detachedBooster()
		if b != nil && (b.catch.Crossed || b.phase == BoosterDestroyed || b.phase == BoosterCaught) {
			break
		}
	}
	if b == nil {
		t.Fatal("flight never separated")
	}
	if !b.towerLink {
		t.Fatal("tower-relative terminal navigation never acquired")
	}
	if !b.catch.Crossed {
		t.Fatalf("booster never crossed the catch plane; phase=%s", b.phase)
	}

	miss := math.Hypot(b.catch.Along-b.tower.ArmReach, b.catch.Across)
	if miss > 25 || b.catch.Vertical > 10 || b.catch.Horizontal > 10 ||
		b.catch.Tilt > 10 || b.catch.AngularRate > 10 {
		t.Fatalf("return did not converge: miss=%.2f m, vertical=%.2f m/s, "+
			"horizontal=%.2f m/s, tilt=%.2f deg, rate=%.2f deg/s",
			miss, b.catch.Vertical, b.catch.Horizontal, b.catch.Tilt, b.catch.AngularRate)
	}
}
