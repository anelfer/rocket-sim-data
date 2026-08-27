package simulator

import (
	"fmt"
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// Диагностика (временная, не часть постоянного набора): полный прогон
// Starship/Super Heavy V3 от старта до приводнения бустера, по нескольким
// зёрнам, с печатью промаха, скорости касания и остаточного вращения.
func TestZZDiagnosticBoosterLandingBaseline(t *testing.T) {
	cfg := vehicle.StarshipConfig()
	dt := 0.05

	for _, seed := range []int64{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20} {
		sim := NewSimulationWithSeed(cfg, DefaultTimeConfig(), seed)
		maxSteps := int(1200.0 / dt)
		steps := 0
		maxAlt := 0.0
		var sepAlt, sepVert, sepHoriz float64
		var ignAlt, ignSpeed float64
		lastPhase := BoosterPhase(-1)
		for ; steps < maxSteps; steps++ {
			sim.mu.Lock()
			sim.step(dt)
			b := sim.booster
			if b != nil {
				alt := b.state.Position.Norm() - physics.EarthRadius
				if alt > maxAlt {
					maxAlt = alt
				}
				if sepAlt == 0 && b.phase == BoosterBoostback {
					sepAlt = alt
					sepVert = b.state.Velocity.Dot(b.state.Position.Unit())
					sepHoriz = b.state.Velocity.Sub(b.state.Position.Unit().Scale(sepVert)).Norm()
				}
				if lastPhase == BoosterBoostback && b.phase == BoosterCoast {
					vert := b.state.Velocity.Dot(b.state.Position.Unit())
					horiz := b.state.Velocity.Sub(b.state.Position.Unit().Scale(vert)).Norm()
					fmt.Printf("seed %d: Boostback->Coast at T+%.1fs alt=%.0fm vert=%.0fm/s horiz=%.0fm/s fuel=%.1ft\n",
						seed, b.elapsed, alt, vert, horiz, b.state.FuelMass/1000)
				}
				if lastPhase == BoosterCoast && b.phase == BoosterLandingBurn {
					ignAlt = alt
					vert := b.state.Velocity.Dot(b.state.Position.Unit())
					horiz := b.state.Velocity.Sub(b.state.Position.Unit().Scale(vert)).Norm()
					ignSpeed = math.Hypot(vert, horiz)
					fmt.Printf("seed %d: Coast->LandingBurn at T+%.1fs alt=%.0fm vert=%.0fm/s horiz=%.0fm/s fuel=%.1ft\n",
						seed, b.elapsed, ignAlt, vert, horiz, b.state.FuelMass/1000)
				}
				lastPhase = b.phase
			}
			done := b != nil && (b.phase == BoosterSplashdown || b.phase == BoosterDestroyed)
			sim.mu.Unlock()
			if done {
				break
			}
		}
		fmt.Printf("seed %d: разделение alt=%.0fм верт=%.0fм/с гор=%.0fм/с; пик высоты=%.0fм; розжиг alt=%.0fм скорость=%.0fм/с\n",
			seed, sepAlt, sepVert, sepHoriz, maxAlt, ignAlt, ignSpeed)

		sim.mu.Lock()
		b := sim.booster
		if b == nil {
			sim.mu.Unlock()
			fmt.Printf("seed %d: бустер так и не отделился за %.0f с\n", seed, float64(steps)*dt)
			continue
		}
		omega := b.attitude.Omega
		omegaDeg := omega.Scale(physics.RadToDeg)
		fmt.Printf("seed %d: фаза=%s elapsed=%.1fс шагов=%d fuel=%.1ft engines=%d\n"+
			"  промах=%.1fм верт.скорость=%.2fм/с гор.скорость=%.2fм/с полная=%.2fм/с\n"+
			"  omega(deg/s) body(x=roll,y=pitch,z=yaw)=(%.2f, %.2f, %.2f) |omega|=%.2f deg/s\n",
			seed, b.phase.String(), b.elapsed, steps, b.state.FuelMass/1000, b.propulsion.Commissioned,
			b.landingMissDistanceLast, b.splashSpeed, b.splashHorizontalSpeed, b.splashTotalSpeed,
			omegaDeg.X, omegaDeg.Y, omegaDeg.Z, omega.Norm()*physics.RadToDeg)
		sim.mu.Unlock()

		if math.IsNaN(b.landingMissDistanceLast) {
			fmt.Printf("seed %d: НЕ ПРИВОДНИЛСЯ (промах NaN)\n", seed)
		}
	}
}
