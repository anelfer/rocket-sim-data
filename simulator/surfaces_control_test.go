package simulator

import (
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Кто держит корпус на входе.
//
// В разрежённых слоях — двигатели ориентации: плавникам нечем работать.
// Ниже, где напор набирает силу, корпус обязаны держать плавники, а двигатели
// ориентации — замолчать. Иначе они дожигают рабочее тело весь спуск, а на
// настоящем корабле его на это не хватило бы.
// -----------------------------------------------------------------------------

func TestFlapsTakeOverFromThrustersInDenseAir(t *testing.T) {
	if testing.Short() {
		t.Skip("длинный прогон")
	}

	m, _ := vehicle.MissionByID("transatmospheric")
	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithSeed(m.Apply(p.Config()), DefaultTimeConfig(), 1)

	var (
		thinSteps, thinRCS   int // выше 100 км
		denseSteps, denseRCS int // на участке заметного напора
		aoaWorst             float64
		flapSpread           float64
	)

	for i := 0; i < 60000; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		phase, alt := sim.phase, sim.state.Altitude()
		sim.telemetry = sim.buildTelemetryLocked()
		tel := sim.telemetry
		rcs := sim.attitude.UsingRCS
		set := sim.attitude.Surfaces
		crashed := sim.crashed

		// Разброс между передними и задними плавниками: им и создаётся момент.
		spread := 0.0
		if set != nil && set.Count() == 4 {
			spread = (set.Deflection[0] + set.Deflection[1]) -
				(set.Deflection[2] + set.Deflection[3])
			if spread < 0 {
				spread = -spread
			}
		}
		sim.mu.Unlock()

		if phase != orbit.PhaseEntry {
			if crashed || phase == orbit.PhaseLanded {
				break
			}
			continue
		}

		switch {
		case alt > 100000:
			thinSteps++
			if rcs {
				thinRCS++
			}
		case tel.DynamicPressure > 2000 && alt > 40000:
			denseSteps++
			if rcs {
				denseRCS++
			}
			if d := tel.TotalAoA - 70; d > aoaWorst || -d > aoaWorst {
				if d < 0 {
					d = -d
				}
				aoaWorst = d
			}
			if spread > flapSpread {
				flapSpread = spread
			}
		}

		if crashed || phase == orbit.PhaseLanded {
			break
		}
	}

	if thinSteps == 0 || denseSteps == 0 {
		t.Fatal("прогон не прошёл ни разрежённый, ни плотный участок")
	}

	dense := float64(denseRCS) / float64(denseSteps)
	thin := float64(thinRCS) / float64(thinSteps)

	t.Logf("двигатели ориентации: выше 100 км %.0f %% времени, в плотных слоях "+
		"%.0f %%; наибольшее отклонение угла атаки %.1f°, разброс плавников %.1f°",
		thin*100, dense*100, aoaWorst, flapSpread*57.2957795)

	if dense > 0.25 {
		t.Errorf("в плотных слоях двигатели ориентации работают %.0f %% времени — "+
			"корпус там обязаны держать плавники", dense*100)
	}
	if aoaWorst > 5 {
		t.Errorf("угол атаки ушёл от семидесяти на %.1f° — плавники не удержали "+
			"корпус", aoaWorst)
	}
}
