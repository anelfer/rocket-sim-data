package simulator

import (
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Горячее разделение.
//
// Смысл схемы в том, что связка ни на секунду не остаётся без тяги: бустер
// гасит всё, кроме трёх центральных камер, корабль зажигает свои двигатели
// прямо в связке, и расхождение идёт под тягой. Проверяется именно это —
// что на участке разделения тяга не падает до нуля и что число работающих
// камер меняется в правильном порядке.
// -----------------------------------------------------------------------------

func TestHotStaging(t *testing.T) {
	if testing.Short() {
		t.Skip("длинный прогон")
	}

	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithSeed(p.Config(), DefaultTimeConfig(), 1)

	var (
		boosterEngines int     // камер бустера до дросселирования
		hotEngines     = 99    // камер на участке разделения
		shipEngines    int     // камер корабля после расхождения
		zeroThrust     bool    // была ли связка без тяги
		minThrust      = 1.0e9 // наименьшая тяга на разделении, Н
	)

	for i := 0; i < 4000; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.telemetry = sim.buildTelemetryLocked()
		phase := sim.phase
		thrust := sim.telemetry.TotalThrust
		running := 0
		for i := range sim.engines {
			if sim.engines[i].Running {
				running++
			}
		}
		sim.mu.Unlock()

		switch phase {
		case orbit.PhaseFirstStage:
			boosterEngines = running

		case orbit.PhaseCoast1:
			if running > 0 && running < hotEngines {
				hotEngines = running
			}
			if thrust < minThrust {
				minThrust = thrust
			}
			if thrust <= 0 {
				zeroThrust = true
			}

		case orbit.PhaseSecondStage:
			shipEngines = running
		}

		if phase == orbit.PhaseSecondStage && sim.ModelTime() > 200 {
			break
		}
	}

	t.Logf("камер: бустер %d → на разделении %d → корабль %d; "+
		"наименьшая тяга на разделении %.2f МН",
		boosterEngines, hotEngines, shipEngines, minThrust/1e6)

	if boosterEngines != 33 {
		t.Errorf("на выведении работают %d камер вместо тридцати трёх",
			boosterEngines)
	}
	if hotEngines != hotStageEngines {
		t.Errorf("на разделении работают %d камер вместо %d",
			hotEngines, hotStageEngines)
	}
	if shipEngines != 6 {
		t.Errorf("корабль зажёг %d камер вместо шести (три обычных "+
			"и три вакуумных)", shipEngines)
	}
	if zeroThrust {
		t.Error("связка осталась без тяги: в горячем разделении весь смысл " +
			"в том, чтобы этого не произошло")
	}
	// Три камеры на нижнем пределе дросселирования дают около трёх меганьютонов.
	// Меньше — значит камеры не держат режим.
	if minThrust < 2.5e6 {
		t.Errorf("тяга на разделении просела до %.2f МН: три камеры Raptor "+
			"даже на нижнем пределе дают около трёх", minThrust/1e6)
	}
}
