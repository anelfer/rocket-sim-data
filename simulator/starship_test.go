package simulator

import (
	"testing"

	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Диагностический прогон Starship.
//
// Тест ничего не утверждает: он печатает трассу полёта, по которой видно,
// замыкается ли орбита и с каким остатком топлива. Утверждать здесь пока
// нечего — программа тангажа задана таблицей, подобранной под Falcon 9,
// и точка выведения плавает. Как только вторая ступень получит замкнутый
// контур наведения, отсюда вырастет нормальная проверка.
// -----------------------------------------------------------------------------

func TestStarshipInsertionTrace(t *testing.T) {
	if testing.Short() {
		t.Skip("длинный прогон")
	}

	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithConfig(p.Config(), DefaultTimeConfig())

	last := -1e9
	for tm := 0.0; tm < 1400; tm += 0.1 {
		sim.step(0.1)
		tl := sim.buildTelemetryLocked()

		if sim.ModelTime()-last >= 60 {
			last = sim.ModelTime()
			t.Logf("T+%6.1f %-20s h=%7.1f км V=%6.0f м/с тяга=%5.2f МН m=%6.0f т ap=%7.1f pe=%8.1f",
				sim.ModelTime(), tl.Phase, tl.Altitude/1000, tl.TotalVelocity,
				tl.TotalThrust/1e6, tl.TotalMass/1000,
				float64(tl.Apoapsis)/1000, float64(tl.Periapsis)/1000)
		}

		if tl.OrbitReached || tl.Crashed {
			t.Logf("ФИНАЛ T+%.1f %s: %.1f × %.1f км, наклонение %.2f°, остаток %.0f т",
				sim.ModelTime(), tl.Phase, float64(tl.Apoapsis)/1000,
				float64(tl.Periapsis)/1000, tl.Inclination, tl.FuelMass/1000)
			return
		}
	}

	tl := sim.buildTelemetryLocked()
	t.Logf("Орбита не замкнута: %s h=%.1f км V=%.0f м/с ap=%.1f pe=%.1f остаток %.0f т",
		tl.Phase, tl.Altitude/1000, tl.TotalVelocity, float64(tl.Apoapsis)/1000,
		float64(tl.Periapsis)/1000, tl.FuelMass/1000)
}
