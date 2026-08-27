package simulator

import (
	"testing"

	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Угловая скорость бустера не должна уходить в численный разнос.
//
// Автопилот возврата может не справиться с манёвром — недостаточная власть
// решётчатых рулей и двигателей ориентации против аэродинамического момента
// статически неустойчивого корпуса на сверхзвуке реалистична сама по себе.
// Но за пределом применимости дискретизации (шаг интегрирования вращения
// рассчитан на постоянную времени в доли секунды) решение перестаёт иметь
// физический смысл и растёт без остановки — см. boosterMaxSpinRate.
// -----------------------------------------------------------------------------

func TestBoosterSpinNeverExceedsStructuralLimit(t *testing.T) {
	if testing.Short() {
		t.Skip("длинный прогон")
	}

	m, _ := vehicle.MissionByID("transatmospheric")
	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithSeed(m.Apply(p.Config()), DefaultTimeConfig(), 1)

	maxOmega := 0.0
	sawBooster := false

	for i := 0; i < 400000; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		var alive bool
		if sim.booster != nil {
			sawBooster = true
			alive = sim.booster.Alive()
			if o := sim.booster.attitude.Omega.Norm(); o > maxOmega {
				maxOmega = o
			}
		}
		crashed := sim.crashed
		sim.mu.Unlock()

		if sim.booster != nil && !alive {
			break
		}
		if crashed {
			break
		}
	}

	if !sawBooster {
		t.Fatal("бустер ни разу не появился — сценарий не дошёл до разделения")
	}

	// Небольшой запас над самим пределом: разрушение фиксируется по факту
	// превышения на границе такта, следующий шаг ещё успевает чуть разогнаться
	// до того, как Alive() станет false.
	const tolerance = 1.5
	if maxOmega > boosterMaxSpinRate*tolerance {
		t.Errorf("угловая скорость бустера ушла в численный разнос: %.3f рад/с "+
			"(предел разрушения %.1f)", maxOmega, boosterMaxSpinRate)
	}
}
