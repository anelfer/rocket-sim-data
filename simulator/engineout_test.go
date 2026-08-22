package simulator

import (
	"testing"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Компенсация отказа двигателя.
//
// Бортовой контур управляет тягой ступени, а не отдельного двигателя. Когда
// один агрегат выбывает, оставшиеся обязаны добрать тягу за него — насколько
// позволяет аттестованный запас форсирования. Полностью потеря девятой части
// тяги не компенсируется, и модель должна показывать это честно, а не делать
// вид, что ничего не произошло.
// -----------------------------------------------------------------------------

// propulsionState возвращает состояние компенсации.
func propulsionState(s *Simulation) (demand, perEngine, deficit float64, out int) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	p := s.propulsion
	return p.ThrottleDemand, p.EngineThrottle, p.ThrustDeficit, p.EnginesOut
}

// При отказе одного двигателя оставшиеся форсируются и полностью
// закрывают потерю.
//
// Штатный полёт держит газ на 93.5 % (см. GNCConfig.NominalThrottle), а не
// на паспортных ста процентах, — запас форсирования до 108 % существует
// как раз для такого случая. Потеря одного двигателя из девяти требует
// 0.935·9/8 ≈ 1.052, что укладывается в запас: девятая доля тяги
// компенсируется целиком, дефицита нет. Это не ослабленная проверка,
// а как раз то поведение, ради которого запас и существует, — Falcon 9
// действительно переживает одиночный отказ практически без потерь.
func TestEngineOutRaisesThrottleOfSurvivors(t *testing.T) {
	s := runningSimulation(t)

	_, before, _, out := propulsionState(s)
	if out != 0 {
		t.Fatalf("до вмешательства не работают %d двигателей", out)
	}
	thrustBefore := totalThrust(s)
	perEngineBefore := engineThrusts(s)["S1-1"]

	mustApply(t, s, control.Command{
		ID: "out", Parameter: "tp.shaft.speed", Engine: "S1-4",
		Mode: control.ModeFailure, Failure: control.FailureStopped,
	})
	runFor(s, 12)

	demand, after, deficit, out := propulsionState(s)
	if out != 1 {
		t.Fatalf("выбывших двигателей %d, ожидался один", out)
	}
	if after <= before {
		t.Errorf("оставшиеся не форсированы: газ %.3f → %.3f", before, after)
	}

	perEngineAfter := engineThrusts(s)["S1-1"]
	if perEngineAfter <= perEngineBefore {
		t.Errorf("тяга исправного двигателя не выросла: %.0f → %.0f кН",
			perEngineBefore/1000, perEngineAfter/1000)
	}

	// Один из девяти при штатных 93.5 % компенсируется целиком: запаса
	// форсирования (до 108 %) на это хватает с остатком.
	if deficit > 0.005 {
		t.Errorf("дефицит тяги %.2f %% при потере одного двигателя из девяти — "+
			"запас форсирования должен закрывать эту потерю целиком", deficit*100)
	}

	t.Logf("Отказ S1-4: газ %.3f → %.3f (требуется %.3f), дефицит %.2f %%, "+
		"тяга двигателя %.0f → %.0f кН, суммарная %.0f → %.0f кН",
		before, after, demand, deficit*100,
		perEngineBefore/1000, perEngineAfter/1000,
		thrustBefore/1000, totalThrust(s)/1000)
}

// При отказе двух двигателей из девяти запаса форсирования уже не хватает,
// и дефицит обязан быть виден честно, а не скрыт.
func TestTwoEnginesOutShowsHonestDeficit(t *testing.T) {
	s := runningSimulation(t)

	for _, id := range []string{"S1-4", "S1-7"} {
		mustApply(t, s, control.Command{
			ID: "out-" + id, Parameter: "tp.shaft.speed", Engine: id,
			Mode: control.ModeFailure, Failure: control.FailureStopped,
		})
	}
	runFor(s, 12)

	_, perEngine, deficit, out := propulsionState(s)
	if out != 2 {
		t.Fatalf("выбывших двигателей %d, ожидалось два", out)
	}

	s.mu.RLock()
	limit := s.propulsion.Engines[0].Config.MaxThrottle
	s.mu.RUnlock()
	if perEngine > limit+1e-9 {
		t.Errorf("газ %.3f выше аттестованного предела %.3f", perEngine, limit)
	}

	// Семь двигателей на пределе форсирования дают 7·1.08 = 7.56 против
	// требуемых 0.935·9 = 8.415 — дефицит около 10 %.
	if deficit < 0.08 || deficit > 0.13 {
		t.Errorf("дефицит тяги %.1f %%, ожидалось около 10 %%", deficit*100)
	}
	t.Logf("Два двигателя из девяти выбыли: газ на предел %.3f, дефицит %.1f %%",
		perEngine, deficit*100)
}

// Форсирование ограничено аттестованным запасом.
func TestCompensationRespectsUprateLimit(t *testing.T) {
	s := runningSimulation(t)

	// Выбивается треть блока: добрать такую потерю невозможно.
	for _, id := range []string{"S1-2", "S1-5", "S1-8"} {
		mustApply(t, s, control.Command{
			ID: "out-" + id, Parameter: "tp.shaft.speed", Engine: id,
			Mode: control.ModeFailure, Failure: control.FailureStopped,
		})
	}
	runFor(s, 12)

	_, perEngine, deficit, out := propulsionState(s)
	if out != 3 {
		t.Fatalf("выбывших двигателей %d, ожидалось три", out)
	}

	s.mu.RLock()
	limit := s.propulsion.Engines[0].Config.MaxThrottle
	s.mu.RUnlock()

	if perEngine > limit+1e-9 {
		t.Errorf("газ %.3f выше аттестованного предела %.3f", perEngine, limit)
	}
	// Шесть двигателей на пределе форсирования дают 6·1.08 = 6.48 против
	// требуемых при штатных 93.5 % газа 0.935·9 = 8.415 — дефицит около 23 %.
	if deficit < 0.20 || deficit > 0.26 {
		t.Errorf("дефицит тяги %.1f %%, ожидалось около 23 %%", deficit*100)
	}
	t.Logf("Три двигателя из девяти выбыли: газ на предел %.3f, дефицит %.1f %%, "+
		"суммарная тяга %.0f кН", perEngine, deficit*100, totalThrust(s)/1000)
}

// Без отказов форсирования быть не должно.
func TestNoCompensationWhenAllEnginesRun(t *testing.T) {
	s := runningSimulation(t)
	runFor(s, 5)

	demand, perEngine, deficit, out := propulsionState(s)
	if out != 0 {
		t.Fatalf("выбывших двигателей %d", out)
	}
	if perEngine != demand {
		t.Errorf("газ на двигатель %.4f отличается от требуемого %.4f при исправном блоке",
			perEngine, demand)
	}
	if deficit != 0 {
		t.Errorf("дефицит тяги %.4f при исправном блоке", deficit)
	}
}

// Возврат двигателя в строй снимает форсирование.
func TestCompensationEndsWhenEngineReturns(t *testing.T) {
	s := runningSimulation(t)

	mustApply(t, s, control.Command{
		ID: "out", Parameter: "tp.shaft.speed", Engine: "S1-7",
		Mode: control.ModeFailure, Failure: control.FailureStopped,
	})
	runFor(s, 12)

	_, forced, _, out := propulsionState(s)
	if out != 1 {
		t.Fatalf("двигатель не выбыл")
	}

	s.RestoreNominal()
	runFor(s, 12)

	demand, restored, deficit, out := propulsionState(s)
	if out != 0 {
		t.Errorf("после возврата в строй выбывших двигателей %d", out)
	}
	if restored >= forced {
		t.Errorf("форсирование не снято: газ %.3f → %.3f", forced, restored)
	}
	if deficit != 0 {
		t.Errorf("дефицит тяги %.4f после восстановления", deficit)
	}
	t.Logf("Возврат в строй: газ %.3f → %.3f при требуемом %.3f",
		forced, restored, demand)
}

// Отказ двигателя должен растягивать работу ступени, а не срывать выведение.
func TestEngineOutExtendsBurn(t *testing.T) {
	if testing.Short() {
		t.Skip("длительный прогон")
	}

	meco := func(fail bool) float64 {
		s := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), 20260729)
		for i := 0; i < 20000; i++ {
			s.mu.Lock()
			s.step(s.Time.Step)
			phase := s.phase
			elapsed := s.elapsed
			s.mu.Unlock()

			if fail && elapsed >= 40 && elapsed < 40+s.Time.Step {
				s.Board().Apply(control.Command{
					ID: "out", Parameter: "tp.shaft.speed", Engine: "S1-4",
					Mode: control.ModeFailure, Failure: control.FailureStopped,
				}, elapsed, nil)
			}
			if phase.String() != "First Stage Burn" && elapsed > 10 {
				return elapsed
			}
		}
		return -1
	}

	nominal := meco(false)
	degraded := meco(true)

	if nominal < 0 || degraded < 0 {
		t.Fatal("первая ступень не отработала")
	}
	if degraded <= nominal {
		t.Errorf("с выбывшим двигателем ступень отработала не дольше: "+
			"%.1f против %.1f с", degraded, nominal)
	}
	t.Logf("Работа первой ступени: штатно %.1f с, с выбывшим двигателем %.1f с "+
		"(+%.1f с)", nominal, degraded, degraded-nominal)
}
