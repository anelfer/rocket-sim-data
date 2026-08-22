package simulator

import (
	"encoding/json"
	"math"
	"testing"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Двигатели как отдельные изделия.
//
// На первой ступени девять двигателей, и у каждого свой турбонасос, своя
// камера и своё сопло. Проверяется, что модель это различает: команда одному
// агрегату не должна задевать остальные восемь.
// -----------------------------------------------------------------------------

// engineStates возвращает обороты вала каждого двигателя, рад/с.
func engineStates(s *Simulation) map[string]float64 {
	s.mu.RLock()
	defer s.mu.RUnlock()

	out := make(map[string]float64, len(s.propulsion.Engines))
	for _, e := range s.propulsion.Engines {
		out[e.ID] = e.Turbopump.Speed
	}
	return out
}

// engineThrusts возвращает тягу каждого двигателя, Н.
func engineThrusts(s *Simulation) map[string]float64 {
	s.mu.RLock()
	defer s.mu.RUnlock()

	out := make(map[string]float64, len(s.propulsion.Engines))
	for _, e := range s.propulsion.Engines {
		out[e.ID] = e.Thrust
	}
	return out
}

// У каждого двигателя должен быть собственный турбонасос.
func TestEachEngineHasOwnTurbopump(t *testing.T) {
	s := runningSimulation(t)

	s.mu.RLock()
	engines := s.propulsion.Engines
	s.mu.RUnlock()

	if len(engines) != 9 {
		t.Fatalf("двигателей %d, ожидалось девять", len(engines))
	}

	seen := make(map[string]bool)
	pumps := make(map[interface{}]bool)
	for _, e := range engines {
		if e.ID == "" {
			t.Error("у двигателя нет идентификатора")
		}
		if seen[e.ID] {
			t.Errorf("идентификатор %q повторяется", e.ID)
		}
		seen[e.ID] = true

		if pumps[e.Turbopump] {
			t.Errorf("двигатель %s делит турбонасос с другим двигателем", e.ID)
		}
		pumps[e.Turbopump] = true
	}

	// Экземпляры не должны быть побитово одинаковыми: приёмочные испытания
	// разбраковывают двигатели в допуске, но не делают их идентичными.
	speeds := engineStates(s)
	var min, max float64 = math.Inf(1), math.Inf(-1)
	for _, v := range speeds {
		min, max = math.Min(min, v), math.Max(max, v)
	}
	spread := (max - min) / max
	if spread == 0 {
		t.Error("все двигатели работают побитово одинаково — разброс не моделируется")
	}
	if spread > 0.05 {
		t.Errorf("разброс оборотов %.1f %% слишком велик для исправного блока", spread*100)
	}
	t.Logf("Разброс оборотов по блоку: %.2f %% (%.0f…%.0f об/мин)",
		spread*100, min*60/(2*math.Pi), max*60/(2*math.Pi))
}

// Команда одному двигателю не должна задевать остальные.
func TestCommandTargetsSingleEngine(t *testing.T) {
	s := runningSimulation(t)
	before := engineStates(s)

	const target = "S1-3"
	mustApply(t, s, control.Command{
		ID: "one", Parameter: "tp.shaft.speed", Engine: target,
		Mode: control.ModeFailure, Failure: control.FailureStopped,
	})

	runFor(s, 8)
	after := engineStates(s)
	thrust := engineThrusts(s)

	if after[target] >= before[target]*0.6 {
		t.Errorf("%s: обороты не упали, %.0f → %.0f рад/с",
			target, before[target], after[target])
	}
	if thrust[target] > 1000 {
		t.Errorf("%s: тяга не пропала, %.0f кН", target, thrust[target]/1000)
	}

	for id, v := range after {
		if id == target {
			continue
		}
		// Соседние двигатели слегка ускоряются: тяга упала, носитель стал
		// легче нагружен, и система управления добавляет газ. Но остановиться
		// они не должны ни в коем случае.
		if v < before[id]*0.9 {
			t.Errorf("%s: обороты просели вслед за соседом, %.0f → %.0f рад/с",
				id, before[id], v)
		}
		if thrust[id] < 500000 {
			t.Errorf("%s: тяга упала до %.0f кН, хотя команда была адресована %s",
				id, thrust[id]/1000, target)
		}
	}

	working := 0
	for id := range after {
		if id != target && thrust[id] > 500000 {
			working++
		}
	}
	t.Logf("Остановлен %s, продолжают работать %d двигателей из 9, "+
		"суммарная тяга %.0f кН",
		target, working, totalThrust(s)/1000)
}

// totalThrust возвращает суммарную тягу блока, Н.
func totalThrust(s *Simulation) float64 {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.propulsion.TotalThrust
}

// Общая команда действует на все двигатели сразу.
func TestCommandWithoutTargetAffectsAll(t *testing.T) {
	s := runningSimulation(t)
	before := engineStates(s)

	mustApply(t, s, control.Command{
		ID: "all", Parameter: "tp.shaft.speed",
		Mode: control.ModeFailure, Failure: control.FailureStopped,
	})

	runFor(s, 8)
	after := engineStates(s)

	for id, v := range after {
		if v >= before[id]*0.6 {
			t.Errorf("%s: обороты не упали, %.0f → %.0f рад/с", id, before[id], v)
		}
	}
}

// Адресная команда перекрывает общую для того же двигателя.
func TestTargetedCommandOverridesGlobal(t *testing.T) {
	s := runningSimulation(t)

	// Общая уставка: минус пятнадцать процентов на всю ступень.
	mustApply(t, s, control.Command{
		ID: "global", Parameter: "tp.shaft.speed",
		Mode: control.ModeStep, Basis: control.BasisPercentNominal, Value: 85,
	})
	// Один двигатель, наоборот, форсирован.
	mustApply(t, s, control.Command{
		ID: "one", Parameter: "tp.shaft.speed", Engine: "S1-5",
		Mode: control.ModeStep, Basis: control.BasisPercentNominal, Value: 108,
	})

	runFor(s, 8)
	after := engineStates(s)

	forced := after["S1-5"]
	for id, v := range after {
		if id == "S1-5" {
			continue
		}
		if forced <= v {
			t.Errorf("форсированный S1-5 (%.0f рад/с) не быстрее %s (%.0f рад/с)",
				forced, id, v)
		}
	}
	t.Logf("Ступень на 85 %% оборотов, S1-5 на 108 %%: %.0f против %.0f об/мин",
		forced*60/(2*math.Pi), after["S1-1"]*60/(2*math.Pi))
}

// Снятие адресного воздействия не должно снимать общее и наоборот.
func TestReleaseIsPerTarget(t *testing.T) {
	s := runningSimulation(t)

	mustApply(t, s, control.Command{
		ID: "g", Parameter: "engine.throttle", Mode: control.ModeStep, Value: 0.8,
	})
	mustApply(t, s, control.Command{
		ID: "e", Parameter: "engine.throttle", Engine: "S1-2",
		Mode: control.ModeStep, Value: 0.5,
	})
	if n := len(s.Board().Effects()); n != 2 {
		t.Fatalf("активных воздействий %d, ожидалось два", n)
	}

	s.Board().Apply(control.Command{
		ID: "rel", Parameter: "engine.throttle", Engine: "S1-2",
		Mode: control.ModeRelease,
	}, s.ModelTime(), s.ControlSample())

	effects := s.Board().Effects()
	if len(effects) != 1 {
		t.Fatalf("после снятия адресного воздействия осталось %d, ожидалось одно",
			len(effects))
	}
	if effects[0].Engine != "" {
		t.Error("снятие адресного воздействия убрало общее")
	}
}

// Снимок для интерфейса должен содержать все двигатели и их насосы.
func TestSnapshotCoversAllEngines(t *testing.T) {
	s := runningSimulation(t)
	snap := s.ControlSnapshot()

	if len(snap.Engines) != 9 {
		t.Errorf("в снимке %d двигателей, ожидалось девять", len(snap.Engines))
	}
	if len(snap.Pumps) != 18 {
		t.Errorf("в снимке %d насосов, ожидалось восемнадцать", len(snap.Pumps))
	}

	primary := 0
	ids := make(map[string]bool)
	for _, e := range snap.Engines {
		ids[e.ID] = true
		if e.Primary {
			primary++
		}
		if !e.ShaftRPM.Defined() || e.ShaftRPM.Value() <= 0 {
			t.Errorf("%s: обороты не определены", e.ID)
		}
	}
	if len(ids) != 9 {
		t.Error("идентификаторы двигателей в снимке повторяются")
	}
	if primary != 1 {
		t.Errorf("ведущих двигателей %d, ожидался один", primary)
	}

	for _, p := range snap.Pumps {
		if p.Engine == "" {
			t.Error("насос не привязан к двигателю")
		}
	}
}

// Телеметрия всех двигателей обязана сериализоваться.
func TestAllEnginesTelemetrySerialises(t *testing.T) {
	s := runningSimulation(t)

	// Один двигатель останавливается: у остановленного агрегата часть величин
	// не определена, и именно на них сериализация ломалась бы.
	mustApply(t, s, control.Command{
		ID: "stop", Parameter: "tp.shaft.speed", Engine: "S1-7",
		Mode: control.ModeFailure, Failure: control.FailureStopped,
	})

	for i := 0; i < 120; i++ {
		runFor(s, 0.5)
		if _, err := json.Marshal(s.ControlSnapshot()); err != nil {
			t.Fatalf("снимок не сериализуется на T+%.1f с: %v", s.ModelTime(), err)
		}
	}
}

// Баки должны пустеть одновременно, иначе остаток второго компонента —
// мёртвый груз.
//
// Соотношение в камере зависит от давлений подачи и по ходу работы ступени
// плывёт. Без системы управления расходованием у Starship окислитель кончался,
// когда в баке оставалось двадцать восемь тонн метана, — этого хватило бы
// на довыведение, но сжечь их было не с чем.
func TestPropellantUtilizationDrainsTanksTogether(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)

	worst := 0.0
	for i := 0; i < 4500; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.mu.Unlock()

		sim.mu.RLock()
		fuel := sim.propulsion.FuelTank.FillFraction()
		ox := sim.propulsion.OxTank.FillFraction()
		running := sim.propulsion.RunningEngines()
		sim.mu.RUnlock()

		// Смотрим только работающую ступень и только после выхода на режим.
		if running == 0 || (fuel > 0.95 && ox > 0.95) {
			continue
		}
		if d := math.Abs(fuel - ox); d > worst {
			worst = d
		}
	}

	if worst > 0.05 {
		t.Errorf("баки расходуются вразнобой: наибольшее расхождение остатков %.1f %%",
			worst*100)
	}
	t.Logf("наибольшее расхождение остатков в баках %.1f %%", worst*100)
}
