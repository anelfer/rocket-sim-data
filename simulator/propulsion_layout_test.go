package simulator

import (
	"testing"

	"rocketTelemetrySim/simulator/vehicle"
)

// Двигательная установка должна собираться по профилю, включая смешанные
// группы двигателей на второй ступени.
func TestStarshipPropulsionLayout(t *testing.T) {
	cfg := vehicle.StarshipConfig()
	s := NewSimulationWithSeed(cfg, DefaultTimeConfig(), 1)

	s.mu.RLock()
	first := s.propulsion.Engines
	s.mu.RUnlock()

	if len(first) != 33 {
		t.Errorf("на первой ступени %d двигателей, ожидалось 33", len(first))
	}
	for _, e := range first {
		if !e.Config.ClosedCycle {
			t.Errorf("%s: схема открытая, ожидалась замкнутая", e.ID)
		}
		if e.Config.Pair.Fuel.Name != "CH4" {
			t.Errorf("%s: горючее %q, ожидался метан", e.ID, e.Config.Pair.Fuel.Name)
		}
	}

	// Вторая ступень: три атмосферных двигателя и три вакуумных.
	groups := vehicle.SecondStageGroups(cfg)
	if n := vehicle.SecondStagePrimaryCount(cfg); n != 3 {
		t.Errorf("атмосферных двигателей второй ступени %d, ожидалось 3", n)
	}
	if len(groups) != 1 || groups[0].Count != 3 {
		t.Fatalf("вакуумных двигателей второй ступени %v, ожидалось 3", groups)
	}
	if groups[0].Config.Nozzle.ExitArea <= groups[0].Config.Nozzle.InitialThroatArea*50 {
		t.Error("у вакуумного двигателя степень расширения должна быть заметно больше")
	}
}

// В замкнутой схеме топливо за борт не выбрасывается.
func TestClosedCycleWastesNoPropellant(t *testing.T) {
	s := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
	for i := 0; i < 300; i++ {
		s.mu.Lock()
		s.step(s.Time.Step)
		s.mu.Unlock()
	}

	s.mu.RLock()
	e := s.propulsion.PrimaryEngine()
	s.mu.RUnlock()

	if e.GasGeneratorFuelFlow != 0 || e.GasGeneratorOxFlow != 0 {
		t.Errorf("в замкнутой схеме расход за борт %.2f/%.2f кг/с",
			e.GasGeneratorFuelFlow, e.GasGeneratorOxFlow)
	}
	if e.GasGeneratorThrust != 0 {
		t.Errorf("в замкнутой схеме выхлоп даёт тягу %.0f Н", e.GasGeneratorThrust)
	}
	if e.Config.OverallMixtureRatio() != e.Config.Pair.OptimalMixtureRatio {
		t.Errorf("соотношение заправки %.3f отличается от камерного %.3f, "+
			"хотя других потребителей нет",
			e.Config.OverallMixtureRatio(), e.Config.Pair.OptimalMixtureRatio)
	}
	t.Logf("Замкнутая схема: за борт 0 кг/с, соотношение заправки %.2f, "+
		"тяга двигателя %.0f кН, Isp %.1f с",
		e.Config.OverallMixtureRatio(), e.Thrust/1000, e.SpecificImpulse)
}
