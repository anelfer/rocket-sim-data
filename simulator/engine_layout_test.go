package simulator

import (
	"testing"

	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Компоновка блока двигателей, дросселирование и удельный импульс.
// -----------------------------------------------------------------------------

func TestEngineLayoutNeighbourClassification(t *testing.T) {
	cfg := vehicle.Falcon9Config()
	engines := vehicle.BuildFirstStageEngines(cfg, 288)

	if len(engines) != 9 {
		t.Fatalf("ожидалось 9 двигателей, получено %d", len(engines))
	}
	if engines[0].Radius != 0 {
		t.Error("двигатель 1 должен быть центральным")
	}

	// Центральный двигатель не имеет ни соседей, ни противоположного.
	for i := 1; i < len(engines); i++ {
		if vehicle.IsOpposite(engines[0], engines[i]) || vehicle.IsNeighbor(engines[0], engines[i]) {
			t.Errorf("центральный двигатель не может быть соседним или противоположным %d", i+1)
		}
	}

	// В кольце из восьми двигателей шаг 45°: сосед через один, противоположный
	// — через четыре.
	ring1, ring2, ring5 := engines[1], engines[2], engines[5]
	if !vehicle.IsNeighbor(ring1, ring2) {
		t.Error("двигатели 2 и 3 должны быть соседними (45°)")
	}
	if !vehicle.IsOpposite(ring1, ring5) {
		t.Error("двигатели 2 и 6 должны быть противоположными (180°)")
	}
	if vehicle.IsNeighbor(ring1, ring5) {
		t.Error("противоположный двигатель не может быть соседним")
	}
}

// -----------------------------------------------------------------------------
// E1/E2: параметры двигателей
// -----------------------------------------------------------------------------

func TestMinThrottleIsRealistic(t *testing.T) {
	cfg := vehicle.Falcon9Config()
	engines := vehicle.BuildFirstStageEngines(cfg, 288)

	ratio := engines[0].MinThrust / engines[0].MaxThrust
	if ratio < 0.3 || ratio > 0.5 {
		t.Errorf("минимальный газ = %.2f от номинала, ожидалось 0.3…0.5", ratio)
	}
}

func TestVacuumEngineISPDoesNotDropAtSeaLevel(t *testing.T) {
	cfg := vehicle.Falcon9Config()
	engines := vehicle.BuildSecondStageEngine(cfg, 288)
	e := engines[0]

	e.UpdateISP(101325)
	atSeaLevel := e.ISP
	e.UpdateISP(0)
	inVacuum := e.ISP

	approx(t, inVacuum, cfg.SecondStage.VacuumISP, 1e-9, "удельный импульс в вакууме")
	approx(t, atSeaLevel, cfg.SecondStage.VacuumISP, 1e-9,
		"удельный импульс вакуумного двигателя не зависит от высоты")
}

func TestFirstStageISPVariesWithPressure(t *testing.T) {
	cfg := vehicle.Falcon9Config()
	e := vehicle.BuildFirstStageEngines(cfg, 288)[0]

	e.UpdateISP(101325)
	sea := e.ISP
	e.UpdateISP(0)
	vac := e.ISP

	approx(t, sea, cfg.FirstStage.SeaLevelISP, 1e-9, "удельный импульс на уровне моря")
	approx(t, vac, cfg.FirstStage.VacuumISP, 1e-9, "удельный импульс в вакууме")
}

// -----------------------------------------------------------------------------
// Max-Q
// -----------------------------------------------------------------------------

func TestEngineIDsAreUniqueAcrossStages(t *testing.T) {
	cfg := vehicle.Falcon9Config()

	first := vehicle.BuildFirstStageEngines(cfg, 288)
	second := vehicle.BuildSecondStageEngine(cfg, 288)

	seen := map[string]bool{}
	for _, e := range append(append([]vehicle.Engine{}, first...), second...) {
		if e.ID == "" {
			t.Fatal("идентификатор двигателя не задан")
		}
		if seen[e.ID] {
			t.Errorf("идентификатор %q повторяется в разных ступенях", e.ID)
		}
		seen[e.ID] = true
	}

	if first[0].ID != "S1-1" {
		t.Errorf("первый двигатель первой ступени = %q, ожидался S1-1", first[0].ID)
	}
	if second[0].ID != "S2-1" {
		t.Errorf("двигатель второй ступени = %q, ожидался S2-1", second[0].ID)
	}
}

// После разделения в телеметрии остаются только двигатели активной ступени.
