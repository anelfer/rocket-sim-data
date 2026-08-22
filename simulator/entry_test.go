package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Возвращение корабля.
//
// Проверяется не «долетел или нет», а физика торможения: корабль обязан гасить
// первую космическую атмосферой, а не двигателями, и делать это высоко, за счёт
// площади борта. Если бы вход считался по миделю, как полёт носом вперёд,
// корабль влетал бы в плотные слои на семи километрах в секунду.
// -----------------------------------------------------------------------------

// flyToEntry доводит корабль до орбиты, командует сход и возвращает сводку
// прохождения атмосферы.
type entrySummary struct {
	reachedOrbit  bool
	deployedAt    float64 // высота выпуска плавников, м
	peakQ         float64 // наибольший скоростной напор, Па
	peakHeat      float64 // наибольший тепловой поток, Вт/м²
	speedAt20km   float64 // скорость на двадцати километрах, м/с
	aoaHigh       float64 // угол атаки в разрежённых слоях, градусы
	stowedOnBoost bool    // плавники прижаты на выведении
}

func flyToEntry(t *testing.T) entrySummary {
	t.Helper()

	sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
	out := entrySummary{stowedOnBoost: true, speedAt20km: math.NaN()}

	requested := false
	for i := 0; i < 60000; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		phase := sim.phase
		alt := sim.state.Altitude()
		set := sim.attitude.Surfaces
		deployed := set != nil && set.Deployed
		sim.mu.Unlock()

		if phase == orbit.PhaseSecondStage && deployed {
			out.stowedOnBoost = false
		}

		if !requested && phase == orbit.PhaseOrbital {
			out.reachedOrbit = true
			sim.RequestEntry()
			requested = true
		}

		if deployed && out.deployedAt == 0 {
			out.deployedAt = alt
		}

		sim.mu.RLock()
		tel := sim.buildTelemetryLocked()
		sim.mu.RUnlock()

		if tel.DynamicPressure > out.peakQ {
			out.peakQ = tel.DynamicPressure
		}
		if tel.HeatFlux > out.peakHeat {
			out.peakHeat = tel.HeatFlux
		}
		if phase == orbit.PhaseEntry && alt > 80000 {
			out.aoaHigh = tel.TotalAoA
		}
		if math.IsNaN(out.speedAt20km) && phase == orbit.PhaseEntry && alt < 20000 {
			out.speedAt20km = tel.TotalVelocity
		}

		sim.mu.RLock()
		done := sim.crashed
		sim.mu.RUnlock()
		if phase == orbit.PhaseLanded || done {
			break
		}
	}
	return out
}

func TestShipEntryBrakesInAtmosphere(t *testing.T) {
	if testing.Short() {
		t.Skip("длинный прогон")
	}

	e := flyToEntry(t)

	if !e.reachedOrbit {
		t.Fatal("корабль не вышел на орбиту, входить не с чего")
	}
	if !e.stowedOnBoost {
		t.Error("плавники раскрыты на выведении — на максимальном напоре " +
			"это опрокинет носитель")
	}
	if e.deployedAt < 100000 {
		t.Errorf("плавники выпущены на %.0f км — поздно, вход начинается выше",
			e.deployedAt/1000)
	}

	// Угол атаки в разрежённых слоях держится автопилотом почти точно:
	// возмущать его там нечему.
	if math.Abs(e.aoaHigh-70) > 10 {
		t.Errorf("угол атаки в разрежённых слоях %.1f°, ожидался около 70°", e.aoaHigh)
	}

	// Торможение атмосферой: к двадцати километрам от первой космической
	// должна остаться малая доля.
	if math.IsNaN(e.speedAt20km) {
		t.Fatal("корабль не дошёл до двадцати километров")
	}
	if e.speedAt20km > 1500 {
		t.Errorf("на двадцати километрах скорость %.0f м/с — атмосфера "+
			"не затормозила корабль", e.speedAt20km)
	}

	// Напор и поток должны попадать в порядки, известные для входа
	// с низкой орбиты.
	if e.peakQ < 10000 || e.peakQ > 120000 {
		t.Errorf("наибольший скоростной напор %.0f кПа вне разумного", e.peakQ/1000)
	}
	if e.peakHeat < 3e5 || e.peakHeat > 5e6 {
		t.Errorf("наибольший тепловой поток %.0f кВт/м² вне разумного", e.peakHeat/1000)
	}

	t.Logf("вход: плавники на %.0f км, угол атаки %.1f°, напор до %.0f кПа, "+
		"поток до %.0f кВт/м², на 20 км %.0f м/с",
		e.deployedAt/1000, e.aoaHigh, e.peakQ/1000, e.peakHeat/1000, e.speedAt20km)
}

// Плавник, взятый оператором, автопилоту не подчиняется.
func TestManualFlapIgnoresAutopilot(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)

	// Доводим до второй ступени, где плавники появляются.
	for i := 0; i < 2000 && sim.attitude.Surfaces == nil; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.mu.Unlock()
	}
	if sim.attitude.Surfaces == nil {
		t.Fatal("плавники не появились со второй ступенью")
	}

	sim.DeployFlaps(true)
	if !sim.SetFlap("aft_left", 40) {
		t.Fatal("команда плавнику не принята")
	}

	for i := 0; i < 200; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.mu.Unlock()
	}

	sim.mu.RLock()
	set := sim.attitude.Surfaces
	idx := set.IndexOf("aft_left")
	command := set.Command[idx] * 57.2957795
	manual := set.Manual[idx]
	sim.mu.RUnlock()

	if !manual {
		t.Error("плавник не отмечен как ручной")
	}
	if math.Abs(command-40) > 0.5 {
		t.Errorf("автопилот перебил ручную команду: %.1f° вместо 40°", command)
	}

	sim.ReleaseFlap("aft_left")
	for i := 0; i < 100; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.mu.Unlock()
	}

	sim.mu.RLock()
	back := sim.attitude.Surfaces.Manual[idx]
	sim.mu.RUnlock()

	if back {
		t.Error("плавник не вернулся автопилоту")
	}
}

// Принудительная посадка недоступна первой ступени, доступна кораблю в любой
// момент его полёта (а не только с орбиты) и не принимается повторно.
func TestRequestLandingWorksMidAscentAfterStaging(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)

	if err := sim.RequestLanding(); err == nil {
		t.Fatal("посадка принята до разделения ступеней — доступна только кораблю")
	}

	// Доводим до второй ступени, всё ещё работающей на разгон: посадка
	// запрашивается посреди этого участка, а не дожидаясь SECO или орбиты.
	for i := 0; i < 60000 && sim.attitude.Surfaces == nil; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.mu.Unlock()
	}

	sim.mu.RLock()
	stage := sim.stage
	phaseBefore := sim.phase
	sim.mu.RUnlock()
	if stage != 2 {
		t.Fatal("не дождались разделения ступеней")
	}
	if phaseBefore != orbit.PhaseSecondStage {
		t.Fatalf("ожидалась работающая вторая ступень, получена фаза %v", phaseBefore)
	}

	if err := sim.RequestLanding(); err != nil {
		t.Fatalf("посадка не принята посреди работы второй ступени: %v", err)
	}

	sim.mu.RLock()
	phaseAfter := sim.phase
	running := vehicle.RunningEngines(sim.engines)
	sim.mu.RUnlock()

	if phaseAfter != orbit.PhaseEntry {
		t.Errorf("фаза %v сразу после команды посадки, ожидался PhaseEntry", phaseAfter)
	}
	if running != 0 {
		t.Errorf("%d двигателей продолжают работать после команды посадки", running)
	}

	if err := sim.RequestLanding(); err == nil {
		t.Error("повторная команда посадки принята, хотя посадка уже идёт")
	}

	// Небольшой прогон вперёд: программа обязана остаться в цепочке фаз
	// возвращения и не терять носитель сразу после нестандартной точки старта.
	for i := 0; i < 200; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		phase := sim.phase
		crashed := sim.crashed
		sim.mu.Unlock()

		if crashed {
			t.Fatalf("носитель потерян через %.1f с после команды посадки", float64(i)*0.1)
		}
		switch phase {
		case orbit.PhaseEntry, orbit.PhaseLandingFlip, orbit.PhaseLandingBurn,
			orbit.PhaseLanded:
		default:
			t.Fatalf("фаза ушла из цепочки возвращения: %v", phase)
		}
	}
}
