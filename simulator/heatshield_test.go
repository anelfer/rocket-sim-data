package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Теплозащита корабля.
//
// Проверяется главное: ориентация обязана решать судьбу аппарата. Один и тот же
// поток при полёте брюхом вперёд приходится на плитки и уносится излучением,
// а при полёте спиной — на голую сталь, которая столько не держит. Если модель
// этой разницы не видит, разворот корабля остаётся картинкой на экране.
// -----------------------------------------------------------------------------

// heatFor прогоняет теплозащиту под заданной ориентацией и возвращает её.
func heatFor(belly physics.Vec3, flux, seconds float64) *HeatShield {
	h := NewHeatShield(52, 9, 250)

	// Корабль падает вдоль −Z инерциальных осей: поток набегает снизу.
	air := physics.Vec3{Z: -7000}

	for t := 0.0; t < seconds; t += 0.1 {
		h.Update(0.1, belly, air, flux)
	}
	return h
}

func TestHeatShieldProtectsOnlyTheSideItCovers(t *testing.T) {
	const flux = 1.0e6 // Вт/м², порядок входа с низкой орбиты

	// Брюхо вниз, то есть навстречу потоку: подставлены плитки.
	belly := heatFor(physics.Vec3{Z: -1}, flux, 400)
	if belly.Exposure < 0.99 {
		t.Fatalf("подставленность брюха %.2f, ожидалась единица", belly.Exposure)
	}
	if belly.Windward.Temperature < belly.Leeward.Temperature {
		t.Error("подставленная потоку сторона холоднее затенённой")
	}
	if belly.Leeward.Damage > 0 {
		t.Errorf("сталь спины повреждена при полёте брюхом вперёд: %.2f",
			belly.Leeward.Damage)
	}
	if belly.BurnedThrough {
		t.Errorf("корпус прогорел при штатной ориентации: плитки %.0f К, "+
			"сталь %.0f К", belly.Windward.Temperature, belly.Leeward.Temperature)
	}

	// Тот же поток, но корабль развёрнут спиной.
	back := heatFor(physics.Vec3{Z: 1}, flux, 400)
	if back.Exposure > -0.99 {
		t.Fatalf("подставленность брюха %.2f, ожидалась минус единица", back.Exposure)
	}
	if back.Leeward.Temperature <= belly.Leeward.Temperature {
		t.Error("сталь спины не нагрелась, хотя корабль идёт спиной к потоку")
	}
	if !back.BurnedThrough {
		t.Errorf("голая сталь выдержала вход спиной вперёд: %.0f К при пределе "+
			"%.0f К, повреждение %.2f", back.Leeward.Temperature,
			back.Leeward.Limit, back.Leeward.Damage)
	}

	t.Logf("брюхом вперёд: плитки %.0f К, сталь %.0f К; спиной: сталь %.0f К",
		belly.Windward.Temperature, belly.Leeward.Temperature,
		back.Leeward.Temperature)
}

// Излучение обязано уносить тепло: без него любая сторона греется без предела.
func TestHeatShieldCoolsByRadiation(t *testing.T) {
	const flux = 3.0e5

	short := heatFor(physics.Vec3{Z: -1}, flux, 200)
	long := heatFor(physics.Vec3{Z: -1}, flux, 1200)

	// Установившаяся температура — та, при которой излучение равно приходу.
	// Борт получает долю потока критической точки, её и берём.
	steady := math.Pow(0.30*flux/(0.85*physics.StefanBoltzmann), 0.25)

	if long.Windward.Temperature > steady*1.05 {
		t.Errorf("плитки перегрелись выше равновесия: %.0f К против %.0f К",
			long.Windward.Temperature, steady)
	}
	if long.Windward.Temperature < short.Windward.Temperature {
		t.Error("температура пошла вниз при неизменном потоке")
	}

	t.Logf("равновесие %.0f К, через 1200 с %.0f К", steady, long.Windward.Temperature)
}

// Ориентация, назначенная оператором, отменяет команду наведения.
func TestManualAttitudeOverridesGuidance(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)

	// Доводим до второй ступени: там есть и плавники, и теплозащита.
	for i := 0; i < 3000 && sim.stage < 2; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.mu.Unlock()
	}
	if sim.stage < 2 {
		t.Fatal("вторая ступень не включилась")
	}

	sim.mu.RLock()
	shield := sim.heatShield != nil
	sim.mu.RUnlock()

	sim.mu.Lock()
	sim.telemetry = sim.buildTelemetryLocked()
	sim.mu.Unlock()
	start := sim.Snapshot().Pitch

	if !shield {
		t.Error("у корабля нет теплозащиты после разделения")
	}

	// Просим угол, заметно отличный от текущего.
	want := start - 25
	sim.SetManualAttitude(want, sim.Snapshot().Yaw, 0)

	for i := 0; i < 900; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.mu.Unlock()
	}

	sim.mu.Lock()
	sim.telemetry = sim.buildTelemetryLocked()
	sim.mu.Unlock()

	tel := sim.Snapshot()
	if tel.HeatShield == nil {
		t.Fatal("состояние теплозащиты не попало в телеметрию")
	}
	if !tel.HeatShield.ManualAttitude {
		t.Error("телеметрия не сообщает о ручной ориентации")
	}

	moved := start - tel.Pitch
	if moved < 10 {
		t.Errorf("корпус не пошёл на ручную команду: было %.1f°, стало %.1f°, "+
			"просили %.1f°", start, tel.Pitch, want)
	}

	sim.ReleaseManualAttitude()
	sim.mu.Lock()
	sim.telemetry = sim.buildTelemetryLocked()
	sim.mu.Unlock()

	if sim.Snapshot().HeatShield.ManualAttitude {
		t.Error("ориентация не вернулась автомату")
	}

	t.Logf("тангаж %.1f° → %.1f° при команде %.1f°", start, tel.Pitch, want)
}
