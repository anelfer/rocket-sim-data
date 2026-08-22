package simulator

import (
	"encoding/json"
	"math"
	"testing"
	"time"

	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Темп публикации телеметрии.
//
// Модель считает с шагом в десятую долю секунды. Отдавать наружу одно состояние
// в секунду означает выбрасывать девять шагов из десяти: на пульте это рывки,
// а быстрые события — заброс давления при запуске, перекладка плавников,
// разворот перед посадкой — проскакивают между кадрами целиком.
// -----------------------------------------------------------------------------

func TestTelemetryRateMatchesModelStep(t *testing.T) {
	cfg := DefaultTimeConfig()

	if cfg.TickInterval > 50*time.Millisecond {
		t.Errorf("телеметрия публикуется раз в %v — реже двадцати герц",
			cfg.TickInterval)
	}

	// Публиковать чаще, чем считает модель, бессмысленно: между тактами
	// состояние не меняется.
	if cfg.TickInterval.Seconds() < cfg.Step {
		t.Errorf("такт публикации %v короче шага модели %.2f с",
			cfg.TickInterval, cfg.Step)
	}
}

// Кадр сцены обязан быть дешёвым.
//
// Смысл отдельного сообщения ровно в этом: полный снимок весит под сотню
// килобайт, и слать его двадцать раз в секунду — полтора мегабайта на каждого
// зрителя. Если кадр сцены разрастётся до сравнимого размера, разделение
// потоков перестанет что-либо давать, а заметить это можно только здесь.
func TestSceneFrameStaysSmall(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
	for i := 0; i < 1500; i++ {
		sim.mu.Lock()
		sim.step(sim.Time.Step)
		sim.telemetry = sim.buildTelemetryLocked()
		sim.mu.Unlock()
	}

	frame, err := json.Marshal(sim.SceneFrame())
	if err != nil {
		t.Fatalf("кадр сцены не сериализуется: %v", err)
	}
	full, err := json.Marshal(sim.ControlSnapshot())
	if err != nil {
		t.Fatalf("снимок не сериализуется: %v", err)
	}

	if len(frame) > 4096 {
		t.Errorf("кадр сцены разросся до %d байт", len(frame))
	}
	if len(frame)*8 > len(full) {
		t.Errorf("кадр сцены %d байт против снимка %d — разделение потоков "+
			"перестало окупаться", len(frame), len(full))
	}

	t.Logf("кадр сцены %d байт, полный снимок %d байт", len(frame), len(full))
}

// Кадр сцены обязан нести всё, что сцена читает, и под теми же именами.
//
// Имена здесь — не украшение: сцена не различает, пришло состояние в полном
// снимке или в лёгком кадре, и читает одни и те же ключи. Разойдись имя хоть
// в одном поле — картинка не сломается, а молча обеднеет: пропадут плавники
// или перестанет светиться теплозащита, и искать причину придётся глазами.
func TestSceneFrameCarriesEverythingSceneReads(t *testing.T) {
	// Поля с omitempty на активном участке пусты, поэтому кадр набирается
	// вручную: проверяются имена в разметке, а не состояние конкретного полёта.
	frame := SceneFrame{
		Flaps:      []FlapTelemetry{{Name: "fwd-left"}},
		HeatShield: &HeatShieldTelemetry{},
		Landing:    &LandingTelemetry{},
	}

	raw, err := json.Marshal(frame)
	if err != nil {
		t.Fatalf("кадр сцены не сериализуется: %v", err)
	}

	var got map[string]json.RawMessage
	if err := json.Unmarshal(raw, &got); err != nil {
		t.Fatalf("кадр сцены не разбирается: %v", err)
	}

	for _, key := range []string{
		"time", "phase", "stage", "altitude", "lat", "lon",
		"totalVelocity", "verticalVelocity", "mach", "totalAoA",
		"dynamicPressure", "heatFlux", "throttle", "enginesRunning",
		"scene", "flaps", "heatShield", "landing",
	} {
		if _, ok := got[key]; !ok {
			t.Errorf("в кадре сцены нет поля %q", key)
		}
	}
}

// Замедленный прогон обязан идти медленнее, а не с обычной скоростью.
//
// Число шагов за такт почти никогда не целое: при четверти скорости на такт
// приходится четверть шага. Округление такой доли давало ноль, а защита
// от нуля поднимала её до целого шага — и замедление не работало вовсе.
func TestSlowMotionAdvancesProportionally(t *testing.T) {
	for _, scale := range []float64{1, 0.5, 0.25, 0.1} {
		sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
		sim.Time.Scale = scale

		// Сто тактов таймера без обращения к настоящим часам.
		const ticks = 100
		for i := 0; i < ticks; i++ {
			sim.mu.Lock()
			sim.substepDebt += sim.Time.Scale * sim.Time.TickInterval.Seconds() / sim.Time.Step
			n := int(sim.substepDebt)
			sim.substepDebt -= float64(n)
			for j := 0; j < n; j++ {
				sim.step(sim.Time.Step)
			}
			sim.mu.Unlock()
		}

		want := scale * float64(ticks) * sim.Time.TickInterval.Seconds()
		got := sim.ModelTime()

		if math.Abs(got-want) > sim.Time.Step {
			t.Errorf("на скорости %g× за %d тактов прошло %.2f с модельного "+
				"времени вместо %.2f", scale, ticks, got, want)
		}
	}
}
