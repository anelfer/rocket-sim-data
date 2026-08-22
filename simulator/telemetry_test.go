package simulator

import (
	"encoding/json"
	"math"
	"testing"
	"time"

	"rocketTelemetrySim/simulator/env"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Телеметрия: сериализация на каждой фазе и доступ из нескольких горутин.
// -----------------------------------------------------------------------------

func TestFirstStageTelemetryExistsFromLaunch(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), testSeed)

	sim.mu.Lock()
	for i := 0; i < 300; i++ {
		sim.step(0.1)
	}
	sim.telemetry = sim.buildTelemetryLocked()
	sim.mu.Unlock()

	tel := sim.Snapshot()
	st := tel.SpentStage
	if st == nil {
		t.Fatal("телеметрия первой ступени отсутствует до разделения")
	}
	if st.Outcome != env.StageAttached.String() {
		t.Errorf("состояние ступени = %q, ожидалось %q", st.Outcome, env.StageAttached.String())
	}

	// Координаты обязаны совпадать с координатами носителя, а не быть нулями.
	if st.Latitude != tel.Latitude || st.Longitude != tel.Longitude {
		t.Errorf("координаты ступени (%.4f, %.4f) не совпадают с координатами ракеты (%.4f, %.4f)",
			st.Latitude, st.Longitude, tel.Latitude, tel.Longitude)
	}
	if math.Abs(st.Latitude) < 1 && math.Abs(st.Longitude) < 1 {
		t.Error("координаты ступени равны нулю — это точка в Гвинейском заливе")
	}
	if st.Altitude != tel.Altitude {
		t.Errorf("высота ступени %.1f не совпадает с высотой ракеты %.1f",
			st.Altitude, tel.Altitude)
	}
	if st.SpinRate != 0 {
		t.Errorf("до разделения ступень не может вращаться: %.3f °/с", st.SpinRate)
	}
	if len(st.Nodes) == 0 {
		t.Error("тепловые узлы ступени не заполнены")
	}
}

// После выхода на орбиту значения становятся конечными.

func TestTelemetryMarshalsAtEveryPhase(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), testSeed)

	seenSuborbital := false
	seenOrbital := false

	sim.mu.Lock()
	defer sim.mu.Unlock()

	for i := 0; i < 20000; i++ {
		sim.step(0.1)
		if i%500 != 0 {
			continue
		}
		tel := sim.buildTelemetryLocked()

		data, err := json.Marshal(tel)
		if err != nil {
			t.Fatalf("телеметрия не сериализуется на T+%.1f с: %v", tel.Time, err)
		}

		var back map[string]any
		if err := json.Unmarshal(data, &back); err != nil {
			t.Fatalf("результат не читается обратно: %v", err)
		}

		if tel.OrbitClosed {
			seenOrbital = true
			if back["periapsis"] == nil {
				t.Error("на замкнутой орбите перицентр не может быть null")
			}
		} else {
			seenSuborbital = true
			if back["periapsis"] != nil {
				t.Errorf("на суборбитальном участке ожидался null, получено %v",
					back["periapsis"])
			}
		}

		if sim.phase == orbit.PhaseOrbital {
			break
		}
	}

	if !seenSuborbital {
		t.Error("суборбитальный участок не проверен")
	}
	if !seenOrbital {
		t.Error("орбитальный участок не проверен")
	}
}

func TestFloatMarshalling(t *testing.T) {
	cases := []struct {
		value Float
		want  string
	}{
		{Float(math.NaN()), "null"},
		{Float(math.Inf(1)), "null"},
		{Float(math.Inf(-1)), "null"},
		{Float(400000), "400000"},
		{Float(-1.5), "-1.5"},
	}
	for _, c := range cases {
		got, err := json.Marshal(c.value)
		if err != nil {
			t.Fatalf("сериализация %v вернула ошибку: %v", float64(c.value), err)
		}
		if string(got) != c.want {
			t.Errorf("получено %s, ожидалось %s", got, c.want)
		}
	}

	var f Float
	if err := json.Unmarshal([]byte("null"), &f); err != nil {
		t.Fatalf("null не читается: %v", err)
	}
	if f.Defined() {
		t.Error("null должен читаться как неопределённое значение")
	}
}

// -----------------------------------------------------------------------------
// D5: конкурентный доступ
// -----------------------------------------------------------------------------

// Обработчики REST читают состояние, пока цикл симуляции его изменяет.
// Тест запускается с -race и должен проходить без предупреждений.

func TestConcurrentAccessIsRaceFree(t *testing.T) {
	sim := NewSimulationWithConfig(vehicle.Falcon9Config(), TimeConfig{
		Step:         0.05,
		TickInterval: time.Millisecond,
		Scale:        20,
	})
	SetCurrentSimulation(sim)

	done := make(chan struct{})
	go func() {
		sim.Run()
		close(done)
	}()

	deadline := time.After(300 * time.Millisecond)
	readers := 8
	stopReaders := make(chan struct{})
	var finished = make(chan struct{}, readers)

	for i := 0; i < readers; i++ {
		go func(n int) {
			defer func() { finished <- struct{}{} }()
			for {
				select {
				case <-stopReaders:
					return
				default:
				}
				_ = sim.Snapshot()
				_ = sim.EngineList()
				_ = sim.Phase()
				if n == 0 {
					_ = sim.SetEngineState("S1-1", 500000, true)
				}
				if n == 1 {
					sim.SetAllEnginesRunning(true)
				}
			}
		}(i)
	}

	<-deadline
	close(stopReaders)
	for i := 0; i < readers; i++ {
		<-finished
	}

	sim.Stop()
	select {
	case <-done:
	case <-time.After(2 * time.Second):
		t.Fatal("цикл симуляции не остановился по команде Stop")
	}

	SetCurrentSimulation(nil)
}

// -----------------------------------------------------------------------------
// Полный прогон выведения
// -----------------------------------------------------------------------------

// runAscent прогоняет симуляцию до достижения орбиты или до тайм-аута.
