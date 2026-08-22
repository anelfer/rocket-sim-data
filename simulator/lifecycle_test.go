package simulator

import (
	"testing"
	"time"

	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Жизненный цикл прогона.
//
// Симуляция должна останавливаться по команде и сообщать о завершении.
// Без этого внешний цикл не может понять, когда безопасно начинать новый
// прогон, и команда оператора теряется.
// -----------------------------------------------------------------------------

// fastSimulation возвращает симуляцию с коротким тактом, чтобы тест
// не ждал реального времени.
func fastSimulation() *Simulation {
	tc := DefaultTimeConfig()
	tc.TickInterval = 20 * time.Millisecond
	tc.Scale = 5
	return NewSimulationWithSeed(vehicle.Falcon9Config(), tc, 20260729)
}

// Stop должен завершать цикл, а Wait — дожидаться выхода из него.
func TestStopEndsRunLoop(t *testing.T) {
	s := fastSimulation()

	go s.Run()

	// Дожидаемся, пока цикл действительно начнёт работать.
	deadline := time.Now().Add(2 * time.Second)
	for s.RunStatus() != RunRunning && time.Now().Before(deadline) {
		time.Sleep(5 * time.Millisecond)
	}
	if s.RunStatus() != RunRunning {
		t.Fatal("цикл симуляции не запустился")
	}
	if s.Finished() {
		t.Fatal("работающая симуляция считается завершённой")
	}

	s.Stop()
	if !s.Wait(2 * time.Second) {
		t.Fatal("цикл не завершился после Stop")
	}
	if !s.Finished() {
		t.Error("завершённая симуляция не сообщает об этом")
	}
	if s.RunStatus() != RunStopped {
		t.Errorf("состояние после остановки %q, ожидалось stopped", s.RunStatus())
	}
}

// Остановка на паузе тоже должна срабатывать: иначе приостановленный прогон
// невозможно было бы завершить, и новый не начался бы никогда.
func TestStopWorksWhilePaused(t *testing.T) {
	s := fastSimulation()
	go s.Run()

	deadline := time.Now().Add(2 * time.Second)
	for s.RunStatus() != RunRunning && time.Now().Before(deadline) {
		time.Sleep(5 * time.Millisecond)
	}

	s.Pause()
	time.Sleep(60 * time.Millisecond)
	if s.RunStatus() != RunPaused {
		t.Fatalf("симуляция не встала на паузу: %s", s.RunStatus())
	}

	s.Stop()
	if !s.Wait(2 * time.Second) {
		t.Fatal("приостановленный прогон не удалось остановить")
	}
}

// Повторный Stop не должен паниковать: обработчик может вызвать его дважды.
func TestStopIsIdempotent(t *testing.T) {
	s := fastSimulation()
	go s.Run()
	time.Sleep(60 * time.Millisecond)

	s.Stop()
	s.Stop()
	if !s.Wait(2 * time.Second) {
		t.Fatal("цикл не завершился")
	}
	s.Stop()
}

// Незапущенная симуляция не считается работающей: ждать её завершения
// бессмысленно, и внешний цикл не должен на ней блокироваться.
func TestFreshSimulationIsNotStarted(t *testing.T) {
	s := fastSimulation()

	if s.Started() {
		t.Error("свежая симуляция считается запущенной")
	}
	if s.Finished() {
		t.Error("свежая симуляция считается завершённой")
	}
	if s.RunStatus() != RunIdle {
		t.Errorf("состояние %q, ожидалось idle", s.RunStatus())
	}
}

// Падение носителя завершает цикл: после этого прогон должен считаться
// оконченным, а не работающим.
func TestCrashEndsRun(t *testing.T) {
	s := fastSimulation()

	// Двигатели выключаются после отрыва, и носитель падает обратно.
	//
	// Дожидаться именно отрыва, а не просто выдержки времени, обязательно:
	// носитель, потерявший тягу на столе, никуда не улетит и не упадёт —
	// он останется стоять, и цикл не завершится никогда.
	go func() {
		deadline := time.Now().Add(10 * time.Second)
		for time.Now().Before(deadline) {
			if s.Snapshot().Altitude > 2000 {
				break
			}
			time.Sleep(10 * time.Millisecond)
		}
		s.EmergencyShutdown()
	}()
	go s.Run()

	if !s.Wait(20 * time.Second) {
		t.Fatal("цикл не завершился после падения носителя")
	}
	if state := s.RunStatus(); state != RunEnded {
		t.Errorf("состояние после падения %q, ожидалось ended", state)
	}
	t.Logf("Падение на T+%.1f с", s.ModelTime())
}
