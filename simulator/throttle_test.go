package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Нижний предел дросселирования.
//
// У камеры есть аттестованный диапазон: у Raptor — от сорока процентов до
// номинала. Наведение само туда никогда не просится, и если его команда
// всё же оказалась ниже предела (округление, переходный процесс), это не
// значит «погасить» — контур обязан удержать предел на автоматике.
//
// Прямая команда оператора — другое дело: он явно просит именно это
// значение, в том числе чтобы проверить, удержится ли камера ниже предела,
// или сорвётся в погасание. Держать её на полу означало бы врать, что
// камера справилась, когда ей на самом деле не дали попробовать.
// -----------------------------------------------------------------------------

// steadyStarship доводит носитель до установившейся работы первой ступени.
func steadyStarship(t *testing.T) *Simulation {
	t.Helper()

	sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
	for i := 0; i < 300; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.mu.Unlock()
	}
	return sim
}

// Прямая команда оператора ниже предела проходит как есть — модель честно
// показывает, что реально подаётся на камеры, а не подменяет её удержанием
// на пределе.
func TestDeepThrottleBelowFloorIsHonored(t *testing.T) {
	sim := steadyStarship(t)

	sim.mu.RLock()
	running := vehicle.RunningEngines(sim.engines)
	sim.mu.RUnlock()

	if running == 0 {
		t.Fatal("двигатели не работают, дросселировать нечего")
	}

	// Просим заведомо ниже предела — так же, как это делает оператор
	// ползунком на пульте, пробуя удержать камеру на экстремально низком газе.
	mustApply(t, sim, control.Command{
		ID: "deep-throttle", Parameter: "engine.throttle",
		Mode: control.ModeSet, Basis: control.BasisAbsolute, Value: 0.15,
	})

	tel := runFor(sim, 60)

	if math.Abs(tel.Throttle-0.15) > 0.02 {
		t.Errorf("уставка %.2f вместо честно поданной команды 0.15", tel.Throttle)
	}
	if tel.ThrottleLimited {
		t.Error("телеметрия сообщает об удержании на пределе — это не наведение, " +
			"а прямая команда оператора, её подменять не должны")
	}
	if tel.ThrottleFloor <= 0 {
		t.Error("предел дросселирования не отдан в телеметрию")
	}

	t.Logf("команда 0.15 → уставка %.2f, работают %d двигателей, тяга %.1f МН",
		tel.Throttle, tel.EnginesRunning, tel.TotalThrust/1e6)
}

// Удержание предела на автоматическом пути (без прямой команды оператора)
// проверяется на уровне двигателя в simulator/propulsion
// (TestAutoThrottleHoldsFloorInsteadOfDying) — там это делается прямой
// подачей EngineInput в обход GNC, а не через sim.throttleCmd, который
// наведение перезаписывает на каждом шаге заново.

// Глубокое дросселирование в пределах диапазона — режим, а не неисправность.
func TestDeepThrottleIsNotAFailure(t *testing.T) {
	sim := steadyStarship(t)

	mustApply(t, sim, control.Command{
		ID: "partial-throttle", Parameter: "engine.throttle",
		Mode: control.ModeSet, Basis: control.BasisAbsolute, Value: 0.45,
	})
	runFor(sim, 60)

	snap := sim.ControlSnapshot()

	var throttled, degraded, other int
	for _, p := range snap.Pumps {
		switch p.State {
		case "Throttled":
			throttled++
		case "Degraded":
			degraded++
		case "Normal":
			other++
		}
	}

	if degraded > 0 {
		t.Errorf("%d агрегатов помечены как неисправные при штатном "+
			"дросселировании", degraded)
	}
	if throttled+other == 0 {
		t.Error("состояние агрегатов не определено")
	}

	t.Logf("на 45 %%: дросселированных %d, номинальных %d, неисправных %d",
		throttled, other, degraded)
}
