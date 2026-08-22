package propulsion

import (
	"math"
	"testing"

	ctrl "rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Нижний предел дросселирования: кто его обязан удерживать, а кто вправе
// его переступить.
//
// Наведение само никогда не просит газ ниже аттестованного предела — но
// если бы попросило (округление, переходный процесс), контур обязан
// удержать предел, а не молча погасить камеру. Прямая же команда
// оператора — это не наведение: он явно просит именно это значение,
// в том числе чтобы проверить, удержится ли камера ниже предела или
// сорвётся в погасание, и держать её на полу означало бы врать, что
// камера справилась, когда ей на самом деле не дали попробовать.
// -----------------------------------------------------------------------------

func TestAutoThrottleHoldsFloorInsteadOfDying(t *testing.T) {
	e := engineAt(RaptorConfig(), methaloxInput(), 6)

	in := methaloxInput()
	in.Throttle = 0.15
	runToSteadyState(e, in, 6)

	if !e.Running {
		t.Fatal("камера погасла при заниженной команде наведения")
	}
	if !e.ThrottleLimited {
		t.Error("предел не удержан: ThrottleLimited должен быть true")
	}
	if math.Abs(e.AppliedThrottle-e.Config.MinThrottle) > 0.02 {
		t.Errorf("уставка %.2f вместо удержанного предела %.2f",
			e.AppliedThrottle, e.Config.MinThrottle)
	}
}

func TestOperatorOverrideBelowFloorIsHonored(t *testing.T) {
	e := engineAt(RaptorConfig(), methaloxInput(), 6)

	in := methaloxInput()
	in.Throttle = 1
	in.Overrides.ThrottleOverride = ctrl.Set(0.15)
	runToSteadyState(e, in, 6)

	if e.ThrottleLimited {
		t.Error("уставка помечена удержанной на пределе — это прямая команда " +
			"оператора, её подменять не должны")
	}
	if math.Abs(e.AppliedThrottle-0.15) > 0.02 {
		t.Errorf("уставка %.2f вместо честно поданной команды 0.15", e.AppliedThrottle)
	}
}
