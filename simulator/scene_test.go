package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Оси корпуса для сцены.
//
// Картинка обязана показывать ту же ориентацию, по которой модель считает
// силы. Проверяется это не «похоже — не похоже», а тем, что тройка осей
// остаётся ортонормированной, что на столе продольная ось смотрит вверх,
// и что угол между продольной осью и потоком совпадает с углом атаки,
// который модель уже посчитала своим способом.
// -----------------------------------------------------------------------------

func norm(v SceneVector) float64 {
	return math.Sqrt(v.East*v.East + v.North*v.North + v.Up*v.Up)
}

func dot(a, b SceneVector) float64 {
	return a.East*b.East + a.North*b.North + a.Up*b.Up
}

func TestSceneAxesAreOrthonormal(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)

	check := func(when string, sc SceneTelemetry) {
		t.Helper()
		for name, v := range map[string]SceneVector{
			"вперёд": sc.Forward, "вправо": sc.Right, "вниз": sc.Down,
		} {
			if math.Abs(norm(v)-1) > 1e-6 {
				t.Errorf("%s: ось «%s» длиной %.6f, а должна быть единичной",
					when, name, norm(v))
			}
		}
		if math.Abs(dot(sc.Forward, sc.Right)) > 1e-6 ||
			math.Abs(dot(sc.Forward, sc.Down)) > 1e-6 ||
			math.Abs(dot(sc.Right, sc.Down)) > 1e-6 {
			t.Errorf("%s: оси корпуса не перпендикулярны", when)
		}
	}

	sim.mu.Lock()
	sim.telemetry = sim.buildTelemetryLocked()
	sim.mu.Unlock()

	pad := sim.Snapshot()
	check("на столе", pad.Scene)

	// На стартовом столе продольная ось смотрит в зенит.
	if pad.Scene.Forward.Up < 0.99 {
		t.Errorf("на столе продольная ось отклонена от вертикали: %+v",
			pad.Scene.Forward)
	}
	if pad.Scene.Downrange > 1 {
		t.Errorf("на столе удаление от площадки %.1f м", pad.Scene.Downrange)
	}

	// Через полторы минуты полёта: ракета уже развёрнута и идёт в атмосфере.
	for i := 0; i < 900; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.mu.Unlock()
	}
	sim.mu.Lock()
	sim.telemetry = sim.buildTelemetryLocked()
	sim.mu.Unlock()

	tel := sim.Snapshot()
	check("на выведении", tel.Scene)

	if tel.Scene.Downrange < 1000 {
		t.Errorf("удаление от площадки %.0f м — ракета никуда не улетела",
			tel.Scene.Downrange)
	}

	// Угол между продольной осью и потоком обязан совпасть с углом атаки,
	// посчитанным моделью независимо.
	if norm(tel.Scene.Airflow) > 0.5 {
		// Поток направлен навстречу корпусу, поэтому угол атаки — это угол
		// между продольной осью и обратным потоком.
		cos := -dot(tel.Scene.Forward, tel.Scene.Airflow)
		aoa := math.Acos(math.Max(-1, math.Min(1, cos))) * 180 / math.Pi

		if math.Abs(aoa-tel.TotalAoA) > 0.5 {
			t.Errorf("угол атаки по осям сцены %.2f°, по модели %.2f°",
				aoa, tel.TotalAoA)
		}
		t.Logf("угол атаки: сцена %.2f°, модель %.2f°", aoa, tel.TotalAoA)
	}
}
