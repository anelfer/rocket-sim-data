package simulator

import (
	"testing"

	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// MECO по остатку топлива — на показании датчика.
//
// Настоящего прямого измерения массы в баке не существует физически
// (см. propulsion.DefaultPropellantSensor): решение о выключении двигателей
// по резерву топлива обязано опираться на показание датчика, а не на
// истинную массу, которую в реальности борт не знает. Задержка датчика
// может сдвинуть момент срабатывания, но она обязана оставаться в разумных,
// явно заданных пределах, а не быть непредсказуемо большой.
// -----------------------------------------------------------------------------

func TestMECOFiresWithinSensorTolerance(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)

	// Порог резерва искусственно поднят почти к полной заправке: так порог
	// по остатку заведомо срабатывает первым (раньше цели по скорости и
	// высоте), и тест не зависит от того, какой профиль полёта выбран
	// по умолчанию и на каком запасе топлива тот в норме завершается.
	sim.mu.Lock()
	sim.Config.FirstStage.FuelReserve = sim.state.FuelMass * 0.98
	reserve := sim.Config.FirstStage.FuelReserve
	step := sim.Time.Step
	sim.mu.Unlock()

	trueCrossingTime, sensedCrossingTime := -1.0, -1.0

	for i := 0; i < 20000; i++ {
		sim.mu.Lock()
		sim.step(step)
		trueFuel := sim.state.FuelMass
		sensedFuel := sim.sensedFuelMass
		elapsed := sim.elapsed
		sim.mu.Unlock()

		if trueCrossingTime < 0 && trueFuel <= reserve {
			trueCrossingTime = elapsed
		}
		if sensedCrossingTime < 0 && sensedFuel <= reserve {
			sensedCrossingTime = elapsed
		}
		if trueCrossingTime >= 0 && sensedCrossingTime >= 0 {
			break
		}
	}

	if trueCrossingTime < 0 || sensedCrossingTime < 0 {
		t.Fatal("порог резерва топлива не был достигнут за время теста")
	}

	// Допуск ориентирован на постоянную времени датчика (1.5 с в
	// DefaultPropellantSensor) — задержка порядка нескольких постоянных
	// времени ожидаема и не является отказом, а вот расхождение на порядок
	// больше сигнализировало бы о выборе неверной сенсорной модели.
	const tolerance = 4.0
	diff := sensedCrossingTime - trueCrossingTime
	if diff < -tolerance || diff > tolerance {
		t.Errorf("порог резерва по датчику пройден на T+%.2f с, по истинному "+
			"остатку — на T+%.2f с; расхождение %.2f с вне допуска ±%.1f с",
			sensedCrossingTime, trueCrossingTime, diff, tolerance)
	}
	t.Logf("порог резерва: истинный T+%.2f с, по датчику T+%.2f с (расхождение %.2f с)",
		trueCrossingTime, sensedCrossingTime, diff)
}
