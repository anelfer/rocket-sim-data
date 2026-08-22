package simulator

import (
	"testing"

	"rocketTelemetrySim/simulator/orbit/sensing"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// sensedNavState — дешёвая проверка на отсутствие тождественной подмены.
//
// Наведение теперь замкнуто по показаниям навигационных датчиков, а не по
// истинному вектору состояния. Эта проверка не про конкретные допуски,
// а про то, что датчик вообще что-то делает: при штатном шуме показание
// обязано разойтись с истиной, а если шум обнулить — вернуться к точному
// совпадению. Если бы sensedNavState по ошибке просто копировал trueNav
// (тождественная подмена), эта проверка провалилась бы в первой части.
// -----------------------------------------------------------------------------

func TestSensedNavDivergesUnderNoiseAndConvergesWithoutIt(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)

	// Несколько тактов, а не один: период обновления датчика (UpdateInterval
	// в DefaultPositionSensor/DefaultVelocitySensor — 0.1 с) больше шага
	// интегрирования (0.05 с), и на первом такте показание ещё держит
	// затравочное значение, не занятое ни разу. Истинное положение/скорость
	// снимаются ДО последнего step(): именно это значение датчик видит на
	// этом такте (UpdateWith читает s.state.* в начале step(), до
	// интегрирования).
	sim.mu.Lock()
	for i := 0; i < 9; i++ {
		sim.step(sim.Time.Step)
	}
	truePos := sim.state.Position
	trueVel := sim.state.Velocity
	sim.step(sim.Time.Step)
	sensedPos := sim.lastValidSensedPosition
	sensedVel := sim.lastValidSensedVelocity
	sim.mu.Unlock()

	if truePos == sensedPos {
		t.Error("показание датчика положения совпало с истинным значением при штатном шуме — датчик не сработал")
	}
	if trueVel == sensedVel {
		t.Error("показание датчика скорости совпало с истинным значением при штатном шуме — датчик не сработал")
	}

	// Без шума, дрейфа, запаздывания и пропусков связи датчик — точный
	// повторитель истинного значения (см. propulsion.Sensor.UpdateWith:
	// TimeConstant=0 передаёт значение без сглаживания, нулевые
	// NoiseFraction/NoiseFloor/DriftRate/OutlierRate/DropoutRate не вносят
	// ничего лишнего).
	ideal := NewSimulationWithSeed(vehicle.StarshipConfig(), DefaultTimeConfig(), 1)
	ideal.mu.Lock()
	var zero sensing.ScalarSensorConfig
	ideal.positionSensor = sensing.NewVectorSensorAt(zero, ideal.state.Position)
	ideal.velocitySensor = sensing.NewVectorSensorAt(zero, ideal.state.Velocity)
	ideal.lastValidSensedPosition = ideal.state.Position
	ideal.lastValidSensedVelocity = ideal.state.Velocity
	idealTruePos := ideal.state.Position
	idealTrueVel := ideal.state.Velocity
	ideal.step(ideal.Time.Step)
	idealSensedPos := ideal.lastValidSensedPosition
	idealSensedVel := ideal.lastValidSensedVelocity
	ideal.mu.Unlock()

	if idealTruePos != idealSensedPos {
		t.Errorf("при нулевом шуме показание положения разошлось с истинным: %+v vs %+v",
			idealSensedPos, idealTruePos)
	}
	if idealTrueVel != idealSensedVel {
		t.Errorf("при нулевом шуме показание скорости разошлось с истинным: %+v vs %+v",
			idealSensedVel, idealTrueVel)
	}
}
