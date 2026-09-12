package simulator

import (
	"math/rand"
	"testing"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Навигация первой ступени по показаниям приборов.
//
// Проверяется не «работают ли датчики» (это уже проверено для корабля,
// sensed_nav_test.go), а именно то, что бустер ими ПОЛЬЗУЕТСЯ: до этой
// работы его наведение читало истинный вектор состояния напрямую, и любой
// из тестов ниже проходил бы тождественно.
// -----------------------------------------------------------------------------

// TestBoosterSensedNavDivergesFromTruth — показания обязаны отличаться от
// истины. Совпадение означало бы, что датчик подключён формально, а
// наведение по-прежнему видит истинное состояние.
func TestBoosterSensedNavDivergesFromTruth(t *testing.T) {
	b := newBoosterForSensorTest(t)
	for i := 0; i < 200; i++ {
		b.updateSensors(0.05)
	}
	if b.sensedPosition == b.state.Position {
		t.Fatal("показание канала положения тождественно истине — датчик не работает")
	}
	if b.sensedVelocity == b.state.Velocity {
		t.Fatal("показание канала скорости тождественно истине — датчик не работает")
	}
	if b.sensedOrientation == b.attitude.Orientation {
		t.Fatal("показание канала ориентации тождественно истине — датчик не работает")
	}
}

// TestBoosterSensedNavStateUsesFilteredSolution — наведение обязано видеть
// НАВИГАЦИОННОЕ РЕШЕНИЕ (сглаженное счислением), а не сырое показание.
func TestBoosterSensedNavStateUsesFilteredSolution(t *testing.T) {
	b := newBoosterForSensorTest(t)
	b.navPosition = b.state.Position.Add(physics.Vec3{X: 1234})
	b.navVelocity = b.state.Velocity.Add(physics.Vec3{Y: 56})
	b.sensedPosition = b.state.Position
	b.sensedVelocity = b.state.Velocity

	nav := b.sensedNavState(b.trueNavForTest(), nil)
	if nav.Position != b.navPosition || nav.Velocity != b.navVelocity {
		t.Fatalf("наведение читает не навигационное решение: pos=%v vel=%v", nav.Position, nav.Velocity)
	}
}

// TestBoosterNavFilterRejectsNoise — навигационное решение обязано быть
// ближе к истине, чем сырое показание.
//
// Сценарий — свободное падение: истинное состояние честно интегрируется
// под тяготением, кажущееся ускорение нулевое (акселерометр в свободном
// падении показывает ноль — см. updateNavigation). Счисление в этих
// условиях воспроизводит истину точно, и всё расхождение, которое видит
// фильтр, — собственный шум приборов. Именно его он и обязан подавить.
func TestBoosterNavFilterRejectsNoise(t *testing.T) {
	b := newBoosterForSensorTest(t)

	const dt = 0.05
	rawWorst, filteredWorst := 0.0, 0.0
	for i := 0; i < 600; i++ {
		// Истина: свободное падение, тот же закон, что и у счисления.
		acc := physics.GravityAcceleration(b.state.Position)
		b.state.Position = b.state.Position.
			Add(b.state.Velocity.Scale(dt)).
			Add(acc.Scale(dt * dt / 2))
		b.state.Velocity = b.state.Velocity.Add(acc.Scale(dt))

		b.updateSensors(dt)
		b.updateNavigation(dt, physics.Vec3{})

		// Первые такты — переходный процесс самих каналов, он не про шум.
		if i < 200 {
			continue
		}
		if d := b.sensedVelocity.Sub(b.state.Velocity).Norm(); d > rawWorst {
			rawWorst = d
		}
		if d := b.navVelocity.Sub(b.state.Velocity).Norm(); d > filteredWorst {
			filteredWorst = d
		}
	}
	if rawWorst <= 0 {
		t.Fatal("канал скорости не шумит — проверять нечего")
	}
	if filteredWorst >= rawWorst {
		t.Fatalf("фильтр не сглаживает: худшее отклонение решения %.3f м/с, сырого показания %.3f м/с",
			filteredWorst, rawWorst)
	}
}

// TestBoosterSensorDropoutHoldsLastValue — на пропуске связи показание
// обязано ДЕРЖАТЬСЯ. Ни возврат к истине (это сделало бы датчик фикцией),
// ни обнуление (наведение отработало бы выдуманный скачок) недопустимы.
func TestBoosterSensorDropoutHoldsLastValue(t *testing.T) {
	b := newBoosterForSensorTest(t)
	for i := 0; i < 50; i++ {
		b.updateSensors(0.05)
	}
	before := b.sensedPosition

	// Принудительное отсутствие данных на канале положения (control.Dead —
	// тот же механизм, которым пульт имитирует отказ прибора).
	dead := control.SensorOverrides{Dead: true}
	b.state.Position = b.state.Position.Add(physics.Vec3{X: 5000})
	for i := 0; i < 20; i++ {
		if m := b.positionSensor.UpdateWith(b.state.Position, 0.05, b.sensorRng, dead); m.Valid {
			t.Fatal("канал считает показание достоверным при отсутствии данных")
		}
	}
	if b.sensedPosition != before {
		t.Fatalf("показание изменилось при отсутствии данных: было %v, стало %v", before, b.sensedPosition)
	}
	if b.sensedPosition == b.state.Position {
		t.Fatal("показание совпало с новой истиной — датчик подглядывает вместо удержания")
	}
}

// TestBoosterAutopilotClosesOnSensedAttitude — автопилот обязан считать
// ошибку по ПОКАЗАНИЮ гироскопа. До этой работы SensedOrientation у бустера
// не заполнялся вовсе, и attitudeError молча уходил в ветку «датчика нет».
func TestBoosterAutopilotClosesOnSensedAttitude(t *testing.T) {
	m, _ := vehicle.MissionByID("transatmospheric")
	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithSeed(m.Apply(p.Config()), DefaultTimeConfig(), 3)
	for i := 0; i < 40000; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		b := sim.booster
		ready := b != nil && b.Alive() && b.elapsed > 0 && b.attitude.SensedOrientation.Norm() > 1e-9
		dead := b != nil && !b.Alive()
		sim.mu.Unlock()
		if ready {
			return
		}
		if dead {
			t.Fatal("бустер разрушился раньше, чем удалось проверить датчик ориентации")
		}
	}
	t.Fatal("SensedOrientation бустера так и не заполнен — автопилот closes на истине")
}

// newBoosterForSensorTest создаёт бустер в представительном состоянии
// отделения. Прогонять для этого целую миссию не нужно: проверяется сама
// измерительная цепочка, а не траектория.
func newBoosterForSensorTest(t *testing.T) *Booster {
	t.Helper()
	cfg := vehicle.StarshipConfig()
	position := physics.GeodeticToECEF(physics.Geodetic{
		Latitude: cfg.LaunchLatitude, Longitude: cfg.LaunchLongitude, Altitude: 60000,
	})
	local := physics.NewLocalFrame(position)
	velocity := local.East.Scale(1200).Add(local.Up.Scale(900))
	rng := rand.New(rand.NewSource(7))
	sensorRng := rand.New(rand.NewSource(11))
	b := NewBooster(cfg, position, velocity, 0, rng, sensorRng, 11, 250, physics.Quaternion{W: 1})
	if b == nil {
		t.Fatal("не удалось создать бустер")
	}
	return b
}

// trueNavForTest — истинное состояние навигации бустера, как его строит Step.
func (b *Booster) trueNavForTest() orbit.NavState {
	return orbit.NavState{Mass: b.dryMass() + b.state.FuelMass}
}
