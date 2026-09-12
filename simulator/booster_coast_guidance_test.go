package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// Нос отклоняется В СТОРОНУ промаха, и возникающая поперечная сила идёт
// ПРОТИВ него. Это единственное место, где знак бокового наведения задан, и
// ошибка в нём не проявилась бы иначе как медленным ростом промаха.
func TestCoastLeanOpposesMiss(t *testing.T) {
	b, nav := boosterInCoast(t, 1)
	retro := nav.AirRelativeVelocity.Unit().Scale(-1)

	for _, c := range []struct {
		name       string
		east, nrth float64
	}{
		{"восток", 800, 0},
		{"север", 0, 800},
		{"юго-запад", -500, -500},
	} {
		b.plan.Valid = true
		b.plan.MissEast, b.plan.MissNorth = c.east, c.nrth
		b.plan.PredictedMiss = math.Hypot(c.east, c.nrth)

		lean := b.coastLeanDir(nav, retro)
		if lean == (physics.Vec3{}) {
			t.Fatalf("промах на %s не дал направления отклонения", c.name)
		}

		miss := nav.Frame.East.Scale(c.east).Add(nav.Frame.North.Scale(c.nrth))
		if lean.Dot(miss) <= 0 {
			t.Errorf("промах на %s: нос отклонён не в сторону промаха (%.3f)",
				c.name, lean.Dot(miss.Unit()))
		}
		if d := math.Abs(lean.Dot(retro)); d > 1e-9 {
			t.Errorf("промах на %s: отклонение не перпендикулярно потоку (%.9f)", c.name, d)
		}

		// Сила по той же модели, что летает корпус. Сравнивается ПРИРОСТ
		// от отклонения, а не полная сила: полная — это прежде всего
		// осевое торможение вдоль потока, и его проекция на горизонталь
		// определяется наклоном траектории, а не работой наведения.
		radius := b.Config.Diameter / 2
		force := func(axis physics.Vec3) physics.Vec3 {
			return physics.BodyAeroForce(nav.DynamicPressure, 2.5,
				math.Pi*radius*radius, b.Config.FirstStageLength*b.Config.Diameter,
				nav.AirRelativeVelocity.Unit(), axis)
		}
		leaned, _ := b.coastLeanedDir(nav, retro, 10)
		gain := force(leaned).Sub(force(retro))
		if gain.Dot(miss) >= 0 {
			t.Errorf("промах на %s: отклонение добавляет силу ПО промаху, а не против (%.1f Н)",
				c.name, gain.Dot(miss.Unit()))
		}
	}
}

// Отклонять корпус на шум прогноза нельзя: у самого минимума промах гуляет
// на метры, а отклонение стоит устойчивости и ресурса рулей.
func TestCoastLeanHasDeadband(t *testing.T) {
	b, nav := boosterInCoast(t, 1)
	retro := nav.AirRelativeVelocity.Unit().Scale(-1)

	b.plan.Valid = true
	b.plan.MissEast, b.plan.MissNorth = 0.2, 0.2
	if lean := b.coastLeanDir(nav, retro); lean != (physics.Vec3{}) {
		t.Errorf("промах в 0.3 м уже двигает корпус: %v", lean)
	}

	b.plan.MissEast, b.plan.MissNorth = math.NaN(), math.NaN()
	if lean := b.coastLeanDir(nav, retro); lean != (physics.Vec3{}) {
		t.Errorf("неопределённый промах двигает корпус: %v", lean)
	}
}

// Разомкнутость: прогнозируемый промах не должен зависеть от того, какое
// отклонение управление командует прямо сейчас. Иначе регулятор двигает
// собственную цель — довернул корпус, и прогноз показывает, что править
// больше нечего.
func TestPredictedMissIsOpenLoop(t *testing.T) {
	b, nav := boosterInCoast(t, 1)

	e1, n1, m1, t1 := b.planMissVector(nav)

	// Команда отклонения меняется на весь режимный диапазон...
	b.plan.Valid = true
	b.plan.MissEast, b.plan.MissNorth = 5000, 5000
	b.plan.PredictedMiss = math.Hypot(5000, 5000)

	e2, n2, m2, t2 := b.planMissVector(nav)
	if e1 != e2 || n1 != n2 || m1 != m2 || t1 != t2 {
		t.Errorf("прогноз промаха поехал за командой: %.3f/%.3f/%.3f/%.3f против %.3f/%.3f/%.3f/%.3f",
			e2, n2, m2, t2, e1, n1, m1, t1)
	}
}
