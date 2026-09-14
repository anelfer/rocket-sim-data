package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// Цель ориентации бустера задаётся НАПРАВЛЕНИЕМ, а не углами: иначе у
// вертикально стоящего корпуса опора крена переворачивается на границе
// азимута и автопилот получает команду на резкий крен на ровном месте.
//
// Проверка на самом автопилоте: две почти совпадающие цели по разные
// стороны от вертикали обязаны дать почти совпадающие ошибки ориентации.
func TestBoosterTargetDoesNotJumpAcrossVertical(t *testing.T) {
	frame := physics.NewLocalFrame(physics.Vec3{X: physics.EarthRadius})

	var a VehicleAttitude
	a.Orientation = physics.AimQuaternion(physics.Quaternion{W: 1},
		frame.Direction(89.99, 0))

	near := frame.Direction(89.99, 0)
	far := frame.Direction(89.99, 180)
	if off := physics.AngleOfAttack(near, far); off > 0.05 {
		t.Fatalf("направления разошлись на %.4f° — проверка не о том", off)
	}

	right := a.Orientation.Rotate(physics.Vec3{Y: 1})
	errOf := func(dir physics.Vec3, useQuaternion bool) physics.Vec3 {
		var q physics.Quaternion
		att := physics.Attitude{}
		if useQuaternion {
			q, _ = physics.AimBasis(dir, right)
		} else {
			pitch, yaw := frame.PitchAzimuth(dir)
			att = physics.Attitude{Pitch: pitch, Yaw: yaw}
		}
		p, y, r := a.attitudeError(att, q, frame)
		return physics.Vec3{X: r, Y: p, Z: y}
	}

	// Прежний путь через углы: разрыв должен быть большим — это и есть
	// неисправность, ради которой всё делалось.
	euler := errOf(near, false).Sub(errOf(far, false)).Norm() * physics.RadToDeg
	if euler < 90 {
		t.Fatalf("эйлеров путь дал разрыв всего %.1f° — "+
			"особенность представления не воспроизвелась", euler)
	}

	// Новый путь через направление: разрыва быть не должно.
	direct := errOf(near, true).Sub(errOf(far, true)).Norm() * physics.RadToDeg
	if direct > 1 {
		t.Fatalf("цель направлением тоже прыгнула на %.3f°", direct)
	}
}

// Проход азимута через ноль и крена через ±180 не создаёт ошибки
// ориентации: корпус доворачивается вокруг оси, границ в этом нет.
func TestAttitudeErrorIgnoresAngularBoundaries(t *testing.T) {
	frame := physics.NewLocalFrame(physics.Vec3{X: physics.EarthRadius})

	var a VehicleAttitude
	worst := 0.0
	prev := math.NaN()

	// Корпус равномерно проходит полный оборот по азимуту при умеренном
	// тангаже — то есть пересекает и границу азимута, и границу крена.
	for step := 0; step <= 360; step++ {
		azimuth := float64(step)
		a.Orientation = physics.QuaternionFromBasis(
			physics.Attitude{Pitch: 20, Yaw: azimuth, Roll: 0}.BodyFrame(frame).Forward,
			physics.Attitude{Pitch: 20, Yaw: azimuth, Roll: 0}.BodyFrame(frame).Right,
			physics.Attitude{Pitch: 20, Yaw: azimuth, Roll: 0}.BodyFrame(frame).Down)

		// Цель — на градус впереди по азимуту: постоянное, ровное
		// рассогласование, которое границы обязаны пережить без следа.
		dir := frame.Direction(20, azimuth+1)
		// Опора крена переносится вместе с корпусом — как в полёте.
		q, _ := physics.AimBasis(dir, a.Orientation.Rotate(physics.Vec3{Y: 1}))
		p, y, r := a.attitudeError(physics.Attitude{}, q, frame)
		mag := physics.Vec3{X: r, Y: p, Z: y}.Norm() * physics.RadToDeg

		if !math.IsNaN(prev) {
			if jump := math.Abs(mag - prev); jump > worst {
				worst = jump
			}
		}
		prev = mag
	}
	if worst > 1e-6 {
		t.Fatalf("ошибка ориентации дрогнула на %.3g° при проходе границ", worst)
	}
}

// Накопленный крен считается ровно один раз за такт: если билдер
// телеметрии начнёт накапливать сам, повторные вызовы за тот же такт
// станут считать поворот по нескольку раз.
func TestRollIntegralAdvancesOncePerStep(t *testing.T) {
	b, _ := boosterInCoast(t, 1)

	roll := b.rollIntegral
	for i := 0; i < 5; i++ {
		boosterTelemetry(b, b.elapsed)
	}
	if got := b.rollIntegral; got != roll {
		t.Fatalf("сборка телеметрии сдвинула накопленный крен: %.6f → %.6f", roll, got)
	}
}

// Накопленный крен — это интеграл проекции угловой скорости на продольную
// ось, и ничего кроме: за такт он обязан прирасти ровно на ω_x·dt.
func TestRollIntegralFollowsBodyRate(t *testing.T) {
	b, sim := boosterInCoast(t, 1)

	before := b.rollIntegral
	rate := b.attitude.Omega.X * physics.RadToDeg
	const dt = 0.1
	b.updateContinuousAngles(dt)
	_ = sim

	if got, want := b.rollIntegral-before, rate*dt; math.Abs(got-want) > 1e-9 {
		t.Fatalf("прирост крена %.6g° против ожидаемого %.6g°", got, want)
	}
}

// У вертикально стоящего корпуса азимут и крен не определены: там они
// описывают один и тот же поворот и распределяются между собой
// произвольно. Телеметрия обязана отдавать «нет значения», а не число.
//
// Наклон оси при этом определён всегда — ради него всё и затевалось.
func TestEulerAnglesUndefinedNearVertical(t *testing.T) {
	for _, tilt := range []float64{0, 3, 9.9, 170.1, 177, 180} {
		if v := azimuthWhenDefined(123, tilt); !math.IsNaN(v) {
			t.Errorf("при наклоне оси %.1f° азимут отдан как %.1f°, ожидался NaN", tilt, v)
		}
	}
	for _, tilt := range []float64{10.1, 45, 90, 135, 169.9} {
		if v := azimuthWhenDefined(123, tilt); math.IsNaN(v) {
			t.Errorf("при наклоне оси %.1f° азимут потерян, а он определён", tilt)
		}
	}
}
