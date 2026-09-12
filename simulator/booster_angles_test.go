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

// Разворачивание углов бустера накапливается ровно один раз за такт: если
// билдер телеметрии начнёт накапливать сам, повторные вызовы за тот же
// такт станут считать поворот по нескольку раз.
func TestContinuousAnglesAdvanceOncePerStep(t *testing.T) {
	b, _ := boosterInCoast(t, 1)

	yaw, roll := b.yawContinuous.Value(), b.rollContinuous.Value()
	for i := 0; i < 5; i++ {
		boosterTelemetry(b, b.elapsed)
	}
	if got := b.yawContinuous.Value(); got != yaw {
		t.Fatalf("сборка телеметрии сдвинула рыскание: %.6f → %.6f", yaw, got)
	}
	if got := b.rollContinuous.Value(); got != roll {
		t.Fatalf("сборка телеметрии сдвинула крен: %.6f → %.6f", roll, got)
	}
}

// Развёрнутые углы согласованы с сырыми: их кратчайшая разность нулевая.
func TestContinuousAnglesAgreeWithRaw(t *testing.T) {
	b, _ := boosterInCoast(t, 1)
	frame := physics.NewLocalFrame(b.state.Position)
	att := b.attitude.AttitudeIn(frame)

	if off := math.Abs(physics.ShortestAngle(b.yawContinuous.Value() - att.Yaw)); off > 1e-6 {
		t.Fatalf("непрерывное рыскание разошлось с сырым на %.4g°", off)
	}
	if off := math.Abs(physics.ShortestAngle(b.rollContinuous.Value() - att.Roll)); off > 1e-6 {
		t.Fatalf("непрерывный крен разошёлся с сырым на %.4g°", off)
	}
}
