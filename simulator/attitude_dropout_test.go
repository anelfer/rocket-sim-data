package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Переживание пропуска связи датчика ориентации.
//
// На пропуске связи Simulation.step не обновляет lastValidSensedOrientation
// (см. комментарий там же) — автопилот продолжает работать с последним
// достоверным показанием, зафиксированным на момент начала пропуска, пока
// корпус по факту продолжает поворачиваться. Эта проверка воспроизводит
// именно то поведение: SensedOrientation держится неизменной весь штатный
// пропуск (см. sensing.DefaultAttitudeSensor: DropoutDuration = 0.3 с), пока
// автопилот продолжает вызывать Update. Ошибка и команда качания камер
// обязаны остаться в допустимых пределах — без насыщения привода.
// -----------------------------------------------------------------------------

func TestAttitudeSurvivesOneNominalDropout(t *testing.T) {
	var a VehicleAttitude
	a.Config = DefaultAttitudeControl()
	frame := testFrame()
	a.Init(physics.Attitude{Pitch: 80, Yaw: 45, Roll: 0}, frame)

	// Небольшая штатная поправка наведения — обычная работа контура вдали
	// от каких-либо аварийных манёвров.
	target := physics.Attitude{Pitch: 75, Yaw: 45, Roll: 0}

	in := AttitudeInput{
		Target:           target,
		Frame:            frame,
		Thrust:           2.0e6,
		Mass:             5.0e5,
		Length:           50,
		Radius:           4.5,
		CenterOfMass:     0.55,
		GimbalArm:        20,
		EngineRingRadius: 4,
		Controllable:     true,
	}

	const dt = 0.02             // штатный период обновления датчика ориентации
	const dropoutDuration = 0.3 // sensing.DefaultAttitudeSensor().DropoutDuration
	steps := int(dropoutDuration/dt) + 1

	// Пропуск связи: показание застывает на последнем достоверном значении —
	// именно так ведёт себя Simulation.step, пока m.Valid == false.
	a.SensedOrientation = a.Orientation

	for i := 0; i < steps; i++ {
		a.Update(dt, in)

		if a.Saturated {
			t.Errorf("тик %d: привод вышел на упор во время штатного пропуска связи датчика ориентации", i)
		}
		if math.Abs(a.GimbalPitch) > a.Config.MaxGimbal+1e-9 ||
			math.Abs(a.GimbalYaw) > a.Config.MaxGimbal+1e-9 {
			t.Errorf("тик %d: качание камер (%.4f/%.4f рад) вышло за предел хода %.4f",
				i, a.GimbalPitch, a.GimbalYaw, a.Config.MaxGimbal)
		}
	}
}
