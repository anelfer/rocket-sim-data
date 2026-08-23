package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Угловое движение носителя.
//
// Ориентация больше не назначается, а получается из уравнений вращения.
// Проверяется именно это: моменты существуют, корпус имеет инерцию, привод
// ограничен, а без управляющего момента статически неустойчивая ракета
// разворачивается потоком.
// -----------------------------------------------------------------------------

// controlOpt — краткая запись заданного значения поправки.
func controlOpt(v float64) control.Opt { return control.Set(v) }

// testFrame возвращает местный базис в точке на экваторе.
func testFrame() physics.LocalFrame {
	return physics.NewLocalFrame(physics.Vec3{X: physics.EarthRadius})
}

// nominalAttitudeInput возвращает условия полёта в плотных слоях.
func nominalAttitudeInput(frame physics.LocalFrame) AttitudeInput {
	return AttitudeInput{
		Target:           physics.Attitude{Pitch: 45},
		Frame:            frame,
		AirRelative:      frame.Direction(45, 0).Scale(400),
		DynamicPressure:  35000,
		Mach:             1.2,
		Thrust:           7.3e6,
		Mass:             400000,
		Length:           47,
		Radius:           1.83,
		CenterOfMass:     0.6,
		GimbalArm:        19,
		EngineRingRadius: 1.6,
		Controllable:     true,
	}
}

// Ориентация должна восстанавливаться из кватерниона без потерь.
func TestAttitudeRoundTrip(t *testing.T) {
	frame := testFrame()

	for _, want := range []physics.Attitude{
		{Pitch: 90, Yaw: 0, Roll: 0},
		{Pitch: 45, Yaw: 90, Roll: 0},
		{Pitch: 30, Yaw: 84, Roll: 26.7},
		{Pitch: -20, Yaw: 200, Roll: -45},
	} {
		var a VehicleAttitude
		a.Init(want, frame)
		got := a.AttitudeIn(frame)

		if math.Abs(got.Pitch-want.Pitch) > 1e-6 {
			t.Errorf("тангаж %.6f, задавался %.6f", got.Pitch, want.Pitch)
		}
		if math.Abs(physics.ShortestAngle(got.Yaw-want.Yaw)) > 1e-6 {
			t.Errorf("курс %.6f, задавался %.6f", got.Yaw, want.Yaw)
		}
		if math.Abs(physics.ShortestAngle(got.Roll-want.Roll)) > 1e-6 {
			t.Errorf("крен %.6f, задавался %.6f", got.Roll, want.Roll)
		}
	}
}

// Рассогласование должно считаться и в вертикальном положении.
//
// Углы Эйлера при тангаже девяносто градусов вырождаются: изменение курса
// становится неотличимо от крена. Ошибка ориентации обязана этого не замечать.
func TestAttitudeErrorHasNoSingularity(t *testing.T) {
	frame := testFrame()

	var a VehicleAttitude
	a.Init(physics.Attitude{Pitch: 90, Yaw: 45, Roll: 0}, frame)

	// Курс на вертикали не определён: тот же корпус можно описать любым курсом.
	// Рассогласование при этом обязано остаться нулевым.
	p, y, r := a.attitudeError(physics.Attitude{Pitch: 90, Yaw: 45, Roll: 0}, frame)
	if math.Abs(p) > 1e-9 || math.Abs(y) > 1e-9 || math.Abs(r) > 1e-9 {
		t.Errorf("ненулевая ошибка при совпадающей ориентации: %.3e/%.3e/%.3e", p, y, r)
	}

	// Наклон на градус от вертикали должен дать ровно градус рассогласования.
	p, y, r = a.attitudeError(physics.Attitude{Pitch: 89, Yaw: 45, Roll: 0}, frame)
	total := math.Sqrt(p*p+y*y+r*r) * physics.RadToDeg
	if math.Abs(total-1) > 0.01 {
		t.Errorf("рассогласование %.4f°, ожидался ровно градус", total)
	}
}

// Знаки качания: положительное отклонение должно поднимать нос.
func TestGimbalSigns(t *testing.T) {
	frame := testFrame()

	run := func(gimbalPitch, gimbalYaw float64) physics.Attitude {
		var a VehicleAttitude
		a.Config = DefaultAttitudeControl()
		a.Init(physics.Attitude{Pitch: 45}, frame)
		a.GimbalPitch, a.GimbalYaw = gimbalPitch, gimbalYaw

		in := nominalAttitudeInput(frame)
		in.DynamicPressure = 0 // только управляющий момент
		for i := 0; i < 50; i++ {
			a.integrateNoControl(0.01, in)
		}
		return a.AttitudeIn(frame)
	}

	up := run(0.05, 0)
	if up.Pitch <= 45 {
		t.Errorf("положительное качание опустило нос: тангаж %.2f", up.Pitch)
	}
	down := run(-0.05, 0)
	if down.Pitch >= 45 {
		t.Errorf("отрицательное качание подняло нос: тангаж %.2f", down.Pitch)
	}
	side := run(0, 0.05)
	if physics.ShortestAngle(side.Yaw) <= 0 {
		t.Errorf("качание по рысканию развернуло не в ту сторону: курс %.2f", side.Yaw)
	}
	t.Logf("Качание ±0.05 рад за полсекунды: тангаж %.2f и %.2f, курс %.2f",
		up.Pitch, down.Pitch, side.Yaw)
}

// Корпус без оперения статически неустойчив: центр давления впереди
// центра масс, и поток уводит нос, а не возвращает его.
//
// Знак этого момента был перепутан: нормальная сила направлялась ПО поперечной
// составляющей скорости, тогда как воздух движется мимо корпуса в обратную
// сторону и сила должна быть направлена ПРОТИВ. Из-за этого ракета без
// оперения получалась устойчивой, отказ управления ничем не грозил,
// а отработавшая ступень летела бы носом вперёд вместо разворота
// двигателями назад.
func TestAirframeIsStaticallyUnstable(t *testing.T) {
	frame := testFrame()

	shape := physics.AeroShape{
		Length: 47, Radius: 1.83,
		CenterOfMass: 0.60,
	}

	var a VehicleAttitude
	a.Init(physics.Attitude{Pitch: 50}, frame)

	// Нос поднят над потоком на пять градусов. Центр давления впереди центра
	// масс, значит момент обязан поднимать нос ещё выше.
	flow := frame.Direction(45, 0).Scale(400)
	torque := shape.AerodynamicTorque(35000, a.Orientation.RotateInverse(flow),
		physics.Vec3{}, 1.2)
	if torque.Y <= 0 {
		t.Errorf("момент по тангажу %.3e Н·м возвращает нос к потоку, "+
			"хотя центр давления впереди центра масс", torque.Y)
	}

	// То же самое в интегрировании: без управления угол атаки расходится.
	in := nominalAttitudeInput(frame)
	in.Controllable = false
	in.Thrust = 0
	in.Overrides.Dead = true

	start := physics.AngleOfAttack(a.Orientation.Body().Forward, in.AirRelative)
	for i := 0; i < 500; i++ {
		a.integrate(0.01, in)
	}
	end := physics.AngleOfAttack(a.Orientation.Body().Forward, in.AirRelative)

	if end <= start {
		t.Errorf("угол атаки не вырос без управления: %.3f° → %.3f°", start, end)
	}
	if a.StaticMargin >= 0 {
		t.Errorf("запас устойчивости %.2f калибра — корпус получился устойчивым, "+
			"хотя оперения у него нет", a.StaticMargin)
	}
	t.Logf("Без управления за пять секунд угол атаки %.2f° → %.2f°, "+
		"запас устойчивости %.2f калибра", start, end, a.StaticMargin)
}

// Знаковый угол атаки должен совпадать по знаку с полным углом.
//
// Прежде перед арктангенсом стоял лишний минус, и на графиках знаковый угол
// расходился с полным: один показывал −25°, другой +25°.
func TestSignedAngleOfAttackMatchesTotal(t *testing.T) {
	frame := testFrame()

	var a VehicleAttitude
	a.Init(physics.Attitude{Pitch: 50}, frame)
	body := a.Orientation.Body()

	// Нос выше потока — поток приходит снизу, угол атаки положителен.
	flow := frame.Direction(45, 0).Scale(400)
	alpha, _ := physics.AeroAngles(body, flow)
	total := physics.AngleOfAttack(body.Forward, flow)

	if alpha <= 0 {
		t.Errorf("угол атаки %.2f° отрицателен, хотя поток приходит снизу", alpha)
	}
	if math.Abs(alpha-total) > 0.01 {
		t.Errorf("знаковый угол %.2f° и полный %.2f° расходятся", alpha, total)
	}

	// Нос ниже потока — угол атаки отрицателен.
	a.Init(physics.Attitude{Pitch: 40}, frame)
	alpha, _ = physics.AeroAngles(a.Orientation.Body(), flow)
	if alpha >= 0 {
		t.Errorf("угол атаки %.2f° положителен, хотя поток приходит сверху", alpha)
	}
	t.Logf("Нос выше потока → +5°, ниже потока → %.2f°", alpha)
}

// Запас устойчивости считается по взаимному положению центра давления
// и центра масс.
func TestStaticMarginSign(t *testing.T) {
	frame := testFrame()

	var a VehicleAttitude
	a.Config = DefaultAttitudeControl()
	a.Init(physics.Attitude{Pitch: 45}, frame)

	in := nominalAttitudeInput(frame)
	in.CenterOfMass = 0.6 // центр масс позади центра давления
	a.integrate(0.01, in)

	if a.StaticMargin >= 0 {
		t.Errorf("запас устойчивости %.2f калибра при центре давления впереди "+
			"центра масс — знак перепутан", a.StaticMargin)
	}

	in.CenterOfMass = 0.35 // центр масс впереди центра давления
	a.integrate(0.01, in)
	if a.StaticMargin <= 0 {
		t.Errorf("запас устойчивости %.2f калибра при центре давления позади "+
			"центра масс", a.StaticMargin)
	}
	t.Logf("Запас устойчивости меняет знак вместе с взаимным положением " +
		"центра давления и центра масс")
}

// Автопилот обязан удерживать неустойчивый корпус.
func TestAutopilotHoldsUnstableAirframe(t *testing.T) {
	frame := testFrame()

	var a VehicleAttitude
	a.Config = DefaultAttitudeControl()
	a.Init(physics.Attitude{Pitch: 46}, frame)

	in := nominalAttitudeInput(frame)

	var worst float64
	for i := 0; i < 1000; i++ { // десять секунд
		a.integrate(0.01, in)
		err := math.Abs(a.AttitudeIn(frame).Pitch - 45)
		worst = math.Max(worst, err)
	}

	final := math.Abs(a.AttitudeIn(frame).Pitch - 45)
	if final > 0.2 {
		t.Errorf("автопилот не свёл ошибку: осталось %.3f°", final)
	}
	if worst > 1.5 {
		t.Errorf("перерегулирование %.2f° при начальном отклонении в градус", worst)
	}
	t.Logf("Начальное отклонение 1°: наибольшая ошибка %.3f°, конечная %.3f°, "+
		"качание %.3f°", worst, final, a.GimbalPitch*physics.RadToDeg)
}

// Привод ограничен по ходу и по скорости перекладки.
func TestGimbalLimits(t *testing.T) {
	frame := testFrame()

	var a VehicleAttitude
	a.Config = DefaultAttitudeControl()
	a.Init(physics.Attitude{Pitch: 0}, frame)

	// Ход привода урезан в двадцать раз: потребный момент теперь заведомо
	// больше располагаемого. Одной большой ошибки ориентации для насыщения
	// мало — автопилот сам ограничивает потребное угловое ускорение.
	in := nominalAttitudeInput(frame)
	in.Overrides.GimbalLimitFactor = controlOpt(0.05)

	a.integrate(0.01, in)
	firstStep := math.Abs(a.GimbalPitch)
	if firstStep > a.Config.GimbalRate*0.01+1e-12 {
		t.Errorf("привод переложился на %.4f рад за шаг при скорости %.4f рад/с",
			firstStep, a.Config.GimbalRate)
	}

	for i := 0; i < 200; i++ {
		a.integrate(0.01, in)
	}
	if math.Abs(a.GimbalPitch) > a.EffectiveLimit+1e-12 {
		t.Errorf("привод вышел за предел хода: %.4f против %.4f рад",
			a.GimbalPitch, a.EffectiveLimit)
	}
	if !a.Saturated {
		t.Error("при урезанном ходе привод обязан быть на упоре")
	}
	t.Logf("Упор: качание %.2f° при пределе %.2f°, запас управляемости %.2f",
		a.GimbalPitch*physics.RadToDeg, a.EffectiveLimit*physics.RadToDeg, a.Authority)
}

// Отклонение камер уводит вектор тяги и стоит потерь.
func TestGimbalCostsThrust(t *testing.T) {
	var a VehicleAttitude
	a.Init(physics.Attitude{Pitch: 90}, testFrame())

	a.GimbalPitch = 0
	if f := a.GimbalLossFactor(); math.Abs(f-1) > 1e-12 {
		t.Errorf("на нулевом качании потери %.6f", 1-f)
	}

	a.GimbalPitch = 5 * physics.DegToRad
	loss := 1 - a.GimbalLossFactor()
	if loss < 0.003 || loss > 0.005 {
		t.Errorf("потери на качании 5° составили %.4f, ожидалось около 0.0038", loss)
	}

	dir := a.ThrustDirection()
	if math.Abs(dir.Norm()-1) > 1e-9 {
		t.Errorf("направление тяги не единичное: %.9f", dir.Norm())
	}
	t.Logf("Качание 5°: потеря продольной тяги %.2f %%", loss*100)
}

// Демпфирование должно гасить вращение в атмосфере.
func TestAeroDampingStopsRotation(t *testing.T) {
	frame := testFrame()

	var a VehicleAttitude
	a.Config = DefaultAttitudeControl()
	a.Init(physics.Attitude{Pitch: 45}, frame)
	a.Omega = physics.Vec3{Y: 0.2} // около 11 °/с

	// Демпфирование проверяется в чистом виде: центр давления совмещён
	// с центром масс, поэтому уводящего момента нет и остаётся только
	// сопротивление вращению. Иначе на неустойчивом корпусе расхождение
	// пересиливает затухание, и проверить одно через другое невозможно.
	in := nominalAttitudeInput(frame)
	in.Controllable = false
	in.Thrust = 0
	in.Overrides.Dead = true
	in.CenterOfMass = 0.5

	start := math.Abs(a.Omega.Y)
	for i := 0; i < 200; i++ {
		a.integrate(0.01, in)
	}
	end := math.Abs(a.Omega.Y)

	if end >= start {
		t.Errorf("вращение не затухло: %.4f → %.4f рад/с", start, end)
	}
	t.Logf("Демпфирование за две секунды: %.3f → %.3f рад/с", start, end)
}

// Плескание топлива уводит носитель через тягу: смещённый центр масс
// оказывается не на линии действия тяги, и возникает момент M = Thrust·Δy.
func TestSloshOffsetProducesTorque(t *testing.T) {
	frame := testFrame()

	var a VehicleAttitude
	a.Init(physics.Attitude{Pitch: 45}, frame)

	// Аэродинамику и автопилот убираем, чтобы момент от плескания был
	// единственным источником вращения — иначе его не отделить от прочих.
	in := nominalAttitudeInput(frame)
	in.Controllable = false
	in.Overrides.Dead = true
	in.DynamicPressure = 0
	in.SloshLateralOffset = 0.05 // 5 см смещения центра масс

	a.integrate(0.01, in)

	want := in.Thrust * in.SloshLateralOffset
	if math.Abs(a.SloshTorque.Y-want) > 1e-6 {
		t.Errorf("момент от плескания %.3e Н·м, ожидалось %.3e", a.SloshTorque.Y, want)
	}
	if a.Omega.Y <= 0 {
		t.Errorf("угловая скорость по тангажу не выросла от плескания: %.3e рад/с",
			a.Omega.Y)
	}

	// Без смещения момента нет — это не тяга сама по себе раскачивает корпус.
	var b VehicleAttitude
	b.Init(physics.Attitude{Pitch: 45}, frame)
	baseline := in
	baseline.SloshLateralOffset = 0
	b.integrate(0.01, baseline)
	if b.SloshTorque.Y != 0 {
		t.Errorf("момент от плескания не нулевой при нулевом смещении: %.3e",
			b.SloshTorque.Y)
	}

	t.Logf("Плескание: смещение %.2f м, тяга %.1f кН → момент %.3e Н·м, ω=%.4f рад/с",
		in.SloshLateralOffset, in.Thrust/1000, a.SloshTorque.Y, a.Omega.Y)
}

// В полёте ориентация должна отставать от команды, а не совпадать с ней.
func TestBodyLagsGuidanceCommand(t *testing.T) {
	s := runningSimulation(t)

	var maxLag, sumLag float64
	n := 0
	for i := 0; i < 300; i++ {
		s.mu.Lock()
		s.step(s.Time.Step)
		s.telemetry = s.buildTelemetryLocked()
		actual := s.telemetry.Pitch
		target := s.gnc.Attitude.Pitch
		s.mu.Unlock()

		lag := math.Abs(actual - target)
		maxLag = math.Max(maxLag, lag)
		sumLag += lag
		n++
	}

	mean := sumLag / float64(n)
	if mean < 1e-6 {
		t.Error("корпус повторяет команду точь-в-точь — угловая динамика не работает")
	}
	if maxLag > 5 {
		t.Errorf("отставание корпуса от команды %.2f° слишком велико", maxLag)
	}
	t.Logf("Отставание корпуса от команды: в среднем %.3f°, наибольшее %.3f°",
		mean, maxLag)
}

// При отказе рулевого тракта носитель теряется.
//
// Управляющий момент пропадает, и статически неустойчивый корпус
// разворачивается набегающим потоком.
func TestControlFailureLosesVehicle(t *testing.T) {
	s := runningSimulation(t)

	before := s.Snapshot()
	if before.ControlTorque <= 0 {
		t.Fatalf("до отказа управляющий момент %.3g Н·м", before.ControlTorque)
	}

	mustApply(t, s, control.Command{
		ID: "tvc-out", Parameter: "control.gimbal_limit",
		Mode: control.ModeFailure, Failure: control.FailureUnpowered,
	})
	// Пятнадцати секунд мало: пока корпус раскручивается, вектор скорости
	// тоже разворачивается при гравитационном развороте, и угол между ними
	// какое-то время почти не растёт. Расхождение становится очевидным позже.
	runFor(s, 30)
	after := s.Snapshot()

	if after.ControlTorque > 1 {
		t.Errorf("после отказа управляющий момент %.3g Н·м", after.ControlTorque)
	}
	if after.TotalAoA <= before.TotalAoA {
		t.Errorf("корпус не развернуло потоком: угол атаки %.1f° → %.1f°",
			before.TotalAoA, after.TotalAoA)
	}
	if math.Abs(after.PitchRate) <= math.Abs(before.PitchRate) {
		t.Errorf("корпус не раскрутился: %.2f → %.2f °/с",
			before.PitchRate, after.PitchRate)
	}
	t.Logf("Отказ рулевого тракта: момент %.3g → %.3g Н·м, "+
		"угол атаки %.1f° → %.1f°, угловая скорость %.2f → %.2f °/с",
		before.ControlTorque, after.ControlTorque,
		before.TotalAoA, after.TotalAoA, before.PitchRate, after.PitchRate)
}

// Сокращение хода привода отнимает запас управляющего момента.
func TestReducedGimbalRaisesAuthority(t *testing.T) {
	s := runningSimulation(t)
	before := s.Snapshot().ControlAuthority

	mustApply(t, s, control.Command{
		ID: "limit", Parameter: "control.gimbal_limit",
		Mode: control.ModeStep, Value: 0.1,
	})
	runFor(s, 10)
	after := s.Snapshot()

	if after.ControlAuthority <= before {
		t.Errorf("запас управляющего момента не сократился: %.3f → %.3f",
			before, after.ControlAuthority)
	}
	if after.GimbalLimit >= 5 {
		t.Errorf("предел хода не изменился: %.2f°", after.GimbalLimit)
	}
	t.Logf("Ход привода урезан вдесятеро: предел %.2f°, потребное отклонение %.2f°, "+
		"запас %.2f, на упоре %v",
		after.GimbalLimit, after.GimbalDemand, after.ControlAuthority,
		after.ControlSaturated)
}
