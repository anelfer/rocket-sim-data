package physics

import (
	"math"
	"testing"
)

// Разворачивание проходит границу 360°/0° без разрыва.
func TestUnwrapperCrossesAzimuthWrap(t *testing.T) {
	var u AngleUnwrapper
	u.Update(359)
	if got := u.Update(1); math.Abs(got-361) > 1e-9 {
		t.Fatalf("359° → 1° развернулось в %.3f, ожидалось 361", got)
	}
	if got := u.Update(3); math.Abs(got-363) > 1e-9 {
		t.Fatalf("продолжение развернулось в %.3f, ожидалось 363", got)
	}
}

// И границу +180°/−180°.
func TestUnwrapperCrossesRollWrap(t *testing.T) {
	var u AngleUnwrapper
	u.Update(179)
	if got := u.Update(-179); math.Abs(got-181) > 1e-9 {
		t.Fatalf("179° → −179° развернулось в %.3f, ожидалось 181", got)
	}
	if got := u.Update(179); math.Abs(got-179) > 1e-9 {
		t.Fatalf("возврат назад развернулся в %.3f, ожидалось 179", got)
	}
}

// Полный оборот виден как оборот, а не как возврат в исходную точку.
func TestUnwrapperCountsFullTurns(t *testing.T) {
	var u AngleUnwrapper
	for turn := 0; turn < 3; turn++ {
		for deg := 0; deg < 360; deg += 10 {
			u.Update(float64(deg))
		}
	}
	u.Update(0)
	if got := u.Value(); math.Abs(got-1080) > 1e-6 {
		t.Fatalf("три оборота дали %.1f°, ожидалось 1080", got)
	}
}

// Развёрнутый угол ни на одном шаге не прыгает сильнее, чем сам поворот.
func TestUnwrapperNeverJumps(t *testing.T) {
	var u AngleUnwrapper
	prev := u.Update(0)
	for i := 1; i <= 2000; i++ {
		raw := NormalizeAzimuth(float64(i) * 7.3)
		got := u.Update(raw)
		if step := math.Abs(got - prev); step > 7.3+1e-6 {
			t.Fatalf("шаг %d: скачок %.3f° при повороте на 7.3°", i, step)
		}
		prev = got
	}
}

// Ошибка ориентации по кватерниону не знает границ Эйлера: смещение цели
// на доли градуса через азимутальный ноль даёт доли градуса поворота.
func TestAimQuaternionContinuousAcrossAzimuthZero(t *testing.T) {
	frame := NewLocalFrame(Vec3{X: EarthRadius})
	current := QuaternionFromBasis(
		frame.Direction(85, 359.9),
		frame.Direction(0, 89.9),
		frame.Direction(85, 359.9).Cross(frame.Direction(0, 89.9)).Unit())

	before := AimQuaternion(current, frame.Direction(85, 359.9))
	after := AimQuaternion(current, frame.Direction(85, 0.1))

	if turn := angleBetweenQuaternions(before, after); turn > 0.5 {
		t.Fatalf("цель повернулась на %.3f° при сдвиге направления на 0.2°", turn)
	}
}

// Та же проверка у самого полюса, где азимут вырождается: направление
// сдвигается на градус, цель обязана повернуться на градус, а не на сто
// восемьдесят.
func TestAimQuaternionSurvivesVerticalPole(t *testing.T) {
	frame := NewLocalFrame(Vec3{X: EarthRadius})
	up := frame.Direction(89.999, 0)
	current := AimQuaternion(Quaternion{W: 1}, up)

	worst := 0.0
	for azimuth := 0.0; azimuth < 360; azimuth += 15 {
		a := AimQuaternion(current, frame.Direction(89.5, azimuth))
		b := AimQuaternion(current, frame.Direction(89.5, azimuth+1))
		if turn := angleBetweenQuaternions(a, b); turn > worst {
			worst = turn
		}
	}
	if worst > 1.5 {
		t.Fatalf("у полюса цель прыгает на %.3f° при сдвиге азимута на 1°", worst)
	}
}

// Цель действительно приводит продольную ось куда просили.
func TestAimQuaternionHitsRequestedDirection(t *testing.T) {
	frame := NewLocalFrame(Vec3{X: EarthRadius})
	current := AimQuaternion(Quaternion{W: 1}, frame.Direction(10, 20))

	for _, want := range []Attitude{
		{Pitch: 90}, {Pitch: -90}, {Pitch: 0, Yaw: 0}, {Pitch: 0, Yaw: 180},
		{Pitch: 45, Yaw: 270}, {Pitch: -30, Yaw: 95},
	} {
		dir := frame.Direction(want.Pitch, want.Yaw)
		got := AimQuaternion(current, dir).Rotate(Vec3{X: 1})
		if off := AngleOfAttack(got, dir); off > 1e-6 {
			t.Fatalf("цель %+v промахнулась на %.3g°", want, off)
		}
	}
}

// Кратчайший поворот крен не назначает: вокруг продольной оси цель не
// поворачивается вовсе.
func TestAimQuaternionAddsNoRoll(t *testing.T) {
	frame := NewLocalFrame(Vec3{X: EarthRadius})
	current := AimQuaternion(Quaternion{W: 1}, frame.Direction(70, 40))

	want := AimQuaternion(current, frame.Direction(60, 130))
	err := current.Conjugate().Multiply(want).Normalized()
	if err.W < 0 {
		err = Quaternion{W: -err.W, X: -err.X, Y: -err.Y, Z: -err.Z}
	}
	// X векторной части — поворот вокруг связанной продольной оси.
	if math.Abs(err.X) > 1e-9 {
		t.Fatalf("кратчайший поворот дал крен: X=%.3g", err.X)
	}
}

func angleBetweenQuaternions(a, b Quaternion) float64 {
	d := a.Conjugate().Multiply(b).Normalized()
	return 2 * math.Acos(math.Min(1, math.Abs(d.W))) * RadToDeg
}

// Свидетельство самой неисправности: круг «направление → тангаж+азимут →
// базис» у вертикально стоящего корпуса рвёт опору крена. Продольная ось
// переживает круг без потерь, а «вправо» разворачивается почти на сто
// восемьдесят градусов — это и есть тот скачок, из-за которого автопилот
// получал команду на резкий крен на ровном месте.
func TestEulerRoundTripBreaksRollReferenceAtPole(t *testing.T) {
	frame := NewLocalFrame(Vec3{X: EarthRadius})

	// Два почти совпадающих направления по разные стороны от вертикали.
	a := frame.Direction(89.99, 0)
	b := frame.Direction(89.99, 180)
	if off := AngleOfAttack(a, b); off > 0.05 {
		t.Fatalf("направления разошлись на %.4f° — проверка не о том", off)
	}

	pitchA, yawA := frame.PitchAzimuth(a)
	pitchB, yawB := frame.PitchAzimuth(b)
	right := func(pitch, yaw float64) Vec3 {
		return Attitude{Pitch: pitch, Yaw: yaw}.BodyFrame(frame).Right
	}

	rollJump := AngleOfAttack(right(pitchA, yawA), right(pitchB, yawB))
	if rollJump < 90 {
		t.Fatalf("опора крена сдвинулась лишь на %.1f° — "+
			"особенность представления не воспроизвелась", rollJump)
	}

	// Кратчайший поворот на тех же данных остаётся непрерывным.
	current := AimQuaternion(Quaternion{W: 1}, a)
	turn := angleBetweenQuaternions(AimQuaternion(current, a), AimQuaternion(current, b))
	if turn > 0.1 {
		t.Fatalf("кватернионная цель тоже прыгнула: %.3f°", turn)
	}
}

// Перевод «кватернион → тангаж/рыскание/крен» обязан быть ТОЧНО обратным
// сборке базиса из тех же углов. Иначе телеметрия показывает не ту
// ориентацию, в которой корпус находится.
func TestAttitudeRoundTripIsExact(t *testing.T) {
	frame := NewLocalFrame(Vec3{X: EarthRadius * 0.6, Y: EarthRadius * 0.8})

	for _, want := range []Attitude{
		{Pitch: 0, Yaw: 0, Roll: 0},
		{Pitch: 45, Yaw: 30, Roll: 60},
		{Pitch: -60, Yaw: 200, Roll: -170},
		{Pitch: 12, Yaw: 359, Roll: 179},
		{Pitch: -12, Yaw: 1, Roll: -179},
		{Pitch: 85, Yaw: 123, Roll: 45},
	} {
		body := want.BodyFrame(frame)
		q := QuaternionFromBasis(body.Forward, body.Right, body.Down)
		got := q.AttitudeIn(frame)

		if math.Abs(got.Pitch-want.Pitch) > 1e-6 ||
			math.Abs(ShortestAngle(got.Yaw-want.Yaw)) > 1e-6 ||
			math.Abs(ShortestAngle(got.Roll-want.Roll)) > 1e-6 {
			t.Fatalf("круг не сошёлся: было %+v, стало %+v", want, got)
		}
	}
}

// Связанная тройка правая: «вперёд векторно вправо» — это «вниз».
// От этого зависят знаки всех моментов.
func TestBodyTripleIsRightHanded(t *testing.T) {
	frame := NewLocalFrame(Vec3{X: EarthRadius})
	for _, a := range []Attitude{
		{Pitch: 20, Yaw: 40, Roll: 0}, {Pitch: -70, Yaw: 310, Roll: 120},
	} {
		b := a.BodyFrame(frame)
		if off := AngleOfAttack(b.Forward.Cross(b.Right), b.Down); off > 1e-6 {
			t.Fatalf("тройка левая при %+v: расхождение %.3g°", a, off)
		}
	}
}

// Положительный крен опускает правый борт — авиационное соглашение.
func TestPositiveRollDropsRightWing(t *testing.T) {
	frame := NewLocalFrame(Vec3{X: EarthRadius})
	level := Attitude{Pitch: 0, Yaw: 0, Roll: 0}.BodyFrame(frame)
	rolled := Attitude{Pitch: 0, Yaw: 0, Roll: 30}.BodyFrame(frame)

	if _, _, up := frame.Decompose(rolled.Right); up >= 0 {
		t.Fatalf("правый борт при крене +30° не опустился: составляющая вверх %.3f", up)
	}
	if _, _, up := frame.Decompose(level.Right); math.Abs(up) > 1e-9 {
		t.Fatalf("правый борт без крена не горизонтален: %.3g", up)
	}
}

// Ось ошибки ориентации ложится на связанные оси в порядке
// (X — крен, Y — тангаж, Z — рыскание): вокруг продольной оси — крен,
// вокруг правого борта — тангаж, вокруг «вниз» — рыскание.
func TestErrorAxisMatchesBodyChannels(t *testing.T) {
	frame := NewLocalFrame(Vec3{X: EarthRadius})
	base := Attitude{Pitch: 10, Yaw: 25, Roll: 0}
	body := base.BodyFrame(frame)
	q := QuaternionFromBasis(body.Forward, body.Right, body.Down)

	const turn = 5 * DegToRad
	for name, axis := range map[string]Vec3{
		"крен": {X: 1}, "тангаж": {Y: 1}, "рыскание": {Z: 1},
	} {
		// Поворот вокруг СВЯЗАННОЙ оси — умножение справа.
		want := q.Multiply(QuaternionFromAxisAngle(axis, turn)).Normalized()
		err := q.Conjugate().Multiply(want).Normalized()
		if err.W < 0 {
			err = Quaternion{W: -err.W, X: -err.X, Y: -err.Y, Z: -err.Z}
		}
		v := Vec3{X: err.X, Y: err.Y, Z: err.Z}
		if off := AngleOfAttack(v, axis); off > 1e-6 {
			t.Fatalf("канал %s ушёл с своей оси на %.3g°", name, off)
		}
		if got := 2 * math.Asin(math.Min(1, v.Norm())) * RadToDeg; math.Abs(got-5) > 1e-6 {
			t.Fatalf("канал %s: угол %.4f° вместо 5", name, got)
		}
	}
}

// Перенесённая опора крена даёт цели ту же непрерывность, что и кратчайший
// поворот, но крен при этом ОСТАЁТСЯ управляемым: цель поворачивается вокруг
// продольной оси ровно настолько, насколько повернулась опора.
func TestAimBasisIsContinuousAndKeepsRoll(t *testing.T) {
	frame := NewLocalFrame(Vec3{X: EarthRadius})

	// Через вертикаль, где азимут вырожден.
	right := frame.Direction(0, 90)
	a, ra := AimBasis(frame.Direction(89.99, 0), right)
	b, _ := AimBasis(frame.Direction(89.99, 180), ra)
	if turn := angleBetweenQuaternions(a, b); turn > 0.1 {
		t.Fatalf("цель прыгнула на %.3f° при сдвиге направления на 0.02°", turn)
	}

	// Продольная ось приходит куда просили.
	want := frame.Direction(30, 200)
	q, r := AimBasis(want, right)
	if off := AngleOfAttack(q.Rotate(Vec3{X: 1}), want); off > 1e-9 {
		t.Fatalf("продольная ось промахнулась на %.3g°", off)
	}
	// Опора ортогональна продольной оси и единичная.
	if math.Abs(r.Dot(want.Unit())) > 1e-9 || math.Abs(r.Norm()-1) > 1e-9 {
		t.Fatalf("опора не ортонормирована: dot=%.3g norm=%.6f", r.Dot(want.Unit()), r.Norm())
	}

	// Крен управляем: развернём опору на 40° вокруг продольной оси — цель
	// обязана развернуться на те же 40°.
	turned := QuaternionFromAxisAngle(want, 40*DegToRad).Rotate(r)
	q2, _ := AimBasis(want, turned)
	if got := angleBetweenQuaternions(q, q2); math.Abs(got-40) > 1e-6 {
		t.Fatalf("крен цели повернулся на %.4f° вместо 40", got)
	}
}

// Вырожденная опора (совпала с продольной осью или не задана) не роняет
// построение: берётся любая перпендикулярная.
func TestAimBasisSurvivesDegenerateReference(t *testing.T) {
	want := Vec3{X: 1}
	for _, ref := range []Vec3{{}, {X: 1}, {X: -1}, {X: 1e-15}} {
		q, r := AimBasis(want, ref)
		if off := AngleOfAttack(q.Rotate(Vec3{X: 1}), want); off > 1e-6 {
			t.Fatalf("опора %+v: продольная ось ушла на %.3g°", ref, off)
		}
		if math.Abs(r.Norm()-1) > 1e-9 || math.Abs(r.Dot(want)) > 1e-9 {
			t.Fatalf("опора %+v: получилась %+v", ref, r)
		}
	}
}
