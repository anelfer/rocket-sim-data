package physics

import (
	"math"
	"testing"
)

// Плоская панель поперёк потока должна давать ньютоновский коэффициент
// давления, равный двум.
func TestNewtonianFlatPlateCoefficient(t *testing.T) {
	// Панель с нормалью против потока: поток идёт вдоль -X, нормаль +X.
	s := ControlSurface{
		Name: "test", Area: 2,
		Neutral: Vec3{X: 1}, Hinge: Vec3{Y: 1},
		Arm: Vec3{X: 0},
	}

	const q = 5000
	air := Vec3{X: 100} // корпус летит по +X, поток идёт по -X

	f := s.Force(0, air, q, 10)

	// Cp = 2 при нормальном падении, значит сила = 2·q·A.
	want := 2 * q * s.Area
	if math.Abs(f.Norm()-want) > 1 {
		t.Errorf("сила %.0f Н, ожидалось %.0f Н", f.Norm(), want)
	}

	// Поток должен сносить панель по своему направлению, то есть по -X.
	if f.X >= 0 {
		t.Errorf("сила направлена против потока: %+v", f)
	}
}

// Панель, поставленная ребром к потоку, силы не создаёт.
func TestSurfaceEdgeOnGivesNoForce(t *testing.T) {
	s := ControlSurface{
		Name: "test", Area: 2,
		Neutral: Vec3{Z: 1}, Hinge: Vec3{Y: 1},
	}

	f := s.Force(0, Vec3{X: 100}, 5000, 10)
	if f.Norm() > 1 {
		t.Errorf("панель ребром к потоку даёт силу %.1f Н", f.Norm())
	}
}

// Сила должна падать как квадрат синуса угла к потоку.
func TestNewtonianFollowsSineSquared(t *testing.T) {
	s := ControlSurface{
		Name: "test", Area: 1,
		Neutral: Vec3{X: 1}, Hinge: Vec3{Y: 1},
	}

	const q = 1000
	air := Vec3{X: 100}

	for _, angle := range []float64{15, 30, 45, 60} {
		rad := angle * DegToRad
		got := s.Force(rad, air, q, 10).Norm()

		// Отклонение панели на угол уводит её нормаль от потока,
		// и коэффициент падает как квадрат косинуса этого угла.
		want := 2 * q * s.Area * math.Cos(rad) * math.Cos(rad)
		if math.Abs(got-want) > want*0.01 {
			t.Errorf("при отклонении %.0f° сила %.0f Н, ожидалось %.0f Н",
				angle, got, want)
		}
	}
}

// Момент считается относительно центра масс: панель впереди и панель позади
// при одинаковом отклонении крутят корпус в разные стороны.
func TestForwardAndAftSurfacesOpposeEachOther(t *testing.T) {
	fwd := ControlSurface{
		Name: "fwd", Area: 15, Arm: Vec3{X: 20},
		Neutral: Vec3{Z: -1}, Hinge: Vec3{Y: 1},
	}
	aft := ControlSurface{
		Name: "aft", Area: 15, Arm: Vec3{X: -15},
		Neutral: Vec3{Z: -1}, Hinge: Vec3{Y: 1},
	}

	// Корпус летит брюхом вперёд: поток снизу.
	air := Vec3{Z: -2000}

	mFwd := fwd.Torque(0, air, 12000, 20, 0)
	mAft := aft.Torque(0, air, 12000, 20, 0)

	if mFwd.Y*mAft.Y >= 0 {
		t.Errorf("моменты не противоположны: передний %.0f, задний %.0f Н·м",
			mFwd.Y, mAft.Y)
	}
}

// TestSurfaceTorqueUsesDynamicLeverFromCoM — регрессия на баг с
// фиксированным Arm.X у решётчатых рулей (GridFins): поверхность с
// PositionFromNose обязана считать момент как (PositionFromNose−comFromNose)
// × F, а не игнорировать comFromNose.
func TestSurfaceTorqueUsesDynamicLeverFromCoM(t *testing.T) {
	s := ControlSurface{
		Name: "fin", Area: 15, PositionFromNose: 40,
		Neutral: Vec3{Z: -1}, Hinge: Vec3{Y: 1},
	}
	air := Vec3{Z: -2000}
	const q = 12000
	const mach = 20

	f := s.Force(0, air, q, mach)

	for _, com := range []float64{0, 20, 39, 40, 41, 60} {
		got := s.Torque(0, air, q, mach, com)
		wantArm := Vec3{X: 40 - com}
		want := wantArm.Cross(f)
		if math.Abs(got.Y-want.Y) > 1e-6*math.Max(1, math.Abs(want.Y)) {
			t.Errorf("com=%.0f: torque.Y=%.4f, ожидание (r_fin-r_COM)×F=%.4f", com, got.Y, want.Y)
		}
	}
}

// TestSurfaceTorqueSignFlipsWhenCoMCrossesFin — если CoM проходит физическую
// позицию руля, плечо и, следовательно, момент обязаны сменить знак: до
// этого момента руль "спереди" CoM, после — "сзади".
func TestSurfaceTorqueSignFlipsWhenCoMCrossesFin(t *testing.T) {
	s := ControlSurface{
		Name: "fin", Area: 15, PositionFromNose: 40,
		Neutral: Vec3{Z: -1}, Hinge: Vec3{Y: 1},
	}
	air := Vec3{Z: -2000}
	const q = 12000
	const mach = 20

	before := s.Torque(0, air, q, mach, 30) // CoM перед рулём (30 < 40)
	after := s.Torque(0, air, q, mach, 50)  // CoM позади руля (50 > 40)

	if before.Y == 0 || after.Y == 0 {
		t.Fatalf("нулевой момент в тесте на смену знака: before=%.4f after=%.4f", before.Y, after.Y)
	}
	if before.Y*after.Y >= 0 {
		t.Errorf("момент не сменил знак при переходе CoM через позицию руля: before=%.4f after=%.4f", before.Y, after.Y)
	}
}

// TestSurfaceTorqueMagnitudeGrowsWithLever — при отдалении CoM от руля
// (растущее |lever|) момент по модулю должен расти линейно, при том же F.
func TestSurfaceTorqueMagnitudeGrowsWithLever(t *testing.T) {
	s := ControlSurface{
		Name: "fin", Area: 15, PositionFromNose: 40,
		Neutral: Vec3{Z: -1}, Hinge: Vec3{Y: 1},
	}
	air := Vec3{Z: -2000}
	const q = 12000
	const mach = 20

	var prevAbs float64
	for i, com := range []float64{39, 35, 30, 20, 0} {
		got := s.Torque(0, air, q, mach, com)
		abs := math.Abs(got.Y)
		lever := math.Abs(40 - com)
		ratio := abs / lever
		if i > 0 && math.Abs(abs) <= prevAbs {
			t.Errorf("com=%.0f: |torque|=%.2f не выросло при увеличении плеча (было %.2f)", com, abs, prevAbs)
		}
		prevAbs = abs
		_ = ratio
	}
}

// TestForceAndTorqueUsesSameForce — регрессия на архитектурный баг:
// аэродинамическая сила панели раньше участвовала только в r×F (момент), а
// в поступательное движение не попадала вовсе. ForceAndTorque обязан
// возвращать РОВНО ту же силу, что и Force() (не пересчитанную заново и не
// какую-то отдельную "силу сопротивления"), а момент — строиться как
// arm×(эта же сила).
func TestForceAndTorqueUsesSameForce(t *testing.T) {
	s := ControlSurface{
		Name: "fin", Area: 15, PositionFromNose: 40,
		Neutral: Vec3{Z: -1}, Hinge: Vec3{Y: 1},
	}
	air := Vec3{Z: -2000}
	const q, mach, deflection, comFromNose = 12000.0, 20.0, 0.3, 25.0

	wantForce := s.Force(deflection, air, q, mach)
	gotForce, gotTorque := s.ForceAndTorque(deflection, air, q, mach, comFromNose)

	if gotForce != wantForce {
		t.Errorf("ForceAndTorque вернул другую силу: %v, ожидалось Force()=%v", gotForce, wantForce)
	}

	wantArm := Vec3{X: 40 - comFromNose}
	wantTorque := wantArm.Cross(wantForce)
	if math.Abs(gotTorque.Y-wantTorque.Y) > 1e-6*math.Max(1, math.Abs(wantTorque.Y)) {
		t.Errorf("момент не равен arm×F: got=%v, want=%v", gotTorque, wantTorque)
	}

	// И старый Torque() обязан давать то же самое, что ForceAndTorque —
	// иначе Torque() и ForceAndTorque() расходятся, что и означало бы
	// двойной счёт/рассинхронизацию силы и момента.
	if legacy := s.Torque(deflection, air, q, mach, comFromNose); legacy != gotTorque {
		t.Errorf("Torque()=%v разошёлся с ForceAndTorque()=%v", legacy, gotTorque)
	}
}

// TestSurfaceWithoutPositionFromNoseIgnoresCoM — старое поведение (плавники
// корабля, ShipFlaps, PositionFromNose не задан) не должно зависеть от
// comFromNose вообще — регрессия на то, чтобы фикс GridFins не задел
// ShipFlaps.
func TestSurfaceWithoutPositionFromNoseIgnoresCoM(t *testing.T) {
	s := ControlSurface{
		Name: "flap", Area: 15, Arm: Vec3{X: 20},
		Neutral: Vec3{Z: -1}, Hinge: Vec3{Y: 1},
	}
	air := Vec3{Z: -2000}
	const q = 12000
	const mach = 20

	ref := s.Torque(0, air, q, mach, 0)
	for _, com := range []float64{-100, 0, 5, 39, 1000} {
		got := s.Torque(0, air, q, mach, com)
		if got != ref {
			t.Errorf("com=%.0f: torque=%v изменился при пустом PositionFromNose (ожидание — как при com=0: %v)", com, got, ref)
		}
	}
}
