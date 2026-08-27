package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Регрессия на отсутствующую поступательную силу решётчатых рулей.
//
// До фикса SurfaceSet.UpdateTorque считал момент от аэродинамической силы
// панелей, а саму силу — нигде не сохранял: она участвовала только в r×F
// и на этом заканчивалась, в totalForce поступательного движения (booster.go,
// RK4Step) не попадая вовсе. Рули создавали момент, как будто вызывающая
// его сила на центр масс корпуса не действует, — нефизично. Тесты здесь
// проверяют, что s.Force теперь несёт ТУ ЖЕ силу, из которой получен момент,
// и что для физически осмысленных конфигураций она ведёт себя так, как
// должна вести себя реальная аэродинамическая сила: тормозит при симметричном
// раскрытии и не обнуляется произвольно при дифференциальной команде.
// -----------------------------------------------------------------------------

// TestSurfaceSetForce_SymmetricDeflectionGivesAxialDrag — п.5, ПЕРЕСМОТРЕНО
// под T-раскладку Super Heavy V3 (fin_1/fin_3 друг напротив друга, fin_2
// перпендикулярно — см. vehicle.gridFinAzimuths).
//
// Для прежнего 120°-кольца одинаковое отклонение всех трёх рулей гасило
// боковую силу ЦЕЛИКОМ (3-кратная поворотная симметрия). У T-раскладки
// этой симметрии нет: fin_1 (белый борт, arm.Z=+r) и fin_3 (спина,
// arm.Z=−r) заданы зеркальными Hinge — при одинаковом по знаку отклонении
// их вклады в Z-составляющую силы СКЛАДЫВАЮТСЯ, а не гасятся (тот же
// геометрический факт, что раньше — при одностороннем диапазоне актуатора —
// делало pitch практически односторонним, см. TestAllocateOptimal_
// PitchIsSymmetric). Измерено: Y гасится (fin_2 сидит
// точно на плоскости зеркальной симметрии Y), Z — нет. Это не баг, а
// реальное свойство новой геометрии: равномерное раскрытие всех рулей
// (наивный "воздушный тормоз") у T-раскладки даёт заметную боковую силу,
// не только продольную — то, что для 120°-кольца было даровой
// симметрией, для T нужно либо принять, либо компенсировать явно.
func TestSurfaceSetForce_SymmetricDeflectionGivesAxialDrag(t *testing.T) {
	surf, _, q, mach, comFromNose := testGridFinSet()
	// Чисто осевой поток — без бокового Y-сноса тестового фикстура,
	// чтобы не путать геометрическую асимметрию рулей с боковым ветром.
	airBody := physics.Vec3{X: -1000}

	deflect := 15.0 * physics.DegToRad
	for i := range surf.Surfaces {
		surf.Deflection[i] = deflect
	}
	surf.UpdateTorque(airBody, q, mach, comFromNose)

	const lateralTolerance = 1.0 // Н — численный нуль на фоне ~1e5 Н продольной силы
	if math.Abs(surf.Force.Y) > lateralTolerance {
		t.Errorf("Y-составляющая не погасилась при симметричном раскрытии (ожидалась — fin_2 на оси "+
			"зеркальной симметрии): Force=%v", surf.Force)
	}
	// Z НЕ обязана гаситься у T-раскладки — см. комментарий к тесту.
	// Проверяем только, что она конечна и не абсурдно велика (не в разы
	// больше продольной).
	if math.Abs(surf.Force.Z) > math.Abs(surf.Force.X)*3 {
		t.Errorf("боковая (Z) составляющая непропорционально велика относительно продольной: Force=%v", surf.Force)
	}

	if surf.Force.X <= 0 {
		t.Fatalf("продольная сила не создана (или неверного знака): Force.X=%.1f", surf.Force.X)
	}

	// Сила обязана тормозить, а не толкать: работа над телом относительно
	// воздуха отрицательна (F·v_rel < 0) — см. TestFinForceDoesNegativeWorkOnAir
	// для явной проверки этого же факта на уровне мировых осей.
	if dot := surf.Force.Dot(airBody); dot >= 0 {
		t.Errorf("сила рулей не тормозит: F·airBody=%.3e (ожидался отрицательный — сила против относительного движения)", dot)
	}
}

// TestSurfaceSetForce_DifferentialCommandStillProducesRealForce — п.6:
// дифференциальная команда (чистый pitch) не обязана давать нулевую боковую
// силу — рули физически одновременно тормозят, создают боковую силу и
// момент, и allocator не должен искусственно обнулять ни одно из них.
// Проверяется только то, что результирующая сила ненулевая и конечная —
// не то, что она равна нулю или какому-то заранее заданному значению.
func TestSurfaceSetForce_DifferentialCommandStillProducesRealForce(t *testing.T) {
	surf, airBody, q, mach, comFromNose := testGridFinSet()

	surf.Allocate(physics.Vec3{Y: 2e6}, airBody, q, mach, comFromNose)
	copy(surf.Deflection, surf.Command)
	surf.UpdateTorque(airBody, q, mach, comFromNose)

	if !surf.Force.IsFinite() {
		t.Fatalf("Force не конечна при дифференциальной команде: %v", surf.Force)
	}
	if surf.Force.Norm() < 1 {
		t.Errorf("дифференциальная команда дала нулевую силу: Force=%v — рули физически не могут "+
			"создавать момент без создающей его силы", surf.Force)
	}
}

// TestSurfaceSetForce_MatchesSumOfPerSurfaceForces — прямая проверка на
// двойной счёт: s.Force обязана быть суммой Force() каждой отдельной
// панели при её текущем отклонении, не пересчитанной как-то иначе.
func TestSurfaceSetForce_MatchesSumOfPerSurfaceForces(t *testing.T) {
	surf, airBody, q, mach, comFromNose := testGridFinSet()
	for i := range surf.Surfaces {
		surf.Deflection[i] = float64(i+1) * 3 * physics.DegToRad // разные углы — не вырожденный случай
	}
	surf.UpdateTorque(airBody, q, mach, comFromNose)

	want := physics.Vec3{}
	for i, s := range surf.Surfaces {
		want = want.Add(s.Force(surf.Deflection[i], airBody, q, mach))
	}

	const tol = 1e-6
	d := surf.Force.Sub(want)
	if d.Norm() > tol*math.Max(1, want.Norm()) {
		t.Errorf("SurfaceSet.Force=%v не равна сумме отдельных Force()=%v", surf.Force, want)
	}
}

// TestForceModelFinForce_ReachesAcceleration — п.4 (translation-часть):
// ForceModel.FinForce обязана попадать в Evaluate().Total как F/m, точно
// так же, как тяга и сопротивление, — иначе фикс SurfaceSet ничего не меняет
// в реальном поступательном движении.
func TestForceModelFinForce_ReachesAcceleration(t *testing.T) {
	base := ForceModel{DryMass: 500000}
	withFin := base
	withFin.FinForce = physics.Vec3{X: 1e6}

	state := VehicleState{Position: physics.Vec3{X: physics.EarthRadius + 100000}}

	accBase := base.Evaluate(state)
	accFin := withFin.Evaluate(state)

	wantDelta := withFin.FinForce.Scale(1 / 500000.0)
	gotDelta := accFin.Total.Sub(accBase.Total)

	const tol = 1e-9
	d := gotDelta.Sub(wantDelta)
	if d.Norm() > tol*math.Max(1, wantDelta.Norm()) {
		t.Errorf("FinForce не отразилась в Total ожидаемым образом: got Δ=%v, want Δ=%v", gotDelta, wantDelta)
	}
	if accFin.FinForce != wantDelta {
		t.Errorf("acc.FinForce=%v, ожидалось %v", accFin.FinForce, wantDelta)
	}
}

// TestForceModelFinForce_ZeroValueIsNoOp — не влияет на поведение тел, у
// которых FinForce не заполняется (пока — все, кроме бустера): нулевой
// вектор должен давать точно тот же Total, что и раньше.
func TestForceModelFinForce_ZeroValueIsNoOp(t *testing.T) {
	fm := ForceModel{DryMass: 500000, Thrust: 1e6, ThrustDir: physics.Vec3{X: 1}}
	state := VehicleState{Position: physics.Vec3{X: physics.EarthRadius + 100000}, FuelMass: 1000}

	acc := fm.Evaluate(state)
	if acc.FinForce != (physics.Vec3{}) {
		t.Errorf("acc.FinForce=%v, ожидался нулевой вектор без FinForce на входе", acc.FinForce)
	}
}
