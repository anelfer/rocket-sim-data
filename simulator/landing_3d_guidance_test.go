package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Регрессия на Stage 4.4 (3D powered landing guidance): desired thrust vector
// строится из вертикальной (Stage 4.3) И горизонтальной (landingHorizontal-
// AccelDemand) задачи разом (landingDesiredThrust), а attitude target —
// из направления этого вектора (landingBurnTarget), а не назначается
// фиксированной вертикалью. Тесты здесь проверяют свойства этой зависимости
// на синтетических состояниях (см. Stage 4.4, п.26-30), без полного
// прогона миссии.
// -----------------------------------------------------------------------------

// nav3D строит NavState с настоящим LocalFrame и заданной горизонтальной
// (vx восток, vy север, м/с) и вертикальной (descentSpeed, м/с,
// положительно при снижении) составляющими скорости. В отличие от navAt
// (Stage 4.3, landing_sequencing_test.go) — та оставляет Frame нулевым,
// чего достаточно для чисто вертикальных тестов (landingRequiredThrust не
// трогает Frame), но недостаточно здесь: landingHorizontalVelocity/
// landingDesiredThrust реально используют Frame.East/North/Up.
func nav3D(altitude, descentSpeed, vx, vy float64) orbit.NavState {
	frame := physics.NewLocalFrame(physics.Vec3{X: physics.EarthRadius + altitude})
	ground := frame.East.Scale(vx).Add(frame.North.Scale(vy)).Add(frame.Up.Scale(-descentSpeed))
	return orbit.NavState{
		Altitude:               altitude,
		RadialVelocity:         -descentSpeed,
		Frame:                  frame,
		GroundRelativeVelocity: ground,
	}
}

// primeDesiredThrust — тот же приём прогрева landingAccelBudgetSmoothed, что
// primeRequiredThrust (landing_vertical_guidance_test.go, Stage 4.3), только
// возвращающий полный 3D вектор.
func primeDesiredThrust(b *Booster, nav orbit.NavState) (physics.Vec3, bool) {
	b.landingAccelBudgetSmoothed = 0
	b.updateLandingAccelBudget(nav, 0)
	return b.landingDesiredThrust(nav)
}

// TestLanding3D_ZeroHorizontalReducesToStage43 — п.26 Stage 4.4: при Vh=0 и
// без горизонтальной position-коррекции (её ещё нет — см. п.21-22) новое
// guidance обязано практически сводиться к чисто вертикальному Stage 4.3:
// нулевая горизонтальная составляющая вектора, вертикальная совпадает с
// landingRequiredThrust.
func TestLanding3D_ZeroHorizontalReducesToStage43(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingMid)
	nav := nav3D(1500, 150, 0, 0)

	vec, saturated := primeDesiredThrust(b, nav)
	if saturated {
		t.Fatalf("saturated=true при нулевой горизонтальной скорости — насыщать нечего")
	}

	vertical := b.landingRequiredThrust(nav)
	e, n, u := nav.Frame.Decompose(vec)
	if math.Abs(e) > 1 || math.Abs(n) > 1 {
		t.Errorf("горизонтальная составляющая не нулевая при Vh=0: east=%.3f north=%.3f", e, n)
	}
	if math.Abs(u-vertical) > 1 {
		t.Errorf("вертикальная составляющая (%.0f Н) разошлась с landingRequiredThrust (%.0f Н) при Vh=0", u, vertical)
	}
}

// TestLanding3D_MirroredHorizontalVelocity — п.27 Stage 4.4: +Vx и -Vx (и
// аналогично по Y) обязаны давать зеркально противоположные горизонтальные
// составляющие требуемой тяги — знак меняется, модуль совпадает.
func TestLanding3D_MirroredHorizontalVelocity(t *testing.T) {
	// Большая группа (13 камер) и умеренный вертикальный спрос — запас над
	// Tmax(13) остаётся достаточным, чтобы сравнение горизонтальной части
	// не упиралось в насыщение по вертикали на обоих знаках разом (см.
	// историю правки — на группе 5 с крутым снижением вертикаль ОДНА уже
	// исчерпывала Tmax(5), и обе стороны зеркально давали строгий 0).
	const altitude, descent, speed = 3000.0, 50.0, 80.0

	bx1 := landingBoosterOnGroup(t, 1, boosterLandingHigh)
	navPosX := nav3D(altitude, descent, speed, 0)
	vecPosX, _ := primeDesiredThrust(bx1, navPosX)

	bx2 := landingBoosterOnGroup(t, 1, boosterLandingHigh)
	navNegX := nav3D(altitude, descent, -speed, 0)
	vecNegX, _ := primeDesiredThrust(bx2, navNegX)

	// Тяга ГАСИТ снос, а не следует за ним: при +Vx (восток) требуемая
	// составляющая отрицательна (запад — тормозит), при -Vx — наоборот.
	ePos, _, _ := navPosX.Frame.Decompose(vecPosX)
	eNeg, _, _ := navNegX.Frame.Decompose(vecNegX)
	if !(ePos < 0 && eNeg > 0) {
		t.Errorf("восточная составляющая не меняет знак зеркально по Vx (должна тормозить снос): +Vx→%.0f Н, -Vx→%.0f Н", ePos, eNeg)
	}
	if math.Abs(ePos+eNeg) > math.Abs(ePos)*0.01+1 {
		t.Errorf("асимметрия между +Vx и -Vx: %.0f Н против %.0f Н", ePos, eNeg)
	}

	by1 := landingBoosterOnGroup(t, 1, boosterLandingHigh)
	navPosY := nav3D(altitude, descent, 0, speed)
	vecPosY, _ := primeDesiredThrust(by1, navPosY)

	by2 := landingBoosterOnGroup(t, 1, boosterLandingHigh)
	navNegY := nav3D(altitude, descent, 0, -speed)
	vecNegY, _ := primeDesiredThrust(by2, navNegY)

	_, nPos, _ := navPosY.Frame.Decompose(vecPosY)
	_, nNeg, _ := navNegY.Frame.Decompose(vecNegY)
	if !(nPos < 0 && nNeg > 0) {
		t.Errorf("северная составляющая не меняет знак зеркально по Vy (должна тормозить снос): +Vy→%.0f Н, -Vy→%.0f Н", nPos, nNeg)
	}
	if math.Abs(nPos+nNeg) > math.Abs(nPos)*0.01+1 {
		t.Errorf("асимметрия между +Vy и -Vy: %.0f Н против %.0f Н", nPos, nNeg)
	}
}

// TestLanding3D_LargerHorizontalNeedsMoreLateralThrust — п.28 Stage 4.4: при
// одинаковом вертикальном состоянии большая |Vh| обязана требовать не
// меньшую боковую тягу, вплоть до физического насыщения. Группа взята
// большая (13 камер) — запаса тяги достаточно, чтобы обе точки остались
// без насыщения и сравнение было честным.
func TestLanding3D_LargerHorizontalNeedsMoreLateralThrust(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)
	const altitude, descent = 3000.0, 50.0

	navSmall := nav3D(altitude, descent, 30, 0)
	vecSmall, satSmall := primeDesiredThrust(b, navSmall)
	navBig := nav3D(altitude, descent, 90, 0)
	vecBig, satBig := primeDesiredThrust(b, navBig)

	if satSmall || satBig {
		t.Fatalf("тест предполагает отсутствие насыщения на большой группе: small=%v big=%v", satSmall, satBig)
	}

	eSmall, _, _ := navSmall.Frame.Decompose(vecSmall)
	eBig, _, _ := navBig.Frame.Decompose(vecBig)
	if !(math.Abs(eBig) > math.Abs(eSmall)) {
		t.Errorf("большая горизонтальная скорость не увеличила боковую тягу: |Vx=30|→%.0f Н, |Vx=90|→%.0f Н",
			math.Abs(eSmall), math.Abs(eBig))
	}
}

// TestLanding3D_SaturatesWithoutNaN — п.29 Stage 4.4: заведомо недостижимое
// состояние (огромная горизонтальная скорость на маленькой группе) обязано
// дать явный saturated=true, конечный (не NaN/Inf) вектор, и сохранить
// вертикальную составляющую НЕТРОНУТОЙ (приоритет вертикальной
// безопасности) — урезается только горизонтальная часть.
func TestLanding3D_SaturatesWithoutNaN(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingLow)
	nav := nav3D(1500, 150, 500, 0)

	vec, saturated := primeDesiredThrust(b, nav)
	if !saturated {
		t.Fatalf("ожидалось saturated=true при заведомо недостижимой горизонтальной скорости")
	}
	if math.IsNaN(vec.X) || math.IsNaN(vec.Y) || math.IsNaN(vec.Z) ||
		math.IsInf(vec.X, 0) || math.IsInf(vec.Y, 0) || math.IsInf(vec.Z, 0) {
		t.Fatalf("desired thrust vector не конечен: %+v", vec)
	}

	vertical := b.landingRequiredThrust(nav)
	_, _, u := nav.Frame.Decompose(vec)
	if math.Abs(u-vertical) > math.Abs(vertical)*0.01+1 {
		t.Errorf("вертикальная составляющая урезана при насыщении (должна быть приоритетом): "+
			"получено %.0f Н, ожидалось %.0f Н", u, vertical)
	}
}

// TestLanding3D_AngularOvershootProtection — п.15,30 Stage 4.4: если
// motion-profile уже несёт большую угловую скорость, а до цели остался
// небольшой угол, профиль обязан ТОРМОЗИТЬ (уменьшать commandedRate), а не
// продолжать его разгонять — иначе тело проскочит цель по инерции.
func TestLanding3D_AngularOvershootProtection(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingMid)
	nav := nav3D(500, 50, 10, 0)

	for i := 0; i < 5; i++ {
		b.landingBurnTarget(nav, 0.1)
	}

	// Искусственно разгоняем профиль до заведомо избыточной угловой
	// скорости — как если бы корпус уже почти долетел до цели, но нёс
	// большую ω (см. п.15: "tilt decreases → nearly vertical → omega still
	// large → overshoot").
	b.landingCommandedRate = 5.0
	before := b.landingCommandedRate

	b.landingBurnTarget(nav, 0.1)

	if b.landingCommandedRate >= before {
		t.Errorf("профиль не тормозит избыточную угловую скорость вблизи цели: было %.3f рад/с, стало %.3f рад/с",
			before, b.landingCommandedRate)
	}
}
