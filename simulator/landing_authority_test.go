package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Регрессия на Stage 4.6 (authority-constrained q-aware guidance):
// landingAuthorityConstrainedThrust не должна урезать вертикальную
// безопасность, обязана быть согласована с landingDesiredThrust при
// достаточной authority, и обязана честно репортить marginLimited/margin
// на синтетических состояниях, где authority заведомо не хватает —
// без NaN и без скрытого проваливания вертикального приоритета.
// -----------------------------------------------------------------------------

// nav3DWithAirflow — nav3D (Stage 4.4) плюс заполненные AirRelativeVelocity/
// DynamicPressure: landingPredictedAoAAndAeroTorque/landingAvailableCorrectiveTorque
// реально их читают (в отличие от большинства функций Stage 4.3/4.4, где
// AirRelativeVelocity/DynamicPressure не участвуют).
func nav3DWithAirflow(altitude, descentSpeed, vx, vy, dynamicPressure float64) orbit.NavState {
	nav := nav3D(altitude, descentSpeed, vx, vy)
	nav.AirRelativeVelocity = nav.GroundRelativeVelocity
	nav.DynamicPressure = dynamicPressure
	return nav
}

// horizontalMag возвращает модуль горизонтальной (East/North) части вектора
// v в местной плоскости nav.Frame — physics.LocalFrame.Decompose возвращает
// (east,north,up) по отдельности, а не готовый модуль, и брать только один
// из первых двух компонентов (например north) ошибочно всякий раз, когда
// вся горизонтальная коррекция лежит в другом (east): такая ошибка
// однажды уже дала ложный "ноль" в этом самом файле.
func horizontalMag(nav orbit.NavState, v physics.Vec3) float64 {
	e, n, _ := nav.Frame.Decompose(v)
	return math.Hypot(e, n)
}

// TestLandingAuthority_ZeroHorizontalStaysUnconstrained — п. Stage 4.6,
// regression: при Vh=0 (candidate — чистая вертикаль, AoA≈180°, синус угла
// атаки около нуля) предсказанный аэродинамический момент пренебрежимо мал,
// margin обязан остаться положительным, и результат должен совпасть с
// landingDesiredThrust (никакого урезания там, где его не за что применять).
func TestLandingAuthority_ZeroHorizontalStaysUnconstrained(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingMid)
	nav := nav3DWithAirflow(1500, 150, 0, 0, 30000)

	plain, plainSat := b.landingDesiredThrust(nav)
	constrained, sat, limited, margin := b.landingAuthorityConstrainedThrust(nav)

	if limited {
		t.Errorf("marginLimited=true при Vh=0 (AoA≈180°, момент около нуля): margin=%.3e", margin)
	}
	if sat != plainSat {
		t.Errorf("saturated разошёлся с landingDesiredThrust: %v vs %v", sat, plainSat)
	}
	if d := constrained.Sub(plain).Norm(); d > plain.Norm()*0.01+1 {
		t.Errorf("при Vh=0 authority-constrained вектор (%+v) заметно разошёлся с landingDesiredThrust (%+v)", constrained, plain)
	}
}

// TestLandingAuthority_VerticalComponentNeverReduced — вертикальная
// безопасность первым приоритетом (п. Stage 4.6, то же требование, что и
// Stage 4.4 п.9): что бы ни решил authority-constraint по горизонтали,
// вертикальная составляющая обязана остаться равна landingRequiredThrust,
// не уменьшенной.
func TestLandingAuthority_VerticalComponentNeverReduced(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingMid)
	// Большая горизонтальная скорость и заметный q — типичная ситуация,
	// где authority-constraint может сработать.
	nav := nav3DWithAirflow(1500, 150, 300, 0, 60000)

	wantVertical := b.landingRequiredThrust(nav)
	constrained, _, _, _ := b.landingAuthorityConstrainedThrust(nav)

	_, _, u := nav.Frame.Decompose(constrained)
	if math.Abs(u-wantVertical) > math.Abs(wantVertical)*0.01+1 {
		t.Errorf("вертикальная составляющая изменилась под authority-constraint: получено %.0f Н, ожидалось %.0f Н",
			u, wantVertical)
	}
}

// TestLandingAuthority_InsufficientMarginReducesHorizontal — заведомо
// недостаточная authority обязана дать marginLimited=true и горизонтальную
// составляющую МЕНЬШЕ, чем у нестеснённого landingDesiredThrust — без
// NaN/Inf.
//
// Обычный (безветренный) сценарий здесь НЕ годится: закон гашения сноса
// (a_h≈-v_h/t, п.5 Stage 4.4) по построению направляет кандидат тяги
// примерно НАВСТРЕЧУ ground-relative скорости — а раз ветра нет, это
// направление и есть примерно ретроград истинного потока, AoA у кандидата
// сам по себе оказывается близко к 180° (безопасно) почти всегда — именно
// это и обнаружено в живых seed-трассировках Stage 4.6 (margin ни разу не
// ушёл в минус). Чтобы честно проверить ветвь урезания, здесь задан СИЛЬНЫЙ
// боковой ветер: AirRelativeVelocity направлен поперёк GroundRelativeVelocity
// — кандидат, гасящий ground-относительный снос, оказывается развёрнут
// почти на 90° от истинного потока, предсказанный AoA/момент велики.
func TestLandingAuthority_InsufficientMarginReducesHorizontal(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)
	nav := nav3D(1500, 50, 500, 0)
	nav.AirRelativeVelocity = nav.Frame.North.Scale(500).Add(nav.Frame.Up.Scale(-50))
	// Триста килопаскалей, а не восемьдесят: располагаемый момент качания
	// вырос вместе с MaxGimbalDegrees (Raptor, ±15° вместо паспортных пяти
	// у Merlin, см. vehicle.Config.MaxGimbalDegrees) — прежние восемьдесят
	// килопаскалей этот запас уже не превышают, сценарий проверял бы ветвь
	// authority-ограничения, которая больше не срабатывает.
	nav.DynamicPressure = 300000

	// Прогрев landingAccelBudgetSmoothed под ЭТОТ синтетический nav (тот
	// же приём, что primeDesiredThrust/primeRequiredThrust, Stage 4.3/4.4):
	// без него сглаженный эталон остаётся от РЕАЛЬНОГО состояния бустера
	// на ignition (другая высота/скорость), несогласованного с синтетикой.
	b.landingAccelBudgetSmoothed = 0
	b.updateLandingAccelBudget(nav, 0)

	plain, _ := b.landingDesiredThrust(nav)
	if horizontalMag(nav, plain) == 0 {
		t.Fatal("тест предполагает, что landingDesiredThrust САМА (без authority-ограничения) уже даёт ненулевую горизонталь")
	}

	constrained, _, limited, margin := b.landingAuthorityConstrainedThrust(nav)

	if !limited {
		t.Fatalf("ожидалось marginLimited=true на заведомо недостаточной authority (сильный боковой ветер, q=80кПа)")
	}
	if math.IsNaN(margin) || math.IsInf(margin, 0) {
		t.Fatalf("margin не конечен: %v", margin)
	}
	if math.IsNaN(constrained.X) || math.IsNaN(constrained.Y) || math.IsNaN(constrained.Z) {
		t.Fatalf("constrained вектор содержит NaN: %+v", constrained)
	}

	plainH := horizontalMag(nav, plain)
	constrainedH := horizontalMag(nav, constrained)
	if constrainedH >= plainH {
		t.Errorf("горизонтальная составляющая не уменьшилась при нехватке authority: было %.0f Н, стало %.0f Н",
			plainH, constrainedH)
	}
}

// TestLandingPredictedAoAAndAeroTorque_AlignedWithFlowIsSafe — кандидат
// направления, СОВПАДАЮЩИЙ с направлением истинного потока (AoA=0°),
// обязан давать нулевой (или пренебрежимо малый) предсказанный момент —
// синус угла атаки равен нулю. Кандидат, ПЕРПЕНДИКУЛЯРНЫЙ потоку (AoA=90°),
// обязан давать заметно больший момент — та же зависимость sin(α)|sin(α)|,
// что и настоящая AerodynamicTorque. Omega зануляется явно: у реального
// снимка booster'а (landingBoosterOnGroup) она может быть ненулевой, а
// AerodynamicTorque ЧЕСТНО добавляет от неё демпфирующий член — здесь
// проверяется именно коэффициентный (от угла атаки) член отдельно.
func TestLandingPredictedAoAAndAeroTorque_AlignedWithFlowIsSafe(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingMid)
	b.attitude.Omega = physics.Vec3{}
	nav := nav3DWithAirflow(1500, 150, 0, 0, 60000)

	flowDir := nav.AirRelativeVelocity.Unit()
	mach := 0.5

	aoaAligned, torqueAligned := b.landingPredictedAoAAndAeroTorque(nav, flowDir, mach)
	if aoaAligned > 1 {
		t.Errorf("кандидат вдоль потока дал AoA=%.2f°, ожидалось ~0°", aoaAligned)
	}
	if torqueAligned > 1e3 {
		t.Errorf("кандидат вдоль потока (AoA≈0) дал заметный момент: %.3e Н·м", torqueAligned)
	}

	perp := nav.Frame.East
	if math.Abs(perp.Dot(flowDir)) > 0.9 {
		perp = nav.Frame.North
	}
	aoaPerp, torquePerp := b.landingPredictedAoAAndAeroTorque(nav, perp, mach)
	if math.Abs(aoaPerp-90) > 5 {
		t.Errorf("кандидат перпендикулярно потоку дал AoA=%.2f°, ожидалось ~90°", aoaPerp)
	}
	if torquePerp <= torqueAligned {
		t.Errorf("перпендикулярный кандидат (AoA≈90°) не дал больший момент, чем выровненный: %.3e против %.3e",
			torquePerp, torqueAligned)
	}
}

// TestLandingAvailableCorrectiveTorque_LargerGroupHasMoreAuthority —
// регрессия совместимая с Stage 4.5 authority map: бóльшая группа камер
// обязана давать не меньший располагаемый корректирующий момент.
func TestLandingAvailableCorrectiveTorque_LargerGroupHasMoreAuthority(t *testing.T) {
	nav := nav3DWithAirflow(1500, 150, 0, 0, 60000)

	b13 := landingBoosterOnGroup(t, 1, boosterLandingHigh)
	b3 := landingBoosterOnGroup(t, 1, boosterLandingLow)

	a13 := b13.landingAvailableCorrectiveTorque(nav, 0.5)
	a3 := b3.landingAvailableCorrectiveTorque(nav, 0.5)

	if a13 <= a3 {
		t.Errorf("группа 13 не дала больше authority, чем группа 3: %.3e против %.3e", a13, a3)
	}
}
