package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Регрессия на Stage 4.12.5 — velocity/frame consistency audit.
//
// Найдено и исправлено этой стадией: (1) checkSplashdown/BoosterTelemetry.
// SplashSpeed считали ТОЛЬКО вертикальную скорость касания, soft/hard
// классификация её же и использовала — приводнение с малым Vz, но большой
// Vh (реально наблюдалось на canonical seed: Vh=86-173 м/с) логировалось бы
// как "мягкое"; (2) BoosterTelemetry.TotalVelocity был сырой ECI-нормой,
// несогласованной с соседними HorizontalVelocity/VerticalVelocity (обе
// ground-relative) — вклад вращения Земли (~420 м/с на широте площадки)
// не пренебрежим против скоростей посадочного участка; (3) outboundHorizontal
// строилась из ECI, а не ground-relative скорости отделения. Все три —
// это НЕ было ошибкой в самой математике разложения (see
// TestVelocityFrames_GroundAndAirInvariantsHold), а путаницей на уровне
// "какую скорость подставили" у конкретных потребителей.
// -----------------------------------------------------------------------------

// TestVelocityFrames_GroundAndAirInvariantsHold — на РЕАЛЬНОМ состоянии
// живого booster'а (не синтетическом векторе) проверяет оба инварианта
// Пифагора независимо: |Vground|²=Vh²+Vz² и |Vair|²=Vh,air²+Vz,air² — без
// переиспользования промежуточных величин друг из друга (каждая
// вычисляется заново, своей формулой).
func TestVelocityFrames_GroundAndAirInvariantsHold(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)

	positionECI := b.state.Position
	velocityECI := b.state.Velocity
	// Ненулевой ветер — иначе air==ground и второй инвариант не проверяет
	// ничего нового по сравнению с первым.
	wind := physics.NewLocalFrame(positionECI).East.Scale(37).Add(physics.NewLocalFrame(positionECI).North.Scale(-19))

	frame := physics.NewLocalFrame(positionECI)

	vGround := velocityECI.Sub(physics.CorotatingVelocity(positionECI))
	vAir := vGround.Sub(wind)

	ge, gn, gu := frame.Decompose(vGround)
	horizGround := math.Hypot(ge, gn)
	totalGround := vGround.Norm()

	ae, an, au := frame.Decompose(vAir)
	horizAir := math.Hypot(ae, an)
	totalAir := vAir.Norm()

	const tol = 1e-6
	if d := math.Abs(totalGround*totalGround - (horizGround*horizGround + gu*gu)); d > tol {
		t.Errorf("ground invariant нарушен: |Vground|²=%.9f, h²+z²=%.9f, Δ=%.3e",
			totalGround*totalGround, horizGround*horizGround+gu*gu, d)
	}
	if d := math.Abs(totalAir*totalAir - (horizAir*horizAir + au*au)); d > tol {
		t.Errorf("air invariant нарушен: |Vair|²=%.9f, h²+z²=%.9f, Δ=%.3e",
			totalAir*totalAir, horizAir*horizAir+au*au, d)
	}
	// Ветер в этой модели всегда чисто горизонтален (env.horizontalWind) —
	// вертикальная (Up) компонента ground и air обязана СОВПАДАТЬ.
	if d := math.Abs(gu - au); d > tol {
		t.Errorf("Up-компонента ground/air разошлась при чисто горизонтальном ветре: %.6f vs %.6f (Δ=%.3e)", gu, au, d)
	}
}

// TestSplashdown_TotalSpeedCombinesHorizontalAndVertical — синтетическая
// проверка: приводнение с БОЛЬШОЙ горизонтальной, но малой вертикальной
// скоростью обязано классифицироваться как жёсткое (не "мягкое" только
// потому, что вертикаль мала) — прямая регрессия на баг, найденный этой
// стадией в checkSplashdown/softLimit.
func TestSplashdown_TotalSpeedCombinesHorizontalAndVertical(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingLow)
	b.phase = BoosterLandingBurn

	// Продавливаем высоту чуть ниже поверхности (не ровно EarthRadius —
	// Unit() не гарантирует норму РОВНО 1.0 в floating point, а
	// checkSplashdown сравнивает Altitude() c нулём строго, без допуска)
	// ПЕРЕД тем, как строить up/frame/CorotatingVelocity — не наоборот:
	// CorotatingVelocity (ω×r) масштабируется с самим радиусом, и
	// вычисление её на ИСХОДНОЙ позиции booster'а (где бы та ни была —
	// высота ignition зависит от текущей физики Boostback/Coast, а не
	// зафиксирована этим тестом) с последующим погружением на
	// EarthRadius-1 рассинхронизировало бы её с тем, что checkSplashdown
	// пересчитает уже на НОВОЙ позиции, — на десятки километров разницы
	// высоты (typical ignition altitude) расхождение выходит за допуск
	// теста, хотя сама проверяемая физика (Пифагор горизонт/вертикаль)
	// ни при чём.
	b.state.Position = b.state.Position.Unit().Scale(physics.EarthRadius - 1.0)
	up := b.state.Position.Unit()
	frame := physics.NewLocalFrame(b.state.Position)

	// Вертикаль ниже softLimit (5 м/с) сама по себе — "мягко" по старому
	// (ошибочному) критерию — но горизонталь огромна.
	const vz = -3.0  // м/с, к поверхности
	const vh = 120.0 // м/с, горизонтально — далеко за пределом "мягкой"

	groundVelocity := up.Scale(vz).Add(frame.East.Scale(vh))
	b.state.Velocity = groundVelocity.Add(physics.CorotatingVelocity(b.state.Position))

	b.checkSplashdown()

	if b.phase != BoosterSplashdown {
		t.Fatalf("checkSplashdown не перевёл фазу: %v", b.phase)
	}
	if math.Abs(b.splashSpeed-3.0) > 0.1 {
		t.Errorf("splashSpeed (вертикаль)=%.2f, ожидалось ~3.0", b.splashSpeed)
	}
	if math.Abs(b.splashHorizontalSpeed-120.0) > 0.5 {
		t.Errorf("splashHorizontalSpeed=%.2f, ожидалось ~120.0", b.splashHorizontalSpeed)
	}
	wantTotal := math.Hypot(3.0, 120.0)
	if math.Abs(b.splashTotalSpeed-wantTotal) > 0.5 {
		t.Errorf("splashTotalSpeed=%.2f, ожидалось ~%.2f (√(верт²+гор²))", b.splashTotalSpeed, wantTotal)
	}
	// Главная регрессия: малая ВЕРТИКАЛЬ (3 м/с < softLimit=5) не должна
	// сама по себе означать "мягкое" приводнение при большой горизонтали.
	const softLimit = 5.0
	if b.splashTotalSpeed <= softLimit {
		t.Errorf("полная скорость касания %.1f м/с ошибочно в пределах softLimit=%.1f — "+
			"горизонтальная составляющая (%.1f м/с) не должна теряться", b.splashTotalSpeed, softLimit, vh)
	}
}
