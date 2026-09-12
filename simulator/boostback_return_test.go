package simulator

import (
	"math"
	"math/rand"
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Регрессия на Stage 4.11 — return-to-launch-site guidance + landing miss
// telemetry.
// -----------------------------------------------------------------------------

func testBoosterConfig(t *testing.T) vehicle.Config {
	t.Helper()
	m, ok := vehicle.MissionByID("transatmospheric")
	if !ok {
		t.Fatal("MissionByID: не найдена миссия transatmospheric")
	}
	p, ok := vehicle.ProfileByID("starship")
	if !ok {
		t.Fatal("ProfileByID: не найден профиль starship")
	}
	return m.Apply(p.Config())
}

// newTestBooster строит Booster напрямую (без прогона Simulation до
// отделения) в заданном ECI-положении/скорости на момент elapsed — быстрый
// unit-level способ проверить launchTarget/landingMiss без минуты реального
// полёта.
func newTestBooster(t *testing.T, position, velocity physics.Vec3, elapsed float64) *Booster {
	t.Helper()
	cfg := testBoosterConfig(t)
	rng := rand.New(rand.NewSource(1))
	sensorRng := rand.New(rand.NewSource(2))
	return NewBooster(cfg, position, velocity, elapsed, rng, sensorRng, 1, 290, physics.Quaternion{W: 1})
}

// TestBoostback_LaunchTargetMatchesConfig — Stage 4.11, п.1,11; Stage 4.12,
// п.1: launch point сохраняется корректно — launchTarget обязан совпадать с
// cfg.LaunchLatitude/LaunchLongitude (те же координаты, из которых
// Simulation строит стартовое положение на t=0, см. simulation.go). Старое
// отдельное поле target (штатная, отдельная точка приводнения Stage
// 4.3-4.6, которую НИКТО фактически не читал) удалено в Stage 4.12,
// п.1 — сравнивать больше не с чем, launchTarget — единственная цель.
func TestBoostback_LaunchTargetMatchesConfig(t *testing.T) {
	cfg := testBoosterConfig(t)
	launch := physics.Geodetic{Latitude: cfg.LaunchLatitude, Longitude: cfg.LaunchLongitude}
	position := physics.GeodeticToECEF(launch)
	velocity := physics.Vec3{X: 100, Y: 100, Z: 100}

	b := newTestBooster(t, position, velocity, 0)

	if b.launchTarget.Latitude != cfg.LaunchLatitude || b.launchTarget.Longitude != cfg.LaunchLongitude {
		t.Errorf("launchTarget=%+v, ожидалось lat=%.4f lon=%.4f",
			b.launchTarget, cfg.LaunchLatitude, cfg.LaunchLongitude)
	}
}

// TestBoostback_ZeroDistanceAtLaunchPoint — Stage 4.11, п.11: горизонтальное
// расстояние до цели обязано быть ~0, когда бустер физически находится в
// точке старта (тот же тест, что подтверждает корректность groundMissAgainst
// на границе, а не только вдалеке).
func TestBoostback_ZeroDistanceAtLaunchPoint(t *testing.T) {
	cfg := testBoosterConfig(t)
	launch := physics.Geodetic{Latitude: cfg.LaunchLatitude, Longitude: cfg.LaunchLongitude}
	position := physics.GeodeticToECEF(launch)
	velocity := physics.Vec3{X: 100, Y: 50, Z: 30}

	b := newTestBooster(t, position, velocity, 0)

	dist, _, _ := b.landingMiss(position, 0)
	if dist > 1.0 {
		t.Errorf("HorizontalDistanceToTarget в точке старта = %.3fм, ожидалось ~0", dist)
	}
}

// TestBoostback_MirroredMissGivesMirroredCorrection — Stage 4.11, п.11:
// зеркальные ошибки положения обязаны давать зеркальную (противоположную)
// требуемую коррекцию — прямая проверка симметрии groundMissAgainst без
// построения Booster: промах "к востоку" и промах "к западу" на ту же
// величину обязаны давать downrange/crossrange, отличающиеся только знаком
// компоненты вдоль соответствующей оси.
func TestBoostback_MirroredMissGivesMirroredCorrection(t *testing.T) {
	target := physics.Geodetic{Latitude: 26.0, Longitude: -97.0}
	targetECEF := physics.GeodeticToECEF(target)
	local := physics.NewLocalFrame(targetECEF)

	// Опорное направление коррекции — "север" в этой точке (простая,
	// детерминированная ось для теста, не завязанная на b.outboundHorizontal).
	refBearing := local.North

	offsetEast := targetECEF.Add(local.East.Scale(5000))  // 5 км к востоку
	offsetWest := targetECEF.Add(local.East.Scale(-5000)) // 5 км к западу

	distE, downE, crossE := groundMissAgainst(offsetEast, 0, target, refBearing, 0)
	distW, downW, crossW := groundMissAgainst(offsetWest, 0, target, refBearing, 0)

	if math.Abs(distE-distW) > 1.0 {
		t.Errorf("зеркальные смещения дали разное расстояние: восток=%.1fм запад=%.1fм", distE, distW)
	}
	if math.Abs(downE-downW) > 1.0 {
		t.Errorf("downrange не должен зависеть от знака ВОСТОК/ЗАПАД смещения при опорной оси СЕВЕР: "+
			"восток=%.1f запад=%.1f", downE, downW)
	}
	if math.Abs(crossE+crossW) > 1.0 {
		t.Errorf("crossrange обязан быть зеркальным (E=-W): восток=%.1f запад=%.1f", crossE, crossW)
	}
	if crossE*crossW >= 0 {
		t.Errorf("crossrange East и West обязаны иметь противоположные знаки: %.1f и %.1f", crossE, crossW)
	}
}

// TestBoostback_PredictedImpactErrorDecreasesWithCorrectiveVelocity —
// Stage 4.11, п.11: правильно направленная (уменьшающая промах) добавка
// к скорости обязана уменьшать прогнозируемый промах — иначе весь смысл
// closed-loop коррекции (landingPredictedImpact + boostbackTarget) неверен.
func TestBoostback_PredictedImpactErrorDecreasesWithCorrectiveVelocity(t *testing.T) {
	cfg := testBoosterConfig(t)

	// Синтетическое состояние: высоко и сбоку от площадки, с горизонтальной
	// скоростью, уводящей ЕЩЁ дальше (типичная геометрия boostback).
	start := physics.Geodetic{Latitude: cfg.LaunchLatitude + 0.3, Longitude: cfg.LaunchLongitude + 0.5, Altitude: 80000}
	position := physics.GeodeticToECEF(start)
	local := physics.NewLocalFrame(position)
	awayVelocity := local.East.Scale(400).Add(local.North.Scale(300)).Add(local.Up.Scale(200))

	b := newTestBooster(t, position, awayVelocity, 0)

	// Наведение работает по ПОКАЗАНИЯМ навигации (см. Booster.sensedNavState),
	// поэтому состояние, чью коррекцию проверяет тест, подаётся так же —
	// через NavState, а не подменой истинного вектора. landingPredictedImpact
	// читает из него только положение и скорость.
	navAtState := func(pos, vel physics.Vec3) orbit.NavState {
		return orbit.NavState{Position: pos, Velocity: vel}
	}

	missBefore, _, _, _, ok := b.landingPredictedImpact(navAtState(position, awayVelocity), 0)
	if !ok {
		t.Fatal("landingPredictedImpact не сошёлся на baseline-состоянии")
	}

	// Коррекция: та же величина скорости, что и в boostbackTarget — против
	// направления промаха (impact-target), горизонтально.
	impactECI, flightTime, ok := predictBallisticImpact(position, awayVelocity)
	if !ok {
		t.Fatal("predictBallisticImpact не сошёлся")
	}
	targetECEF := physics.GeodeticToECEF(b.launchTarget)
	impactECEF := physics.ECIToECEF(impactECI, flightTime)
	localAtCurrent := physics.NewLocalFrame(position)
	me, mn, _ := localAtCurrent.Decompose(impactECEF.Sub(targetECEF))
	n := math.Hypot(me, mn)
	if n < 1 {
		t.Fatal("тестовое состояние уже точно на цели — переподобрать geometry теста")
	}
	correction := localAtCurrent.East.Scale(-me / n).Add(localAtCurrent.North.Scale(-mn / n))

	corrected := awayVelocity.Add(correction.Scale(200)) // 200 м/с коррекции
	missAfter, _, _, _, ok := b.landingPredictedImpact(navAtState(position, corrected), 0)
	if !ok {
		t.Fatal("landingPredictedImpact не сошёлся после коррекции")
	}

	if missAfter >= missBefore {
		t.Errorf("промах не уменьшился от правильно направленной коррекции: было %.0fм, стало %.0fм",
			missBefore, missAfter)
	}
}

// TestBoostback_EarthFixedTargetDoesNotDriftWithTime — Stage 4.11, п.1,11:
// сравнение позиций обязано идти во вращающейся (Earth-fixed) системе —
// расстояние от НЕПОДВИЖНОЙ (относительно поверхности) точки до цели не
// должно "дрейфовать" только из-за того, что t в вызове выросло: booster,
// висящий над ТОЙ ЖЕ геодезической точкой в разные моменты elapsed (то
// есть его ECI-положение поворачивается вместе с Землёй), обязан давать
// одно и то же landingMiss.
func TestBoostback_EarthFixedTargetDoesNotDriftWithTime(t *testing.T) {
	cfg := testBoosterConfig(t)
	launch := physics.Geodetic{Latitude: cfg.LaunchLatitude, Longitude: cfg.LaunchLongitude}
	// Точка в 50 км к северу от площадки, физически неподвижная над
	// поверхностью (ECEF-положение постоянно) — но её ECI-представление в
	// разные elapsed различается ровно на поворот Земли.
	fixedGeo := physics.Geodetic{Latitude: launch.Latitude + 0.45, Longitude: launch.Longitude, Altitude: 10000}
	fixedECEF := physics.GeodeticToECEF(fixedGeo)

	b := newTestBooster(t, physics.ECEFToECI(fixedECEF, 0), physics.Vec3{}, 0)

	const dt = 300.0 // 5 минут вращения Земли — не пренебрежимо (см. EarthAngularVelocity)
	posAtT0 := physics.ECEFToECI(fixedECEF, 0)
	posAtT1 := physics.ECEFToECI(fixedECEF, dt)

	dist0, down0, cross0 := b.landingMiss(posAtT0, 0)
	dist1, down1, cross1 := b.landingMiss(posAtT1, dt)

	const tol = 1.0 // метр — округление формул, не физический дрейф
	if math.Abs(dist0-dist1) > tol {
		t.Errorf("HorizontalDistanceToTarget задрейфовал со временем: t=0 → %.2fм, t=%.0f → %.2fм (Δ=%.2fм)",
			dist0, dt, dist1, dist1-dist0)
	}
	if math.Abs(down0-down1) > tol || math.Abs(cross0-cross1) > tol {
		t.Errorf("downrange/crossrange задрейфовали со временем: t=0 → (%.2f,%.2f), t=%.0f → (%.2f,%.2f)",
			down0, cross0, dt, down1, cross1)
	}

	// Контрольная проверка того, что дрейф вообще возможен ЗАМЕТИТЬ: если
	// бы landingMiss сравнивал ECI-позиции без перевода во вращающуюся
	// систему, посчитанное "расстояние" от posAtT0 (взятого как есть, БЕЗ
	// поправки на elapsed) до цели на момент dt отличалось бы заметно —
	// подтверждаем, что вращение Земли за 5 минут даёт значимый (не
	// пренебрежимый) сдвиг, то есть предыдущая проверка не тривиальна.
	naiveDist, _, _ := b.landingMiss(posAtT0, dt) // намеренно НЕПРАВИЛЬНЫЙ elapsed
	if math.Abs(naiveDist-dist0) < 100 {
		t.Skip("вращение Земли за выбранный dt даёт слишком малый сдвиг, чтобы тест был показательным")
	}
}
