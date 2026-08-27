package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Регрессия на SurfaceSet.Allocate() решётчатых рулей бустера (allocateOptimal).
//
// Общее состояние для всех тестов: q и Mach в диапазоне, где рули реально
// работают на Coast/входе, поток почти вдоль оси корпуса (двигателями вперёд)
// с небольшим поперечным составляющим — то же, что видит реальный полёт.
// -----------------------------------------------------------------------------

func testGridFinSet() (*SurfaceSet, physics.Vec3, float64, float64, float64) {
	length, radius := 72.3, 4.5
	com := 0.539
	comFromNose := com * length
	surf := NewSurfaceSet(vehicle.GridFins(length, radius))
	surf.Deployed = true
	airBody := physics.Vec3{X: -1000, Y: 30, Z: 0}
	const q, mach = 50000.0, 3.5
	return surf, airBody, q, mach, comFromNose
}

func achievedTorque(surf *SurfaceSet, mcmd, airBody physics.Vec3, q, mach, comFromNose float64) physics.Vec3 {
	surf.Allocate(mcmd, airBody, q, mach, comFromNose)
	copy(surf.Deflection, surf.Command) // мгновенно для чтения достигнутого момента
	return surf.UpdateTorque(airBody, q, mach, comFromNose)
}

// TestAllocateOptimal_SignPitchYaw — п.7 "Sign tests": +pitch/-pitch и
// +yaw/-yaw должны давать момент правильного знака по своей оси.
func TestAllocateOptimal_SignPitchYaw(t *testing.T) {
	const m = 2e6
	cases := []struct {
		name string
		cmd  physics.Vec3
		axis func(physics.Vec3) float64
		want float64 // знак ожидаемого момента
	}{
		{"+pitch", physics.Vec3{Y: m}, func(v physics.Vec3) float64 { return v.Y }, +1},
		{"-pitch", physics.Vec3{Y: -m}, func(v physics.Vec3) float64 { return v.Y }, -1},
		{"+yaw", physics.Vec3{Z: m}, func(v physics.Vec3) float64 { return v.Z }, +1},
		{"-yaw", physics.Vec3{Z: -m}, func(v physics.Vec3) float64 { return v.Z }, -1},
	}
	for _, c := range cases {
		surf, airBody, q, mach, comFromNose := testGridFinSet()
		got := achievedTorque(surf, c.cmd, airBody, q, mach, comFromNose)
		val := c.axis(got)
		if val*c.want <= 0 {
			t.Errorf("%s: achieved=%v, ожидался знак %+.0f по своей оси, получили %.4e", c.name, got, c.want, val)
		}
	}
}

// TestAllocateOptimal_RollIsSymmetric — РЕГРЕССИЯ, заменившая прежний
// TestAllocateOptimal_RollIsOneSided. Раньше рули могли отклоняться только
// в [0, MaxDeflection], и крен был физически односторонним: все рули кольца
// дают одинаковый по знаку вклад (surfaceWeights: wRoll=1), а отрицательное
// отклонение было запрещено самой моделью актуатора, а не механикой руля —
// см. physics.ControlSurface.MinDeflection и историю в комментарии у
// vehicle.GridFins. С исправленным диапазоном [-MaxDeflection,MaxDeflection]
// у решётчатых рулей крен должен стать симметричным: +roll и -roll обязаны
// достигать сопоставимых по модулю значений СВОЕГО знака.
func TestAllocateOptimal_RollIsSymmetric(t *testing.T) {
	const m = 5e6

	surfPos, airBody, q, mach, comFromNose := testGridFinSet()
	gotPos := achievedTorque(surfPos, physics.Vec3{X: m}, airBody, q, mach, comFromNose)
	if gotPos.X <= 0 {
		t.Errorf("+roll: achieved.X=%.4e, ожидался положительный", gotPos.X)
	}

	surfNeg, _, _, _, _ := testGridFinSet()
	gotNeg := achievedTorque(surfNeg, physics.Vec3{X: -m}, airBody, q, mach, comFromNose)
	if math.IsNaN(gotNeg.X) {
		t.Fatalf("-roll: achieved.X = NaN")
	}
	if gotNeg.X >= 0 {
		t.Errorf("-roll: achieved.X=%.4e — ожидался отрицательный (симметричный диапазон должен давать оба знака)", gotNeg.X)
	}
	const tolerance = 0.30 // 30% — геометрия T-раскладки не обязана быть идеально зеркальной
	if math.Abs(math.Abs(gotNeg.X)-gotPos.X) > gotPos.X*tolerance {
		t.Errorf("-roll/+roll не симметричны в пределах %.0f%%: +roll=%.4e -roll=%.4e", tolerance*100, gotPos.X, gotNeg.X)
	}
}

// TestAllocateOptimal_YawIsSymmetric — геометрия кольца через 120° сменилась
// на T-образную раскладку Super Heavy V3 (fin_1/fin_3 друг напротив друга —
// "порт"/"старборд", fin_2 перпендикулярно им, на стороне без башни — см.
// vehicle.gridFinAzimuths). Старая проверка M(+cmd)≈-M(-cmd) для pitch И yaw
// разом опиралась на 3-кратную поворотную симметрию 120°-кольца, которой
// у T-раскладки нет. Замерено заново (см. TestAllocateOptimal_PitchIsOneSided
// рядом) — из двух осей симметричной по факту осталась только yaw:
// единственный офсетный по Y руль (fin_2) даёт обеим полярностям yaw
// сопоставимую реализуемость, поскольку не зависит от противофазного
// "открытия" пары fin_1/fin_3, которая для pitch и оказывается односторонней.
//
// Поток строго осевой — без бокового Y-составляющего фикстура
// testGridFinSet(), чтобы не путать боковой снос потока с геометрией рулей.
func TestAllocateOptimal_YawIsSymmetric(t *testing.T) {
	const m = 1.5e6
	airBody := physics.Vec3{X: -1000}
	_, _, q, mach, comFromNose := testGridFinSet()

	surfP, _, _, _, _ := testGridFinSet()
	plus := achievedTorque(surfP, physics.Vec3{Z: m}, airBody, q, mach, comFromNose).Z
	surfN, _, _, _, _ := testGridFinSet()
	minus := achievedTorque(surfN, physics.Vec3{Z: -m}, airBody, q, mach, comFromNose).Z

	if math.Abs(plus+minus) > math.Abs(plus)*0.20 {
		t.Errorf("yaw: M(+cmd)=%.4e, M(-cmd)=%.4e — не симметричны в пределах 20%%", plus, minus)
	}
}

// TestAllocateOptimal_PitchIsSymmetric — РЕГРЕССИЯ, заменившая прежний
// TestAllocateOptimal_PitchIsOneSided. Раньше pitch у T-раскладки был почти
// так же односторонним, как и roll: +pitch на команду 1.5e6 достигал
// ≈1.48e6, а -pitch — лишь ≈-1e2 (практически ноль). Причина оказалась НЕ
// геометрической, а в модели актуатора: pitch-момент делают противоположные
// fin_1 (белый борт, arm.Z=+r) и fin_3 (спина, arm.Z=−r), и при ОДНОСТОРОННЕМ
// диапазоне [0,MaxDeflection] fin_3 физически не мог дотянуться до угла,
// который дал бы противоположный fin_1 вклад, — то, что выглядело как
// "физика геометрии", было отражением этого искусственного ограничения (см.
// physics.ControlSurface.MinDeflection и историю в комментарии у
// vehicle.GridFins). С симметричным диапазоном [-MaxDeflection,MaxDeflection]
// fin_3 может уйти в отрицательный угол и разблокировать противоположный
// pitch — тест проверяет именно это.
func TestAllocateOptimal_PitchIsSymmetric(t *testing.T) {
	const m = 1.5e6
	airBody := physics.Vec3{X: -1000}
	_, _, q, mach, comFromNose := testGridFinSet()

	surfPos, _, _, _, _ := testGridFinSet()
	gotPos := achievedTorque(surfPos, physics.Vec3{Y: m}, airBody, q, mach, comFromNose)
	if gotPos.Y <= 0 {
		t.Errorf("+pitch: achieved.Y=%.4e, ожидался положительный", gotPos.Y)
	}

	surfNeg, _, _, _, _ := testGridFinSet()
	gotNeg := achievedTorque(surfNeg, physics.Vec3{Y: -m}, airBody, q, mach, comFromNose)
	if math.IsNaN(gotNeg.Y) {
		t.Fatalf("-pitch: achieved.Y = NaN")
	}
	if gotNeg.Y >= 0 {
		t.Errorf("-pitch: achieved.Y=%.4e — ожидался отрицательный (симметричный диапазон должен давать оба знака)", gotNeg.Y)
	}
	const tolerance = 0.30
	if math.Abs(math.Abs(gotNeg.Y)-gotPos.Y) > gotPos.Y*tolerance {
		t.Errorf("-pitch/+pitch не симметричны в пределах %.0f%%: +pitch=%.4e -pitch=%.4e", tolerance*100, gotPos.Y, gotNeg.Y)
	}
}

// TestAllocateOptimal_MixedAxisDoesNotSacrificeComponents — при запросе
// нескольких осей сразу allocator не должен полностью жертвовать одной ради
// другой: достигнутая величина по каждой запрошенной оси должна быть
// сопоставима с тем, что достижимо в одиночной команде той же величины
// (не обязательно равна — geometry ограничивает одновременную реализацию
// всех трёх, — но не должна проваливаться в шум).
func TestAllocateOptimal_MixedAxisDoesNotSacrificeComponents(t *testing.T) {
	const m = 1e6

	surfPitchOnly, airBody, q, mach, comFromNose := testGridFinSet()
	pitchAlone := achievedTorque(surfPitchOnly, physics.Vec3{Y: m}, airBody, q, mach, comFromNose).Y

	surfYawOnly, _, _, _, _ := testGridFinSet()
	yawAlone := achievedTorque(surfYawOnly, physics.Vec3{Z: m}, airBody, q, mach, comFromNose).Z

	surfMixed, _, _, _, _ := testGridFinSet()
	mixed := achievedTorque(surfMixed, physics.Vec3{Y: m, Z: m}, airBody, q, mach, comFromNose)

	const minFraction = 0.25 // геометрия не обязана давать 100%, но не должна давать ~0
	if math.Abs(mixed.Y) < math.Abs(pitchAlone)*minFraction {
		t.Errorf("pitch+yaw: achieved.Y=%.4e — pitch принесён в жертву (одиночный pitch давал %.4e)", mixed.Y, pitchAlone)
	}
	if math.Abs(mixed.Z) < math.Abs(yawAlone)*minFraction {
		t.Errorf("pitch+yaw: achieved.Z=%.4e — yaw принесён в жертву (одиночный yaw давал %.4e)", mixed.Z, yawAlone)
	}
}

// TestAllocateOptimal_SaturationIsWellBehaved — п.7 "Saturation tests":
// запрос заведомо недостижимого момента не должен давать NaN, не должен
// выходить за физические пределы углов, не должен менять знак произвольно
// (achieved обязан быть тем же знаком, что и cmd, просто меньше по модулю).
func TestAllocateOptimal_SaturationIsWellBehaved(t *testing.T) {
	const huge = 1e12 // заведомо недостижимо ни при каком угле
	surf, airBody, q, mach, comFromNose := testGridFinSet()
	got := achievedTorque(surf, physics.Vec3{Y: huge}, airBody, q, mach, comFromNose)

	if math.IsNaN(got.X) || math.IsNaN(got.Y) || math.IsNaN(got.Z) {
		t.Fatalf("achieved содержит NaN: %v", got)
	}
	for i, surface := range surf.Surfaces {
		if surf.Command[i] < surface.MinDeflection-1e-9 || surf.Command[i] > surface.MaxDeflection+1e-9 {
			t.Errorf("surface %s: Command=%.4f вне [%.4f,%.4f]", surface.Name, surf.Command[i],
				surface.MinDeflection, surface.MaxDeflection)
		}
	}
	if got.Y <= 0 {
		t.Errorf("achieved.Y=%.4e сменил знак относительно запроса (+huge)", got.Y)
	}
	if got.Y >= huge {
		t.Errorf("achieved.Y=%.4e >= запроса %.4e — насыщение не сработало", got.Y, huge)
	}
}

// TestAllocateOptimal_ZeroCommandStaysNearCurrent — п.7 "Zero command":
// при Mcmd≈0 allocator не должен гонять рули между большими углами без
// нужды. Проверяется на состоянии, где текущее положение УЖЕ даёт близкий
// к нулю момент (осевой поток, все рули на нуле — по трёхкратной симметрии
// кольца момент в этой точке действительно нулевой), — то есть Mcmd=0
// совпадает с реальным равновесием, и заново решать задачу, уводя рули
// куда-то ещё, незачем. Это не то же самое, что "рули обязаны игнорировать
// Mcmd=0, если текущее положение создаёт лишний момент" — в такой ситуации
// tracking (приоритет №1) обязан их сдвинуть, и это не баг.
func TestAllocateOptimal_ZeroCommandStaysNearCurrent(t *testing.T) {
	surf, _, q, mach, comFromNose := testGridFinSet()
	airBody := physics.Vec3{X: -1000} // чисто осевой поток — на нуле момент действительно нулевой

	surf.Allocate(physics.Vec3{}, airBody, q, mach, comFromNose)

	for i, surface := range surf.Surfaces {
		got := surf.Command[i] * physics.RadToDeg
		if math.Abs(got) > 1 {
			t.Errorf("surface %s: Command=%.2f°, ожидалось ~0° (уже равновесие) при Mcmd=0", surface.Name, got)
		}
	}
}

// TestAllocateLegacy_UnaffectedByGridFinFix — ShipFlaps (без PositionFromNose)
// обязаны и дальше идти через allocateLegacy без изменений — прямая проверка
// диспетчеризации hasDynamicLever.
func TestAllocateLegacy_UnaffectedByGridFinFix(t *testing.T) {
	surf := NewSurfaceSet(vehicle.ShipFlaps(50, 4.5))
	surf.Deployed = true
	if surf.hasDynamicLever() {
		t.Fatal("ShipFlaps не должны иметь PositionFromNose — диспетчеризация сломана")
	}
}

// -----------------------------------------------------------------------------
// Регрессия на дефект allocateOptimal с T-раскладкой: при совместном
// pitch+yaw(+roll) запросе solver сходился к решению, почти идеально
// закрывающему pitch, но starvation/sign-flip делало yaw — хотя yaw в
// одиночку физически достижим на многие МН·м. Состояние для этих тестов —
// то самое проблемное (seed=1, t≈306.5с трассы), не синтетика "из воздуха".
// -----------------------------------------------------------------------------

// traceProblemState — состояние и запрос, на которых был найден и
// воспроизведён дефект (см. предыдущий разбор в чате): q/Mach на высоте
// ~6.5 км, почти осевой поток, mcmd — реальный RequestedTorque из живой
// трассы, где yaw схлопывался в неверный знак.
func traceProblemState() (length, radius, comFromNose, q, mach float64, airBody, mcmd physics.Vec3) {
	length, radius = 72.3, 4.5
	comFromNose = 0.539 * length
	q, mach = 180000.0, 2.7
	airBody = physics.Vec3{X: -750}
	mcmd = physics.Vec3{X: -3.443e6, Y: 1.377e7, Z: 1.644e7}
	return
}

// TestAllocateOptimal_YawAloneStaysCorrectSign — базовая проверка: чистый
// yaw без pitch/roll обязан достигаться с правильным знаком и на
// существенную долю физически достижимого (уже не 0, как в дефекте).
func TestAllocateOptimal_YawAloneStaysCorrectSign(t *testing.T) {
	length, radius, comFromNose, q, mach, airBody, mcmd := traceProblemState()
	surf := NewSurfaceSet(vehicle.GridFins(length, radius))
	surf.Deployed = true

	got := achievedTorque(surf, physics.Vec3{Z: mcmd.Z}, airBody, q, mach, comFromNose)
	if got.Z <= 0 {
		t.Fatalf("yaw-alone: achieved.Z=%.4e — неверный знак (запрошен положительный)", got.Z)
	}
	if got.Z < mcmd.Z*0.5 {
		t.Errorf("yaw-alone: achieved.Z=%.4e — меньше половины запрошенного %.4e при отсутствии конкурирующих осей",
			got.Z, mcmd.Z)
	}
}

// TestAllocateOptimal_PitchDoesNotFlipYawSign — п. "pitch+yaw не должен
// менять знак yaw, если существует feasible solution с correct-sign yaw":
// регрессия непосредственно на найденный дефект. С добавлением pitch к
// тому же yaw-запросу знак достигнутого yaw обязан остаться прежним
// (положительным) — раньше он схлопывался в отрицательный (~-2.2e5 при
// запрошенных +1.644e7).
func TestAllocateOptimal_PitchDoesNotFlipYawSign(t *testing.T) {
	length, radius, comFromNose, q, mach, airBody, mcmd := traceProblemState()
	surf := NewSurfaceSet(vehicle.GridFins(length, radius))
	surf.Deployed = true

	got := achievedTorque(surf, physics.Vec3{Y: mcmd.Y, Z: mcmd.Z}, airBody, q, mach, comFromNose)
	if got.Z <= 0 {
		t.Errorf("pitch+yaw: achieved.Z=%.4e сменил знак относительно запроса (+%.4e)", got.Z, mcmd.Z)
	}
}

// TestAllocateOptimal_MixedAxisIsParetoReasonable — п. "добавление pitch не
// должно обрушать yaw с 8.27 MN·m до ~0 без физической необходимости":
// полный (roll+pitch+yaw) запрос — самый жёсткий случай найденного дефекта
// (roll физически недостижим в свою сторону) — обязан всё равно оставить
// yaw на существенной доле одиночно достижимого, а не близко к нулю.
// Порог (30% от одиночного достижимого) — не "идеальный" Парето-баланс
// (обе оси конкурируют за одни и те же рули, полного совмещения не будет),
// а граница, отделяющая найденный дефект (~0-1%) от разумного компромисса.
func TestAllocateOptimal_MixedAxisIsParetoReasonable(t *testing.T) {
	length, radius, comFromNose, q, mach, airBody, mcmd := traceProblemState()

	surfYawAlone := NewSurfaceSet(vehicle.GridFins(length, radius))
	surfYawAlone.Deployed = true
	yawAlone := achievedTorque(surfYawAlone, physics.Vec3{Z: mcmd.Z}, airBody, q, mach, comFromNose).Z

	surfMixed := NewSurfaceSet(vehicle.GridFins(length, radius))
	surfMixed.Deployed = true
	mixed := achievedTorque(surfMixed, mcmd, airBody, q, mach, comFromNose)

	const minFraction = 0.3
	if mixed.Z < yawAlone*minFraction {
		t.Errorf("mixed (roll+pitch+yaw): achieved.Z=%.4e — меньше %.0f%% одиночно достижимого yaw=%.4e "+
			"(дефект: было ~0)", mixed.Z, minFraction*100, yawAlone)
	}
	// pitch тоже не должен быть принесён в жертву совсем — solver обязан
	// найти совместное решение, а не откатиться к yaw-only.
	if mixed.Y < mcmd.Y*0.5 {
		t.Errorf("mixed: achieved.Y=%.4e — меньше половины запрошенного pitch=%.4e", mixed.Y, mcmd.Y)
	}
}

// TestAllocateOptimal_StartRobustness — permutation/start-state robustness:
// результат раскладки не должен существенно зависеть от того, с какого
// Deflection (по сути — с какого предыдущего такта) стартует solver.
// Дефект проявлялся именно как чувствительность к старту (см. разбор в
// чате: current/zero/mid/max сходились в одно и то же ПЛОХОЕ решение) —
// здесь проверяется обратное: РАЗНЫЕ старты сходятся к СОПОСТАВИМО ХОРОШИМ
// решениям (не обязательно идентичным, но с невязкой одного порядка).
func TestAllocateOptimal_StartRobustness(t *testing.T) {
	length, radius, comFromNose, q, mach, airBody, mcmd := traceProblemState()

	startDeflections := [][]float64{
		{0, 0, 0},
		{20 * physics.DegToRad, 20 * physics.DegToRad, 20 * physics.DegToRad},
		{20 * physics.DegToRad, 0, 0},
		{0, 20 * physics.DegToRad, 0},
		{0, 0, 20 * physics.DegToRad},
		{10 * physics.DegToRad, 5 * physics.DegToRad, 15 * physics.DegToRad},
	}

	var losses []float64
	for _, d0 := range startDeflections {
		surf := NewSurfaceSet(vehicle.GridFins(length, radius))
		surf.Deployed = true
		copy(surf.Deflection, d0)
		surf.Allocate(mcmd, airBody, q, mach, comFromNose)
		copy(surf.Deflection, surf.Command)
		got := surf.UpdateTorque(airBody, q, mach, comFromNose)
		d := mcmd.Sub(got)
		losses = append(losses, d.Dot(d))
	}

	minLoss, maxLoss := losses[0], losses[0]
	for _, l := range losses {
		if l < minLoss {
			minLoss = l
		}
		if l > maxLoss {
			maxLoss = l
		}
	}
	// Разброс между лучшим и худшим результатом по разным стартам не
	// должен превышать разумный множитель — раньше он был огромным
	// (плохое решение при любом старте против найденной вручную комбинации
	// в разы точнее).
	const maxSpreadFactor = 3.0
	if maxLoss > minLoss*maxSpreadFactor {
		t.Errorf("разброс невязки по стартам слишком велик: min=%.4e max=%.4e (>%.1fx)",
			minLoss, maxLoss, maxSpreadFactor)
	}
}

// TestAllocateOptimal_NoNaNUnderExtremeCommand — no NaN / saturation safety:
// заведомо недостижимая команда по всем трём осям разом не должна давать
// NaN/Inf ни в Command, ни в достигнутом моменте, и углы обязаны остаться
// в физических пределах.
func TestAllocateOptimal_NoNaNUnderExtremeCommand(t *testing.T) {
	length, radius, comFromNose, q, mach, airBody, _ := traceProblemState()
	surf := NewSurfaceSet(vehicle.GridFins(length, radius))
	surf.Deployed = true

	const huge = 1e15
	got := achievedTorque(surf, physics.Vec3{X: -huge, Y: huge, Z: -huge}, airBody, q, mach, comFromNose)
	if math.IsNaN(got.X) || math.IsNaN(got.Y) || math.IsNaN(got.Z) ||
		math.IsInf(got.X, 0) || math.IsInf(got.Y, 0) || math.IsInf(got.Z, 0) {
		t.Fatalf("achieved содержит NaN/Inf: %v", got)
	}
	for i, surface := range surf.Surfaces {
		if surf.Command[i] < surface.MinDeflection-1e-9 || surf.Command[i] > surface.MaxDeflection+1e-9 {
			t.Errorf("surface %s: Command=%.4f вне [%.4f,%.4f]", surface.Name, surf.Command[i],
				surface.MinDeflection, surface.MaxDeflection)
		}
	}
}

// TestAllocateOptimal_OneSidedAxesRemainPhysicalConstraint была снята: её
// посылка (-pitch/-roll физически недостижимы у решётчатых рулей) оказалась
// не подтверждённым физическим фактом, а следствием одностороннего
// [0,MaxDeflection] в модели актуатора — см. историю у
// vehicle.GridFins/physics.ControlSurface.MinDeflection и новые регрессии
// в surfaces_deflection_bounds_test.go (TestAllocateOptimal_RollIsSymmetric
// и TestAllocateOptimal_PitchIsSymmetric в этом файле проверяют
// восстановленную симметрию напрямую).
