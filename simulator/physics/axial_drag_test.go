package physics

import (
	"math"
	"testing"
)

// -----------------------------------------------------------------------------
// Регрессия на orientation-blind осевое сопротивление: раньше один и тот же
// DragCoefficient(mach) применялся и носом, и двигателями вперёд — здесь
// проверяется, что теперь это разные, физически обоснованные кривые, что
// граница между ними не даёт разрыва силы, и что штатный (носовой) режим
// не изменился ради посадочного фикса.
// -----------------------------------------------------------------------------

// TestOrientationFromAngleOfAttack_Direction — п.5: 0° → NoseFirst,
// 180° → EngineFirst, ровно 90° — граница (EngineFirst не включает её саму,
// см. OrientationFromAngleOfAttack).
func TestOrientationFromAngleOfAttack_Direction(t *testing.T) {
	cases := []struct {
		aoa  float64
		want FlowOrientation
	}{
		{0, NoseFirst},
		{45, NoseFirst},
		{89.9, NoseFirst},
		{90, NoseFirst},
		{90.1, EngineFirst},
		{135, EngineFirst},
		{180, EngineFirst},
	}
	for _, c := range cases {
		if got := OrientationFromAngleOfAttack(c.aoa); got != c.want {
			t.Errorf("aoa=%.1f: got=%v, want=%v", c.aoa, got, c.want)
		}
	}
}

// TestAxialDragCoefficient_DirectionDiffers — п.12 "Direction test": при
// одном и том же Mach nose-first и engine-first обязаны давать разный Cd —
// иначе фикс ничего не изменил.
func TestAxialDragCoefficient_DirectionDiffers(t *testing.T) {
	for _, mach := range []float64{0.3, 0.8, 1.0, 1.2, 2, 3, 4, 6} {
		nose := AxialDragCoefficient(mach, NoseFirst)
		engine := AxialDragCoefficient(mach, EngineFirst)
		if nose == engine {
			t.Errorf("mach=%.1f: nose-first Cd == engine-first Cd == %.3f — ориентация не различается", mach, nose)
		}
		// Тупой торец обязан сопротивляться сильнее обтекаемого носа —
		// иначе выбор кривой физически бессмыслен.
		if engine <= nose {
			t.Errorf("mach=%.1f: engine-first Cd=%.3f не больше nose-first Cd=%.3f", mach, engine, nose)
		}
	}
}

// TestAxialDragCoefficient_NoseFirstUnchanged — regression: старая
// (носовая, штатный подъём) кривая не должна была отличаться от
// DragCoefficient(mach) — Stage 2 не имеет права менять подъём.
func TestAxialDragCoefficient_NoseFirstUnchanged(t *testing.T) {
	for mach := 0.0; mach <= 8; mach += 0.25 {
		want := DragCoefficient(mach)
		got := AxialDragCoefficient(mach, NoseFirst)
		if got != want {
			t.Errorf("mach=%.2f: AxialDragCoefficient(NoseFirst)=%.4f != DragCoefficient=%.4f", mach, got, want)
		}
	}
}

// TestEngineFirstDragCoefficient_NoDiscontinuities — п.10: subsonic →
// transonic → supersonic → hypersonic без скачков (кривая должна быть
// непрерывной и не должна давать резкий пик именно у M=1, который был бы
// неотличим от искусственного бага).
func TestEngineFirstDragCoefficient_NoDiscontinuities(t *testing.T) {
	const step = 0.02
	prev := EngineFirstDragCoefficient(0)
	for mach := step; mach <= 8; mach += step {
		cur := EngineFirstDragCoefficient(mach)
		if math.Abs(cur-prev) > 0.05 {
			t.Errorf("mach=%.2f: скачок Cd %.3f → %.3f (Δ=%.3f) за шаг %.2f по Маху",
				mach, prev, cur, cur-prev, step)
		}
		prev = cur
	}
}

// TestEngineFirstDragCoefficient_StaysWithinPhysicalReferenceRange —
// сверхзвуковой участок (M=2.49…4.63, диапазон измерений Stallings, NASA
// TR R-274) обязан лежать в опубликованном для плоского торца диапазоне
// Cd≈1.7-1.9 — не быть подогнан под желаемую Vign отдельно от этого диапазона.
func TestEngineFirstDragCoefficient_StaysWithinPhysicalReferenceRange(t *testing.T) {
	for _, mach := range []float64{2.5, 3.0, 3.5, 4.0, 4.6} {
		cd := EngineFirstDragCoefficient(mach)
		if cd < 1.55 || cd > 1.95 {
			t.Errorf("mach=%.2f: Cd=%.3f вне диапазона Stallings (примерно 1.55-1.95 "+
				"с учётом интерполяции у нижней границы измерений)", mach, cd)
		}
	}
}

// TestEngineFirstDragCoefficient_HypersonicPlateau — п.9 "M=5+": выше M5
// коэффициент не должен продолжать расти без ограничения (Mach number
// independence principle).
func TestEngineFirstDragCoefficient_HypersonicPlateau(t *testing.T) {
	at5 := EngineFirstDragCoefficient(5)
	at8 := EngineFirstDragCoefficient(8)
	at20 := EngineFirstDragCoefficient(20)
	if at8 != at5 || at20 != at5 {
		t.Errorf("Cd не выходит на плато выше M5: M5=%.3f M8=%.3f M20=%.3f", at5, at8, at20)
	}
}

// TestAxialDragCoefficient_HandlesInvalidMach — не должен давать NaN/паники
// на вырожденном входе (тот же контракт, что и у DragCoefficient).
func TestAxialDragCoefficient_HandlesInvalidMach(t *testing.T) {
	for _, mach := range []float64{-1, math.NaN()} {
		for _, o := range []FlowOrientation{NoseFirst, EngineFirst} {
			got := AxialDragCoefficient(mach, o)
			if math.IsNaN(got) || got <= 0 {
				t.Errorf("mach=%v orientation=%v: Cd=%v — ожидался конечный положительный запасной вариант", mach, o, got)
			}
		}
	}
}
