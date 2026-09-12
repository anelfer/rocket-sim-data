package socp

import (
	"math"
	"testing"
)

// Проверки решателя строятся на задачах с ИЗВЕСТНЫМ аналитическим ответом:
// сверять численный оптимизатор с самим собой бессмысленно, а «сошлось —
// значит верно» скрывает ровно тот класс ошибок (перепутанный знак в
// двойственной части, неверное масштабирование конуса), из-за которых он
// сходится к неправильной точке уверенно и быстро.

func dense(rows, cols int, vals ...float64) *Sparse {
	b := NewSparseBuilder(rows, cols)
	for i := 0; i < rows; i++ {
		for j := 0; j < cols; j++ {
			b.Add(i, j, vals[i*cols+j])
		}
	}
	return b.Build()
}

// Линейная программа: min −x−y при x+y ≤ 1, x ≥ 0, y ≥ 0.
// Оптимум — весь отрезок x+y=1, значение −1.
func TestSolve_LinearProgram(t *testing.T) {
	p := &Problem{
		C: []float64{-1, -1},
		G: dense(3, 2,
			1, 1,
			-1, 0,
			0, -1),
		H: []float64{1, 0, 0},
		K: Cone{Linear: 3},
	}
	s := Solve(p, DefaultOptions())
	if s.Status != StatusOptimal {
		t.Fatalf("статус %v (итераций %d, зазор %g)", s.Status, s.Iterations, s.Gap)
	}
	if math.Abs(s.Objective+1) > 1e-6 {
		t.Fatalf("значение %g, ожидалось −1", s.Objective)
	}
	if x := s.X[0] + s.X[1]; math.Abs(x-1) > 1e-6 {
		t.Fatalf("x+y = %g, ожидалось 1", x)
	}
}

// Проекция на конус: min t при ‖(x−3, y−4)‖ ≤ t.
// Ответ очевиден: t = 0 при x=3, y=4 — но задача записана так, что решателю
// приходится работать именно с конусом второго порядка, а не с линейной частью.
func TestSolve_SecondOrderConeProjection(t *testing.T) {
	// Переменные: (x, y, t). Конус (t, x−3, y−4) ∈ Q³ означает
	// G·v + s = h при s ∈ Q³, где s = (t, x−3, y−4).
	// s = h − G·v ⇒ G = [[0,0,−1],[−1,0,0],[0,−1,0]], h = (0,−3,−4).
	p := &Problem{
		C: []float64{0, 0, 1},
		G: dense(3, 3,
			0, 0, -1,
			-1, 0, 0,
			0, -1, 0),
		H: []float64{0, -3, -4},
		K: Cone{SOC: []int{3}},
	}
	s := Solve(p, DefaultOptions())
	if s.Status != StatusOptimal {
		t.Fatalf("статус %v", s.Status)
	}
	if math.Abs(s.X[0]-3) > 1e-5 || math.Abs(s.X[1]-4) > 1e-5 || math.Abs(s.X[2]) > 1e-5 {
		t.Fatalf("решение (%g, %g, %g), ожидалось (3, 4, 0)", s.X[0], s.X[1], s.X[2])
	}
}

// Наименьшая по норме точка на прямой x+y=2: минимизировать t при
// ‖(x,y)‖ ≤ t и x+y = 2. Ответ — x=y=1, t=√2.
func TestSolve_MinNormWithEquality(t *testing.T) {
	p := &Problem{
		C: []float64{0, 0, 1},
		A: dense(1, 3, 1, 1, 0),
		B: []float64{2},
		G: dense(3, 3,
			0, 0, -1,
			-1, 0, 0,
			0, -1, 0),
		H: []float64{0, 0, 0},
		K: Cone{SOC: []int{3}},
	}
	s := Solve(p, DefaultOptions())
	if s.Status != StatusOptimal {
		t.Fatalf("статус %v", s.Status)
	}
	if math.Abs(s.X[0]-1) > 1e-5 || math.Abs(s.X[1]-1) > 1e-5 {
		t.Fatalf("решение (%g, %g), ожидалось (1, 1)", s.X[0], s.X[1])
	}
	if math.Abs(s.X[2]-math.Sqrt2) > 1e-5 {
		t.Fatalf("норма %g, ожидалось √2", s.X[2])
	}
}

// Смешанный конус: линейная часть и конус второго порядка в одной задаче.
// min −x при ‖(x,y)‖ ≤ 1 и y ≥ 0.5. Ответ: x = √(1−0.25) = √0.75.
func TestSolve_MixedCone(t *testing.T) {
	// Переменные (x,y). Строки: линейная y ≥ 0.5 ⇒ −y + s = −0.5, s ≥ 0;
	// конус (1, x, y): s = (1, x, y) ⇒ G = [[0,0],[−1,0],[0,−1]], h = (1,0,0).
	b := NewSparseBuilder(4, 2)
	b.Add(0, 1, -1)
	b.Add(2, 0, -1)
	b.Add(3, 1, -1)
	p := &Problem{
		C: []float64{-1, 0},
		G: b.Build(),
		H: []float64{-0.5, 1, 0, 0},
		K: Cone{Linear: 1, SOC: []int{3}},
	}
	s := Solve(p, DefaultOptions())
	if s.Status != StatusOptimal {
		t.Fatalf("статус %v", s.Status)
	}
	if want := math.Sqrt(0.75); math.Abs(s.X[0]-want) > 1e-5 {
		t.Fatalf("x = %g, ожидалось %g", s.X[0], want)
	}
	if math.Abs(s.X[1]-0.5) > 1e-5 {
		t.Fatalf("y = %g, ожидалось 0.5", s.X[1])
	}
}

// Заведомо неразрешимая задача: x ≥ 1 и x ≤ 0 одновременно. Решатель обязан
// сообщить об этом статусом, а не вернуть какую-нибудь точку молча.
func TestSolve_InfeasibleIsReported(t *testing.T) {
	p := &Problem{
		C: []float64{1},
		G: dense(2, 1, -1, 1),
		H: []float64{-1, 0},
		K: Cone{Linear: 2},
	}
	s := Solve(p, DefaultOptions())
	if s.Status == StatusOptimal {
		t.Fatalf("неразрешимая задача решена как оптимальная: x=%v", s.X)
	}
}

// Масштабирование Нестерова—Тодда обязано удовлетворять определяющему
// тождеству W·z = W⁻¹·s. Проверка прямая, на случайных, но фиксированных
// (детерминированных) точках внутри конуса.
func TestScaling_NesterovToddIdentity(t *testing.T) {
	k := Cone{Linear: 2, SOC: []int{4, 3}}
	s := []float64{1.5, 0.25, 3.0, 1.0, 0.5, -0.5, 2.0, 1.0, 0.3}
	z := []float64{0.5, 2.00, 5.0, -1.0, 2.0, 1.0, 4.0, -1.0, 2.0}

	sc := k.prepare()
	if !k.update(s, z, &sc) {
		t.Fatal("масштабирование не построилось на заведомо внутренних точках")
	}
	wz := make([]float64, len(z))
	wis := make([]float64, len(s))
	k.apply(&sc, z, wz)
	k.applyInv(&sc, s, wis)
	for i := range wz {
		if math.Abs(wz[i]-wis[i]) > 1e-9*(1+math.Abs(wz[i])) {
			t.Fatalf("W·z ≠ W⁻¹·s в координате %d: %g против %g", i, wz[i], wis[i])
		}
	}
}

// W⁻¹ обязана быть обратной к W — тождество, которое легко нарушить,
// перепутав знак отражения J.
func TestScaling_InverseIsInverse(t *testing.T) {
	k := Cone{SOC: []int{4}}
	s := []float64{3.0, 1.0, 0.5, -0.5}
	z := []float64{5.0, -1.0, 2.0, 1.0}
	sc := k.prepare()
	if !k.update(s, z, &sc) {
		t.Fatal("масштабирование не построилось")
	}
	x := []float64{0.7, -1.3, 2.1, 0.4}
	tmp := make([]float64, 4)
	back := make([]float64, 4)
	k.apply(&sc, x, tmp)
	k.applyInv(&sc, tmp, back)
	for i := range x {
		if math.Abs(back[i]-x[i]) > 1e-10*(1+math.Abs(x[i])) {
			t.Fatalf("W⁻¹Wx ≠ x в координате %d: %g против %g", i, back[i], x[i])
		}
	}
}

// Шаг до границы конуса обязан быть точным: в точке u+α·du определитель
// конуса обращается в ноль, а чуть раньше — строго положителен.
func TestCone_MaxStepTouchesBoundary(t *testing.T) {
	k := Cone{SOC: []int{3}}
	u := []float64{2, 0, 0}
	du := []float64{-1, 1, 0}
	alpha := k.maxStep(u, du)
	at := func(a float64) []float64 {
		return []float64{u[0] + a*du[0], u[1] + a*du[1], u[2] + a*du[2]}
	}
	if d := jdet(at(alpha)); math.Abs(d) > 1e-9 {
		t.Fatalf("на шаге α=%g определитель %g, ожидался ноль", alpha, d)
	}
	if d := jdet(at(alpha * 0.99)); d <= 0 {
		t.Fatalf("до границы определитель %g, ожидался положительный", d)
	}
}

// Сборка разреженной матрицы: дубликаты складываются, порядок добавления
// не влияет на результат.
func TestSparse_DuplicatesAccumulate(t *testing.T) {
	b := NewSparseBuilder(2, 3)
	b.Add(0, 2, 1)
	b.Add(1, 0, 5)
	b.Add(0, 0, 2)
	b.Add(0, 2, 3)
	m := b.Build()

	x := []float64{1, 1, 1}
	out := make([]float64, 2)
	m.MulVec(x, out)
	if out[0] != 6 || out[1] != 5 {
		t.Fatalf("A·1 = %v, ожидалось [6 5]", out)
	}

	y := []float64{1, 1}
	outT := make([]float64, 3)
	m.MulTVec(y, outT)
	if outT[0] != 7 || outT[1] != 0 || outT[2] != 4 {
		t.Fatalf("Aᵀ·1 = %v, ожидалось [7 0 4]", outT)
	}
}
