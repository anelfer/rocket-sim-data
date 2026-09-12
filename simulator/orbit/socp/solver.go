package socp

import "math"

// Problem — задача в стандартной форме (см. обзор пакета в sparse.go).
//
//	минимизировать   cᵀx
//	при условиях     A x = b
//	                 G x + s = h,  s ∈ K
type Problem struct {
	C []float64
	A *Sparse
	B []float64
	G *Sparse
	H []float64
	K Cone
}

// Status — исход решения. Не bool и не error: «не сошлось» и «задача
// неразрешима» — разные факты, и наведение обязано различать их (первое
// значит «взять предыдущее решение», второе — «цель физически недостижима,
// пересмотреть задачу», см. gfold).
type Status int

const (
	// StatusOptimal — найдено решение с невязками и зазором двойственности
	// ниже допуска.
	StatusOptimal Status = iota

	// StatusInfeasible — задача не имеет допустимых решений (обнаружено по
	// расходящемуся сертификату, см. ниже).
	StatusInfeasible

	// StatusUnbounded — целевая функция не ограничена снизу на допустимом
	// множестве.
	StatusUnbounded

	// StatusMaxIterations — предел итераций исчерпан. Возвращаемая точка
	// допустима с точностью до достигнутых невязок, но оптимальность не
	// доказана.
	StatusMaxIterations

	// StatusNumerical — разложение или масштабирование потеряли смысл
	// (вырожденная пара s,z, неконечные числа). Возвращать такую точку в
	// наведение нельзя.
	StatusNumerical
)

// String возвращает название исхода для логов и телеметрии.
func (s Status) String() string {
	switch s {
	case StatusOptimal:
		return "optimal"
	case StatusInfeasible:
		return "infeasible"
	case StatusUnbounded:
		return "unbounded"
	case StatusMaxIterations:
		return "max-iterations"
	case StatusNumerical:
		return "numerical-failure"
	default:
		return "?"
	}
}

// Options — настройки метода.
type Options struct {
	// MaxIterations — предел числа итераций. Метод внутренней точки сходится
	// сверхлинейно, и для задач размера G-FOLD штатно укладывается в
	// полтора-два десятка итераций; предел нужен только чтобы плохо
	// обусловленная задача не тратила такт наведения целиком.
	MaxIterations int

	// Tolerance — относительный допуск по невязкам и зазору двойственности.
	Tolerance float64

	// Regularization — δ статической регуляризации KKT (см. kkt.go).
	Regularization float64

	// StepFraction — доля шага до границы конуса (0<f<1). Ровно до границы
	// шагать нельзя: барьер там обращается в бесконечность.
	StepFraction float64

	// StallIterations — сколько итераций подряд метод имеет право не
	// улучшать невязки, прежде чем счёт прекращается.
	//
	// Это не «на всякий случай», а существенная часть цены решения.
	// Наведение перебирает время полёта, и заведомо неразрешимые пробные
	// значения — норма, а не исключение: на них метод внутренней точки не
	// расходится красиво, он просто перестаёт двигаться и досиживает до
	// предела итераций. Измерено: неразрешимая постановка тратила все 50
	// итераций там, где через 15 уже было видно, что улучшения нет, —
	// то есть втрое дороже, чем разрешимая, хотя пользы от неё ноль.
	StallIterations int
}

// DefaultOptions — настройки, с которыми решается задача посадки.
//
// Допуск 1e-8 относительный: величины задачи после обезразмеривания (см.
// gfold — положение делится на характерную высоту, время на характерное
// время) имеют порядок единицы, и 1e-8 здесь означает восемь верных знаков,
// а не «восемь знаков от километра».
func DefaultOptions() Options {
	return Options{
		MaxIterations:   50,
		Tolerance:       1e-8,
		Regularization:  1e-9,
		StepFraction:    0.99,
		StallIterations: 8,
	}
}

// Solution — результат решения.
type Solution struct {
	X, Y, Z, S []float64

	Status     Status
	Iterations int

	// Objective — значение cᵀx в найденной точке.
	Objective float64

	// Gap — зазор двойственности sᵀz/степень конуса.
	Gap float64

	// PrimalResidual, DualResidual — относительные невязки прямых и
	// двойственных условий в найденной точке.
	PrimalResidual, DualResidual float64
}

// Solve решает задачу методом внутренней точки с предиктор-корректором
// Мехротры.
//
// Алгоритм по шагам (обозначения — как в описании Problem):
//
//  1. Начальная точка: решение KKT при W=I, сдвинутое внутрь конуса
//     (см. Cone.initialPoint). Это стандартный «тёплый» старт ECOS: он
//     заметно ближе к центральному пути, чем формальное s=z=e, и экономит
//     несколько итераций на каждом решении.
//  2. Невязки rx, ry, rz и зазор μ = sᵀz/degree.
//  3. Масштабирование Нестерова—Тодда W по текущей паре (s,z), λ = W·z.
//  4. Аффинное направление (предиктор): правая часть условия дополняющей
//     нежёсткости — −λ∘λ, то есть «шаг, полностью убивающий зазор».
//  5. Параметр центрирования σ = (1−α_aff)³ — эвристика Мехротры: чем
//     дальше удалось шагнуть по аффинному направлению, тем меньше нужно
//     центрирования.
//  6. Комбинированное направление (корректор) с правой частью
//     −λ∘λ − (W⁻¹ds_aff)∘(W dz_aff) + σμ·e.
//  7. Шаг с коэффициентом StepFraction до границы конуса.
func Solve(p *Problem, opt Options) Solution {
	n := len(p.C)
	m := p.K.Dim()
	pRows := 0
	if p.A != nil {
		pRows = p.A.Rows
	}

	sol := Solution{
		X: make([]float64, n),
		Y: make([]float64, pRows),
		Z: make([]float64, m),
		S: make([]float64, m),
	}

	if m != len(p.H) || (p.G != nil && (p.G.Rows != m || p.G.Cols != n)) {
		sol.Status = StatusNumerical
		return sol
	}

	kkt := newKKT(n, pRows)

	x, y, z, s := sol.X, sol.Y, sol.Z, sol.S
	if !initialPoint(p, kkt, opt, x, y, z, s) {
		sol.Status = StatusNumerical
		return sol
	}

	degree := float64(p.K.Degree())
	normC := 1 + norm2(p.C)
	normB := 1 + norm2(p.B)
	normH := 1 + norm2(p.H)

	// Рабочие векторы. Все выделяются один раз: цикл ниже исполняется
	// десятки раз за решение, и решение — десятки раз за посадку.
	rx := make([]float64, n)
	ry := make([]float64, pRows)
	rz := make([]float64, m)
	lambda := make([]float64, m)
	dlambda := make([]float64, m)
	rhsC := make([]float64, m)
	tmpM := make([]float64, m)
	tmpM2 := make([]float64, m)
	q := make([]float64, m)
	dx := make([]float64, n)
	dy := make([]float64, pRows)
	dz := make([]float64, m)
	ds := make([]float64, m)
	dsAff := make([]float64, m)
	dzAff := make([]float64, m)
	rhs := make([]float64, n+pRows)
	out := make([]float64, n+pRows)
	sc := p.K.prepare()

	bestMerit := math.Inf(1)
	stall := 0

	for iter := 0; iter <= opt.MaxIterations; iter++ {
		// Невязки условий оптимальности:
		//   rx = −(c + Aᵀy + Gᵀz)   — двойственная допустимость
		//   ry = b − A·x            — прямая допустимость (равенства)
		//   rz = h − G·x − s        — прямая допустимость (конус)
		copy(rx, p.C)
		p.A.MulTVec(y, rx)
		p.G.MulTVec(z, rx)
		for i := range rx {
			rx[i] = -rx[i]
		}
		for i := range ry {
			ry[i] = 0
		}
		p.A.MulVec(x, ry)
		for i := range ry {
			ry[i] = p.B[i] - ry[i]
		}
		for i := range rz {
			rz[i] = 0
		}
		p.G.MulVec(x, rz)
		for i := range rz {
			rz[i] = p.H[i] - rz[i] - s[i]
		}

		gap := 0.0
		for i := range s {
			gap += s[i] * z[i]
		}
		mu := gap / degree

		pres := math.Max(norm2(ry)/normB, norm2(rz)/normH)
		dres := norm2(rx) / normC
		sol.Iterations = iter
		sol.Gap = mu
		sol.PrimalResidual = pres
		sol.DualResidual = dres
		sol.Objective = dot(p.C, x)

		if pres <= opt.Tolerance && dres <= opt.Tolerance &&
			mu <= opt.Tolerance*math.Max(1, math.Abs(sol.Objective)) {
			sol.Status = StatusOptimal
			return sol
		}
		if iter == opt.MaxIterations {
			sol.Status = classifyStalled(pres, dres, mu, opt)
			return sol
		}

		// Единая мера продвижения: худшая из относительных невязок и
		// нормированный зазор. Если она перестала падать — метод стоит, и
		// оставшиеся итерации ничего не добавят.
		merit := math.Max(pres, math.Max(dres, mu/(1+math.Abs(sol.Objective))))
		if merit < bestMerit*0.99 {
			bestMerit = merit
			stall = 0
		} else {
			stall++
		}
		if opt.StallIterations > 0 && stall >= opt.StallIterations {
			sol.Status = classifyStalled(pres, dres, mu, opt)
			return sol
		}

		if !p.K.update(s, z, &sc) {
			sol.Status = StatusNumerical
			return sol
		}
		p.K.apply(&sc, z, lambda)

		// Матрица KKT зависит только от масштабирования — она одна и та же
		// для предиктора и корректора этой итерации. Факторизация здесь
		// самая дорогая операция метода, и повторять её ради второй правой
		// части было бы буквально удвоением стоимости решения.
		if !kkt.assemble(p, &sc, opt.Regularization) {
			sol.Status = StatusNumerical
			return sol
		}

		// --- предиктор: правая часть −λ∘λ, то есть dλ = −λ.
		for i := range lambda {
			dlambda[i] = -lambda[i]
		}
		if !solveDirection(p, kkt, &sc, rx, ry, rz, dlambda, q, tmpM, rhs, out, dx, dy, dz, ds) {
			sol.Status = StatusNumerical
			return sol
		}
		copy(dsAff, ds)
		copy(dzAff, dz)

		alphaAff := math.Min(p.K.maxStep(s, dsAff), p.K.maxStep(z, dzAff))
		alphaAff = math.Min(alphaAff, 1)
		if alphaAff < 0 {
			alphaAff = 0
		}
		sigma := math.Pow(1-alphaAff, 3)

		// --- корректор: −λ∘λ − (W⁻¹ds_aff)∘(W dz_aff) + σμ·e.
		p.K.applyInv(&sc, dsAff, tmpM)
		p.K.apply(&sc, dzAff, tmpM2)
		p.K.prod(tmpM, tmpM2, rhsC)
		p.K.prod(lambda, lambda, tmpM)
		for i := range rhsC {
			rhsC[i] = -tmpM[i] - rhsC[i]
		}
		addIdentity(p.K, rhsC, sigma*mu)
		if !p.K.divide(lambda, rhsC, dlambda) {
			sol.Status = StatusNumerical
			return sol
		}
		if !solveDirection(p, kkt, &sc, rx, ry, rz, dlambda, q, tmpM, rhs, out, dx, dy, dz, ds) {
			sol.Status = StatusNumerical
			return sol
		}

		alpha := math.Min(p.K.maxStep(s, ds), p.K.maxStep(z, dz))
		if math.IsInf(alpha, 1) {
			alpha = 1
		}
		alpha = math.Min(opt.StepFraction*alpha, 1)
		if alpha <= 0 || math.IsNaN(alpha) {
			sol.Status = classifyStalled(pres, dres, mu, opt)
			return sol
		}

		for i := range x {
			x[i] += alpha * dx[i]
		}
		for i := range y {
			y[i] += alpha * dy[i]
		}
		for i := range z {
			z[i] += alpha * dz[i]
			s[i] += alpha * ds[i]
		}
	}

	sol.Status = StatusMaxIterations
	return sol
}

// solveDirection решает KKT для заданного dλ и раскладывает результат в
// (dx,dy,dz,ds). Вывод формул — в kkt.go; здесь буквально они:
//
//	q  = rz − W·dλ
//	⎡GᵀW⁻²G  Aᵀ⎤⎡dx⎤ = ⎡rx + GᵀW⁻²q⎤
//	⎣A       0 ⎦⎣dy⎦   ⎣ry          ⎦
//	ds = rz − G·dx
//	dz = W⁻²(G·dx − q)
func solveDirection(p *Problem, kkt *kktSystem, sc *scaling,
	rx, ry, rz, dlambda, q, tmp, rhs, out, dx, dy, dz, ds []float64) bool {

	n := len(p.C)

	p.K.apply(sc, dlambda, tmp)
	for i := range q {
		q[i] = rz[i] - tmp[i]
	}

	// tmp := W⁻²·q
	p.K.applyInv(sc, q, ds)
	p.K.applyInv(sc, ds, tmp)

	copy(rhs[:n], rx)
	p.G.MulTVec(tmp, rhs[:n])
	copy(rhs[n:], ry)

	kkt.solve(rhs, out)
	copy(dx, out[:n])
	copy(dy, out[n:])

	// ds = rz − G·dx
	for i := range ds {
		ds[i] = 0
	}
	p.G.MulVec(dx, ds)
	for i := range ds {
		ds[i] = rz[i] - ds[i]
	}

	// dz = W⁻²(G·dx − q)
	for i := range tmp {
		tmp[i] = -q[i]
	}
	p.G.MulVec(dx, tmp)
	p.K.applyInv(sc, tmp, dz)
	p.K.applyInv(sc, dz, tmp)
	copy(dz, tmp)

	for _, v := range [][]float64{dx, dy, dz, ds} {
		for _, e := range v {
			if math.IsNaN(e) || math.IsInf(e, 0) {
				return false
			}
		}
	}
	return true
}

// initialPoint строит стартовую точку: решение KKT при W=I, сдвинутое
// внутрь конуса.
func initialPoint(p *Problem, kkt *kktSystem, opt Options, x, y, z, s []float64) bool {
	n := len(p.C)
	pRows := len(y)
	rhs := make([]float64, n+pRows)
	out := make([]float64, n+pRows)

	if !kkt.assemble(p, nil, opt.Regularization) {
		return false
	}

	// Прямая часть: минимальная по норме точка, удовлетворяющая равенствам.
	for i := range rhs {
		rhs[i] = 0
	}
	p.G.MulTVec(p.H, rhs[:n])
	copy(rhs[n:], p.B)
	kkt.solve(rhs, out)
	copy(x, out[:n])
	for i := range s {
		s[i] = -p.H[i]
	}
	p.G.MulVec(x, s)
	for i := range s {
		s[i] = -s[i]
	}
	p.K.initialPoint(s)

	// Двойственная часть.
	for i := range rhs {
		rhs[i] = 0
	}
	for i := 0; i < n; i++ {
		rhs[i] = -p.C[i]
	}
	kkt.solve(rhs, out)
	copy(y, out[n:])
	for i := range z {
		z[i] = 0
	}
	p.G.MulVec(out[:n], z)
	p.K.initialPoint(z)

	for _, v := range [][]float64{x, y, z, s} {
		for _, e := range v {
			if math.IsNaN(e) || math.IsInf(e, 0) {
				return false
			}
		}
	}
	return true
}

// classifyStalled различает «не сошлось по числу итераций» и «сертификат
// неразрешимости»: если прямая невязка упала, а двойственная нет (или
// наоборот) при разошедшемся зазоре, задача неразрешима с соответствующей
// стороны. Это грубее полноценного вложения в самодвойственную задачу, но
// честно: сомнительный случай возвращается как StatusMaxIterations, а не
// выдаётся за доказанную неразрешимость.
func classifyStalled(pres, dres, mu float64, opt Options) Status {
	const certain = 1e3
	switch {
	case pres <= opt.Tolerance && dres > certain*opt.Tolerance && mu > 1:
		return StatusUnbounded
	case dres <= opt.Tolerance && pres > certain*opt.Tolerance && mu > 1:
		return StatusInfeasible
	default:
		return StatusMaxIterations
	}
}

// addIdentity прибавляет k·e, где e — единица конуса.
func addIdentity(k Cone, v []float64, val float64) {
	for i := 0; i < k.Linear; i++ {
		v[i] += val
	}
	off := k.Linear
	for _, n := range k.SOC {
		v[off] += val
		off += n
	}
}

func dot(a, b []float64) float64 {
	s := 0.0
	for i := range a {
		s += a[i] * b[i]
	}
	return s
}
