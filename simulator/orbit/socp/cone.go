package socp

import "math"

// Cone описывает конус K = R₊^Linear × Q^{SOC[0]} × … × Q^{SOC[k−1]}.
//
// Порядок строк в G/h жёстко привязан к этому описанию: сначала Linear строк
// неотрицательного ортанта, затем блоки конусов второго порядка подряд, в
// порядке SOC. Отдельного «индекса блока» нет намеренно — он выводится из
// самого описания (см. blocks), и рассинхронизировать их поэтому нечем.
type Cone struct {
	// Linear — размерность неотрицательного ортанта (число обычных
	// линейных неравенств).
	Linear int

	// SOC — размерности конусов второго порядка. Размерность 1 не
	// допускается: Q¹ = R₊ это уже линейная часть, и держать её отдельным
	// блоком значило бы иметь два разных пути для одного и того же случая.
	SOC []int
}

// Dim — полная размерность конуса (число строк в G/h).
func (k Cone) Dim() int {
	d := k.Linear
	for _, n := range k.SOC {
		d += n
	}
	return d
}

// Degree — «степень» конуса: число, на которое делится sᵀz при вычислении
// параметра центрирования μ. Для ортанта это его размерность (каждая
// координата — самостоятельное условие дополняющей нежёсткости), для конуса
// второго порядка ЛЮБОЙ размерности — единица: у него одно собственное
// значение пары (λ₀±‖λ₁‖), а не d штук.
func (k Cone) Degree() int {
	return k.Linear + len(k.SOC)
}

// block — границы блока конуса второго порядка в векторе длины Dim().
type block struct{ lo, hi int }

// blocks возвращает границы всех SOC-блоков.
func (k Cone) blocks() []block {
	out := make([]block, len(k.SOC))
	off := k.Linear
	for i, n := range k.SOC {
		out[i] = block{off, off + n}
		off += n
	}
	return out
}

// jdet — «определитель» вектора в конусе второго порядка: u₀² − ‖u₁‖².
// Положителен строго внутри конуса, ноль на границе.
func jdet(u []float64) float64 {
	s := u[0] * u[0]
	for i := 1; i < len(u); i++ {
		s -= u[i] * u[i]
	}
	return s
}

// initialPoint приводит вектор u в строго внутреннюю точку конуса,
// сдвигая его вдоль единицы конуса e = (1,0,…,0).
//
// Сдвиг именно вдоль e, а не проекция на конус: проекция кладёт точку НА
// границу, где логарифмический барьер обращается в бесконечность и метод
// внутренней точки не может стартовать в принципе. Величина сдвига — ровно
// столько, чтобы уйти от границы на единицу масштаба самой невязки.
func (k Cone) initialPoint(u []float64) {
	if k.Linear > 0 {
		min := math.Inf(1)
		for i := 0; i < k.Linear; i++ {
			if u[i] < min {
				min = u[i]
			}
		}
		if min <= 0 {
			shift := 1 - min
			for i := 0; i < k.Linear; i++ {
				u[i] += shift
			}
		}
	}
	off := k.Linear
	for _, n := range k.SOC {
		v := u[off : off+n]
		off += n
		norm := norm2(v[1:])
		if margin := v[0] - norm; margin <= 0 {
			v[0] += 1 - margin
		}
	}
}

// maxStep возвращает наибольшее α ≥ 0, при котором u + α·du ещё принадлежит
// конусу (+Inf, если ограничения нет).
//
// Считается напрямую по определению конуса, а не через масштабированные
// переменные: условие (u₀+αdu₀)² − ‖u₁+αdu₁‖² ≥ 0 — обычный квадратный
// трёхчлен по α, и его меньший положительный корень и есть искомый шаг.
// Через масштабирование Нестерова—Тодда получилось бы то же самое число, но
// с лишним преобразованием, в котором нечему быть точнее.
func (k Cone) maxStep(u, du []float64) float64 {
	alpha := math.Inf(1)

	for i := 0; i < k.Linear; i++ {
		if du[i] < 0 {
			if a := -u[i] / du[i]; a < alpha {
				alpha = a
			}
		}
	}

	off := k.Linear
	for _, n := range k.SOC {
		v, dv := u[off:off+n], du[off:off+n]
		off += n

		// a·α² + b·α + c ≥ 0 при c = jdet(v) > 0 (текущая точка внутри).
		qa := jdet(dv)
		qb := 2 * v[0] * dv[0]
		for i := 1; i < len(v); i++ {
			qb -= 2 * v[i] * dv[i]
		}
		qc := jdet(v)

		lim := math.Inf(1)
		switch {
		case qa == 0:
			// Линейный случай: b·α + c ≥ 0.
			if qb < 0 {
				lim = -qc / qb
			}
		default:
			disc := qb*qb - 4*qa*qc
			if disc >= 0 {
				sq := math.Sqrt(disc)
				// Корни в устойчивой форме (без вычитания близких чисел).
				var r1, r2 float64
				if qb >= 0 {
					r1 = (-qb - sq) / (2 * qa)
					r2 = 2 * qc / (-qb - sq)
				} else {
					r1 = (-qb + sq) / (2 * qa)
					r2 = 2 * qc / (-qb + sq)
				}
				for _, r := range [2]float64{r1, r2} {
					if r > 0 && r < lim && !math.IsNaN(r) {
						lim = r
					}
				}
			} else if qa < 0 {
				// Ветвь параболы вниз без вещественных корней невозможна
				// при qc > 0 — оставляем +Inf, шаг ограничит условие на
				// первую координату ниже.
				lim = math.Inf(1)
			}
		}
		// Первая координата обязана остаться неотрицательной: у трёхчлена
		// выше есть вторая ветвь (v₀ < 0 при формально положительном
		// определителе), и без этой проверки шаг мог бы «перепрыгнуть» в
		// зеркальный конус −K.
		if dv[0] < 0 {
			if a := -v[0] / dv[0]; a < lim {
				lim = a
			}
		}
		if lim < alpha {
			alpha = lim
		}
	}
	return alpha
}

// prod — произведение Йордана u ∘ v в алгебре конуса: покомпонентное для
// ортанта, (uᵀv, u₀v₁ + v₀u₁) для конуса второго порядка. Условие
// дополняющей нежёсткости s ∘ z = 0 записывается именно через него.
func (k Cone) prod(u, v, out []float64) {
	for i := 0; i < k.Linear; i++ {
		out[i] = u[i] * v[i]
	}
	off := k.Linear
	for _, n := range k.SOC {
		uu, vv, oo := u[off:off+n], v[off:off+n], out[off:off+n]
		off += n
		dot := 0.0
		for i := range uu {
			dot += uu[i] * vv[i]
		}
		oo[0] = dot
		for i := 1; i < len(uu); i++ {
			oo[i] = uu[0]*vv[i] + vv[0]*uu[i]
		}
	}
}

// divide решает λ ∘ x = b относительно x (обращение стрелочной матрицы
// Arw(λ)). Для ортанта это деление по координатам, для конуса второго
// порядка — замкнутая формула обратной стрелочной матрицы.
func (k Cone) divide(lambda, b, out []float64) bool {
	for i := 0; i < k.Linear; i++ {
		if lambda[i] == 0 {
			return false
		}
		out[i] = b[i] / lambda[i]
	}
	off := k.Linear
	for _, n := range k.SOC {
		l, rhs, x := lambda[off:off+n], b[off:off+n], out[off:off+n]
		off += n
		det := jdet(l)
		if det == 0 || l[0] == 0 {
			return false
		}
		// x₀ = (l₀·b₀ − l₁ᵀb₁)/det
		// x₁ = (−l₁·b₀ + (det/l₀)·b₁ + l₁(l₁ᵀb₁)/l₀)/det
		dotL1B1 := 0.0
		for i := 1; i < len(l); i++ {
			dotL1B1 += l[i] * rhs[i]
		}
		x[0] = (l[0]*rhs[0] - dotL1B1) / det
		for i := 1; i < len(l); i++ {
			x[i] = (-l[i]*rhs[0] + (det/l[0])*rhs[i] + l[i]*dotL1B1/l[0]) / det
		}
	}
	return true
}

// scaling — масштабирование Нестерова—Тодда для текущей пары (s,z).
//
// Смысл: найти такое W (симметричное, положительно определённое), что
// W·z = W⁻¹·s. В этих координатах прямая и двойственная переменные
// становятся симметричны, и шаг Ньютона по условию дополняющей нежёсткости
// перестаёт зависеть от того, какая из них «дальше» от границы. Без такого
// масштабирования метод внутренней точки для конусов не сходится за
// разумное число итераций — это не оптимизация реализации, а часть метода.
//
// Для ортанта W = diag(√(sᵢ/zᵢ)). Для конуса второго порядка W имеет
// «стрелочный» вид, полностью определяемый одним вектором w с w₀²−‖w₁‖²=1
// и множителем η.
//
// Хранение — плоскими массивами с общей нарезкой по блокам, а не срезом на
// блок: масштабирование пересчитывается на КАЖДОЙ итерации метода, блоков
// в задаче G-FOLD под сотню, и выделение памяти на каждый блок каждой
// итерации кормило бы сборщик мусора ровно в самом горячем цикле.
type scaling struct {
	// diag — √(sᵢ/zᵢ) для линейной части.
	diag []float64

	// eta — множитель η по блокам конусов второго порядка.
	eta []float64

	// w — векторы w всех блоков подряд; блок bi занимает w[off[bi]:off[bi+1]].
	w   []float64
	off []int
}

// blockW возвращает вектор w блока bi.
func (sc *scaling) blockW(bi int) []float64 { return sc.w[sc.off[bi]:sc.off[bi+1]] }

// prepare выделяет память под масштабирование конуса ОДИН раз.
func (k Cone) prepare() scaling {
	sc := scaling{
		diag: make([]float64, k.Linear),
		eta:  make([]float64, len(k.SOC)),
		off:  make([]int, len(k.SOC)+1),
	}
	total := 0
	for i, n := range k.SOC {
		sc.off[i] = total
		total += n
	}
	sc.off[len(k.SOC)] = total
	sc.w = make([]float64, total)
	return sc
}

// update пересчитывает масштабирование по текущей паре (s,z) в уже
// выделенную память.
func (k Cone) update(s, z []float64, sc *scaling) bool {
	for i := 0; i < k.Linear; i++ {
		if s[i] <= 0 || z[i] <= 0 {
			return false
		}
		sc.diag[i] = math.Sqrt(s[i] / z[i])
	}
	off := k.Linear
	for bi, n := range k.SOC {
		ss, zz := s[off:off+n], z[off:off+n]
		off += n

		ds, dz := jdet(ss), jdet(zz)
		if ds <= 0 || dz <= 0 {
			return false
		}
		sqrtS, sqrtZ := math.Sqrt(ds), math.Sqrt(dz)

		// γ² = (1 + s̄·z̄)/2, где s̄ = s/√det s, z̄ = z/√det z.
		dot := 0.0
		for i := 0; i < n; i++ {
			dot += (ss[i] / sqrtS) * (zz[i] / sqrtZ)
		}
		gamma := math.Sqrt((1 + dot) / 2)
		if gamma <= 0 || math.IsNaN(gamma) {
			return false
		}

		// w = (s̄ + J·z̄)/(2γ), J = diag(1,−I). Прямой подстановкой
		// проверяется w₀²−‖w₁‖² = 1.
		w := sc.blockW(bi)
		w[0] = (ss[0]/sqrtS + zz[0]/sqrtZ) / (2 * gamma)
		for i := 1; i < n; i++ {
			w[i] = (ss[i]/sqrtS - zz[i]/sqrtZ) / (2 * gamma)
		}
		sc.eta[bi] = math.Pow(ds/dz, 0.25)
	}
	return true
}

// apply вычисляет out = W·x.
func (k Cone) apply(sc *scaling, x, out []float64) {
	for i := 0; i < k.Linear; i++ {
		out[i] = sc.diag[i] * x[i]
	}
	off := k.Linear
	for bi, n := range k.SOC {
		socApply(sc.eta[bi], sc.blockW(bi), false, x[off:off+n], out[off:off+n])
		off += n
	}
}

// applyInv вычисляет out = W⁻¹·x.
//
// Обратная матрица берётся не численно, а из тождества Ŵ·J·Ŵ = J для
// стрелочной матрицы с w₀²−‖w₁‖²=1 (проверяется прямой подстановкой),
// где J = diag(1,−I). Отсюда W·J·W = η²·J, значит
//
//	W⁻¹ = J·W·J/η² = Ŵ(w₀, −w₁)/η,
//
// то есть ТА ЖЕ стрелочная матрица с обращённым знаком вектора w и
// множителем 1/η вместо η. Дополнительных отражений самого вектора не
// нужно: они уже поглощены сменой знака w.
func (k Cone) applyInv(sc *scaling, x, out []float64) {
	for i := 0; i < k.Linear; i++ {
		out[i] = x[i] / sc.diag[i]
	}
	off := k.Linear
	for bi, n := range k.SOC {
		socApply(1/sc.eta[bi], sc.blockW(bi), true, x[off:off+n], out[off:off+n])
		off += n
	}
}

// socApply — умножение на стрелочную матрицу
// W = η·[ w₀ w₁ᵀ ; w₁ I + w₁w₁ᵀ/(1+w₀) ]. При negate=true знак w₁
// обращается на лету (это и есть обратная матрица, см. applyInv) — без
// построения второго вектора.
func socApply(eta float64, w []float64, negate bool, x, out []float64) {
	sign := 1.0
	if negate {
		sign = -1
	}
	dot := 0.0
	for i := 1; i < len(w); i++ {
		dot += sign * w[i] * x[i]
	}
	out[0] = eta * (w[0]*x[0] + dot)
	c := dot / (1 + w[0])
	for i := 1; i < len(w); i++ {
		wi := sign * w[i]
		out[i] = eta * (wi*x[0] + x[i] + wi*c)
	}
}

func norm2(v []float64) float64 {
	s := 0.0
	for _, x := range v {
		s += x * x
	}
	return math.Sqrt(s)
}
