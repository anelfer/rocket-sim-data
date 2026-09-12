// Package socp — решатель задач конического программирования второго порядка
// (SOCP), написанный на голом stdlib.
//
// Зачем он здесь. Наведение посадочного импульса построено на G-FOLD
// (см. simulator/orbit/gfold) — а G-FOLD это не формула, а выпуклая задача
// оптимизации, которую борт решает заново каждые несколько секунд. Готовой
// библиотеки взять неоткуда: у проекта нет и не должно быть внешних
// зависимостей, кроме веб-сервера и Prometheus (go.mod), — значит численное
// ядро пишется здесь целиком.
//
// Это не «универсальный оптимизатор на все случаи». Он умеет ровно тот класс
// задач, который порождает G-FOLD:
//
//	минимизировать   cᵀx
//	при условиях     A x = b            (линейные равенства)
//	                 G x + s = h,  s ∈ K (конические неравенства)
//	                 K = R₊^Linear × Q^{SOC[0]} × … × Q^{SOC[n−1]}
//
// где Q^d — конус второго порядка (Лоренца) размерности d:
// Q^d = { (u₀,u₁) ∈ R×R^{d−1} : ‖u₁‖ ≤ u₀ }.
//
// Метод — прямо-двойственный метод внутренней точки с масштабированием
// Нестерова—Тодда и предиктор-корректором Мехротры: тот же алгоритм, что и в
// ECOS/CVXOPT, только без разреженной факторизации общего вида (задача
// G-FOLD маленькая и структурированная, см. kkt.go).
package socp

// Sparse — разреженная матрица в построчном формате (CSR).
//
// Плотное хранение здесь было бы не «проще, но медленнее», а именно неверным
// по существу: в задаче G-FOLD строка ограничения на тягу в узле k трогает
// четыре переменные из сотни, и плотная матрица заставила бы каждое умножение
// перебирать девяносто шесть заведомых нулей. Формирование нормальной матрицы
// KKT (kkt.go) — квадратично по числу ненулей в строке, так что разреженность
// здесь экономит не проценты, а порядки.
type Sparse struct {
	Rows, Cols int

	// RowPtr — границы строк в Col/Val: строка i занимает
	// Col[RowPtr[i]:RowPtr[i+1]]. Длина Rows+1.
	RowPtr []int
	Col    []int
	Val    []float64
}

// SparseBuilder накапливает элементы в произвольном порядке и собирает из них
// Sparse. Порядок добавления не влияет на результат — это важно не для
// удобства, а для воспроизводимости: симуляция детерминирована по зерну (см.
// Simulation.sensorRng), и решатель не имеет права зависеть от порядка обхода
// чего бы то ни было.
type SparseBuilder struct {
	rows, cols int
	rowOf      []int
	colOf      []int
	val        []float64
}

// NewSparseBuilder создаёт накопитель для матрицы rows×cols.
func NewSparseBuilder(rows, cols int) *SparseBuilder {
	return &SparseBuilder{rows: rows, cols: cols}
}

// Add добавляет слагаемое v в элемент (r,c). Именно СЛАГАЕМОЕ, а не
// присваивание: строитель задачи (gfold) собирает строку ограничения из
// нескольких независимых вкладов, и повторный вызов для той же клетки обязан
// их сложить, а не затереть предыдущий. Нулевые значения не хранятся.
func (b *SparseBuilder) Add(r, c int, v float64) {
	if v == 0 {
		return
	}
	if r < 0 || r >= b.rows || c < 0 || c >= b.cols {
		panic("socp: индекс за границами матрицы")
	}
	b.rowOf = append(b.rowOf, r)
	b.colOf = append(b.colOf, c)
	b.val = append(b.val, v)
}

// Build собирает CSR. Дубликаты (одна и та же клетка, добавленная несколько
// раз) складываются; элементы внутри строки идут по возрастанию столбца.
func (b *SparseBuilder) Build() *Sparse {
	counts := make([]int, b.rows+1)
	for _, r := range b.rowOf {
		counts[r+1]++
	}
	for i := 0; i < b.rows; i++ {
		counts[i+1] += counts[i]
	}

	// Раскладка по строкам счётной сортировкой — устойчивая и без карт:
	// порядок внутри строки пока произвольный, ниже он приводится к
	// возрастанию столбца слиянием дубликатов.
	pos := make([]int, b.rows)
	copy(pos, counts[:b.rows])
	col := make([]int, len(b.val))
	val := make([]float64, len(b.val))
	for i, r := range b.rowOf {
		p := pos[r]
		col[p] = b.colOf[i]
		val[p] = b.val[i]
		pos[r]++
	}

	out := &Sparse{
		Rows:   b.rows,
		Cols:   b.cols,
		RowPtr: make([]int, b.rows+1),
		Col:    make([]int, 0, len(val)),
		Val:    make([]float64, 0, len(val)),
	}
	for i := 0; i < b.rows; i++ {
		lo, hi := counts[i], counts[i+1]
		sortRow(col[lo:hi], val[lo:hi])
		out.RowPtr[i] = len(out.Col)
		for j := lo; j < hi; j++ {
			if j > lo && col[j] == col[j-1] {
				out.Val[len(out.Val)-1] += val[j]
				continue
			}
			out.Col = append(out.Col, col[j])
			out.Val = append(out.Val, val[j])
		}
		// Слияние могло обнулить клетку — хранить явный ноль незачем.
		for len(out.Val) > out.RowPtr[i] && out.Val[len(out.Val)-1] == 0 {
			out.Col = out.Col[:len(out.Col)-1]
			out.Val = out.Val[:len(out.Val)-1]
		}
	}
	out.RowPtr[b.rows] = len(out.Col)
	return out
}

// sortRow — сортировка вставками пары (индекс, значение) по индексу. Строки
// здесь короткие (единицы-десятки элементов), и вставками это быстрее любой
// общей сортировки с её накладными расходами на интерфейсы.
func sortRow(col []int, val []float64) {
	for i := 1; i < len(col); i++ {
		c, v := col[i], val[i]
		j := i - 1
		for j >= 0 && col[j] > c {
			col[j+1], val[j+1] = col[j], val[j]
			j--
		}
		col[j+1], val[j+1] = c, v
	}
}

// MulVec складывает A·x в out (out += A·x, out НЕ обнуляется вызовом —
// вызывающая сторона решает, накапливать или начинать с нуля).
func (m *Sparse) MulVec(x, out []float64) {
	if m == nil {
		return
	}
	for i := 0; i < m.Rows; i++ {
		sum := 0.0
		for k := m.RowPtr[i]; k < m.RowPtr[i+1]; k++ {
			sum += m.Val[k] * x[m.Col[k]]
		}
		out[i] += sum
	}
}

// MulTVec складывает Aᵀ·y в out (накопление, как и MulVec).
func (m *Sparse) MulTVec(y, out []float64) {
	if m == nil {
		return
	}
	for i := 0; i < m.Rows; i++ {
		yi := y[i]
		if yi == 0 {
			continue
		}
		for k := m.RowPtr[i]; k < m.RowPtr[i+1]; k++ {
			out[m.Col[k]] += m.Val[k] * yi
		}
	}
}
