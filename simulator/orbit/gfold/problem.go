package gfold

import (
	"math"

	"rocketTelemetrySim/simulator/orbit/socp"
	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Сборка конической задачи.
//
// Оси посадочной системы: X — восток, Y — север, Z — вверх; начало — точка
// посадки на уровне поверхности. Вертикаль здесь третья координата, а не
// первая, как в статьях JPL: у проекта уже есть physics.LocalFrame ровно с
// таким порядком осей (East/North/Up), и держать в наведении вторую,
// переставленную систему значило бы завести ровно тот класс ошибок, который
// потом ищется неделями — перепутанные местами оси.
//
// Переменные задачи (см. discretize.go, состояния исключены):
//
//	x = (u₀ … u_N, σ₀ … σ_N [, промах])
//
// Ограничения (нумерация по разделам статьи):
//
//	линейные:  σ_k ≤ μ₂_k·(1 − δ_k)            верхняя граница тяги
//	           n̂·u_k ≥ σ_k·cos θ_max            угол отклонения тяги
//	           δ_k ≥ 0                          область применимости оценок
//	конусы:    ‖u_k‖ ≤ σ_k                      расщепление (то самое, что
//	                                             снимает невыпуклость)
//	           ‖(δ_k−1, σ_k/μ₁_k−1)‖ ≤ σ_k/μ₁_k  нижняя граница тяги
//	           ‖E·(r_k−вершина)‖ ≤ ctg γ·(…)     посадочный конус
//	равенства: v_N = v_кас, r_N = 0 (или только высота)
//
// где δ_k = z_k − z₀_k, а μ₁,μ₂ — границы тяги, отнесённые к минимально
// возможной массе в этом узле (discretize.go, massProfile).
//
// Про нижнюю границу тяги отдельно. Записать её как σ ≥ ρ₁e^{−z} нельзя:
// это ограничение выпукло (надграфик выпуклой функции), но не сводится к
// конусу второго порядка. Разложение экспоненты по Тейлору до второго
// порядка даёт неравенство, которое, во-первых, СВОДИТСЯ к конусу
// (квадратичная функция), во-вторых — заведомо СТРОЖЕ исходного
// (e^{−δ} ≤ 1−δ+δ²/2 при δ ≥ 0). Второе важнее первого: оценка строже
// исходной, значит любое решение приближённой задачи допустимо и для
// точной, и «починка» решения после счёта не нужна ни при каких условиях.
// Ошибка приближения — O(δ³/6), а δ на посадочном импульсе не превосходит
// нескольких сотых.
// -----------------------------------------------------------------------------

// mode — какая из трёх задач собирается.
type mode int

const (
	// modeExactFuel — минимум топлива при ТОЧНОМ попадании в цель.
	// Пробуется первой: при возврате на площадку цель обычно достижима, и
	// тогда весь двухэтапный механизм минимального промаха не нужен.
	modeExactFuel mode = iota

	// modeMinMiss — минимум горизонтального промаха (Problem 3). Решается,
	// только если точное попадание недостижимо, и отвечает на вопрос
	// «насколько близко вообще можно сесть».
	modeMinMiss

	// modeBoundedFuel — минимум топлива при промахе не хуже уже найденного
	// (Problem 4). Порядок именно такой — сначала точность, потом
	// экономия, — и он ЛЕКСИКОГРАФИЧЕСКИЙ, а не взвешенная сумма двух
	// целей: никакой вес не воспроизводит приоритет «точность важнее»
	// одинаково хорошо на всех начальных условиях, и подбирать его пришлось
	// бы заново под каждый сценарий.
	modeBoundedFuel
)

// layout — раскладка переменных задачи по индексам.
type layout struct {
	nodes int
	miss  int // индекс переменной промаха, −1 если её нет
	n     int
}

func newLayout(nodes int, m mode) layout {
	l := layout{nodes: nodes, miss: -1}
	l.n = 4 * (nodes + 1)
	if m == modeMinMiss {
		l.miss = l.n
		l.n++
	}
	return l
}

func (l layout) accel(k, axis int) int { return 3*k + axis }
func (l layout) sigma(k int) int       { return 3*(l.nodes+1) + k }

// build собирает коническую задачу. Возвращает также масштабы и таблицы,
// нужные для обратного пересчёта решения в физические единицы.
func build(s Setup, m mode) (*socp.Problem, scales, weights, []float64) {
	sc := newScales(s)
	w := newWeights(s.Nodes)
	z0 := massProfile(s, sc)
	posOff, velOff := affineOffsets(s, sc, w)

	n := s.Nodes
	lay := newLayout(n, m)
	dt := s.Step / sc.time
	alpha := s.Alpha * sc.length / sc.time

	// Границы тяги, отнесённые к минимально возможной массе узла.
	mu1 := make([]float64, n+1)
	mu2 := make([]float64, n+1)
	for k := 0; k <= n; k++ {
		scaleRho := sc.time * sc.time / (sc.mass * sc.length)
		mu1[k] = s.Rho1[k] * scaleRho * math.Exp(-z0[k])
		mu2[k] = s.Rho2[k] * scaleRho * math.Exp(-z0[k])
	}

	// --- равенства.
	posRows := 3
	if m != modeExactFuel {
		posRows = 1 // свободен только горизонт, высота касания задана всегда
	}
	pRows := 3 + posRows
	ab := socp.NewSparseBuilder(pRows, lay.n)
	bvec := make([]float64, pRows)

	vTarget := s.TargetVelocity.Scale(sc.time / sc.length)
	rTarget := s.TargetPosition.Scale(1 / sc.length)
	for axis := 0; axis < 3; axis++ {
		for i := 0; i <= n; i++ {
			if wv := w.vel[n][i]; wv != 0 {
				ab.Add(axis, lay.accel(i, axis), dt*wv)
			}
		}
		bvec[axis] = component(vTarget, axis) - component(velOff[n], axis)
	}
	if m == modeExactFuel {
		for axis := 0; axis < 3; axis++ {
			row := 3 + axis
			for i := 0; i <= n; i++ {
				if wp := w.pos[n][i]; wp != 0 {
					ab.Add(row, lay.accel(i, axis), dt*dt*wp)
				}
			}
			bvec[row] = component(rTarget, axis) - component(posOff[n], axis)
		}
	} else {
		for i := 0; i <= n; i++ {
			if wp := w.pos[n][i]; wp != 0 {
				ab.Add(3, lay.accel(i, 2), dt*dt*wp)
			}
		}
		bvec[3] = rTarget.Z - posOff[n].Z
	}

	// --- конические ограничения.
	glideOn := s.GlideSlope > 0 && s.GlideSlope < math.Pi/2
	glideFrom := s.GlideSkipNodes
	if glideFrom < 1 {
		glideFrom = 1
	}
	glideCount := 0
	if glideOn && n >= glideFrom {
		glideCount = n - glideFrom + 1
	}

	linear := 3 * (n + 1)
	cone := socp.Cone{Linear: linear}
	for k := 0; k <= n; k++ {
		cone.SOC = append(cone.SOC, 4) // ‖u_k‖ ≤ σ_k
	}
	for k := 0; k <= n; k++ {
		cone.SOC = append(cone.SOC, 3) // нижняя граница тяги
	}
	for i := 0; i < glideCount; i++ {
		cone.SOC = append(cone.SOC, 3) // посадочный конус
	}
	if m != modeExactFuel {
		cone.SOC = append(cone.SOC, 3) // промах: цель или ограничение
	}

	rows := cone.Dim()
	gb := socp.NewSparseBuilder(rows, lay.n)
	h := make([]float64, rows)

	// Линейная часть.
	for k := 0; k <= n; k++ {
		// σ_k + μ₂_k·z_k ≤ μ₂_k·(1 + z₀_k), где z_k = −α·Δt·Σ wᵛ σ_i.
		row := 3 * k
		gb.Add(row, lay.sigma(k), 1)
		for i := 0; i <= n; i++ {
			if wv := w.vel[k][i]; wv != 0 {
				gb.Add(row, lay.sigma(i), -mu2[k]*alpha*dt*wv)
			}
		}
		h[row] = mu2[k] * (1 + z0[k])

		// −n̂·u_k + cos θ·σ_k ≤ 0.
		row = 3*k + 1
		axis := s.PointAxis
		if axis.Norm() < 1e-12 {
			axis = physics.Vec3{Z: 1}
		}
		axis = axis.Unit()
		gb.Add(row, lay.accel(k, 0), -axis.X)
		gb.Add(row, lay.accel(k, 1), -axis.Y)
		gb.Add(row, lay.accel(k, 2), -axis.Z)
		gb.Add(row, lay.sigma(k), math.Cos(s.ThetaMax))
		h[row] = 0

		// −z_k ≤ −z₀_k, то есть δ_k ≥ 0.
		row = 3*k + 2
		for i := 0; i <= n; i++ {
			if wv := w.vel[k][i]; wv != 0 {
				gb.Add(row, lay.sigma(i), alpha*dt*wv)
			}
		}
		h[row] = -z0[k]
	}

	// Конусы ‖u_k‖ ≤ σ_k.
	off := linear
	for k := 0; k <= n; k++ {
		base := off + 4*k
		gb.Add(base, lay.sigma(k), -1)
		for axis := 0; axis < 3; axis++ {
			gb.Add(base+1+axis, lay.accel(k, axis), -1)
		}
	}
	off += 4 * (n + 1)

	// Нижняя граница тяги: ‖(δ_k − 1, σ_k/μ₁_k − 1)‖ ≤ σ_k/μ₁_k.
	for k := 0; k <= n; k++ {
		base := off + 3*k
		inv := 0.0
		if mu1[k] > 0 {
			inv = 1 / mu1[k]
		}
		gb.Add(base, lay.sigma(k), -inv)
		h[base] = 0

		// δ_k − 1 = (−z₀_k − 1) − α·Δt·Σ wᵛ σ_i
		for i := 0; i <= n; i++ {
			if wv := w.vel[k][i]; wv != 0 {
				gb.Add(base+1, lay.sigma(i), alpha*dt*wv)
			}
		}
		h[base+1] = -z0[k] - 1

		gb.Add(base+2, lay.sigma(k), -inv)
		h[base+2] = -1
	}
	off += 3 * (n + 1)

	// Посадочный конус — относительно ФАКТИЧЕСКОЙ точки касания r_N, а не
	// относительно цели.
	//
	// Это не мелочь оформления. Если строить конус вокруг цели, то задача
	// минимального промаха (modeMinMiss) становится неразрешимой ровно
	// тогда, когда она нужнее всего: терминальный узел лежит на высоте
	// ноль, конус вокруг цели на нулевой высоте вырождается в точку, и
	// любой ненулевой промах его нарушает — то есть «сядь как можно ближе»
	// превращается в «сядь точно в цель или никак». Конус вокруг самой
	// точки касания выражает то, ради чего он и нужен: подход к площадке
	// сверху, а не по пологой траектории и не из-под поверхности, — и при
	// этом остаётся выполнимым при любом промахе. Разность r_k − r_N так же
	// аффинна по управлению, как и само r_k, так что выпуклость не
	// страдает.
	if glideCount > 0 {
		cot := 1 / math.Tan(s.GlideSlope)
		apex := s.GlideApexDepth / sc.length
		for j := 0; j < glideCount; j++ {
			k := glideFrom + j
			base := off + 3*j
			for i := 0; i <= n; i++ {
				wp := w.pos[k][i] - w.pos[n][i]
				if wp == 0 {
					continue
				}
				gb.Add(base, lay.accel(i, 2), -cot*dt*dt*wp)
				gb.Add(base+1, lay.accel(i, 0), -dt*dt*wp)
				gb.Add(base+2, lay.accel(i, 1), -dt*dt*wp)
			}
			d := posOff[k].Sub(posOff[n])
			h[base] = cot * (d.Z + apex)
			h[base+1] = d.X
			h[base+2] = d.Y
		}
		off += 3 * glideCount
	}

	// Промах: либо целевая переменная (minMiss), либо ограничение сверху.
	if m != modeExactFuel {
		base := off
		if m == modeMinMiss {
			gb.Add(base, lay.miss, -1)
			h[base] = 0
		} else {
			tol := s.MissTolerance
			if tol < 0 {
				tol = 0
			}
			// Бесконечный допуск означает «горизонтальная цель не
			// задана»: конус промаха при таком радиусе не связывает
			// ничего. Записывается конечным большим числом, а не
			// бесконечностью, — бесконечность в матрице ограничений
			// сделала бы задачу численно бессмысленной.
			limit := tol / sc.length
			if math.IsInf(limit, 1) || limit > 1e6 {
				limit = 1e6
			}
			h[base] = limit
		}
		for i := 0; i <= n; i++ {
			wp := w.pos[n][i]
			if wp == 0 {
				continue
			}
			gb.Add(base+1, lay.accel(i, 0), -dt*dt*wp)
			gb.Add(base+2, lay.accel(i, 1), -dt*dt*wp)
		}
		h[base+1] = posOff[n].X - rTarget.X
		h[base+2] = posOff[n].Y - rTarget.Y
	}

	// --- целевая функция.
	c := make([]float64, lay.n)
	switch m {
	case modeMinMiss:
		c[lay.miss] = 1
	default:
		// −z̃_N = α·Δt·Σ wᵛ_{N,i}·σ_i — доля израсходованной массы.
		// Весовые коэффициенты wᵛ_{N,·} и есть правило трапеций.
		for i := 0; i <= n; i++ {
			if wv := w.vel[n][i]; wv != 0 {
				c[lay.sigma(i)] = alpha * dt * wv
			}
		}
	}

	return &socp.Problem{
		C: c, A: ab.Build(), B: bvec,
		G: gb.Build(), H: h, K: cone,
	}, sc, w, z0
}

func component(v physics.Vec3, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}
