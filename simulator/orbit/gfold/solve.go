package gfold

import (
	"math"

	"rocketTelemetrySim/simulator/orbit/socp"
	"rocketTelemetrySim/simulator/physics"
)

// Solve решает задачу при ФИКСИРОВАННОМ времени полёта (Setup.Nodes·Step),
// по приоритетной схеме Blackmore—Açıkmeşe—Scharf:
//
//  1. минимум топлива при точном попадании в цель;
//  2. если точное попадание недостижимо — минимум промаха;
//  3. затем минимум топлива при промахе не хуже найденного.
//
// Порядок именно такой, а не «сразу минимум промаха»: при возврате на
// площадку цель, как правило, достижима, и тогда шаги 2-3 не нужны вовсе —
// а лишний вызов решателя на каждом такте наведения стоит ровно столько же,
// сколько нужный.
func Solve(s Setup) Trajectory {
	switch s.Objective {
	case ObjectiveReachTarget:
		return solveMode(s, modeExactFuel)
	case ObjectiveSoftLanding:
		// Горизонтальная цель не задаётся: допуск на промах заведомо
		// больше любого достижимого за время полёта смещения, так что
		// ограничение на промах не связывает, и остаётся чистый минимум
		// топлива при мягком касании.
		free := s
		free.MissTolerance = math.Inf(1)
		return solveMode(free, modeBoundedFuel)
	}

	if t := solveMode(s, modeExactFuel); t.Status == StatusOptimal {
		return t
	}

	best := solveMode(s, modeMinMiss)
	if best.Status != StatusOptimal {
		return best
	}

	// Допуск на промах: найденное значение плюс небольшая добавка. Без неё
	// вторая задача сплошь и рядом оказывается неразрешимой «на волосок» —
	// ограничение промаха ровно по достигнутому оптимуму не выполнимо
	// численно, потому что сам оптимум известен лишь с точностью решателя.
	relaxed := s
	relaxed.MissTolerance = best.Miss + math.Max(0.01*best.Miss, 0.5)
	if cheap := solveMode(relaxed, modeBoundedFuel); cheap.Status == StatusOptimal {
		return cheap
	}
	return best
}

// solveMode собирает и решает одну задачу выбранного вида.
func solveMode(s Setup, m mode) Trajectory {
	if s.Nodes < 1 || s.Step <= 0 || len(s.Rho1) <= s.Nodes || len(s.Rho2) <= s.Nodes {
		return Trajectory{Status: StatusFailed}
	}
	p, sc, w, z0 := build(s, m)
	sol := socp.Solve(p, socp.DefaultOptions())

	switch sol.Status {
	case socp.StatusOptimal:
	case socp.StatusInfeasible, socp.StatusUnbounded:
		return Trajectory{Status: StatusInfeasible, Iterations: sol.Iterations}
	default:
		return Trajectory{Status: StatusFailed, Iterations: sol.Iterations}
	}
	return reconstruct(s, m, sc, w, z0, sol)
}

// reconstruct переводит решение конической задачи обратно в физические
// величины и восстанавливает опорную траекторию.
func reconstruct(s Setup, m mode, sc scales, w weights, z0 []float64, sol socp.Solution) Trajectory {
	n := s.Nodes
	lay := newLayout(n, m)
	dt := s.Step / sc.time
	alpha := s.Alpha * sc.length / sc.time
	posOff, velOff := affineOffsets(s, sc, w)

	t := Trajectory{
		Status:     StatusOptimal,
		Step:       s.Step,
		Nodes:      n,
		Accel:      make([]physics.Vec3, n+1),
		Sigma:      make([]float64, n+1),
		Position:   make([]physics.Vec3, n+1),
		Velocity:   make([]physics.Vec3, n+1),
		Mass:       make([]float64, n+1),
		Iterations: sol.Iterations,
	}

	u := make([]physics.Vec3, n+1)
	for k := 0; k <= n; k++ {
		u[k] = physics.Vec3{
			X: sol.X[lay.accel(k, 0)],
			Y: sol.X[lay.accel(k, 1)],
			Z: sol.X[lay.accel(k, 2)],
		}
		t.Accel[k] = u[k].Scale(sc.accel)
		t.Sigma[k] = sol.X[lay.sigma(k)] * sc.accel
	}

	for k := 0; k <= n; k++ {
		sumP, sumV := physics.Vec3{}, physics.Vec3{}
		sumZ := 0.0
		for i := 0; i <= k; i++ {
			if wp := w.pos[k][i]; wp != 0 {
				sumP = sumP.Add(u[i].Scale(wp))
			}
			if wv := w.vel[k][i]; wv != 0 {
				sumV = sumV.Add(u[i].Scale(wv))
				sumZ += wv * sol.X[lay.sigma(i)]
			}
		}
		t.Position[k] = posOff[k].Add(sumP.Scale(dt * dt)).Scale(sc.length)
		t.Velocity[k] = velOff[k].Add(sumV.Scale(dt)).Scale(sc.length / sc.time)
		t.Mass[k] = sc.mass * math.Exp(-alpha*dt*sumZ)
	}

	// Невязка теоремы о lossless convexification: в точном решении
	// σ_k = ‖u_k‖ тождественно. Меряется всегда и публикуется наружу —
	// это единственный способ увидеть, что какое-то из ограничений вывело
	// задачу за пределы доказанного (см. Trajectory.Slack).
	for k := 0; k <= n; k++ {
		norm := t.Accel[k].Norm()
		den := math.Max(t.Sigma[k], 1e-9)
		if d := (t.Sigma[k] - norm) / den; d > t.Slack {
			t.Slack = d
		}
	}

	recoverExecutable(&t, s, z0)
	return t
}

// recoverExecutable превращает решение РЕЛАКСИРОВАННОЙ задачи в план,
// который ступень действительно способна отработать, и заново прогоняет по
// нему траекторию.
//
// Зачем это нужно. Теорема о lossless convexification — утверждение про
// НЕПРЕРЫВНУЮ задачу: в её оптимуме ‖u‖=σ всюду, кроме множества меры нуль.
// В дискретизованной задаче «множество меры нуль» — это отдельный узел, и
// на нём равенство может не выполняться, потому что величина σ входит в
// целевую функцию, а u — только в ограничения: если нижняя граница тяги уже
// прижала σ_k к упору, уменьшение ‖u_k‖ ничего не стоит, зато меняет
// траекторию в выгодную сторону. Наблюдается это, как правило, ровно в
// НУЛЕВОМ узле — то есть в той самой команде, которую наведение и отдаёт
// на исполнение прямо сейчас (измерено: тяга 2.35 МН при физическом
// минимуме группы 3.07 МН).
//
// Отдать такую команду нельзя: камера не работает ниже своего предела
// дросселирования, и двигательная установка всё равно поднимет тягу до
// него (см. propulsion.Engine — уставка ниже MinThrottle удерживается на
// пределе, а не гасит камеру). То есть аппарат полетит НЕ по той
// траектории, которую наведение посчитало, и разойдётся с ней тем сильнее,
// чем дольше. Поэтому модуль тяги здесь загоняется в физические границы —
// именно модуль, направление сохраняется, как и рекомендуют реализации
// метода, — а состояния пересчитываются заново по УЖЕ ИСПРАВЛЕННОМУ
// управлению. После этого Position/Velocity/Mass описывают то, что
// действительно произойдёт при исполнении плана, а не то, что было бы,
// если бы двигатель умел невозможное; промах и расход считаются по ним же.
//
// Сама невязка при этом не прячется: Trajectory.Slack измерен ДО поправки
// и уходит в телеметрию как есть.
func recoverExecutable(t *Trajectory, s Setup, z0 []float64) {
	n := t.Nodes
	dt := t.Step

	// Проход 1: модуль ускорения — в границы, посчитанные по массе из
	// решения (масса на этом шаге ещё плановая; поправка второго порядка,
	// и следующий проход её уточняет).
	for k := 0; k <= n; k++ {
		mass := t.Mass[k]
		if mass <= 0 {
			continue
		}
		lo, hi := s.Rho1[k]/mass, s.Rho2[k]/mass
		norm := t.Accel[k].Norm()
		switch {
		case norm < 1e-9:
			// Направления нет вовсе — держим тягу вдоль опорной оси:
			// выключить камеру наведение не может, а произвольное
			// направление было бы хуже вертикали.
			axis := s.PointAxis
			if axis.Norm() < 1e-12 {
				axis = physics.Vec3{Z: 1}
			}
			t.Accel[k] = axis.Unit().Scale(lo)
		case norm < lo:
			t.Accel[k] = t.Accel[k].Scale(lo / norm)
		case norm > hi:
			t.Accel[k] = t.Accel[k].Scale(hi / norm)
		}
		t.Sigma[k] = t.Accel[k].Norm()
	}

	// Проход 2: масса по фактическому расходу и состояния по фактическому
	// управлению. Масса — из d(ln m)/dt = −α·σ (тяга равна m·σ, поэтому
	// логарифм массы линеен по σ и рекурсия явная), состояния — тем же
	// линейным восполнением, на котором задача и решалась.
	mass := t.Mass[0]
	pos, vel := t.Position[0], t.Velocity[0]
	for k := 0; k <= n; k++ {
		t.Mass[k] = mass
		t.Position[k] = pos
		t.Velocity[k] = vel
		if k == n {
			break
		}
		b0 := s.Gravity
		b1 := s.Gravity
		if s.Bias != nil && k+1 < len(s.Bias) {
			b0 = b0.Add(s.Bias[k])
			b1 = b1.Add(s.Bias[k+1])
		}
		a0 := t.Accel[k].Add(b0)
		a1 := t.Accel[k+1].Add(b1)
		pos = pos.Add(vel.Scale(dt)).Add(a0.Scale(dt * dt / 3)).Add(a1.Scale(dt * dt / 6))
		vel = vel.Add(a0.Add(a1).Scale(dt / 2))
		mass *= math.Exp(-s.Alpha * dt * (t.Sigma[k] + t.Sigma[k+1]) / 2)
	}

	d := t.Position[n].Sub(s.TargetPosition)
	t.Miss = math.Hypot(d.X, d.Y)
	t.FuelUsed = t.Mass[0] - t.Mass[n]
	_ = z0
}

// PlanOptions — параметры перебора по времени полёта.
type PlanOptions struct {
	// MinNodes, MaxNodes — границы перебора по числу интервалов сетки.
	// Шаг сетки при этом ФИКСИРОВАН (Setup.Step), а меняется число узлов:
	// иначе каждое пробное время полёта считалось бы со своей ошибкой
	// дискретизации, и перебор сравнивал бы между собой величины разной
	// точности — самая частая причина, по которой поиск по времени полёта
	// сходится не туда.
	MinNodes, MaxNodes int

	// Coarse — сколько точек грубого просмотра. Аналитические границы
	// времени полёта необходимы, но не достаточны: оба конца отрезка почти
	// всегда неразрешимы, и золотое сечение, запущенное сразу, увидело бы
	// только плато «неразрешимо» и остановилось бы на нём.
	Coarse int

	// Refine — сколько шагов уточнения золотым сечением после грубого
	// просмотра.
	Refine int
}

// DefaultPlanOptions — перебор, с которым наведение работает в полёте.
func DefaultPlanOptions(minNodes, maxNodes int) PlanOptions {
	return PlanOptions{MinNodes: minNodes, MaxNodes: maxNodes, Coarse: 6, Refine: 6}
}

// Plan подбирает время полёта и возвращает лучшую найденную траекторию.
//
// Критерий сравнения лексикографический, тот же, что и внутри одного
// решения: траектория, попадающая в цель, всегда лучше промахивающейся,
// сколько бы топлива та ни экономила; среди попадающих — та, что дешевле.
// Сравнивать их взвешенной суммой было бы неверно по той же причине, по
// которой Solve не сводит две задачи в одну.
func Plan(s Setup, opt PlanOptions) Trajectory {
	lo, hi := opt.MinNodes, opt.MaxNodes
	if lo < 2 {
		lo = 2
	}
	if hi < lo {
		return Trajectory{Status: StatusInfeasible}
	}

	cache := map[int]Trajectory{}
	eval := func(n int) Trajectory {
		if t, ok := cache[n]; ok {
			return t
		}
		trial := s
		trial.Nodes = n
		t := Solve(trial)
		cache[n] = t
		return t
	}

	best := Trajectory{Status: StatusInfeasible}
	bestN := -1
	better := func(a, b Trajectory) bool {
		if b.Status != StatusOptimal {
			return false
		}
		if a.Status != StatusOptimal {
			return true
		}
		// Промах важнее топлива, но различать промахи ниже метра
		// бессмысленно: это заведомо ниже точности самой постановки
		// (сетка, замороженное сопротивление, шум навигации).
		if math.Abs(a.Miss-b.Miss) > 1 {
			return b.Miss < a.Miss
		}
		return b.FuelUsed < a.FuelUsed
	}

	coarse := opt.Coarse
	if coarse < 2 {
		coarse = 2
	}
	for i := 0; i < coarse; i++ {
		n := lo
		if coarse > 1 {
			n = lo + (hi-lo)*i/(coarse-1)
		}
		if t := eval(n); better(best, t) {
			best, bestN = t, n
		}
	}
	if bestN < 0 {
		return Trajectory{Status: StatusInfeasible}
	}

	// Уточнение золотым сечением внутри отрезка между соседними точками
	// грубого просмотра — там, где найден минимум.
	span := (hi - lo) / coarse
	if span < 1 {
		span = 1
	}
	a, b := bestN-span, bestN+span
	if a < lo {
		a = lo
	}
	if b > hi {
		b = hi
	}
	const invPhi = 0.6180339887498949
	for i := 0; i < opt.Refine && b-a > 2; i++ {
		c := b - int(math.Round(float64(b-a)*invPhi))
		d := a + int(math.Round(float64(b-a)*invPhi))
		if c == d {
			d = c + 1
		}
		if d > b {
			d = b
		}
		tc, td := eval(c), eval(d)
		if better(best, tc) {
			best, bestN = tc, c
		}
		if better(best, td) {
			best, bestN = td, d
		}
		if better(td, tc) {
			b = d
		} else {
			a = c
		}
	}

	// Соседи найденного минимума: сетка целочисленная, и после сечения
	// оптимум может оказаться на соседнем узле — проверить это дешевле,
	// чем оставить необъяснимую единичную погрешность по времени полёта.
	for _, n := range []int{bestN - 1, bestN + 1} {
		if n < lo || n > hi {
			continue
		}
		if t := eval(n); better(best, t) {
			best, bestN = t, n
		}
	}
	return best
}
