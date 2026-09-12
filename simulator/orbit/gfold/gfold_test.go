package gfold

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// Тождества весовых таблиц: при ПОСТОЯННОМ управлении дискретизация обязана
// воспроизводить равноускоренное движение ТОЧНО, а это в точности означает
// Σᵢ wᵛ_{k,i} = k и Σᵢ wʳ_{k,i} = k²/2. Любая ошибка на единицу в индексах
// (а их в этих формулах четыре разных случая) ломает одно из двух тождеств.
func TestWeights_Identities(t *testing.T) {
	const n = 12
	w := newWeights(n)
	for k := 0; k <= n; k++ {
		sv, sp := 0.0, 0.0
		for i := 0; i <= n; i++ {
			sv += w.vel[k][i]
			sp += w.pos[k][i]
		}
		if math.Abs(sv-float64(k)) > 1e-12 {
			t.Fatalf("k=%d: Σwᵛ = %g, ожидалось %d", k, sv, k)
		}
		if want := float64(k*k) / 2; math.Abs(sp-want) > 1e-12 {
			t.Fatalf("k=%d: Σwʳ = %g, ожидалось %g", k, sp, want)
		}
	}
}

// Веса за пределами k обязаны быть нулевыми: управление в будущем узле не
// может влиять на состояние в прошлом.
func TestWeights_Causality(t *testing.T) {
	const n = 8
	w := newWeights(n)
	for k := 0; k <= n; k++ {
		for i := k + 1; i <= n; i++ {
			if w.vel[k][i] != 0 || w.pos[k][i] != 0 {
				t.Fatalf("вес в узле k=%d от будущего управления i=%d ненулевой", k, i)
			}
		}
	}
}

// Аффинное отображение «управление → состояние» обязано совпадать с прямым
// пошаговым интегрированием того же линейно восполненного управления.
// Это проверка ровно того места, где исключение состояний могло бы
// разойтись с динамикой, которую оно якобы представляет.
func TestWeights_MatchStepwisePropagation(t *testing.T) {
	const n = 10
	const dt = 0.7
	w := newWeights(n)

	// Детерминированная, но нетривиальная последовательность управлений.
	u := make([]physics.Vec3, n+1)
	for i := range u {
		f := float64(i)
		u[i] = physics.Vec3{X: math.Sin(f), Y: 0.3 * f, Z: 2 + math.Cos(2*f)}
	}

	r0 := physics.Vec3{X: 10, Y: -5, Z: 2000}
	v0 := physics.Vec3{X: 3, Y: 1, Z: -100}

	// Прямая рекурсия FOH.
	r, v := r0, v0
	stepR := make([]physics.Vec3, n+1)
	stepV := make([]physics.Vec3, n+1)
	stepR[0], stepV[0] = r0, v0
	for k := 0; k < n; k++ {
		r = r.Add(v.Scale(dt)).Add(u[k].Scale(dt * dt / 3)).Add(u[k+1].Scale(dt * dt / 6))
		v = v.Add(u[k].Add(u[k+1]).Scale(dt / 2))
		stepR[k+1], stepV[k+1] = r, v
	}

	// Аффинные формулы.
	for k := 0; k <= n; k++ {
		sumP, sumV := physics.Vec3{}, physics.Vec3{}
		for i := 0; i <= k; i++ {
			sumP = sumP.Add(u[i].Scale(w.pos[k][i]))
			sumV = sumV.Add(u[i].Scale(w.vel[k][i]))
		}
		gotR := r0.Add(v0.Scale(float64(k) * dt)).Add(sumP.Scale(dt * dt))
		gotV := v0.Add(sumV.Scale(dt))
		if d := gotR.Sub(stepR[k]).Norm(); d > 1e-9*(1+stepR[k].Norm()) {
			t.Fatalf("положение в узле %d: аффинно %v, пошагово %v", k, gotR, stepR[k])
		}
		if d := gotV.Sub(stepV[k]).Norm(); d > 1e-9*(1+stepV[k].Norm()) {
			t.Fatalf("скорость в узле %d: аффинно %v, пошагово %v", k, gotV, stepV[k])
		}
	}
}

// superHeavyLanding — постановка по реальным данным Super Heavy на
// трёхкамерной посадочной группе (см. vehicle.StarshipConfig).
func superHeavyLanding(pos, vel physics.Vec3, maxNodes int) Setup {
	const perEngine = 2.56e6
	const engines = 3
	rho1, rho2 := ConstantBounds(perEngine*engines*0.40, perEngine*engines, maxNodes)
	return Setup{
		Vehicle: Vehicle{
			// Масса на розжиге посадочного импульса и сухая масса ступени
			// по vehicle.StarshipConfig: 275 т конструкции плюс остаток
			// топлива. Соотношение существенно: при трёх камерах и этой
			// массе ρ₁/m < g < ρ₂/m, то есть аппарат не может ни зависнуть
			// на нижнем пределе дросселирования, ни разогнаться вверх на
			// верхнем — ровно тот режим, для которого задача поставлена.
			WetMass:        400000,
			DryMass:        275000,
			Alpha:          1 / (321 * physics.G0),
			Rho1:           rho1,
			Rho2:           rho2,
			ThetaMax:       45 * physics.DegToRad,
			PointAxis:      physics.Vec3{Z: 1},
			GlideSlope:     30 * physics.DegToRad,
			GlideSkipNodes: 2,
			GlideApexDepth: 5,
		},
		State:          State{Position: pos, Velocity: vel},
		Gravity:        physics.Vec3{Z: -9.81},
		TargetVelocity: physics.Vec3{Z: -1.5},
		Step:           1.0,
	}
}

// Чисто вертикальная посадка: аппарат обязан прийти в цель с заданной
// скоростью, не выйдя за границы тяги, и потратить физически осмысленное
// количество топлива.
func TestSolve_VerticalLanding(t *testing.T) {
	s := superHeavyLanding(physics.Vec3{Z: 2000}, physics.Vec3{Z: -100}, 60)
	traj := Plan(s, DefaultPlanOptions(6, 60))
	if traj.Status != StatusOptimal {
		t.Fatalf("статус %v", traj.Status)
	}

	n := traj.Nodes
	if got := traj.Position[n]; math.Abs(got.Z) > 1 || math.Hypot(got.X, got.Y) > 1 {
		t.Fatalf("терминальное положение %v, ожидалось начало координат", got)
	}
	if got := traj.Velocity[n]; math.Abs(got.Z+1.5) > 0.5 || math.Hypot(got.X, got.Y) > 0.5 {
		t.Fatalf("терминальная скорость %v, ожидалось (0,0,−1.5)", got)
	}

	// Восстановленная тяга обязана лежать в паспортных границах — ровно то
	// свойство, ради которого границы приближены СТРОЖЕ исходных.
	for k := 0; k <= n; k++ {
		thrust := traj.Accel[k].Norm() * traj.Mass[k]
		if thrust < s.Rho1[k]*0.999 || thrust > s.Rho2[k]*1.001 {
			t.Fatalf("узел %d: тяга %.3f МН вне границ [%.3f, %.3f] МН",
				k, thrust/1e6, s.Rho1[k]/1e6, s.Rho2[k]/1e6)
		}
	}
	if traj.Mass[n] < s.DryMass {
		t.Fatalf("масса в конце %.0f кг ниже допустимой %.0f кг", traj.Mass[n], s.DryMass)
	}
	if traj.FuelUsed <= 0 {
		t.Fatalf("расход топлива %.0f кг", traj.FuelUsed)
	}
}

// Теорема о lossless convexification: в оптимуме ‖u‖ = σ. Проверяется на
// постановке БЕЗ ограничений на состояние (посадочный конус выключен) и с
// полным допустимым углом отклонения тяги — именно тот случай, который
// теорема покрывает без оговорок.
func TestSolve_RelaxationIsLossless(t *testing.T) {
	s := superHeavyLanding(physics.Vec3{X: 300, Z: 1800}, physics.Vec3{X: -20, Z: -90}, 60)
	s.GlideSlope = 0
	s.ThetaMax = math.Pi / 2

	traj := Plan(s, DefaultPlanOptions(6, 60))
	if traj.Status != StatusOptimal {
		t.Fatalf("статус %v", traj.Status)
	}
	if traj.Slack > 1e-4 {
		t.Fatalf("невязка ‖u‖=σ равна %.3g — релаксация перестала быть точной", traj.Slack)
	}
}

// Достижимый боковой увод: цель в стороне, но в пределах располагаемой
// тяги — наведение обязано попасть в неё, а не «примерно туда».
func TestSolve_ReachableDivertHitsTarget(t *testing.T) {
	s := superHeavyLanding(physics.Vec3{X: 800, Y: -400, Z: 3000}, physics.Vec3{X: -30, Z: -120}, 80)
	traj := Plan(s, DefaultPlanOptions(6, 80))
	if traj.Status != StatusOptimal {
		t.Fatalf("статус %v", traj.Status)
	}
	if traj.Miss > 2 {
		t.Fatalf("промах %.1f м при достижимой цели", traj.Miss)
	}
}

// Недостижимая цель: наведение обязано вернуть ЧЕСТНЫЙ минимальный промах,
// а не отказ. Отказ здесь означал бы «не знаю, что делать» и оставил бы
// аппарат без команды на посадочном импульсе; минимальный промах — это
// ответ «ближе сесть физически нельзя, вот куда сядем».
func TestSolve_UnreachableTargetReportsMinimumMiss(t *testing.T) {
	// Цель в сорока километрах в стороне при скромной высоте — заведомо
	// вне досягаемости за время, которое остаётся до касания.
	s := superHeavyLanding(physics.Vec3{X: 40000, Z: 3000}, physics.Vec3{X: -50, Z: -120}, 80)
	traj := Plan(s, DefaultPlanOptions(6, 80))
	if traj.Status != StatusOptimal {
		t.Fatalf("статус %v, ожидалось решение с ненулевым промахом", traj.Status)
	}
	if traj.Miss <= 100 {
		t.Fatalf("промах %.1f м — цель не могла быть достигнута", traj.Miss)
	}
	// Но сесть при этом обязан мягко: промах и жёсткость касания — разные
	// задачи, и вторая не приносится в жертву первой.
	if v := traj.Velocity[traj.Nodes]; math.Abs(v.Z+1.5) > 1 {
		t.Fatalf("вертикальная скорость касания %.2f м/с при промахе", v.Z)
	}
}

// Ограничение на угол отклонения тяги обязано соблюдаться в решении.
func TestSolve_ThrustPointingRespected(t *testing.T) {
	s := superHeavyLanding(physics.Vec3{X: 600, Z: 2500}, physics.Vec3{X: -25, Z: -110}, 60)
	s.ThetaMax = 20 * physics.DegToRad
	traj := Plan(s, DefaultPlanOptions(6, 60))
	if traj.Status != StatusOptimal {
		t.Fatalf("статус %v", traj.Status)
	}
	for k := 0; k <= traj.Nodes; k++ {
		a := traj.Accel[k]
		if a.Norm() < 1e-9 {
			continue
		}
		angle := math.Acos(physics.Clamp(a.Unit().Z, -1, 1))
		if angle > s.ThetaMax+1e-3 {
			t.Fatalf("узел %d: отклонение тяги %.2f°, предел %.2f°",
				k, angle*physics.RadToDeg, s.ThetaMax*physics.RadToDeg)
		}
	}
}

// Посадочный конус обязан соблюдаться там, где он наложен.
func TestSolve_GlideSlopeRespected(t *testing.T) {
	s := superHeavyLanding(physics.Vec3{X: 900, Z: 2500}, physics.Vec3{X: -40, Z: -110}, 60)
	s.GlideSlope = 25 * physics.DegToRad
	traj := Plan(s, DefaultPlanOptions(6, 60))
	if traj.Status != StatusOptimal {
		t.Fatalf("статус %v", traj.Status)
	}
	cot := 1 / math.Tan(s.GlideSlope)
	end := traj.Position[traj.Nodes]
	for k := s.GlideSkipNodes; k <= traj.Nodes; k++ {
		d := traj.Position[k].Sub(end)
		horiz := math.Hypot(d.X, d.Y)
		limit := cot * (d.Z + s.GlideApexDepth)
		if horiz > limit+1 {
			t.Fatalf("узел %d: смещение %.1f м от точки касания при разрешённых %.1f м (превышение высоты %.1f м)",
				k, horiz, limit, d.Z)
		}
	}
}

// Восстановление траектории обязано быть согласовано само с собой: масса в
// узлах должна отвечать проинтегрированному расходу при найденной тяге.
func TestSolve_MassProfileMatchesThrust(t *testing.T) {
	s := superHeavyLanding(physics.Vec3{X: 200, Z: 2000}, physics.Vec3{Z: -100}, 60)
	traj := Plan(s, DefaultPlanOptions(6, 60))
	if traj.Status != StatusOptimal {
		t.Fatalf("статус %v", traj.Status)
	}
	mass := traj.Mass[0]
	for k := 0; k < traj.Nodes; k++ {
		f0 := traj.Sigma[k] * traj.Mass[k]
		f1 := traj.Sigma[k+1] * traj.Mass[k+1]
		mass -= s.Alpha * traj.Step * (f0 + f1) / 2
	}
	if d := math.Abs(mass - traj.Mass[traj.Nodes]); d > 0.005*traj.FuelUsed+1 {
		t.Fatalf("масса в конце %.0f кг, проинтегрированная %.0f кг", traj.Mass[traj.Nodes], mass)
	}
}
