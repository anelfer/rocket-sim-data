package orbit

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Проверка математики PEG: замкнутые формулы L,J,S,Q и решение краевой
// задачи сверяются с независимым численным интегрированием, а не только
// друг с другом — иначе тест мог бы совпасть с ошибочной формулой.
// -----------------------------------------------------------------------------

// simpsonIntegrate — квадратура Симпсона на равномерной сетке из n
// (чётного) отрезков. Нужна как независимый от computePEGMoments способ
// посчитать те же интегралы.
func simpsonIntegrate(f func(float64) float64, a, b float64, n int) float64 {
	if n%2 != 0 {
		n++
	}
	h := (b - a) / float64(n)
	sum := f(a) + f(b)
	for i := 1; i < n; i++ {
		x := a + float64(i)*h
		if i%2 == 0 {
			sum += 2 * f(x)
		} else {
			sum += 4 * f(x)
		}
	}
	return sum * h / 3
}

func TestPEGMomentsMatchQuadrature(t *testing.T) {
	cases := []struct {
		ve, tauPrime, tau float64
	}{
		{3000, 200, 50},
		{4400, 600, 120},
		{3500, 150, 100},
	}

	for _, c := range cases {
		accel := func(s float64) float64 { return c.ve / (c.tauPrime - s) }

		wantL := simpsonIntegrate(accel, 0, c.tau, 200000)
		wantJ := simpsonIntegrate(func(s float64) float64 { return accel(s) * s }, 0, c.tau, 200000)
		wantS := simpsonIntegrate(func(s float64) float64 { return (c.tau - s) * accel(s) }, 0, c.tau, 200000)
		wantQ := simpsonIntegrate(func(s float64) float64 { return (c.tau - s) * accel(s) * s }, 0, c.tau, 200000)

		got := computePEGMoments(c.ve, c.tauPrime, c.tau)

		check := func(name string, got, want float64) {
			t.Helper()
			if diff := math.Abs(got - want); diff > 1e-6*math.Abs(want) {
				t.Errorf("(ve=%.0f τ'=%.0f τ=%.0f) %s = %.6f, квадратура даёт %.6f",
					c.ve, c.tauPrime, c.tau, name, got, want)
			}
		}
		check("L", got.L, wantL)
		check("J", got.J, wantJ)
		check("S", got.S, wantS)
		check("Q", got.Q, wantQ)
	}
}

func TestPegBurnTimeForDeltaVInvertsL(t *testing.T) {
	cases := []struct {
		ve, tauPrime, deltaV float64
	}{
		{3000, 200, 500},
		{4400, 600, 2000},
		{3200, 300, 50},
	}

	for _, c := range cases {
		tau := pegBurnTimeForDeltaV(c.ve, c.tauPrime, c.deltaV)
		got := computePEGMoments(c.ve, c.tauPrime, tau).L
		if diff := math.Abs(got - c.deltaV); diff > 1e-6*c.deltaV {
			t.Errorf("(ve=%.0f τ'=%.0f Δv=%.0f) L(τ)=%.6f, ожидалось %.6f",
				c.ve, c.tauPrime, c.deltaV, got, c.deltaV)
		}
	}
}

// TestPEGSteeringDegenerateIsFlat — вырожденный случай: нет ни гравитации,
// ни ошибки по радиусу/радиальной скорости (цель уже достигнута). Решение
// обязано быть постоянным горизонтальным тангажем (A=B=0).
func TestPEGSteeringDegenerateIsFlat(t *testing.T) {
	m := computePEGMoments(3000, 200, 50)
	a, b, ok := solvePEGSteering(m, 0, 1_000_000, 0, 1_000_000, 0, 50)
	if !ok {
		t.Fatal("solvePEGSteering вернул ok=false на невырожденном τ")
	}
	if math.Abs(a) > 1e-9 || math.Abs(b) > 1e-9 {
		t.Errorf("вырожденный случай: A=%.3e B=%.3e, ожидались оба ≈0", a, b)
	}
}

// TestPEGSteeringSatisfiesBoundaryConditions — небанальный случай: решение
// A,B подставляется обратно в численное интегрирование той же модели
// (мелкий шаг), и проверяется, что оба краевых условия (ṙ(τ)=0, r(τ)=r_T)
// действительно выполняются. Это сквозная проверка формул L,J,S,Q и
// системы 2×2 — не только их взаимной согласованности.
func TestPEGSteeringSatisfiesBoundaryConditions(t *testing.T) {
	const (
		ve       = 3000.0
		tauPrime = 200.0
		tau      = 80.0
		gEff     = 9.5
		rdotNow  = -20.0
		rNow     = 6_950_000.0 - 50_000.0
		deltaR   = 50_000.0
	)
	rTarget := rNow + deltaR

	m := computePEGMoments(ve, tauPrime, tau)
	a, b, ok := solvePEGSteering(m, gEff, rNow, rdotNow, rTarget, 0, tau)
	if !ok {
		t.Fatal("solvePEGSteering вернул ok=false на невырожденном τ")
	}

	// Рунге-Кутта 4-го порядка по состоянию (r, ṙ) — точнее Эйлера на
	// порядки при том же числе шагов, не путает погрешность дискретизации
	// с ошибкой самой формулы.
	accel := func(s float64) float64 { return ve/(tauPrime-s)*(a+b*s) - gEff }
	deriv := func(s float64, y [2]float64) [2]float64 {
		return [2]float64{y[1], accel(s)}
	}

	const steps = 20000
	dt := tau / steps
	y := [2]float64{rNow, rdotNow}
	for i := 0; i < steps; i++ {
		s := float64(i) * dt
		k1 := deriv(s, y)
		y2 := [2]float64{y[0] + dt/2*k1[0], y[1] + dt/2*k1[1]}
		k2 := deriv(s+dt/2, y2)
		y3 := [2]float64{y[0] + dt/2*k2[0], y[1] + dt/2*k2[1]}
		k3 := deriv(s+dt/2, y3)
		y4 := [2]float64{y[0] + dt*k3[0], y[1] + dt*k3[1]}
		k4 := deriv(s+dt, y4)
		y[0] += dt / 6 * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0])
		y[1] += dt / 6 * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
	}
	r, rdot := y[0], y[1]

	if math.Abs(rdot) > 1e-3 {
		t.Errorf("ṙ(τ) = %.6g, ожидался 0", rdot)
	}
	if math.Abs(r-rTarget) > 1.0 {
		t.Errorf("r(τ) = %.3f, ожидался %.3f (Δ=%.3f)", r, rTarget, r-rTarget)
	}
}

// -----------------------------------------------------------------------------
// PEGGuidance.Pitch — отказоустойчивость на вырожденных входах и корректная
// работа сглаживания/латча смены фазы.
// -----------------------------------------------------------------------------

func testGNCConfig() GNCConfig {
	return DefaultGNCConfig(400000, 51.6)
}

func TestPEGGuidancePitchDegenerateInputsFallBackSafely(t *testing.T) {
	cfg := testGNCConfig()

	t.Run("двигатель не работает", func(t *testing.T) {
		var g PEGGuidance
		nav := NavState{
			Phase:    PhaseSecondStage,
			Elements: physics.OrbitalElements{Elliptical: true, ApoapsisAltitude: 350000},
			Mass:     0,
		}
		const prev = 17.3
		got := g.Pitch(nav, cfg, 0.05, prev)
		if got != prev {
			t.Errorf("нерабочий двигатель: ожидалась предыдущая команда %v, получено %v", prev, got)
		}
	})

	t.Run("избыток энергии", func(t *testing.T) {
		var g PEGGuidance
		nav := NavState{
			Phase:    PhaseSecondStage,
			Elements: physics.OrbitalElements{Elliptical: false, ApoapsisAltitude: math.Inf(1)},
		}
		got := g.Pitch(nav, cfg, 0.05, 5)
		if got != cfg.PEGMinPitch {
			t.Errorf("гиперболическая траектория: ожидался PEGMinPitch=%v, получено %v", cfg.PEGMinPitch, got)
		}
	})

	t.Run("почти выработанное топливо", func(t *testing.T) {
		var g PEGGuidance
		vT := physics.CircularOrbitVelocity(cfg.TargetOrbitAltitude)
		nav := NavState{
			Phase:              PhaseSecondStage,
			Position:           physics.Vec3{X: physics.EarthRadius + 300000},
			RadialVelocity:     10,
			TransverseVelocity: vT - 3000, // большой дефицит при малой массе
			Elements:           physics.OrbitalElements{Elliptical: true, ApoapsisAltitude: 350000},
			Mass:               5000,
			AvailableThrust:    2000000,
			ISP:                350,
		}
		got := g.Pitch(nav, cfg, 0.05, 3)
		if math.IsNaN(got) {
			t.Fatal("получен NaN")
		}
		if got < cfg.PEGMinPitch-1e-9 || got > cfg.PEGMaxPitch+1e-9 {
			t.Errorf("тангаж %v вышел за пределы [%v, %v]", got, cfg.PEGMinPitch, cfg.PEGMaxPitch)
		}
	})
}

// TestPEGGuidanceHoldsLastValidBelowMinTgo — когда прогнозируемое время до
// отсечки меньше pegMinTgo, наведение обязано держать последнюю команду,
// а не решать вырождающуюся систему.
func TestPEGGuidanceHoldsLastValidBelowMinTgo(t *testing.T) {
	cfg := testGNCConfig()
	vT := physics.CircularOrbitVelocity(cfg.TargetOrbitAltitude)

	var g PEGGuidance
	nav := NavState{
		Phase:              PhaseSecondStage,
		Position:           physics.Vec3{X: physics.EarthRadius + 300000},
		RadialVelocity:     0,
		TransverseVelocity: vT - 0.5, // дефицит зажимается снизу pegMinDeltaV
		Elements:           physics.OrbitalElements{Elliptical: true, ApoapsisAltitude: 400000},
		Mass:               100000,
		AvailableThrust:    1000000,
		ISP:                350,
	}

	const prev = 12.5
	got := g.Pitch(nav, cfg, 0.05, prev)
	if got != prev {
		t.Errorf("τ < pegMinTgo: ожидалась предыдущая команда %v, получено %v", prev, got)
	}
	if g.PredictedTgo >= pegMinTgo {
		t.Errorf("PredictedTgo=%.3f должен быть меньше pegMinTgo=%.1f в этом сценарии", g.PredictedTgo, pegMinTgo)
	}
}

// TestPEGGuidanceResetsOnPhaseChange — смена фазы не должна протаскивать
// сглаженный прогноз tgo из предыдущей фазы: первый такт новой фазы обязан
// использовать "сырое" значение.
func TestPEGGuidanceResetsOnPhaseChange(t *testing.T) {
	cfg := testGNCConfig()
	vT := physics.CircularOrbitVelocity(cfg.TargetOrbitAltitude)

	base := NavState{
		Position:        physics.Vec3{X: physics.EarthRadius + 300000},
		RadialVelocity:  50,
		Elements:        physics.OrbitalElements{Elliptical: true, ApoapsisAltitude: 350000},
		Mass:            400000,
		AvailableThrust: 5000000,
		ISP:             350,
	}

	var g PEGGuidance

	nav1 := base
	nav1.Phase = PhaseSecondStage
	nav1.TransverseVelocity = vT - 300
	g.Pitch(nav1, cfg, 0.05, 10)
	if !g.initialized {
		t.Fatal("после первого вызова состояние должно быть инициализировано")
	}

	nav2 := base
	nav2.Phase = PhaseCircularization // смена фазы
	nav2.TransverseVelocity = vT - 1500
	g.Pitch(nav2, cfg, 0.05, 10)

	// Эталон — тот же вызов на СВЕЖЕМ состоянии (без истории первой фазы).
	// Если бы латч смены фазы не сработал, g.PredictedTgo оказался бы
	// сглажен в сторону значения из первой фазы и разошёлся бы с эталоном.
	var fresh PEGGuidance
	fresh.lastPhase = PhaseCircularization
	fresh.Pitch(nav2, cfg, 0.05, 10)

	if diff := math.Abs(g.PredictedTgo - fresh.PredictedTgo); diff > 1e-6*fresh.PredictedTgo {
		t.Errorf("после смены фазы τ должно совпасть со «свежим» расчётом (%.4f), "+
			"а не быть сглаженным со старой фазой; получено %.4f", fresh.PredictedTgo, g.PredictedTgo)
	}
}
