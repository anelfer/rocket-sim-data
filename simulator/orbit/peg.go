package orbit

import (
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// PEG (Powered Explicit Guidance) — линейно-тангенциальное наведение
// закрытой формы для разреженного участка второй ступени.
//
// Прежняя схема (ApoapsisPitch + AltitudeHold, два ПИД-регулятора с ручным
// переключением по порогу "arrived") гонялась сначала за высотой апоцентра,
// потом за вертикальной скоростью — то есть по очереди, а не одновременно.
// Именно из-за этой последовательности апоцентр стабильно перелетал цель
// (см. campaign_test.go: knownApoapsisOvershoot) — ступень доразгонялась
// уже после того, как апоцентр пройден, и он успевал уйти на сотню
// километров выше, пока перицентр ещё не дотянул до цели.
//
// PEG решает обе задачи одним законом: закон рулевого угла
//
//	sin θ(t) = A + B·t     (θ — тангаж над местным горизонтом, t=0 — сейчас)
//
// подбирается так, чтобы на предсказанный момент отсечки τ ОДНОВРЕМЕННО
// радиус пришёл к цели (r(τ)=r_T) и радиальная скорость обнулилась
// (ṙ(τ)=0) — апоцентр и требуемая орбитальная скорость приходят к цели
// вместе, а не последовательно. Ниже — вывод и реализация.
//
// -----------------------------------------------------------------------------
// Вывод
//
// При постоянных тяге F и удельном импульсе Isp на прогнозируемом участке
// [0, τ] подстановка
//
//	ve = Isp·g0                (скорость истечения)
//	ṁ  = F/ve                  (расход массы)
//	τ' = m0/ṁ = m0·ve/F        (время полной выработки текущей массы при
//	                             текущем расходе — чисто кинематическая
//	                             константа такта, не прогноз "сколько топлива
//	                             реально осталось")
//	a(t) = F/m(t) = ve/(τ'−t)  (тяговое ускорение)
//
// делает интегралы тяги элементарными:
//
//	L(τ) = ∫₀^τ a(t)dt        = ve·ln(τ'/(τ'−τ))          — формула Циолковского
//	J(τ) = ∫₀^τ a(t)·t dt     = τ'·L − ve·τ
//	S(τ) = ∫₀^τ (τ−t)a(t)dt   = τ·L − J = (τ−τ')·L + ve·τ  — момент для положения
//	Q(τ) = ∫₀^τ (τ−t)a(t)t dt = τ'·S − ve·τ²/2
//
// Формулы проверены численно (метод Симпсона) до 10 значащих цифр.
//
// Трансверсальный канал: тяга у носителя на этом участке почти вдоль
// траектории (θ умеренный), поэтому cos θ(t) ≈ 1 — приближение первого
// порядка, чья ошибка не накапливается, а гасится тем, что весь закон
// пересчитывается заново на следующем такте (тот же принцип, что уже
// применён ко всем ПИД в этом пакете). Тогда набираемая трансверсальная
// скорость ≈ L(τ), и время до отсечки получается обращением L(τ)=ΔV
// в замкнутом виде — тот же приём, что уже в estimateBurnDuration
// (flight_phases.go):
//
//	ΔV_T  = vT − v_transverse,now
//	τ     = τ'·(1 − exp(−ΔV_T/ve))
//
// Радиальный канал — краевая задача на скорость И положение (это и есть
// "explicit" часть: одновременное попадание в r_T и ṙ_T=0, а не только
// в вертикальную скорость). Гравитация берётся замороженной на такте:
//
//	g_eff = GM/r² − v_transverse²/r    (= r̈ неуправляемого радиального
//	                                      движения; при v_transverse→vT,
//	                                      круговая орбита, g_eff→0)
//
//	ṙ(τ) = ṙ_now + A·L + B·J − g_eff·τ        = 0    (Eq. I)
//	r(τ) = r_now + ṙ_now·τ + A·S + B·Q − ½g_eff·τ² = r_T  (Eq. II)
//
// откуда
//
//	A·L + B·J = C1,  C1 = g_eff·τ − ṙ_now
//	A·S + B·Q = C2,  C2 = (r_T − r_now) − ṙ_now·τ + ½g_eff·τ²
//	D = L·Q − J·S
//	A = (C1·Q − C2·J)/D
//	B = (C2·L − C1·S)/D
//	θ(0) = arcsin(clamp(A, −1, 1))
//
// Вырожденный случай (g_eff=0, ṙ_now=0, r_T=r_now — уже на цели по высоте
// и скорости) даёт C1=C2=0 ⇒ A=B=0 ⇒ θ=0: постоянный горизонтальный
// тангаж, как и должно быть. Небанальный случай проверен подстановкой
// решения обратно в численное интегрирование Eq. I/Eq. II — оба условия
// выполняются с точностью ~1e-6.
// -----------------------------------------------------------------------------

const (
	// pegTgoFilterTime — постоянная времени сглаживания прогноза времени до
	// отсечки (τ), с.
	//
	// nav.TransverseVelocity/RadialVelocity на этой фазе приходят с
	// зашумлённого датчика (sensedNavState, постоянная времени сенсора
	// 0.3 с) — без фильтра шум напрямую тряс бы τ, а через него —
	// определитель D и саму команду тангажа. Тот же приём, что
	// DerivativeFilterTime в pid.go и filteredResidual в attitude_dynamics.go:
	// фильтруется именно то звено, где чувствительность к шуму наибольшая.
	pegTgoFilterTime = 1.5

	// pegMinTgo — нижняя граница τ, с. Определитель системы D=LQ−JS убывает
	// как τ⁴ при τ→0 (следует из разложения L,J,S,Q в ряд по малым τ), и
	// ниже этого порога решение для A,B численно неустойчиво. Вместо того
	// чтобы решать вырожденную систему, держим последнюю выданную команду —
	// тот же принцип hold-last-valid, что уже применён к показаниям
	// датчиков при пропуске связи (simulation.go, sensor.go).
	pegMinTgo = 3.0

	// pegMaxBurnFraction — верхний предел τ относительно τ' (полной
	// выработки массы при текущем расходе). L(τ) имеет полюс при τ→τ' —
	// нельзя просить времени до отсечки больше, чем физически можно
	// прогореть. Тот же порядок запаса, что уже принят в проекте
	// (shouldSECO: cutoff*=0.97; старый vacuumAscentPitch: apoapsis>=target*0.98).
	pegMaxBurnFraction = 0.98

	// pegMinDeltaV — нижний порог требуемого трансверсального приращения
	// скорости, м/с. Шум датчика или переходный процесс могут дать
	// ΔV_T≤0 (скорость как будто уже на цели или выше) — не даём τ уйти
	// в ноль или отрицательное значение через этот канал.
	pegMinDeltaV = 1.0
)

// pegMoments — замкнутые интегралы тягового ускорения при постоянном
// расходе на участке [0, τ]. L,J — скоростной канал, S,Q — канал положения
// (без них PEG не отличался бы от регулятора по одной вертикальной скорости).
type pegMoments struct {
	L, J, S, Q float64
}

// computePEGMoments считает L,J,S,Q для a(t)=ve/(τ'−t) на участке [0,τ].
// τ должно быть строго меньше τ' — иначе a(t) уходит в полюс; вызывающая
// сторона обязана заранее ограничить τ (см. pegMaxBurnFraction).
func computePEGMoments(ve, tauPrime, tau float64) pegMoments {
	l := ve * math.Log(tauPrime/(tauPrime-tau))
	j := tauPrime*l - ve*tau
	s := tau*l - j
	q := tauPrime*s - ve*tau*tau/2
	return pegMoments{L: l, J: j, S: s, Q: q}
}

// pegBurnTimeForDeltaV обращает L(τ)=Δv: τ = τ'·(1−exp(−Δv/ve)).
//
// Тот же вид, что формула Циолковского в estimateBurnDuration
// (flight_phases.go) — время горения на заданное приращение скорости при
// текущем расходе. Используется только как первое приближение τ перед
// итерацией Ньютона (см. Pitch) — предполагает cosθ≈1, что верно лишь
// для малых углов.
func pegBurnTimeForDeltaV(ve, tauPrime, deltaV float64) float64 {
	return tauPrime * (1 - math.Exp(-deltaV/ve))
}

// pegActualTransverseGain считает ДЕЙСТВИТЕЛЬНО набираемую трансверсальную
// скорость на участке [0,τ] для уже решённого закона sinθ(t)=A+Bt.
//
// В отличие от L(τ) (которая предполагает всю тягу вдоль трансверсали,
// cosθ≈1) здесь учитывается настоящий cosθ(t)=√(1−sin²θ(t)) — интеграл
// ∫a(t)cosθ(t)dt в замкнутом виде не берётся, считается квадратурой
// Симпсона. Число узлов фиксировано и невелико: подынтегральная функция
// гладкая, а вызывается это внутри итерации Ньютона на каждом такте
// наведения — точность нужна умеренная, а не предельная.
func pegActualTransverseGain(ve, tauPrime, a, b, tau float64) float64 {
	const steps = 40
	f := func(t float64) float64 {
		sinTheta := physics.Clamp(a+b*t, -1, 1)
		cosTheta := math.Sqrt(1 - sinTheta*sinTheta)
		return (ve / (tauPrime - t)) * cosTheta
	}
	return simpsonQuadrature(f, 0, tau, steps)
}

// simpsonQuadrature — квадратура Симпсона на равномерной сетке из n
// (чётного) отрезков.
func simpsonQuadrature(f func(float64) float64, lo, hi float64, n int) float64 {
	if n%2 != 0 {
		n++
	}
	h := (hi - lo) / float64(n)
	sum := f(lo) + f(hi)
	for i := 1; i < n; i++ {
		x := lo + float64(i)*h
		if i%2 == 0 {
			sum += 2 * f(x)
		} else {
			sum += 4 * f(x)
		}
	}
	return sum * h / 3
}

// solvePEGSteering решает систему 2×2 относительно коэффициентов A,B
// закона sin θ(t)=A+Bt по краевым условиям радиального канала:
// ṙ(τ)=rdotTarget, r(τ)=rTarget.
//
// ok=false, если определитель системы пренебрежимо мал (τ слишком близко
// к нулю) — вызывающая сторона обязана в этом случае не использовать A,B.
func solvePEGSteering(m pegMoments, gEff, rNow, rdotNow, rTarget, rdotTarget, tau float64) (a, b float64, ok bool) {
	c1 := gEff*tau - rdotNow + rdotTarget
	c2 := (rTarget - rNow) - rdotNow*tau + 0.5*gEff*tau*tau

	d := m.L*m.Q - m.J*m.S
	if math.Abs(d) < 1e-9 {
		return 0, 0, false
	}

	a = (c1*m.Q - c2*m.J) / d
	b = (c2*m.L - c1*m.S) / d
	return a, b, true
}

// PEGGuidance — персистентное состояние explicit guidance между тактами.
//
// В отличие от PIDController здесь не накапливается ошибка — состояние
// нужно только для сглаживания предсказанного времени до отсечки τ по
// тактам (защита от шума датчика скорости) и для самостоятельного
// обнаружения входа в новую фазу полёта.
type PEGGuidance struct {
	// PredictedTgo — сглаженная оценка времени до отсечки, с. Экспортирована
	// для диагностики (см. GNCSystem.PredictedTgo).
	PredictedTgo float64

	lastPhase   FlightPhase
	initialized bool
}

// Reset возвращает состояние наведения в исходное.
func (p *PEGGuidance) Reset() {
	*p = PEGGuidance{}
}

// Pitch считает тангаж на текущем такте по PEG и обновляет внутреннее
// состояние (PredictedTgo).
//
// previousPitch — последняя выданная команда тангажа (g.Attitude.Pitch у
// вызывающего GNCSystem). Используется как отказоустойчивый выход при
// вырожденных входных данных — hold-last-valid, тот же принцип, что уже
// применён к показаниям датчиков при пропуске связи.
func (p *PEGGuidance) Pitch(nav NavState, cfg GNCConfig, dt, previousPitch float64) float64 {
	// Самообнаружение смены фазы — без внешнего вызова Reset(), по образцу
	// латча ascending в GNCSystem.
	if nav.Phase != p.lastPhase {
		p.Reset()
		p.lastPhase = nav.Phase
	}

	apoapsis := nav.Elements.ApoapsisAltitude

	// Избыток энергии / гиперболическая траектория — модель PEG (целевая
	// круговая скорость, целевой радиус) на это не рассчитана. Тот же
	// guard, что был в прежней схеме.
	if math.IsInf(apoapsis, 1) || !nav.Elements.Elliptical {
		return cfg.PEGMinPitch
	}

	// Двигатель не работает или нет входных данных — наводить нечем.
	if nav.Mass <= 0 || nav.AvailableThrust <= 0 || nav.ISP <= 0 {
		return previousPitch
	}

	ve := nav.ISP * physics.G0
	tauPrime := nav.Mass * ve / nav.AvailableThrust

	vT := physics.CircularOrbitVelocity(cfg.TargetOrbitAltitude)
	deltaVT := vT - nav.TransverseVelocity
	if deltaVT < pegMinDeltaV {
		deltaVT = pegMinDeltaV
	}

	rNow := nav.Position.Norm()
	rTarget := physics.EarthRadius + cfg.TargetOrbitAltitude
	gEff := physics.GM/(rNow*rNow) - nav.TransverseVelocity*nav.TransverseVelocity/rNow

	clampTau := func(t float64) float64 {
		if maxTau := tauPrime * pegMaxBurnFraction; t > maxTau {
			t = maxTau
		}
		if t < pegMinTgo/2 {
			t = pegMinTgo / 2
		}
		return t
	}

	// Совместное решение τ и (A,B) методом Ньютона.
	//
	// L(τ)=ΔV_T (обращение формулы Циолковского) предполагает, что вся тяга
	// идёт вдоль трансверсали (cosθ≈1) — верно только для малых углов. На
	// этом участке тангаж сразу после разделения ступеней держится в районе
	// 30-40°+ заметную часть горения, а cos 42° ≈ 0.74: без поправки
	// прогноз времени до отсечки систематически занижен, набор
	// трансверсальной скорости отстаёт от роста (осциллирующего) апоцентра,
	// и SECO — снаружи, по апоцентру — срабатывает на глубоко эллиптической
	// орбите, которую последующее довыведение уже не может исправить.
	// Один проход с усреднённой поправкой cosθ (как раньше) недостаточен:
	// A,B сами получены на неверном τ и дают plохую оценку. Поэтому здесь —
	// настоящая итерация: на каждом шаге решаем (A,B) точно для текущего τ,
	// затем численно (квадратурой) считаем ДЕЙСТВИТЕЛЬНО набираемую
	// трансверсальную скорость с учётом настоящего cosθ(t), и правим τ
	// шагом Ньютона по невязке. Стартовая точка — прогноз предыдущего такта
	// (если есть), иначе грубая оценка с cosθ≈1.
	tau := pegBurnTimeForDeltaV(ve, tauPrime, deltaVT)
	if p.initialized {
		tau = p.PredictedTgo
	}
	tau = clampTau(tau)

	var a, b float64
	solved := false
	const (
		maxPegIterations = 6
		pegDeltaVTol     = 0.1 // м/с — сходимость по невязке трансверсальной скорости
	)
	for iter := 0; iter < maxPegIterations; iter++ {
		m := computePEGMoments(ve, tauPrime, tau)
		var ok bool
		a, b, ok = solvePEGSteering(m, gEff, rNow, nav.RadialVelocity, rTarget, 0, tau)
		if !ok {
			solved = false
			break
		}
		solved = true

		gained := pegActualTransverseGain(ve, tauPrime, a, b, tau)
		errDV := gained - deltaVT
		if math.Abs(errDV) < pegDeltaVTol {
			break
		}

		thetaTau := math.Asin(physics.Clamp(a+b*tau, -1, 1))
		marginal := (ve / (tauPrime - tau)) * math.Cos(thetaTau)
		if marginal < 1e-3 {
			// Почти вертикальный тангаж на конце участка — предельный вклад
			// в трансверсальную скорость исчезающе мал, шаг Ньютона
			// вырождается (деление на маленькое число). Раздвигаем грубо,
			// а не проваливаемся в NaN/Inf.
			if errDV > 0 {
				tau *= 0.9
			} else {
				tau *= 1.1
			}
		} else {
			tau -= errDV / marginal
		}
		tau = clampTau(tau)
	}
	if !solved {
		return previousPitch
	}

	// Сглаживание τ по тактам — уже сошедшееся значение, а не промежуточные
	// итерации. nav.TransverseVelocity/RadialVelocity приходят с
	// зашумлённого датчика (см. sensedNavState), и без фильтра шум
	// транслировался бы прямо в команду.
	if p.initialized && dt > 0 {
		decay := math.Exp(-dt / pegTgoFilterTime)
		tau = tau + (p.PredictedTgo-tau)*decay
		tau = clampTau(tau)
	}
	p.PredictedTgo = tau
	p.initialized = true

	if tau < pegMinTgo {
		return previousPitch
	}

	// Решаем ещё раз на уже сглаженном τ — иначе PredictedTgo (то, чем
	// распоряжается остальной борт) и τ, из которого получена фактическая
	// команда, разошлись бы.
	m := computePEGMoments(ve, tauPrime, tau)
	a, _, ok := solvePEGSteering(m, gEff, rNow, nav.RadialVelocity, rTarget, 0, tau)
	if !ok {
		return previousPitch
	}

	pitchDeg := math.Asin(physics.Clamp(a, -1, 1)) * physics.RadToDeg
	return physics.Clamp(pitchDeg, cfg.PEGMinPitch, cfg.PEGMaxPitch)
}
