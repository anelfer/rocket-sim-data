package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Интегратор: сохранение энергии, порядок сходимости, сопротивление.
// -----------------------------------------------------------------------------

func TestRK4ConservesEnergyOnCoast(t *testing.T) {
	altitude := 400000.0
	r := physics.EarthRadius + altitude
	speed := physics.CircularOrbitVelocity(altitude)

	state := VehicleState{
		Position: physics.Vec3{X: r, Y: 0, Z: 0},
		Velocity: physics.Vec3{X: 0, Y: speed, Z: 0},
		FuelMass: 0,
	}
	// Без тяги и без сопротивления: площадь миделя нулевая.
	fm := ForceModel{DryMass: 10000, Area: 0}

	initial := SpecificEnergy(state)

	// Один полный виток.
	const dt = 1.0
	steps := int(2 * math.Pi * math.Sqrt(r*r*r/physics.GM) / dt)
	for i := 0; i < steps; i++ {
		state = RK4Step(state, fm, dt)
	}

	drift := math.Abs(SpecificEnergy(state)-initial) / math.Abs(initial)
	if drift > 1e-9 {
		t.Errorf("дрейф энергии за виток = %.3g, ожидалось меньше 1e-9", drift)
	}
}

// Круговая орбита должна оставаться круговой, и через период ракета
// возвращается в исходную точку.
//
// Скорость берётся из фактической величины гравитации в этой точке, а не из
// точечной формулы √(GM/r): в поле со сплюснутостью экваториальная гравитация
// на 0.14 % сильнее, и орбита, «круговая» по точечной модели, получилась бы
// эллиптической с размахом высоты около 20 км.

func TestRK4CircularOrbitClosure(t *testing.T) {
	altitude := 400000.0
	r := physics.EarthRadius + altitude

	pos := physics.Vec3{X: r, Y: 0, Z: 0}
	speed := math.Sqrt(r * physics.GravityAcceleration(pos).Norm())

	start := VehicleState{
		Position: pos,
		Velocity: physics.Vec3{X: 0, Y: speed, Z: 0},
	}
	fm := ForceModel{DryMass: 10000, Area: 0}

	period := 2 * math.Pi * r / speed
	// Шаг подбирается так, чтобы виток укладывался в целое число шагов:
	// иначе невязка замыкания определялась бы не точностью метода,
	// а недолётом на неполный шаг.
	const steps = 12000
	dt := period / steps

	state := start
	minAlt, maxAlt := math.Inf(1), math.Inf(-1)
	for i := 0; i < steps; i++ {
		state = RK4Step(state, fm, dt)
		a := state.Altitude()
		minAlt = math.Min(minAlt, a)
		maxAlt = math.Max(maxAlt, a)
	}

	if maxAlt-minAlt > 100 {
		t.Errorf("разброс высоты на круговой орбите = %.1f м, ожидалось меньше 100 м",
			maxAlt-minAlt)
	}

	miss := state.Position.Sub(start.Position).Norm()
	if miss > 1000 {
		t.Errorf("невязка замыкания витка = %.1f м, ожидалось меньше 1000 м", miss)
	}
}

// Шаг интегрирования должен влиять на результат предсказуемо:
// у метода 4-го порядка уменьшение шага вдвое снижает ошибку примерно в 16 раз.

func TestRK4ConvergenceOrder(t *testing.T) {
	r := physics.EarthRadius + 400000
	speed := physics.CircularOrbitVelocity(400000)
	start := VehicleState{
		Position: physics.Vec3{X: r, Y: 0, Z: 0},
		Velocity: physics.Vec3{X: 0, Y: speed, Z: 0},
	}
	fm := ForceModel{DryMass: 10000, Area: 0}

	// Опорное решение с очень мелким шагом.
	propagate := func(dt float64, total float64) VehicleState {
		s := start
		for elapsed := 0.0; elapsed < total-1e-9; elapsed += dt {
			s = RK4Step(s, fm, dt)
		}
		return s
	}

	const horizon = 600.0
	reference := propagate(0.05, horizon)
	coarse := propagate(2.0, horizon)
	fine := propagate(1.0, horizon)

	errCoarse := coarse.Position.Sub(reference.Position).Norm()
	errFine := fine.Position.Sub(reference.Position).Norm()

	if errFine >= errCoarse {
		t.Errorf("уменьшение шага не снизило ошибку: %.4g → %.4g", errCoarse, errFine)
	}
	if errFine > 1e-3 {
		t.Errorf("ошибка при шаге 1 с = %.4g м, ожидалась субмиллиметровая", errFine)
	}
}

// Сопротивление считается по скорости относительно вращающейся атмосферы,
// а не по инерциальной скорости.

func TestDragUsesAirRelativeVelocity(t *testing.T) {
	g := physics.Geodetic{Latitude: 0, Longitude: 0, Altitude: 0}
	pos := physics.ECEFToECI(physics.GeodeticToECEF(g), 0)

	// Ракета неподвижна относительно поверхности: сопротивления быть не должно,
	// хотя инерциальная скорость равна 465 м/с.
	state := VehicleState{
		Position: pos,
		Velocity: physics.CorotatingVelocity(pos),
	}
	fm := ForceModel{DryMass: 1000, Area: 10}
	acc := fm.Evaluate(state)

	if acc.DragForce > 1e-6 {
		t.Errorf("у неподвижной относительно земли ракеты сопротивление = %.4g Н", acc.DragForce)
	}
	if state.Velocity.Norm() < 400 {
		t.Fatal("инерциальная скорость точки на экваторе должна быть около 465 м/с")
	}
}

// -----------------------------------------------------------------------------
// B1: тепловая модель
// -----------------------------------------------------------------------------

// Стенка заглушенного двигателя должна остывать к температуре среды,
// а не разогреваться до 1071 °C, как при перепутанном знаке.

func TestOrbitalDragDecaysOrbit(t *testing.T) {
	altitude := 200000.0
	r := physics.EarthRadius + altitude
	speed := physics.CircularOrbitVelocity(altitude)

	state := VehicleState{
		Position: physics.Vec3{X: r, Y: 0, Z: 0},
		Velocity: physics.Vec3{X: 0, Y: speed, Z: 0},
	}
	fm := ForceModel{DryMass: 5000, Area: 10}

	initial := SpecificEnergy(state)
	for i := 0; i < 20000; i++ {
		state = RK4Step(state, fm, 1.0)
	}
	final := SpecificEnergy(state)

	if final >= initial {
		t.Error("орбита не деградирует: сопротивление на высоте 200 км не учитывается")
	}
	t.Logf("Потеря удельной энергии за 20000 с на высоте 200 км: %.1f Дж/кг",
		initial-final)
}

// -----------------------------------------------------------------------------
// Stage 2: осевое сопротивление зависит от того, какой конец корпуса
// встречает поток (physics.AxialDragCoefficient) — регрессия на прежнюю
// orientation-blind модель (один и тот же DragCoefficient(mach) для 0° и
// 180° угла атаки при одном |cos α|).
// -----------------------------------------------------------------------------

// TestDragArea_EngineFirstExceedsNoseFirst — п.3: при одинаковых Mach и
// |cos α| (только знак угла атаки различается — 10° против 170°) сопротивление
// теперь обязано различаться, и тупой торец (170°, engine-first) обязан
// давать БОЛЬШЕЕ сопротивление, чем нос (10°, nose-first) — не совпадать,
// как было раньше.
func TestDragArea_EngineFirstExceedsNoseFirst(t *testing.T) {
	fm := ForceModel{Area: 60, SideArea: 600}
	const mach = 4.0

	noseFirst := fm.dragArea(mach, 10) // |cos 10°| ≈ |cos 170°|
	engineFirst := fm.dragArea(mach, 170)

	if engineFirst <= noseFirst {
		t.Errorf("engine-first dragArea=%.1f не больше nose-first dragArea=%.1f при |cos α| практически равных",
			engineFirst, noseFirst)
	}
}

// TestDragArea_NoseFirstAscentUnchanged — regression: п.6 "nose-first
// regression" — прямой подъём (малые углы атаки, около 0°) обязан по-прежнему
// использовать ту же кривую, что и раньше (DragCoefficient), Stage 2 не имеет
// права менять сопротивление на активном участке.
func TestDragArea_NoseFirstAscentUnchanged(t *testing.T) {
	fm := ForceModel{Area: 60, SideArea: 600}
	for _, mach := range []float64{0.5, 0.9, 1.0, 1.1, 2, 3, 5} {
		for _, aoa := range []float64{0, 2, 5, 10} {
			got := fm.dragArea(mach, aoa)
			a := aoa * physics.DegToRad
			want := physics.DragCoefficient(mach)*fm.Area*math.Abs(math.Cos(a)) +
				1.4*fm.SideArea*math.Abs(math.Sin(a))
			if math.Abs(got-want) > 1e-9*math.Max(1, want) {
				t.Errorf("mach=%.1f aoa=%.0f: dragArea=%.4f, ожидалось (старая формула)=%.4f", mach, aoa, got, want)
			}
		}
	}
}

// TestDragArea_ContinuousThroughOrientationTransition — п.10/12 "Transition
// test": переход через 90° угла атаки (где выбор ориентации переключается)
// не должен давать скачок силы — осевая проекция площади |cos α| сама
// стремится к нулю на этой границе, гася разницу коэффициентов.
func TestDragArea_ContinuousThroughOrientationTransition(t *testing.T) {
	fm := ForceModel{Area: 60, SideArea: 600}
	const mach = 3.0

	var prev float64
	first := true
	const step = 0.05
	for aoa := 85.0; aoa <= 95.0; aoa += step {
		got := fm.dragArea(mach, aoa)
		if !first && math.Abs(got-prev) > 5 {
			t.Errorf("aoa=%.2f: скачок dragArea %.2f → %.2f при переходе через 90°", aoa, prev, got)
		}
		prev = got
		first = false
	}
}

// TestForceModelDrag_IsAlwaysDissipative — п.13: сила сопротивления не
// должна добавлять энергии — F_D·V_air ≤ 0 — ни для nose-first, ни для
// engine-first, ни при каком проверенном числе Маха.
func TestForceModelDrag_IsAlwaysDissipative(t *testing.T) {
	state := VehicleState{Position: physics.Vec3{X: physics.EarthRadius + 20000}}
	for _, aoa := range []float64{0, 10, 45, 89, 91, 135, 170, 180} {
		fm := ForceModel{DryMass: 500000, Area: 60, SideArea: 600, AngleOfAttack: aoa}
		acc := fm.Evaluate(state)
		vRel := state.AirRelativeVelocityWithWind(physics.Vec3{})
		if dot := acc.Drag.Dot(vRel); dot > 1e-6 {
			t.Errorf("aoa=%.0f: Drag·V_air=%.3e > 0 — сопротивление добавляет энергию", aoa, dot)
		}
	}
}
