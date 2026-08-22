package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/env"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Отработавшая ступень: вход в атмосферу, разрушение, обломки.
// -----------------------------------------------------------------------------

func TestTelemetryDropsSpentStageEngines(t *testing.T) {
	sim, _ := runUntilPhase(t, orbit.PhaseSecondStage, 400)

	tel := sim.Snapshot()
	if len(tel.Engines) != 1 {
		t.Fatalf("после разделения ожидался 1 двигатель, получено %d", len(tel.Engines))
	}
	if tel.Engines[0].ID != "S2-1" {
		t.Errorf("идентификатор двигателя = %q, ожидался S2-1", tel.Engines[0].ID)
	}
}

// -----------------------------------------------------------------------------
// Отработавшая первая ступень
// -----------------------------------------------------------------------------

func TestSpentStageSeparatesAndFlies(t *testing.T) {
	sim, _ := runUntilPhase(t, orbit.PhaseSecondStage, 400)

	tel := sim.Snapshot()
	if tel.SpentStage == nil {
		t.Fatal("после разделения должна появиться отработавшая ступень")
	}
	if tel.SpentStage.Outcome != env.StageFlying.String() {
		t.Errorf("сразу после разделения ступень должна лететь, получено %q",
			tel.SpentStage.Outcome)
	}
	if tel.SpentStage.Altitude <= 0 {
		t.Errorf("высота отработавшей ступени = %.1f м", tel.SpentStage.Altitude)
	}
}

// Ступень обязана прекратить существование: разрушиться в атмосфере или упасть.
// Раньше она просто исчезала, а её метрики застывали на последних значениях.

func TestSpentStageReentersAndIsDestroyed(t *testing.T) {
	sim, tel := runAscent(t, 2000)
	if tel.SpentStage == nil {
		t.Fatal("отработавшая ступень не создана")
	}

	if tel.SpentStage.Outcome == env.StageFlying.String() {
		t.Errorf("за %.0f с ступень должна была войти в атмосферу, состояние %q",
			tel.Time, tel.SpentStage.Outcome)
	}

	if tel.SpentStage.PeakDynamicQ < 10000 {
		t.Errorf("максимальный напор при входе = %.1f кПа, ожидался заметный",
			tel.SpentStage.PeakDynamicQ/1000)
	}
	if tel.SpentStage.MaxAltitude < sim.Config.MECOMinAltitude {
		t.Errorf("апогей ступени после разделения = %.1f км, ожидался выше высоты MECO",
			tel.SpentStage.MaxAltitude/1000)
	}

	t.Logf("Первая ступень: %s на T+%.0f с, апогей %.1f км, пиковый напор %.1f кПа, поток %.1f кВт/м²",
		tel.SpentStage.Outcome, tel.SpentStage.OutcomeTime,
		tel.SpentStage.MaxAltitude/1000, tel.SpentStage.PeakDynamicQ/1000,
		tel.SpentStage.PeakHeatFlux/1000)
}

// Пустая ступень входит в атмосферу без теплозащиты и разрушается
// от аэродинамических нагрузок раньше, чем успевает прогреться.

func TestSpentStageBreaksUpUnderLoad(t *testing.T) {
	// Ступень на 100 км с орбитальной по величине скоростью, направленной вниз
	// под углом, — заведомо жёсткий вход.
	altitude := 100000.0
	r := physics.EarthRadius + altitude
	pos := physics.Vec3{X: r, Y: 0, Z: 0}
	frame := physics.NewLocalFrame(pos)
	vel := frame.Direction(-30, 90).Scale(3000)

	stage := env.NewSpentStage("test", pos, vel, 25600, 42.6, 1.83, 300, 0,
		physics.Vec3{}, env.CalmWind())

	for i := 0; i < 20000 && stage.Alive(); i++ {
		stage.Step(0.1, float64(i)*0.1)
	}

	if stage.Alive() {
		t.Fatalf("ступень не разрушилась: высота %.1f км", stage.Altitude()/1000)
	}
	if stage.Outcome != env.StageBrokenUp {
		t.Errorf("исход = %v, ожидалось разрушение в атмосфере", stage.Outcome)
	}
	if stage.Altitude() < 10000 {
		t.Errorf("разрушение на высоте %.1f км, ожидалось выше 10 км", stage.Altitude()/1000)
	}
}

// Медленно падающая ступень должна долететь до поверхности целой.

func TestSpentStageImpactsWhenSlow(t *testing.T) {
	altitude := 3000.0
	r := physics.EarthRadius + altitude
	pos := physics.Vec3{X: r, Y: 0, Z: 0}

	stage := env.NewSpentStage("test", pos, physics.CorotatingVelocity(pos),
		25600, 42.6, 1.83, 300, 0, physics.Vec3{}, env.CalmWind())

	for i := 0; i < 20000 && stage.Alive(); i++ {
		stage.Step(0.1, float64(i)*0.1)
	}

	if stage.Outcome != env.StageImpacted {
		t.Errorf("исход = %v, ожидалось падение на поверхность", stage.Outcome)
	}
}

// -----------------------------------------------------------------------------
// Ориентация корпуса
// -----------------------------------------------------------------------------

// Ролл-программа доворачивает носитель с ориентации пускового устройства
// на плоскость выведения.

func TestStageTumblesOnReentry(t *testing.T) {
	pos := physics.Vec3{X: physics.EarthRadius + 90000}
	frame := physics.NewLocalFrame(pos)
	// Входим под углом к горизонту с заметной скоростью.
	vel := frame.Direction(-25, 90).Scale(2200)

	stage := env.NewSpentStage("test", pos, vel, 25600, 42.6, 1.83, 300, 0,
		physics.Vec3{Y: 0.02, Z: 0.01}, env.CalmWind())

	initialSpin := stage.SpinRate()

	for i := 0; i < 20000 && stage.Alive(); i++ {
		stage.Step(0.05, float64(i)*0.05)
	}

	if stage.PeakSpinRate <= initialSpin*2 {
		t.Errorf("ступень не раскрутилась: начальная скорость %.3f рад/с, пиковая %.3f",
			initialSpin, stage.PeakSpinRate)
	}
	t.Logf("Кувыркание: с %.2f до %.1f °/с, разрушение — %s",
		initialSpin*180/math.Pi, stage.PeakSpinRate*180/math.Pi, stage.OutcomeReason)
}

// Кувыркающееся тело тормозится сильнее ориентированного по потоку:
// боковая проекция ступени вчетверо больше миделя.

func TestTumblingIncreasesDrag(t *testing.T) {
	shape := physics.DefaultStageShape(42.6, 1.83)

	axial := shape.EffectiveArea(0)
	broadside := shape.EffectiveArea(math.Pi / 2)

	if broadside <= axial*3 {
		t.Errorf("боковая проекция %.1f м² должна быть заметно больше миделя %.1f м²",
			broadside, axial)
	}
	approx(t, axial, shape.ReferenceArea(), 1e-9, "площадь при осевом обтекании")
}

// -----------------------------------------------------------------------------
// Многоузловой нагрев
// -----------------------------------------------------------------------------

// Носок и наветренная сторона греются сильнее донной части, укрытой следом.

func TestThermalNodesHeatDifferently(t *testing.T) {
	pos := physics.Vec3{X: physics.EarthRadius + 70000}
	frame := physics.NewLocalFrame(pos)
	vel := frame.Direction(-30, 90).Scale(3000)

	// Без начальной закрутки корпус сохраняет ориентацию, и разница
	// между узлами видна в чистом виде.
	stage := env.NewSpentStage("test", pos, vel, 25600, 42.6, 1.83, 300, 0,
		physics.Vec3{}, env.CalmWind())

	for i := 0; i < 4000 && stage.Alive(); i++ {
		stage.Step(0.02, float64(i)*0.02)
	}

	var nose, base *env.ThermalNode
	for i := range stage.Nodes {
		switch stage.Nodes[i].Name {
		case "nose":
			nose = &stage.Nodes[i]
		case "base":
			base = &stage.Nodes[i]
		}
	}
	if nose == nil || base == nil {
		t.Fatal("узлы конструкции не найдены")
	}

	// Ступень разворачивается двигателями вперёд и так и входит в атмосферу.
	//
	// Это не случайность и не ошибка: центр давления при поперечном обтекании
	// лежит примерно посередине корпуса, а центр масс — у двигателей, где
	// сосредоточена их масса. Устойчивое положение у такого тела одно —
	// тяжёлым концом вперёд. Именно поэтому возвращаемые ступени защищают
	// донную часть, а не носовую, и садятся двигателями вниз.
	//
	// Раньше здесь ожидался нагрев носка: знак аэродинамического момента был
	// перепутан, и ступень летела носом вперёд, то есть в неустойчивом
	// положении, в котором она физически удержаться не может.
	if base.PeakFlux <= nose.PeakFlux {
		t.Errorf("поток на донной части %.3g Вт/м² не превышает поток на носке %.3g, "+
			"хотя ступень входит в атмосферу двигателями вперёд",
			base.PeakFlux, nose.PeakFlux)
	}
	t.Logf("Пиковый поток: донная часть %.1f кВт/м², носок %.1f кВт/м² — "+
		"ступень развернуло двигателями вперёд",
		base.PeakFlux/1000, nose.PeakFlux/1000)
}

// -----------------------------------------------------------------------------
// Обломки
// -----------------------------------------------------------------------------

// После разрушения часть обломков обязана достигнуть поверхности.
// Раньше «разрушилась» означало, что до земли не долетало вообще ничего.

func TestDebrisReachesSurface(t *testing.T) {
	pos := physics.Vec3{X: physics.EarthRadius + 25000}
	frame := physics.NewLocalFrame(pos)
	vel := frame.Direction(-40, 90).Scale(1500)

	field := env.EstimateDebrisField(pos, vel, env.CalmWind(), 0)
	if field == nil {
		t.Fatal("поле обломков не построено")
	}

	reached := 0
	for _, imp := range field.Impacts {
		if imp.Reached {
			reached++
		}
	}
	if reached == 0 {
		t.Fatal("ни один фрагмент не достиг поверхности")
	}
	if field.SurvivingMassFraction <= 0 {
		t.Error("доля дошедшей массы равна нулю")
	}
	if field.SemiMajorAxis <= 0 || field.SemiMinorAxis <= 0 {
		t.Error("зона падения имеет нулевой размер")
	}

	t.Logf("Обломки: %d из %d классов достигли земли, зона %.1f × %.1f км, %.0f %% массы",
		reached, len(field.Impacts),
		field.SemiMajorAxis*2/1000, field.SemiMinorAxis*2/1000,
		field.SurvivingMassFraction*100)
}

// Плотные фрагменты улетают дальше лёгких: у них выше баллистический
// коэффициент, и они дольше сохраняют скорость.

func TestHeavyFragmentsFlyFurther(t *testing.T) {
	pos := physics.Vec3{X: physics.EarthRadius + 30000}
	frame := physics.NewLocalFrame(pos)
	vel := frame.Direction(-30, 90).Scale(1800)

	field := env.EstimateDebrisField(pos, vel, env.CalmWind(), 0)

	var lightest, heaviest env.FragmentImpact
	minBeta, maxBeta := math.Inf(1), math.Inf(-1)
	for _, imp := range field.Impacts {
		if !imp.Reached {
			continue
		}
		b := imp.Class.BallisticCoefficient()
		if b < minBeta {
			minBeta, lightest = b, imp
		}
		if b > maxBeta {
			maxBeta, heaviest = b, imp
		}
	}

	if math.IsInf(minBeta, 1) || minBeta == maxBeta {
		t.Skip("недостаточно долетевших фрагментов для сравнения")
	}

	if heaviest.Downrange <= lightest.Downrange {
		t.Errorf("плотный фрагмент (β=%.0f) улетел на %.1f км, лёгкий (β=%.0f) на %.1f км",
			maxBeta, heaviest.Downrange/1000, minBeta, lightest.Downrange/1000)
	}
	if heaviest.ImpactSpeed <= lightest.ImpactSpeed {
		t.Errorf("плотный фрагмент падает со скоростью %.0f м/с, лёгкий %.0f м/с",
			heaviest.ImpactSpeed, lightest.ImpactSpeed)
	}
}

// -----------------------------------------------------------------------------
// Статистика по серии пусков
// -----------------------------------------------------------------------------

// flightResult — итог одного пуска.
type flightResult struct {
	Seed         int64
	Phase        orbit.FlightPhase
	Telemetry    Telemetry
	PeakAoA      float64
	StageOutcome env.StageOutcome
}

// flyOnce прогоняет один полный пуск.
