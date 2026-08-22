package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"

	"rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Тепловая модель двигателя: стенка, камера, сопло, аэродинамический нагрев.
// -----------------------------------------------------------------------------

func TestWallCoolsWhenEngineIsOff(t *testing.T) {
	e := vehicle.Engine{
		MaxThrust: 845000,
		Running:   false,
		Thermal:   vehicle.DefaultEngineThermalConfig(),
	}
	e.SetTemperatures(1500)

	env := vehicle.EnvironmentState{
		Altitude:      300000,
		Density:       physics.AtmosphericDensity(300000),
		AmbientTempK:  physics.AtmosphericTemperature(300000),
		RelativeSpeed: 0,
		Mach:          0,
	}

	for i := 0; i < 20000; i++ {
		e.UpdateThermalState(1.0, env)
	}

	equilibrium := physics.RadiativeEquilibriumTemperature(300000)
	if math.Abs(e.WallTempK-equilibrium) > 30 {
		t.Errorf("стенка выключенного двигателя остановилась на %.1f К, ожидалось около %.1f К",
			e.WallTempK, equilibrium)
	}
	if e.WallTempC() > 100 {
		t.Errorf("стенка выключенного двигателя не должна оставаться раскалённой: %.1f °C",
			e.WallTempC())
	}
}

// При работе двигателя стенка выходит на температуру, ограниченную
// регенеративным охлаждением.

func TestWallEquilibriumWhenRunning(t *testing.T) {
	e := vehicle.Engine{
		MaxThrust: 845000,
		Thrust:    845000,
		Running:   true,
		Thermal:   vehicle.DefaultEngineThermalConfig(),
	}
	e.SetTemperatures(288)

	env := vehicle.EnvironmentState{Altitude: 0, Density: 1.225, AmbientTempK: 288, RelativeSpeed: 0}
	for i := 0; i < 2000; i++ {
		e.UpdateThermalState(0.1, env)
	}

	if e.WallTempK < 600 || e.WallTempK > 1100 {
		t.Errorf("стенка работающего двигателя = %.0f К, ожидалось 600…1100 К", e.WallTempK)
	}
	// Стенка обязана быть заметно холоднее газа в камере — в этом смысл
	// регенеративного охлаждения.
	if e.WallTempK >= e.ChamberTempK {
		t.Errorf("стенка (%.0f К) не может быть горячее камеры (%.0f К)",
			e.WallTempK, e.ChamberTempK)
	}
}

// B4: дросселирование почти не меняет температуру в камере.
// Раньше линейная зависимость от тяги роняла её на 1100 °C при газе 65 %.

func TestChamberTemperatureWeaklyDependsOnThrottle(t *testing.T) {
	run := func(throttle float64) float64 {
		e := vehicle.Engine{
			MaxThrust: 845000,
			Thrust:    845000 * throttle,
			Running:   true,
			Thermal:   vehicle.DefaultEngineThermalConfig(),
		}
		e.SetTemperatures(288)
		env := vehicle.EnvironmentState{Altitude: 10000, Density: 0.41, AmbientTempK: 223}
		for i := 0; i < 500; i++ {
			e.UpdateThermalState(0.1, env)
		}
		return e.ChamberTempK
	}

	full := run(1.0)
	throttled := run(0.65)

	if full < 3300 || full > 3600 {
		t.Errorf("температура в камере на номинале = %.0f К, ожидалось 3300…3600 К", full)
	}
	drop := full - throttled
	if drop > 100 {
		t.Errorf("дросселирование до 65%% уронило температуру на %.0f К, ожидалось меньше 100 К", drop)
	}
}

// Газ на срезе сопла должен быть существенно холоднее, чем в камере.

func TestNozzleColderThanChamber(t *testing.T) {
	e := vehicle.Engine{
		MaxThrust: 845000,
		Thrust:    845000,
		Running:   true,
		Thermal:   vehicle.DefaultEngineThermalConfig(),
	}
	e.SetTemperatures(288)
	env := vehicle.EnvironmentState{Altitude: 0, Density: 1.225, AmbientTempK: 288}
	for i := 0; i < 500; i++ {
		e.UpdateThermalState(0.1, env)
	}

	ratio := e.NozzleTempK / e.ChamberTempK
	if ratio > 0.6 || ratio < 0.3 {
		t.Errorf("отношение температур срез/камера = %.3f, ожидалось 0.3…0.6", ratio)
	}
}

// B6: схема устойчива при любом шаге.

func TestThermalModelStableAtLargeSteps(t *testing.T) {
	for _, dt := range []float64{0.1, 1, 5, 30, 120} {
		e := vehicle.Engine{
			MaxThrust: 845000,
			Thrust:    845000,
			Running:   true,
			Thermal:   vehicle.DefaultEngineThermalConfig(),
		}
		e.SetTemperatures(288)
		env := vehicle.EnvironmentState{Altitude: 0, Density: 1.225, AmbientTempK: 288}

		for i := 0; i < 200; i++ {
			e.UpdateThermalState(dt, env)
			if math.IsNaN(e.WallTempK) || e.WallTempK < 0 || e.WallTempK > 5000 {
				t.Fatalf("шаг %.1f с: расходимость, стенка = %.4g К", dt, e.WallTempK)
			}
		}
	}
}

// B3: аэродинамический нагрев действительно влияет на конструкцию.

func TestAeroHeatingRaisesWallTemperature(t *testing.T) {
	build := func() vehicle.Engine {
		e := vehicle.Engine{MaxThrust: 845000, Running: false, Thermal: vehicle.DefaultEngineThermalConfig()}
		e.SetTemperatures(250)
		return e
	}

	altitude := 40000.0
	density := physics.AtmosphericDensity(altitude)
	ambient := physics.AtmosphericTemperature(altitude)
	sound := physics.SpeedOfSound(ambient)

	cold := build()
	hot := build()

	slowEnv := vehicle.EnvironmentState{Altitude: altitude, Density: density, AmbientTempK: ambient}
	fastEnv := vehicle.EnvironmentState{
		Altitude: altitude, Density: density, AmbientTempK: ambient,
		RelativeSpeed: 6 * sound, Mach: 6,
	}

	for i := 0; i < 300; i++ {
		cold.UpdateThermalState(0.1, slowEnv)
		hot.UpdateThermalState(0.1, fastEnv)
	}

	if hot.WallTempK <= cold.WallTempK+50 {
		t.Errorf("обдув на M=6 не разогрел конструкцию: %.1f К против %.1f К",
			hot.WallTempK, cold.WallTempK)
	}
}

// -----------------------------------------------------------------------------
// E5: компоновка блока двигателей
// -----------------------------------------------------------------------------

// Перегретая стенка обязана прогорать, а не просто светиться предупреждением.
//
// Раньше запас до предела уходил в минус на сотни градусов, а двигатель
// продолжал работать как ни в чём не бывало: унос материала при таком
// перегреве составлял доли микрона за весь полёт. Оператор видел «стенка
// близка к прогару» и не получал никаких последствий.
func TestCoolingLossBurnsThroughAndStopsEngine(t *testing.T) {
	s := runningSimulation(t)

	before := s.Snapshot()
	if before.Propulsion.Engine.WallMargin <= 0 {
		t.Fatalf("на номинале стенка уже за пределом: запас %.0f К",
			before.Propulsion.Engine.WallMargin)
	}

	mustApply(t, s, control.Command{
		ID: "cooling-off", Parameter: "nozzle.cooling_flow",
		Mode: control.ModeFailure, Failure: control.FailureStopped,
	})

	// Проверяется именно ход событий: стенка сначала уходит за предел,
	// потом прогорает, и только потом двигатель гаснет. Смотреть один
	// снимок в конце нельзя — у погасшего двигателя стенка остывает,
	// и запас снова становится положительным.
	var worstMargin = before.Propulsion.Engine.WallMargin
	var burnedAt = -1.0

	for i := 0; i < 12; i++ {
		tel := runFor(s, 2)
		e := tel.Propulsion.Engine

		if e.WallMargin < worstMargin {
			worstMargin = e.WallMargin
		}
		if e.BurnedThrough && burnedAt < 0 {
			burnedAt = tel.Time
			break
		}
	}

	if worstMargin >= before.Propulsion.Engine.WallMargin {
		t.Errorf("без охлаждения стенка не нагрелась: запас %.0f → %.0f К",
			before.Propulsion.Engine.WallMargin, worstMargin)
	}
	if burnedAt < 0 {
		t.Fatalf("стенка не прогорела за двадцать четыре секунды без охлаждения: "+
			"худший запас %.0f К", worstMargin)
	}
	// Двигатели блока прогорают не строго одновременно: у каждого свой
	// разброс характеристик. Поэтому тяга должна пропасть не мгновенно
	// с первым прогаром, а в течение нескольких секунд после него.
	after := runFor(s, 6)
	if after.Propulsion.TotalThrust > 0.05*before.Propulsion.TotalThrust {
		t.Errorf("через шесть секунд после прогара ступень всё ещё даёт %.2f МН из %.2f МН",
			after.Propulsion.TotalThrust/1e6, before.Propulsion.TotalThrust/1e6)
	}

	t.Logf("прогар на T+%.1f с, худший запас %.0f К, тяга ступени %.2f → %.2f МН",
		burnedAt, worstMargin,
		before.Propulsion.TotalThrust/1e6, after.Propulsion.TotalThrust/1e6)
}

// Штатное охлаждение стенку не портит: повреждение не копится вовсе.
func TestNominalCoolingKeepsWallIntact(t *testing.T) {
	s := runningSimulation(t)
	tel := runFor(s, 60)

	e := tel.Propulsion.Engine
	if e.WallDamage > 0 {
		t.Errorf("на штатном режиме стенка повреждается: %.4f при запасе %.0f К",
			e.WallDamage, e.WallMargin)
	}
}
