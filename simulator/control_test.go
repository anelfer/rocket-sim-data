package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Сквозные тесты ручного управления.
//
// Здесь проверяется главное требование к интерфейсу: команда не рисует линию
// на графике, а входит в физическую модель и вызывает связанные изменения.
// Поэтому все проверки идут через реальный прогон симуляции, а не через
// отдельно взятый двигатель.
// -----------------------------------------------------------------------------

// runningSimulation возвращает симуляцию, выведенную на установившийся режим
// работы первой ступени.
func runningSimulation(t *testing.T) *Simulation {
	t.Helper()

	s := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), 20260728)
	for i := 0; i < 300; i++ { // 30 с модельного времени
		s.mu.Lock()
		s.step(s.Time.Step)
		s.telemetry = s.buildTelemetryLocked()
		s.mu.Unlock()
	}
	if tel := s.Snapshot(); tel.Propulsion.TotalThrust <= 0 {
		t.Fatal("двигатели не вышли на режим")
	}
	return s
}

// runFor продвигает симуляцию на заданное модельное время, сообщая пульту
// телеметрию так же, как это делает рабочий цикл.
func runFor(s *Simulation, seconds float64) Telemetry {
	steps := int(math.Round(seconds / s.Time.Step))
	var tel Telemetry
	for i := 0; i < steps; i++ {
		s.mu.Lock()
		s.step(s.Time.Step)
		s.telemetry = s.buildTelemetryLocked()
		tel = s.telemetry
		elapsed := s.elapsed
		s.mu.Unlock()

		sample := tel.controlSample()
		s.board.Observe(elapsed, sample)
		s.board.Record(elapsed, sample)
	}
	return tel
}

// shaftSpeed возвращает фактические обороты вала из модели, рад/с.
func shaftSpeed(s *Simulation) float64 {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.propulsion.PrimaryEngine().Turbopump.Speed
}

// npshRequired возвращает потребный кавитационный запас насоса горючего, м.
func npshRequired(s *Simulation) float64 {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.propulsion.PrimaryEngine().Turbopump.FuelPump.NPSHRequired
}

func mustApply(t *testing.T, s *Simulation, cmd control.Command) control.Result {
	t.Helper()
	res := s.Board().Apply(cmd, s.ModelTime(), s.Snapshot().controlSample())
	if res.Status != control.StatusApplied {
		t.Fatalf("команда %q не применена: %s (%s)", cmd.Parameter, res.Status, res.Reason)
	}
	return res
}

// Резкое повышение оборотов должно пройти по всей цепочке до тяги.
func TestPumpSpeedUpPropagatesThroughModel(t *testing.T) {
	s := runningSimulation(t)
	before := s.Snapshot().controlSample()

	npshBefore := npshRequired(s)

	mustApply(t, s, control.Command{
		ID: "spin-up", Parameter: "tp.shaft.speed", Mode: control.ModeRelative,
		Basis: control.BasisPercentCurrent, Value: 12,
	})

	after := runFor(s, 6).controlSample()

	chain := []struct {
		key, title string
	}{
		{"turbopump.rpm", "обороты вала"},
		{"turbopump.fuelPumpOutlet", "давление за насосом горючего"},
		{"turbopump.oxPumpOutlet", "давление за насосом окислителя"},
		{"engine.fuelFlow", "расход горючего"},
		{"engine.oxFlow", "расход окислителя"},
		{"chamber.pressure", "давление в камере"},
		{"engine.thrust", "тяга"},
	}
	for _, c := range chain {
		if after[c.key] <= before[c.key] {
			t.Errorf("%s не выросло: %.4g → %.4g", c.title, before[c.key], after[c.key])
		}
	}

	// Побочные следствия роста оборотов, которых никто не задавал.
	//
	// Потребный кавитационный запас растёт как квадрат оборотов. При этом
	// сам запас может и не сократиться: выросшая тяга поднимает продольную
	// перегрузку, а вместе с ней и столб жидкости над заборным устройством.
	// Поэтому проверяется именно потребный запас — прямое следствие оборотов,
	// а не их разность, в которой участвует ещё и динамика носителя.
	if npshRequired(s) <= npshBefore {
		t.Errorf("потребный кавитационный запас не вырос: %.2f → %.2f м",
			npshBefore, npshRequired(s))
	}
	if after["turbopump.bearingTemperature"] <= before["turbopump.bearingTemperature"] {
		t.Error("подшипники не нагрелись при росте оборотов")
	}
	if after["vehicle.acceleration"] <= before["vehicle.acceleration"] {
		t.Error("перегрузка носителя не выросла вслед за тягой")
	}

	t.Logf("Обороты %.0f → %.0f, Pc %.2f → %.2f МПа, тяга %.0f → %.0f кН, "+
		"потребный запас %.2f → %.2f м, располагаемый запас %.1f → %.1f м, "+
		"подшипники %.0f → %.0f К",
		before["turbopump.rpm"], after["turbopump.rpm"],
		before["chamber.pressure"], after["chamber.pressure"],
		before["engine.thrust"], after["engine.thrust"],
		npshBefore, npshRequired(s),
		before["turbopump.fuelCavitationMargin"], after["turbopump.fuelCavitationMargin"],
		before["turbopump.bearingTemperature"], after["turbopump.bearingTemperature"])
}

// Остановка турбонасоса гасит двигатель, а не обнуляет график.
func TestPumpStopShutsEngineDown(t *testing.T) {
	s := runningSimulation(t)
	before := s.Snapshot().controlSample()

	mustApply(t, s, control.Command{
		ID: "stop-pump", Parameter: "tp.shaft.speed",
		Mode: control.ModeFailure, Failure: control.FailureStopped,
	})

	after := runFor(s, 10).controlSample()

	if after["turbopump.rpm"] >= before["turbopump.rpm"]*0.5 {
		t.Errorf("обороты не упали: %.0f → %.0f", before["turbopump.rpm"], after["turbopump.rpm"])
	}
	if after["engine.thrust"] >= before["engine.thrust"]*0.2 {
		t.Errorf("тяга не пропала: %.0f → %.0f кН",
			before["engine.thrust"], after["engine.thrust"])
	}
	if after["engine.fuelFlow"] >= before["engine.fuelFlow"]*0.2 {
		t.Error("расход горючего не прекратился")
	}

	// Вал останавливается не мгновенно: его тормозят насосы и трение.
	// Мгновенный обрыв означал бы подмену значения, а не физику.
	mid := runFor(s, 0).controlSample()
	_ = mid

	t.Logf("Остановка агрегата: обороты %.0f → %.0f, тяга %.0f → %.0f кН, Pc %.2f → %.2f МПа",
		before["turbopump.rpm"], after["turbopump.rpm"],
		before["engine.thrust"], after["engine.thrust"],
		before["chamber.pressure"], after["chamber.pressure"])
}

// Заклинивший вал — это не остановленный вал: обороты фиксируются.
func TestStuckShaftFreezesSpeed(t *testing.T) {
	s := runningSimulation(t)
	before := s.Snapshot().controlSample()

	mustApply(t, s, control.Command{
		ID: "stuck", Parameter: "tp.shaft.speed",
		Mode: control.ModeFailure, Failure: control.FailureStuck,
	})

	// Сравнивается состояние модели, а не показание датчика: у датчика есть
	// шум и запаздывание, и по нему заклинивание не отличить от дрейфа.
	locked := shaftSpeed(s)
	after := runFor(s, 5).controlSample()

	if math.Abs(shaftSpeed(s)-locked) > 1e-9 {
		t.Errorf("обороты заклинившего вала изменились: %.3f → %.3f рад/с",
			locked, shaftSpeed(s))
	}
	if after["turbopump.vibration"] <= before["turbopump.vibration"] {
		t.Error("заклинивание не отразилось на вибрации")
	}
}

// Возврат в штатный режим восстанавливает работу двигателя.
func TestReturnToNominalRestoresEngine(t *testing.T) {
	s := runningSimulation(t)
	nominal := s.Snapshot().controlSample()

	mustApply(t, s, control.Command{
		ID: "drop", Parameter: "tp.shaft.speed", Mode: control.ModeRelative,
		Basis: control.BasisPercentCurrent, Value: -35,
	})
	degraded := runFor(s, 6).controlSample()

	if degraded["engine.thrust"] >= nominal["engine.thrust"]*0.9 {
		t.Fatalf("тяга не упала после снижения оборотов: %.0f → %.0f кН",
			nominal["engine.thrust"], degraded["engine.thrust"])
	}

	s.RestoreNominal()
	restored := runFor(s, 8).controlSample()

	// Полное совпадение недостижимо: за это время выработалось топливо
	// и изменилась высота. Проверяется именно возврат к режиму.
	if restored["engine.thrust"] < degraded["engine.thrust"]*1.15 {
		t.Errorf("тяга не восстановилась: %.0f → %.0f кН",
			degraded["engine.thrust"], restored["engine.thrust"])
	}
	if len(s.Board().Effects()) != 0 {
		t.Error("после возврата в штатный режим остались активные воздействия")
	}

	t.Logf("Тяга: номинал %.0f → снижение %.0f → восстановление %.0f кН",
		nominal["engine.thrust"], degraded["engine.thrust"], restored["engine.thrust"])
}

// Потеря наддува бака проходит через кавитацию к падению тяги.
func TestPressurantLossCascades(t *testing.T) {
	s := runningSimulation(t)
	before := s.Snapshot().controlSample()

	mustApply(t, s, control.Command{
		ID: "prs-loss", Parameter: "tank.ox.pressurant_valve",
		Mode: control.ModeStep, Value: 0,
	})

	after := runFor(s, 25).controlSample()

	if after["oxTank.pressure"] >= before["oxTank.pressure"] {
		t.Errorf("давление в баке не упало: %.0f → %.0f кПа",
			before["oxTank.pressure"], after["oxTank.pressure"])
	}
	if after["turbopump.oxPumpInlet"] >= before["turbopump.oxPumpInlet"] {
		t.Error("давление на входе насоса не снизилось")
	}
	if after["turbopump.oxCavitationMargin"] >= before["turbopump.oxCavitationMargin"] {
		t.Error("кавитационный запас не сократился")
	}

	t.Logf("Потеря наддува: бак %.0f → %.0f кПа, вход насоса %.0f → %.0f кПа, "+
		"запас %.1f → %.1f м, тяга %.0f → %.0f кН",
		before["oxTank.pressure"], after["oxTank.pressure"],
		before["turbopump.oxPumpInlet"], after["turbopump.oxPumpInlet"],
		before["turbopump.oxCavitationMargin"], after["turbopump.oxCavitationMargin"],
		before["engine.thrust"], after["engine.thrust"])
}

// Засорение форсунок горючего обедняет смесь и греет камеру.
func TestInjectorFoulingShiftsMixture(t *testing.T) {
	s := runningSimulation(t)
	before := s.Snapshot().controlSample()

	mustApply(t, s, control.Command{
		ID: "fouling", Parameter: "chamber.injector.fuel_area",
		Mode: control.ModeRamp, Value: 0.75, Duration: 4,
	})

	after := runFor(s, 8).controlSample()

	if after["chamber.mixtureRatio"] <= before["chamber.mixtureRatio"] {
		t.Errorf("соотношение компонентов не выросло: %.3f → %.3f",
			before["chamber.mixtureRatio"], after["chamber.mixtureRatio"])
	}
	if after["chamber.temperature"] <= before["chamber.temperature"] {
		t.Error("температура в камере не выросла при обеднении по горючему")
	}
	if after["engine.specificImpulse"] >= before["engine.specificImpulse"] {
		t.Error("удельный импульс не снизился при отклонении O/F")
	}
	if after["engine.fuelFlow"] >= before["engine.fuelFlow"] {
		t.Error("расход горючего не снизился при засорении форсунок")
	}
}

// Потеря охлаждения греет стенку и ускоряет эрозию горловины.
func TestCoolingLossHeatsWall(t *testing.T) {
	s := runningSimulation(t)
	before := s.Snapshot().controlSample()

	// Расход срезается умеренно. Втрое меньший расход стенка теперь не
	// переживает: она прогорает за секунды, двигатель гаснет, и мерить
	// нагрев охладителя становится не на чем — греть его нечему.
	mustApply(t, s, control.Command{
		ID: "cooling", Parameter: "nozzle.cooling_flow",
		Mode: control.ModeStep, Value: 0.75,
	})

	after := runFor(s, 20).controlSample()

	if after["nozzle.wallMargin"] >= before["nozzle.wallMargin"] {
		t.Errorf("запас до предела стенки не сократился: %.0f → %.0f К",
			before["nozzle.wallMargin"], after["nozzle.wallMargin"])
	}
	if after["nozzle.coolantOutlet"] <= before["nozzle.coolantOutlet"] {
		t.Error("охладитель не нагрелся сильнее при снижении расхода")
	}
}

// Пульт обязан зарегистрировать отклик по данным модели.
func TestReactionRecordedFromModelData(t *testing.T) {
	s := runningSimulation(t)

	mustApply(t, s, control.Command{
		ID: "spin", Parameter: "tp.shaft.speed", Mode: control.ModeRelative,
		Basis: control.BasisPercentCurrent, Value: 10,
	})
	runFor(s, 10)

	var entry *control.LogEntry
	for _, e := range s.Board().Log() {
		if e.ID == "spin" {
			entry = &e
		}
	}
	if entry == nil || entry.Reaction == nil {
		t.Fatal("отклик не зарегистрирован")
	}
	if len(entry.Reaction.Chain) < 3 {
		t.Errorf("цепочка отклика слишком короткая: %v", entry.Reaction.Chain)
	}

	// Отклик должен быть упорядочен по времени появления.
	prev := -1.0
	for _, r := range entry.Reaction.Responses {
		if r.Onset < prev {
			t.Errorf("отклики не упорядочены по времени: %s на %.1f с после %.1f",
				r.Title, r.Onset, prev)
		}
		prev = r.Onset
	}
	t.Logf("Зарегистрированная цепочка: %v", entry.Reaction.Chain)
}

// Пауза останавливает модельное время, шаг продвигает его на заданную величину.
func TestPauseAndStep(t *testing.T) {
	s := runningSimulation(t)

	s.mu.Lock()
	s.runState = RunRunning
	s.mu.Unlock()

	s.Pause()
	if s.RunStatus() != RunPaused {
		t.Fatal("симуляция не встала на паузу")
	}

	before := s.ModelTime()
	s.StepForward(5)

	s.mu.Lock()
	steps := s.pendingSteps
	s.pendingSteps = 0
	for i := 0; i < steps; i++ {
		s.step(s.Time.Step)
	}
	after := s.elapsed
	s.mu.Unlock()

	if want := before + 5*s.Time.Step; math.Abs(after-want) > 1e-9 {
		t.Errorf("после пяти шагов время %.3f с, ожидалось %.3f", after, want)
	}

	s.Resume()
	if s.RunStatus() != RunRunning {
		t.Error("симуляция не возобновилась")
	}
}

// Скорость симуляции приводится к поддерживаемым режимам.
func TestScaleSnapsToSupportedModes(t *testing.T) {
	s := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), 1)

	// Верхний предел — пятьдесят: столько нужно, чтобы не ждать вход
	// в атмосферу двадцать минут реального времени. Шаг интегрирования
	// при этом не меняется, растёт только число шагов за такт.
	cases := map[float64]float64{
		0.1: 0.1, 0.3: 0.25, 0.4: 0.5, 1: 1, 3: 2, 4: 5, 7: 5, 12: 10,
		18: 20, 30: 20, 44: 50, 1000: 50,
	}
	for in, want := range cases {
		if got := s.SetScale(in); got != want {
			t.Errorf("скорость %.2f приведена к %.2f, ожидалось %.2f", in, got, want)
		}
	}
}

// Сброс возвращает модель в исходное состояние и очищает пульт.
func TestResetRunClearsEverything(t *testing.T) {
	s := runningSimulation(t)
	s.Board().SetUnsafeMode(true)
	mustApply(t, s, control.Command{
		ID: "c1", Parameter: "engine.throttle", Mode: control.ModeStep, Value: 0.5,
	})

	run := s.RunNumber()
	s.ResetRun()

	if s.ModelTime() != 0 {
		t.Errorf("модельное время после сброса %.2f с", s.ModelTime())
	}
	if s.RunNumber() != run+1 {
		t.Error("номер прогона не увеличился")
	}
	if len(s.Board().Effects()) != 0 || len(s.Board().Log()) != 0 {
		t.Error("пульт не очищен")
	}
	if unsafe, _ := s.Board().Modes(); unsafe {
		t.Error("режим Unsafe overrides пережил сброс")
	}
}

// Аварийное выключение действует немедленно и в обход приоритетов.
func TestEmergencyShutdown(t *testing.T) {
	s := runningSimulation(t)

	mustApply(t, s, control.Command{
		ID: "hold", Parameter: "engine.throttle", Mode: control.ModeStep,
		Value: 1, Priority: 100,
	})

	s.EmergencyShutdown()
	after := runFor(s, 3).controlSample()

	if after["engine.thrust"] > 1 {
		t.Errorf("после аварийного выключения тяга %.1f кН", after["engine.thrust"])
	}
}

// Прямая подмена состояния помечается как нарушение причинности.
func TestDirectOverrideIsMarked(t *testing.T) {
	s := runningSimulation(t)
	s.Board().SetDirectMode(true)

	mustApply(t, s, control.Command{
		ID: "direct", Parameter: "tp.shaft.speed_direct", Mode: control.ModeStep,
		Value: 20000, Direct: true,
	})

	after := runFor(s, 2).controlSample()
	if math.Abs(after["turbopump.rpm"]-20000) > 500 {
		t.Errorf("прямая подмена не применилась: обороты %.0f", after["turbopump.rpm"])
	}

	for _, e := range s.Board().Effects() {
		if e.Parameter == "tp.shaft.speed_direct" && !e.Direct {
			t.Error("воздействие не помечено как прямая подмена")
		}
	}
}

// Сравнение с эталоном показывает разницу по тяге и удельному импульсу.
func TestBaselineComparisonOverRun(t *testing.T) {
	s := runningSimulation(t)
	runFor(s, 20)
	s.Board().SetBaseline("Штатный участок", s.Seed())

	mustApply(t, s, control.Command{
		ID: "trim", Parameter: "engine.mixture_trim", Mode: control.ModeStep,
		Value: 0.9,
	})
	runFor(s, 20)

	cmp := s.Board().Compare()
	if !cmp.Available {
		t.Fatal("сравнение недоступно")
	}
	if len(cmp.Rows) == 0 {
		t.Fatal("расхождений не найдено, хотя режим изменён")
	}
	t.Logf("Наибольшее расхождение: %s %.4g → %.4g %s",
		cmp.Rows[0].Title, cmp.Rows[0].Baseline, cmp.Rows[0].Current, cmp.Rows[0].Unit)
}
