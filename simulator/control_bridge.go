package simulator

import (
	"fmt"
	"math"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Связь пульта воздействий с физической моделью.
//
// Пульт не имеет доступа к внутренним переменным симуляции. Обмен идёт в обе
// стороны через две узкие точки: поправки, которые модель применяет на шаге,
// и снимок наблюдаемых величин, по которому пульт судит об отклике.
// -----------------------------------------------------------------------------

// Board возвращает пульт воздействий этой симуляции.
func (s *Simulation) Board() *control.Board { return s.board }

// ControlSample возвращает снимок наблюдаемых величин на текущий момент.
//
// Нужен при приёме команды: относительные воздействия отсчитываются
// от фактического значения, а не от номинала.
func (s *Simulation) ControlSample() control.Sample {
	return s.Snapshot().controlSample()
}

// deficitOrNaN возвращает недобор тяги, если ступень её вообще просит.
func deficitOrNaN(demand, deficit float64) float64 {
	if demand <= 0 {
		return math.NaN()
	}
	return deficit
}

// controlSample собирает наблюдаемые величины в единицах интерфейса.
//
// Единицы намеренно те же, что видит оператор: мегапаскали, килоньютоны,
// тонны. Иначе пороги чувствительности пришлось бы задавать в системе СИ,
// а сравнивать значения на экране — в другой.
// shieldValue отдаёт величину теплозащиты или NaN, если теплозащиты нет.
func shieldValue(h *HeatShieldTelemetry, pick func(*HeatShieldTelemetry) float64) float64 {
	if h == nil {
		return math.NaN()
	}
	return pick(h)
}

func (t Telemetry) controlSample() control.Sample {
	p := t.Propulsion
	e := p.Engine

	sample := control.Sample{
		"turbopump.rpm":                  e.ShaftRPM.Value(),
		"turbopump.fuelPumpOutlet":       e.FuelPumpOutlet.Value() / 1e6,
		"turbopump.oxPumpOutlet":         e.OxPumpOutlet.Value() / 1e6,
		"turbopump.fuelPumpInlet":        e.FuelPumpInlet.Value() / 1e3,
		"turbopump.oxPumpInlet":          e.OxPumpInlet.Value() / 1e3,
		"turbopump.turbinePower":         e.TurbinePower / 1e6,
		"turbopump.fuelPumpPower":        e.FuelPumpPower / 1e6,
		"turbopump.oxPumpPower":          e.OxPumpPower / 1e6,
		"turbopump.fuelCavitationMargin": e.FuelCavitationMargin,
		"turbopump.oxCavitationMargin":   e.OxCavitationMargin,
		"turbopump.vibration":            e.Vibration.Value(),
		"turbopump.bearingTemperature":   e.BearingTemperature.Value(),
		"turbopump.powerMargin":          e.PowerMargin / 1e6,
		"turbopump.mechanicalLoss":       e.MechanicalLoss / 1e6,
		"turbopump.angularAcceleration":  e.AngularAcceleration,

		// Недобор тяги против требуемого. Пока ступень тяги не просит,
		// величина не определена: у выключенного двигателя недобора нет,
		// есть отсутствие команды. Ноль здесь означал бы «всё в порядке»,
		// а это не то же самое.
		"engine.thrustDeficit": deficitOrNaN(p.ThrottleDemand, p.ThrustDeficit),

		"engine.fuelFlow":        p.TotalFuelFlow,
		"engine.oxFlow":          p.TotalOxFlow,
		"engine.thrust":          p.TotalThrust / 1e3,
		"engine.specificImpulse": p.SpecificImpulse,

		"chamber.mixtureRatio":      e.MixtureRatio,
		"chamber.pressure":          e.ChamberPressureMean / 1e6,
		"chamber.temperature":       e.ChamberTemperature,
		"chamber.cstarEfficiency":   e.CStarEfficiency,
		"chamber.stabilityMargin":   e.StabilityMargin,
		"chamber.pressureRMS":       e.PressureRMS / 1e3,
		"chamber.dominantFrequency": e.DominantFrequency,
		"chamber.temperatureSpread": e.TemperatureSpread * e.ChamberTemperature,

		"valves.fuel":   e.FuelValve,
		"valves.ox":     e.OxValve,
		"valves.gasGen": e.GasGenValve,
		"valves.bypass": e.BypassValve,

		"nozzle.wallMargin":    e.WallMargin,
		"nozzle.wallDamage":    e.WallDamage,
		"nozzle.wallHeatFlux":  e.WallHeatFlux / 1e6,
		"nozzle.coolingFlow":   e.CoolingFlow,
		"nozzle.coolantOutlet": e.CoolantOutlet.Value(),
		"nozzle.throatArea":    e.ThroatArea * 1e4,
		"nozzle.throatErosion": e.ThroatErosion,
		"nozzle.coolingDrop":   e.CoolingDrop / 1e3,
		"nozzle.coolantInlet":  e.CoolantInlet,

		"fuelTank.mass":           p.FuelTank.Mass / 1000,
		"oxTank.mass":             p.OxTank.Mass / 1000,
		"fuelTank.pressure":       p.FuelTank.Pressure / 1e3,
		"oxTank.pressure":         p.OxTank.Pressure / 1e3,
		"fuelTank.sloshAmplitude": p.FuelTank.SloshAmplitude,
		"oxTank.sloshAmplitude":   p.OxTank.SloshAmplitude,
		"fuelTank.temperature":    p.FuelTank.Temperature,
		"oxTank.temperature":      p.OxTank.Temperature,
		"fuelTank.lineDrop":       p.FuelTank.LineDrop / 1e3,
		"oxTank.lineDrop":         p.OxTank.LineDrop / 1e3,
		"fuelTank.pressurantFlow": p.FuelTank.PressurantFlow,
		"oxTank.pressurantFlow":   p.OxTank.PressurantFlow,
		"fuelTank.depletionTime":  p.FuelTank.DepletionTime.Value(),
		"oxTank.depletionTime":    p.OxTank.DepletionTime.Value(),

		// Теплозащита появляется только у корабля и только после разделения.
		// Пока её нет, величины не определены: ноль запаса означал бы, что
		// корпус на пределе, а нулевое повреждение — что теплозащита цела.
		"shield.tileMargin":  shieldValue(t.HeatShield, func(h *HeatShieldTelemetry) float64 { return h.Tiles.Margin }),
		"shield.steelMargin": shieldValue(t.HeatShield, func(h *HeatShieldTelemetry) float64 { return h.Steel.Margin }),
		"shield.damage": shieldValue(t.HeatShield, func(h *HeatShieldTelemetry) float64 {
			return math.Max(h.Tiles.Damage, h.Steel.Damage)
		}),
		"shield.exposure": shieldValue(t.HeatShield, func(h *HeatShieldTelemetry) float64 { return h.Exposure }),

		"vehicle.angleOfAttack": t.TotalAoA,
		"vehicle.pitchRate":     t.PitchRate,
		"control.gimbalPitch":   t.GimbalPitch,
		"control.gimbalDemand":  t.GimbalDemand,
		"control.authority":     t.ControlAuthority,
		"control.staticMargin":  t.StaticMargin,

		"vehicle.altitude": t.Altitude / 1000,
		"vehicle.velocity": t.TotalVelocity,
		"vehicle.mass":     t.TotalMass / 1000,
	}

	// Продольная перегрузка — то, что чувствует конструкция: тяга минус
	// сопротивление, отнесённые к массе.
	if t.TotalMass > 0 {
		sample["vehicle.acceleration"] = (t.TotalThrust - t.DragForce) /
			(t.TotalMass * physics.G0)
	}

	// Величины, которые ещё не определены, в снимок не попадают: пульт
	// отличает отсутствие данных от нуля.
	for k, v := range sample {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			delete(sample, k)
		}
	}
	return sample
}

// calibrateBoard сообщает пульту штатные значения активной ступени.
//
// Номинальные обороты вала и площадь горловины у ступеней разные, и после
// разделения уставки должны отсчитываться от новых значений, а не от старых.
// Вызывается под удержанным mu.
func (s *Simulation) calibrateBoard() {
	if s.board == nil || s.propulsion == nil {
		return
	}
	e := s.propulsion.PrimaryEngine()
	if e == nil {
		return
	}
	cfg := e.Config

	nominals := map[string]float64{
		"tp.shaft.speed":        cfg.Turbopump.DesignSpeed * 60 / (2 * math.Pi),
		"tp.shaft.speed_direct": cfg.Turbopump.DesignSpeed * 60 / (2 * math.Pi),
		"chamber.pressure":      cfg.Chamber.NominalPressure / 1e6,
		"nozzle.throat_area":    cfg.Nozzle.InitialThroatArea * 1e4,
		"tank.fuel.pressure":    s.propulsion.FuelTank.Config.TargetPressure / 1e3,
		"tank.ox.pressure":      s.propulsion.OxTank.Config.TargetPressure / 1e3,
		"tank.fuel.temperature": s.propulsion.FuelTank.Temperature,
		"tank.ox.temperature":   s.propulsion.OxTank.Temperature,
		"tank.fuel.mass":        s.propulsion.FuelTank.Mass,
		"tank.ox.mass":          s.propulsion.OxTank.Mass,
	}
	s.board.Calibrate(nominals)
}

// -----------------------------------------------------------------------------
// Управление ходом симуляции
// -----------------------------------------------------------------------------

// RunState — состояние хода симуляции.
type RunState string

const (
	RunIdle    RunState = "idle"
	RunRunning RunState = "running"
	RunPaused  RunState = "paused"
	RunStopped RunState = "stopped"
	RunEnded   RunState = "ended"
)

// Pause приостанавливает продвижение модельного времени.
//
// Телеметрия при этом продолжает публиковаться: оператору нужно видеть
// состояние остановленной модели, чтобы разобраться в нём.
func (s *Simulation) Pause() {
	s.mu.Lock()
	if s.runState == RunRunning {
		s.runState = RunPaused
	}
	s.mu.Unlock()
}

// Resume возобновляет ход симуляции.
func (s *Simulation) Resume() {
	s.mu.Lock()
	if s.runState == RunPaused {
		s.runState = RunRunning
	}
	s.mu.Unlock()
}

// StepForward выполняет заданное число шагов интегрирования на паузе.
func (s *Simulation) StepForward(steps int) {
	if steps <= 0 {
		steps = 1
	}
	s.mu.Lock()
	if s.runState == RunPaused {
		s.pendingSteps += steps
	}
	s.mu.Unlock()
}

// SetScale задаёт скорость течения модельного времени.
func (s *Simulation) SetScale(scale float64) float64 {
	// Допустимые режимы. Произвольное значение приводится к ближайшему:
	// иначе можно было бы задать шаг, при котором интегратор расходится.
	//
	// Ускорение не трогает шаг интегрирования — оно лишь увеличивает число
	// шагов за такт таймера. Поэтому пятидесятикратный режим считает ровно
	// ту же физику, что и однократный, просто быстрее: он нужен, чтобы
	// не сидеть двадцать минут в ожидании входа в атмосферу.
	allowed := []float64{0.1, 0.25, 0.5, 1, 2, 5, 10, 20, 50}
	best := allowed[0]
	for _, v := range allowed {
		if math.Abs(v-scale) < math.Abs(best-scale) {
			best = v
		}
	}

	s.mu.Lock()
	s.Time.Scale = best
	s.mu.Unlock()
	return best
}

// RunStatus возвращает состояние хода симуляции.
func (s *Simulation) RunStatus() RunState {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.runState
}

// ModelTime возвращает текущее модельное время, с.
func (s *Simulation) ModelTime() float64 {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.elapsed
}

// Scale возвращает текущую скорость течения модельного времени.
func (s *Simulation) Scale() float64 {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.Time.Scale
}

// RunNumber возвращает номер прогона.
func (s *Simulation) RunNumber() int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.runNumber
}

// SetRunNumber задаёт номер прогона.
func (s *Simulation) SetRunNumber(n int) {
	s.mu.Lock()
	s.runNumber = n
	s.mu.Unlock()
}

// Scenario возвращает название выбранного сценария.
func (s *Simulation) Scenario() string {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.scenario
}

// SetScenario задаёт название выбранного сценария.
func (s *Simulation) SetScenario(name string) {
	s.mu.Lock()
	s.scenario = name
	s.mu.Unlock()
}

// IgniteEngines запускает все камеры активной ступени по прямой команде
// оператора, в обход наведения.
//
// Нужна там, где наведение само двигатели не запустит: на орбите фаза
// полёта считается непитаемой, computeThrottle отдаёт нулевой газ, и
// «Штатный режим» (RestoreNominal) там бессилен — он перезапускает камеры
// только на активных участках выведения. Именно поэтому оператор, погасивший
// двигатели или оказавшийся на орбите с выключенной ступенью, не мог зажечь
// её заново ничем, кроме как дождаться, пока автоматика коррекции орбиты
// сама решит, что пора.
//
// Само зажигание тяги не даёт: камеры выходят на холостой режим с нулевой
// командой газа, и нужную тягу оператор задаёт отдельно, ползунком тяги
// или командой engine.throttle — тем же путём, каким уже управляют
// дросселированием на активном участке. Так зажигание и дросселирование
// остаются независимыми действиями, как и на настоящем пульте.
//
// Возвращает ошибку, если зажигание невозможно, а не молча ничего не делает:
// раньше вызов ничего не сообщал об отказе, интерфейс не имел возможности
// отличить реальное зажигание от отклонённой команды и показывал одно и то же
// подтверждение в обоих случаях.
func (s *Simulation) IgniteEngines() error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.propulsion == nil {
		return fmt.Errorf("симуляция ещё не готова")
	}
	if s.crashed {
		return fmt.Errorf("носитель потерян, зажигание невозможно")
	}
	if s.phase == orbit.PhaseLanded {
		return fmt.Errorf("корабль на посадочной площадке, зажигание всех камер отключено")
	}

	// Отказ «двигатель остановлен» держит камеру выключенной каждый шаг —
	// без снятия он тут же погасит только что зажжённую камеру заново,
	// а стартовавшую тем же вызовом Start() операция ничего об этом не узнает.
	if s.board != nil {
		s.board.ReleaseFailuresOf("engine.throttle")
	}
	s.startAllEngines()

	// Огонь теперь оператора, а не автоматики поддержания орбиты: если она
	// сама в этот момент вела коррекционный импульс, флаг сбрасывается,
	// и maintainOrbit больше не станет гасить эти камеры по своему усмотрению —
	// раньше именно это и происходило, ровно через один такт после команды.
	s.orbitCorrectionBurn = false
	return nil
}

// IgniteEngine запускает одну конкретную камеру по идентификатору
// ("S1-14", "S2-V2" и подобные) — например, чтобы точечно перезапустить
// один агрегат после отказа, не трогая остальные.
//
// Идентификатор ищется среди реальных камер двигательной установки
// (s.propulsion.Engines) — именно они считают тягу; список s.engines
// синхронизируется тем же ID заодно, но только когда он там есть: для
// вакуумных камер второй ступени эти два списка называют агрегаты
// по-разному, и это не повод отказывать в зажигании существующей камеры.
func (s *Simulation) IgniteEngine(id string) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.propulsion == nil || s.crashed || s.phase == orbit.PhaseLanded {
		return fmt.Errorf("зажигание недоступно в текущей фазе полёта")
	}

	found := false
	for _, e := range s.propulsion.Engines {
		if e.ID == id {
			e.Start()
			found = true
			break
		}
	}
	if !found {
		return fmt.Errorf("неизвестный двигатель %q", id)
	}

	// Тот же отказ, что и в IgniteEngines, но только на этом адресате: снимать
	// отказ соседних камер точечная команда не должна.
	if s.board != nil {
		s.board.ReleaseFailure("engine.throttle", id)
	}

	// Та же причина, что и в IgniteEngines: этот огонь больше не в ведении
	// автоматики поддержания орбиты.
	s.orbitCorrectionBurn = false

	for i := range s.engines {
		if s.engines[i].ID == id {
			s.engines[i].Running = true
			break
		}
	}
	return nil
}

// EmergencyShutdown немедленно выключает все двигатели.
//
// Это не воздействие на параметр, а команда по цепи аварийного выключения,
// поэтому она не проходит через пульт и не может быть заблокирована
// приоритетом другого воздействия.
func (s *Simulation) EmergencyShutdown() {
	s.mu.Lock()
	defer s.mu.Unlock()

	for i := range s.engines {
		s.engines[i].Running = false
	}
	if s.propulsion != nil {
		s.propulsion.StopAll()
	}
	s.throttleCmd = 0
}

// RestoreNominal снимает все воздействия и возвращает двигательную установку
// под управление модели.
//
// Если двигатель успел погаснуть из-за наложенного отказа, он запускается
// заново: снятие отказа само по себе камеру не зажигает, а оператор,
// нажимая «вернуть штатный режим», ожидает именно восстановления работы.
// Выход на режим при этом остаётся переходным процессом — турбонасосу
// снова нужны секунды на раскрутку.
func (s *Simulation) RestoreNominal() {
	if s.board != nil {
		s.board.ReleaseAll()
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	if s.propulsion == nil || s.crashed {
		return
	}
	switch s.phase {
	case orbit.PhaseFirstStage, orbit.PhaseSecondStage, orbit.PhaseCircularization:
		s.propulsion.StartAll()
		for i := range s.engines {
			s.engines[i].Running = true
		}
	}
}

// ResetRun полностью возвращает симуляцию в исходное состояние и очищает пульт.
func (s *Simulation) ResetRun() {
	if s.board != nil {
		s.board.Reset()
	}
	s.mu.Lock()
	s.initState()
	s.runNumber++
	s.runState = RunPaused
	s.pendingSteps = 0
	s.mu.Unlock()
	s.mu.Lock()
	s.calibrateBoard()
	s.mu.Unlock()
}
