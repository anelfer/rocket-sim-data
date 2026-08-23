package simulator

import (
	"log"
	"math"
	"math/rand"
	"sync"
	"time"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/env"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/orbit/sensing"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/propulsion"
	"rocketTelemetrySim/simulator/vehicle"
)

// CurrentSimulation — активная симуляция. Доступ только через SetCurrentSimulation
// и GetCurrentSimulation, защищённые собственным мьютексом.
var (
	currentSimulation *Simulation
	currentMu         sync.RWMutex
)

// SetCurrentSimulation устанавливает активную симуляцию.
func SetCurrentSimulation(s *Simulation) {
	currentMu.Lock()
	currentSimulation = s
	currentMu.Unlock()
}

// GetCurrentSimulation возвращает активную симуляцию (может быть nil).
func GetCurrentSimulation() *Simulation {
	currentMu.RLock()
	defer currentMu.RUnlock()
	return currentSimulation
}

// -----------------------------------------------------------------------------
// Параметры хода времени
// -----------------------------------------------------------------------------

// TimeConfig задаёт связь модельного и реального времени.
//
// Раньше шаг был зашит сразу в четырёх местах и по-разному: переменная deltaT,
// поле Dt, неявная единица в формуле обновления координат и такая же неявная
// единица в расчёте расхода топлива. Изменить шаг было невозможно.
type TimeConfig struct {
	// Step — шаг интегрирования уравнений движения, с.
	Step float64

	// TickInterval — интервал между публикациями телеметрии в реальном времени.
	TickInterval time.Duration

	// Scale — сколько секунд модельного времени проходит за одну секунду
	// реального. 1 — реальное время.
	Scale float64
}

// DefaultTimeConfig возвращает настройки по умолчанию: шаг интегрирования
// 0.05 с, публикация телеметрии двадцать раз в секунду.
//
// Публиковать раз в секунду, как было в самом начале, значит выбрасывать
// почти все состояния: на пульте это выглядело как рывки, а через снимок
// телеметрии терялось всё, что происходит быстрее секунды, — заброс давления
// при запуске, перекладка плавников, разворот перед посадкой.
//
// Шаг и такт публикации держатся равными намеренно. Публиковать чаще, чем
// считает модель, бессмысленно: между тактами состояние не меняется, и в
// поток уходят дубли. Считать чаще, чем публикуем, — значит снова выбрасывать
// посчитанное.
//
// Двадцать герц выбраны как предел полезного: шаг модели стоит 0.1 мс,
// то есть при таком темпе счёт занимает две тысячных доли ядра, а сцена
// получает вдвое больше опорных состояний для промежуточных кадров. Идти
// дальше смысла нет — глазу хватает промежуточных кадров браузера, а вот
// сборка снимка в JSON стоит уже 0.3 мс, и её частоту поднимать нельзя.
func DefaultTimeConfig() TimeConfig {
	return TimeConfig{
		Step:         0.05,
		TickInterval: 50 * time.Millisecond,
		Scale:        1.0,
	}
}

// -----------------------------------------------------------------------------
// Симуляция
// -----------------------------------------------------------------------------

// Simulation — состояние симулятора целиком.
type Simulation struct {
	Config vehicle.Config
	Time   TimeConfig

	mu sync.RWMutex

	// Состояние, защищённое mu.
	state        VehicleState
	engines      []vehicle.Engine
	dryMass      float64
	area         float64
	elapsed      float64
	phase        orbit.FlightPhase
	stage        int
	gnc          *orbit.GNCSystem
	maxQ         MaxQState
	telemetry    Telemetry
	fairingGone  bool
	mecoTime     float64
	secoTime     float64
	circStart    float64
	crashed      bool
	released     bool
	orbitReached bool
	throttleCmd  float64

	// orbitCorrectionBurn сообщает, что двигатели сейчас работают по
	// собственному решению автоматики поддержания орбиты (maintainOrbit),
	// а не по команде оператора.
	//
	// Раньше maintainOrbit гасила двигатели всякий раз, когда они работают,
	// а коррекция уже не нужна, — не различая, кто их зажёг. Оператор,
	// вручную зажёгший камеры на орбите, получал их обратно погашенными
	// в течение одного такта: автоматика видела «горит, а нужды нет» и тут же
	// объявляла коррекцию завершённой. Флаг помечает именно тот огонь, что
	// зажгла сама автоматика, — гасить она имеет право только его.
	orbitCorrectionBurn bool

	// propulsion — двигательная установка активной ступени: баки, наддув,
	// турбонасосы, камеры сгорания, сопла и охлаждение.
	propulsion *PropulsionSystem

	// attitude — угловое движение корпуса: моменты, качание камер, автопилот.
	attitude VehicleAttitude

	// prevAcceleration — ускорение предыдущего шага. Нужно, чтобы посчитать
	// гидростатический напор в баках и возбуждение колебаний жидкости
	// до того, как будет известна тяга текущего шага.
	prevAxialAccel   float64
	prevLateralAccel float64
	prevAngularAccel float64

	// spentStage — отработавшая первая ступень. Продолжает интегрироваться
	// отдельным телом до входа в атмосферу или удара о поверхность.
	spentStage *env.SpentStage

	// wind — профиль ветра, разыгранный один раз на весь полёт.
	wind env.WindModel

	// dispersion — отклонения характеристик от номинала для этого пуска.
	dispersion Dispersion

	// seed — зерно генератора. Один и тот же seed даёт побитово одинаковый
	// полёт, что позволяет воспроизвести любой прогон.
	seed int64

	// Entry — профиль возвращения корабля.
	Entry EntryConfig

	// entryRequested — оператор скомандовал сход с орбиты.
	entryRequested bool

	// deorbitSettled — корпус пришёл к ретроградной ориентации.
	deorbitSettled bool

	// heatShield — теплозащита корабля: плитки с наветренной стороны
	// и голая сталь с подветренной.
	heatShield *HeatShield

	// throttleFloor — нижний предел дросселирования активной ступени, доля.
	// throttleLimited сообщает, что уставка оператора удержана на этом пределе.
	throttleFloor   float64
	throttleLimited bool

	// landingThrottle — уставка посадочного контура, доля номинала.
	// На посадке она заменяет команду наведения выведения.
	landingThrottle float64

	// stagingThrottle — уставка центральных камер на горячем разделении.
	// Наведение об этом участке не знает: для него ступень уже выключена.
	stagingThrottle float64

	// substepDebt — недосчитанная доля шага интегрирования.
	//
	// Число шагов за такт таймера почти никогда не целое, и остаток нельзя
	// ни отбрасывать, ни округлять: на замедленных скоростях из него состоит
	// вся разница между режимами.
	substepDebt float64

	// Итог касания: скорость и отклонение корпуса от вертикали в момент
	// контакта, а также признак опрокидывания.
	landingSpeed float64
	landingTilt  float64
	toppled      bool

	// manualAttitude — ориентация, назначенная оператором вручную.
	// Пока она задана, наведение своей команды не выдаёт.
	manualAttitude *physics.Attitude

	// board — пульт ручных воздействий. Единственный путь, которым внешнее
	// управление попадает в физическую модель.
	board *control.Board

	// runState — состояние хода симуляции.
	runState RunState

	// pendingSteps — число шагов, которые нужно выполнить на паузе.
	pendingSteps int

	// runNumber — порядковый номер прогона.
	runNumber int

	// scenario — название выбранного сценария воздействий.
	scenario string

	// startedAt — момент запуска в реальном времени.
	startedAt time.Time

	// haltedAt — момент, когда прогон перестал идти (пауза, остановка,
	// завершение). Пока прогон не остановлен, поле нулевое.
	//
	// RealTime в снимке считается не как «сейчас минус startedAt» всегда —
	// иначе после Stop оно продолжало бы расти вечно, хотя модельное время
	// уже замерло: снимок состояния оператор может запросить в любой
	// момент, в том числе спустя часы после остановки прогона.
	haltedAt time.Time

	// fuelSensor — датчик остатка горючего первой ступени. Настоящего
	// датчика массы не существует физически (см. DefaultPropellantSensor);
	// это единственное измерение, от которого зависит решение о выключении
	// двигателей по остатку топлива (shouldMECO) — оно не должно судить
	// по истинной массе в баке, которую в реальности борт не знает.
	fuelSensor *propulsion.Sensor

	// sensedFuelMass — последнее достоверное показание fuelSensor, кг.
	// На пропуске связи не обновляется: держит предыдущее показание, а не
	// откатывается к истинной массе и не проваливается в ноль.
	sensedFuelMass float64

	// positionSensor/velocitySensor — навигационные каналы положения и
	// скорости в ECI. Наведение (GNCSystem.Update) решает по их показаниям,
	// а не по истинному вектору состояния: реальный борт видит только то,
	// что говорит навигационная система, и умеет ошибаться так же, как она.
	positionSensor *sensing.VectorSensor
	velocitySensor *sensing.VectorSensor

	// lastValidSensedPosition/lastValidSensedVelocity — последнее достоверное
	// показание на случай пропуска связи. На пропуске наведение обязано
	// держать последний известный вектор состояния, а не откатываться
	// к истинному (тогда датчик был бы подделкой) и не проваливаться в ноль
	// (тогда наведение среагировало бы на выдуманный скачок).
	lastValidSensedPosition physics.Vec3
	lastValidSensedVelocity physics.Vec3

	// attitudeSensor — датчик ориентации корпуса (гироскоп/звёздный датчик).
	// Автопилот (attitudeError в attitude_dynamics.go) решает по его
	// показанию, а не по истинной ориентации: реальный борт не имеет
	// прямого доступа к своей истинной ориентации, только к тому, что
	// говорит гироскоп.
	attitudeSensor *sensing.AttitudeSensor

	// lastValidSensedOrientation — последнее достоверное показание на
	// случай пропуска связи. Держится неизменным на пропуске: ни отката
	// к истинной ориентации, ни проваливания в единичный кватернион.
	lastValidSensedOrientation physics.Quaternion

	// sensorRng — отдельный генератор для шума всех измерительных каналов
	// (двигательные датчики, остаток топлива, навигация). Не общий с rng:
	// иначе изменение характеристик датчика (число каналов, частота
	// обновления) сдвигало бы поток rng и меняло бы саму физическую
	// траекторию полёта — шум измерения не должен возвращаться в модель
	// даже косвенно, через общий генератор.
	sensorRng *rand.Rand

	rng  *rand.Rand
	stop chan struct{}
	done chan struct{}
}

// Dispersion — отклонения характеристик носителя от номинала.
//
// Ни один реальный пуск не совпадает с номиналом: тяга двигателей отличается
// от паспортной, плотность атмосферы в день пуска отличается от стандартной,
// заправка выполняется с допуском. Без этих разбросов все прогоны симулятора
// совпадают побитово, и статистику по выведению собрать невозможно.
type Dispersion struct {
	// ThrustFactor — множитель тяги, около 1.
	ThrustFactor float64

	// ISPFactor — множитель удельного импульса, около 1.
	ISPFactor float64

	// DensityFactor — множитель плотности атмосферы, около 1.
	DensityFactor float64

	// DryMassDelta — отклонение сухой массы, кг.
	DryMassDelta float64

	// FuelMassFactor — множитель заправки, около 1.
	FuelMassFactor float64
}

// NominalDispersion возвращает нулевые отклонения.
func NominalDispersion() Dispersion {
	return Dispersion{
		ThrustFactor:   1,
		ISPFactor:      1,
		DensityFactor:  1,
		FuelMassFactor: 1,
	}
}

// RandomDispersion разыгрывает отклонения по типичным допускам.
func RandomDispersion(rng *rand.Rand) Dispersion {
	return Dispersion{
		// Тяга двигателя воспроизводится примерно с точностью 0.5 %.
		ThrustFactor: 1 + rng.NormFloat64()*0.005,
		// Удельный импульс — 0.3 %.
		ISPFactor: 1 + rng.NormFloat64()*0.003,
		// Плотность атмосферы в конкретный день отличается от стандартной
		// на единицы процентов, в термосфере — заметно сильнее.
		DensityFactor: 1 + rng.NormFloat64()*0.04,
		// Точность взвешивания конструкции — сотни килограммов.
		DryMassDelta: rng.NormFloat64() * 150,
		// Заправка выполняется с допуском 0.2 %.
		FuelMassFactor: 1 + rng.NormFloat64()*0.002,
	}
}

// NewSimulation создаёт симуляцию с конфигурацией Falcon 9 и случайным seed.
func NewSimulation() *Simulation {
	return NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), time.Now().UnixNano())
}

// NewSimulationWithConfig создаёт симуляцию с заданной конфигурацией
// и случайным seed.
func NewSimulationWithConfig(cfg vehicle.Config, tc TimeConfig) *Simulation {
	return NewSimulationWithSeed(cfg, tc, time.Now().UnixNano())
}

// NewSimulationWithSeed создаёт симуляцию с заданным зерном генератора.
//
// Один и тот же seed полностью определяет полёт: профиль ветра, отклонения
// характеристик от номинала, начальную закрутку ступени при разделении
// и моменты отказов двигателей. Это позволяет воспроизвести любой прогон.
func NewSimulationWithSeed(cfg vehicle.Config, tc TimeConfig, seed int64) *Simulation {
	s := &Simulation{
		Config:    cfg,
		Time:      tc,
		seed:      seed,
		board:     control.NewBoard(),
		runState:  RunIdle,
		runNumber: 1,
		scenario:  "Штатный полёт",
		stop:      make(chan struct{}),
		done:      make(chan struct{}),
	}
	s.initState()
	s.calibrateBoard()
	return s
}

// Seed возвращает зерно генератора этого прогона.
func (s *Simulation) Seed() int64 {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.seed
}

// splitMix64 — детерминированное перемешивание 64-битного числа (алгоритм
// SplitMix64). Нужно только для того, чтобы развести из одного seed два
// независимых, но воспроизводимых потока ГСЧ — физику и шум датчиков, — не
// расходуя ни одного броска ни одного из этих потоков.
func splitMix64(x uint64) uint64 {
	x += 0x9E3779B97F4A7C15
	z := x
	z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9
	z = (z ^ (z >> 27)) * 0x94D049BB133111EB
	return z ^ (z >> 31)
}

// initState приводит симуляцию в состояние «на стартовом столе».
// Вызывается и из конструктора, и из Reset — состояние сбрасывается полностью,
// без забытых полей.
func (s *Simulation) initState() {
	cfg := s.Config

	// Генератор пересоздаётся от того же зерна, поэтому Reset даёт в точности
	// тот же полёт: тот же ветер, те же отклонения, те же отказы.
	s.rng = rand.New(rand.NewSource(s.seed))

	// sensorRng разводится от того же seed чистым арифметическим
	// перемешиванием (splitMix64), а не броском s.rng: если бы для этого
	// понадобился хотя бы один вызов s.rng.Int63(), сама эта развязка
	// сдвинула бы поток s.rng на один бросок и заново испортила
	// воспроизводимость прежних сидов. Перемешивание детерминировано:
	// тот же seed всегда даёт тот же поток шума датчиков, но он не пересекается
	// с потоком физической случайности.
	s.sensorRng = rand.New(rand.NewSource(int64(splitMix64(uint64(s.seed)))))

	s.wind = env.NewWindProfile(s.rng)
	s.dispersion = RandomDispersion(s.rng)

	// Положение стартового комплекса в инерциальной системе на момент t = 0.
	launch := physics.Geodetic{
		Latitude:  cfg.LaunchLatitude,
		Longitude: cfg.LaunchLongitude,
		Altitude:  0,
	}
	position := physics.ECEFToECI(physics.GeodeticToECEF(launch), 0)

	// Начальная скорость — вращение Земли. Учитывается ровно один раз:
	// это скорость точки старта в инерциальной системе. Ни в наземный трек,
	// ни в переход на орбиту она больше руками не добавляется.
	velocity := physics.CorotatingVelocity(position)

	ambient := physics.EffectiveEnvironmentTemperature(0, 0)

	s.state = VehicleState{
		Position: position,
		Velocity: velocity,
	}
	s.engines = vehicle.BuildFirstStageEngines(cfg, ambient)

	// Двигательная установка первой ступени. Конструкция первой ступени везёт
	// на себе всю вторую ступень с заправкой, нагрузку и обтекатель, поэтому
	// они входят в её сухую массу.
	s.propulsion = NewPropulsionSystem(StageProps{
		StructureMass: cfg.FirstStage.DryMass + s.dispersion.DryMassDelta +
			cfg.SecondStage.DryMass + cfg.SecondStage.FuelMass + cfg.FairingMass,
		PayloadMass:        cfg.PayloadMass,
		PropellantMass:     cfg.FirstStage.FuelMass * s.dispersion.FuelMassFactor,
		MixtureRatio:       cfg.FirstStage.MixtureRatio,
		EngineCount:        int(cfg.FirstStage.EngineCount),
		StagePrefix:        "S1",
		EngineConfig:       vehicle.FirstStageEngineConfig(cfg),
		FuelTankHeight:     cfg.FirstStage.FuelTankHeight,
		OxTankHeight:       cfg.FirstStage.OxTankHeight,
		FuelTankPressure:   cfg.FirstStage.FuelTankPressure,
		OxTankPressure:     cfg.FirstStage.OxTankPressure,
		FuelPressurantMass: cfg.FirstStage.FuelPressurantMass,
		OxPressurantMass:   cfg.FirstStage.OxPressurantMass,
	}, s.rng, s.sensorRng, ambient)

	s.dryMass = s.propulsion.DryMass()
	s.state.FuelMass = s.propulsion.PropellantMass()
	s.fuelSensor = propulsion.NewSensorAt(propulsion.DefaultPropellantSensor(s.state.FuelMass), s.state.FuelMass)
	s.sensedFuelMass = s.state.FuelMass
	s.positionSensor = sensing.NewVectorSensorAt(sensing.DefaultPositionSensor(), s.state.Position)
	s.velocitySensor = sensing.NewVectorSensorAt(sensing.DefaultVelocitySensor(), s.state.Velocity)
	s.lastValidSensedPosition = s.state.Position
	s.lastValidSensedVelocity = s.state.Velocity
	s.area = cfg.CrossSectionArea()
	s.prevAxialAccel = 0
	s.prevLateralAccel = 0
	s.prevAngularAccel = 0
	s.elapsed = 0
	s.startedAt = time.Time{}
	s.haltedAt = time.Time{}
	s.phase = orbit.PhasePreLaunch
	s.Entry = DefaultEntryConfig()
	s.entryRequested = false
	s.stage = 1
	gncConfig := orbit.DefaultGNCConfig(cfg.TargetOrbitAltitude, cfg.TargetInclination)
	gncConfig.MinThrottle = cfg.FirstStage.MinThrottle
	gncConfig.LaunchPadRoll = cfg.InitialRoll()
	if cfg.PitchProgramStretch > 0 {
		gncConfig.PitchProgramStretch = cfg.PitchProgramStretch
	}
	s.gnc = orbit.NewGNCSystem(gncConfig)
	s.maxQ = MaxQState{}
	s.fairingGone = false
	s.mecoTime = 0
	s.secoTime = 0
	s.circStart = 0
	s.crashed = false
	s.released = false
	s.orbitReached = false
	s.orbitCorrectionBurn = false
	s.throttleCmd = 1.0
	s.spentStage = nil

	// Ориентация на столе — вертикальная, с азимутом пуска. Дальше корпус
	// поворачивается только моментами.
	s.attitude = VehicleAttitude{Config: DefaultAttitudeControl()}
	azimuth, _ := physics.LaunchAzimuthForInclination(
		cfg.LaunchLatitude, cfg.TargetInclination, true)
	s.attitude.Init(physics.Attitude{
		Pitch: 90,
		Yaw:   azimuth,
		Roll:  cfg.InitialRoll(),
	}, physics.NewLocalFrame(position))
	s.attitudeSensor = sensing.NewAttitudeSensor(sensing.DefaultAttitudeSensor())
	s.lastValidSensedOrientation = s.attitude.Orientation
	s.telemetry = s.buildTelemetryLocked()
}

// Reset полностью возвращает симуляцию в исходное состояние.
func (s *Simulation) Reset() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.initState()
}

// Stop останавливает цикл симуляции.
func (s *Simulation) Stop() {
	select {
	case <-s.stop:
		// уже остановлена
	default:
		close(s.stop)
	}
}

// Wait ждёт завершения цикла симуляции, но не дольше timeout.
//
// Возвращает false, если цикл не успел завершиться. Ожидание ограничено
// намеренно: заблокировать обработчик HTTP навсегда из-за застрявшего цикла
// хуже, чем ответить, что остановка не подтверждена.
func (s *Simulation) Wait(timeout time.Duration) bool {
	select {
	case <-s.done:
		return true
	case <-time.After(timeout):
		return false
	}
}

// Started сообщает, был ли запущен цикл симуляции.
//
// Свежесозданная симуляция ещё не крутится, и ждать её завершения бессмысленно:
// канал done закрывается только выходом из Run.
func (s *Simulation) Started() bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.runState != RunIdle
}

// Finished сообщает, что цикл симуляции уже завершился.
func (s *Simulation) Finished() bool {
	select {
	case <-s.done:
		return true
	default:
		return false
	}
}

// Run выполняет цикл симуляции до остановки или падения ракеты.
//
// Реальное время и модельное разведены: за один такт таймера модель
// продвигается на Scale·TickInterval секунд модельного времени, разбитых
// на шаги по Time.Step. Пауза останавливает модельное время, но не публикацию
// телеметрии — оператору нужно видеть состояние остановленной модели.
func (s *Simulation) Run() {
	defer close(s.done)

	ticker := time.NewTicker(s.Time.TickInterval)
	defer ticker.Stop()

	s.mu.Lock()
	s.runState = RunRunning
	s.startedAt = time.Now()
	s.haltedAt = time.Time{}
	s.mu.Unlock()

	// Публикуем состояние на стартовом столе до первого шага интегрирования.
	// Так первая точка наземного трека приходится ровно на стартовую площадку,
	// а не на момент T+1 с, когда ракета уже сошла со стола.
	s.mu.RLock()
	initial := s.telemetry
	s.mu.RUnlock()
	initial.Publish()

	for {
		select {
		case <-s.stop:
			s.mu.Lock()
			s.runState = RunStopped
			s.haltedAt = time.Now()
			s.mu.Unlock()
			log.Println("Simulation stopped")
			return
		case <-ticker.C:
		}

		s.mu.Lock()

		// Скорость симуляции меняется на ходу, поэтому число шагов за такт
		// пересчитывается каждый раз, а не вычисляется однажды при запуске.
		//
		// Дробный остаток переносится на следующий такт. Без этого замедление
		// не работало бы вовсе: при десятикратном замедлении на такт приходится
		// одна десятая шага, округление давало ноль, а защита от нуля поднимала
		// его до целого шага — и модель шла с обычной скоростью.
		s.substepDebt += s.Time.Scale * s.Time.TickInterval.Seconds() / s.Time.Step
		substeps := int(s.substepDebt)
		s.substepDebt -= float64(substeps)

		switch s.runState {
		case RunPaused:
			// На паузе модель продвигается только по явной команде.
			substeps = s.pendingSteps
			s.pendingSteps = 0
		case RunRunning:
		default:
			substeps = 0
		}

		for i := 0; i < substeps; i++ {
			s.step(s.Time.Step)
			if s.crashed {
				break
			}
		}
		s.telemetry = s.buildTelemetryLocked()
		telemetry := s.telemetry
		crashed := s.crashed
		elapsed := s.elapsed
		board := s.board
		s.mu.Unlock()

		telemetry.Publish()

		// Пульт сравнивает телеметрию с той, что была до воздействия, и
		// определяет отклик по фактическим данным модели.
		if board != nil {
			sample := telemetry.controlSample()
			board.Observe(elapsed, sample)
			board.Record(elapsed, sample)
		}

		if crashed {
			s.mu.Lock()
			s.runState = RunEnded
			s.haltedAt = time.Now()
			s.mu.Unlock()
			log.Println("Simulation ended: vehicle impacted the surface")
			return
		}
	}
}

// -----------------------------------------------------------------------------
// Один шаг интегрирования
// -----------------------------------------------------------------------------

// step продвигает симуляцию на dt секунд. Вызывается под удержанным mu.
func (s *Simulation) step(dt float64) {
	if s.crashed {
		return
	}

	// 1. Навигация: полное состояние выводится из вектора состояния в ECI.
	nav := s.navState()

	// Показание датчика остатка топлива — раз за такт, до проверки условий
	// смены фазы. MECO по остатку горючего обязан судить о нём так же,
	// как судил бы борт, а не подсматривать в истинную массу в баке.
	// На пропуске связи показание просто не обновляется — держит предыдущее,
	// а не откатывается к истине и не проваливается в ноль.
	if v, valid := s.fuelSensor.UpdateWith(s.state.FuelMass, dt, s.sensorRng, control.SensorOverrides{}); valid {
		s.sensedFuelMass = v
	}

	// Показания навигационных датчиков (положение/скорость в ECI) — тоже раз
	// за такт, той же логикой держания последнего достоверного значения на
	// пропуске связи. Идут в наведение (см. ниже), а не в проверку смены фазы
	// и не в остальные использования nav в этом шаге (атмосфера, тепловой
	// поток, угол атаки для аэродинамики) — воздух не знает, что думает
	// навигационная система.
	if m := s.positionSensor.UpdateWith(s.state.Position, dt, s.sensorRng, control.SensorOverrides{}); m.Valid {
		s.lastValidSensedPosition = m.Value
	}
	if m := s.velocitySensor.UpdateWith(s.state.Velocity, dt, s.sensorRng, control.SensorOverrides{}); m.Valid {
		s.lastValidSensedVelocity = m.Value
	}

	// Показание датчика ориентации — той же логикой: раз за такт, держит
	// последнее достоверное значение на пропуске связи. Идёт только в
	// вычисление ошибки автопилота (attitudeError), а не в само
	// интегрирование истинной ориентации по угловой скорости.
	if m := s.attitudeSensor.UpdateWith(s.attitude.Orientation, dt, s.sensorRng, control.SensorOverrides{}); m.Valid {
		s.lastValidSensedOrientation = m.Orientation
	}

	// 2. Обновление фазы полёта.
	s.updateFlightPhase(nav)
	nav.Phase = s.phase

	// 3. Наведение: требуемые углы и уровень газа.
	//
	// На посадке тягой распоряжается посадочный контур, а не наведение
	// выведения: оно про эту задачу ничего не знает и на всякой работающей
	// фазе просит полный газ. Раньше так и было — посадочный контур считал
	// уставку, а следующей же строкой её затирала единица, и корабль летел
	// к земле на полной тяге, развёрнутый поперёк движения.
	// Датчик подменяет вход только для контуров выведения (тангаж по
	// апоцентру, азимут, довыведение) — их и разбирал план Фазы 3, и только
	// под них добавлен фильтр производной в PIDController. На посадочном
	// участке наведение попадает в ту же ветку computeTargetPitch
	// (progradePitch по вектору скорости), но там скорость мала, и шум
	// датчика скорости (абсолютный порог, не доля) даёт огромную ошибку
	// направления — ровно то, чем должна заниматься Фаза 4 (датчик
	// ориентации), а не эта. До неё посадочный и пассивные участки остаются
	// на истинной навигации.
	navForGuidance := nav
	switch s.phase {
	case orbit.PhaseFirstStage, orbit.PhaseSecondStage, orbit.PhaseCircularization:
		navForGuidance = s.sensedNavState(nav)
	}
	command := s.gnc.Update(navForGuidance, dt)
	if s.inLanding() {
		command.Throttle = s.landingThrottle
	}

	// Горячее разделение: наведение считает этот участок пассивным, а ступень
	// на нём продолжает работать тремя центральными камерами.
	if s.phase == orbit.PhaseCoast1 {
		command.Throttle = s.stagingThrottle
	}
	s.throttleCmd = command.Throttle

	// 4. Дросселирование двигателей с учётом Max-Q и ограничения перегрузки.
	atm := physics.Atmosphere(nav.Altitude)
	atm.Density *= s.dispersion.DensityFactor
	s.maxQ.Observe(nav.DynamicPressure, nav.Altitude, nav.AirRelativeVelocity.Norm(), s.elapsed)
	s.applyThrottle(command.Throttle, nav.DynamicPressure, atm.Pressure, dt)

	// 4.1. Двигательная установка.
	//
	// Тяга и расход не задаются, а вычисляются из состояния баков, оборотов
	// турбонасоса и давления в камере. Ускорение предыдущего шага задаёт
	// гидростатический напор в баках и возбуждает колебания жидкости.
	// Ручные воздействия оператора. Они входят в расчёт как условия работы
	// агрегатов, а не как подмена результата: дальше по цепочке всё считает
	// физическая модель.
	var overrides control.Overrides
	if s.board != nil {
		overrides = s.board.Advance(s.elapsed)
	}

	s.propulsion.Update(dt, s.throttleCmd, PropulsionEnvironment{
		Overrides:       overrides,
		AmbientPressure: atm.Pressure,
		StructureTemperature: physics.EffectiveEnvironmentTemperature(
			nav.Altitude, nav.AirRelativeVelocity.Norm()),
		AxialAcceleration:   s.prevAxialAccel,
		LateralAcceleration: s.prevLateralAccel,
		AngularAcceleration: s.prevAngularAccel,
		Gravity:             physics.GravityMagnitudeAtAltitude(nav.Altitude),
	})
	s.syncEngineTelemetry(atm)

	// 4.2. Угловое движение корпуса.
	//
	// Наведение назначает потребную ориентацию, но корпус приходит к ней
	// не мгновенно: автопилот отклоняет камеры, отклонённая тяга создаёт
	// момент, и уже он разворачивает ракету против аэродинамического момента
	// и инерции. Ориентация — результат интегрирования, а не команда.
	// На возвращении ориентацию задаёт не наведение выведения, а профиль
	// входа: торможение брюхом, затем разворот в вертикаль.
	target := command.Attitude()
	if s.inEntry() {
		target = s.entryTarget(nav)
	}
	if s.manualAttitude != nil {
		target = *s.manualAttitude
	}

	// Севший корабль ориентацией не управляет.
	//
	// Он стоит на опорах, и его положение определяет грунт, а не автопилот.
	// Прежде наведение продолжало работать и после касания: цель возвращалась
	// к профилю входа, то есть к полёту брюхом, и корабль на площадке
	// послушно ложился набок.
	if s.phase == orbit.PhaseLanded {
		s.holdLandedAttitude(dt, nav)
	} else {
		s.updateAttitude(dt, target, nav, atm, overrides.Control)
	}

	// Теплозащита: какая сторона подставлена потоку, та и греется.
	s.updateHeatShield(dt, navSnapshot{
		AirRelative: nav.AirRelativeVelocity,
		HeatFlux: physics.StagnationHeatFlux(atm.Density,
			nav.AirRelativeVelocity.Norm(), s.Config.Diameter/2),
	})

	// 5. Сборка силовой модели. Направление тяги задаётся фактической
	//    ориентацией корпуса с учётом отклонения камер.
	thrustDir := s.attitude.ThrustDirection()
	fm := ForceModel{
		DryMass:       s.dryMass,
		Thrust:        s.propulsion.TotalThrust,
		ThrustDir:     thrustDir,
		MassFlow:      s.propulsion.TotalFuelFlow + s.propulsion.TotalOxFlow,
		Area:          s.area,
		Wind:          s.windVelocity(s.state.Position, nav.Altitude),
		DensityFactor: s.dispersion.DensityFactor,

		// Площадь борта и фактический угол атаки: корпус, поставленный
		// поперёк потока, тормозит бортом, а не миделем.
		SideArea:      s.sideArea(),
		AngleOfAttack: s.bodyAngleOfAttack(nav.AirRelativeVelocity),
	}

	// 5.1. Удержание на стартовом столе.
	//
	// Двигатели выходят на режим не мгновенно: турбонасосу нужны секунды на
	// раскрутку. Всё это время носитель удерживается захватами и отпускается
	// только когда тяга уверенно превысила вес. Без этого ракета проваливалась
	// бы сквозь стартовое сооружение в первые доли секунды.
	mass := s.dryMass + s.state.FuelMass
	weight := mass * physics.GravityMagnitudeAtAltitude(nav.Altitude)
	if !s.released {
		if fm.Thrust > weight*1.05 {
			s.released = true
			log.Printf("🔓 Отрыв на T+%.1f с: тяга %.2f МН при весе %.2f МН",
				s.elapsed, fm.Thrust/1e6, weight/1e6)
		} else {
			// Ракета стоит на столе: расход топлива идёт, но движения нет.
			//
			// Положение при этом не остаётся постоянным в инерциальной системе:
			// стартовый стол вращается вместе с Землёй. Если этого не учесть,
			// координаты ракеты начнут уползать относительно площадки ещё
			// до отрыва.
			s.state.FuelMass = s.propulsion.PropellantMass()
			s.elapsed += dt

			pad := physics.Geodetic{
				Latitude:  s.Config.LaunchLatitude,
				Longitude: s.Config.LaunchLongitude,
			}
			s.state.Position = physics.ECEFToECI(physics.GeodeticToECEF(pad), s.elapsed)
			s.state.Velocity = physics.CorotatingVelocity(s.state.Position)

			s.updateEngineThermal(dt, nav, atm)
			s.prevAxialAccel = 0
			return
		}
	}

	// 6. Интегрирование методом Рунге — Кутты 4-го порядка.
	next := RK4Step(s.state, fm, dt)
	if !next.Position.IsFinite() || !next.Velocity.IsFinite() {
		log.Println("⚠️ Численная неустойчивость: шаг отброшен")
		s.crashed = true
		return
	}
	s.state = next
	s.elapsed += dt

	// Масса компонентов — величина, выведенная из баков, а не из состояния
	// интегратора. Баки учитывают ещё и утечку, поэтому именно они являются
	// источником истины.
	s.state.FuelMass = s.propulsion.PropellantMass()

	// Ускорения для следующего шага: гидростатический напор и возбуждение
	// колебаний жидкости считаются по ним.
	//
	// Осевое ускорение — это составляющая вдоль корпуса, а не вдоль местной
	// вертикали. На выведении ракета стоит носом вверх, и разницы нет; на
	// возвращении корабль летит боком, и разница решает всё. Прежде здесь
	// брали вертикаль, и на развороте перед посадкой модель считала, что
	// компоненты в баках не прижаты ни к чему: напор на входе насосов падал,
	// давление в камере рушилось, и двигатели гасли на трёхстах метрах.
	//
	// Прижимает компоненты не гравитация, а всё остальное: тяга и торможение.
	// В свободном падении осадка нет, и это правильно — потому настоящий
	// корабль и держит расходные баки под наддувом.
	acc := fm.Evaluate(s.state)
	apparent := acc.Total.Sub(acc.Gravity)
	forward := s.attitude.Orientation.Rotate(physics.Vec3{X: 1})

	axial := apparent.Dot(forward)
	s.prevAxialAccel = math.Max(0, axial)
	s.prevLateralAccel = apparent.Sub(forward.Scale(axial)).Norm()
	s.prevAngularAccel = s.gnc.PitchRate * math.Pi / 180 / math.Max(dt, 1e-6)

	// 7. Тепловое состояние наружной обшивки двигателей.
	s.updateEngineThermal(dt, nav, atm)

	// 8. Сброс головного обтекателя.
	s.checkFairingJettison()

	// 9. Случайные отказы и события при малом остатке топлива.
	s.applyRandomEvents(dt)

	// 10. Отработавшая ступень летит своей траекторией.
	if s.spentStage != nil {
		wasAlive := s.spentStage.Alive()
		s.spentStage.Step(dt, s.elapsed)
		if wasAlive && !s.spentStage.Alive() {
			s.logSpentStageOutcome()
		}
	}

	// 11. Проверка столкновения с поверхностью.
	//
	// Приговор выносит посадочный контур: касание с малой скоростью — это
	// посадка, а не падение. Общая проверка ставила крушение всякий раз,
	// когда высота доходила до нуля, и мягкое касание записывалось в аварию.
	if s.state.Altitude() <= 0 && s.phase != orbit.PhasePreLaunch {
		// Скорость касания снимается до того, как корпус ставится на грунт:
		// после обнуления судить уже не о чем. Прежде приговор выносился
		// следующим шагом, когда скорость была нулевой, и удар о землю
		// на семидесяти метрах в секунду записывался как мягкая посадка.
		descent := -s.state.RadialVelocity()
		landing := s.inLanding()

		s.state.Position = s.state.Position.Unit().Scale(physics.EarthRadius)
		s.state.Velocity = physics.CorotatingVelocity(s.state.Position)
		s.shutdownAllEngines()

		switch {
		case landing:
			s.finishLanding(descent)
		case s.phase != orbit.PhaseLanded:
			s.crashed = true
		}
	}
}

// windVelocity возвращает скорость ветра в точке, в осях ECI.
func (s *Simulation) windVelocity(position physics.Vec3, altitude float64) physics.Vec3 {
	if s.wind == nil {
		return physics.Vec3{}
	}
	return s.wind.Velocity(position, altitude, s.elapsed)
}

// navState собирает состояние навигации.
func (s *Simulation) navState() orbit.NavState {
	mass := s.dryMass + s.state.FuelMass
	available := s.availableThrust()
	altitude := s.state.Altitude()
	return orbit.NewNavState(
		s.state.Position, s.state.Velocity,
		s.windVelocity(s.state.Position, altitude),
		mass, available, s.currentStage().VacuumISP, s.elapsed, s.phase,
	)
}

// sensedNavState собирает NavState из показаний навигационных датчиков —
// то, чем на самом деле распоряжается наведение.
//
// Положение и скорость берутся с датчиков (либо последнее достоверное
// значение, если сейчас пропуск связи), а масса, доступная тяга, удельный
// импульс, время и фаза полёта остаются истинными: это не измеряемые
// датчиком величины, а внутренний учёт борта (масса — по расходу и
// заправке, удельный импульс — паспортная характеристика двигателя, фаза —
// по собственной логике полётной программы). Ветер вычисляется в точке
// показанного датчиком положения — так же, как атмосферная модель
// в реальности видела бы обстановку там, где, по её мнению, находится
// ракета, а не там, где она находится на самом деле.
func (s *Simulation) sensedNavState(trueNav orbit.NavState) orbit.NavState {
	position := s.lastValidSensedPosition
	velocity := s.lastValidSensedVelocity
	altitude := position.Norm() - physics.EarthRadius
	return orbit.NewNavState(
		position, velocity,
		s.windVelocity(position, altitude),
		trueNav.Mass, trueNav.AvailableThrust, trueNav.ISP, trueNav.Time, trueNav.Phase,
	)
}

// availableThrust возвращает суммарную номинальную тягу работающих двигателей.
func (s *Simulation) availableThrust() float64 {
	total := 0.0
	for i := range s.engines {
		if s.engines[i].Running {
			total += s.engines[i].MaxThrust
		}
	}
	return total
}

// applyThrottle распределяет команду газа по двигателям.
func (s *Simulation) applyThrottle(command, dynamicPressure, ambientPressure, dt float64) {
	stage := s.currentStage()

	// Ограничение по скоростному напору действует только в атмосфере.
	target := command
	if q := MaxQThrottle(dynamicPressure); q < target {
		target = q
	}
	target = physics.Clamp(target, 0, 1)

	// Нижний предел дросселирования — свойство камеры, а не пожелание.
	// Ниже него турбонасос не удерживает давление, форсунки перестают
	// распылять как надо, и горение срывается. У Raptor аттестованный
	// диапазон — от сорока процентов до номинала, у Merlin примерно так же.
	//
	// Поэтому уставка ниже предела не проходит «как есть»: она удерживается
	// на пределе, а не тихо гасит камеру. Прежде команда уходила в модель
	// двигателя нетронутой, давление в камере падало ниже четверти
	// номинального, и двигатель гас — оператор же видел только то, что
	// подвинул ползунок на пятнадцать процентов.
	s.throttleFloor = stage.MinThrottle
	s.throttleLimited = false
	if target > 0 && target < stage.MinThrottle {
		target = stage.MinThrottle
		s.throttleLimited = true
	}

	// Плавное изменение газа: 30 % в секунду.
	s.throttleCmd = SmoothThrottleTransition(s.throttleCmd, target, 0.30, dt)

	nominal := vehicle.ThrustAtAltitude(stage, ambientPressure)
	minThrust := nominal * stage.MinThrottle

	for i := range s.engines {
		e := &s.engines[i]
		if !e.Running {
			e.Thrust = 0
			continue
		}
		e.MaxThrust = nominal
		e.MinThrust = minThrust
		if s.throttleCmd <= 0 {
			e.Thrust = 0
			continue
		}
		e.Thrust = physics.Clamp(nominal*s.throttleCmd, minThrust, nominal)
	}
}

// currentStage возвращает конфигурацию активной ступени.
func (s *Simulation) currentStage() vehicle.StageConfig {
	if s.stage == 1 {
		return s.Config.FirstStage
	}
	return s.Config.SecondStage
}

// checkFairingJettison сбрасывает головной обтекатель по достижении высоты,
// на которой аэродинамический нагрев уже безопасен для нагрузки.
func (s *Simulation) checkFairingJettison() {
	if s.fairingGone || s.state.Altitude() < s.Config.FairingJettisonAltitude {
		return
	}
	if s.phase != orbit.PhaseSecondStage {
		return
	}
	s.fairingGone = true
	s.dryMass -= s.Config.FairingMass
	if s.propulsion != nil {
		s.propulsion.StructureMass -= s.Config.FairingMass
	}
	log.Printf("📦 Сброс головного обтекателя на T+%.1f с, высота %.1f км",
		s.elapsed, s.state.Altitude()/1000)
}

// applyRandomEvents моделирует отказы двигателей.
// Вероятности заданы в расчёте на секунду и приводятся к текущему шагу,
// иначе изменение шага интегрирования меняло бы частоту аварий.
func (s *Simulation) applyRandomEvents(dt float64) {
	if !s.phase.Powered() {
		return
	}

	// Опасен не малый остаток сам по себе, а близкая выработка.
	//
	// Доля от заправки для этого не годится: корабль садится, имея в баках
	// проценты от стартового запаса, и это его штатное состояние, а не отказ
	// на подходе. Смотреть надо на время до опустошения при текущем расходе —
	// именно тогда заборное устройство начинает хватать газ.
	lowFuel := false
	if s.propulsion != nil {
		const margin = 8.0 // с
		if t := s.propulsion.FuelTank.DepletionTime(); t < margin {
			lowFuel = true
		}
		if t := s.propulsion.OxTank.DepletionTime(); t < margin {
			lowFuel = true
		}
	}

	const (
		failureRatePerSecond        = 1.5e-4
		lowFuelFailureRatePerSecond = 5.0e-3
	)

	rate := failureRatePerSecond
	if lowFuel {
		rate = lowFuelFailureRatePerSecond
	}

	// Отказ имеет смысл только если останется чем лететь.
	if vehicle.RunningEngines(s.engines) < 2 {
		return
	}

	if s.rng.Float64() >= rate*dt {
		return
	}

	running := make([]int, 0, len(s.engines))
	for i := range s.engines {
		if s.engines[i].Running {
			running = append(running, i)
		}
	}
	idx := running[s.rng.Intn(len(running))]
	s.engines[idx].Running = false
	s.engines[idx].Thrust = 0
	log.Printf("🚨 Отказ двигателя %d на T+%.1f с, высота %.1f км",
		idx+1, s.elapsed, s.state.Altitude()/1000)
	vehicle.BalanceEngines(s.engines, idx, s.rng.Float64)
}

func (s *Simulation) shutdownAllEngines() {
	for i := range s.engines {
		s.engines[i].Running = false
		s.engines[i].Thrust = 0
	}
	if s.propulsion != nil {
		s.propulsion.StopAll()
	}
}

// startAllEngines запускает все двигатели активной ступени.
func (s *Simulation) startAllEngines() {
	for i := range s.engines {
		s.engines[i].Running = true
	}
	if s.propulsion != nil {
		s.propulsion.StartAll()
	}
}

// logSpentStageOutcome записывает в журнал судьбу отработавшей ступени.
func (s *Simulation) logSpentStageOutcome() {
	st := s.spentStage
	switch st.Outcome {
	case env.StageBrokenUp:
		log.Printf("💥 Первая ступень разрушилась на T+%.1f с, высота %.1f км (%s)",
			st.OutcomeTime, st.OutcomeAltitude/1000, st.OutcomeReason)
		log.Printf("   Пиковый напор %.1f кПа, поток %.1f кВт/м², вращение %.1f °/с",
			st.PeakDynamicQ/1000, st.PeakHeatFlux/1000, st.PeakSpinRate*180/math.Pi)
		for i := range st.Nodes {
			n := &st.Nodes[i]
			log.Printf("   Узел %-5s: максимум %.0f °C при пределе %.0f °C",
				n.Name, physics.KelvinToCelsius(n.PeakTemp),
				physics.KelvinToCelsius(n.MeltingPoint))
		}
		if d := st.Debris; d != nil {
			log.Printf("   Обломки: зона %.1f × %.1f км, центр %.4f°, %.4f°, "+
				"до поверхности дошло %.0f %% массы",
				d.SemiMajorAxis*2/1000, d.SemiMinorAxis*2/1000,
				d.CenterLatitude, d.CenterLongitude,
				d.SurvivingMassFraction*100)
			for _, f := range d.Impacts {
				if f.Reached {
					log.Printf("     %-18s β=%.0f кг/м² → %.4f°, %.4f° (%.1f км, %.0f м/с)",
						f.Class.Name, f.Class.BallisticCoefficient(),
						f.Latitude, f.Longitude, f.Downrange/1000, f.ImpactSpeed)
				} else {
					log.Printf("     %-18s β=%.0f кг/м² → разрушился в атмосфере",
						f.Class.Name, f.Class.BallisticCoefficient())
				}
			}
		}
	case env.StageImpacted:
		log.Printf("🌊 Первая ступень упала на T+%.1f с: %.4f°, %.4f°",
			st.OutcomeTime, st.OutcomeLatitude, st.OutcomeLongitude)
		log.Printf("   Апогей после разделения %.1f км, пиковый напор %.1f кПа",
			st.MaxAltitude/1000, st.PeakDynamicQ/1000)
	}
}

func machNumber(speed, soundSpeed float64) float64 {
	if soundSpeed <= 0 {
		return 0
	}
	return speed / soundSpeed
}

// Layout возвращает обводы носителя текущего прогона.
//
// Геометрия принадлежит носителю, а не симуляции, поэтому считает её пакет
// vehicle. Здесь только берётся конфигурация того изделия, которое летит
// прямо сейчас, — иначе после смены профиля чертёж разошёлся бы с моделью.
func (s *Simulation) Layout() vehicle.Layout {
	s.mu.RLock()
	cfg := s.Config
	s.mu.RUnlock()

	return vehicle.NewLayout(cfg)
}
