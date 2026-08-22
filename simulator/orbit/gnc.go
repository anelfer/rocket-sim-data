package orbit

import (
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Система управления полётом (GNC).
//
// Navigation — оценка текущего состояния.
// Guidance    — расчёт требуемого направления тяги и уровня дросселирования.
// Control     — отработка команд с учётом ограничений на скорость разворота.
//
// Ключевое отличие от прежней реализации: контуры действительно замкнуты.
// Угол тангажа на вакуумном участке считает ПИД-регулятор по ошибке высоты
// апоцентра, азимут — регулятор по ошибке наклонения, а не таблица вида
// «высота 3–50 км → тангаж от 80° до 30° линейно».
// -----------------------------------------------------------------------------

// NavState — результат работы навигации на текущем шаге.
type NavState struct {
	Time     float64
	Position physics.Vec3
	Velocity physics.Vec3

	Altitude            float64
	RadialVelocity      float64      // вертикальная составляющая, м/с
	TransverseVelocity  float64      // горизонтальная составляющая (инерциальная), м/с
	AirRelativeVelocity physics.Vec3 // скорость относительно воздуха, с учётом ветра

	// GroundRelativeVelocity — скорость относительно поверхности, без ветра.
	//
	// Именно она задаёт форму траектории и служит сигналом для гравитационного
	// разворота. Скорость относительно воздуха для этого не годится: на старте
	// ракета почти неподвижна относительно земли, и вектор скорости
	// относительно воздуха равен просто ветру — то есть направлен горизонтально.
	// Наведение по такому сигналу опрокидывает ракету сразу после отрыва.
	GroundRelativeVelocity physics.Vec3

	Geodetic physics.Geodetic
	Frame    physics.LocalFrame
	Elements physics.OrbitalElements

	DynamicPressure float64
	Mass            float64
	AvailableThrust float64

	Phase FlightPhase
}

// NewNavState собирает состояние навигации из вектора состояния в ECI.
//
// wind — скорость воздуха относительно вращающейся Земли. Она обязана входить
// в скорость относительно потока: именно от неё зависят число Маха, скоростной
// напор и угол атаки. Сдвиг ветра на участке максимального напора — основной
// источник поперечной нагрузки на корпус.
func NewNavState(
	position, velocity, wind physics.Vec3,
	mass, availableThrust, t float64,
	phase FlightPhase,
) NavState {
	up := position.Unit()
	radial := velocity.Dot(up)
	transverse := velocity.Sub(up.Scale(radial)).Norm()

	altitude := position.Norm() - physics.EarthRadius
	ground := velocity.Sub(physics.CorotatingVelocity(position))
	vRel := ground.Sub(wind)
	atm := physics.Atmosphere(altitude)

	return NavState{
		Time:                   t,
		Position:               position,
		Velocity:               velocity,
		Altitude:               altitude,
		RadialVelocity:         radial,
		TransverseVelocity:     transverse,
		AirRelativeVelocity:    vRel,
		GroundRelativeVelocity: ground,
		Geodetic:               physics.ECIToGeodetic(position, t),
		Frame:                  physics.NewLocalFrame(position),
		Elements:               physics.ElementsFromState(position, velocity, t),
		DynamicPressure:        physics.DynamicPressure(atm.Density, vRel.Norm()),
		Mass:                   mass,
		AvailableThrust:        availableThrust,
		Phase:                  phase,
	}
}

// GuidanceCommand — выход системы наведения.
type GuidanceCommand struct {
	Pitch    float64 // угол места продольной оси, градусы над местным горизонтом
	Azimuth  float64 // азимут продольной оси, градусы от севера
	Roll     float64 // крен вокруг продольной оси, градусы
	Throttle float64 // уровень дросселирования, 0…1
}

// Attitude возвращает команду в виде ориентации корпуса.
func (c GuidanceCommand) Attitude() physics.Attitude {
	return physics.Attitude{Pitch: c.Pitch, Yaw: c.Azimuth, Roll: c.Roll}
}

// GNCConfig — настройки системы управления.
type GNCConfig struct {
	TargetOrbitAltitude float64 // м
	TargetInclination   float64 // градусы

	VerticalRiseAltitude float64 // высота вертикального участка, м
	PitchKickAltitude    float64 // высота окончания начального доворота, м
	PitchKickAngle       float64 // величина начального доворота, градусы

	// AtmosphericQThreshold — скоростной напор, ниже которого управление
	// переходит с гравитационного разворота на замкнутый контур по апоцентру.
	AtmosphericQThreshold float64 // Па

	// RateLimits — предельные угловые скорости корпуса по трём осям, град/с.
	RateLimits physics.RateLimits

	// RollProgramAltitude — высота завершения ролл-программы, м.
	// Реальный носитель после отрыва разворачивается вокруг продольной оси,
	// совмещая плоскость тангажа с плоскостью выведения: стартовое сооружение
	// ориентировано жёстко, а азимут пуска от него не зависит.
	RollProgramAltitude float64

	// LaunchPadRoll — крен на стартовом столе, градусы. Задаётся ориентацией
	// пускового устройства.
	LaunchPadRoll float64

	// MaxAxialAcceleration — ограничение продольной перегрузки, м/с².
	// Реальные носители дросселируют двигатели к концу работы ступени,
	// чтобы не превысить допустимую нагрузку на конструкцию и нагрузку.
	MaxAxialAcceleration float64

	// MaxAngleOfAttack — абсолютный предел угла атаки, градусы.
	MaxAngleOfAttack float64

	// MaxQAlpha — предельное произведение скоростного напора на угол атаки,
	// Па·град.
	//
	// Поперечная нагрузка на корпус пропорциональна именно произведению q·α,
	// а не углу самому по себе. Ограничение на один только угол либо избыточно
	// жёсткое в разрежённом воздухе, либо недостаточное у земли. При жёстком
	// пределе в 5° ветер на малой высоте продавливал команду наведения вслед
	// за потоком, разворот шёл слишком рано и траектория заваливалась.
	MaxQAlpha float64

	MinThrottle float64

	// NominalThrottle — потолок командного газа в штатном полёте, доля
	// номинала.
	//
	// Паспортные сто процентов — это испытательный, а не полётный режим:
	// в реальных пусках дроссель не задирают выше 93.5 % даже на участке,
	// где наведение просит «полный газ». Разница — не пожелание, а запас,
	// который остаётся строго для одного случая: компенсации отказавшего
	// двигателя оставшимися. Тот контур работает уровнем ниже, на ступени
	// двигательной установки (perEngine = throttle·intended/running), и
	// раздаёт исправным камерам именно этот запас между потолком наведения
	// и аттестованным пределом форсирования камеры (обычно 108 %). Заложи
	// наведение сразу сто процентов — раздавать после отказа было бы нечего,
	// компенсация упиралась бы в тот же потолок, что и штатная работа.
	NominalThrottle float64

	// PitchProgramStretch — растяжение программы тангажа по высоте.
	//
	// Таблица тангажа привязана к высоте, а высота — это не время и не
	// скорость. Носитель с меньшей стартовой тяговооружённостью набирает
	// ту же высоту заметно медленнее и с меньшей скоростью, поэтому на
	// той же отметке ему рано ложиться в горизонт: вектор скорости ещё
	// почти вертикален, и корпус пойдёт под большим углом атаки.
	//
	// Множитель растягивает таблицу: на двойке программа отрабатывается
	// вдвое выше по высоте, то есть разворот идёт вдвое положе.
	PitchProgramStretch float64
}

// DefaultGNCConfig возвращает настройки для выведения на низкую орбиту.
func DefaultGNCConfig(targetAltitude, targetInclination float64) GNCConfig {
	return GNCConfig{
		TargetOrbitAltitude:   targetAltitude,
		TargetInclination:     targetInclination,
		VerticalRiseAltitude:  400,
		PitchKickAltitude:     2000,
		PitchKickAngle:        13.0,
		AtmosphericQThreshold: 1500,
		RateLimits: physics.RateLimits{
			Pitch: 1.5,
			Yaw:   2.0,
			Roll:  8.0,
		},
		RollProgramAltitude:  6000,
		LaunchPadRoll:        0,
		MaxAxialAcceleration: 4.5 * physics.G0,
		MaxAngleOfAttack:     25.0,
		// В районе максимального напора (34 кПа) допускается около 5°.
		MaxQAlpha:           170000,
		MinThrottle:         0.40,
		NominalThrottle:     0.935,
		PitchProgramStretch: 1.0,
	}
}

// GNCSystem — система управления полётом.
type GNCSystem struct {
	Config GNCConfig

	// Замкнутые контуры.
	ApoapsisPitch   PIDController // тангаж по ошибке высоты апоцентра
	AltitudeHold    PIDController // тангаж по вертикальной скорости на целевой высоте
	RadialHold      PIDController // тангаж по радиальной скорости при довыведении
	InclinationTrim PIDController // азимут по ошибке наклонения
	CircThrottle    PIDController // дросселирование при довыведении

	// Attitude — текущая отработанная ориентация корпуса.
	Attitude physics.Attitude

	// PreviousAttitude нужна для вычисления фактических угловых скоростей.
	PreviousAttitude physics.Attitude

	// Угловые скорости корпуса, град/с.
	PitchRate float64
	YawRate   float64
	RollRate  float64

	// Аэродинамические углы, градусы.
	AngleOfAttack float64
	SideslipAngle float64
	TotalAoA      float64

	// ascending — латч направления движения по широте.
	// Определяется по знаку северной составляющей скорости с гистерезисом:
	// прежняя схема сравнивала текущую широту с предыдущей и намертво
	// залипала в точке максимальной широты, где широта перестаёт меняться.
	ascending bool

	// Диагностика.
	LastCommand     GuidanceCommand
	TargetApoapsis  float64
	InclinationDeg  float64
	AzimuthAchieved bool
}

// NewGNCSystem создаёт систему управления.
func NewGNCSystem(cfg GNCConfig) *GNCSystem {
	g := &GNCSystem{
		Config: cfg,

		// Выход — абсолютный угол тангажа, вход — ошибка высоты апоцентра.
		// Сто километров недобора дают примерно пятнадцать градусов
		// подъёма носа.
		ApoapsisPitch: PIDController{
			Kp:            1.5e-4,
			Ki:            1.0e-7,
			Kd:            1.5e-3,
			MinOutput:     -12,
			MaxOutput:     42,
			IntegralLimit: 5,
		},

		// Удержание высоты на разгонном участке. Вход — ошибка вертикальной
		// скорости, выход — абсолютный угол тангажа.
		//
		// Ход намеренно широкий. Пока скорость заметно ниже первой космической,
		// центробежная разгрузка мала, и удержать высоту можно только заметным
		// подъёмом носа: при тяговооружённости полтора это градусов тридцать
		// пять. Узкий предел в четырнадцать градусов означал, что ступень
		// удержать высоту не может в принципе, — она снижалась с работающими
		// двигателями и на низкой цели доходила до земли.
		AltitudeHold: PIDController{
			Kp:            0.10,
			Ki:            4.0e-4,
			Kd:            0.03,
			MinOutput:     -20,
			MaxOutput:     40,
			IntegralLimit: 20,
		},

		RadialHold: PIDController{
			// 100 м/с снижения дают примерно +2° к тангажу.
			Kp:            0.02,
			Ki:            1.0e-4,
			Kd:            0.05,
			MinOutput:     -15,
			MaxOutput:     15,
			IntegralLimit: 5,
		},

		InclinationTrim: PIDController{
			// 1° ошибки наклонения даёт примерно 3° поправки к азимуту.
			Kp:            3.0,
			Ki:            0.01,
			Kd:            1.0,
			MinOutput:     -15,
			MaxOutput:     15,
			IntegralLimit: 5,
		},

		CircThrottle: PIDController{
			// 200 м/с дефицита скорости — потолок газа (см. NominalThrottle
			// в computeThrottle; предел задан здесь же, а не только внешним
			// отсечением результата, — иначе интеграл продолжал бы копиться
			// к паспортным ста процентам, которых регулятору всё равно
			// никогда не отдадут).
			Kp:            0.005,
			Ki:            0.0002,
			Kd:            0.001,
			MinOutput:     0,
			MaxOutput:     nominalThrottleOr1(cfg),
			IntegralLimit: 0.5,
		},

		Attitude: physics.Attitude{
			Pitch: 90,
			Yaw:   90,
			Roll:  cfg.LaunchPadRoll,
		},
		ascending:      true,
		TargetApoapsis: cfg.TargetOrbitAltitude,
	}
	g.PreviousAttitude = g.Attitude
	return g
}

// Reset возвращает систему в исходное состояние.
func (g *GNCSystem) Reset() {
	g.ApoapsisPitch.Reset()
	g.AltitudeHold.Reset()
	g.RadialHold.Reset()
	g.InclinationTrim.Reset()
	g.CircThrottle.Reset()
	g.Attitude = physics.Attitude{Pitch: 90, Yaw: 90, Roll: g.Config.LaunchPadRoll}
	g.PreviousAttitude = g.Attitude
	g.PitchRate, g.YawRate, g.RollRate = 0, 0, 0
	g.AngleOfAttack, g.SideslipAngle, g.TotalAoA = 0, 0, 0
	g.ascending = true
	g.LastCommand = GuidanceCommand{}
}

// Update выполняет полный цикл наведения и возвращает отработанную команду.
func (g *GNCSystem) Update(nav NavState, dt float64) GuidanceCommand {
	target := physics.Attitude{
		Pitch: g.computeTargetPitch(nav, dt),
		Yaw:   g.computeTargetAzimuth(nav, dt),
		Roll:  g.computeTargetRoll(nav),
	}
	throttle := g.computeThrottle(nav, dt)

	// В плотных слоях команда ограничивается по углу атаки: корпус не выдержит
	// большого угла при высоком скоростном напоре. Ограничение работает как
	// подтягивание команды к направлению набегающего потока.
	target = g.limitAngleOfAttack(nav, target)

	// Control: корпус разворачивается с конечной угловой скоростью.
	g.PreviousAttitude = g.Attitude
	g.Attitude = g.Attitude.Approach(target, g.Config.RateLimits, dt)
	g.PitchRate, g.YawRate, g.RollRate = g.Attitude.AngularRates(g.PreviousAttitude, dt)

	// Аэродинамические углы считаются по фактической ориентации корпуса.
	body := g.Attitude.BodyFrame(nav.Frame)
	g.AngleOfAttack, g.SideslipAngle = physics.AeroAngles(body, nav.AirRelativeVelocity)
	g.TotalAoA = physics.AngleOfAttack(body.Forward, nav.AirRelativeVelocity)

	g.LastCommand = GuidanceCommand{
		Pitch:    g.Attitude.Pitch,
		Azimuth:  g.Attitude.Yaw,
		Roll:     g.Attitude.Roll,
		Throttle: throttle,
	}
	return g.LastCommand
}

// limitAngleOfAttack подтягивает команду ориентации к направлению набегающего
// потока, если требуемый угол атаки превышает допустимый.
//
// Ограничение действует только там, где есть заметный скоростной напор:
// в вакууме угол атаки не создаёт нагрузок и смысла не имеет.
func (g *GNCSystem) limitAngleOfAttack(nav NavState, target physics.Attitude) physics.Attitude {
	cfg := g.Config
	if cfg.MaxAngleOfAttack <= 0 || nav.DynamicPressure < 100 {
		return target
	}
	if nav.AirRelativeVelocity.Norm() < minGuidanceSpeed {
		return target
	}

	// Предел по поперечной нагрузке: α_доп = (q·α)_доп / q.
	// В разрежённом воздухе ограничение фактически снимается, у земли
	// и в районе максимального напора — становится жёстким.
	effective := cfg.MaxAngleOfAttack
	if cfg.MaxQAlpha > 0 {
		effective = math.Min(effective, cfg.MaxQAlpha/nav.DynamicPressure)
	}

	flowPitch, flowYaw := nav.Frame.PitchAzimuth(nav.AirRelativeVelocity)

	target.Pitch = flowPitch + physics.Clamp(target.Pitch-flowPitch, -effective, effective)
	deltaYaw := physics.ShortestAngle(target.Yaw - flowYaw)
	target.Yaw = physics.NormalizeAzimuth(flowYaw + physics.Clamp(deltaYaw, -effective, effective))
	return target
}

// -----------------------------------------------------------------------------
// Guidance: крен
// -----------------------------------------------------------------------------

// computeTargetRoll реализует ролл-программу.
//
// Стартовое сооружение ориентировано жёстко, а азимут пуска от него не зависит,
// поэтому сразу после отрыва носитель разворачивается вокруг продольной оси,
// совмещая плоскость тангажа с плоскостью выведения. После завершения
// программы крен удерживается нулевым относительно плоскости полёта.
func (g *GNCSystem) computeTargetRoll(nav NavState) float64 {
	cfg := g.Config

	if nav.Phase == PhasePreLaunch || nav.Altitude < cfg.VerticalRiseAltitude {
		return cfg.LaunchPadRoll
	}

	if nav.Altitude >= cfg.RollProgramAltitude {
		return 0
	}

	progress := (nav.Altitude - cfg.VerticalRiseAltitude) /
		(cfg.RollProgramAltitude - cfg.VerticalRiseAltitude)
	return cfg.LaunchPadRoll * (1 - progress)
}

// -----------------------------------------------------------------------------
// Guidance: тангаж
// -----------------------------------------------------------------------------

// minGuidanceSpeed — скорость, ниже которой направление вектора скорости
// не является пригодным сигналом наведения.
//
// При скорости в десятки метров в секунду угол наклона траектории определяется
// не движением ракеты, а ветром и шумом, и замыкать по нему контур нельзя.
const minGuidanceSpeed = 60.0

// progradePitch возвращает угол наклона траектории.
//
// Сигналом служит скорость ОТНОСИТЕЛЬНО ПОВЕРХНОСТИ, а не относительно воздуха.
// Разница принципиальна на старте: ракета почти неподвижна относительно земли,
// поэтому её скорость относительно воздуха равна ветру и направлена
// горизонтально. Наведение по такому вектору командует немедленный поворот
// к горизонту, ракета опрокидывается и падает.
//
// Ветер при этом никуда не девается: он входит в сопротивление, в скоростной
// напор и в угол атаки, а ограничение по углу атаки не даёт корпусу
// развернуться поперёк потока.
func (g *GNCSystem) progradePitch(nav NavState) float64 {
	v := nav.GroundRelativeVelocity
	if v.Norm() < minGuidanceSpeed {
		// Вектор скорости ещё не установился — держим вертикаль.
		return 90
	}
	pitch, _ := nav.Frame.PitchAzimuth(v)
	return pitch
}

func (g *GNCSystem) computeTargetPitch(nav NavState, dt float64) float64 {
	switch nav.Phase {
	case PhasePreLaunch:
		return 90

	case PhaseFirstStage:
		return g.firstStagePitch(nav)

	case PhaseSecondStage:
		return g.vacuumAscentPitch(nav, dt)

	case PhaseCircularization:
		// Довыведение в апоцентре: тяга по местному горизонту, регулятор
		// удерживает радиальную скорость около нуля. В апоцентре она и так
		// близка к нулю, поэтому команда получается почти горизонтальной —
		// это и есть оптимальное направление для подъёма перицентра.
		return g.RadialHold.Update(0, nav.RadialVelocity, dt)

	default:
		// Двигатели не работают — держим ориентацию по вектору скорости.
		return g.progradePitch(nav)
	}
}

// firstStagePitch реализует классическую схему «доворот и гравитационный
// разворот».
//
// Ключевой момент — удержание угла после доворота. Если сразу перейти на
// слежение за вектором скорости, ракета развернётся обратно вверх: сразу
// после доворота вектор скорости ещё почти вертикальный, и «пограде» окажется
// круче удерживаемого угла. Разворот при этом не начнётся вовсе. Поэтому
// команда берётся как минимум из удерживаемого угла и угла вектора скорости:
// пока скорость круче — держим наклон, как только она догнала — идём за ней
// с нулевым углом атаки.
func (g *GNCSystem) firstStagePitch(nav NavState) float64 {
	cfg := g.Config
	alt := nav.Altitude

	// Вертикальный подъём: пока ракета не отошла от стартового сооружения.
	if alt < cfg.VerticalRiseAltitude {
		return 90
	}

	// Начальный доворот.
	hold := 90 - cfg.PitchKickAngle
	if alt < cfg.PitchKickAltitude {
		progress := (alt - cfg.VerticalRiseAltitude) /
			(cfg.PitchKickAltitude - cfg.VerticalRiseAltitude)
		return 90 - cfg.PitchKickAngle*progress
	}

	prograde := g.progradePitch(nav)
	if prograde > hold {
		// Вектор скорости ещё круче удерживаемого угла — держим наклон.
		return hold
	}

	// Программа тангажа.
	//
	// Свободный гравитационный разворот здесь не годится ни как закон
	// управления, ни как расчётная траектория. При высокой тяговооружённости
	// он даёт слишком крутой подъём: ракета уходит на девяносто километров,
	// имея вертикальную скорость больше горизонтальной, и вторая ступень
	// потом тратит топливо на разворот этой скорости в горизонт. При низкой,
	// наоборот, траектория заваливается, и ракета вязнет в плотных слоях.
	//
	// Реальные носители летят по заранее рассчитанной программе тангажа.
	// Гравитационный разворот при этом остаётся расчётной целью программы:
	// профиль подобран так, что угол атаки в плотных слоях остаётся малым,
	// а ограничение по произведению q·α не даёт команде отойти от потока.
	return FirstStagePitchProgram(alt / g.pitchStretch())
}

// pitchProgramAltitudes и pitchProgramAngles задают программу тангажа
// первой ступени: высота (м) и угол над местным горизонтом (градусы).
//
// Профиль соответствует носителю среднего класса с тяговооружённостью около
// полутора на отрыве: к десяти километрам ракета уже наклонена на шестьдесят
// градусов, к моменту разделения ступеней — примерно на пятнадцать.
// Между узлами угол интерполируется линейно, поэтому команда непрерывна.
var (
	pitchProgramAltitudes = []float64{2000, 5000, 10000, 20000, 35000, 55000, 70000, 95000}
	pitchProgramAngles    = []float64{78, 72, 64, 54, 45, 37, 31, 25}
)

// pitchStretch возвращает множитель растяжения программы тангажа.
func (g *GNCSystem) pitchStretch() float64 {
	if g.Config.PitchProgramStretch > 0 {
		return g.Config.PitchProgramStretch
	}
	return 1
}

// FirstStagePitchProgram возвращает программный угол тангажа на заданной
// высоте.
func FirstStagePitchProgram(altitude float64) float64 {
	if altitude <= pitchProgramAltitudes[0] {
		return pitchProgramAngles[0]
	}
	last := len(pitchProgramAltitudes) - 1
	if altitude >= pitchProgramAltitudes[last] {
		return pitchProgramAngles[last]
	}
	for i := 1; i <= last; i++ {
		if altitude < pitchProgramAltitudes[i] {
			t := (altitude - pitchProgramAltitudes[i-1]) /
				(pitchProgramAltitudes[i] - pitchProgramAltitudes[i-1])
			return pitchProgramAngles[i-1] +
				t*(pitchProgramAngles[i]-pitchProgramAngles[i-1])
		}
	}
	return pitchProgramAngles[last]
}

// vacuumAscentPitch — наведение на разрежённом участке.
//
// Тангаж задаётся напрямую ПИД-регулятором по ошибке высоты апоцентра, а не
// поправкой к «пограде». Причина в том, что тяговооружённость второй ступени
// меньше единицы: удерживать высоту тягой она не может, и единственный способ
// выйти на орбиту — почти горизонтальный разгон с расчётом прийти в апоцентр
// ровно с нулевой вертикальной скоростью. Регулятор по апоцентру делает именно
// это: пока апоцентр ниже цели — нос приподнят, по мере приближения к цели
// команда плавно уходит к горизонту.
func (g *GNCSystem) vacuumAscentPitch(nav NavState, dt float64) float64 {
	target := g.Config.TargetOrbitAltitude

	// Требуемая вертикальная скорость пропорциональна недобору высоты.
	// Вдали от цели ступень набирает высоту, у цели требование плавно
	// сходит к нулю, и вся тяга разворачивается в горизонт.
	//
	// Управление именно по вертикальной скорости, а не напрямую по апоцентру,
	// устойчивее: апоцентр вблизи орбитальной скорости крайне чувствителен
	// к малым изменениям, и регулятор по нему раскачивался, перебрасывая
	// верхнюю точку орбиты на сотни километров за цель.
	apoapsis := nav.Elements.ApoapsisAltitude

	// Незамкнутая траектория означает избыток энергии — опускаем нос.
	if math.IsInf(apoapsis, 1) || !nav.Elements.Elliptical {
		return g.ApoapsisPitch.MinOutput
	}

	// Пока ступень идёт снизу и верхняя точка ниже цели, управляем апоцентром:
	// регулятор приподнимает нос ровно настолько, чтобы прийти в цель.
	//
	// Но одного апоцентра мало, и это не мелочь. Высота апоцентра ничего
	// не говорит о том, где ступень находится: «апоцентр двести километров»
	// одинаково верно и для машины, идущей на двухстах, и для той, что уже
	// падает и проходит девяносто. Регулятор по апоцентру во втором случае
	// доволен и держит горизонт, пока ступень не воткнётся в землю. Поэтому
	// как только высота набрана, управление переходит к удержанию высоты.
	arrived := nav.Altitude >= target*0.9 || apoapsis >= target*0.98
	if !arrived {
		g.AltitudeHold.Reset()
		return g.ApoapsisPitch.Update(target, apoapsis, dt)
	}

	// Дальше по апоцентру управлять нельзя, и это не тонкость настройки,
	// а смена смысла задачи. Ракета уже пришла на целевую высоту; всякий
	// последующий горизонтальный разгон делает эту точку перицентром и
	// поднимает противоположную сторону орбиты. Регулятор, которому велено
	// «держать апоцентр», в такой обстановке может только опускать нос — и
	// опускает, пока ступень не воткнётся в землю с работающими двигателями.
	// Ровно это и происходило на низкой цели.
	//
	// Правильная задача здесь другая: держать высоту и разгоняться в горизонт,
	// пока перицентр не поднимется до заданного. Потребная вертикальная
	// скорость берётся пропорционально недобору высоты и ограничивается —
	// резких подъёмов на этом участке быть не должно.
	wantVertical := physics.Clamp((target-nav.Altitude)*0.03, -60, 60)
	return g.AltitudeHold.Update(wantVertical, nav.RadialVelocity, dt)
}

// -----------------------------------------------------------------------------
// Guidance: азимут
// -----------------------------------------------------------------------------

func (g *GNCSystem) computeTargetAzimuth(nav NavState, dt float64) float64 {
	cfg := g.Config

	// Направление движения по широте определяем по северной составляющей
	// скорости с зоной нечувствительности — это даёт устойчивый гистерезис.
	// Признак «широта выросла с прошлого шага» здесь не работает: в точке
	// максимальной широты она не меняется, и латч залипает навсегда.
	_, vNorth, _ := nav.Frame.Decompose(nav.Velocity)
	const deadband = 5.0 // м/с
	if vNorth > deadband {
		g.ascending = true
	} else if vNorth < -deadband {
		g.ascending = false
	}

	base, reachable := physics.LaunchAzimuthForInclination(
		nav.Geodetic.Latitude, cfg.TargetInclination, g.ascending,
	)
	g.AzimuthAchieved = reachable

	current := nav.Elements.InclinationDegrees()
	g.InclinationDeg = current

	// До схода со стартового стола довороты бессмысленны.
	if nav.Altitude < cfg.VerticalRiseAltitude {
		return base
	}

	// Наклонение осмысленно только когда есть заметный момент импульса.
	if nav.TransverseVelocity < 200 {
		return base
	}

	// На пассивном участке тяги нет, доворачивать нечем. Регулятор при этом
	// нельзя оставлять включённым: он накапливал бы интеграл в отсутствие
	// управляющего воздействия и выдал бы бросок при следующем включении.
	if !nav.Phase.Powered() {
		return base
	}

	trim := g.InclinationTrim.Update(cfg.TargetInclination, current, dt)

	// Знак поправки.
	//
	// На восходящей ветви азимут отсчитывается от севера к востоку: чем он
	// больше, тем меньше северная составляющая скорости и тем МЕНЬШЕ
	// получается наклонение. Значит, чтобы поднять наклонение, азимут нужно
	// уменьшать — поправка вычитается.
	//
	// На нисходящей ветви азимут лежит в юго-восточном секторе, и там
	// зависимость обратная: рост азимута усиливает движение к югу и
	// увеличивает наклонение.
	if g.ascending {
		return physics.NormalizeAzimuth(base - trim)
	}
	return physics.NormalizeAzimuth(base + trim)
}

// -----------------------------------------------------------------------------
// Guidance: дросселирование
// -----------------------------------------------------------------------------

func (g *GNCSystem) computeThrottle(nav NavState, dt float64) float64 {
	cfg := g.Config

	if !nav.Phase.Powered() {
		return 0
	}

	throttle := 1.0

	switch nav.Phase {
	case PhaseSecondStage:
		// Точное выведение. У самой цели апоцентр становится крайне
		// чувствительным к скорости: последние десятки метров в секунду
		// поднимают его на сотню километров. Поэтому на подходе к целевой
		// высоте тяга убирается — так апоцентр не проскакивает, а перицентр
		// продолжает подниматься, поскольку тяга здесь уже почти
		// горизонтальна.
		el := nav.Elements
		if el.Elliptical {
			// Тяга убирается только на подходе к целевой высоте — там, где
			// апоцентр ещё поднимается и его надо посадить на цель мягко.
			//
			// Как только высота набрана, тяга возвращается полностью, и это
			// принципиально. Дальше апоцентр держит не дроссель, а тангаж,
			// а работы у ступени остаётся много: поднять перицентр с глубины
			// в тысячи километров до заданного. Дросселировать на этом участке
			// значит растянуть разгон, добавить потерь на тяготение и просесть
			// по высоте — ступень уходила вниз и врезалась в землю
			// с работающими двигателями.
			const (
				trimStart = 0.92
				trimEnd   = 0.98
			)
			ap := el.ApoapsisAltitude
			lo := cfg.TargetOrbitAltitude * trimStart
			hi := cfg.TargetOrbitAltitude * trimEnd

			// На целевой высоте тяга не убирается ни при каком апоцентре:
			// там ступень уже держит высоту тангажом, и всякий недобор тяги
			// оборачивается просадкой.
			arrived := nav.Altitude >= cfg.TargetOrbitAltitude*0.9

			if !arrived && ap > lo && ap < hi {
				excess := (ap - lo) / (hi - lo)
				throttle = physics.Clamp(1-excess*0.5, cfg.MinThrottle, 1)
			}
		}

	case PhaseCircularization:
		// Довыведение: регулятор по дефициту орбитальной скорости.
		required := physics.CircularOrbitVelocity(nav.Altitude)
		throttle = g.CircThrottle.Update(required, nav.TransverseVelocity, dt)
	}

	// Ограничение продольной перегрузки. К концу работы ступени масса падает
	// втрое, и без дросселирования перегрузка ушла бы за 6 g.
	if nav.Mass > 0 && nav.AvailableThrust > 0 {
		maxThrottle := cfg.MaxAxialAcceleration * nav.Mass / nav.AvailableThrust
		if maxThrottle < throttle {
			throttle = maxThrottle
		}
	}

	// Потолок штатного газа — не паспортные сто процентов, а полётный
	// предел (см. NominalThrottle). Отсекается здесь ещё раз, хотя
	// PhaseCircularization уже ограничен тем же потолком в самом
	// регуляторе (см. NewGNCSystem): PhaseSecondStage посчитан без
	// оглядки на предел, и без общего отсечения именно эта ветка
	// проскакивала бы выше него.
	return physics.Clamp(throttle, 0, nominalThrottleOr1(cfg))
}

// nominalThrottleOr1 возвращает потолок штатного газа, приводя
// незаданное значение к единице — так нулевое поле структуры остаётся
// осмысленным для конфигураций, которые эту величину не задают.
func nominalThrottleOr1(cfg GNCConfig) float64 {
	if cfg.NominalThrottle <= 0 {
		return 1
	}
	return cfg.NominalThrottle
}

// -----------------------------------------------------------------------------
// Control: ограничение угловых скоростей
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Поддержание орбиты
// -----------------------------------------------------------------------------

// OrbitCorrection описывает потребность в коррекции орбиты.
type OrbitCorrection struct {
	Needed  bool
	DeltaV  float64 // требуемое приращение скорости, м/с
	Reason  string
	AtApsis bool // находимся ли в окрестности апсиды, где коррекция эффективна
}

// CheckOrbitCorrection оценивает, нужна ли коррекция орбиты.
//
// В отличие от прежней версии эта проверка действительно вызывается из
// основного цикла: раньше орбитальная ветка завершалась безусловным continue,
// и весь код поддержания орбиты был недостижим.
func (g *GNCSystem) CheckOrbitCorrection(nav NavState) OrbitCorrection {
	el := nav.Elements
	if !el.Elliptical {
		return OrbitCorrection{}
	}

	// Апсида: радиальная скорость проходит через ноль.
	atApsis := math.Abs(nav.RadialVelocity) < 15

	// Перицентр опустился в плотные слои — орбита деградирует.
	if el.PeriapsisAltitude < g.Config.TargetOrbitAltitude*0.6 {
		required := physics.CircularOrbitVelocity(nav.Altitude)
		return OrbitCorrection{
			Needed:  true,
			DeltaV:  required - nav.TransverseVelocity,
			Reason:  "periapsis decay",
			AtApsis: atApsis,
		}
	}

	// Орбита слишком вытянута.
	if el.Eccentricity > 0.02 {
		required := physics.CircularOrbitVelocity(nav.Altitude)
		return OrbitCorrection{
			Needed:  true,
			DeltaV:  required - nav.TransverseVelocity,
			Reason:  "eccentricity",
			AtApsis: atApsis,
		}
	}

	return OrbitCorrection{}
}
