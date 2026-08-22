package simulator

import (
	"math"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Угловая динамика носителя.
//
// До сих пор ориентация корпуса задавалась кинематически: система управления
// назначала углы, и корпус мгновенно принимал их с точностью до ограничения
// угловой скорости. Моментов не существовало, поэтому не существовало и причин,
// по которым ракета вообще поворачивается.
//
// Здесь ориентация становится результатом уравнений вращения:
//
//	I·ω̇ + ω × (I·ω) = M_аэро + M_упр
//
// Носитель без оперения статически неустойчив: центр давления при поперечном
// обтекании лежит примерно посередине корпуса, а центр масс — ниже, у двигателей.
// Аэродинамический момент не возвращает корпус к потоку, а уводит от него, и
// удерживает ракету только качание камер. Именно поэтому отказ рулевого привода
// на активном участке означает потерю носителя за секунды, а не постепенное
// ухудшение управляемости.
// -----------------------------------------------------------------------------

// AttitudeControlConfig — параметры рулевого тракта.
type AttitudeControlConfig struct {
	// MaxGimbal — предельное отклонение камер от оси, рад.
	MaxGimbal float64

	// GimbalRate — скорость перекладки привода, рад/с.
	GimbalRate float64

	// RollAuthority — доля управляющего момента тангажа, доступная по крену.
	//
	// Крен создаётся разнонаправленным отклонением камер по касательной,
	// плечо здесь равно радиусу расстановки двигателей, а не длине корпуса,
	// поэтому располагаемый момент на порядок меньше.
	RollAuthority float64

	// Bandwidth — собственная частота замкнутого контура, рад/с.
	//
	// Автопилот управляет не отклонением камер напрямую, а потребным угловым
	// ускорением: сначала вычисляется, с каким ускорением надо разворачивать
	// корпус, затем по тензору инерции — потребный момент, и только потом
	// по тяге и плечу — угол качания. Благодаря этому настройка не зависит
	// ни от массы, ни от тяги. Прямая настройка по углу качания работала
	// на тяжёлой первой ступени и уводила в автоколебания лёгкую вторую:
	// при вчетверо меньшей инерции и том же плече тот же угол давал
	// вчетверо большее ускорение.
	Bandwidth float64

	// Damping — относительное демпфирование замкнутого контура.
	Damping float64

	// MaxAngularAccel — ограничение потребного углового ускорения, рад/с².
	// Не даёт автопилоту требовать разворота, который сорвёт конструкцию.
	MaxAngularAccel float64

	// RollBandwidth — собственная частота канала крена, рад/с.
	// Ниже, чем у тангажа: располагаемый момент по крену на порядок меньше.
	RollBandwidth float64

	// RCSMoment — располагаемый момент двигателей ориентации, Н·м.
	//
	// На пассивном участке главные двигатели выключены, и качать нечем:
	// управляющий момент создаётся отклонением тяги, а тяги нет. Без отдельных
	// двигателей ориентации ступень уходила бы в свободный дрейф и приходила
	// к манёвру довыведения повёрнутой куда попало. Момент невелик — сотни
	// ньютон-метров против мегаваттных моментов качания, — поэтому разворот
	// на пассивном участке занимает десятки секунд, а не доли.
	RCSMoment float64
}

// DefaultAttitudeControl возвращает параметры рулевого тракта носителя.
//
// Предельное отклонение камер у носителей такого класса — пять градусов,
// скорость привода около десяти градусов в секунду. Больше не нужно: при
// тяге в семь меганьютонов и плече в двадцать метров пяти градусов хватает
// на управляющий момент в двенадцать меганьютон-метров.
func DefaultAttitudeControl() AttitudeControlConfig {
	return AttitudeControlConfig{
		MaxGimbal:     5 * physics.DegToRad,
		GimbalRate:    10 * physics.DegToRad,
		RollAuthority: 0.06,

		// Полоса пропускания полрадиана в секунду — период около двенадцати
		// секунд. Медленнее программы тангажа, которая меняется на полградуса
		// в секунду, и достаточно быстро, чтобы парировать уводящий момент
		// неустойчивого корпуса.
		Bandwidth:       0.5,
		Damping:         0.9,
		MaxAngularAccel: 0.35,
		RollBandwidth:   0.25,

		// Восемь блоков по двести ньютонов на плече в два с половиной метра.
		RCSMoment: 4000,
	}
}

// VehicleAttitude — угловое состояние носителя.
type VehicleAttitude struct {
	Config AttitudeControlConfig

	// Orientation — ориентация корпуса в инерциальных осях.
	Orientation physics.Quaternion

	// Omega — угловая скорость в связанных осях, рад/с.
	// X — крен, Y — тангаж, Z — рыскание.
	Omega physics.Vec3

	// Gimbal — фактическое отклонение камер, рад.
	GimbalPitch, GimbalYaw, GimbalRoll float64

	// GimbalCommand — команда автопилота до ограничений, рад.
	CommandPitch, CommandYaw, CommandRoll float64

	// Моменты последнего шага в связанных осях, Н·м.
	AeroTorque    physics.Vec3
	ControlTorque physics.Vec3

	// Inertia — тензор инерции на текущем шаге, кг·м².
	Inertia physics.InertiaTensor

	// StaticMargin — запас статической устойчивости в калибрах.
	//
	// Отрицательное значение означает, что центр давления впереди центра масс
	// и корпус сам от потока уводится. Для носителя без оперения это норма.
	StaticMargin float64

	// Authority — отношение потребного управляющего момента к располагаемому.
	// Значения выше единицы означают, что приводы вышли на упор.
	Authority float64

	// Saturated сообщает, что привод стоит на упоре.
	Saturated bool

	// EffectiveLimit — действующий предел хода привода с учётом воздействий, рад.
	EffectiveLimit float64

	// UsingRCS сообщает, что ориентацией управляют двигатели ориентации,
	// а не качание камер.
	UsingRCS bool

	// SurfaceTrim — найденная контуром поправка среднего положения плавников,
	// доля хода. Ею компенсируется собственная разбалансировка пары.
	SurfaceTrim float64

	// Attached сообщает, что ориентация ещё не инициализирована.
	initialised bool
	// Surfaces — аэродинамические управляющие поверхности, если они есть.
	//
	// Третий орган управления после качания камер и двигателей ориентации.
	// Работает только от потока: на активном участке молчит, на входе
	// в атмосферу становится единственным.
	Surfaces *SurfaceSet

	// SurfaceTorque — момент от управляющих поверхностей, Н·м.
	SurfaceTorque physics.Vec3

	// SurfaceAuthority — располагаемый момент поверхностей по тангажу, Н·м.
	SurfaceAuthority float64

	// SensedOrientation — показание датчика ориентации, которым в
	// attitudeError подменяется истинная ориентация при вычислении ошибки
	// автопилота. Заполняется снаружи (Simulation.updateAttitude) один раз
	// за такт, ДО вызова Update — датчик не опрашивается на каждом
	// внутреннем подшаге интегрирования, как и остальные измерительные
	// каналы симуляции.
	//
	// Нулевое значение (ещё не заполнено — например, в тестах, работающих
	// с VehicleAttitude напрямую, без Simulation) означает «датчика нет»:
	// attitudeError считает ошибку по истинной ориентации, как и до
	// появления датчика. Само интегрирование истинного Orientation по
	// угловой скорости эта подмена не затрагивает никогда — датчик влияет
	// только на ветку вычисления ошибки.
	SensedOrientation physics.Quaternion

	// filteredResidual — сглаженный недобор момента (want − доставлено
	// плавниками), по которому решается, включать ли двигатели ориентации
	// (см. интегратор, случай cfg.RCSMoment > 0).
	//
	// Сглаживается только этот недобор, а не сама ошибка ориентации: контур
	// качания камер обязан оставаться быстрым (посадочный манёвр считает
	// доли секунды), а вот решение «включать ли РСУ» на пассивном участке —
	// медленное и разовое, и там сырой шум датчика (см.
	// sensing.DefaultAttitudeSensor), помноженный на момент инерции корпуса
	// и коэффициент пропорциональности контура, выходит далеко за зону
	// нечувствительности (RCSMoment·rcsDeadband) и заставляет двигатели
	// молотить непрерывно там, где раньше молчали. Реальный борт тоже не
	// принимает решение о расходе рабочего тела по одиночному отсчёту
	// гироскопа — только по сглаженной оценке.
	filteredResidual physics.Vec3
}

// AttitudeInput — внешние условия для расчёта углового движения.
type AttitudeInput struct {
	// Target — потребная ориентация, назначенная наведением.
	Target physics.Attitude

	// Frame — местный горизонтальный базис.
	Frame physics.LocalFrame

	// AirRelative — скорость относительно воздуха в инерциальных осях, м/с.
	AirRelative physics.Vec3

	// DynamicPressure — скоростной напор, Па.
	DynamicPressure float64

	// Mach — число Маха.
	Mach float64

	// Thrust — суммарная тяга, Н.
	Thrust float64

	// Mass — масса носителя, кг.
	Mass float64

	// Length, Radius — геометрия корпуса, м.
	Length, Radius float64

	// CenterOfMass — положение центра масс от носа, доля длины.
	CenterOfMass float64

	// GimbalArm — расстояние от центра масс до плоскости качания камер, м.
	GimbalArm float64

	// EngineRingRadius — радиус расстановки двигателей, м.
	EngineRingRadius float64

	// Overrides — ручные воздействия на рулевой тракт.
	Overrides control.ControlOverrides

	// Controllable сообщает, что рулевой тракт работоспособен.
	//
	// Без тяги качать нечем: управляющий момент создаётся отклонением вектора
	// тяги, и на выключенных двигателях он равен нулю независимо от того,
	// что делает привод.
	Controllable bool
}

// Init задаёт начальную ориентацию по назначенной наведением.
//
// Ракета на стартовом столе стоит вертикально и никуда не поворачивается:
// начинать интегрирование с произвольного положения было бы неверно.
func (a *VehicleAttitude) Init(target physics.Attitude, frame physics.LocalFrame) {
	body := target.BodyFrame(frame)
	a.Orientation = physics.QuaternionFromBasis(body.Forward, body.Right, body.Down)
	a.Omega = physics.Vec3{}
	a.GimbalPitch, a.GimbalYaw, a.GimbalRoll = 0, 0, 0
	a.filteredResidual = physics.Vec3{}
	a.initialised = true
}

// Update продвигает угловое движение на dt секунд.
//
// Шаг интегрирования вращения берётся мельче шага поступательного движения:
// постоянная времени неустойчивого корпуса при высоком напоре составляет доли
// секунды, и на шаге в одну десятую решение разошлось бы.
func (a *VehicleAttitude) Update(dt float64, in AttitudeInput) {
	if !a.initialised {
		a.Init(in.Target, in.Frame)
		return
	}
	if dt <= 0 {
		return
	}

	const maxStep = 0.01
	steps := int(math.Ceil(dt / maxStep))
	if steps < 1 {
		steps = 1
	}
	h := dt / float64(steps)

	for i := 0; i < steps; i++ {
		a.integrate(h, in)
	}
}

// integrate выполняет один малый шаг углового движения.
func (a *VehicleAttitude) integrate(dt float64, in AttitudeInput) {
	shape := physics.AeroShape{
		Length:                 in.Length,
		Radius:                 in.Radius,
		CenterOfMass:           in.CenterOfMass,
		CenterOfPressureAxial:  0.30,
		CenterOfPressureNormal: 0.50,
	}
	// Запас устойчивости в калибрах. Положительный означает, что центр
	// давления позади центра масс и поток возвращает корпус к себе;
	// отрицательный — что центр давления впереди и поток уводит.
	// У носителя без оперения он отрицательный, и это норма.
	a.StaticMargin = (shape.CenterOfPressureNormal - in.CenterOfMass) *
		in.Length / (2 * in.Radius)

	a.Inertia = physics.CylinderInertia(in.Mass, in.Radius, in.Length)

	// --- 1. Аэродинамический момент ------------------------------------------
	//
	// Считается в связанных осях: поток переводится из инерциальных осей
	// поворотом, обратным ориентации корпуса.
	airBody := a.Orientation.RotateInverse(in.AirRelative)
	a.AeroTorque = shape.AerodynamicTorque(in.DynamicPressure, airBody, a.Omega, in.Mach)

	// --- 2. Автопилот ---------------------------------------------------------
	//
	// Ошибка ориентации вычисляется как поворот от текущего положения
	// к потребному. Малые углы этого поворота и есть рассогласование
	// по тангажу, рысканию и крену.
	errPitch, errYaw, errRoll := a.attitudeError(in.Target, in.Frame)

	// Ручные воздействия на рулевой тракт.
	ov := in.Overrides
	cfg := a.Config
	cfg.MaxGimbal *= ov.GimbalLimitFactor.Or(1)
	cfg.GimbalRate *= ov.GimbalRateFactor.Or(1)
	cfg.Bandwidth *= ov.BandwidthFactor.Or(1)
	cfg.Damping *= ov.DampingFactor.Or(1)
	cfg.RCSMoment *= ov.RCSFactor.Or(1)
	a.EffectiveLimit = cfg.MaxGimbal

	// Потребное угловое ускорение по каждому каналу.
	kp := cfg.Bandwidth * cfg.Bandwidth
	kd := 2 * cfg.Damping * cfg.Bandwidth
	rollKp := cfg.RollBandwidth * cfg.RollBandwidth
	rollKd := 2 * cfg.Damping * cfg.RollBandwidth

	accPitch := clampAbs(kp*errPitch-kd*a.Omega.Y, cfg.MaxAngularAccel)
	accYaw := clampAbs(kp*errYaw-kd*a.Omega.Z, cfg.MaxAngularAccel)
	accRoll := clampAbs(rollKp*errRoll-rollKd*a.Omega.X, cfg.MaxAngularAccel)

	// Потребный момент и потребное отклонение камер. Аэродинамический момент
	// компенсируется явно: автопилот знает, что поток уводит корпус, и
	// закладывает противодействие, а не ждёт появления ошибки.
	wantPitch := a.Inertia.Iyy*accPitch - a.AeroTorque.Y
	wantYaw := a.Inertia.Izz*accYaw - a.AeroTorque.Z
	wantRoll := a.Inertia.Ixx*accRoll - a.AeroTorque.X

	side := in.Thrust * in.GimbalArm
	rollSide := in.Thrust * in.EngineRingRadius
	_ = rollSide

	a.CommandPitch = gimbalFor(wantPitch, side)
	a.CommandYaw = gimbalFor(wantYaw, side)
	a.CommandRoll = gimbalFor(wantRoll, rollSide)
	_ = rollSide

	// --- 3. Привод ------------------------------------------------------------
	//
	// Камеры перекладываются с конечной скоростью и упираются в механический
	// предел. Отсюда и запаздывание контура, и потеря управляемости при
	// требовании, превышающем располагаемое отклонение.
	// Заклинивший привод команды не отрабатывает и остаётся где стоял.
	if !ov.Frozen {
		step := cfg.GimbalRate * dt
		a.GimbalPitch = approachClamped(a.GimbalPitch, a.CommandPitch, step, cfg.MaxGimbal)
		a.GimbalYaw = approachClamped(a.GimbalYaw, a.CommandYaw, step, cfg.MaxGimbal)
		a.GimbalRoll = approachClamped(a.GimbalRoll, a.CommandRoll, step,
			cfg.MaxGimbal*cfg.RollAuthority)
	}

	// Постоянное смещение привода: механическая неисправность, из-за которой
	// камеры стоят под углом независимо от команды.
	bias := ov.GimbalBias.Or(0)

	required := math.Hypot(a.CommandPitch, a.CommandYaw)
	a.Authority = required / math.Max(cfg.MaxGimbal, 1e-9)
	a.Saturated = required > cfg.MaxGimbal

	// --- 4. Аэродинамические поверхности --------------------------------------
	//
	// Плавники и решётчатые рули отрабатывают тот же потребный момент, что
	// и камеры, но их власть определяется скоростным напором. Автопилот здесь
	// один на все органы: он задаёт потребный момент, а кто его создаст —
	// вопрос того, что доступно в этой фазе полёта.
	//
	// Считаются они раньше двигателей ориентации, и это не порядок строк,
	// а порядок подчинения. Пока поток даёт власть, корпус держат плавники;
	// двигателям ориентации остаётся то, чего плавники не добрали. Прежде оба
	// органа получали одну и ту же команду независимо, и двигатели ориентации
	// молотили весь вход — в плотных слоях, где момент плавников на два порядка
	// больше, и запаса рабочего тела на такое не хватило бы ни у одного
	// корабля.
	// Момент прошлого шага — то, что плавники дали на самом деле. По нему
	// и правится балансировка: обнулять его перед сравнением значит сравнивать
	// потребность саму с собой.
	delivered := a.SurfaceTorque

	a.SurfaceTorque = physics.Vec3{}
	a.SurfaceAuthority = 0

	if a.Surfaces.Count() > 0 {
		a.SurfaceAuthority = a.Surfaces.PitchAuthority(airBody, in.DynamicPressure, in.Mach)

		demand := func(want float64) float64 {
			if a.SurfaceAuthority < 1 {
				return 0
			}
			return clampAbs(want/a.SurfaceAuthority, 1)
		}

		// Балансировка. Симметричное положение плавников само по себе момента
		// не обнуляет: передняя пара меньше задней, но сидит на большем плече,
		// и на угле атаки в семьдесят градусов пара разбалансирована. Без
		// поправки контур упирается в постоянный остаток — и раньше этот
		// остаток молча дожигали двигатели ориентации.
		//
		// Поправка ищется интегратором: пока плавники дают не тот момент,
		// который нужен, среднее положение медленно ползёт, пока разница
		// не сойдёт к нулю. Так и балансируют настоящий аппарат — рулевой
		// триммер находит положение сам, а не рассчитывается заранее.
		//
		// Копится поправка только пока плавникам есть чем работать. Порог не
		// формальный: на развороте перед посадкой корпус встаёт к потоку
		// ребром, власть падает на порядок, и ошибка, делённая на неё, улетает
		// в небо. Интегратор упирался в упор, а потом столько же времени
		// выбирался обратно — корабль успевал проскочить вертикаль и завалиться
		// на другую сторону. Поэтому при слабой власти накопленное не держится,
		// а спокойно сходит к нулю.
		const trimGain = 0.6 // 1/с
		switch {
		case a.SurfaceAuthority > math.Abs(wantPitch) && a.SurfaceAuthority > 1:
			err := clampAbs((wantPitch-delivered.Y)/a.SurfaceAuthority, 1)
			a.SurfaceTrim = clampAbs(a.SurfaceTrim+trimGain*err*dt, 1)
		default:
			a.SurfaceTrim = approachClamped(a.SurfaceTrim, 0, trimGain*dt, 1)
		}

		a.Surfaces.Allocate(
			clampAbs(demand(wantPitch)+a.SurfaceTrim, 1),
			demand(wantYaw),
			demand(wantRoll))
		a.Surfaces.Advance(dt)
		a.SurfaceTorque = a.Surfaces.UpdateTorque(airBody, in.DynamicPressure, in.Mach)
	}

	// --- 5. Управляющий момент ------------------------------------------------
	//
	// Отклонённая камера создаёт боковую составляющую тяги T·sin δ,
	// приложенную на плече от центра масс до плоскости качания.
	switch {
	case ov.Dead:
		// Обесточенный рулевой тракт: моменту взяться неоткуда.
		a.ControlTorque = physics.Vec3{}
		a.UsingRCS = false

	case in.Controllable && in.Thrust > 0:
		a.ControlTorque = physics.Vec3{
			X: in.Thrust * math.Sin(a.GimbalRoll) * in.EngineRingRadius,
			Y: side * math.Sin(a.GimbalPitch+bias),
			Z: side * math.Sin(a.GimbalYaw),
		}
		a.UsingRCS = false

	case cfg.RCSMoment > 0 && !ov.Dead:
		// Пассивный участок: работают двигатели ориентации. Момент постоянный
		// и на порядки меньше, поэтому разворот идёт медленно.
		//
		// Берут они на себя только недобор: то, что уже создали плавники,
		// повторять незачем. В разрежённых слоях недобор равен всей потребности,
		// и корпус держат только они; ниже, когда поток набирает силу, остаток
		// сходит к нулю сам собой, и двигатели замолкают.
		residual := physics.Vec3{
			X: wantRoll - a.SurfaceTorque.X,
			Y: wantPitch - a.SurfaceTorque.Y,
			Z: wantYaw - a.SurfaceTorque.Z,
		}

		// Сглаживание — только когда подключён датчик ориентации (иначе
		// residual точен и фильтровать нечего, поведение остаётся прежним
		// для прямых тестов VehicleAttitude). Решение «включать ли РСУ» на
		// пассивном участке медленное и разовое, а сырой шум датчика (см.
		// sensing.DefaultAttitudeSensor), помноженный на момент инерции
		// корпуса и коэффициент пропорциональности контура, выходит далеко
		// за зону нечувствительности (RCSMoment·rcsDeadband) и заставляет
		// двигатели молотить непрерывно там, где раньше молчали. Постоянная
		// времени здесь на порядки больше, чем у контура качания камер
		// (который остаётся быстрым и сырым — см. attitudeError): решение
		// о расходе рабочего тела реальный борт тоже не принимает по
		// одиночному отсчёту гироскопа.
		if a.SensedOrientation.Norm() > 1e-9 && dt > 0 {
			const residualFilterTime = 1.5
			decay := math.Exp(-dt / residualFilterTime)
			a.filteredResidual = residual.Add(a.filteredResidual.Sub(residual).Scale(decay))
			residual = a.filteredResidual
		} else {
			a.filteredResidual = residual
		}

		a.ControlTorque = physics.Vec3{
			X: clampAbs(residual.X, cfg.RCSMoment*cfg.RollAuthority*10),
			Y: clampAbs(residual.Y, cfg.RCSMoment),
			Z: clampAbs(residual.Z, cfg.RCSMoment),
		}

		// Малый остаток не стоит сожжённого рабочего тела: двигатели ориентации
		// работают импульсами и на мелкую рассогласовку не отзываются.
		const rcsDeadband = 0.02
		if a.ControlTorque.Norm() < cfg.RCSMoment*rcsDeadband {
			a.ControlTorque = physics.Vec3{}
		}

		a.UsingRCS = a.ControlTorque.Norm() > 0
		a.GimbalPitch, a.GimbalYaw, a.GimbalRoll = 0, 0, 0

	default:
		a.ControlTorque = physics.Vec3{}
		a.UsingRCS = false
	}

	// --- 6. Уравнения вращения ------------------------------------------------
	torque := a.AeroTorque.Add(a.ControlTorque).Add(a.SurfaceTorque)
	alpha := a.Inertia.AngularAcceleration(a.Omega, torque)

	a.Omega = a.Omega.Add(alpha.Scale(dt))

	// Кватернион ориентации интегрируется и нормируется: без нормировки
	// численная ошибка накапливается и корпус начинает «раздуваться».
	q := a.Orientation.Add(a.Orientation.Derivative(a.Omega).Scale(dt))
	a.Orientation = q.Normalized()
}

// PointingError возвращает угол между продольной осью и потребным
// направлением, градусы.
//
// Нужен там, где важно не «примерно туда», а факт готовности: перед выдачей
// тормозного импульса корпус обязан прийти к ретроградной ориентации,
// иначе импульс уйдёт мимо и вместо схода с орбиты поднимет её.
func (a *VehicleAttitude) PointingError(target physics.Attitude,
	frame physics.LocalFrame) float64 {

	want := frame.Direction(target.Pitch, target.Yaw)
	have := a.Orientation.Rotate(physics.Vec3{X: 1})

	cos := physics.Clamp(want.Unit().Dot(have.Unit()), -1, 1)
	return math.Acos(cos) * physics.RadToDeg
}

// attitudeError возвращает рассогласование в связанных осях, рад.
//
// Считать ошибку разностью углов Эйлера нельзя. На вертикальном участке
// тангаж равен девяноста градусам, и представление вырождается: изменение
// курса становится неотличимо от крена, рассогласование по курсу обращается
// в произвольную величину, автопилот начинает с ней бороться и опрокидывает
// ракету через несколько секунд после отрыва.
//
// Поэтому ошибка вычисляется как поворот от текущей ориентации к потребной:
//
//	q_ошибки = q_текущей⁻¹ · q_потребной
//
// Вектор этого поворота, взятый в связанных осях, и есть рассогласование
// по крену, тангажу и рысканию. Особых точек у такого представления нет.
func (a *VehicleAttitude) attitudeError(target physics.Attitude,
	frame physics.LocalFrame) (pitch, yaw, roll float64) {

	body := target.BodyFrame(frame)
	want := physics.QuaternionFromBasis(body.Forward, body.Right, body.Down)

	// Ошибку считаем от показания датчика, а не от истинной ориентации:
	// автопилот, как и настоящий борт, знает только то, что говорит
	// гироскоп. Нулевое SensedOrientation означает «датчик не подключен»
	// (прямые тесты VehicleAttitude без Simulation) — тогда используется
	// истинная ориентация, как и раньше.
	//
	// Показание здесь сырое, без сглаживания: контур качания камер должен
	// оставаться быстрым даже на посадочном манёвре, где счёт идёт на доли
	// секунды. Сглаживание, где оно нужно (решение о переходе на двигатели
	// ориентации на пассивном участке), применяется отдельно и только там —
	// см. комментарий у filteredResidual в integrate().
	current := a.Orientation
	if a.SensedOrientation.Norm() > 1e-9 {
		current = a.SensedOrientation
	}

	err := current.Conjugate().Multiply(want).Normalized()

	// Кватернионы q и −q задают один поворот. Берём тот, что соответствует
	// повороту меньше половины оборота, иначе автопилот поведёт корпус длинным путём.
	if err.W < 0 {
		err = physics.Quaternion{W: -err.W, X: -err.X, Y: -err.Y, Z: -err.Z}
	}

	// Вектор поворота: угол умножается на ось. Для малых углов это просто
	// удвоенная векторная часть, для больших нужен арккосинус.
	angle := 2 * math.Acos(math.Min(1, err.W))
	axis := physics.Vec3{X: err.X, Y: err.Y, Z: err.Z}
	if n := axis.Norm(); n > 1e-12 {
		axis = axis.Scale(angle / n)
	} else {
		axis = physics.Vec3{}
	}
	return axis.Y, axis.Z, axis.X
}

// ThrustDirection возвращает направление тяги в инерциальных осях.
//
// Отклонённая камера уводит вектор тяги от продольной оси, поэтому продольная
// составляющая уменьшается на косинус отклонения. Эти потери на управление
// невелики, но они настоящие: при пяти градусах теряется четыре десятых
// процента тяги.
func (a *VehicleAttitude) ThrustDirection() physics.Vec3 {
	body := a.Orientation.Body()

	cp, sp := math.Cos(a.GimbalPitch), math.Sin(a.GimbalPitch)
	cy, sy := math.Cos(a.GimbalYaw), math.Sin(a.GimbalYaw)

	// Знаки согласованы с управляющим моментом. Нос поднимается тогда, когда
	// тяга отклонена ВНИЗ: сила приложена в хвосте, позади центра масс, и
	// толкает хвост вниз, а нос вверх. Разворот вправо, наоборот, требует
	// отклонения тяги влево. Прежде продольная сила уводилась в другую
	// сторону, чем создающая момент, и боковая перегрузка получалась обратной.
	dir := body.Forward.Scale(cp * cy).
		Add(body.Down.Scale(sp)).
		Sub(body.Right.Scale(sy))

	if n := dir.Norm(); n > 1e-9 {
		return dir.Scale(1 / n)
	}
	return body.Forward
}

// GimbalLossFactor возвращает долю тяги, остающуюся вдоль продольной оси.
func (a *VehicleAttitude) GimbalLossFactor() float64 {
	return math.Cos(a.GimbalPitch) * math.Cos(a.GimbalYaw)
}

// AttitudeIn возвращает углы ориентации в местном базисе.
func (a *VehicleAttitude) AttitudeIn(frame physics.LocalFrame) physics.Attitude {
	return a.Orientation.AttitudeIn(frame)
}

// Rates возвращает угловые скорости в градусах в секунду.
func (a *VehicleAttitude) Rates() (pitch, yaw, roll float64) {
	return a.Omega.Y * physics.RadToDeg,
		a.Omega.Z * physics.RadToDeg,
		a.Omega.X * physics.RadToDeg
}

// gimbalFor возвращает отклонение камер, дающее потребный момент.
//
// Момент от отклонённой тяги равен T·L·sin δ, откуда δ = arcsin(M/(T·L)).
// Если потребный момент больше располагаемого, возвращается угол за пределом
// хода — привод упрётся в упор, и это будет видно по признаку насыщения.
func gimbalFor(moment, capacity float64) float64 {
	if capacity <= 1e-6 {
		return 0
	}
	ratio := moment / capacity
	if ratio > 1 {
		// За пределом арксинуса. Возвращается угол больше прямого, чтобы
		// признак насыщения сработал, а привод встал на упор.
		return math.Pi / 2 * ratio
	}
	if ratio < -1 {
		return -math.Pi / 2 * -ratio
	}
	return math.Asin(ratio)
}

// clampAbs ограничивает величину по модулю.
func clampAbs(v, limit float64) float64 {
	return math.Max(-limit, math.Min(limit, v))
}

// approachClamped подводит текущее значение к цели с ограничением скорости
// и упором в механический предел.
func approachClamped(current, target, step, limit float64) float64 {
	target = math.Max(-limit, math.Min(limit, target))
	if d := target - current; math.Abs(d) <= step {
		current = target
	} else if d > 0 {
		current += step
	} else {
		current -= step
	}
	return math.Max(-limit, math.Min(limit, current))
}

// -----------------------------------------------------------------------------
// Связь с полётной моделью
// -----------------------------------------------------------------------------

// updateAttitude продвигает угловое движение носителя на шаг.
// Вызывается под удержанным mu.
func (s *Simulation) updateAttitude(dt float64, target physics.Attitude,
	nav orbit.NavState, atm physics.AtmosphereState, ov control.ControlOverrides) {

	cfg := s.Config
	mass := s.dryMass + s.state.FuelMass

	// Длина активной ступени вместе с тем, что она везёт.
	length := cfg.FirstStageLength
	if s.stage > 1 {
		length = secondStageLength(cfg)
	}

	// Центр масс уезжает к двигателям по мере выработки: полный бак смещает
	// его к носу, пустой оставляет массу конструкции внизу. Именно поэтому
	// запас устойчивости меняется по ходу работы ступени.
	fill := 0.0
	if s.propulsion != nil {
		fill = s.propulsion.FillFraction()
	}
	const (
		comFull  = 0.52 // доля длины от носа при полных баках
		comEmpty = 0.72 // при пустых
	)
	com := comEmpty + (comFull-comEmpty)*fill

	// На возвращении центр масс уходит вперёд: посадочное топливо держат
	// в носовых расходных баках. Это не мелочь компоновки, а условие
	// управляемости: при полёте брюхом центр давления лежит у середины
	// корпуса, и если центр масс останется у двигателей, аэродинамический
	// момент будет в сотни раз больше того, что могут дать плавники.
	if s.inEntry() {
		com = 0.50 // центр давления при полёте брюхом
	}

	// Плечо качания: от центра масс до плоскости среза сопел.
	arm := (1 - com) * length

	thrust := 0.0
	if s.propulsion != nil {
		thrust = s.propulsion.TotalThrust
	}

	// На стартовом столе корпус удерживается захватами и повернуться не может.
	// Интегрировать вращение до отрыва значит позволить ветру развернуть
	// закреплённую ракету.
	if !s.released {
		s.attitude.Omega = physics.Vec3{}
		return
	}

	// Момент двигателей ориентации — свойство изделия.
	if cfg.RCSMoment > 0 {
		s.attitude.Config.RCSMoment = cfg.RCSMoment
	}

	// Показание датчика ориентации — то, чем реально распоряжается
	// автопилот при вычислении ошибки (attitudeError). Истинная Orientation
	// продолжает интегрироваться по угловой скорости ниже, эта подмена её
	// не затрагивает.
	s.attitude.SensedOrientation = s.lastValidSensedOrientation

	s.attitude.Update(dt, AttitudeInput{
		Overrides:        ov,
		Target:           target,
		Frame:            nav.Frame,
		AirRelative:      nav.AirRelativeVelocity,
		DynamicPressure:  nav.DynamicPressure,
		Mach:             machNumber(nav.AirRelativeVelocity.Norm(), atm.SoundSpeed),
		Thrust:           thrust,
		Mass:             mass,
		Length:           length,
		Radius:           cfg.Diameter / 2,
		CenterOfMass:     com,
		GimbalArm:        arm,
		EngineRingRadius: cfg.EngineRingRadius,
		Controllable:     thrust > 0,
	})
}

// Attitude возвращает угловое состояние носителя.
func (s *Simulation) Attitude() VehicleAttitude {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.attitude
}

// integrateNoControl интегрирует вращение только под управляющим моментом.
// Используется в тестах для проверки знаков.
func (a *VehicleAttitude) integrateNoControl(dt float64, in AttitudeInput) {
	a.Inertia = physics.CylinderInertia(in.Mass, in.Radius, in.Length)
	side := in.Thrust * in.GimbalArm
	a.ControlTorque = physics.Vec3{
		X: in.Thrust * math.Sin(a.GimbalRoll) * in.EngineRingRadius,
		Y: side * math.Sin(a.GimbalPitch),
		Z: side * math.Sin(a.GimbalYaw),
	}
	alpha := a.Inertia.AngularAcceleration(a.Omega, a.ControlTorque)
	a.Omega = a.Omega.Add(alpha.Scale(dt))
	a.Orientation = a.Orientation.Add(a.Orientation.Derivative(a.Omega).Scale(dt)).Normalized()
}
