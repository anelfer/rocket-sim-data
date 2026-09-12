package simulator

import (
	"math"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/orbit/gfold"
	"rocketTelemetrySim/simulator/orbit/sensing"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Наведение посадочного импульса первой ступени по G-FOLD.
//
// Что заменено. Прежний закон решал две задачи по отдельности: вертикальный
// канал вёл снижение по профилю v(h)=√(v_кас²+2·a·h), горизонтальный гасил
// снос к нулю за оценённое время до касания. Ни у того, ни у другого не было
// понятия «точка посадки»: попытка добавить его отдельным членом коррекции
// положения уже предпринималась и была откачена — при времени до касания в
// единицы секунд и унаследованном промахе в километры требуемое боковое
// ускорение выходило за все физические пределы, а направление цели начинало
// колебаться быстрее, чем корпус успевал поворачиваться. Итог был измерим:
// приводнение мягкое (2.1-2.4 м/с), но за 56-87 км от площадки, причём с
// четвертью заправки (225-235 т), так и не потраченной ни на что.
//
// Причина не в подборе коэффициентов. Задача «прийти в точку с нулевой
// скоростью, уложившись в границы тяги и не потратив лишнего» — это задача
// оптимального управления, и решать её надо как задачу оптимального
// управления, а не двумя независимыми регуляторами, каждый из которых видит
// свою половину. Ровно это и делает G-FOLD (см. simulator/orbit/gfold):
// строит траекторию целиком, с явной целью и явными ограничениями, и
// пересчитывает её на каждом такте наведения.
//
// Что оставлено без изменений (и почему): выбор группы камер
// (updateLandingEngineGroup), ограничение по располагаемому управляющему
// моменту (landingAuthorityConstrainedThrust), профиль разворота цели
// (landingBurnTarget), сглаживание дросселя. Всё это — не закон наведения,
// а исполнительная часть: она отрабатывает ЛЮБОЙ требуемый вектор тяги, и
// заменять её вместе с законом не было причин.
// -----------------------------------------------------------------------------

// Постоянные времени контура наведения.
const (
	// gfoldResolveInterval — период пересчёта траектории на посадочном
	// импульсе, с. Не «чем чаще, тем лучше»: между пересчётами наведение
	// летит по УЖЕ найденной оптимальной траектории, и пересчёт нужен не
	// для управления как такового, а чтобы учесть накопившееся расхождение
	// с планом. Две секунды — тот же порядок, что постоянная времени
	// самой тяги (throttleFilterTime): пересчитывать план быстрее, чем
	// двигатель успевает отозваться на предыдущую команду, значит
	// планировать по несуществующему состоянию.
	gfoldResolveInterval = 2.0

	// gfoldMinTimeToGo — ниже этого остатка времени полёта задача больше
	// не пересчитывается, и наведение доводит посадку по последней
	// найденной траектории, с обратной связью. Величина — удвоенная
	// постоянная времени тяги: траекторию короче того времени, за которое
	// тяга физически успевает измениться, планировать не о чем.
	gfoldMinTimeToGo = 2 * throttleFilterTime

	// gfoldNodes — целевое число интервалов сетки. Шаг выбирается как
	// t_ост/gfoldNodes (см. gfoldStep), а не наоборот: так стоимость
	// решения постоянна на всём импульсе (она растёт как куб числа узлов),
	// а точность сетки САМА улучшается по мере приближения к площадке —
	// там, где она и нужна.
	gfoldNodes = 20

	// gfoldMinStep, gfoldMaxStep — границы шага сетки, с.
	gfoldMinStep = 0.25
	gfoldMaxStep = 5.0

	// gfoldCoastProbeInterval — период проверки «пора ли зажигать» на
	// пассивном участке, с (см. gfoldIgnitionReady).
	gfoldCoastProbeInterval = 5.0

	// gfoldGlideSlope — угол посадочного конуса над горизонтом, рад.
	// Конус строится вокруг САМОЙ точки касания (см. gfold/problem.go) и
	// запрещает подход к ней снизу или по пологой траектории. Сорок пять
	// градусов — не подобранное значение: это угол, при котором
	// горизонтальное удаление от точки касания не превосходит превышения
	// над ней, то есть аппарат всё время «над» своей целью, а не сбоку.
	gfoldGlideSlope = 45 * physics.DegToRad

	// gfoldGlideApex — насколько ниже точки касания опущена вершина
	// конуса, м. У самой вершины конус вырождается в точку, и терминальный
	// узел оказывался бы ровно в его особенности. Половина диаметра
	// ступени — характерный размер самого аппарата, меньше которого
	// говорить о его положении бессмысленно.
	gfoldGlideApex = 4.5

	// gfoldMaxTilt — потолок угла отклонения тяги от вертикали, рад.
	// Реально используемый угол считается по РАСПОЛАГАЕМОМУ управляющему
	// моменту (см. landingPointingLimit) и почти всегда меньше; потолок
	// нужен, чтобы на малом скоростном напоре, где момента хватает на что
	// угодно, наведение не запланировало полёт боком.
	gfoldMaxTilt = 60 * physics.DegToRad
)

// gfoldTouchdownTolerance — при каком промахе цель считается достигнутой, м.
//
// Не круглое число: это утроенный порог шума канала положения навигации
// (sensing.DefaultPositionSensor). Требовать от наведения точности выше
// той, с которой борт вообще знает своё положение, бессмысленно — «промах»
// в пределах этой величины неотличим от нулевого никакими средствами
// самого аппарата.
var gfoldTouchdownTolerance = 3 * sensing.DefaultPositionSensor().NoiseFloor

// gfoldGuidance — состояние контура наведения посадочного импульса.
type gfoldGuidance struct {
	// traj — последняя найденная траектория. nil означает «решения ещё
	// нет»: наведение в этом случае не выдумывает команду, а честно
	// сообщает об этом вызывающей стороне.
	traj *gfold.Trajectory

	// solvedAt — модельное время, на которое посчитана traj, с.
	solvedAt float64

	// group — число камер, при котором решалась задача. Смена группы
	// меняет обе границы тяги скачком, и прежняя траектория может стать
	// не просто неоптимальной, а недопустимой — это отдельный повод
	// пересчитать, не дожидаясь очередного периода.
	group int

	// probedAt — когда в последний раз проверялась достижимость площадки
	// на пассивном участке (см. gfoldIgnitionReady).
	probedAt float64

	// consecutiveFailures — сколько решений подряд не удалось получить.
	// Задаёт выдержку между попытками (см. gfoldRetryInterval): бортовой
	// оптимизатор, которому задача не даётся, не должен молотить её заново
	// каждые две секунды — состояние за это время меняется мало, ответ
	// будет тот же, а такт наведения уходит целиком. То же и в жизни:
	// повторять расчёт чаще, чем меняются исходные данные, бессмысленно.
	consecutiveFailures int

	// command — требуемый вектор тяги (Н, оси ECI), посчитанный РОВНО ОДИН
	// РАЗ за такт (см. updateLandingGuidance). Кэш здесь не ради скорости:
	// вниз по такту требуемая тяга запрашивается трижды по независимым
	// поводам (выбор группы камер, цель ориентации, уставка дросселя), и
	// пересчёт задачи оптимизации на каждый из них означал бы и тройную
	// стоимость, и — хуже — три РАЗНЫХ ответа на одном такте.
	command physics.Vec3
	valid   bool

	// Результат последней проверки достижимости площадки на пассивном
	// участке (см. gfoldIgnitionReady) — по нему видно, ПОЧЕМУ импульс
	// ещё не зажжён: цель недостижима вовсе, решатель не сошёлся или
	// решение есть, но промах пока велик.
	probeStatus gfold.Status
	probeMiss   float64
	probeTof    float64
	probeGate   bool

	// tilt — предельный угол отклонения тяги от вертикали, с которым
	// решалась последняя задача (см. landingPointingLimit). Тем же
	// пределом ограничивается и команда обратной связи.
	tilt float64

	// Диагностика, уходящая в телеметрию.
	status     gfold.Status
	solves     int
	failures   int
	miss       float64
	slack      float64
	timeOfFlig float64
	trackPos   float64
	trackVel   float64
	nodes      int
	step       float64
	fuelPlan   float64
}

// landingFrame — оси посадочной системы координат: начало в точке посадки,
// X на восток, Y на север, Z вверх, всё в осях ECEF (то есть система
// связана с вращающейся Землёй).
//
// Почему во вращающейся, а не в инерциальной. Цель посадки неподвижна
// относительно поверхности, а не относительно звёзд: за девяносто секунд
// посадочного импульса площадка уезжает в инерциальном пространстве почти
// на сорок километров. В связанной с Землёй системе цель стоит на месте, а
// плата за это — силы Кориолиса и центробежная, которые входят в уравнение
// движения. Они линейны по скорости и положению, то есть по переменным
// задачи, и записать их прямо в динамику значило бы потерять её линейность
// вместе со всей выпуклостью. Поэтому они замораживаются по предыдущей
// найденной траектории и уходят в известное поузловое слагаемое (см.
// gfoldBias) — тем же приёмом, что и аэродинамическое сопротивление.
type landingFrame struct {
	origin physics.Vec3 // ECEF-положение точки посадки
	east   physics.Vec3
	north  physics.Vec3
	up     physics.Vec3
	// elapsed — момент, на который построена система (нужен для перевода
	// векторов обратно в ECI).
	elapsed float64
}

// newLandingFrame строит посадочную систему координат вокруг цели возврата.
func (b *Booster) newLandingFrame() landingFrame {
	origin := physics.GeodeticToECEF(b.landingAimPoint())
	local := physics.NewLocalFrame(origin)
	return landingFrame{
		origin:  origin,
		east:    local.East,
		north:   local.North,
		up:      local.Up,
		elapsed: b.elapsed,
	}
}

// project переводит ECI-положение в координаты посадочной системы.
func (f landingFrame) project(positionECI physics.Vec3, elapsed float64) physics.Vec3 {
	d := physics.ECIToECEF(positionECI, elapsed).Sub(f.origin)
	return physics.Vec3{X: d.Dot(f.east), Y: d.Dot(f.north), Z: d.Dot(f.up)}
}

// projectVelocity переводит скорость ОТНОСИТЕЛЬНО ПОВЕРХНОСТИ (ECI-оси) в
// координаты посадочной системы.
func (f landingFrame) projectVelocity(groundVelocityECI physics.Vec3, elapsed float64) physics.Vec3 {
	v := physics.ECIToECEF(groundVelocityECI, elapsed)
	return physics.Vec3{X: v.Dot(f.east), Y: v.Dot(f.north), Z: v.Dot(f.up)}
}

// toECI переводит вектор из посадочной системы обратно в оси ECI.
func (f landingFrame) toECI(v physics.Vec3, elapsed float64) physics.Vec3 {
	ecef := f.east.Scale(v.X).Add(f.north.Scale(v.Y)).Add(f.up.Scale(v.Z))
	return physics.ECEFToECI(ecef, elapsed)
}

// rotationVector возвращает угловую скорость Земли в осях посадочной
// системы — нужна для сил Кориолиса и центробежной.
func (f landingFrame) rotationVector() physics.Vec3 {
	w := physics.Vec3{Z: physics.EarthAngularVelocity} // в ECEF ось вращения — Z
	return physics.Vec3{X: w.Dot(f.east), Y: w.Dot(f.north), Z: w.Dot(f.up)}
}

// gfoldState возвращает положение и скорость ступени в посадочной системе
// по ПОКАЗАНИЯМ навигации.
func (b *Booster) gfoldState(nav orbit.NavState, f landingFrame) gfold.State {
	return gfold.State{
		Position: f.project(nav.Position, b.elapsed),
		Velocity: f.projectVelocity(nav.GroundRelativeVelocity, b.elapsed),
	}
}

// gfoldStep выбирает шаг сетки под остаток времени полёта.
func gfoldStep(timeToGo float64) float64 {
	step := timeToGo / gfoldNodes
	return physics.Clamp(step, gfoldMinStep, gfoldMaxStep)
}

// landingPointingLimit возвращает предельный угол отклонения тяги от
// вертикали, который корпус способен УДЕРЖАТЬ на текущем скоростном напоре,
// рад.
//
// Это тот же вопрос, на который отвечает landingAuthorityConstrainedThrust
// (Stage 4.6), но заданный на этап раньше. Там наклон, уже назначенный
// наведением, урезается постфактум; здесь ограничение входит в САМУ задачу
// оптимизации, и наведение просто не может запланировать траекторию, для
// которой не хватит управляющего момента. Разница существенна: урезанная
// постфактум команда рассогласуется с траекторией, по которой считался
// весь остальной план, а ограничение внутри задачи учитывается всеми
// остальными узлами сразу.
//
// Считается двоичным поиском по углу: для каждого кандидата берётся
// предсказанный аэродинамический момент (landingPredictedAoAAndAeroTorque)
// и сравнивается с реально располагаемым (landingAvailableCorrectiveTorque)
// — те же две функции, которыми пользуется и постфактумное ограничение, а
// не отдельная упрощённая оценка.
func (b *Booster) landingPointingLimit(nav orbit.NavState, indices []int) float64 {
	mach := 0.0
	if ss := physics.Atmosphere(nav.Altitude).SoundSpeed; ss > 0 {
		mach = nav.AirRelativeVelocity.Norm() / ss
	}
	// Момент считается для ЗАДАННОГО состава камер, а не для работающих
	// сейчас: на пассивном участке (проверка «пора ли зажигать») камеры
	// ещё погашены, и располагаемый момент от них нулевой — предел,
	// посчитанный по ним, запретил бы любой наклон и объявил бы
	// достижимую площадку недостижимой.
	available := b.availableCorrectiveTorqueFor(nav, mach, indices)

	// Ось наклона выбирается в плоскости «вертикаль — направление сноса»:
	// именно туда наведение и захочет отклонить тягу, чтобы гасить снос.
	lateral := landingHorizontalVelocity(nav)
	if lateral.Norm() < 1e-6 {
		lateral = nav.Frame.East
	}
	lateral = lateral.Unit()

	holds := func(angle float64) bool {
		dir := nav.Frame.Up.Scale(math.Cos(angle)).Add(lateral.Scale(math.Sin(angle)))
		_, aero := b.landingPredictedAoAAndAeroTorque(nav, dir, mach)
		return available >= aero
	}

	if holds(gfoldMaxTilt) {
		return gfoldMaxTilt
	}
	lo, hi := 0.0, gfoldMaxTilt
	for i := 0; i < 12; i++ {
		mid := (lo + hi) / 2
		if holds(mid) {
			lo = mid
		} else {
			hi = mid
		}
	}
	return lo
}

// gfoldVehicle собирает паспортную часть задачи для группы из n камер.
func (b *Booster) gfoldVehicle(nav orbit.NavState, engines, nodes int,
	altitudes []float64, indices []int) gfold.Vehicle {

	stage := b.Config.FirstStage

	// Границы тяги — по ТЕКУЩЕЙ группе камер, обе: и верхняя, и нижняя.
	//
	// Нижняя граница здесь не формальность. Тринадцать камер на минимальном
	// газе дают около 18 м/с² — вдвое больше веса, то есть ступень с
	// тринадцатью работающими камерами не может снижаться в принципе, она
	// может только тормозить. Отсюда двусторонняя область допустимых
	// состояний: высоты должно хватать, чтобы погасить снижение на полной
	// тяге, и одновременно её не должно быть СЛИШКОМ много — иначе аппарат
	// остановится в воздухе и дальше пойдёт вверх. Именно эту область и
	// проверяет условие розжига (см. gfoldIgnitionBand): пока состояние в
	// ней, задача с постоянной группой разрешима, и никакого расписания
	// гашения камер внутри плана не нужно.
	//
	// Смена группы происходит снаружи (updateLandingEngineGroup, по тому же
	// критерию, что и всегда) и служит поводом немедленно пересчитать
	// задачу с новыми границами — см. gfoldUpdate. Попытка запланировать
	// расписание гашения ВНУТРИ траектории была опробована и отклонена:
	// расписание строится по опорной траектории, опорная — по расписанию,
	// и на первом же решении, когда опорной ещё нет, замыкание этого круга
	// давало то заведомо неверную группу, то заведомо неразрешимую задачу.
	rho1 := make([]float64, nodes+1)
	rho2 := make([]float64, nodes+1)
	minThrottle := stage.MinThrottle
	if len(b.propulsion.Engines) > 0 {
		minThrottle = b.propulsion.Engines[0].Config.MinThrottle
	}
	for k := range rho1 {
		alt := nav.Altitude
		if altitudes != nil && k < len(altitudes) {
			alt = altitudes[k]
		}
		per := vehicle.ThrustAtAltitude(stage, physics.Atmosphere(alt).Pressure)
		rho2[k] = per * float64(engines)
		rho1[k] = rho2[k] * minThrottle
	}

	// Удельный импульс — паспортная характеристика двигателя, не измерение
	// (см. NavState.ISP). Берётся на текущей высоте, между уровнем моря и
	// вакуумом, тем же интерполированием, что и тяга.
	isp := stage.VacuumISP
	if p0 := physics.Atmosphere(0).Pressure; p0 > 0 {
		ratio := physics.Clamp(physics.Atmosphere(nav.Altitude).Pressure/p0, 0, 1)
		isp = stage.VacuumISP - (stage.VacuumISP-stage.SeaLevelISP)*ratio
	}

	// Нижняя граница массы — сухая масса ПЛЮС неприкосновенный остаток.
	// Наведение не имеет права планировать посадку «в ноль по топливу»:
	// у настоящей ступени часть остатка физически недоступна (заборные
	// устройства, осадка), и планировать её расход значит планировать
	// срыв горения на последних секундах.
	dry := b.dryMass() + boosterFuelReserveFraction*b.Config.FirstStage.FuelReserve

	return gfold.Vehicle{
		WetMass:        b.dryMass() + b.state.FuelMass,
		DryMass:        dry,
		Alpha:          1 / (isp * physics.G0),
		Rho1:           rho1,
		Rho2:           rho2,
		ThetaMax:       b.gfoldTilt(nav, indices),
		PointAxis:      physics.Vec3{Z: 1},
		GlideSlope:     gfoldGlideSlope,
		GlideSkipNodes: 2,
		GlideApexDepth: gfoldGlideApex,
	}
}

// gfoldBias считает известное поузловое ускорение, не зависящее от
// управления: аэродинамическое сопротивление, силы Кориолиса и
// центробежную, поправку на изменение силы тяжести с высотой.
//
// Опорная траектория (pos/vel/mass) — либо предыдущее решение, либо
// баллистический прогноз, если решения ещё нет. В этом и состоит приём:
// величины, которые в честной постановке зависели бы от переменных задачи
// (сопротивление — квадратично по скорости, Кориолис — линейно), берутся
// по УЖЕ известной близкой траектории и входят в задачу как константы.
// Ошибка такого замораживания второго порядка малости и снимается
// повторным решением (итерация Пикара): поправки составляют единицы
// процентов от командного ускорения, и одного уточнения достаточно.
//
// Ветер сюда не входит намеренно: бортовая система не измеряет ветер и не
// может его знать (см. заголовок booster_sensing.go). Сопротивление
// считается по скорости относительно поверхности — то есть наведение
// систематически ошибается на ветровую составляющую, ровно как ошибался бы
// настоящий борт, и эта ошибка гасится обратной связью, а не отсутствует
// по построению.
func (b *Booster) gfoldBias(f landingFrame, gravity physics.Vec3,
	pos, vel []physics.Vec3, mass []float64) []physics.Vec3 {

	omega := f.rotationVector()
	radius := b.Config.Diameter / 2
	frontal := math.Pi * radius * radius
	side := b.Config.FirstStageLength * b.Config.Diameter
	padAlt := b.landingAimPoint().Altitude

	// Опорные значения в самой точке посадки: они уже учтены постоянным
	// членом gravity, и в поузловую поправку должна входить только разница.
	gOrigin := physics.GravityAcceleration(f.origin)
	cfOrigin := centrifugalAccel(f.origin)

	out := make([]physics.Vec3, len(pos))
	for k := range pos {
		r := pos[k]

		// Высота узла над поверхностью с поправкой на кривизну: при
		// удалении в десятки километров плоская модель завышала бы её на
		// сотни метров, а плотность воздуха меняется с высотой заметно.
		alt := r.Z + padAlt - (r.X*r.X+r.Y*r.Y)/(2*physics.EarthRadius)
		atm := physics.Atmosphere(alt)

		// Сопротивление.
		var drag physics.Vec3
		if m := mass[k]; m > 0 {
			v := vel[k]
			if speed := v.Norm(); speed > 1 && atm.Density > 0 && atm.SoundSpeed > 0 {
				q := 0.5 * atm.Density * speed * speed

				// Сопротивление — по ТОЙ ЖЕ формуле, что и у физики, и у
				// остальных прогнозов возврата (physics.BodyDragArea), и по
				// ТОЙ ЖЕ ориентации, в которой ступень проходит терминальный
				// участок: двигателями вперёд.
				//
				// Здесь стоял коэффициент обтекаемого тела носом вперёд
				// (DragCoefficient, около 0.35) по одному миделю. Ступень
				// же идёт тупым торцом, и её коэффициент на сверхзвуке около
				// 1.8 — впятеро больше. Ровно это расхождение однажды уже
				// нашли и починили в наведении посадочного импульса (см.
				// комментарий к physics.BodyDragArea), но в поузловую
				// поправку выпуклой задачи та правка не дошла: терминальная
				// задача планировала торможение по аэродинамике, которой у
				// этой ступени нет, и недобирала его впятеро.
				//
				// Угол атаки здесь 180° не по допущению, а по построению
				// задачи: тормозить, не выставив тягу против скорости,
				// нельзя, и весь терминальный участок корпус стоит
				// ретроградом. На этом угле поперечный член обращается в
				// ноль, и модель площади сводится к осевому коэффициенту по
				// миделю — то есть выбор формулы тут ни на что, кроме
				// величины Cd, не влияет.
				area := physics.BodyDragArea(speed/atm.SoundSpeed, 180, frontal, side)
				drag = v.Scale(-q * area / (m * speed))
			}
		}

		// Кориолис и центробежная — в осях посадочной системы.
		coriolis := omega.Cross(vel[k]).Scale(-2)

		ecef := f.origin.Add(f.east.Scale(r.X)).Add(f.north.Scale(r.Y)).Add(f.up.Scale(r.Z))
		dGrav := physics.GravityAcceleration(ecef).Sub(gOrigin)
		dCentri := centrifugalAccel(ecef).Sub(cfOrigin)
		extra := dGrav.Add(dCentri)
		local := physics.Vec3{
			X: extra.Dot(f.east),
			Y: extra.Dot(f.north),
			Z: extra.Dot(f.up),
		}

		out[k] = drag.Add(coriolis).Add(local)
	}
	_ = gravity
	return out
}

// centrifugalAccel — центробежное ускорение в точке ECEF, ось вращения Z.
// Не «поправка», а физическая сила инерции: посадочная система связана с
// вращающейся Землёй, и без этого члена уравнение движения в ней было бы
// просто неверным.
func centrifugalAccel(ecef physics.Vec3) physics.Vec3 {
	w := physics.EarthAngularVelocity
	return physics.Vec3{X: ecef.X * w * w, Y: ecef.Y * w * w}
}

// gfoldGravity возвращает постоянный член ускорения в посадочной системе:
// притяжение плюс центробежная сила в самой точке посадки.
func (f landingFrame) gfoldGravity() physics.Vec3 {
	a := physics.GravityAcceleration(f.origin).Add(centrifugalAccel(f.origin))
	return physics.Vec3{X: a.Dot(f.east), Y: a.Dot(f.north), Z: a.Dot(f.up)}
}

// brakingReference строит грубую опорную траекторию для ПЕРВОГО решения,
// когда предыдущего ещё нет: торможение на предельной тяге текущей группы
// камер до остановки снижения.
//
// Свободное падение в этой роли не годится, и это не мелочь. Опорная
// траектория нужна не только для замораживания сопротивления, но и для
// расписания гашения камер (gfoldThrustSchedule), а расписание решает,
// когда снижение станет достаточно малым, чтобы хватило меньшей группы.
// По свободному падению снижение НИКОГДА не становится малым — оно только
// растёт, — и расписание остаётся на тринадцати камерах до конца
// траектории. А тринадцать камер на минимальном газе дают вдвое больше
// веса, то есть аппарат, который не умеет снижаться: задача неразрешима
// почти при любом времени полёта. Измерено: решатель переставал сходиться
// на пятой секунде импульса и не сходился больше ни разу.
//
// Торможение на полной тяге — грубая, но качественно ВЕРНАЯ опора: она
// проходит те же этапы, что и настоящий импульс (быстрое снижение,
// торможение, малая скорость у земли), поэтому расписание получается
// осмысленным. Уточняется она первым же решением (см. gfoldSolve).
func brakingReference(st gfold.State, gravity physics.Vec3, mass, alpha, thrust float64,
	nodes int, step float64) (pos, vel []physics.Vec3, m []float64) {

	pos = make([]physics.Vec3, nodes+1)
	vel = make([]physics.Vec3, nodes+1)
	m = make([]float64, nodes+1)

	p, v, mm := st.Position, st.Velocity, mass
	for k := 0; k <= nodes; k++ {
		pos[k], vel[k], m[k] = p, v, mm

		// Тяга направлена против скорости — так и тормозят.
		acc := gravity
		if speed := v.Norm(); speed > 1 && mm > 0 {
			acc = acc.Add(v.Scale(-thrust / (mm * speed)))
		}
		p = p.Add(v.Scale(step)).Add(acc.Scale(step * step / 2))
		v = v.Add(acc.Scale(step))
		mm -= alpha * thrust * step

		// Ниже поверхности и ниже сухой массы опора смысла не имеет:
		// дальше она просто держится на месте, чтобы расписание видело
		// «медленно и низко», а не улетающие в минус числа.
		if p.Z < 0 {
			p.Z = 0
			v = physics.Vec3{}
		}
		if mm < 1 {
			mm = 1
		}
	}
	return pos, vel, m
}

// resampleReference пересаживает предыдущую траекторию на новую сетку —
// линейным восполнением, тем же, на котором она решалась.
func resampleReference(prev *gfold.Trajectory, since float64, nodes int, step float64) (pos, vel []physics.Vec3, m []float64) {
	pos = make([]physics.Vec3, nodes+1)
	vel = make([]physics.Vec3, nodes+1)
	m = make([]float64, nodes+1)
	for k := 0; k <= nodes; k++ {
		t := since + float64(k)*step
		_, p, v := prev.Command(t)
		pos[k], vel[k], m[k] = p, v, prev.MassAt(t)
	}
	return pos, vel, m
}

// gfoldSolve решает задачу наведения от текущего (по показаниям) состояния.
//
// timeToGo — оценка остатка времени полёта, задающая шаг сетки и центр
// перебора по числу узлов. Перебор ведётся вокруг неё, а не по всему
// допустимому диапазону: время полёта меняется от такта к такту медленно и
// непрерывно, и полный поиск нужен только при первом решении.
func (b *Booster) gfoldSolve(nav orbit.NavState, engines int, indices []int,
	timeToGo float64, wide bool) *gfold.Trajectory {

	frame := b.newLandingFrame()
	st := b.gfoldState(nav, frame)
	gravity := frame.gfoldGravity()

	step := gfoldStep(timeToGo)
	center := int(math.Round(timeToGo / step))
	if center < 4 {
		center = 4
	}

	lo, hi := center-2, center+2
	opts := gfold.PlanOptions{Coarse: 3, Refine: 2}
	if wide {
		// Первое решение (или проверка достижимости на пассивном участке):
		// опорного времени полёта ещё нет, и перебирать приходится широко.
		lo, hi = 4, 3*gfoldNodes
		opts = gfold.PlanOptions{Coarse: 6, Refine: 5}
	}
	if lo < 4 {
		lo = 4
	}
	opts.MinNodes, opts.MaxNodes = lo, hi

	mass := b.dryMass() + b.state.FuelMass

	// Опорная траектория для замораживания сопротивления и сил инерции.
	// Опорная берётся только у ЖИВОЙ траектории. Пережившая своё время
	// полёта не годится: за её концом Trajectory.Command отдаёт
	// терминальный узел — нулевую высоту и нулевую скорость, — и
	// построенное по такой опоре расписание гашения камер немедленно
	// сваливается на три камеры на всю траекторию (на нулевой скорости
	// тормозить нечего). Дальше задача с тремя камерами при скорости
	// снижения около километра в секунду неразрешима — и наведение
	// объявляет неразрешимым манёвр, который на самом деле выполним.
	// Измерено: ровно с этого начиналась потеря ступени, все последующие
	// пересчёты возвращали «нет решения».
	var refPos, refVel []physics.Vec3
	var refMass []float64
	prev := b.gfold.traj
	alive := prev != nil && prev.Status == gfold.StatusOptimal &&
		b.elapsed-b.gfold.solvedAt <= prev.TimeOfFlight()
	if alive {
		refPos, refVel, refMass = resampleReference(prev, b.elapsed-b.gfold.solvedAt, hi, step)
	} else {
		per := vehicle.ThrustAtAltitude(b.Config.FirstStage, physics.Atmosphere(nav.Altitude).Pressure)
		isp := b.Config.FirstStage.VacuumISP
		refPos, refVel, refMass = brakingReference(st, gravity, mass,
			1/(isp*physics.G0), per*float64(engines), hi, step)
	}

	setup := gfold.Setup{
		Vehicle:        b.gfoldVehicle(nav, engines, hi, altitudesOf(refPos, b.landingAimPoint().Altitude), indices),
		State:          st,
		Gravity:        gravity,
		Bias:           b.gfoldBias(frame, gravity, refPos, refVel, refMass),
		TargetVelocity: physics.Vec3{Z: -landingTouchdownSpeed},
		Step:           step,
	}

	// Постановка: пока площадка достижима — точное попадание с минимумом
	// топлива (это и есть G-FOLD в своём прямом назначении: дальний увод к
	// заданной точке за минимальный расход). Как только достижимость
	// теряется — мягкая посадка со свободной точкой касания.
	//
	// Промежуточная постановка «минимум промаха» в замкнутом контуре не
	// используется намеренно: разбор — у gfold.Objective.
	setup.Objective = gfold.ObjectiveReachTarget
	traj := gfold.Plan(setup, opts)
	if traj.Status != gfold.StatusOptimal {
		setup.Objective = gfold.ObjectiveSoftLanding
		traj = gfold.Plan(setup, opts)
	}

	// Узкий перебор вокруг предыдущего времени полёта не нашёл решения —
	// повторяем широким. Узкое окно строится в предположении, что
	// потребное время полёта меняется медленно, и обычно так и есть; но
	// когда аппарат заметно уходит от плана (или план оказался
	// оптимистичным), потребное время может скакнуть в разы, и окно
	// шириной в пару узлов его просто не содержит. Измерено: наведение
	// объявляло задачу неразрешимой на одиннадцати километрах, тогда как
	// решение существовало — просто вдвое длиннее, чем искали. Широкий
	// перебор дороже, но платится он только на отказе, а не каждый такт.
	if traj.Status != gfold.StatusOptimal && !wide {
		wideOpts := gfold.PlanOptions{MinNodes: 4, MaxNodes: 3 * gfoldNodes, Coarse: 6, Refine: 5}
		for _, obj := range []gfold.Objective{gfold.ObjectiveReachTarget, gfold.ObjectiveSoftLanding} {
			setup.Objective = obj
			if again := gfold.Plan(setup, wideOpts); again.Status == gfold.StatusOptimal {
				traj = again
				break
			}
		}
	}
	if traj.Status != gfold.StatusOptimal {
		return &traj
	}

	// Уточнение: сопротивление и силы инерции пересчитываются по найденной
	// траектории и задача решается ещё раз — уже на выбранном числе узлов,
	// без повторного перебора по времени полёта.
	refined := setup
	refined.Nodes = traj.Nodes
	refined.Objective = setup.Objective
	refined.Bias = b.gfoldBias(frame, gravity, traj.Position, traj.Velocity, traj.Mass)
	refined.Vehicle = b.gfoldVehicle(nav, engines, traj.Nodes,
		altitudesOf(traj.Position, b.landingAimPoint().Altitude), indices)
	if second := gfold.Solve(refined); second.Status == gfold.StatusOptimal {
		return &second
	}
	return &traj
}

// altitudesOf возвращает высоты узлов опорной траектории над поверхностью
// с поправкой на кривизну — по ним берётся тяга камеры (она зависит от
// внешнего давления).
func altitudesOf(pos []physics.Vec3, padAltitude float64) []float64 {
	out := make([]float64, len(pos))
	for k, r := range pos {
		out[k] = r.Z + padAltitude - (r.X*r.X+r.Y*r.Y)/(2*physics.EarthRadius)
	}
	return out
}

// runningEngineIndices — индексы работающих камер (для оценки
// располагаемого управляющего момента).
func (b *Booster) runningEngineIndices() []int {
	idx := make([]int, 0, len(b.propulsion.Engines))
	for i, e := range b.propulsion.Engines {
		if e.Running {
			idx = append(idx, i)
		}
	}
	return idx
}

// gfoldUpdate ведёт контур наведения посадочного импульса на текущем такте:
// решает задачу заново, когда для этого есть повод, и возвращает требуемое
// УСКОРЕНИЕ от тяги в осях посадочной системы.
//
// Между пересчётами команда берётся с уже найденной траектории тем же
// линейным восполнением, на котором она решалась, плюс обратная связь по
// расхождению с ней (см. gfoldTrack). Разделение здесь принципиальное:
// G-FOLD — не регулятор, а генератор опорной траектории. Пытаться
// использовать его как регулятор (пересчитывать каждый такт и брать нулевой
// узел как команду) значило бы платить полную стоимость решения сто раз в
// секунду и при этом остаться без демпфирования по ошибке слежения.
func (b *Booster) gfoldUpdate(nav orbit.NavState, dt float64) (physics.Vec3, bool) {
	if !b.gfoldApplicable(nav) {
		b.gfold.traj = nil
		return physics.Vec3{}, false
	}
	g := &b.gfold
	frame := b.newLandingFrame()
	st := b.gfoldState(nav, frame)
	engines := b.propulsion.Commissioned
	if engines <= 0 {
		engines = boosterLandingHigh
	}

	since := b.elapsed - g.solvedAt
	timeToGo := 0.0
	if g.traj != nil && g.traj.Status == gfold.StatusOptimal {
		timeToGo = g.traj.TimeOfFlight() - since
	}

	// Поводы пересчитать: решения ещё нет; истёк период; сменилась группа
	// камер (обе границы тяги изменились скачком, и прежняя траектория
	// может стать не просто неоптимальной, а недопустимой).
	need := g.traj == nil || g.traj.Status != gfold.StatusOptimal ||
		since >= gfoldRetryInterval(g.consecutiveFailures) || g.group != engines

	// Ниже этого остатка времени пересчитывать нечего (см. gfoldMinTimeToGo):
	// добивается посадка по последней траектории с обратной связью.
	if g.traj != nil && g.traj.Status == gfold.StatusOptimal && timeToGo < gfoldMinTimeToGo {
		need = false
	}

	if need {
		hint := timeToGo
		wide := false
		if hint < gfoldMinTimeToGo {
			// Опорного времени полёта нет (первое решение) — придётся
			// перебирать широко.
			hint = float64(gfoldNodes) * gfoldStep(30)
			wide = true
		}
		traj := b.gfoldSolve(nav, engines, b.runningEngineIndices(), hint, wide)
		g.solves++
		g.status = traj.Status
		if traj.Status == gfold.StatusOptimal {
			g.traj = traj
			g.solvedAt = b.elapsed
			g.group = engines
			g.miss = traj.Miss
			g.slack = traj.Slack
			g.timeOfFlig = traj.TimeOfFlight()
			g.nodes = traj.Nodes
			g.step = traj.Step
			g.fuelPlan = traj.FuelUsed
			since = 0
			g.consecutiveFailures = 0
		} else {
			// Провалившийся пересчёт НЕ отменяет предыдущее решение:
			// лететь по устаревшему, но допустимому плану безопаснее, чем
			// по несуществующему. Отказ считается и виден в телеметрии.
			g.failures++
			g.consecutiveFailures++
			g.solvedAt = b.elapsed
		}
	}

	// План, переживший собственное расчётное время полёта, больше ничего
	// не описывает: опорная точка в нём стоит на месте (Trajectory.Command
	// за концом отдаёт последний узел), а аппарат продолжает двигаться —
	// ошибка слежения растёт линейно, обратная связь по ней требует
	// всё большего ускорения, и наведение начинает бороться с собственным
	// устаревшим планом. Измерено: ошибка доходила до четырёх километров,
	// а команда разворачивала тягу ВНИЗ. Такой план объявляется
	// недействительным, и управление уходит на резервный закон
	// (см. landingDesiredThrust).
	if g.traj == nil || g.traj.Status != gfold.StatusOptimal || since > g.traj.TimeOfFlight() {
		return physics.Vec3{}, false
	}
	return b.gfoldTrack(g.traj, st, since), true
}

// gfoldTrack — закон слежения за опорной траекторией.
//
//	u = u_опорн + Kp·(r_опорн − r) + Kd·(v_опорн − v)
//
// Коэффициенты — критически демпфированные (ζ=1), с собственной частотой
// ωn = min(3/t_ост, ω_макс): чем меньше остаётся времени, тем жёстче
// сходится ошибка, но не жёстче предела. Предел ω_макс привязан к полосе
// контура ориентации (b.attitude.Config.Bandwidth), а не назначен числом:
// контур положения обязан быть заметно медленнее контура ориентации,
// иначе они начинают бороться друг с другом — команда по положению просит
// довернуть корпус быстрее, чем корпус доворачивается, и ошибка растёт от
// самой попытки её исправить.
func (b *Booster) gfoldTrack(traj *gfold.Trajectory, st gfold.State, since float64) physics.Vec3 {
	accel, refPos, refVel := traj.Command(since)

	timeToGo := traj.TimeOfFlight() - since
	if timeToGo < gfoldMinTimeToGo {
		timeToGo = gfoldMinTimeToGo
	}

	omegaMax := 0.5
	if bw := b.attitude.Config.Bandwidth; bw > 0 {
		omegaMax = bw / 5
	}
	omega := math.Min(3/timeToGo, omegaMax)

	posErr := refPos.Sub(st.Position)
	velErr := refVel.Sub(st.Velocity)

	b.gfold.trackPos = posErr.Norm()
	b.gfold.trackVel = velErr.Norm()

	cmd := accel.
		Add(posErr.Scale(omega * omega)).
		Add(velErr.Scale(2 * omega))

	// Направление команды не выпускается за тот же конус, в котором
	// решалась сама задача. Без этого ограничения обратная связь по
	// большой ошибке способна развернуть требуемую тягу куда угодно, вплоть
	// до направления ВНИЗ (наблюдалось: наклон до 150° от вертикали —
	// аппарат разгонялся к воде вместо торможения). Ограничение по модулю
	// такого не ловит: команда остаётся в границах тяги, будучи направлена
	// в противоположную сторону.
	//
	// Урезается именно направление, модуль сохраняется: не хватает не
	// тяги, а разрешённого наклона.
	return clampTilt(cmd, b.gfold.tilt)
}

// clampTilt возвращает вектор, отклонённый от вертикали не более чем на
// limit, с тем же модулем.
func clampTilt(v physics.Vec3, limit float64) physics.Vec3 {
	n := v.Norm()
	if n < 1e-9 || limit <= 0 {
		return physics.Vec3{Z: n}
	}
	dir := v.Scale(1 / n)
	cosA := physics.Clamp(dir.Z, -1, 1)
	if cosA >= math.Cos(limit) {
		return v
	}
	// Горизонтальная часть направления задаёт плоскость поворота; в ней
	// вектор кладётся ровно на образующую конуса.
	horiz := physics.Vec3{X: dir.X, Y: dir.Y}
	if horiz.Norm() < 1e-12 {
		return physics.Vec3{Z: n}
	}
	horiz = horiz.Unit()
	return physics.Vec3{Z: math.Cos(limit)}.
		Add(horiz.Scale(math.Sin(limit))).
		Scale(n)
}

// gfoldDesiredThrust возвращает требуемый вектор тяги (Н) в ОСЯХ ECI —
// то, что дальше по такту отрабатывают ограничение по управляющему моменту,
// выбор группы камер и дроссель.
//
// Модуль загоняется в физические границы текущей группы: команду ниже
// нижнего предела дросселирования двигательная установка всё равно поднимет
// до него сама (propulsion.Engine), и наведение, не учитывающее этого,
// планировало бы траекторию, по которой аппарат заведомо не полетит.
func (b *Booster) gfoldDesiredThrust(nav orbit.NavState, dt float64) (physics.Vec3, bool) {
	accel, ok := b.gfoldUpdate(nav, dt)
	if !ok {
		return physics.Vec3{}, false
	}

	mass := b.dryMass() + b.state.FuelMass
	if mass <= 0 {
		return physics.Vec3{}, false
	}

	thrust := accel.Scale(mass)
	perEngine := vehicle.ThrustAtAltitude(b.Config.FirstStage, physics.Atmosphere(nav.Altitude).Pressure)
	engines := float64(b.propulsion.Commissioned)
	if perEngine > 0 && engines > 0 {
		minThrottle := b.Config.FirstStage.MinThrottle
		if len(b.propulsion.Engines) > 0 {
			minThrottle = b.propulsion.Engines[0].Config.MinThrottle
		}
		hi := perEngine * engines
		lo := hi * minThrottle
		if n := thrust.Norm(); n > 1e-9 {
			thrust = thrust.Scale(physics.Clamp(n, lo, hi) / n)
		} else {
			thrust = nav.Frame.Up.Scale(lo)
			return thrust, true
		}
	}

	frame := b.newLandingFrame()
	return frame.toECI(thrust, b.elapsed), true
}

// updateLandingGuidance пересчитывает команду наведения посадочного
// импульса. Вызывается РОВНО ОДИН РАЗ за такт — из updatePhase, до выбора
// группы камер (см. поле gfoldGuidance.command).
func (b *Booster) updateLandingGuidance(nav orbit.NavState, dt float64) {
	vec, ok := b.gfoldDesiredThrust(nav, dt)
	b.gfold.command, b.gfold.valid = vec, ok
}

// gfoldReachable — необходимое (но не достаточное) условие того, что
// площадка вообще достижима: существует ли время t, за которое аппарат,
// разгоняясь В ЛЮБУЮ сторону предельной тягой, успевает свести расхождение
// с целью к нулю.
//
//	‖r_цель − r₀ − v₀·t − ½·g·t²‖ ≤ ½·(ρ₂/m)·t²
//
// Это ослабление настоящей задачи: отброшены нижняя граница тяги, расход
// массы, сопротивление, ограничение на угол тяги и посадочный конус —
// то есть множество достижимых точек здесь заведомо ШИРЕ настоящего. Если
// условие не выполняется, решения нет наверняка, и запускать решатель
// незачем. Проверка стоит микросекунды против десятков миллисекунд на
// решение, а на пассивном участке она отсекает большую часть проверок.
func gfoldReachable(st gfold.State, gravity physics.Vec3, aMax, tMin, tMax float64) bool {
	if aMax <= 0 || tMax <= tMin {
		return false
	}
	const samples = 24
	for i := 0; i <= samples; i++ {
		t := tMin + (tMax-tMin)*float64(i)/samples
		if t <= 0 {
			continue
		}
		drift := st.Position.Add(st.Velocity.Scale(t)).Add(gravity.Scale(t * t / 2))
		if drift.Norm() <= 0.5*aMax*t*t {
			return true
		}
	}
	return false
}

// gfoldIgnitionReady решает, пора ли зажигать посадочный импульс.
//
// Критерий — не высота и не таймер, а ОБЛАСТЬ РАЗРЕШИМОСТИ самой задачи
// наведения, и он двусторонний.
//
// Снизу всё привычно: высоты должно хватить, чтобы погасить снижение на
// полной тяге группы, иначе тормозить уже поздно.
//
// Сверху — то, чего у прежнего критерия не было вовсе. Тринадцать камер
// даже на нижнем пределе дросселирования дают около 18 м/с², вдвое больше
// веса: ступень с зажжённой посадочной группой не умеет снижаться, она
// умеет только тормозить. Значит, зажечь СЛИШКОМ РАНО — такая же ошибка,
// как зажечь слишком поздно: аппарат погасит снижение высоко над водой,
// после чего начнёт набирать высоту, и никакая последующая работа
// наведения этого не исправит — топливо будет потрачено на подъём. Ровно
// это и наблюдалось: ступень «зависала» на десяти километрах и уходила
// вверх, выжигая остаток.
//
//	v²/(2·(ρ₂/m − g))  ≤  h  ≤  v²/(2·(ρ₁/m − g))
//
// Розжиг — по НИЖНЕЙ границе с запасом (landingBurnIgnitionAltitude,
// кинематический тормозной путь на доле landingDecelBudget располагаемого
// замедления), а не по верхней. Розжиг у верхней границы был опробован и
// отклонён: он выглядит привлекательно (чем раньше зажечь, тем больше
// свободы у наведения), но область разрешимости после розжига СЖИМАЕТСЯ —
// на минимальном газе тринадцати камер ступень тормозит, скорость падает,
// а вместе с ней и верхняя граница v²/(2·(ρ₁/m − g)), причём быстрее, чем
// убывает высота. Аппарат выпадает из области сверху через десяток секунд
// после розжига и остаток пути идёт без плана (измерено: розжиг на 51.7 км
// при границах 12.5…52.1 км, потеря разрешимости на тринадцатой секунде).
//
// Розжиг у нижней границы с запасом лежит внутри области с полуторным
// запасом по обеим сторонам (отношение границ равно отношению предельного
// и минимального замедления, около четырёх) и оставляет наведению именно
// тот участок, для которого группа и предназначена, — торможение.
//
// Границы области при этом считаются и публикуются (gfoldIgnitionBand):
// по ним видно, в каком месте своей области разрешимости идёт импульс.
func (b *Booster) gfoldIgnitionReady(nav orbit.NavState) bool {
	alt, feasible := b.landingBurnIgnitionAltitude(nav)

	// Отказ поиска не должен превращаться в розжиг где попало.
	//
	// Когда ни одна ступенька лестницы не даёт мягкого касания, поиск
	// возвращает ТЕКУЩУЮ высоту (см. landingBurnIgnitionAltitudePredicted),
	// а «текущая высота» в этой проверке означает «зажигай сейчас» — где бы
	// ступень ни находилась. Признак «зажигать уже поздно» имеет смысл
	// только внутри области разрешимости: выше её верхней границы даже
	// минимальный газ гасит снижение раньше земли, то есть опоздать там
	// физически невозможно, и розжиг там — не торможение, а зависание.
	//
	// Наблюдалось на опытной модели сопротивления пассивного участка (см.
	// коммент к coastAngleOfAttack): посадочный импульс на тридцати
	// километрах, три сотни тонн в никуда, касание на тридцати м/с. Ограничение
	// относится ИМЕННО к отказу поиска: годную высоту выше границы
	// (лестница берёт её с запасом на разворот) урезать нельзя — это
	// отняло бы сотню-другую метров у настоящего торможения.
	if !feasible {
		if _, hi, ok := b.gfoldIgnitionBand(nav); ok && alt > hi {
			return false
		}
	}
	return nav.Altitude <= alt
}

// gfoldIgnitionBand возвращает границы высоты, в которых задача посадки
// разрешима для группы, зажигаемой на посадочный импульс. ok=false, если
// снижения ещё нет (ступень идёт вверх) или данных недостаточно.
func (b *Booster) gfoldIgnitionBand(nav orbit.NavState) (lo, hi float64, ok bool) {
	descent := -nav.RadialVelocity
	if descent <= 0 {
		return 0, 0, false
	}
	mass := b.dryMass() + b.state.FuelMass
	perEngine := vehicle.ThrustAtAltitude(b.Config.FirstStage, physics.Atmosphere(nav.Altitude).Pressure)
	if mass <= 0 || perEngine <= 0 {
		return 0, 0, false
	}
	minThrottle := b.Config.FirstStage.MinThrottle
	if len(b.propulsion.Engines) > 0 {
		minThrottle = b.propulsion.Engines[0].Config.MinThrottle
	}

	group := float64(boosterLandingHigh)
	g := physics.GravityMagnitudeAtAltitude(nav.Altitude)
	drag := b.dragDecel(nav)

	aMax := perEngine*group/mass - g + drag
	aMin := perEngine*group*minThrottle/mass - g + drag
	if aMax <= 0 {
		return 0, 0, false
	}
	lo = landingStoppingDistance(descent, aMax)
	if aMin <= 0 {
		return lo, math.Inf(1), true
	}
	return lo, landingStoppingDistance(descent, aMin), true
}

// gfoldStatusString — исход последнего решения задачи наведения для
// телеметрии. Пустая строка, пока задача ни разу не решалась (до
// посадочного импульса): пустое поле честнее выдуманного «optimal».
func gfoldStatusString(b *Booster) string {
	if b.gfold.solves == 0 {
		return ""
	}
	return b.gfold.status.String()
}

// gfoldTilt возвращает предельный угол отклонения тяги и запоминает его:
// тем же пределом ограничивается не только сама задача, но и команда
// обратной связи (см. gfoldTrack).
func (b *Booster) gfoldTilt(nav orbit.NavState, indices []int) float64 {
	b.gfold.tilt = b.landingPointingLimit(nav, indices)
	return b.gfold.tilt
}

// gfoldApplicable сообщает, находится ли ступень в том режиме, для которого
// выпуклая постановка вообще верна.
//
// Ограничение не в решателе и не в реализации — оно в самой МОДЕЛИ. G-FOLD
// требует, чтобы всё, кроме тяги, входило в динамику известным слагаемым,
// не зависящим от управления (см. gfold.Setup.Bias). На участке
// высокоскоростного торможения это неверно: сопротивление зависит от того,
// КАК корпус повёрнут к потоку, а поворот — это и есть управление. Площадь
// миделя 64 м², площадь борта 639 м² — разница на порядок, и наклон тяги на
// полсотни градусов меняет сопротивление в разы. План, посчитанный по
// сопротивлению «вдоль оси», расходится с действительностью на десятки
// м/с² — измерено: ступень тормозила втрое сильнее плана, выходила из
// области разрешимости и остаток пути шла без плана вовсе.
//
// Признак применимости — ТЕРМИНАЛЬНАЯ группа камер (три), и это физический
// признак, а не подобранный порог. Именно у неё, и только у неё, границы
// тяги охватывают вес: ρ₁/m < g < ρ₂/m. Это значит, что ступень способна и
// снижаться, и зависать, и тормозить, — то есть множество достижимых
// траекторий имеет ненулевой объём, а не вырождается в узкий коридор. На
// тринадцати камерах ρ₁/m вдвое больше веса: снижаться нельзя, и любая
// ошибка сразу выносит состояние за пределы разрешимости. К этому моменту
// ступень уже прошла плотные слои на сверхзвуке, скоростной напор упал, и
// оба возражения — и по аэродинамике, и по вырожденности — снимаются
// одновременно, потому что причина у них общая: это конец участка
// торможения и начало собственно посадки.
//
// До этого момента посадочным импульсом распоряжается кинематический
// профиль торможения (landingBackupThrust) — тот, что вёл посадку и
// раньше, и который на своём участке работает.
func (b *Booster) gfoldApplicable(nav orbit.NavState) bool {
	n := b.propulsion.Commissioned
	if n <= 0 || len(b.propulsion.Engines) == 0 {
		return false
	}

	// Условие — ФИЗИЧЕСКОЕ, а не номер группы.
	//
	// Выпуклая постановка предполагает, что множество достижимых
	// траекторий имеет ненулевой объём: аппарат должен уметь и снижаться,
	// и висеть, и тормозить. Снижаться он умеет тогда и только тогда,
	// когда минимальная тяга работающей группы меньше веса. Если она
	// больше, снижение невозможно в принципе, любая ошибка немедленно
	// выносит состояние за пределы разрешимости, и задача вырождается.
	//
	// Раньше здесь стояло «группа не больше терминальной» — тот же смысл,
	// но выраженный через номер, а не через физику. Разница не
	// косметическая: номер не знает ни массы, ни высоты, и на практике
	// откладывал передачу управления до двадцати-сорока метров над водой,
	// где вести уже нечего. По условию ниже управление переходит там, где
	// оно становится физически осмысленным, — обычно на переходе 13→5,
	// то есть за сотни метров до касания.
	perEngine := vehicle.ThrustAtAltitude(b.Config.FirstStage,
		physics.Atmosphere(nav.Altitude).Pressure)
	mass := b.dryMass() + b.state.FuelMass
	if perEngine <= 0 || mass <= 0 {
		return false
	}
	minThrust := perEngine * float64(n) * b.minThrottle()
	weight := mass * physics.GravityMagnitudeAtAltitude(nav.Altitude)

	return minThrust < gfoldDescentMargin*weight
}

// gfoldDescentMargin — насколько минимальная тяга группы должна быть ниже
// веса, чтобы снижение считалось возможным не впритык. Девять десятых —
// запас на разброс тяги и оценку массы: ровно на границе «минимальная тяга
// равна весу» аппарат висит, а не снижается, и множество достижимых
// траекторий вырождается в точку.
const gfoldDescentMargin = 0.9

// gfoldPlanUsable решает, годится ли текущий план к исполнению.
//
// План принимается, только если он не хуже того, что даёт резервный закон.
// Резервный гасит снос и садится там, где окажется, — то есть его промах
// равен нынешней горизонтальной ошибке положения. План, который кончается
// ДАЛЬШЕ от площадки, чем ступень находится сейчас, не помогает ничем: это
// не посадка в точку, а полёт в сторону, и исполнять его — значит менять
// скорость касания на промах, которого он всё равно не уменьшает.
//
// Сравнение идёт с текущей ошибкой, а не с константой: одна величина
// сравнивается с другой того же смысла, и порога подбирать не приходится.
//
// Измерено, зачем это нужно: после того как передача управления сдвинулась
// с двадцати метров на семьсот, наведение начало отдавать планы с промахом
// в тридцать два километра и временем полёта в полторы минуты на высоте
// семисот метров. Ступень честно летела по такому плану и приходила к воде
// на семидесяти пяти м/с с наклоном в сто сорок градусов и пустыми баками.
func (b *Booster) gfoldPlanUsable(nav orbit.NavState) bool {
	if b.gfold.traj == nil || b.gfold.traj.Status != gfold.StatusOptimal {
		return false
	}
	// Порог абсолютный — допуск на касание, а не сравнение с текущей
	// ошибкой положения. Сравнение с ней пробовалось и отвергнуто замером:
	// после увода точка прицеливания уезжает, ошибка положения растёт
	// вместе с ней, и порог сам себя ослабляет — план с промахом в
	// четыре с половиной километра проходил и вёл ступень к воде на
	// пятидесяти двух м/с с семью тоннами в баках.
	//
	// Смысл порога прост: наведение по траектории имеет право командовать
	// тогда, когда его траектория действительно приводит в точку. Если не
	// приводит — спуск ведёт резервный закон, который гасит снос и садится
	// мягко там, где окажется. Промах при этом принимается; скорость
	// касания — нет.
	return b.gfold.miss <= gfoldTouchdownTolerance
}

// gfoldRetryInterval — выдержка до следующей попытки решить задачу
// наведения, с, по числу отказов подряд.
//
// При нормальной работе — штатный период пересчёта. После отказов интервал
// удваивается с каждым разом до gfoldMaxRetryInterval: задача, которая не
// решилась сейчас, почти наверняка не решится и через две секунды —
// состояние за это время меняется несопоставимо меньше, чем нужно, чтобы
// неразрешимая постановка стала разрешимой.
//
// Это не только экономия. Отказ стоит дороже удачи: не сошедшийся перебор
// по времени полёта запускает ещё и широкий, вчетверо более дорогой (см.
// gfoldSolve). Без выдержки участок, на котором задача не даётся, съедал
// больше машинного времени, чем весь остальной полёт (измерено: полный
// прогон под -race переставал укладываться в десять минут против прежних
// семи секунд).
func gfoldRetryInterval(failures int) float64 {
	interval := gfoldResolveInterval
	for i := 0; i < failures && interval < gfoldMaxRetryInterval; i++ {
		interval *= 2
	}
	return math.Min(interval, gfoldMaxRetryInterval)
}

// gfoldMaxRetryInterval — предел выдержки между попытками, с. Дальше
// растягивать нельзя: состояние всё-таки меняется, и задача, неразрешимая
// минуту назад, может стать разрешимой.
const gfoldMaxRetryInterval = 30.0
