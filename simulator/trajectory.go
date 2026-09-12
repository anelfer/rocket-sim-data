package simulator

import (
	"math"
	"sort"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Запись траектории прогона.
//
// Зачем отдельная запись, если есть телеметрия. Телеметрия — поток мгновенных
// снимков для живого пульта: она отвечает на вопрос «что сейчас». Вопрос
// «как летели» — другой, и на него нельзя ответить, подписавшись на поток:
// прогон длится минуты, оператор приходит смотреть после, а восстанавливать
// траекторию по накопленным в браузере кадрам значило бы зависеть от того,
// была ли вкладка открыта и не пропустила ли она секунду связи.
//
// Поэтому здесь — отдельный, воспроизводимый прогон: по зерну строится та же
// самая симуляция, прогоняется от старта до конца и записывается её
// СОБСТВЕННОЕ состояние. Никаких приближений, интерполяций «на глаз» и
// дорисованных дуг: каждая точка — это то, где тело действительно было на
// соответствующем такте интегрирования, а каждое событие — тот такт, на
// котором модель действительно сменила фазу.
//
// Координаты записываются в ECEF (связанной с Землёй) системе, а не в ECI.
// Разница здесь принципиальная: за десять минут полёта Земля поворачивается
// на два с половиной градуса, и в инерциальных осях стартовая площадка
// уезжает на полторы сотни километров. Трасса, нарисованная в ECI, показала
// бы возврат бустера «мимо» площадки просто потому, что площадка за это
// время уехала. В связанной системе площадка стоит на месте — и картинка
// отвечает на тот вопрос, который у неё спрашивают.
// -----------------------------------------------------------------------------

// TrajectoryPoint — один отсчёт траектории тела.
type TrajectoryPoint struct {
	Time float64 `json:"t"`

	// X, Y, Z — положение в ECEF, м (см. заголовок файла).
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`

	Latitude  float64 `json:"lat"`
	Longitude float64 `json:"lon"`
	Altitude  float64 `json:"alt"`

	// Speed — модуль скорости ОТНОСИТЕЛЬНО ПОВЕРХНОСТИ, м/с. Именно она
	// имеет смысл при взгляде на трассу: инерциальная скорость на старте
	// равна четырёмстам с лишним метрам в секунду просто от вращения Земли,
	// и «скорость ракеты, стоящей на столе» сбивала бы с толку.
	Speed float64 `json:"v"`

	// Vertical — вертикальная составляющая скорости, м/с.
	Vertical float64 `json:"vz"`

	// Downrange — расстояние по поверхности до точки посадки, м.
	Downrange float64 `json:"dr"`

	Phase string `json:"phase"`
}

// TrajectoryEvent — событие полёта, привязанное к точке траектории.
type TrajectoryEvent struct {
	Time float64 `json:"t"`

	// Body — «ship» или «booster».
	Body string `json:"body"`

	// Kind — устойчивый идентификатор события для раскраски и фильтрации.
	Kind string `json:"kind"`

	// Label — подпись для человека.
	Label string `json:"label"`

	// Index — номер точки в траектории соответствующего тела.
	Index int `json:"index"`

	Altitude  float64 `json:"alt"`
	Speed     float64 `json:"v"`
	Downrange float64 `json:"dr"`
}

// TrajectorySite — точка на поверхности (старт, цель посадки).
type TrajectorySite struct {
	Latitude  float64 `json:"lat"`
	Longitude float64 `json:"lon"`
	Altitude  float64 `json:"alt"`

	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// TrajectoryRun — записанный прогон целиком.
type TrajectoryRun struct {
	Seed    int64  `json:"seed"`
	Profile string `json:"profile"`
	Mission string `json:"mission"`

	// EarthRadius — радиус сферы, на которой построены координаты, м.
	// Отдаётся явно, чтобы отрисовщику не приходилось знать константы
	// модели: он рисует шар именно того радиуса, от которого отсчитаны
	// высоты.
	EarthRadius float64 `json:"earthRadius"`

	// Duration — модельное время конца записи, с.
	Duration float64 `json:"duration"`

	// Truncated сообщает, что запись оборвана по пределу времени, а не
	// потому что полёт закончился. Без этого признака оборванная трасса
	// выглядела бы как «корабль исчез».
	Truncated bool `json:"truncated"`

	Pad     TrajectorySite `json:"pad"`
	Landing TrajectorySite `json:"landing"`

	Ship    []TrajectoryPoint `json:"ship"`
	Booster []TrajectoryPoint `json:"booster"`

	Events []TrajectoryEvent `json:"events"`
}

// TrajectoryOptions — настройки записи.
type TrajectoryOptions struct {
	// Step — шаг интегрирования, с. Ноль означает штатный шаг модели: запись
	// не имеет права считать физику грубее, чем её считает сама модель,
	// иначе показанная траектория перестанет быть той, которая получается
	// в полёте.
	Step float64

	// Sample — период записи отсчётов, с. Влияет только на подробность
	// картинки, но не на физику: между отсчётами модель всё равно считает
	// каждый шаг.
	Sample float64

	// MaxDuration — предел модельного времени записи, с.
	MaxDuration float64
}

// DefaultTrajectoryOptions — настройки, с которыми страница «Траектория»
// запрашивает прогон.
//
// Отсчёт раз в полсекунды: на характерных скоростях выведения (несколько
// километров в секунду) это узел трассы примерно каждые полтора километра —
// заметно подробнее, чем различает глаз на экране, и при этом на десять
// минут полёта выходит порядка тысячи точек на тело, то есть сотни
// килобайт, а не десятки мегабайт.
func DefaultTrajectoryOptions() TrajectoryOptions {
	return TrajectoryOptions{
		Step:        0,
		Sample:      0.5,
		MaxDuration: 1500,
	}
}

// trajectoryRecorder ведёт запись одного тела.
type trajectoryRecorder struct {
	body      string
	points    []TrajectoryPoint
	lastPhase string
	nextAt    float64
	done      bool
}

// RecordTrajectory прогоняет симуляцию с заданным зерном от старта до конца
// полёта и возвращает записанные траектории обоих тел вместе с событиями.
//
// Прогон полностью свой: он не трогает и не читает живую симуляцию пульта.
// Одно и то же зерно всегда даёт одну и ту же запись — на этом держится
// смысл выбора прогона на странице.
func RecordTrajectory(cfg vehicle.Config, seed int64, opt TrajectoryOptions) *TrajectoryRun {
	tc := DefaultTimeConfig()
	if opt.Step > 0 {
		tc.Step = opt.Step
	}
	dt := tc.Step
	if opt.Sample <= 0 {
		opt.Sample = 0.5
	}
	if opt.MaxDuration <= 0 {
		opt.MaxDuration = 1500
	}

	sim := NewSimulationWithSeed(cfg, tc, seed)

	pad := physics.Geodetic{Latitude: cfg.LaunchLatitude, Longitude: cfg.LaunchLongitude}
	// Цель возврата первой ступени — стартовый комплекс (см. Booster.launchTarget).
	// Отдельной «точки посадки» у модели нет, и выдумывать её здесь нельзя.
	landing := pad

	run := &TrajectoryRun{
		Seed:        seed,
		EarthRadius: physics.EarthRadius,
		Pad:         siteAt(pad),
		Landing:     siteAt(landing),
	}

	ship := &trajectoryRecorder{body: "ship"}
	booster := &trajectoryRecorder{body: "booster"}

	// Момент разворота бустера (boostbackBurnStart) — не смена фазы, а
	// отдельно зафиксированный моделью факт, и в события он попадает
	// отдельной проверкой.
	flipSeen := false

	steps := int(opt.MaxDuration/dt) + 1
	for i := 0; i < steps; i++ {
		sim.mu.Lock()
		sim.step(dt)
		t := sim.elapsed

		nav := sim.navState()
		shipPoint := trajectoryPointOf(t, sim.state.Position, sim.state.Velocity,
			nav.Phase.String(), landing)
		ship.observe(t, shipPoint, opt.Sample, run, "Корабль")

		if b := sim.booster; b != nil {
			bp := trajectoryPointOf(t, b.state.Position, b.state.Velocity,
				b.phase.String(), landing)
			first := len(booster.points) == 0
			booster.observe(t, bp, opt.Sample, run, "Бустер")
			if first && len(booster.points) > 0 {
				// Появление бустера в модели И ЕСТЬ отделение: отдельной
				// фазы для него нет, объект просто создаётся в этот такт
				// (см. performStageSeparation).
				run.addEvent(booster, "booster", "separation",
					"Отделение первой ступени", t)
			}

			if !flipSeen && !math.IsNaN(b.boostbackBurnStart) {
				flipSeen = true
				run.addEvent(booster, "booster", "flip",
					"Разворот завершён, начат разворотный импульс", b.boostbackBurnStart)
			}
		}

		shipDone := sim.crashed || sim.phase == orbit.PhaseLanded
		hasBooster := sim.booster != nil
		boosterDone := hasBooster && (sim.booster.phase == BoosterSplashdown ||
			sim.booster.phase == BoosterDestroyed ||
			sim.booster.phase == BoosterCaught)
		sim.mu.Unlock()

		run.Duration = t
		if shipDone && (boosterDone || !hasBooster) {
			break
		}
		if i == steps-1 {
			run.Truncated = true
		}
	}

	// Пустые срезы, а не nil: в JSON nil превращается в null, и странице
	// пришлось бы отличать «бустера не было» от «поле забыли». Пустой
	// список говорит то же самое и не требует особого случая на каждой
	// стороне. Бустера действительно может не быть — у Falcon 9 возврат
	// первой ступени не моделируется.
	run.Ship = ship.points
	if run.Ship == nil {
		run.Ship = []TrajectoryPoint{}
	}
	run.Booster = booster.points
	if run.Booster == nil {
		run.Booster = []TrajectoryPoint{}
	}
	if run.Events == nil {
		run.Events = []TrajectoryEvent{}
	}

	sort.SliceStable(run.Events, func(i, j int) bool {
		return run.Events[i].Time < run.Events[j].Time
	})
	return run
}

// observe записывает отсчёт, если подошёл срок, и отмечает смену фазы.
//
// Смена фазы записывается ВСЕГДА и своим отдельным отсчётом, независимо от
// периода выборки: событие обязано лежать ровно на той точке траектории, где
// оно произошло, иначе метка на карте поедет на полкилометра относительно
// самой траектории и станет враньём.
func (r *trajectoryRecorder) observe(t float64, p TrajectoryPoint, sample float64,
	sink trajectoryEventSink, title string) {

	// Первая точка тела — тоже событие: тело появилось в модели, и его
	// начальная фаза уже что-то значит. Для бустера это и есть начало
	// разворотного импульса (он создаётся сразу в фазе Boostback), и без
	// этой ветки оно потерялось бы вовсе — не как «не нашли», а молча.
	if r.lastPhase == "" {
		r.points = append(r.points, p)
		sink.addEvent(r, r.body, phaseEventKind(p.Phase), title+": "+p.Phase, t)
		r.lastPhase = p.Phase
		r.nextAt = t + sample
		return
	}

	if p.Phase != r.lastPhase {
		r.points = append(r.points, p)
		sink.addEvent(r, r.body, phaseEventKind(p.Phase), title+": "+p.Phase, t)
		r.lastPhase = p.Phase
		r.nextAt = t + sample
		return
	}

	if t+1e-9 >= r.nextAt {
		r.points = append(r.points, p)
		r.nextAt = t + sample
	}
}

// trajectoryEventSink — куда складывать найденные события.
//
// Записей траектории две: воспроизводимая по зерну (собирается сразу в
// TrajectoryRun) и живая (копится по ходу прогона, см. trajectory_live.go).
// Обнаружение смены фазы у них обязано быть общим — иначе одно и то же
// место полёта отмечалось бы на двух трассах по-разному, — а вот место
// хранения разное. Отсюда узкий интерфейс в один метод.
type trajectoryEventSink interface {
	addEvent(r *trajectoryRecorder, body, kind, label string, t float64)
}

// addEvent привязывает событие к последней записанной точке тела.
func (run *TrajectoryRun) addEvent(r *trajectoryRecorder, body, kind, label string, t float64) {
	idx := len(r.points) - 1
	if idx < 0 {
		return
	}
	p := r.points[idx]
	run.Events = append(run.Events, TrajectoryEvent{
		Time:      t,
		Body:      body,
		Kind:      kind,
		Label:     label,
		Index:     idx,
		Altitude:  p.Altitude,
		Speed:     p.Speed,
		Downrange: p.Downrange,
	})
}

// phaseEventKind переводит название фазы в устойчивый идентификатор события.
// Неизвестная фаза получает общий вид «phase» — новая фаза модели попадёт на
// график как обычная смена режима, а не потеряется молча.
func phaseEventKind(phase string) string {
	switch phase {
	// Первая ступень.
	case "Pre-Launch":
		return "prelaunch"
	case "First Stage Burn":
		return "liftoff"
	case "Stage Separation Coast":
		return "separation"

	// Корабль.
	case "Second Stage Burn":
		return "second-stage"
	case "Coast to Apoapsis":
		return "coast"
	case "Circularization Burn":
		return "circularization"
	case "Orbital Flight":
		return "orbit"
	case "Deorbit Burn":
		return "deorbit"
	case "Atmospheric Entry":
		return "entry-burn"
	case "Landing Flip":
		return "landing-flip"
	case "Landed":
		return "touchdown"

	// Бустер. «Landing Burn» общая с кораблём — тела различает поле Body.
	case "Boostback":
		return "boostback-start"
	case "Coast":
		return "boostback-end"
	case "Landing Burn":
		return "landing-burn"
	case "Splashdown":
		return "touchdown"
	case "Destroyed":
		return "destroyed"

	default:
		return "phase"
	}
}

// trajectoryPointOf собирает отсчёт из состояния тела.
func trajectoryPointOf(t float64, positionECI, velocityECI physics.Vec3,
	phase string, target physics.Geodetic) TrajectoryPoint {

	geo := physics.ECIToGeodetic(positionECI, t)
	ecef := physics.ECIToECEF(positionECI, t)

	up := positionECI.Unit()
	ground := velocityECI.Sub(physics.CorotatingVelocity(positionECI))

	return TrajectoryPoint{
		Time:      t,
		X:         ecef.X,
		Y:         ecef.Y,
		Z:         ecef.Z,
		Latitude:  geo.Latitude,
		Longitude: geo.Longitude,
		Altitude:  positionECI.Norm() - physics.EarthRadius,
		Speed:     ground.Norm(),
		Vertical:  ground.Dot(up),
		Downrange: greatCircleDistance(target.Latitude, target.Longitude, geo.Latitude, geo.Longitude),
		Phase:     phase,
	}
}

// siteAt переводит точку на поверхности в тот же ECEF, в котором записана
// траектория.
func siteAt(g physics.Geodetic) TrajectorySite {
	ecef := physics.GeodeticToECEF(g)
	return TrajectorySite{
		Latitude:  g.Latitude,
		Longitude: g.Longitude,
		Altitude:  g.Altitude,
		X:         ecef.X,
		Y:         ecef.Y,
		Z:         ecef.Z,
	}
}
