package simulator

import (
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Запись траектории ИДУЩЕГО прогона.
//
// Отличие от RecordTrajectory (trajectory.go) не в способе, а в том, чей это
// полёт. Там строится отдельная симуляция по зерну — воспроизводимая, но
// чистая: в ней нет и не может быть того, что оператор делал с пультом.
// Здесь записывается тот самый прогон, за которым оператор смотрит: со
// всеми его командами, отказами, ручным управлением и паузами.
//
// Воспроизвести живой прогон по зерну нельзя даже теоретически. Зерно
// задаёт ветер, разбросы и шум приборов — но не то, что человек нажмёт и
// когда. Стоит один раз тронуть тягу или створки, и повтор по зерну
// разойдётся с тем, что было на самом деле. Поэтому запись ведётся прямо
// по ходу полёта, а не восстанавливается после.
//
// Запись НИЧЕГО не считает: она берёт уже посчитанное моделью состояние на
// такте. На физику, наведение и управление она не влияет никак — это
// наблюдатель, а не участник.
// -----------------------------------------------------------------------------

// liveTrajectorySample — период записи отсчёта, с модельного времени.
//
// Та же величина, что и у записи по зерну (DefaultTrajectoryOptions): на
// характерных скоростях выведения это узел трассы примерно каждые полтора
// километра — подробнее, чем различает глаз, и без лишнего веса.
const liveTrajectorySample = 0.5

// liveTrajectoryLimit — предел числа отсчётов на тело.
//
// При периоде в полсекунды это чуть больше трёх часов модельного времени —
// с запасом на любой прогон, который имеет смысл разглядывать. Предел
// нужен не ради памяти как таковой (отсчёт весит около сотни байт), а
// потому что прогон, оставленный на ночь, иначе рос бы без границы.
// Достигнув предела, запись прекращается, и это честно видно по признаку
// Truncated: обрывать начало значило бы потерять старт, ради которого
// трассу и смотрят.
const liveTrajectoryLimit = 24000

// liveTrajectory — состояние записи идущего прогона.
type liveTrajectory struct {
	ship    trajectoryRecorder
	booster trajectoryRecorder
	events  []TrajectoryEvent

	// flipSeen — разворот бустера отмечается один раз (см. observeLive).
	flipSeen bool

	// truncated — запись упёрлась в предел числа отсчётов.
	truncated bool
}

// reset очищает запись. Вызывается при начале нового прогона: старая трасса
// к новому полёту отношения не имеет.
//
// Имена тел проставляются здесь же и обязательно. Они попадают в каждое
// событие и по ним страница решает, к какой из двух трасс его отнести;
// пустое имя означало бы, что событие корабля рисуется на трассе бустера —
// в лучшем случае не там, где было, в худшем за пределами массива.
func (l *liveTrajectory) reset() {
	*l = liveTrajectory{}
	l.ship.body = "ship"
	l.booster.body = "booster"
}

// observeLive записывает состояние обоих тел на текущем такте.
//
// Вызывается из step() под уже удерживаемым s.mu — отдельной блокировки
// не нужно и заводить её нельзя: запись обязана видеть ровно то состояние,
// которое сложилось на этом такте, а не то, что успело измениться, пока
// она ждала свой замок.
func (s *Simulation) observeLive() {
	l := &s.liveTraj
	if l.truncated {
		return
	}
	if l.ship.body == "" {
		// Запись ещё ни разу не инициализирована (нулевое значение поля).
		l.reset()
	}
	if len(l.ship.points) >= liveTrajectoryLimit ||
		len(l.booster.points) >= liveTrajectoryLimit {
		l.truncated = true
		return
	}

	t := s.elapsed
	target := physics.Geodetic{
		Latitude:  s.Config.LaunchLatitude,
		Longitude: s.Config.LaunchLongitude,
	}

	sink := &liveEventSink{live: l}

	shipPoint := trajectoryPointOf(t, s.state.Position, s.state.Velocity,
		s.phase.String(), target)
	l.ship.observe(t, shipPoint, liveTrajectorySample, sink, "Корабль")

	if b := s.booster; b != nil {
		bp := trajectoryPointOf(t, b.state.Position, b.state.Velocity,
			b.phase.String(), target)
		first := len(l.booster.points) == 0
		l.booster.observe(t, bp, liveTrajectorySample, sink, "Бустер")
		if first && len(l.booster.points) > 0 {
			sink.addEvent(&l.booster, "booster", "separation",
				"Отделение первой ступени", t)
		}
		if !l.flipSeen && !math.IsNaN(b.boostbackBurnStart) {
			l.flipSeen = true
			sink.addEvent(&l.booster, "booster", "flip",
				"Разворот завершён, начат разворотный импульс", b.boostbackBurnStart)
		}
	}
}

// LiveTrajectory возвращает снимок траектории идущего прогона.
//
// Отдаётся копия: вызывающая сторона (веб-обработчик) читает её уже без
// замка, а сама запись в это время продолжает расти. Отдать внутренние
// срезы напрямую значило бы позволить читать их во время дописывания.
func (s *Simulation) LiveTrajectory() *TrajectoryRun {
	s.mu.RLock()
	defer s.mu.RUnlock()

	l := &s.liveTraj
	pad := physics.Geodetic{
		Latitude:  s.Config.LaunchLatitude,
		Longitude: s.Config.LaunchLongitude,
	}

	run := &TrajectoryRun{
		Seed:        s.seed,
		EarthRadius: physics.EarthRadius,
		Duration:    s.elapsed,
		Truncated:   l.truncated,
		Pad:         siteAt(pad),
		Landing:     siteAt(pad),
		Ship:        append([]TrajectoryPoint(nil), l.ship.points...),
		Booster:     append([]TrajectoryPoint(nil), l.booster.points...),
		Events:      append([]TrajectoryEvent(nil), l.events...),
	}
	if run.Ship == nil {
		run.Ship = []TrajectoryPoint{}
	}
	if run.Booster == nil {
		run.Booster = []TrajectoryPoint{}
	}
	if run.Events == nil {
		run.Events = []TrajectoryEvent{}
	}
	return run
}

// liveEventSink — приёмник событий живой записи.
//
// Нужен потому, что trajectoryRecorder.observe складывает события в
// TrajectoryRun, а живая запись собирается не в него: TrajectoryRun здесь
// строится заново на каждый запрос снимка, а события копятся всё время.
// Приёмник даёт обеим записям пользоваться одним и тем же кодом обнаружения
// смены фазы — иначе логика «что считать событием» разошлась бы между
// живой трассой и трассой по зерну, и одно и то же место полёта
// отмечалось бы на них по-разному.
type liveEventSink struct{ live *liveTrajectory }

func (s *liveEventSink) addEvent(r *trajectoryRecorder, body, kind, label string, t float64) {
	idx := len(r.points) - 1
	if idx < 0 {
		return
	}
	p := r.points[idx]
	s.live.events = append(s.live.events, TrajectoryEvent{
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
