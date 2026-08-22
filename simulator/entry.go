package simulator

import (
	"fmt"
	"log"
	"math"
	"strings"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Возвращение корабля.
//
// Корабль входит в атмосферу не носом, а брюхом: угол атаки под семьдесят
// градусов, тормозит вся площадь борта, а не мидель. Так снижается баллистический
// коэффициент, торможение начинается выше и идёт мягче, а тепловой поток
// размазывается по большой площади вместо одной точки. Расплата — корпус
// в таком положении неустойчив по всем каналам, и удерживать его приходится
// непрерывно, четырьмя плавниками.
//
// Двигатели на входе выключены, поэтому качать нечего: единственный орган
// управления — плавники, и власть их определяется скоростным напором.
// В разрежённых слоях они бесполезны, и корпус держат двигатели ориентации;
// ниже плавники становятся сильнее любого другого органа.
//
// Перед посадкой корабль разворачивается из горизонтального положения
// в вертикальное и включает двигатели. Разворот делается теми же плавниками:
// задние прижимаются, передние раскрываются, момент поднимает нос.
// -----------------------------------------------------------------------------

// EntryConfig — параметры возвращения.
type EntryConfig struct {
	// TargetPeriapsis — высота перицентра после схода с орбиты, м.
	// Отрицательная: траектория должна пересекать атмосферу.
	TargetPeriapsis float64

	// InterfaceAltitude — высота входа в атмосферу, м.
	InterfaceAltitude float64

	// AngleOfAttack — потребный угол атаки на торможении, градусы.
	AngleOfAttack float64

	// FlipAltitude — высота начала разворота перед посадкой, м.
	FlipAltitude float64

	// LandingBurnAltitude — высота включения посадочных двигателей, м.
	LandingBurnAltitude float64

	// TouchdownSpeed — предельная скорость касания, м/с.
	TouchdownSpeed float64
}

// DefaultEntryConfig возвращает штатный профиль возвращения.
func DefaultEntryConfig() EntryConfig {
	return EntryConfig{
		// Перицентр после схода с орбиты. Не глубоко под поверхностью:
		// такой вход был бы отвесным, и корабль влетал бы в плотные слои
		// на первой космической. Двадцать пять километров дают пологую
		// траекторию, на которой торможение растянуто на минуты.
		TargetPeriapsis:     25000,
		InterfaceAltitude:   120000,
		AngleOfAttack:       70,
		FlipAltitude:        5000,
		LandingBurnAltitude: 3500,
		TouchdownSpeed:      6,
	}
}

// RequestEntry командует сход с орбиты.
//
// Команда доступна только на орбите: сойти с неё, не будучи на ней,
// нельзя, а прерывать выведение ради посадки бессмысленно.
func (s *Simulation) RequestEntry() bool {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.phase != orbit.PhaseOrbital {
		return false
	}
	s.entryRequested = true
	return true
}

// EntryRequested сообщает, запрошен ли сход с орбиты.
func (s *Simulation) EntryRequested() bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.entryRequested
}

// RequestLanding запускает посадочную программу немедленно, из любой фазы
// полёта корабля — в отличие от RequestEntry, замкнутая орбита не нужна.
//
// Нужна тем, кто поднял корабль вручную (например, суборбитальным прыжком
// под собственным управлением) и хочет в любой точке траектории отдать
// тягу и ориентацию бортовому компьютеру возвращения, не дожидаясь, пока
// до этого дойдёт автоматика выведения сама. Посадочная логика реагирует
// только на текущую высоту и скорость снижения — у неё нет прицеливания
// по дальности, — поэтому запускать её можно с любой точки, где корабль
// уже падает или вот-вот начнёт: см. updateEntry/updateLandingFlip/
// updateLandingBurn.
//
// Доступна только кораблю (второй ступени): плавники и посадочные камеры
// ставятся в строй лишь при разделении, и индексы посадочных двигателей
// (landingEngines) считаются в предположении, что s.stage == 2.
func (s *Simulation) RequestLanding() error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.propulsion == nil {
		return fmt.Errorf("симуляция ещё не готова")
	}
	if s.crashed {
		return fmt.Errorf("носитель потерян, посадка невозможна")
	}
	if s.stage != 2 {
		return fmt.Errorf("посадочная программа доступна только кораблю после " +
			"разделения ступеней")
	}
	if s.phase == orbit.PhaseLanded {
		return fmt.Errorf("корабль уже на посадочной площадке")
	}
	if s.inEntry() {
		return fmt.Errorf("посадка уже идёт")
	}

	s.entryRequested = false
	s.shutdownAllEngines()
	s.throttleCmd = 0
	s.phase = orbit.PhaseEntry

	// Радиальная скорость знаковая: команда может застать корабль ещё
	// набирающим высоту (после ручного прыжка), и «скорость снижения»
	// в минусе в этот момент — не ошибка, а честное отражение того,
	// что снижение ещё не началось.
	log.Printf("🛬 Посадка начата по команде оператора на T+%.1f с: высота %.1f км, "+
		"радиальная скорость %.0f м/с (- набор высоты, + снижение)",
		s.elapsed, s.state.Altitude()/1000, -s.state.RadialVelocity())
	return nil
}

// inEntry сообщает, что корабль возвращается и управляется плавниками.
func (s *Simulation) inEntry() bool {
	switch s.phase {
	case orbit.PhaseDeorbit, orbit.PhaseEntry,
		orbit.PhaseLandingFlip, orbit.PhaseLandingBurn:
		return true
	}
	return false
}

// startDeorbit разворачивает корабль против движения.
//
// Двигатели пока не включаются: сначала корпус должен прийти к ретроградной
// ориентации. Импульс, выданный в произвольную сторону, орбиту не понижает,
// а поднимает — так и вело себя изделие, пока зажигание шло сразу.
func (s *Simulation) startDeorbit() {
	s.phase = orbit.PhaseDeorbit
	s.throttleCmd = 0
	s.deorbitSettled = false

	log.Printf("🛬 Разворот на тормозной импульс, T+%.1f с: высота %.1f км",
		s.elapsed, s.state.Altitude()/1000)
}

// updateDeorbit ведёт тормозной импульс до нужного перицентра.
func (s *Simulation) updateDeorbit(nav orbit.NavState) {
	// Пока корпус не пришёл к ретроградной ориентации, двигатели молчат.
	if !s.deorbitSettled {
		err := s.attitude.PointingError(retrogradeAttitude(nav), nav.Frame)
		if err > 5 {
			return
		}

		s.deorbitSettled = true
		s.throttleCmd = 0.4
		s.startAllEngines()
		log.Printf("🔥 Тормозной импульс на T+%.1f с: ошибка наведения %.1f°",
			s.elapsed, err)
		return
	}

	el := physics.ElementsFromState(s.state.Position, s.state.Velocity, s.elapsed)

	// Импульс закончен, когда перицентр опустился в атмосферу.
	if el.PeriapsisAltitude <= s.Entry.TargetPeriapsis || s.state.FuelMass <= 0 {
		s.shutdownAllEngines()
		s.throttleCmd = 0
		s.phase = orbit.PhaseEntry
		s.deployFlaps()

		log.Printf("🪂 Тормозной импульс окончен на T+%.1f с: перицентр %.1f км, "+
			"остаток топлива %.1f т",
			s.elapsed, el.PeriapsisAltitude/1000, s.state.FuelMass/1000)
	}
}

// updateEntry следит за прохождением атмосферы.
func (s *Simulation) updateEntry(nav orbit.NavState) {
	// Плавники выпускаются на подходе к плотным слоям. После тормозного
	// импульса это делается сразу, но при трансатмосферном полёте импульса
	// не было: корабль пришёл на вход прямо с выведения, и раскрывать
	// поверхности надо здесь. Раньше не нужно — выше ста двадцати километров
	// они всё равно ничего не дают.
	set := s.attitude.Surfaces
	if set != nil && !set.Deployed &&
		nav.Altitude <= s.Entry.InterfaceAltitude &&
		s.state.RadialVelocity() < 0 {

		s.deployFlaps()
	}

	// Разворот начинается не на назначенной высоте, а тогда, когда её уже
	// едва хватает на торможение.
	//
	// Высота посадочного импульса — величина не назначаемая, а вычисляемая:
	// она следует из скорости, располагаемой тяги и массы. Сверху к ней
	// добавляется путь, который корабль пролетит, пока разворачивается.
	// Прежде и разворот, и зажигание стояли на постоянных высотах в пять
	// и три с половиной километра — и корабль приходил к земле, не успев
	// ни развернуться, ни затормозить.
	if nav.Altitude <= s.flipAltitude(nav) {
		s.phase = orbit.PhaseLandingFlip

		// На разворот зажигается одна камера.
		//
		// Тяга здесь нужна не для торможения, а для момента: качание камеры
		// даёт то, чего не дают плавники, — они как раз держат корпус плашмя.
		// Три камеры на этом участке жгли бы по тонне компонентов в секунду
		// впустую: разворот занимает десятки секунд, и корабль приходил
		// к площадке с пустыми баками.
		s.startLandingEngines(1)
		s.landingThrottle = landingFlipThrottle

		// Задние плавники складываются первыми.
		//
		// Так и делает настоящий корабль: убрать заднюю пару — значит снять
		// подъёмную силу с хвоста, и нос сам идёт вверх. Передние пока
		// работают: пока корпус лежит плашмя, они единственное, чем можно
		// парировать вращение.
		s.stowFlaps("aft")

		log.Printf("🔄 Разворот перед посадкой на T+%.1f с: высота %.0f м, "+
			"снижение %.0f м/с, зажигание одной камеры, задние плавники сложены",
			s.elapsed, nav.Altitude, -s.state.RadialVelocity())
	}
}

// landingFlipThrottle — уровень тяги на развороте.
//
// Развернуть корпус только плавниками на такой скорости нельзя: нужен момент
// от качания камер, а он есть лишь тогда, когда камеры работают. Тяга при этом
// держится небольшой — задача развернуться, а не подскочить.
const landingFlipThrottle = 0.45

// flipAltitude возвращает высоту начала разворота.
func (s *Simulation) flipAltitude(nav orbit.NavState) float64 {
	descent := -s.state.RadialVelocity()
	if descent <= 0 {
		return s.Entry.FlipAltitude
	}

	// Время на разворот: корпус нужно перевести из полёта брюхом в вертикаль.
	// Стотонная машина разворачивается на сотню градусов не мгновенно, и пока
	// она разворачивается, она продолжает падать.
	const flipDuration = 16.0
	return s.brakingAltitude(nav) + descent*flipDuration
}

// brakingAltitude возвращает высоту, с которой корабль ещё успевает погасить
// снижение располагаемой тягой.
//
// Это классическая посадка «в последний момент»: тормозить раньше невыгодно,
// потому что тяга тратится на зависание, а опоздать нельзя вовсе. Запас берётся
// множителем: считаем, что использовать можно не всю тягу, часть уходит
// на парирование сноса и на ошибки.
func (s *Simulation) brakingAltitude(nav orbit.NavState) float64 {
	descent := -s.state.RadialVelocity()
	if descent <= 0 {
		return s.Entry.LandingBurnAltitude
	}

	mass := s.dryMass + s.state.FuelMass
	thrust := s.landingThrust(nav)
	if mass <= 0 || thrust <= 0 {
		return s.Entry.LandingBurnAltitude
	}

	const (
		usable = 0.8 // доля тяги, отдаваемая на торможение
		margin = 1.5 // запас на неточность наведения и доворот
	)
	g := physics.GravityMagnitudeAtAltitude(nav.Altitude)

	decel := usable*thrust/mass - g
	if decel <= 1 {
		// Тяги не хватает даже на зависание: тормозить надо было раньше,
		// и высота ничего уже не решает.
		return s.Entry.LandingBurnAltitude
	}
	return math.Max(margin*descent*descent/(2*decel), 120)
}

// landingThrust возвращает располагаемую тягу посадочных камер на этой высоте.
func (s *Simulation) landingThrust(nav orbit.NavState) float64 {
	stage := s.currentStage()
	nominal := vehicle.ThrustAtAltitude(stage, physics.Atmosphere(nav.Altitude).Pressure)
	return nominal * float64(s.landingEngineCount())
}

// updateLandingFlip доворачивает корпус в вертикаль и переходит к торможению.
func (s *Simulation) updateLandingFlip(nav orbit.NavState) {
	s.landingThrottle = landingFlipThrottle

	// К торможению переходим по высоте, а не по завершению разворота: ждать
	// идеальной вертикали, падая со скоростью в сотни метров в секунду,
	// означает не затормозить вовсе.
	//
	// Но и держаться за высоту до последнего нельзя. Как только корпус сходит
	// с положения плашмя, плавники теряют власть — они работают площадью борта,
	// а борт уходит из потока, — зато корпус на промежуточных углах даёт
	// огромный опрокидывающий момент. Одной камеры против него не хватает,
	// и разворот, дойдя до середины, откатывался обратно. Настоящий корабль
	// этого разрыва не знает: у него разворот и есть посадочный импульс.
	// Поэтому камеры зажигаются, едва корпус повернулся достаточно, чтобы
	// их тяга шла вниз, а не вбок.
	const burnPointing = 50.0 // градусы до вертикали
	pointing := s.attitude.PointingError(retrogradeGroundAttitude(nav), nav.Frame)

	if nav.Altitude <= s.brakingAltitude(nav) || pointing <= burnPointing {
		s.phase = orbit.PhaseLandingBurn

		// Теперь нужна тяга, а не момент: зажигаются все посадочные камеры.
		s.startLandingEngines(s.landingEngineCount())

		// И складываются передние плавники: корпус уже держат камеры,
		// а раскрытые плавники на вертикали только ловят ветер.
		s.stowFlaps("fwd")

		// Высота вынудила зажечь камеры раньше, чем корпус успел довернуться
		// хоть сколько-то разумно. Это не штатный сценарий, и лог обязан
		// сказать об этом прямо, а не подать зажигание под тем же заголовком,
		// что и нормальный посадочный импульс: если тяга ушла в сторону
		// от вертикали на девяносто градусов и больше, торможения она
		// не даст, куда бы ни была направлена, и последствие — жёсткое
		// касание, которое сам компьютер должен был бы отличить от штатной
		// посадки заранее, а не только по факту разбора после падения.
		const emergencyPointing = 90.0
		if pointing >= emergencyPointing {
			log.Printf("🚨 Аварийное зажигание на T+%.1f с: высота %.0f м вынудила "+
				"начать посадочный импульс при ошибке наведения %.0f° — тяга "+
				"направлена не туда, торможения не будет", s.elapsed, nav.Altitude, pointing)
		}

		log.Printf("🔥 Посадочный импульс на T+%.1f с: высота %.0f м, "+
			"вертикальная скорость %.0f м/с, ошибка наведения %.0f°, камер %d",
			s.elapsed, nav.Altitude, -s.state.RadialVelocity(),
			pointing, s.runningLandingEngines())
	}
}

// updateLandingBurn ведёт торможение до касания.
func (s *Simulation) updateLandingBurn(nav orbit.NavState) {
	descent := -s.state.RadialVelocity()

	if nav.Altitude <= 0 {
		s.finishLanding(descent)
		return
	}

	mass := s.dryMass + s.state.FuelMass
	g := physics.GravityMagnitudeAtAltitude(nav.Altitude)
	perEngine := s.engineThrustAtAltitude(nav)

	if perEngine <= 0 || mass <= 0 {
		return
	}

	// Потребное замедление.
	//
	// Выше площадки торможения корабль гасит снижение по остатку высоты:
	// приходить к ней он обязан не с нулевой скоростью, а с восемью метрами
	// в секунду — на ноль тормозить нельзя, иначе корабль зависнет и сожжёт
	// остаток топлива, вися над площадкой. Ниже держится расписание скорости:
	// чем ближе земля, тем медленнее снижение, и у самых опор оно сходит
	// к скорости касания.
	// Расписание подобрано под то, чем корабль тормозит в конце: под две
	// камеры на нижнем пределе дросселирования. Они дают около двух g, то есть
	// зависнуть на них нельзя — можно только гасить снижение. Значит, подходить
	// к площадке надо быстро и тормозить до последнего: приди корабль к сорока
	// метрам на восьми метрах в секунду, тяги минимального режима хватило бы,
	// чтобы отбросить его обратно вверх.
	const (
		flare      = 40.0 // высота перехода к расписанию, м
		flareSpeed = 22.0 // скорость на этой высоте, м/с
		touchdown  = 2.0  // скорость у самой земли, м/с
		gain       = 1.6  // жёсткость удержания расписания, 1/с
	)

	var need float64
	if nav.Altitude > flare {
		height := nav.Altitude - flare
		need = g + (descent*descent-flareSpeed*flareSpeed)/(2*math.Max(height, 1))
	} else {
		target := touchdown + (flareSpeed-touchdown)*nav.Altitude/flare
		need = g + gain*(descent-target)
	}

	required := math.Max(need*mass, 0)

	// Сколько камер оставить.
	//
	// Три камеры на нижнем пределе дросселирования дают почти три g — таким
	// корабль не сядет, он подпрыгнет. Поэтому по мере торможения лишняя
	// камера гасится: тормозит корабль на трёх, касается на двух — так же,
	// как это делает настоящий аппарат.
	//
	// Меньше двух не оставляем, и дело не в тяге, а в управлении: на одной
	// камере момент по крену создавать нечем, а у самой земли корпус надо
	// держать.
	const minEngines = 2

	floor := s.currentStage().MinThrottle
	if floor <= 0 {
		floor = 0.4
	}

	want := s.runningLandingEngines()
	for want > minEngines && float64(want)*perEngine*floor > required {
		want--
	}
	if want < s.runningLandingEngines() {
		s.setLandingEngineCount(want)
	}

	available := float64(s.runningLandingEngines()) * perEngine
	if available <= 0 {
		return
	}
	s.landingThrottle = physics.Clamp(required/available, 0, 1)
}

// engineThrustAtAltitude возвращает тягу одной камеры на этой высоте.
func (s *Simulation) engineThrustAtAltitude(nav orbit.NavState) float64 {
	return vehicle.ThrustAtAltitude(s.currentStage(),
		physics.Atmosphere(nav.Altitude).Pressure)
}

// runningLandingEngines возвращает число работающих посадочных камер.
func (s *Simulation) runningLandingEngines() int {
	n := 0
	for _, i := range s.landingEngines() {
		if s.engines[i].Running {
			n++
		}
	}
	return n
}

// setLandingEngineCount гасит лишние посадочные камеры.
//
// Гасятся именно лишние: заново их уже не зажигают. Повторный запуск камеры
// в полёте — отдельная операция со своими условиями, и притворяться, что
// она бесплатна, не стоит.
func (s *Simulation) setLandingEngineCount(n int) {
	idx := s.landingEngines()
	if n < 2 {
		n = 2
	}
	if n > len(idx) {
		n = len(idx)
	}

	kept := 0
	live := make(map[string]bool, n)
	for _, i := range idx {
		if !s.engines[i].Running {
			continue
		}
		if kept < n {
			kept++
			live[s.engines[i].ID] = true
			continue
		}
		s.engines[i].Running = false
		s.engines[i].Thrust = 0
	}

	if s.propulsion == nil {
		return
	}
	for _, e := range s.propulsion.Engines {
		if isVacuumEngine(e.ID) {
			continue
		}
		if !live[e.ID] {
			e.Stop()
		}
	}
	s.propulsion.Commissioned = kept

	log.Printf("🔻 Посадка на %d камерах на T+%.1f с, высота %.0f м",
		kept, s.elapsed, s.state.Altitude())
}

// finishLanding подводит итог посадки.
//
// Приговор выносится по двум величинам: скорости касания и наклону корпуса.
// Одной скорости мало — корабль, коснувшийся площадки мягко, но с креном
// в двадцать градусов, на опорах не устоит: центр тяжести уйдёт за пятно
// опоры, и стотонная машина завалится набок под собственным весом.
func (s *Simulation) finishLanding(descent float64) {
	s.shutdownAllEngines()
	s.throttleCmd = 0
	s.phase = orbit.PhaseLanded

	nav := s.navState()
	upright := physics.Attitude{Pitch: 90, Yaw: s.attitude.AttitudeIn(nav.Frame).Yaw}
	s.landingTilt = s.attitude.PointingError(upright, nav.Frame)
	s.landingSpeed = descent

	// Предел по наклону: опоры корабля разнесены на радиус корпуса, и
	// вертикаль он держит с запасом в полтора десятка градусов, не больше.
	const tiltLimit = 15.0

	switch {
	case descent > s.Entry.TouchdownSpeed:
		s.crashed = true
		log.Printf("💥 Жёсткое касание на T+%.1f с: вертикальная скорость %.1f м/с "+
			"при пределе %.1f", s.elapsed, descent, s.Entry.TouchdownSpeed)

	case s.landingTilt > tiltLimit:
		s.crashed = true
		s.toppled = true
		log.Printf("💥 Корабль завалился на T+%.1f с: касание мягкое (%.1f м/с), "+
			"но корпус отклонён от вертикали на %.0f° при пределе %.0f°",
			s.elapsed, descent, s.landingTilt, tiltLimit)

	default:
		log.Printf("🏁 Посадка на T+%.1f с: вертикальная скорость %.1f м/с, "+
			"отклонение от вертикали %.1f°, остаток топлива %.1f т",
			s.elapsed, descent, s.landingTilt, s.state.FuelMass/1000)
	}
}

// LandingTelemetry — итог касания для интерфейса.
type LandingTelemetry struct {
	// Speed — вертикальная скорость касания, м/с.
	Speed float64 `json:"speed"`

	// Limit — предел, который выдерживает конструкция, м/с.
	Limit float64 `json:"limit"`

	// Tilt — отклонение корпуса от вертикали, градусы.
	// После опрокидывания растёт до горизонтали.
	Tilt float64 `json:"tilt"`

	// Toppled сообщает, что корабль завалился набок.
	Toppled bool `json:"toppled"`

	// Intact сообщает, что корабль сел и остался цел.
	Intact bool `json:"intact"`

	// FuelLeft — остаток компонентов, кг.
	FuelLeft float64 `json:"fuelLeft"`
}

// buildLandingTelemetry собирает итог посадки.
// Вызывается под удержанным mu.
func (s *Simulation) buildLandingTelemetry() *LandingTelemetry {
	if s.phase != orbit.PhaseLanded {
		return nil
	}
	return &LandingTelemetry{
		Speed:    s.landingSpeed,
		Limit:    s.Entry.TouchdownSpeed,
		Tilt:     s.landingTilt,
		Toppled:  s.toppled,
		Intact:   !s.crashed,
		FuelLeft: s.state.FuelMass,
	}
}

// holdLandedAttitude держит корпус на площадке.
//
// Устоявший корабль остаётся стоять: угловые скорости гасятся опорами, углы
// не меняются. Опрокинувшийся заваливается набок — не мгновенно, а за пару
// секунд, как и падает стотонная конструкция: момент даёт вес, приложенный
// за пятном опоры.
// Вызывается под удержанным mu.
func (s *Simulation) holdLandedAttitude(dt float64, nav orbit.NavState) {
	a := &s.attitude
	a.ControlTorque = physics.Vec3{}
	a.SurfaceTorque = physics.Vec3{}
	a.AeroTorque = physics.Vec3{}
	a.UsingRCS = false

	if !s.toppled {
		a.Omega = physics.Vec3{}
		return
	}

	// Падение набок: поворот вокруг горизонтальной оси до касания борта.
	const (
		rate  = 0.55 // угловая скорость падения, рад/с
		limit = 90.0 // до горизонтали
	)

	if s.landingTilt >= limit {
		a.Omega = physics.Vec3{}
		return
	}

	s.landingTilt += rate * physics.RadToDeg * dt

	// Ось падения — не связанная ось корпуса, а горизонтальная ось,
	// перпендикулярная плоскости, в которой корабль уже накренён. Раньше
	// здесь стояла фиксированная связанная ось Y (тангаж): корабль заваливался
	// вокруг неё независимо от того, в какую сторону света он на самом деле
	// наклонился при касании, — крен по крену давал то же падение, что и крен
	// по рысканию, и на сцене корпус мог завалиться в произвольном направлении,
	// не совпадающем с фактическим наклоном.
	//
	// Плоскость наклона задаётся местной вертикалью и продольной осью корпуса:
	// именно в ней лежит опрокидывающий момент от смещённого центра тяжести.
	// Ось находится как их векторное произведение и остаётся горизонтальной
	// на всём падении, поэтому вычисляется заново на каждом шаге, а не хранится:
	// это не оптимизация, а гарантия того, что при мелкой численной ошибке
	// падение не уйдёт из исходной плоскости.
	up := s.state.Position.Unit()
	forward := a.Orientation.Rotate(physics.Vec3{X: 1})
	lean := forward.Sub(up.Scale(forward.Dot(up)))

	var omega physics.Vec3
	if n := lean.Norm(); n > 1e-6 {
		hinge := up.Cross(lean.Scale(1 / n))
		// Omega нужна в связанных осях: Derivative кватерниона берёт угловую
		// скорость именно в них, а ось падения найдена в инерциальных.
		omega = a.Orientation.RotateInverse(hinge.Scale(rate))
	}
	a.Omega = omega

	q := a.Orientation.Add(a.Orientation.Derivative(a.Omega).Scale(dt))
	a.Orientation = q.Normalized()
	_ = nav
}

// deployFlaps выпускает плавники в поток.
func (s *Simulation) deployFlaps() {
	if s.attitude.Surfaces != nil {
		s.attitude.Surfaces.Deployed = true
	}
}

// stowFlaps прижимает к борту пару плавников: "aft" — заднюю, "fwd" — переднюю.
//
// Сложенные плавники выводятся из-под автопилота: дальше ими не управляют,
// они просто убраны. Именно так проходит посадка у настоящего корабля —
// сначала складывается задняя пара, следом передняя.
func (s *Simulation) stowFlaps(group string) {
	set := s.attitude.Surfaces
	if set == nil {
		return
	}
	for i, surface := range set.Surfaces {
		if strings.HasPrefix(surface.Name, group) {
			set.SetManual(i, 0)
		}
	}
}

// inLanding сообщает, что тягой распоряжается посадочный контур, а не наведение
// выведения.
func (s *Simulation) inLanding() bool {
	return s.phase == orbit.PhaseLandingFlip || s.phase == orbit.PhaseLandingBurn
}

// landingEngines возвращает номера камер, которыми садится корабль.
//
// Садится он не всеми: у корабля шесть камер, но три из них — вакуумные,
// с насадком под давление в тысячные доли атмосферы. У земли такой насадок
// работает на срыве потока, и включать его нельзя. Поэтому посадочными
// остаются камеры с обычным соплом, и их тяги для стотонного корабля
// более чем достаточно.
func (s *Simulation) landingEngines() []int {
	// Камеры с обычным соплом идут в списке первыми — в том же порядке, в каком
	// они собираются в двигательной установке.
	count := vehicle.SecondStagePrimaryCount(s.Config)
	if count <= 0 || count > len(s.engines) {
		count = len(s.engines)
	}

	out := make([]int, 0, count)
	for i := 0; i < count; i++ {
		out = append(out, i)
	}
	return out
}

// landingEngineCount возвращает число посадочных камер.
func (s *Simulation) landingEngineCount() int { return len(s.landingEngines()) }

// isVacuumEngine распознаёт вакуумную камеру по идентификатору вида "S2-V1".
func isVacuumEngine(id string) bool {
	i := strings.LastIndex(id, "-")
	return i >= 0 && i+1 < len(id) && id[i+1] == 'V'
}

// startLandingEngines зажигает заданное число посадочных камер и гасит
// вакуумные.
func (s *Simulation) startLandingEngines(count int) {
	idx := s.landingEngines()
	if count < 1 {
		count = 1
	}
	if count > len(idx) {
		count = len(idx)
	}

	live := make(map[int]bool, count)
	for _, i := range idx[:count] {
		live[i] = true
	}
	for i := range s.engines {
		s.engines[i].Running = live[i]
	}

	if s.propulsion != nil {
		// В двигательной установке вакуумные камеры отличаются суффиксом
		// в идентификаторе: "S2-V1" против "S2-1". Порядок обычных камер
		// тот же, что и в списке носителя, поэтому зажигаем первые count.
		lit := 0
		for _, e := range s.propulsion.Engines {
			if isVacuumEngine(e.ID) {
				e.Stop()
				continue
			}
			if lit < count {
				e.Start()
				lit++
			} else {
				e.Stop()
			}
		}
		// Погашенные камеры — не отказ, а план полёта.
		s.propulsion.Commissioned = count

		// Питание с расходных баков: иначе на развороте, когда корабль ещё
		// лежит боком, компоненты прижаты к борту и насосы срывает.
		s.propulsion.FuelTank.HeaderFeed = true
		s.propulsion.OxTank.HeaderFeed = true
	}
}

// entryTarget возвращает потребную ориентацию на возвращении.
//
// На торможении корпус ставится под угол атаки к набегающему потоку: нос
// приподнят относительно вектора скорости ровно настолько, чтобы угол между
// ними составлял заданную величину. Перед посадкой корабль разворачивается
// в вертикаль.
func (s *Simulation) entryTarget(nav orbit.NavState) physics.Attitude {
	switch s.phase {
	case orbit.PhaseDeorbit:
		// Тормозной импульс: соплами вперёд, против движения.
		return retrogradeAttitude(nav)

	case orbit.PhaseLandingFlip, orbit.PhaseLandingBurn:
		// Тяга направляется против движения относительно воздуха: так гасится
		// и снижение, и снос.
		//
		// Именно относительно поверхности, а не инерциально и не относительно
		// воздуха. Инерциальная скорость у земли на две трети состоит
		// из вращения Земли, а воздушная у самой площадки перекошена ветром.
		if nav.Altitude < 200 {
			// У самой земли корпус доворачивается в вертикаль: садиться боком
			// нельзя, а сноса к этому моменту почти не остаётся.
			_, azimuth := nav.Frame.PitchAzimuth(nav.GroundRelativeVelocity)
			return physics.Attitude{Pitch: 90, Yaw: azimuth, Roll: 0}
		}
		return retrogradeGroundAttitude(nav)
	}

	// Торможение брюхом. Угол атаки — это угол между продольной осью
	// и вектором скорости, поэтому потребный угол места оси равен наклону
	// траектории плюс потребный угол атаки: корпус задирает нос над
	// направлением падения и подставляет потоку борт.
	path := flightPathAngle(nav)
	pitch := path + s.entryAngleOfAttack(nav)

	_, azimuth := nav.Frame.PitchAzimuth(nav.AirRelativeVelocity)
	return physics.Attitude{
		Pitch: physics.Clamp(pitch, -90, 90),
		Yaw:   azimuth,
		Roll:  0,
	}
}

// entryAngleOfAttack возвращает потребный угол атаки на снижении.
//
// Он не постоянен, и это не тонкость, а суть манёвра. На гиперзвуке траектория
// пологая: поток идёт почти горизонтально, и чтобы подставить ему брюхо,
// корпусу достаточно семидесяти градусов к вектору скорости — нос при этом
// заметно выше хвоста. К концу спуска траектория становится отвесной, корабль
// падает вертикально вниз, и подставить брюхо тому же потоку можно только
// одним положением: поперёк, то есть под прямым углом к скорости. Корпус
// ложится горизонтально — так и выглядит настоящее торможение брюхом.
//
// Раньше угол держался постоянным. На отвесном участке это давало корпус,
// наклонённый носом вниз на двадцать градусов: разворачиваться перед посадкой
// пришлось бы на сто десять градусов вместо девяноста, и выглядело это так,
// будто корабль падает носом.
func (s *Simulation) entryAngleOfAttack(nav orbit.NavState) float64 {
	const (
		terminal = 90.0 // поперёк потоку на дозвуке
		fast     = 3.0  // число Маха, выше которого держится гиперзвуковой угол
		slow     = 1.0  // ниже — уже установившееся падение
	)

	hypersonic := s.Entry.AngleOfAttack

	speed := nav.AirRelativeVelocity.Norm()
	sound := physics.Atmosphere(nav.Altitude).SoundSpeed
	if sound <= 0 || speed <= 0 {
		return hypersonic
	}

	mach := speed / sound
	switch {
	case mach >= fast:
		return hypersonic
	case mach <= slow:
		return terminal
	}

	k := (fast - mach) / (fast - slow)
	return hypersonic + (terminal-hypersonic)*k
}

// retrogradeAttitude возвращает ориентацию против вектора скорости.
//
// Тормозной импульс выдаётся соплами вперёд, поэтому продольная ось смотрит
// назад по движению: угол места меняет знак, азимут разворачивается на сто
// восемьдесят градусов.
func retrogradeAttitude(nav orbit.NavState) physics.Attitude {
	pitch, azimuth := nav.Frame.PitchAzimuth(nav.Velocity)
	return physics.Attitude{Pitch: -pitch, Yaw: math.Mod(azimuth+180, 360), Roll: 0}
}

// retrogradeGroundAttitude возвращает ориентацию против движения относительно
// поверхности.
//
// Тормозить надо именно эту скорость. Инерциальная не годится: в ней сидит
// вращение Земли — четыреста метров в секунду на восток, и «против движения»
// у земли означало бы нос почти горизонтально на запад.
//
// Скорость относительно воздуха не годится тоже, и это тоньше. На высоте
// разница невелика, но у самой площадки корабль снижается со скоростью
// в десятки метров в секунду, а ветер дует со скоростью того же порядка.
// Вектор воздушной скорости оказывается наклонён на десятки градусов
// от вертикали, и корабль послушно разворачивался вслед за ветром: к сотне
// метров он подходил, отклонившись от вертикали на полсотни градусов,
// доворачивать было уже некогда, и тяга уходила вбок вместо торможения.
func retrogradeGroundAttitude(nav orbit.NavState) physics.Attitude {
	v := nav.GroundRelativeVelocity
	if v.Norm() < 1 {
		return physics.Attitude{Pitch: 90, Yaw: 0, Roll: 0}
	}

	pitch, azimuth := nav.Frame.PitchAzimuth(v)
	return physics.Attitude{Pitch: -pitch, Yaw: math.Mod(azimuth+180, 360), Roll: 0}
}

// flightPathAngle возвращает наклон траектории к горизонту, градусы.
// На снижении он отрицателен.
func flightPathAngle(nav orbit.NavState) float64 {
	if nav.AirRelativeVelocity.Norm() < 1 {
		return 0
	}
	pitch, _ := nav.Frame.PitchAzimuth(nav.AirRelativeVelocity)
	return pitch
}

// sideArea возвращает площадь борта активной ступени, м².
//
// Корпус — цилиндр, поэтому борт равен произведению длины на диаметр.
// У корабля это без малого пятьсот квадратных метров против шестидесяти
// у миделя: именно ради этой разницы вход и делают брюхом.
func (s *Simulation) sideArea() float64 {
	cfg := s.Config
	length := cfg.FirstStageLength
	if s.stage > 1 {
		length = secondStageLength(cfg)
	}
	return length * cfg.Diameter
}

// bodyAngleOfAttack возвращает угол между продольной осью и потоком, градусы.
func (s *Simulation) bodyAngleOfAttack(airRelative physics.Vec3) float64 {
	forward := s.attitude.Orientation.Rotate(physics.Vec3{X: 1})
	return physics.AngleOfAttack(forward, airRelative)
}

// secondStageLength возвращает длину верхней ступени, м.
//
// Раньше она принималась за треть первой. Для среднего носителя это близко
// к правде, для сверхтяжёлого — вдвое меньше действительного: корабль длиной
// пятьдесят два метра считался двадцатичетырёхметровым, и площадь борта,
// на которой держится всё торможение при входе, выходила вдвое меньше.
func secondStageLength(cfg vehicle.Config) float64 {
	if cfg.TotalLength > cfg.FirstStageLength && cfg.FirstStageLength > 0 {
		return cfg.TotalLength - cfg.FirstStageLength
	}
	return cfg.FirstStageLength / 3
}

// SetFlap задаёт ручной угол плавника, градусы.
//
// Поверхность, взятая в ручное управление, автопилоту больше не подчиняется:
// он продолжает считать потребный момент, но эту панель не двигает. Так
// устроен и настоящий пульт — переключатель «автомат — ручное» стоит
// на каждом приводе отдельно.
func (s *Simulation) SetFlap(name string, degrees float64) bool {
	s.mu.Lock()
	defer s.mu.Unlock()

	set := s.attitude.Surfaces
	i := set.IndexOf(name)
	if i < 0 {
		return false
	}

	set.SetManual(i, degrees*physics.DegToRad)
	return true
}

// ReleaseFlap возвращает плавник автопилоту.
func (s *Simulation) ReleaseFlap(name string) bool {
	s.mu.Lock()
	defer s.mu.Unlock()

	set := s.attitude.Surfaces
	i := set.IndexOf(name)
	if i < 0 {
		return false
	}

	set.ReleaseManual(i)
	return true
}

// DeployFlaps выпускает или убирает поверхности по команде оператора.
func (s *Simulation) DeployFlaps(deployed bool) bool {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.attitude.Surfaces == nil {
		return false
	}
	s.attitude.Surfaces.Deployed = deployed
	return true
}
