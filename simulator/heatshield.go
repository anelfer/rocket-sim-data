package simulator

import (
	"log"
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Теплозащита корабля.
//
// У корабля две стороны, и они принципиально разные. Наветренная закрыта
// керамическими плитками и держит температуру, при которой сталь давно бы
// потекла. Подветренная — голая нержавеющая сталь: там расчёт на то, что
// в тени за корпусом поток разрежен и греет на порядок слабее.
//
// Отсюда и весь смысл полёта брюхом: корабль обязан идти плитками вперёд.
// Развернись он на сто восемьдесят градусов по крену — и тот же поток
// придётся на сталь, которая столько не держит. Это не деталь оформления,
// а условие выживания аппарата, и модель обязана его знать: иначе ориентация
// корпуса на входе не значит ничего, кроме картинки на экране.
//
// Считается это просто. Тепловой поток в критической точке даёт модель
// нагрева; на какую сторону он придётся — определяет угол между брюхом
// и набегающим потоком. Затенённая сторона получает малую долю: за корпусом
// поток отрывается, и прямого обдува там нет.
// -----------------------------------------------------------------------------

// HeatShieldSide — одна сторона корпуса.
type HeatShieldSide struct {
	Name string

	// Temperature — температура поверхности, К.
	Temperature float64

	// Limit — предельная температура материала, К.
	Limit float64

	// HeatCapacity — теплоёмкость стороны, Дж/К.
	HeatCapacity float64

	// Area — площадь стороны, м².
	Area float64

	// Emissivity — степень черноты: чем выше, тем больше сторона
	// переизлучает обратно.
	Emissivity float64

	// Damage — накопленное повреждение, 0…1.
	Damage float64

	// HeatFlux — поток, приходящийся на эту сторону, Вт/м².
	HeatFlux float64
}

// HeatShield — теплозащита корпуса из двух сторон.
type HeatShield struct {
	Windward HeatShieldSide // сторона с плитками
	Leeward  HeatShieldSide // голая сталь

	// Exposure — косинус угла между брюхом и потоком.
	// Единица означает, что плитки смотрят точно в поток, минус единица —
	// что корабль идёт спиной вперёд.
	Exposure float64

	// BurnedThrough сообщает, что корпус прогорел.
	BurnedThrough bool
}

// NewHeatShield собирает теплозащиту корабля.
//
// Теплоёмкость считается по материалу, а не назначается на глаз, и это важнее,
// чем кажется. Греется не «теплозащита» целиком, а её поверхностный слой:
// плитка толщиной в два сантиметра и лёгкая, как пенопласт, выходит на
// равновесие с потоком за десятки секунд — именно поэтому на возвращении она
// светится. С теплоёмкостью всей конструкции корабль проходил бы вход,
// нагревшись на полторы сотни градусов, и вопрос «зачем ему плитки» оставался
// бы без ответа.
func NewHeatShield(length, diameter, ambient float64) *HeatShield {
	side := length * diameter / 2

	// Керамическая плитка: два сантиметра, плотность как у лёгкого пенопласта.
	const (
		tileThickness = 0.02   // м
		tileDensity   = 300.0  // кг/м³
		tileHeat      = 1000.0 // Дж/(кг·К)
	)

	// Нержавеющая сталь обшивки: четыре миллиметра.
	const (
		steelThickness = 0.004  // м
		steelDensity   = 7900.0 // кг/м³
		steelHeat      = 500.0  // Дж/(кг·К)
	)

	return &HeatShield{
		Windward: HeatShieldSide{
			Name: "tiles", Temperature: ambient, Limit: 1700,
			HeatCapacity: side * tileThickness * tileDensity * tileHeat,
			Area:         side, Emissivity: 0.85,
		},
		Leeward: HeatShieldSide{
			// Нержавеющая сталь окисляется в первые же секунды входа
			// и излучает заметно лучше полированной.
			Name: "steel", Temperature: ambient, Limit: 1100,
			HeatCapacity: side * steelThickness * steelDensity * steelHeat,
			Area:         side, Emissivity: 0.45,
		},
	}
}

// Update продвигает нагрев сторон на шаг.
//
// belly — единичный вектор от корпуса в сторону брюха в инерциальных осях,
// airRelative — скорость относительно воздуха. Их скалярное произведение
// и определяет, какая сторона подставлена потоку.
func (h *HeatShield) Update(dt float64, belly, airRelative physics.Vec3,
	stagnationFlux float64) {

	if h == nil {
		return
	}

	speed := airRelative.Norm()
	if speed > 1 {
		// airRelative — скорость корпуса относительно воздуха, поэтому поток
		// набегает с противоположной стороны и приходится на ту сторону,
		// нормаль которой смотрит по вектору скорости. Брюхо подставлено,
		// когда оно смотрит туда же, куда корабль летит: на торможении
		// брюхом это и есть полёт «животом вперёд».
		h.Exposure = belly.Unit().Dot(airRelative.Scale(1 / speed))
	}

	// Затенённая сторона получает малую долю: за корпусом поток отрывается,
	// и прямого обдува там нет. Доля не ноль — донные вихри всё же греют.
	const shadow = 0.05

	// Поток в критической точке — это наибольшее, что вообще приходится на
	// аппарат, и приходится оно на пятачок у носа. Вдоль борта пограничный
	// слой уже нарос, скачок отошёл, и удельный поток падает в разы. Мерить
	// весь борт по критической точке — значит сжечь корабль на бумаге:
	// при миллионе ватт на квадрат равновесная температура превышает две
	// тысячи градусов, а плитки на возвращении светятся примерно на полутора.
	const flank = 0.30

	incident := stagnationFlux * flank

	front := math.Max(0, h.Exposure)
	back := math.Max(0, -h.Exposure)

	h.Windward.HeatFlux = incident * (front + shadow*back)
	h.Leeward.HeatFlux = incident * (back + shadow*front)

	h.Windward.advance(dt)
	h.Leeward.advance(dt)

	if h.Windward.Damage >= 1 || h.Leeward.Damage >= 1 {
		h.BurnedThrough = true
	}
}

// advance интегрирует температуру стороны и копит повреждение.
func (s *HeatShieldSide) advance(dt float64) {
	if s.HeatCapacity <= 0 {
		return
	}

	// Приход тепла против переизлучения. Именно излучение и уносит основную
	// часть тепла на входе: при полутора тысячах градусов квадратный метр
	// сбрасывает сотни киловатт.
	radiated := s.Emissivity * physics.StefanBoltzmann *
		math.Pow(math.Max(s.Temperature, 0), 4)

	net := (s.HeatFlux - radiated) * s.Area
	s.Temperature += net * dt / s.HeatCapacity

	if s.Temperature < 3 {
		s.Temperature = 3
	}

	// Перегрев копится так же, как в стенке камеры: квадратично по превышению.
	if excess := s.Temperature - s.Limit; excess > 0 {
		const scale = 250.0
		s.Damage += (excess / scale) * (excess / scale) / 10 * dt
		if s.Damage > 1 {
			s.Damage = 1
		}
	}
}

// updateHeatShield продвигает теплозащиту корабля.
// Вызывается под удержанным mu.
func (s *Simulation) updateHeatShield(dt float64, nav navSnapshot) {
	if s.heatShield == nil {
		return
	}

	// Брюхо — это направление связанной оси Z в инерциальных осях.
	belly := s.attitude.Orientation.Rotate(physics.Vec3{Z: 1})

	s.heatShield.Update(dt, belly, nav.AirRelative, nav.HeatFlux)

	if s.heatShield.BurnedThrough && !s.crashed {
		s.crashed = true
		s.logHeatShieldFailure()
	}
}

// navSnapshot — то немногое, что нужно теплозащите от навигации.
type navSnapshot struct {
	AirRelative physics.Vec3
	HeatFlux    float64
}

// logHeatShieldFailure записывает в журнал причину гибели аппарата.
func (s *Simulation) logHeatShieldFailure() {
	h := s.heatShield
	side := h.Windward
	if h.Leeward.Damage > h.Windward.Damage {
		side = h.Leeward
	}

	name := "плитки"
	if side.Name == "steel" {
		name = "голая сталь подветренной стороны"
	}

	logHeatShield(s.elapsed, name, side.Temperature, side.Limit, h.Exposure)
}

// logHeatShield печатает обстоятельства прогара.
func logHeatShield(t float64, side string, temp, limit, exposure float64) {
	orientation := "плитками вперёд"
	if exposure < 0 {
		orientation = "спиной вперёд"
	} else if math.Abs(exposure) < 0.3 {
		orientation = "боком к потоку"
	}

	log.Printf("🔥 Прогар на T+%.1f с: %s, %.0f К при пределе %.0f К. "+
		"Корабль шёл %s (подставленность брюха %.2f)",
		t, side, temp, limit, orientation, exposure)
}

// SetManualAttitude назначает ориентацию вручную.
//
// Пока она задана, наведение своей команды не выдаёт: корпус идёт туда, куда
// сказал оператор. Это и есть способ подставить потоку ту сторону, какую он
// считает нужным, — например, развернуть корабль плитками от потока
// и посмотреть, сколько проживёт голая сталь.
func (s *Simulation) SetManualAttitude(pitch, yaw, roll float64) {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.manualAttitude = &physics.Attitude{Pitch: pitch, Yaw: yaw, Roll: roll}
}

// ReleaseManualAttitude возвращает ориентацию системе управления.
func (s *Simulation) ReleaseManualAttitude() {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.manualAttitude = nil
}

// HeatShieldTelemetry — состояние теплозащиты для интерфейса.
type HeatShieldTelemetry struct {
	// Exposure — подставленность брюха потоку: единица — плитками вперёд,
	// минус единица — спиной.
	Exposure float64 `json:"exposure"`

	Tiles HeatShieldSideTelemetry `json:"tiles"`
	Steel HeatShieldSideTelemetry `json:"steel"`

	BurnedThrough bool `json:"burnedThrough"`

	// ManualAttitude сообщает, что ориентацию задаёт оператор.
	ManualAttitude bool `json:"manualAttitude"`
}

// HeatShieldSideTelemetry — состояние одной стороны.
type HeatShieldSideTelemetry struct {
	Temperature float64 `json:"temperature"` // К
	Limit       float64 `json:"limit"`       // К
	Margin      float64 `json:"margin"`      // К
	HeatFlux    float64 `json:"heatFlux"`    // Вт/м²
	Damage      float64 `json:"damage"`      // 0…1
}

// buildHeatShieldTelemetry собирает состояние теплозащиты.
func (s *Simulation) buildHeatShieldTelemetry() *HeatShieldTelemetry {
	h := s.heatShield
	if h == nil {
		return nil
	}

	side := func(x HeatShieldSide) HeatShieldSideTelemetry {
		return HeatShieldSideTelemetry{
			Temperature: x.Temperature,
			Limit:       x.Limit,
			Margin:      x.Limit - x.Temperature,
			HeatFlux:    x.HeatFlux,
			Damage:      x.Damage,
		}
	}

	return &HeatShieldTelemetry{
		Exposure:       h.Exposure,
		Tiles:          side(h.Windward),
		Steel:          side(h.Leeward),
		BurnedThrough:  h.BurnedThrough,
		ManualAttitude: s.manualAttitude != nil,
	}
}
