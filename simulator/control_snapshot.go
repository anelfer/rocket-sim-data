package simulator

import (
	"math"
	"time"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/propulsion"
)

// -----------------------------------------------------------------------------
// Снимок состояния для пульта управления.
//
// Интерфейс получает готовую картину: что задано оператором, что скомандовано
// системой управления, что происходит в модели и что показывают приборы.
// Эти четыре величины намеренно разделены — в них и заключена вся разница
// между «нарисовать значение» и «изменить режим работы агрегата».
// -----------------------------------------------------------------------------

// ParamValue — состояние одного управляемого параметра.
type ParamValue struct {
	ID        string `json:"id"`
	Subsystem string `json:"subsystem"`
	Unit      string `json:"unit"`

	// vehicle.Engine — двигатель, которому адресовано воздействие. Пусто, если
	// параметр под управлением модели или воздействие общее для ступени.
	Engine string `json:"engine,omitempty"`

	// Target — конечное значение, заданное оператором. Не определено,
	// если параметр под управлением модели.
	Target Float `json:"target"`

	// Commanded — значение, действующее прямо сейчас. На плавном переходе
	// отличается от целевого: команда ещё отрабатывается.
	Commanded Float `json:"commanded"`

	// Actual — фактическое значение в физической модели.
	Actual Float `json:"actual"`

	// Measured — то, что показывает датчик: с шумом, задержкой и дрейфом.
	Measured Float `json:"measured"`

	// Nominal — штатное значение для этой ступени.
	Nominal float64 `json:"nominal"`

	// Manual сообщает, что параметр под ручным управлением.
	Manual bool `json:"manual"`

	// Setting — текущее значение самого параметра: номинал, если оператор
	// его не трогал, или заданная величина, если трогал.
	//
	// Это и есть то, чем управляет оператор. Раньше в строке параметра
	// показывалась не она, а наблюдаемая величина из поля Reads: у множителя
	// напора с номиналом единица горело «40,39» — давление за насосом
	// в мегапаскалях.
	Setting Float `json:"setting"`

	// ReadsTitle, ReadsUnit — что именно наблюдает этот параметр
	// и в каких единицах. Величина чужая, поэтому и подписывать её надо
	// своим именем и своей размерностью.
	ReadsTitle string `json:"readsTitle,omitempty"`
	ReadsUnit  string `json:"readsUnit,omitempty"`

	// Status — состояние параметра: normal, manual, no_data.
	Status string `json:"status"`
}

// PumpView — состояние одного насоса для быстрой панели эксперимента.
type PumpView struct {
	Engine string `json:"engine"`
	Name   string `json:"name"`
	Side   string `json:"side"`
	State  string `json:"state"`

	TargetRPM    Float   `json:"targetRpm"`
	CommandedRPM Float   `json:"commandedRpm"`
	ActualRPM    float64 `json:"actualRpm"`
	MeasuredRPM  Float   `json:"measuredRpm"`
	NominalRPM   float64 `json:"nominalRpm"`

	// Величины ниже могут быть не определены: на остановленном вале
	// кавитационный запас и КПД смысла не имеют. Такие значения уходят
	// в интерфейс как null, а не как ноль: ноль здесь означал бы
	// «запас исчерпан», что неверно.
	InletPressure  Float `json:"inletPressure"`  // кПа
	OutletPressure Float `json:"outletPressure"` // МПа
	Head           Float `json:"head"`           // м
	MassFlow       Float `json:"massFlow"`       // кг/с
	Power          Float `json:"power"`          // МВт
	Efficiency     Float `json:"efficiency"`

	Vibration          Float `json:"vibration"`          // g
	BearingTemperature Float `json:"bearingTemperature"` // К
	CavitationMargin   Float `json:"cavitationMargin"`   // м
	NPSHRequired       Float `json:"npshRequired"`       // м
	CavitationSeverity Float `json:"cavitationSeverity"`
	AxialDisplacement  Float `json:"axialDisplacement"` // мм
}

// ValveView — состояние одного клапана.
type ValveView struct {
	Engine    string  `json:"engine"`
	ID        string  `json:"id"`
	Title     string  `json:"title"`
	Subsystem string  `json:"subsystem"`
	Commanded Float   `json:"commanded"`
	Actual    Float   `json:"actual"`
	Measured  Float   `json:"measured"`
	Rate      float64 `json:"rate"`
	Status    string  `json:"status"`
	ChangedAt float64 `json:"changedAt"`
}

// Alarm — предупреждение по наблюдаемой величине.
type Alarm struct {
	Key      string `json:"key"`
	Title    string `json:"title"`
	Value    Float  `json:"value"`
	Unit     string `json:"unit"`
	Severity string `json:"severity"`
	Message  string `json:"message"`
}

// EngineView — сводка по одному двигателю блока.
//
// Двигателей на первой ступени девять, и каждый — отдельное изделие со своим
// турбонасосом. Сводка нужна, чтобы отказ одного агрегата было видно на фоне
// восьми исправных.
type EngineView struct {
	ID      string `json:"id"`
	Running bool   `json:"running"`
	State   string `json:"state"`

	ShaftRPM        Float `json:"shaftRpm"`        // об/мин, фактические
	Thrust          Float `json:"thrust"`          // кН
	ChamberPressure Float `json:"chamberPressure"` // МПа
	MixtureRatio    Float `json:"mixtureRatio"`
	SpecificImpulse Float `json:"specificImpulse"` // с
	Vibration       Float `json:"vibration"`       // g
	BearingTemp     Float `json:"bearingTemp"`     // К
	WallMargin      Float `json:"wallMargin"`      // К
	FuelCavMargin   Float `json:"fuelCavMargin"`   // м
	OxCavMargin     Float `json:"oxCavMargin"`     // м

	// Manual сообщает, что на этот двигатель наложено адресное воздействие.
	Manual bool `json:"manual"`

	// Primary отмечает двигатель, по которому ведётся спектральный анализ.
	Primary bool `json:"primary"`
}

// ControlSnapshot — полное состояние для интерфейса управления.
type ControlSnapshot struct {
	ModelTime float64  `json:"modelTime"`
	RealTime  float64  `json:"realTime"`
	SentAt    int64    `json:"sentAt"` // миллисекунды Unix, для оценки задержки
	RunState  RunState `json:"runState"`
	Scale     float64  `json:"scale"`
	RunNumber int      `json:"runNumber"`
	Scenario  string   `json:"scenario"`
	Seed      int64    `json:"seed"`
	Phase     string   `json:"phase"`
	Stage     int      `json:"stage"`

	UnsafeMode bool `json:"unsafeMode"`
	DirectMode bool `json:"directMode"`

	Telemetry Telemetry      `json:"telemetry"`
	Effects   []control.View `json:"effects"`
	Values    []ParamValue   `json:"values"`
	Pumps     []PumpView     `json:"pumps"`
	Valves    []ValveView    `json:"valves"`
	Alarms    []Alarm        `json:"alarms"`

	// Engines — сводка по всем двигателям ступени.
	Engines []EngineView `json:"engines"`

	// EnginesOut — число двигателей, не создающих тяги.
	EnginesOut int `json:"enginesOut"`

	// EngineThrottle — уровень тяги, поданный на исправные двигатели.
	// Выше требуемого, если часть двигателей выбыла.
	EngineThrottle float64 `json:"engineThrottle"`

	// ThrottleDemand — требуемый от ступени уровень тяги.
	ThrottleDemand float64 `json:"throttleDemand"`

	// ThrustDeficit — недобор тяги против требуемого, доля.
	ThrustDeficit float64 `json:"thrustDeficit"`

	// Selected — двигатель, предлагаемый интерфейсом по умолчанию.
	Selected string `json:"selected"`
}

// ControlSnapshot собирает состояние для интерфейса управления.
func (s *Simulation) ControlSnapshot() ControlSnapshot {
	tel := s.Snapshot()

	s.mu.RLock()
	snap := ControlSnapshot{
		ModelTime: s.elapsed,
		RunState:  s.runState,
		Scale:     s.Time.Scale,
		RunNumber: s.runNumber,
		Scenario:  s.scenario,
		Seed:      s.seed,
		Phase:     tel.Phase,
		Stage:     s.stage,
		Telemetry: tel,

		EnginesOut:     s.propulsion.EnginesOut,
		EngineThrottle: s.propulsion.EngineThrottle,
		ThrottleDemand: s.propulsion.ThrottleDemand,
		ThrustDeficit:  s.propulsion.ThrustDeficit,
	}
	// Пока прогон идёт, реальное время — часы на стене. Как только он
	// остановлен, поставлен на паузу или завершён, оно замирает на
	// haltedAt: снимок можно запросить и через час после Stop, и реальное
	// время не должно расти вместе с этим часом.
	if !s.startedAt.IsZero() {
		ref := time.Now()
		if s.runState != RunRunning && !s.haltedAt.IsZero() {
			ref = s.haltedAt
		}
		snap.RealTime = ref.Sub(s.startedAt).Seconds()
	}
	engine := s.propulsion.PrimaryEngine()
	all := append([]*propulsion.Engine(nil), s.propulsion.Engines...)
	s.mu.RUnlock()

	snap.SentAt = time.Now().UnixMilli()
	snap.UnsafeMode, snap.DirectMode = s.board.Modes()
	snap.Effects = s.board.Effects()

	sample := tel.controlSample()
	snap.Values = s.paramValues(sample)
	snap.Alarms = alarmsFor(sample)

	// Адресные воздействия — по двигателям, чтобы интерфейс мог отметить,
	// какой агрегат сейчас под ручным управлением.
	manual := make(map[string]bool)
	for _, e := range snap.Effects {
		if e.Engine != "" {
			manual[e.Engine] = true
		}
	}

	byID := make(map[string]propulsion.EngineTelemetry, len(tel.Propulsion.Engines))
	for _, t := range tel.Propulsion.Engines {
		byID[t.ID] = t
	}

	snap.Engines = make([]EngineView, 0, len(all))
	for _, e := range all {
		snap.Engines = append(snap.Engines, engineView(e, byID[e.ID], manual[e.ID]))
	}

	// Насосы отдаются по всем двигателям сразу: выбор агрегата в интерфейсе
	// не должен требовать обращения к серверу, а состояние соседних насосов
	// нужно видеть одновременно.
	snap.Pumps = make([]PumpView, 0, 2*len(all))
	snap.Valves = make([]ValveView, 0, 4*len(all))
	for _, e := range all {
		snap.Pumps = append(snap.Pumps, pumpViews(e, byID[e.ID], snap.Values)...)
		snap.Valves = append(snap.Valves, valveViews(e, byID[e.ID])...)
	}
	if engine != nil {
		snap.Selected = engine.ID
	}
	return snap
}

// engineView собирает сводку по одному двигателю.
func engineView(e *propulsion.Engine, t propulsion.EngineTelemetry, manual bool) EngineView {
	tp := e.Turbopump

	// Состояние двигателя определяется состоянием его турбонасоса: именно
	// агрегат первым выходит за пределы при любом отказе подачи. Смотреть
	// нужно на оба насоса — кавитация в тракте окислителя не менее опасна,
	// чем в тракте горючего, и по одному насосу её видно не было бы.
	state := pumpState(tp, tp.FuelPump, e.Running)
	if tp.OxPump.CavitationSeverity > tp.FuelPump.CavitationSeverity {
		state = pumpState(tp, tp.OxPump, e.Running)
	}

	return EngineView{
		ID:              e.ID,
		Running:         e.Running,
		State:           state,
		ShaftRPM:        Float(tp.RPM()),
		Thrust:          Float(e.Thrust / 1e3),
		ChamberPressure: Float(e.Chamber.MeanPressure / 1e6),
		MixtureRatio:    Float(e.Chamber.MixtureRatio),
		SpecificImpulse: Float(e.SpecificImpulse),
		Vibration:       Float(tp.Vibration),
		BearingTemp:     Float(tp.BearingTemperature),
		WallMargin:      Float(e.Nozzle.MinimumMargin()),
		FuelCavMargin:   Float(tp.FuelPump.CavitationMargin),
		OxCavMargin:     Float(tp.OxPump.CavitationMargin),
		Manual:          manual,
		Primary:         e.Detailed,
	}
}

// paramValues строит состояние каждого управляемого параметра.
func (s *Simulation) paramValues(sample control.Sample) []ParamValue {
	effects := make(map[string]control.View, len(s.board.Effects()))
	for _, e := range s.board.Effects() {
		effects[e.Parameter] = e
	}

	params := control.Params()
	out := make([]ParamValue, 0, len(params))

	for _, p := range params {
		v := ParamValue{
			ID:        p.ID,
			Subsystem: p.Subsystem,
			Unit:      p.Unit,
			Nominal:   p.Nominal,
			Status:    "normal",
		}
		v.Setting = Float(p.Nominal)
		v.ReadsTitle, v.ReadsUnit = watchTitleUnit(p.Reads)

		if e, ok := effects[p.ID]; ok {
			v.Manual = true
			v.Engine = e.Engine
			v.Target = Float(e.Target)
			v.Commanded = Float(e.Current)
			v.Setting = Float(e.Current)
			v.Nominal = e.Nominal
			v.Status = "manual"
		} else {
			v.Target = Float(math.NaN())
			v.Commanded = Float(math.NaN())
		}

		// Фактическое и измеренное значения берутся из телеметрии. Для
		// большинства параметров это одна и та же величина, для оборотов
		// и давлений — разные: одна из модели, другая из датчика.
		// Наблюдаемая величина показывается как есть, но состояние строки
		// по ней больше не определяется.
		//
		// Прежде цвет параметра выставлялся сравнением чужой величины
		// с пределами самого параметра: давление за насосом в сорок
		// мегапаскалей сравнивалось с пределами множителя напора, у которого
		// критический порог — 1.2. В итоге на штатном режиме почти вся левая
		// панель горела красным, и отличить настоящее отклонение от этой
		// путаницы было невозможно.
		//
		// За состоянием наблюдаемых величин следят предупреждения: там пороги
		// заданы для самих величин, а не для множителей.
		if p.Reads != "" {
			if got, ok := sample[p.Reads]; ok {
				v.Measured = Float(got)
				v.Actual = Float(got)
			} else {
				v.Measured = Float(math.NaN())
				v.Actual = Float(math.NaN())
				if !v.Manual {
					v.Status = "no_data"
				}
			}
		}
		out = append(out, v)
	}
	return out
}

// classifyByWatch относит значение к диапазону по описанию параметра реестра.
// watchTitleUnit возвращает название и единицы наблюдаемой величины.
func watchTitleUnit(key string) (string, string) {
	if key == "" {
		return "", ""
	}
	for _, w := range control.Watches() {
		if w.Key == key {
			return w.Title, w.Unit
		}
	}
	return key, ""
}

// pumpViews собирает состояние обоих насосов.
//
// Вал у насосов общий, поэтому обороты у них одинаковые: раскрутить один
// насос независимо от другого физически нельзя. Различаются условия на входе,
// напор, мощность и кавитационный запас — именно они и показываются отдельно.
func pumpViews(e *propulsion.Engine, t propulsion.EngineTelemetry, values []ParamValue) []PumpView {
	tp := e.Turbopump
	nominal := tp.Config.DesignSpeed * 60 / (2 * math.Pi)

	var target, commanded Float = Float(math.NaN()), Float(math.NaN())
	for _, v := range values {
		if v.ID == "tp.shaft.speed" && v.Manual && (v.Engine == "" || v.Engine == e.ID) {
			target, commanded = v.Target, v.Commanded
		}
	}

	build := func(side, name string, p propulsion.Pump, inlet, outlet, flow, power float64) PumpView {
		return PumpView{
			Engine: e.ID, Name: name, Side: side,
			State:        pumpState(tp, p, e.Running),
			TargetRPM:    target,
			CommandedRPM: commanded,
			ActualRPM:    tp.RPM(),
			MeasuredRPM:  Float(t.ShaftRPM),
			NominalRPM:   nominal,

			InletPressure:      Float(inlet / 1e3),
			OutletPressure:     Float(outlet / 1e6),
			Head:               Float(p.Head),
			MassFlow:           Float(flow),
			Power:              Float(power / 1e6),
			Efficiency:         Float(p.EffectiveEfficiency),
			Vibration:          Float(tp.Vibration),
			BearingTemperature: Float(tp.BearingTemperature),
			CavitationMargin:   Float(p.CavitationMargin),
			NPSHRequired:       Float(p.NPSHRequired),
			CavitationSeverity: Float(p.CavitationSeverity),
			AxialDisplacement:  Float(tp.AxialDisplacement * 1000),
		}
	}

	return []PumpView{
		build("fuel", "Насос горючего", tp.FuelPump,
			tp.FuelPump.InletPressure, tp.FuelPump.OutletPressure,
			tp.FuelPump.MassFlow, tp.FuelPump.Power),
		build("ox", "Насос окислителя", tp.OxPump,
			tp.OxPump.InletPressure, tp.OxPump.OutletPressure,
			tp.OxPump.MassFlow, tp.OxPump.Power),
	}
}

// bearingLimit — температура подшипников, выше которой узел считается
// перегретым, К.
//
// Порог выставлен по фактическому поведению модели, а не по интуиции:
// на штатном режиме подшипники прогреваются до 430…460 К, а к концу работы
// ступени, когда падает кавитационный запас и растёт вибрация, доходят
// до 550 К. Порог ниже этого уровня означал бы предупреждение на исправном
// агрегате, и оператор перестал бы обращать на него внимание.
const bearingLimit = 600.0

// pumpState определяет состояние насоса по данным модели.
//
// Состояние не задаётся оператором: оно выводится из оборотов, кавитационного
// запаса, температуры подшипников и осевого смещения ротора.
func pumpState(tp *propulsion.Turbopump, p propulsion.Pump, running bool) string {
	fraction := tp.SpeedFraction()

	switch {
	case tp.Speed < 1:
		return "Stopped"

	// Выбегающий вал остановленного двигателя — это остановленный агрегат,
	// а не работающий с пониженной эффективностью.
	case !running && fraction < 0.5:
		return "Stopped"
	case math.Abs(tp.AngularAcceleration) < 1e-9 && tp.Vibration > 30:
		return "Stuck"
	case p.CavitationSeverity > 0.5:
		return "Failed"
	case p.CavitationSeverity > 0:
		return "Cavitation"
	case fraction > 1.08:
		return "Overspeed"
	case tp.BearingTemperature > bearingLimit:
		return "Bearing overheating"
	// Осевое смещение ротора допустимо до двух миллиметров: упорный
	// подшипник рассчитан на неуравновешенный перепад давления на
	// крыльчатках, и на номинале ротор всегда стоит не в нуле.
	case math.Abs(tp.AxialDisplacement) > 2e-3:
		return "Shaft displacement"
	// Потеря КПД — это действительно ухудшение: насос при тех же оборотах
	// даёт меньший напор.
	case p.EffectiveEfficiency < p.Config.Efficiency*0.9:
		return "Degraded"

	// А низкие обороты сами по себе ничего плохого не означают. Глубокое
	// дросселирование — штатный режим: агрегат исправен, просто работает
	// не на номинале. Прежде это состояние называлось «Degraded», и панель
	// показывала неисправность там, где двигатель просто убавили.
	case fraction < 0.75:
		return "Throttled"
	default:
		return "Normal"
	}
}

// valveViews собирает состояние клапанов.
func valveViews(e *propulsion.Engine, t propulsion.EngineTelemetry) []ValveView {
	tp := e.Turbopump

	items := []struct {
		id, title, subsystem string
		actual               float64
	}{
		{"valve.fuel", "Главный клапан горючего", "Компоненты", tp.FuelValve},
		{"valve.ox", "Главный клапан окислителя", "Компоненты", tp.OxValve},
		{"valve.gasgen", "Клапан газогенератора", "Турбина", tp.GasGenValve},
		{"valve.bypass", "Перепускной клапан турбины", "Турбина", tp.BypassValve},
	}

	out := make([]ValveView, 0, len(items))
	for _, it := range items {
		status := "normal"
		switch {
		case it.actual <= 0.01:
			status = "closed"
		case it.actual < 0.95:
			status = "throttled"
		}
		out = append(out, ValveView{
			Engine: e.ID,
			ID:     it.id, Title: it.title, Subsystem: it.subsystem,
			Commanded: Float(it.actual), Actual: Float(it.actual), Measured: Float(it.actual),
			Rate: tp.Config.ValveRate, Status: status,
		})
	}
	return out
}

// alarmsFor формирует предупреждения по фактическим значениям.
func alarmsFor(sample control.Sample) []Alarm {
	limits := []struct {
		key, title, unit string
		warn, crit       float64
		high             bool
		message          string
	}{
		{"turbopump.vibration", "Вибрация турбонасоса", "g", 6, 15, true,
			"Рост вибрации разрушает подшипники и уплотнения"},
		{"turbopump.bearingTemperature", "Температура подшипников", "К", 560, 680, true,
			"Перегрев подшипников ведёт к заклиниванию вала"},
		{"turbopump.fuelCavitationMargin", "Кавитационный запас горючего", "м", 3, 0, false,
			"Кавитация срывает подачу и раскачивает камеру"},
		{"turbopump.oxCavitationMargin", "Кавитационный запас окислителя", "м", 3, 0, false,
			"Кавитация срывает подачу и раскачивает камеру"},
		{"chamber.stabilityMargin", "Запас устойчивости горения", "", 0.35, 0.12, false,
			"Потеря устойчивости разрушает камеру за секунды"},
		{"nozzle.wallMargin", "Запас до предела стенки", "К", 200, 50, false,
			"Стенка близка к прогару"},
		// Ноль здесь — это норма, а не тревога: на установившемся режиме
		// турбина отдаёт ровно столько, сколько забирают насосы, и запас
		// обязан быть нулевым. Прежние пороги (тревога ниже 0.05 МВт)
		// светились на каждом штатном полёте.
		{"turbopump.powerMargin", "Запас мощности турбины", "МВт", -0.5, -3, false,
			"Турбине не хватает мощности, обороты падают"},
		{"nozzle.wallDamage", "Повреждение стенки", "", 0.05, 0.5, true,
			"Перегретая стенка теряет прочность и прогорает"},
		// Прямой признак того, что ступень не получает требуемой тяги:
		// встал турбонасос, погас или прогорел двигатель, выбыла камера.
		// Раньше это ловилось косвенно, через запас мощности турбины,
		// и вместе с ложными срабатываниями на штатном режиме.
		{"engine.thrustDeficit", "Недобор тяги", "доля", 0.03, 0.25, true,
			"Ступень не получает требуемой тяги"},

		// Теплозащита корабля. Голая сталь спины держит вдвое меньше плиток,
		// поэтому запас у неё свой: развернулся корпус спиной к потоку —
		// и до предела остаются секунды.
		{"shield.tileMargin", "Запас плиток теплозащиты", "К", 200, 60, false,
			"Плитки близки к пределу материала"},
		{"shield.steelMargin", "Запас голой стали", "К", 150, 40, false,
			"Подветренная сторона греется: корпус развёрнут не той стороной"},
		{"shield.damage", "Повреждение теплозащиты", "", 0.02, 0.3, true,
			"Теплозащита разрушается, за ней прогар корпуса"},
	}

	var out []Alarm
	for _, l := range limits {
		v, ok := sample[l.key]
		if !ok || math.IsNaN(v) {
			continue
		}
		severity := ""
		if l.high {
			switch {
			case v >= l.crit:
				severity = "critical"
			case v >= l.warn:
				severity = "warning"
			}
		} else {
			switch {
			case v <= l.crit:
				severity = "critical"
			case v <= l.warn:
				severity = "warning"
			}
		}
		if severity == "" {
			continue
		}
		out = append(out, Alarm{
			Key: l.key, Title: l.title, Value: Float(v), Unit: l.unit,
			Severity: severity, Message: l.message,
		})
	}
	return out
}
