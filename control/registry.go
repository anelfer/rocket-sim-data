package control

import "math"

// -----------------------------------------------------------------------------
// Реестр управляемых параметров.
//
// Реестр — единственное место, где описано, что вообще можно менять. Интерфейс
// строится по нему, проверка команд идёт по нему же, и физическая модель
// получает поправки тоже через него. Добавить параметр — значит добавить сюда
// одну запись, а не править интерфейс, валидацию и модель по отдельности.
// -----------------------------------------------------------------------------

// Kind — характер управляемой величины.
type Kind string

const (
	// KindNumeric — непрерывная величина с диапазоном.
	KindNumeric Kind = "numeric"

	// KindFraction — доля от нуля до единицы, показывается в процентах.
	KindFraction Kind = "fraction"

	// KindFactor — безразмерный множитель к штатному значению.
	KindFactor Kind = "factor"
)

// Param — описание управляемого параметра.
type Param struct {
	ID        string `json:"id"`
	Title     string `json:"title"`
	Subsystem string `json:"subsystem"`
	Group     string `json:"group"`
	Unit      string `json:"unit"`
	Kind      Kind   `json:"kind"`

	// Nominal — штатное значение. Для множителей — единица.
	Nominal float64 `json:"nominal"`

	// Min, Max — абсолютные допустимые пределы. Выход за них разрешён только
	// в режиме Unsafe overrides.
	Min float64 `json:"min"`
	Max float64 `json:"max"`

	// WarnLo, WarnHi — границы номинального диапазона. За ними параметр
	// считается работающим в предупредительной зоне.
	WarnLo float64 `json:"warnLo"`
	WarnHi float64 `json:"warnHi"`

	// CritLo, CritHi — границы предупредительного диапазона. За ними зона
	// критическая.
	CritLo float64 `json:"critLo"`
	CritHi float64 `json:"critHi"`

	// Step — шаг кнопок быстрого изменения.
	Step float64 `json:"step"`

	// Direct отмечает прямую подмену состояния модели: причинность на этой
	// величине временно нарушается.
	Direct bool `json:"direct"`

	// Consequence — что произойдёт при выходе за допустимые пределы.
	// Показывается оператору при подтверждении опасной команды.
	Consequence string `json:"consequence"`

	// Reads — ключ фактического значения в снимке телеметрии. По нему
	// интерфейс показывает, к чему привело воздействие.
	Reads string `json:"reads"`

	// Affects — параметры, на которые эта величина влияет напрямую.
	// Используется для построения цепочки причин и следствий.
	Affects []string `json:"affects,omitempty"`

	// scale — множитель перевода из единиц интерфейса в единицы модели.
	scale float64

	// apply записывает значение в поправки. Значение приходит в единицах
	// модели, то есть уже умноженное на scale.
	apply func(ov *Overrides, v float64)

	// fail применяет отказ.
	fail func(ov *Overrides, f Failure)
}

// ToModel переводит значение из единиц интерфейса в единицы модели.
func (p Param) ToModel(v float64) float64 {
	if p.scale == 0 {
		return v
	}
	return v * p.scale
}

// ToDisplay переводит значение модели в единицы интерфейса.
func (p Param) ToDisplay(v float64) float64 {
	if p.scale == 0 {
		return v
	}
	return v / p.scale
}

// Clamp ограничивает значение допустимыми пределами.
func (p Param) Clamp(v float64) float64 {
	return math.Max(p.Min, math.Min(p.Max, v))
}

// Classify возвращает состояние параметра по его значению.
func (p Param) Classify(v float64) string {
	switch {
	case math.IsNaN(v):
		return "no_data"
	case v < p.CritLo || v > p.CritHi:
		return "critical"
	case v < p.WarnLo || v > p.WarnHi:
		return "warning"
	default:
		return "normal"
	}
}

const rpmToRad = 2 * math.Pi / 60

// registry — все управляемые параметры по идентификаторам.
var registry = buildRegistry()

// order — порядок вывода параметров в интерфейсе.
var order []string

// Lookup возвращает описание параметра.
func Lookup(id string) (Param, bool) {
	p, ok := registry[id]
	return p, ok
}

// Params возвращает все параметры в порядке объявления.
func Params() []Param {
	out := make([]Param, 0, len(order))
	for _, id := range order {
		out = append(out, registry[id])
	}
	return out
}

func buildRegistry() map[string]Param {
	list := []Param{
		// ---------------------------------------------------------------------
		// Турбонасосный агрегат
		// ---------------------------------------------------------------------
		{
			ID: "tp.shaft.speed", Title: "Уставка оборотов вала",
			Subsystem: "turbopump", Group: "Вал", Unit: "об/мин", Kind: KindNumeric,
			Nominal: 32394, Min: 0, Max: 55000,
			WarnLo: 24000, WarnHi: 34000, CritLo: 15000, CritHi: 40000,
			Step: 500, scale: rpmToRad,
			Consequence: "Превышение предельных оборотов разрушает крыльчатку " +
				"и выводит подшипники за температурный предел.",
			Reads: "turbopump.rpm",
			Affects: []string{"turbopump.fuelPumpOutlet", "turbopump.oxPumpOutlet",
				"engine.fuelFlow", "engine.oxFlow", "chamber.pressure", "engine.thrust"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Turbopump.SpeedTarget = Set(v)
			},
			fail: func(ov *Overrides, f Failure) {
				switch f {
				case FailureStuck:
					ov.Engine.Turbopump.Locked = true
				case FailureStopped, FailureUnpowered:
					ov.Engine.Turbopump.Unpowered = true
				}
			},
		},
		{
			ID: "tp.shaft.spin_rate_limit", Title: "Ограничение углового ускорения",
			Subsystem: "turbopump", Group: "Вал", Unit: "рад/с²", Kind: KindNumeric,
			Nominal: 0, Min: 0, Max: 20000,
			WarnLo: 0, WarnHi: 20000, CritLo: 0, CritHi: 20000, Step: 100,
			Consequence: "Слишком жёсткое ограничение не даёт агрегату выйти на режим.",
			Reads:       "turbopump.angularAcceleration",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Turbopump.SpinRateLimit = Set(v)
			},
		},
		{
			ID: "tp.shaft.torque_factor", Title: "Множитель момента турбины",
			Subsystem: "turbopump", Group: "Турбина", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 2,
			WarnLo: 0.9, WarnHi: 1.1, CritLo: 0.6, CritHi: 1.3, Step: 0.05,
			Consequence: "Падение момента турбины роняет обороты и давление в камере.",
			Reads:       "turbopump.turbinePower",
			Affects:     []string{"turbopump.rpm", "chamber.pressure", "engine.thrust"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Turbopump.TurbineTorqueFactor = Set(v)
			},
		},
		{
			ID: "tp.shaft.mech_load", Title: "Множитель механических потерь",
			Subsystem: "turbopump", Group: "Вал", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 10,
			WarnLo: 0.8, WarnHi: 1.5, CritLo: 0.5, CritHi: 3, Step: 0.1,
			Consequence: "Рост трения перегревает подшипники и тормозит вал.",
			Reads:       "turbopump.mechanicalLoss",
			Affects:     []string{"turbopump.rpm", "turbopump.bearingTemperature"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Turbopump.MechanicalLoadFactor = Set(v)
			},
		},
		{
			ID: "tp.bearing.cooling", Title: "Множитель теплоотвода подшипников",
			Subsystem: "turbopump", Group: "Подшипники", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0.05, Max: 3,
			WarnLo: 0.8, WarnHi: 1.5, CritLo: 0.4, CritHi: 2, Step: 0.05,
			Consequence: "Потеря охлаждения подшипников ведёт к заклиниванию вала.",
			Reads:       "turbopump.bearingTemperature",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Turbopump.BearingCoolingFactor = Set(v)
			},
		},
		{
			ID: "tp.vibration.bias", Title: "Добавка вибрации",
			Subsystem: "turbopump", Group: "Подшипники", Unit: "g", Kind: KindNumeric,
			Nominal: 0, Min: 0, Max: 60,
			WarnLo: 0, WarnHi: 6, CritLo: 0, CritHi: 15, Step: 0.5,
			Consequence: "Высокая вибрация разрушает уплотнения и подшипники.",
			Reads:       "turbopump.vibration",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Turbopump.VibrationBias = Set(v)
			},
		},
		{
			ID: "tp.shaft.speed_direct", Title: "Обороты вала напрямую",
			Subsystem: "turbopump", Group: "Отладка", Unit: "об/мин", Kind: KindNumeric,
			Nominal: 32394, Min: 0, Max: 60000,
			WarnLo: 0, WarnHi: 40000, CritLo: 0, CritHi: 50000,
			Step: 500, scale: rpmToRad, Direct: true,
			Consequence: "Баланс мощности перестаёт выполняться: обороты больше " +
				"не следуют из момента турбины.",
			Reads: "turbopump.rpm",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Turbopump.SpeedDirect = Set(v)
			},
		},
		{
			ID: "tp.bearing.temp_direct", Title: "Температура подшипников напрямую",
			Subsystem: "turbopump", Group: "Отладка", Unit: "К", Kind: KindNumeric,
			Nominal: 300, Min: 70, Max: 1500,
			WarnLo: 200, WarnHi: 560, CritLo: 100, CritHi: 680, Step: 10, Direct: true,
			Consequence: "Тепловой баланс подшипникового узла перестаёт считаться.",
			Reads:       "turbopump.bearingTemperature",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Turbopump.BearingTempDirect = Set(v)
			},
		},
	}

	list = append(list, pumpParams("fuel", "горючего")...)
	list = append(list, pumpParams("ox", "окислителя")...)
	list = append(list, valveParams()...)
	list = append(list, tankParams("fuel", "горючего")...)
	list = append(list, tankParams("ox", "окислителя")...)
	list = append(list, chamberParams()...)
	list = append(list, nozzleParams()...)
	list = append(list, engineParams()...)
	list = append(list, controlParams()...)

	m := make(map[string]Param, len(list))
	order = make([]string, 0, len(list))
	for _, p := range list {
		m[p.ID] = p
		order = append(order, p.ID)
	}
	return m
}

// pumpParams строит набор параметров для одного насоса.
func pumpParams(side, genitive string) []Param {
	prefix := "tp." + side + "_pump."
	group := "Насос " + genitive

	pick := func(ov *Overrides) *PumpOverrides {
		if side == "fuel" {
			return &ov.Engine.Turbopump.FuelPump
		}
		return &ov.Engine.Turbopump.OxPump
	}
	reads := "turbopump." + side + "Pump"

	return []Param{
		{
			ID: prefix + "head", Title: "Множитель напора",
			Subsystem: "turbopump", Group: group, Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 1.5,
			WarnLo: 0.9, WarnHi: 1.1, CritLo: 0.7, CritHi: 1.3, Step: 0.05,
			Consequence: "Падение напора снижает давление подачи и расход через форсунки.",
			Reads:       reads + "Outlet",
			Affects: []string{"engine." + side + "Flow", "chamber.pressure",
				"chamber.mixtureRatio", "engine.thrust"},
			apply: func(ov *Overrides, v float64) { pick(ov).HeadFactor = Set(v) },
		},
		{
			ID: prefix + "efficiency", Title: "Множитель КПД",
			Subsystem: "turbopump", Group: group, Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0.1, Max: 1.3,
			WarnLo: 0.9, WarnHi: 1.1, CritLo: 0.7, CritHi: 1.2, Step: 0.05,
			Consequence: "Падение КПД увеличивает потребляемую мощность и тормозит вал.",
			Reads:       reads + "Power",
			Affects:     []string{"turbopump.rpm", "turbopump.powerMargin"},
			apply:       func(ov *Overrides, v float64) { pick(ov).EfficiencyFactor = Set(v) },
		},
		{
			ID: prefix + "inlet_bias", Title: "Смещение давления на входе",
			Subsystem: "turbopump", Group: group, Unit: "кПа", Kind: KindNumeric,
			Nominal: 0, Min: -400, Max: 400,
			WarnLo: -30, WarnHi: 30, CritLo: -120, CritHi: 120,
			Step: 10, scale: 1000,
			Consequence: "Падение давления на входе вводит насос в кавитацию.",
			Reads:       reads + "Inlet",
			Affects: []string{"turbopump." + side + "CavitationMargin",
				"turbopump.vibration", "chamber.stabilityMargin"},
			apply: func(ov *Overrides, v float64) { pick(ov).InletPressureBias = Set(v) },
		},
		{
			ID: prefix + "suction", Title: "Множитель потребного запаса",
			Subsystem: "turbopump", Group: group, Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0.2, Max: 6,
			WarnLo: 0.9, WarnHi: 1.2, CritLo: 0.5, CritHi: 2, Step: 0.1,
			Consequence: "Повреждение шнека вводит насос в кавитацию при штатном наддуве.",
			Reads:       "turbopump." + side + "CavitationMargin",
			Affects:     []string{"turbopump.vibration", "chamber.pressureRMS"},
			apply:       func(ov *Overrides, v float64) { pick(ov).SuctionFactor = Set(v) },
		},
		{
			ID: prefix + "gas", Title: "Доля газа на входе",
			Subsystem: "turbopump", Group: group, Unit: "", Kind: KindFraction,
			Nominal: 0, Min: 0, Max: 1,
			WarnLo: 0, WarnHi: 0.02, CritLo: 0, CritHi: 0.1, Step: 0.05,
			Consequence: "Прорыв газа наддува в тракт срывает подачу компонента.",
			Reads:       "turbopump." + side + "CavitationMargin",
			Affects:     []string{"engine." + side + "Flow", "turbopump.vibration"},
			apply:       func(ov *Overrides, v float64) { pick(ov).GasIngestion = Set(v) },
			fail: func(ov *Overrides, f Failure) {
				if f == FailureStopped || f == FailureStuck {
					pick(ov).HeadFactor = Set(0)
				}
			},
		},
	}
}

// valveDescriptor описывает один клапан.
type valveDescriptor struct {
	id, title, group string
	pick             func(*ValveOverrides) *ValveOverride
	reads            string
	affects          []string
}

func valveDescriptors() []valveDescriptor {
	return []valveDescriptor{
		{"fuel", "Главный клапан горючего", "Компоненты",
			func(v *ValveOverrides) *ValveOverride { return &v.Fuel }, "valves.fuel",
			[]string{"engine.fuelFlow", "chamber.mixtureRatio", "chamber.pressure"}},
		{"ox", "Главный клапан окислителя", "Компоненты",
			func(v *ValveOverrides) *ValveOverride { return &v.Ox }, "valves.ox",
			[]string{"engine.oxFlow", "chamber.mixtureRatio", "chamber.pressure"}},
		{"gasgen", "Клапан газогенератора", "Турбина",
			func(v *ValveOverrides) *ValveOverride { return &v.GasGen }, "valves.gasGen",
			[]string{"turbopump.turbinePower", "turbopump.rpm", "chamber.pressure"}},
		{"bypass", "Перепускной клапан турбины", "Турбина",
			func(v *ValveOverrides) *ValveOverride { return &v.Bypass }, "valves.bypass",
			[]string{"turbopump.turbinePower", "turbopump.rpm"}},
		{"cooling", "Клапан охлаждающего контура", "Охлаждение",
			func(v *ValveOverrides) *ValveOverride { return &v.Cooling }, "nozzle.coolingFlow",
			[]string{"nozzle.wallMargin", "nozzle.throatErosion"}},
		{"fuel_prs", "Клапан наддува бака горючего", "Наддув",
			func(v *ValveOverrides) *ValveOverride { return &v.FuelPrs }, "fuelTank.pressure",
			[]string{"turbopump.fuelCavitationMargin", "engine.fuelFlow"}},
		{"ox_prs", "Клапан наддува бака окислителя", "Наддув",
			func(v *ValveOverrides) *ValveOverride { return &v.OxPrs }, "oxTank.pressure",
			[]string{"turbopump.oxCavitationMargin", "engine.oxFlow"}},
	}
}

func valveParams() []Param {
	var out []Param
	for _, d := range valveDescriptors() {
		d := d
		out = append(out,
			Param{
				ID: "valve." + d.id + ".command", Title: d.title,
				Subsystem: "valves", Group: d.group, Unit: "", Kind: KindFraction,
				Nominal: 1, Min: 0, Max: 1,
				WarnLo: 0.9, WarnHi: 1, CritLo: 0.5, CritHi: 1, Step: 0.05,
				Consequence: "Прикрытие клапана меняет расход и соотношение компонентов.",
				Reads:       d.reads, Affects: d.affects,
				apply: func(ov *Overrides, v float64) {
					d.pick(&ov.Engine.Turbopump.Valves).Command = Set(v)
				},
				fail: func(ov *Overrides, f Failure) {
					vo := d.pick(&ov.Engine.Turbopump.Valves)
					switch f {
					case FailureStuck, FailureUnresponsive, FailureFrozen:
						vo.Stuck = true
					case FailureStopped, FailureUnpowered:
						vo.Command = Set(0)
					case FailureBadSensor:
						vo.SensorBias = Set(0.25)
					}
				},
			},
			Param{
				ID: "valve." + d.id + ".max_open", Title: d.title + ": предел открытия",
				Subsystem: "valves", Group: d.group, Unit: "", Kind: KindFraction,
				Nominal: 1, Min: 0, Max: 1,
				WarnLo: 0.9, WarnHi: 1, CritLo: 0.5, CritHi: 1, Step: 0.05,
				Consequence: "Ограничение хода не даёт системе управления выйти на режим.",
				Reads:       d.reads,
				apply: func(ov *Overrides, v float64) {
					d.pick(&ov.Engine.Turbopump.Valves).MaxOpen = Set(v)
				},
			},
			Param{
				ID: "valve." + d.id + ".rate", Title: d.title + ": скорость перекладки",
				Subsystem: "valves", Group: d.group, Unit: "", Kind: KindFactor,
				Nominal: 1, Min: 0.02, Max: 10,
				WarnLo: 0.5, WarnHi: 2, CritLo: 0.1, CritHi: 5, Step: 0.1,
				Consequence: "Медленный привод затягивает переходные процессы.",
				Reads:       d.reads,
				apply: func(ov *Overrides, v float64) {
					d.pick(&ov.Engine.Turbopump.Valves).RateFactor = Set(v)
				},
			},
			Param{
				ID: "valve." + d.id + ".delay", Title: d.title + ": задержка реакции",
				Subsystem: "valves", Group: d.group, Unit: "с", Kind: KindNumeric,
				Nominal: 0, Min: 0, Max: 10,
				WarnLo: 0, WarnHi: 0.3, CritLo: 0, CritHi: 1, Step: 0.1,
				Consequence: "Запаздывание привода раскачивает контур регулирования.",
				Reads:       d.reads,
				apply: func(ov *Overrides, v float64) {
					d.pick(&ov.Engine.Turbopump.Valves).Delay = Set(v)
				},
			},
		)
	}
	return out
}

func tankParams(side, genitive string) []Param {
	prefix := "tank." + side + "."
	group := "Бак " + genitive
	reads := side + "Tank."

	pick := func(ov *Overrides) *TankOverrides {
		if side == "fuel" {
			return &ov.FuelTank
		}
		return &ov.OxTank
	}

	return []Param{
		{
			ID: prefix + "pressurant_valve", Title: "Клапан наддува",
			Subsystem: "tanks", Group: group, Unit: "", Kind: KindFraction,
			Nominal: 1, Min: 0, Max: 1,
			WarnLo: 0.5, WarnHi: 1, CritLo: 0.1, CritHi: 1, Step: 0.05,
			Consequence: "Потеря наддува роняет давление на входе насоса и вызывает кавитацию.",
			Reads:       reads + "pressure",
			Affects: []string{"turbopump." + side + "CavitationMargin",
				"engine." + side + "Flow", "chamber.pressure"},
			apply: func(ov *Overrides, v float64) { pick(ov).PressurantValve = Set(v) },
			fail: func(ov *Overrides, f Failure) {
				if f == FailureStopped || f == FailureUnpowered {
					pick(ov).PressurantValve = Set(0)
				}
			},
		},
		{
			ID: prefix + "pressurant_flow", Title: "Множитель расхода наддува",
			Subsystem: "tanks", Group: group, Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 3,
			WarnLo: 0.8, WarnHi: 1.5, CritLo: 0.4, CritHi: 2, Step: 0.1,
			Consequence: "Нехватка наддува ведёт к постепенному падению давления в баке.",
			Reads:       reads + "pressurantFlow",
			apply:       func(ov *Overrides, v float64) { pick(ov).PressurantFlowFactor = Set(v) },
		},
		{
			ID: prefix + "leak", Title: "Утечка компонента",
			Subsystem: "tanks", Group: group, Unit: "кг/с", Kind: KindNumeric,
			Nominal: 0, Min: 0, Max: 200,
			WarnLo: 0, WarnHi: 0.5, CritLo: 0, CritHi: 5, Step: 1,
			Consequence: "Утечка сокращает запас компонента и меняет момент выработки бака.",
			Reads:       reads + "mass",
			Affects:     []string{reads + "depletionTime", "engine.thrust"},
			apply:       func(ov *Overrides, v float64) { pick(ov).LeakRate = Set(v) },
		},
		{
			ID: prefix + "blockage", Title: "Засорение магистрали",
			Subsystem: "tanks", Group: group, Unit: "", Kind: KindFraction,
			Nominal: 0, Min: 0, Max: 1,
			WarnLo: 0, WarnHi: 0.1, CritLo: 0, CritHi: 0.4, Step: 0.05,
			Consequence: "Полное перекрытие магистрали прекращает подачу компонента.",
			Reads:       reads + "lineDrop",
			Affects: []string{"turbopump." + side + "PumpInlet",
				"engine." + side + "Flow", "chamber.mixtureRatio"},
			apply: func(ov *Overrides, v float64) { pick(ov).LineBlockage = Set(v) },
			fail: func(ov *Overrides, f Failure) {
				if f == FailureRuptured || f == FailureStopped {
					pick(ov).LineRuptured = true
				}
			},
		},
		{
			ID: prefix + "slosh_excitation", Title: "Возбуждение колебаний жидкости",
			Subsystem: "tanks", Group: group, Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 20,
			WarnLo: 0, WarnHi: 2, CritLo: 0, CritHi: 8, Step: 0.5,
			Consequence: "Сильные колебания жидкости раскачивают давление на входе насоса.",
			Reads:       reads + "sloshAmplitude",
			Affects:     []string{"turbopump." + side + "CavitationMargin"},
			apply:       func(ov *Overrides, v float64) { pick(ov).SloshExcitationFactor = Set(v) },
		},
		{
			ID: prefix + "slosh_damping", Title: "Демпфирование колебаний жидкости",
			Subsystem: "tanks", Group: group, Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0.02, Max: 10,
			WarnLo: 0.5, WarnHi: 2, CritLo: 0.1, CritHi: 5, Step: 0.1,
			Consequence: "Потеря демпферов делает колебания жидкости незатухающими.",
			Reads:       reads + "sloshAmplitude",
			apply:       func(ov *Overrides, v float64) { pick(ov).SloshDampingFactor = Set(v) },
		},
		{
			ID: prefix + "mass", Title: "Масса компонента напрямую",
			Subsystem: "tanks", Group: group + ": отладка", Unit: "кг", Kind: KindNumeric,
			Nominal: 0, Min: 0, Max: 400000,
			WarnLo: 0, WarnHi: 400000, CritLo: 0, CritHi: 400000,
			Step: 1000, Direct: true,
			Consequence: "Баланс масс перестаёт сходиться: расход больше не связан с остатком.",
			Reads:       reads + "mass",
			apply:       func(ov *Overrides, v float64) { pick(ov).MassDirect = Set(v) },
		},
		{
			ID: prefix + "pressure", Title: "Давление в баке напрямую",
			Subsystem: "tanks", Group: group + ": отладка", Unit: "кПа", Kind: KindNumeric,
			Nominal: 300, Min: 0, Max: 1000,
			WarnLo: 200, WarnHi: 400, CritLo: 100, CritHi: 600,
			Step: 10, scale: 1000, Direct: true,
			Consequence: "Давление перестаёт следовать из расхода газа наддува.",
			Reads:       reads + "pressure",
			apply:       func(ov *Overrides, v float64) { pick(ov).PressureDirect = Set(v) },
		},
		{
			ID: prefix + "temperature", Title: "Температура компонента напрямую",
			Subsystem: "tanks", Group: group + ": отладка", Unit: "К", Kind: KindNumeric,
			Nominal: 288, Min: 60, Max: 400,
			WarnLo: 70, WarnHi: 300, CritLo: 60, CritHi: 350,
			Step: 5, Direct: true,
			Consequence: "Плотность и давление насыщенных паров изменятся скачком.",
			Reads:       reads + "temperature",
			apply:       func(ov *Overrides, v float64) { pick(ov).TemperatureDirect = Set(v) },
		},
	}
}

func chamberParams() []Param {
	return []Param{
		{
			ID: "chamber.injector.fuel_area", Title: "Проходное сечение форсунок горючего",
			Subsystem: "chamber", Group: "Форсунки", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0.05, Max: 1.5,
			WarnLo: 0.95, WarnHi: 1.05, CritLo: 0.8, CritHi: 1.2, Step: 0.05,
			Consequence: "Засорение форсунок поднимает перепад, снижает расход и меняет O/F.",
			Reads:       "engine.fuelFlow",
			Affects: []string{"chamber.mixtureRatio", "chamber.temperature",
				"chamber.pressure", "engine.thrust", "nozzle.wallMargin"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Chamber.InjectorFuelAreaFactor = Set(v)
			},
		},
		{
			ID: "chamber.injector.ox_area", Title: "Проходное сечение форсунок окислителя",
			Subsystem: "chamber", Group: "Форсунки", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0.05, Max: 1.5,
			WarnLo: 0.95, WarnHi: 1.05, CritLo: 0.8, CritHi: 1.2, Step: 0.05,
			Consequence: "Засорение форсунок окислителя обогащает смесь по горючему.",
			Reads:       "engine.oxFlow",
			Affects: []string{"chamber.mixtureRatio", "chamber.temperature",
				"chamber.pressure", "engine.thrust"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Chamber.InjectorOxAreaFactor = Set(v)
			},
		},
		{
			ID: "chamber.cstar_efficiency", Title: "Множитель полноты сгорания",
			Subsystem: "chamber", Group: "Горение", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0.3, Max: 1.1,
			WarnLo: 0.97, WarnHi: 1.02, CritLo: 0.9, CritHi: 1.05, Step: 0.01,
			Consequence: "Неполнота сгорания снижает характеристическую скорость и удельный импульс.",
			Reads:       "chamber.cstarEfficiency",
			Affects:     []string{"chamber.pressure", "engine.thrust", "engine.specificImpulse"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Chamber.CStarEfficiencyFactor = Set(v)
			},
		},
		{
			ID: "chamber.stability_bias", Title: "Смещение запаса устойчивости",
			Subsystem: "chamber", Group: "Устойчивость", Unit: "", Kind: KindNumeric,
			Nominal: 0, Min: -1, Max: 0.5,
			WarnLo: -0.05, WarnHi: 0.5, CritLo: -0.2, CritHi: 0.5, Step: 0.05,
			Consequence: "Потеря запаса устойчивости переводит камеру в режим " +
				"высокочастотной неустойчивости и разрушает её за секунды.",
			Reads: "chamber.stabilityMargin",
			Affects: []string{"chamber.pressureRMS", "chamber.dominantFrequency",
				"nozzle.wallHeatFlux", "turbopump.vibration"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Chamber.StabilityMarginBias = Set(v)
			},
		},
		{
			ID: "chamber.oscillation", Title: "Множитель амплитуды пульсаций",
			Subsystem: "chamber", Group: "Устойчивость", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 50,
			WarnLo: 0, WarnHi: 2, CritLo: 0, CritHi: 10, Step: 0.5,
			Consequence: "Рост пульсаций нагружает форсуночную головку и стенки.",
			Reads:       "chamber.pressureRMS",
			Affects:     []string{"nozzle.wallHeatFlux", "engine.thrust"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Chamber.OscillationFactor = Set(v)
			},
		},
		{
			ID: "chamber.oscillation_freq", Title: "Частота пульсаций",
			Subsystem: "chamber", Group: "Устойчивость", Unit: "Гц", Kind: KindNumeric,
			Nominal: 0, Min: 0, Max: 250,
			WarnLo: 0, WarnHi: 250, CritLo: 0, CritHi: 250, Step: 5,
			Consequence: "Совпадение с собственной частотой конструкции разрушает камеру.",
			Reads:       "chamber.dominantFrequency",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Chamber.OscillationFrequency = Set(v)
			},
		},
		{
			ID: "chamber.temp_spread", Title: "Добавка температурной неоднородности",
			Subsystem: "chamber", Group: "Горение", Unit: "К", Kind: KindNumeric,
			Nominal: 0, Min: 0, Max: 1500,
			WarnLo: 0, WarnHi: 150, CritLo: 0, CritHi: 500, Step: 25,
			Consequence: "Неравномерное поле температур прожигает стенку локально.",
			Reads:       "chamber.temperatureSpread",
			Affects:     []string{"nozzle.wallMargin", "nozzle.throatErosion"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Chamber.TemperatureSpreadBias = Set(v)
			},
		},
		{
			ID: "chamber.heat_flux", Title: "Множитель теплового потока в стенки",
			Subsystem: "chamber", Group: "Горение", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 6,
			WarnLo: 0.8, WarnHi: 1.3, CritLo: 0.5, CritHi: 2, Step: 0.1,
			Consequence: "Рост потока перегревает стенку и ускоряет эрозию горловины.",
			Reads:       "nozzle.wallHeatFlux",
			Affects:     []string{"nozzle.wallMargin", "nozzle.throatErosion", "chamber.pressure"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Chamber.HeatFluxFactor = Set(v)
			},
		},
		{
			ID: "chamber.pressure", Title: "Давление в камере напрямую",
			Subsystem: "chamber", Group: "Отладка", Unit: "МПа", Kind: KindNumeric,
			Nominal: 9.7, Min: 0, Max: 25,
			WarnLo: 8, WarnHi: 10.5, CritLo: 5, CritHi: 14,
			Step: 0.2, scale: 1e6, Direct: true,
			Consequence: "Давление перестаёт следовать из расхода и площади горловины.",
			Reads:       "chamber.pressure",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Chamber.PressureDirect = Set(v)
			},
		},
		{
			ID: "chamber.temperature", Title: "Температура в камере напрямую",
			Subsystem: "chamber", Group: "Отладка", Unit: "К", Kind: KindNumeric,
			Nominal: 3500, Min: 300, Max: 5000,
			WarnLo: 3200, WarnHi: 3700, CritLo: 2500, CritHi: 4200,
			Step: 50, Direct: true,
			Consequence: "Температура перестаёт следовать из соотношения компонентов.",
			Reads:       "chamber.temperature",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Chamber.TemperatureDirect = Set(v)
			},
		},
	}
}

func nozzleParams() []Param {
	return []Param{
		{
			ID: "nozzle.cooling_flow", Title: "Множитель расхода охладителя",
			Subsystem: "nozzle", Group: "Охлаждение", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 2,
			WarnLo: 0.9, WarnHi: 1.2, CritLo: 0.6, CritHi: 1.5, Step: 0.05,
			Consequence: "Потеря охлаждения прожигает стенку камеры за десятки секунд.",
			Reads:       "nozzle.coolingFlow",
			Affects: []string{"nozzle.wallMargin", "nozzle.throatErosion",
				"nozzle.throatArea", "chamber.pressure", "engine.thrust"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Nozzle.CoolingFlowFactor = Set(v)
			},
			fail: func(ov *Overrides, f Failure) {
				if f == FailureStopped || f == FailureRuptured {
					ov.Engine.Nozzle.CoolingFlowFactor = Set(0)
				}
			},
		},
		{
			ID: "nozzle.cooling_conductance", Title: "Множитель теплоотдачи в тракт",
			Subsystem: "nozzle", Group: "Охлаждение", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 2,
			WarnLo: 0.9, WarnHi: 1.2, CritLo: 0.6, CritHi: 1.5, Step: 0.05,
			Consequence: "Засорение каналов оставляет расход прежним, но стенка перестаёт охлаждаться.",
			Reads:       "nozzle.wallMargin",
			Affects:     []string{"nozzle.throatErosion", "nozzle.throatArea"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Nozzle.CoolingConductanceFactor = Set(v)
			},
		},
		{
			ID: "nozzle.cooling_drop", Title: "Множитель потерь давления в тракте",
			Subsystem: "nozzle", Group: "Охлаждение", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 8,
			WarnLo: 0.8, WarnHi: 1.5, CritLo: 0.5, CritHi: 3, Step: 0.1,
			Consequence: "Рост сопротивления тракта снижает давление подачи горючего.",
			Reads:       "nozzle.coolingDrop",
			Affects:     []string{"engine.fuelFlow", "chamber.mixtureRatio", "chamber.pressure"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Nozzle.CoolingDropFactor = Set(v)
			},
		},
		{
			ID: "nozzle.coolant_inlet_temp", Title: "Температура охладителя на входе",
			Subsystem: "nozzle", Group: "Охлаждение", Unit: "К", Kind: KindNumeric,
			Nominal: 288, Min: 200, Max: 600,
			WarnLo: 250, WarnHi: 330, CritLo: 220, CritHi: 420, Step: 5,
			Consequence: "Горячий охладитель теряет способность отводить тепло.",
			Reads:       "nozzle.coolantInlet",
			Affects:     []string{"nozzle.wallMargin", "nozzle.coolantOutlet"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Nozzle.CoolantInletTemp = Set(v)
			},
		},
		{
			ID: "nozzle.erosion_rate", Title: "Множитель скорости эрозии горловины",
			Subsystem: "nozzle", Group: "Горловина", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 500,
			WarnLo: 0, WarnHi: 3, CritLo: 0, CritHi: 25, Step: 1,
			Consequence: "Разгар горловины необратимо снижает давление в камере и тягу.",
			Reads:       "nozzle.throatErosion",
			Affects:     []string{"nozzle.throatArea", "chamber.pressure", "engine.thrust"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Nozzle.ErosionRateFactor = Set(v)
			},
		},
		{
			ID: "nozzle.hot_spot", Title: "Местное ухудшение охлаждения",
			Subsystem: "nozzle", Group: "Стенка", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 1,
			WarnLo: 0.8, WarnHi: 1, CritLo: 0.4, CritHi: 1, Step: 0.05,
			Consequence: "Локальный перегрев прожигает стенку в одном сечении.",
			Reads:       "nozzle.wallMargin",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Nozzle.HotSpotFactor = Set(v)
				if ov.Engine.Nozzle.HotSpotStation == "" {
					ov.Engine.Nozzle.HotSpotStation = "throat"
				}
			},
		},
		{
			ID: "nozzle.throat_area", Title: "Площадь горловины напрямую",
			Subsystem: "nozzle", Group: "Отладка", Unit: "см²", Kind: KindNumeric,
			Nominal: 539, Min: 100, Max: 2000,
			WarnLo: 520, WarnHi: 570, CritLo: 450, CritHi: 700,
			Step: 5, scale: 1e-4, Direct: true,
			Consequence: "Площадь перестаёт следовать из эрозии; давление в камере " +
				"изменится скачком.",
			Reads: "nozzle.throatArea",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Nozzle.ThroatAreaDirect = Set(v)
			},
		},
	}
}

// controlParams описывает управляемые параметры рулевого тракта.
func controlParams() []Param {
	return []Param{
		{
			ID: "control.gimbal_limit", Title: "Предельный ход привода",
			Subsystem: "control", Group: "Рулевой тракт", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 2,
			WarnLo: 0.8, WarnHi: 1.2, CritLo: 0.4, CritHi: 1.5, Step: 0.05,
			Consequence: "Сокращение хода лишает автопилот запаса управляющего " +
				"момента: статически неустойчивый корпус разворачивается потоком.",
			Reads:   "control.gimbalDemand",
			Affects: []string{"control.authority", "vehicle.angleOfAttack"},
			apply: func(ov *Overrides, v float64) {
				ov.Control.GimbalLimitFactor = Set(v)
			},
			fail: func(ov *Overrides, f Failure) {
				switch f {
				case FailureFrozen, FailureStuck:
					ov.Control.Frozen = true
				case FailureStopped, FailureUnpowered, FailureUnresponsive:
					ov.Control.Dead = true
				}
			},
		},
		{
			ID: "control.gimbal_rate", Title: "Скорость перекладки привода",
			Subsystem: "control", Group: "Рулевой тракт", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0.02, Max: 3,
			WarnLo: 0.7, WarnHi: 1.5, CritLo: 0.3, CritHi: 2, Step: 0.05,
			Consequence: "Медленный привод запаздывает и раскачивает контур.",
			Reads:       "control.gimbalPitch",
			Affects:     []string{"vehicle.pitchRate", "vehicle.angleOfAttack"},
			apply: func(ov *Overrides, v float64) {
				ov.Control.GimbalRateFactor = Set(v)
			},
		},
		{
			ID: "control.gimbal_bias", Title: "Смещение привода",
			Subsystem: "control", Group: "Рулевой тракт", Unit: "°", Kind: KindNumeric,
			Nominal: 0, Min: -6, Max: 6,
			WarnLo: -0.2, WarnHi: 0.2, CritLo: -1, CritHi: 1,
			Step: 0.1, scale: math.Pi / 180,
			Consequence: "Заклинивший в отклонённом положении привод создаёт " +
				"непрерывный уводящий момент, который автопилот вынужден парировать.",
			Reads:   "control.gimbalPitch",
			Affects: []string{"control.authority", "vehicle.angleOfAttack"},
			apply: func(ov *Overrides, v float64) {
				ov.Control.GimbalBias = Set(v)
			},
		},
		{
			ID: "control.bandwidth", Title: "Полоса пропускания автопилота",
			Subsystem: "control", Group: "Автопилот", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0.05, Max: 4,
			WarnLo: 0.7, WarnHi: 1.5, CritLo: 0.3, CritHi: 2.5, Step: 0.05,
			Consequence: "Слишком широкая полоса раскачивает корпус, слишком " +
				"узкая не успевает парировать уводящий момент.",
			Reads:   "vehicle.pitchRate",
			Affects: []string{"vehicle.angleOfAttack", "control.gimbalPitch"},
			apply: func(ov *Overrides, v float64) {
				ov.Control.BandwidthFactor = Set(v)
			},
		},
		{
			ID: "control.damping", Title: "Демпфирование автопилота",
			Subsystem: "control", Group: "Автопилот", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 3,
			WarnLo: 0.6, WarnHi: 1.5, CritLo: 0.2, CritHi: 2, Step: 0.05,
			Consequence: "Без демпфирования контур входит в автоколебания.",
			Reads:       "vehicle.pitchRate",
			Affects:     []string{"vehicle.angleOfAttack"},
			apply: func(ov *Overrides, v float64) {
				ov.Control.DampingFactor = Set(v)
			},
		},
		{
			ID: "control.rcs", Title: "Момент двигателей ориентации",
			Subsystem: "control", Group: "Ориентация", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 3,
			WarnLo: 0.7, WarnHi: 1.5, CritLo: 0.3, CritHi: 2, Step: 0.05,
			Consequence: "Без двигателей ориентации ступень на пассивном участке " +
				"уходит в свободный дрейф и приходит к манёвру повёрнутой куда попало.",
			Reads:   "vehicle.pitchRate",
			Affects: []string{"vehicle.angleOfAttack"},
			apply: func(ov *Overrides, v float64) {
				ov.Control.RCSFactor = Set(v)
			},
			fail: func(ov *Overrides, f Failure) {
				if f == FailureStopped || f == FailureUnpowered {
					ov.Control.RCSFactor = Set(0)
				}
			},
		},
	}
}

func engineParams() []Param {
	return []Param{
		{
			ID: "engine.throttle", Title: "Уровень тяги",
			Subsystem: "engine", Group: "Команды", Unit: "", Kind: KindFraction,
			Nominal: 1, Min: 0, Max: 1.1,
			WarnLo: 0.4, WarnHi: 1, CritLo: 0.2, CritHi: 1.05, Step: 0.05,
			Consequence: "Глубокое дросселирование ниже предела гасит двигатель.",
			Reads:       "engine.thrust",
			Affects: []string{"turbopump.rpm", "chamber.pressure",
				"engine.specificImpulse", "vehicle.acceleration"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.ThrottleOverride = Set(v)
			},
			fail: func(ov *Overrides, f Failure) {
				if f == FailureStopped || f == FailureUnpowered {
					ov.Engine.ForceShutdown = true
				}
			},
		},
		{
			ID: "engine.mixture_trim", Title: "Поправка соотношения компонентов",
			Subsystem: "engine", Group: "Команды", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0.3, Max: 1.7,
			WarnLo: 0.95, WarnHi: 1.05, CritLo: 0.8, CritHi: 1.2, Step: 0.01,
			Consequence: "Сильное отклонение O/F перегревает стенки либо гасит камеру.",
			Reads:       "chamber.mixtureRatio",
			Affects: []string{"chamber.temperature", "chamber.cstarEfficiency",
				"engine.specificImpulse", "nozzle.wallMargin"},
			apply: func(ov *Overrides, v float64) {
				ov.Engine.MixtureTrimOverride = Set(v)
			},
		},
		{
			ID: "engine.sensor_noise", Title: "Множитель шума датчиков",
			Subsystem: "engine", Group: "Измерения", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 50,
			WarnLo: 0, WarnHi: 3, CritLo: 0, CritHi: 10, Step: 0.5,
			Consequence: "Зашумлённые показания не влияют на физику, но скрывают её от оператора.",
			Reads:       "chamber.pressure",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Sensors.NoiseFactor = Set(v)
			},
			fail: func(ov *Overrides, f Failure) {
				switch f {
				case FailureFrozen:
					ov.Engine.Sensors.Frozen = true
				case FailureBadSensor, FailureStopped:
					ov.Engine.Sensors.Dead = true
				}
			},
		},
		{
			ID: "engine.sensor_drift", Title: "Множитель дрейфа датчиков",
			Subsystem: "engine", Group: "Измерения", Unit: "", Kind: KindFactor,
			Nominal: 1, Min: 0, Max: 50,
			WarnLo: 0, WarnHi: 3, CritLo: 0, CritHi: 10, Step: 0.5,
			Consequence: "Дрейф нуля уводит показания от фактических значений.",
			Reads:       "chamber.pressure",
			apply: func(ov *Overrides, v float64) {
				ov.Engine.Sensors.DriftFactor = Set(v)
			},
		},
	}
}
