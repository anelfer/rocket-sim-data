package propulsion

import "math"

// -----------------------------------------------------------------------------
// Реестр параметров.
//
// Для каждой публикуемой величины здесь записаны единица измерения, номинальный,
// предупредительный и критический диапазоны, а также словесное описание того,
// из чего величина вычисляется и с чем связана.
//
// Реестр — не документация ради документации: из него генерируются пороги
// панелей Grafana, поэтому подпись на графике и поведение модели не могут
// разойтись. Если параметр меняет смысл, правится одно место.
// -----------------------------------------------------------------------------

// Severity — состояние параметра относительно допусков.
type Severity int

const (
	SeverityNormal Severity = iota
	SeverityWarning
	SeverityCritical
	SeverityUnknown
)

// String возвращает название состояния.
func (s Severity) String() string {
	switch s {
	case SeverityNormal:
		return "Normal"
	case SeverityWarning:
		return "Warning"
	case SeverityCritical:
		return "Critical"
	default:
		return "Unknown"
	}
}

// Range — диапазон значений. Границы включительные; бесконечность означает,
// что с этой стороны ограничения нет.
type Range struct {
	Min float64
	Max float64
}

// Contains сообщает, попадает ли значение в диапазон.
func (r Range) Contains(v float64) bool {
	return v >= r.Min && v <= r.Max
}

// Unbounded возвращает диапазон без ограничений.
func Unbounded() Range { return Range{Min: math.Inf(-1), Max: math.Inf(1)} }

// AtLeast возвращает диапазон «не меньше чем».
func AtLeast(v float64) Range { return Range{Min: v, Max: math.Inf(1)} }

// AtMost возвращает диапазон «не больше чем».
func AtMost(v float64) Range { return Range{Min: math.Inf(-1), Max: v} }

// Between возвращает диапазон между двумя значениями.
func Between(lo, hi float64) Range { return Range{Min: lo, Max: hi} }

// Parameter — описание одной публикуемой величины.
type Parameter struct {
	// Metric — имя метрики Prometheus.
	Metric string

	// Title — название на панели.
	Title string

	// Unit — единица измерения в обозначениях Grafana.
	Unit string

	// Initial — значение в начальный момент.
	Initial float64

	// Nominal — штатный диапазон.
	Nominal Range

	// Warning — предупредительный диапазон: выход за него означает отклонение,
	// но не аварию.
	Warning Range

	// Derivation — как величина вычисляется.
	Derivation string

	// Related — параметры, с которыми величина связана напрямую.
	Related []string

	// PanelType — рекомендуемый тип панели Grafana.
	PanelType string
}

// Classify возвращает состояние параметра для заданного значения.
//
// Внутри штатного диапазона — Normal, внутри предупредительного — Warning,
// за его пределами — Critical. Неопределённое значение даёт Unknown, чтобы
// пропуск данных не выглядел как авария.
func (p Parameter) Classify(v float64) Severity {
	if math.IsNaN(v) {
		return SeverityUnknown
	}
	if p.Nominal.Contains(v) {
		return SeverityNormal
	}
	if p.Warning.Contains(v) {
		return SeverityWarning
	}
	return SeverityCritical
}

// Registry — набор описаний параметров, доступных по имени метрики.
type Registry struct {
	parameters []Parameter
	byMetric   map[string]Parameter
}

// NewRegistry собирает реестр.
func NewRegistry(params []Parameter) *Registry {
	r := &Registry{
		parameters: params,
		byMetric:   make(map[string]Parameter, len(params)),
	}
	for _, p := range params {
		r.byMetric[p.Metric] = p
	}
	return r
}

// All возвращает все параметры в порядке объявления.
func (r *Registry) All() []Parameter { return r.parameters }

// Lookup возвращает описание параметра по имени метрики.
func (r *Registry) Lookup(metric string) (Parameter, bool) {
	p, ok := r.byMetric[metric]
	return p, ok
}

// -----------------------------------------------------------------------------
// Описания параметров двигательной установки
// -----------------------------------------------------------------------------

// ParameterRegistry возвращает реестр всех параметров двигательной установки.
func ParameterRegistry() *Registry {
	return NewRegistry(append(append(append(append(
		massParameters(),
		feedParameters()...),
		turbopumpParameters()...),
		combustionParameters()...),
		nozzleParameters()...))
}

func massParameters() []Parameter {
	return []Parameter{
		{
			Metric: "rocket_dry_mass_kg", Title: "Сухая масса конструкции", Unit: "kg",
			Initial: 154000, Nominal: AtLeast(0), Warning: AtLeast(0),
			Derivation: "конструкция ступени плюс всё, что она везёт; " +
				"уменьшается только при сбросе обтекателя и разделении",
			Related:   []string{"rocket_mass_kg"},
			PanelType: "stat",
		},
		{
			Metric: "rocket_payload_mass_kg", Title: "Масса полезной нагрузки", Unit: "kg",
			Initial: 15000, Nominal: AtLeast(0), Warning: AtLeast(0),
			Derivation: "постоянна на всём полёте",
			Related:    []string{"rocket_mass_kg"},
			PanelType:  "stat",
		},
		{
			Metric: "rocket_fuel_mass_kg", Title: "Масса горючего", Unit: "kg",
			Initial: 122500, Nominal: AtLeast(0), Warning: AtLeast(0),
			Derivation: "начальная заправка минус накопленный расход через " +
				"двигатели минус утечка; независимой величиной не является",
			Related:   []string{"engine_fuel_flow_kgps", "tank_fuel_level_percent", "rocket_mass_kg"},
			PanelType: "timeseries",
		},
		{
			Metric: "rocket_oxidizer_mass_kg", Title: "Масса окислителя", Unit: "kg",
			Initial: 288500, Nominal: AtLeast(0), Warning: AtLeast(0),
			Derivation: "начальная заправка минус накопленный расход минус утечка",
			Related:    []string{"engine_ox_flow_kgps", "tank_ox_level_percent", "rocket_mass_kg"},
			PanelType:  "timeseries",
		},
		{
			Metric: "tank_fuel_level_percent", Title: "Заполненность бака горючего", Unit: "percent",
			Initial: 100, Nominal: AtLeast(3), Warning: AtLeast(1),
			Derivation: "объём жидкости, делённый на объём бака; объём считается " +
				"по массе и плотности при текущей температуре",
			Related:   []string{"rocket_fuel_mass_kg", "tank_fuel_temperature_k"},
			PanelType: "gauge",
		},
		{
			Metric: "tank_ox_level_percent", Title: "Заполненность бака окислителя", Unit: "percent",
			Initial: 100, Nominal: AtLeast(3), Warning: AtLeast(1),
			Derivation: "то же для окислителя; из-за большего коэффициента " +
				"расширения сильнее зависит от температуры",
			Related:   []string{"rocket_oxidizer_mass_kg", "tank_ox_temperature_k"},
			PanelType: "gauge",
		},
		{
			Metric: "tank_fuel_temperature_k", Title: "Температура горючего", Unit: "kelvin",
			Initial: 288, Nominal: Between(270, 300), Warning: Between(255, 320),
			Derivation: "прогрев от окружающей среды и от возврата горячего " +
				"горючего из охлаждающего тракта",
			Related:   []string{"tank_fuel_density_kgm3", "cooling_outlet_temperature_k"},
			PanelType: "timeseries",
		},
		{
			Metric: "tank_ox_temperature_k", Title: "Температура окислителя", Unit: "kelvin",
			Initial: 90, Nominal: Between(85, 95), Warning: Between(80, 100),
			Derivation: "криогенный компонент постоянно подогревается извне; " +
				"рост температуры поднимает давление паров и снижает кавитационный запас",
			Related:   []string{"tank_ox_density_kgm3", "pump_ox_cavitation_margin_m"},
			PanelType: "timeseries",
		},
		{
			Metric: "tank_fuel_depletion_seconds", Title: "Время до выработки горючего", Unit: "s",
			Initial: math.Inf(1), Nominal: AtLeast(5), Warning: AtLeast(2),
			Derivation: "остаток массы, делённый на текущую скорость расходования " +
				"с учётом утечки",
			Related:   []string{"rocket_fuel_mass_kg", "engine_fuel_flow_kgps"},
			PanelType: "stat",
		},
		{
			Metric: "tank_ox_depletion_seconds", Title: "Время до выработки окислителя", Unit: "s",
			Initial: math.Inf(1), Nominal: AtLeast(5), Warning: AtLeast(2),
			Derivation: "остаток массы, делённый на текущую скорость расходования",
			Related:    []string{"rocket_oxidizer_mass_kg", "engine_ox_flow_kgps"},
			PanelType:  "stat",
		},
	}
}

func feedParameters() []Parameter {
	return []Parameter{
		{
			Metric: "tank_fuel_pressure_pa", Title: "Давление в баке горючего", Unit: "pressurepa",
			Initial: 300000, Nominal: Between(250000, 350000), Warning: Between(200000, 400000),
			Derivation: "давление газа наддува в свободном объёме; падает, если " +
				"расход наддува не поспевает за ростом свободного объёма",
			Related:   []string{"press_fuel_gas_pressure_pa", "pump_fuel_inlet_pressure_pa"},
			PanelType: "timeseries",
		},
		{
			Metric: "tank_ox_pressure_pa", Title: "Давление в баке окислителя", Unit: "pressurepa",
			Initial: 320000, Nominal: Between(270000, 370000), Warning: Between(220000, 420000),
			Derivation: "то же для бака окислителя",
			Related:    []string{"press_ox_gas_pressure_pa", "pump_ox_inlet_pressure_pa"},
			PanelType:  "timeseries",
		},
		{
			Metric: "pump_fuel_inlet_pressure_pa", Title: "Давление перед насосом горючего", Unit: "pressurepa",
			Initial: 300000, Nominal: AtLeast(200000), Warning: AtLeast(140000),
			Derivation: "давление в баке плюс гидростатический напор столба " +
				"жидкости при осевой перегрузке минус потери в трубопроводе",
			Related: []string{"tank_fuel_pressure_pa", "pump_fuel_cavitation_margin_m",
				"line_fuel_pressure_drop_pa"},
			PanelType: "timeseries",
		},
		{
			Metric: "pump_ox_inlet_pressure_pa", Title: "Давление перед насосом окислителя", Unit: "pressurepa",
			Initial: 320000, Nominal: AtLeast(220000), Warning: AtLeast(150000),
			Derivation: "то же для окислителя",
			Related:    []string{"tank_ox_pressure_pa", "pump_ox_cavitation_margin_m"},
			PanelType:  "timeseries",
		},
		{
			Metric: "pump_fuel_outlet_pressure_pa", Title: "Давление за насосом горючего", Unit: "pressurepa",
			Initial: 0, Nominal: Between(1.0e7, 1.6e7), Warning: Between(7.0e6, 1.8e7),
			Derivation: "давление на входе плюс напор насоса; напор растёт как " +
				"квадрат оборотов вала и падает при кавитации",
			Related: []string{"turbopump_rpm", "engine_fuel_flow_kgps",
				"injector_fuel_pressure_drop_pa"},
			PanelType: "timeseries",
		},
		{
			Metric: "pump_ox_outlet_pressure_pa", Title: "Давление за насосом окислителя", Unit: "pressurepa",
			Initial: 0, Nominal: Between(1.0e7, 1.6e7), Warning: Between(7.0e6, 1.8e7),
			Derivation: "то же для окислителя",
			Related:    []string{"turbopump_rpm", "engine_ox_flow_kgps"},
			PanelType:  "timeseries",
		},
		{
			Metric: "engine_fuel_flow_kgps", Title: "Расход горючего", Unit: "kgs",
			Initial: 0, Nominal: Between(30, 100), Warning: Between(20, 115),
			Derivation: "истечение через форсунки: ṁ = Cd·A·√(2ρ·Δp), где Δp — " +
				"перепад между давлением за насосом и давлением в камере",
			Related: []string{"pump_fuel_outlet_pressure_pa", "chamber_pressure_pa",
				"engine_mixture_ratio"},
			PanelType: "timeseries",
		},
		{
			Metric: "engine_ox_flow_kgps", Title: "Расход окислителя", Unit: "kgs",
			Initial: 0, Nominal: Between(70, 240), Warning: Between(50, 270),
			Derivation: "то же для окислителя",
			Related:    []string{"pump_ox_outlet_pressure_pa", "engine_mixture_ratio"},
			PanelType:  "timeseries",
		},
		{
			Metric: "engine_total_flow_kgps", Title: "Суммарный расход", Unit: "kgs",
			Initial: 0, Nominal: Between(100, 340), Warning: Between(70, 380),
			Derivation: "сумма расходов горючего и окислителя; независимой " +
				"величиной не является",
			Related:   []string{"engine_fuel_flow_kgps", "engine_ox_flow_kgps", "engine_thrust_newton"},
			PanelType: "timeseries",
		},
		{
			Metric: "engine_mixture_ratio", Title: "Соотношение компонентов O/F", Unit: "none",
			Initial: 0, Nominal: Between(2.15, 2.55), Warning: Between(1.9, 2.9),
			Derivation: "отношение расхода окислителя к расходу горючего; " +
				"отклонение от оптимума 2.34 снижает c*, меняет температуру " +
				"и нагружает стенки",
			Related: []string{"engine_fuel_flow_kgps", "engine_ox_flow_kgps",
				"combustion_cstar_mps", "chamber_wall_temperature_k"},
			PanelType: "timeseries",
		},
		{
			Metric: "injector_fuel_pressure_drop_pa", Title: "Перепад на форсунках горючего", Unit: "pressurepa",
			Initial: 0, Nominal: AtLeast(0.15), Warning: AtLeast(0.08),
			Derivation: "разность между давлением за насосом и давлением в камере; " +
				"отношение перепада к давлению камеры ниже 0.15 — классический " +
				"признак склонности к неустойчивому горению",
			Related:   []string{"pump_fuel_outlet_pressure_pa", "chamber_pressure_pa", "combustion_stability_margin"},
			PanelType: "timeseries",
		},
		{
			Metric: "press_fuel_gas_pressure_pa", Title: "Давление газа наддува горючего", Unit: "pressurepa",
			Initial: 2.1e7, Nominal: AtLeast(3.0e6), Warning: AtLeast(1.0e6),
			Derivation: "запас гелия в баллоне; расходуется на поддержание " +
				"давления в баке по мере роста свободного объёма",
			Related:   []string{"tank_fuel_pressure_pa", "press_fuel_gas_mass_kg"},
			PanelType: "timeseries",
		},
		{
			Metric: "press_fuel_gas_mass_kg", Title: "Остаток газа наддува горючего", Unit: "kg",
			Initial: 30, Nominal: AtLeast(3), Warning: AtLeast(1),
			Derivation: "начальный запас минус накопленный расход",
			Related:    []string{"press_fuel_flow_kgps"},
			PanelType:  "timeseries",
		},
		{
			Metric: "press_ox_gas_pressure_pa", Title: "Давление газа наддува окислителя", Unit: "pressurepa",
			Initial: 2.1e7, Nominal: AtLeast(3.0e6), Warning: AtLeast(1.0e6),
			Derivation: "то же для бака окислителя",
			Related:    []string{"tank_ox_pressure_pa"},
			PanelType:  "timeseries",
		},
		{
			Metric: "press_ox_gas_mass_kg", Title: "Остаток газа наддува окислителя", Unit: "kg",
			Initial: 70, Nominal: AtLeast(5), Warning: AtLeast(2),
			Derivation: "начальный запас минус накопленный расход",
			Related:    []string{"press_ox_flow_kgps"},
			PanelType:  "timeseries",
		},
		{
			Metric: "line_fuel_pressure_drop_pa", Title: "Потери давления в магистрали горючего", Unit: "pressurepa",
			Initial: 0, Nominal: AtMost(120000), Warning: AtMost(200000),
			Derivation: "квадратичная зависимость от расхода: Δp = k·ṁ²/ρ",
			Related:    []string{"engine_fuel_flow_kgps", "pump_fuel_inlet_pressure_pa"},
			PanelType:  "timeseries",
		},
		{
			Metric: "tank_fuel_slosh_amplitude_m", Title: "Колебания жидкости в баке горючего", Unit: "lengthm",
			Initial: 0, Nominal: AtMost(0.25), Warning: AtMost(0.6),
			Derivation: "затухающий осциллятор, возбуждаемый поперечным и угловым " +
				"ускорением; собственная частота зависит от уровня жидкости",
			Related:   []string{"tank_fuel_level_percent", "pump_fuel_inlet_pressure_pa"},
			PanelType: "timeseries",
		},
		{
			Metric: "tank_fuel_leak_kgps", Title: "Утечка горючего", Unit: "kgs",
			Initial: 0, Nominal: AtMost(0.01), Warning: AtMost(0.5),
			Derivation: "дополнительная потеря массы помимо расхода через двигатель; " +
				"при её наличии бак пустеет быстрее, чем требует тяга",
			Related:   []string{"rocket_fuel_mass_kg", "tank_fuel_depletion_seconds"},
			PanelType: "timeseries",
		},
	}
}

func turbopumpParameters() []Parameter {
	return []Parameter{
		{
			Metric: "turbopump_rpm", Title: "Обороты вала ТНА", Unit: "rotrpm",
			Initial: 0, Nominal: Between(28000, 38000), Warning: Between(20000, 42000),
			Derivation: "интегрирование момента на валу: обороты растут, пока " +
				"мощность турбины превышает мощность насосов и механические потери",
			Related: []string{"turbine_power_w", "pump_fuel_power_w", "pump_ox_power_w",
				"pump_fuel_outlet_pressure_pa"},
			PanelType: "timeseries",
		},
		{
			Metric: "turbopump_angular_acceleration_radps2", Title: "Угловое ускорение вала", Unit: "none",
			Initial: 0, Nominal: Between(-400, 900), Warning: Between(-900, 1600),
			Derivation: "избыточный момент, делённый на момент инерции ротора",
			Related:    []string{"turbopump_torque_nm", "turbopump_rpm"},
			PanelType:  "timeseries",
		},
		{
			Metric: "turbine_power_w", Title: "Мощность турбины", Unit: "watt",
			Initial: 0, Nominal: Between(3.0e6, 8.0e6), Warning: Between(1.5e6, 9.5e6),
			Derivation: "расход через газогенератор, умноженный на располагаемый " +
				"теплоперепад и КПД турбины",
			Related:   []string{"valve_gasgen_position", "turbopump_rpm", "turbine_temperature_k"},
			PanelType: "timeseries",
		},
		{
			Metric: "pump_fuel_power_w", Title: "Мощность насоса горючего", Unit: "watt",
			Initial: 0, Nominal: Between(5.0e5, 2.5e6), Warning: Between(0, 3.5e6),
			Derivation: "ṁ·Δp/(ρ·КПД); растёт как куб оборотов",
			Related:    []string{"turbopump_rpm", "engine_fuel_flow_kgps"},
			PanelType:  "timeseries",
		},
		{
			Metric: "pump_ox_power_w", Title: "Мощность насоса окислителя", Unit: "watt",
			Initial: 0, Nominal: Between(1.0e6, 4.5e6), Warning: Between(0, 6.0e6),
			Derivation: "то же для окислителя; выше из-за большего расхода",
			Related:    []string{"turbopump_rpm", "engine_ox_flow_kgps"},
			PanelType:  "timeseries",
		},
		{
			Metric: "turbopump_torque_nm", Title: "Крутящий момент на валу", Unit: "none",
			Initial: 0, Nominal: Between(-300, 2500), Warning: Between(-900, 3500),
			Derivation: "разность мощности турбины и потребляемой мощности, " +
				"делённая на угловую скорость",
			Related:   []string{"turbine_power_w", "pump_fuel_power_w", "pump_ox_power_w"},
			PanelType: "timeseries",
		},
		{
			Metric: "pump_fuel_cavitation_margin_m", Title: "Кавитационный запас насоса горючего", Unit: "lengthm",
			Initial: 0, Nominal: AtLeast(12), Warning: AtLeast(4),
			Derivation: "превышение располагаемого кавитационного запаса над " +
				"требуемым; располагаемый считается от давления на входе за " +
				"вычетом давления насыщенных паров, требуемый растёт как квадрат оборотов",
			Related: []string{"pump_fuel_inlet_pressure_pa", "tank_fuel_temperature_k",
				"turbopump_rpm", "turbopump_vibration_g"},
			PanelType: "timeseries",
		},
		{
			Metric: "pump_ox_cavitation_margin_m", Title: "Кавитационный запас насоса окислителя", Unit: "lengthm",
			Initial: 0, Nominal: AtLeast(10), Warning: AtLeast(3),
			Derivation: "то же для окислителя; у криогенного компонента запас " +
				"чувствительнее к прогреву",
			Related:   []string{"pump_ox_inlet_pressure_pa", "tank_ox_temperature_k"},
			PanelType: "timeseries",
		},
		{
			Metric: "turbopump_vibration_g", Title: "Вибрация подшипников", Unit: "accG",
			Initial: 0, Nominal: AtMost(6), Warning: AtMost(14),
			Derivation: "остаточный дисбаланс ротора, растущий как квадрат оборотов, " +
				"плюс резкий вклад кавитации при исчерпании запаса",
			Related: []string{"turbopump_rpm", "pump_fuel_cavitation_margin_m",
				"turbopump_bearing_temperature_k"},
			PanelType: "timeseries",
		},
		{
			Metric: "turbopump_bearing_temperature_k", Title: "Температура подшипников", Unit: "kelvin",
			Initial: 288, Nominal: AtMost(420), Warning: AtMost(520),
			Derivation: "тепловыделение от трения, пропорциональное оборотам и " +
				"вибрации, минус отвод тепла компонентом",
			Related:   []string{"turbopump_rpm", "turbopump_vibration_g"},
			PanelType: "timeseries",
		},
		{
			Metric: "turbopump_axial_displacement_mm", Title: "Осевое смещение ротора", Unit: "lengthmm",
			Initial: 0, Nominal: AtMost(0.25), Warning: AtMost(0.6),
			Derivation: "неуравновешенное осевое усилие от перепада давления " +
				"на крыльчатках, отнесённое к жёсткости упорного подшипника",
			Related:   []string{"pump_ox_outlet_pressure_pa", "turbopump_vibration_g"},
			PanelType: "timeseries",
		},
		{
			Metric: "valve_fuel_position", Title: "Положение клапана горючего", Unit: "percentunit",
			Initial: 0, Nominal: Between(0, 1), Warning: Between(0, 1),
			Derivation: "команда системы управления, отработанная с конечной " +
				"скоростью перекладки",
			Related:   []string{"engine_fuel_flow_kgps", "engine_mixture_ratio"},
			PanelType: "timeseries",
		},
		{
			Metric: "valve_ox_position", Title: "Положение клапана окислителя", Unit: "percentunit",
			Initial: 0, Nominal: Between(0, 1), Warning: Between(0, 1),
			Derivation: "то же для окислителя",
			Related:    []string{"engine_ox_flow_kgps", "engine_mixture_ratio"},
			PanelType:  "timeseries",
		},
		{
			Metric: "valve_gasgen_position", Title: "Положение клапана газогенератора", Unit: "percentunit",
			Initial: 0, Nominal: Between(0, 1), Warning: Between(0, 1),
			Derivation: "задаёт расход через турбину и тем самым обороты ТНА; " +
				"начало причинной цепочки управления тягой",
			Related:   []string{"turbine_power_w", "turbopump_rpm"},
			PanelType: "timeseries",
		},
		{
			Metric: "valve_bypass_position", Title: "Положение байпасного клапана", Unit: "percentunit",
			Initial: 0, Nominal: Between(0, 0.6), Warning: Between(0, 0.9),
			Derivation: "перепуск части рабочего тела мимо турбины; используется " +
				"для ограничения оборотов",
			Related:   []string{"turbine_power_w", "turbopump_rpm"},
			PanelType: "timeseries",
		},
	}
}

func combustionParameters() []Parameter {
	return []Parameter{
		{
			Metric: "chamber_pressure_pa", Title: "Давление в камере сгорания", Unit: "pressurepa",
			Initial: 0, Nominal: Between(8.5e6, 1.05e7), Warning: Between(6.0e6, 1.2e7),
			Derivation: "из условия запирания горловины: p = ṁ·c*/A_гор; " +
				"складывается из среднего значения, пульсаций и шума датчика",
			Related: []string{"engine_total_flow_kgps", "combustion_cstar_mps",
				"nozzle_throat_area_m2", "engine_thrust_newton"},
			PanelType: "timeseries",
		},
		{
			Metric: "chamber_pressure_mean_pa", Title: "Среднее давление в камере", Unit: "pressurepa",
			Initial: 0, Nominal: Between(8.5e6, 1.05e7), Warning: Between(6.0e6, 1.2e7),
			Derivation: "давление без пульсаций и шума",
			Related:    []string{"chamber_pressure_pa"},
			PanelType:  "timeseries",
		},
		{
			Metric: "combustion_cstar_mps", Title: "Характеристическая скорость c*", Unit: "velocityms",
			Initial: 0, Nominal: Between(1700, 1800), Warning: Between(1550, 1830),
			Derivation: "идеальное значение для текущего соотношения компонентов, " +
				"умноженное на полноту сгорания",
			Related: []string{"engine_mixture_ratio", "combustion_cstar_efficiency",
				"chamber_pressure_pa"},
			PanelType: "timeseries",
		},
		{
			Metric: "combustion_cstar_efficiency", Title: "Полнота сгорания c*", Unit: "percentunit",
			Initial: 0, Nominal: AtLeast(0.96), Warning: AtLeast(0.9),
			Derivation: "качество распыла и смешения; падает при отклонении " +
				"соотношения компонентов, малом перепаде на форсунках и " +
				"развитии пульсаций",
			Related: []string{"engine_mixture_ratio", "injector_fuel_pressure_drop_pa",
				"combustion_pressure_rms_pa"},
			PanelType: "gauge",
		},
		{
			Metric: "combustion_efficiency", Title: "Эффективность горения", Unit: "percentunit",
			Initial: 0, Nominal: AtLeast(0.97), Warning: AtLeast(0.92),
			Derivation: "доля химической энергии, перешедшей в тепловую",
			Related:    []string{"engine_mixture_ratio", "chamber_temperature_k"},
			PanelType:  "gauge",
		},
		{
			Metric: "nozzle_thrust_coefficient", Title: "Коэффициент тяги Cf", Unit: "none",
			Initial: 0, Nominal: Between(1.45, 1.85), Warning: Between(1.2, 1.95),
			Derivation: "из газодинамики сопла по отношению давлений в камере, " +
				"на срезе и снаружи, и по степени расширения",
			Related: []string{"chamber_pressure_pa", "nozzle_exit_pressure_pa",
				"engine_thrust_newton"},
			PanelType: "timeseries",
		},
		{
			Metric: "chamber_temperature_k", Title: "Температура в камере", Unit: "kelvin",
			Initial: 288, Nominal: Between(3200, 3600), Warning: Between(2900, 3750),
			Derivation: "определяется топливной парой и соотношением компонентов; " +
				"максимум температуры смещён к стехиометрии, поэтому двигатель " +
				"работает на богатой смеси",
			Related:   []string{"engine_mixture_ratio", "chamber_wall_heat_flux_w_m2"},
			PanelType: "timeseries",
		},
		{
			Metric: "combustion_mixture_deviation", Title: "Отклонение O/F от номинала", Unit: "percentunit",
			Initial: 0, Nominal: Between(-0.06, 0.06), Warning: Between(-0.15, 0.15),
			Derivation: "относительное отклонение фактического соотношения " +
				"компонентов от оптимального",
			Related:   []string{"engine_mixture_ratio", "combustion_cstar_mps"},
			PanelType: "timeseries",
		},
		{
			Metric: "combustion_pressure_rms_pa", Title: "Среднеквадратичные пульсации давления", Unit: "pressurepa",
			Initial: 0, Nominal: AtMost(2.0e5), Warning: AtMost(5.0e5),
			Derivation: "среднеквадратичное отклонение давления камеры в скользящем " +
				"окне; штатно около 1–2 % от среднего давления",
			Related: []string{"chamber_pressure_pa", "combustion_stability_margin",
				"combustion_dominant_frequency_hz"},
			PanelType: "timeseries",
		},
		{
			Metric: "combustion_pressure_peak_pa", Title: "Амплитуда пульсаций давления", Unit: "pressurepa",
			Initial: 0, Nominal: AtMost(4.0e5), Warning: AtMost(1.0e6),
			Derivation: "максимальное отклонение от среднего в окне наблюдения",
			Related:    []string{"combustion_pressure_rms_pa"},
			PanelType:  "timeseries",
		},
		{
			Metric: "combustion_dominant_frequency_hz", Title: "Доминирующая частота пульсаций", Unit: "hertz",
			Initial: 0, Nominal: Unbounded(), Warning: Unbounded(),
			Derivation: "положение максимума спектра давления камеры; при развитии " +
				"неустойчивости в спектре появляется выраженный пик",
			Related:   []string{"combustion_pressure_rms_pa", "combustion_state"},
			PanelType: "timeseries",
		},
		{
			Metric: "combustion_stability_margin", Title: "Запас устойчивости горения", Unit: "percentunit",
			Initial: 1, Nominal: AtLeast(0.35), Warning: AtLeast(0.12),
			Derivation: "снижается при малом перепаде на форсунках относительно " +
				"давления камеры, отклонении соотношения компонентов и " +
				"пульсациях расхода из-за кавитации",
			Related: []string{"injector_fuel_pressure_drop_pa", "combustion_mixture_deviation",
				"pump_fuel_cavitation_margin_m", "combustion_state"},
			PanelType: "gauge",
		},
		{
			Metric: "combustion_state", Title: "Состояние горения", Unit: "none",
			Initial: 0, Nominal: AtMost(0), Warning: AtMost(1),
			Derivation: "0 — Stable, 1 — Marginal, 2 — Unstable; определяется " +
				"запасом устойчивости",
			Related:   []string{"combustion_stability_margin"},
			PanelType: "stat",
		},
		{
			Metric: "chamber_temperature_nonuniformity", Title: "Температурная неоднородность камеры", Unit: "percentunit",
			Initial: 0, Nominal: AtMost(0.06), Warning: AtMost(0.15),
			Derivation: "неравномерность смешения; растёт при отклонении " +
				"соотношения компонентов и падении перепада на форсунках",
			Related:   []string{"combustion_cstar_efficiency", "chamber_wall_heat_flux_w_m2"},
			PanelType: "timeseries",
		},
	}
}

func nozzleParameters() []Parameter {
	return []Parameter{
		{
			Metric: "chamber_wall_heat_flux_w_m2", Title: "Тепловой поток в стенку камеры", Unit: "watt",
			Initial: 0, Nominal: AtMost(6.0e7), Warning: AtMost(9.0e7),
			Derivation: "по соотношению Бартца: растёт с давлением камеры и " +
				"температурой газа, максимален в районе критического сечения",
			Related: []string{"chamber_pressure_pa", "chamber_temperature_k",
				"nozzle_wall_temperature_k"},
			PanelType: "timeseries",
		},
		{
			Metric: "nozzle_wall_temperature_k", Title: "Температура стенки по участкам", Unit: "kelvin",
			Initial: 288, Nominal: AtMost(850), Warning: AtMost(1050),
			Derivation: "баланс между потоком от газа и отводом в охлаждающий " +
				"тракт; участок критического сечения нагружен сильнее всех",
			Related: []string{"chamber_wall_heat_flux_w_m2", "cooling_flow_kgps",
				"nozzle_wall_margin_k"},
			PanelType: "timeseries",
		},
		{
			Metric: "nozzle_wall_margin_k", Title: "Запас до критической температуры", Unit: "kelvin",
			Initial: 700, Nominal: AtLeast(150), Warning: AtLeast(50),
			Derivation: "разность между предельной температурой материала и " +
				"фактической температурой самого горячего участка",
			Related:   []string{"nozzle_wall_temperature_k", "cooling_flow_kgps"},
			PanelType: "gauge",
		},
		{
			Metric: "cooling_flow_kgps", Title: "Расход в охлаждающем тракте", Unit: "kgs",
			Initial: 0, Nominal: AtLeast(25), Warning: AtLeast(15),
			Derivation: "часть расхода горючего, направляемая в рубашку " +
				"охлаждения перед подачей в форсунки",
			Related: []string{"engine_fuel_flow_kgps", "nozzle_wall_temperature_k",
				"cooling_outlet_temperature_k"},
			PanelType: "timeseries",
		},
		{
			Metric: "cooling_inlet_temperature_k", Title: "Температура охладителя на входе", Unit: "kelvin",
			Initial: 288, Nominal: Between(270, 310), Warning: Between(255, 330),
			Derivation: "равна температуре горючего в баке",
			Related:    []string{"tank_fuel_temperature_k"},
			PanelType:  "timeseries",
		},
		{
			Metric: "cooling_outlet_temperature_k", Title: "Температура охладителя на выходе", Unit: "kelvin",
			Initial: 288, Nominal: AtMost(560), Warning: AtMost(650),
			Derivation: "вход плюс отношение отведённой мощности к произведению " +
				"расхода и теплоёмкости",
			Related: []string{"cooling_flow_kgps", "chamber_wall_heat_flux_w_m2",
				"cooling_effectiveness"},
			PanelType: "timeseries",
		},
		{
			Metric: "cooling_pressure_drop_pa", Title: "Перепад давления в охлаждающем тракте", Unit: "pressurepa",
			Initial: 0, Nominal: AtMost(2.5e6), Warning: AtMost(4.0e6),
			Derivation: "квадратичен по расходу; вычитается из давления, " +
				"доступного на форсунках",
			Related:   []string{"cooling_flow_kgps", "injector_fuel_pressure_drop_pa"},
			PanelType: "timeseries",
		},
		{
			Metric: "cooling_effectiveness", Title: "Эффективность охлаждения", Unit: "percentunit",
			Initial: 1, Nominal: AtLeast(0.85), Warning: AtLeast(0.7),
			Derivation: "отношение фактически отведённого теплового потока " +
				"к потребному; падает при снижении расхода охладителя",
			Related:   []string{"cooling_flow_kgps", "nozzle_wall_temperature_k"},
			PanelType: "gauge",
		},
		{
			Metric: "nozzle_throat_area_m2", Title: "Площадь критического сечения", Unit: "areaM2",
			Initial: 0, Nominal: Unbounded(), Warning: Unbounded(),
			Derivation: "начальная площадь плюс накопленная эрозия; рост площади " +
				"снижает давление в камере и меняет тягу даже при неизменных клапанах",
			Related: []string{"nozzle_throat_erosion_mm", "chamber_pressure_pa",
				"nozzle_thrust_coefficient"},
			PanelType: "timeseries",
		},
		{
			Metric: "nozzle_throat_erosion_mm", Title: "Эрозия критического сечения", Unit: "lengthmm",
			Initial: 0, Nominal: AtMost(0.5), Warning: AtMost(1.5),
			Derivation: "скорость уноса материала резко растёт с температурой " +
				"стенки и тепловым потоком; интегрируется по времени работы",
			Related:   []string{"nozzle_wall_temperature_k", "nozzle_throat_area_m2"},
			PanelType: "timeseries",
		},
		{
			Metric: "nozzle_expansion_ratio", Title: "Степень расширения сопла", Unit: "none",
			Initial: 16, Nominal: Unbounded(), Warning: Unbounded(),
			Derivation: "отношение площади среза к площади горловины; уменьшается " +
				"по мере эрозии горловины",
			Related:   []string{"nozzle_throat_area_m2", "nozzle_exit_pressure_pa"},
			PanelType: "timeseries",
		},
		{
			Metric: "nozzle_exit_pressure_pa", Title: "Давление на срезе сопла", Unit: "pressurepa",
			Initial: 0, Nominal: Between(4.0e4, 1.0e5), Warning: Between(2.0e4, 1.4e5),
			Derivation: "из изоэнтропического расширения по степени расширения " +
				"и давлению в камере",
			Related: []string{"chamber_pressure_pa", "nozzle_expansion_ratio",
				"nozzle_expansion_regime"},
			PanelType: "timeseries",
		},
		{
			Metric: "nozzle_exhaust_velocity_mps", Title: "Скорость истечения", Unit: "velocityms",
			Initial: 0, Nominal: Between(2600, 3400), Warning: Between(2200, 3600),
			Derivation: "произведение характеристической скорости на коэффициент тяги",
			Related:    []string{"combustion_cstar_mps", "nozzle_thrust_coefficient", "engine_isp"},
			PanelType:  "timeseries",
		},
		{
			Metric: "nozzle_expansion_regime", Title: "Режим расширения струи", Unit: "none",
			Initial: 0, Nominal: Between(0.6, 1.6), Warning: Between(0.35, 2.5),
			Derivation: "отношение давления на срезе к атмосферному: меньше " +
				"единицы — перерасширение, больше — недорасширение",
			Related:   []string{"nozzle_exit_pressure_pa", "engine_thrust_newton"},
			PanelType: "timeseries",
		},
	}
}
