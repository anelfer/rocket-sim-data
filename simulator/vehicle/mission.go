package vehicle

import "sort"

// -----------------------------------------------------------------------------
// Задание на полёт.
//
// Носитель и задание — разные вещи. Одна и та же ракета может выводиться
// на круговую орбиту, а может лететь по трансатмосферной траектории: разгон
// почти до первой космической, но перицентр оставлен внутри атмосферы.
// Разница не в железе, а в том, где наведение выключает двигатели.
//
// Именно так летали испытания Starship: корабль набирает скорость, лишь
// немного не доходящую до орбитальной, и потому возвращается сам, без единого
// включения двигателя. Тормозной импульс ему не нужен — он и не может быть
// нужен, потому что орбиты, с которой надо сходить, не существует. Замкнутая
// орбита требует обратного: довыведения в апоцентре и отдельного манёвра
// на возвращение.
//
// Задание описывает только две высоты — верхнюю и нижнюю точку целевой
// траектории. Всё остальное модель считает сама.
// -----------------------------------------------------------------------------

// Mission — задание на полёт.
type Mission struct {
	// ID — устойчивый идентификатор для выбора.
	ID string `json:"id"`

	// Title — название для интерфейса.
	Title string `json:"title"`

	// Summary — чем это задание отличается от другого.
	Summary string `json:"summary"`

	// ApoapsisAltitude — целевая высота верхней точки, м.
	ApoapsisAltitude float64 `json:"apoapsisAltitude"`

	// PeriapsisAltitude — целевая высота нижней точки, м.
	//
	// Отрицательная величина означает, что траектория пересекает плотные слои:
	// корабль вернётся сам. Равная верхней — круговая орбита.
	PeriapsisAltitude float64 `json:"periapsisAltitude"`

	// Notes — оговорки, которые честно показать оператору до пуска.
	Notes []string `json:"notes,omitempty"`
}

// Suborbital сообщает, что задание не предполагает замкнутой орбиты.
func (m Mission) Suborbital() bool {
	return m.PeriapsisAltitude < atmosphereInterface
}

// atmosphereInterface — высота, ниже которой перицентр означает возвращение.
//
// Сто двадцать километров: на этой высоте торможение уже заметно, и орбита
// с таким перицентром не переживает и витка.
const atmosphereInterface = 120000

// Apply накладывает задание на конфигурацию носителя.
func (m Mission) Apply(cfg Config) Config {
	cfg.TargetOrbitAltitude = m.ApoapsisAltitude
	cfg.TargetPeriapsisAltitude = m.PeriapsisAltitude
	return cfg
}

// missions — реестр заданий.
var missions = map[string]Mission{
	"orbital": {
		ID:    "orbital",
		Title: "Орбитальное",
		Summary: "Выведение на круговую орбиту с довыведением в апоцентре. " +
			"Вернуться можно только тормозным импульсом: перицентр лежит " +
			"выше атмосферы, и сама траектория вниз не пойдёт.",
		ApoapsisAltitude:  400000,
		PeriapsisAltitude: 400000,
		Notes: []string{
			"Возвращение начинается по команде «Сход с орбиты».",
			"Довыведение требует запаса топлива в апоцентре.",
		},
	},
	"transatmospheric": {
		ID:    "transatmospheric",
		Title: "Трансатмосферное",
		Summary: "Разгон почти до первой космической, но перицентр оставлен " +
			"внутри атмосферы. Корабль проходит верхнюю точку и входит сам — " +
			"так летали испытательные полёты Starship.",
		ApoapsisAltitude:  200000,
		PeriapsisAltitude: -25000,
		Notes: []string{
			"Тормозной импульс не нужен и недоступен: орбиты нет.",
			"От выключения двигателей до входа проходит около получаса.",
		},
	},
}

// Missions возвращает список заданий по порядку.
func Missions() []Mission {
	out := make([]Mission, 0, len(missions))
	for _, m := range missions {
		out = append(out, m)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].ID < out[j].ID })
	return out
}

// MissionByID возвращает задание по идентификатору.
func MissionByID(id string) (Mission, bool) {
	m, ok := missions[id]
	return m, ok
}

// DefaultMission возвращает задание по умолчанию.
func DefaultMission() Mission { return missions["orbital"] }
