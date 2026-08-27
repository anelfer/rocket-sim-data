package vehicle

import (
	"sort"

	"rocketTelemetrySim/simulator/propulsion"
)

// -----------------------------------------------------------------------------
// Профили носителей.
//
// Раньше конфигурация была одна и вызывалась из полудюжины мест напрямую.
// Реестр позволяет держать несколько носителей рядом и переключаться между
// ними, не трогая ни модель, ни интерфейс: профиль описывает только исходные
// данные, а вся физика остаётся общей.
//
// Профиль обязан быть цельным: массы, тяга, геометрия и параметры двигателя
// должны соответствовать друг другу. Смешивать, скажем, керосиновый двигатель
// с метановыми баками бессмысленно — модель посчитает, но результат не будет
// значить ничего.
// -----------------------------------------------------------------------------

// EngineGroup — группа однотипных двигателей в составе ступени.
//
// Описание состава двигательной установки принадлежит носителю, а не сборщику:
// у сверхтяжёлой ракеты на второй ступени стоят камеры двух разных типов,
// и знать об этом должен профиль.
type EngineGroup struct {
	// Suffix — обозначение группы в идентификаторе, например "V"
	// для вакуумных камер. Пустое значение означает основную группу.
	Suffix string

	Count  int
	Config propulsion.EngineConfig
}

// Profile — описание носителя для выбора оператором.
type Profile struct {
	// ID — устойчивый идентификатор, по которому профиль выбирается.
	ID string `json:"id"`

	// Title — название для интерфейса.
	Title string `json:"title"`

	// Summary — короткая справка: чем этот носитель отличается.
	Summary string `json:"summary"`

	// Каталожные данные, показываемые до запуска.
	FirstStageEngines  int     `json:"firstStageEngines"`
	SecondStageEngines int     `json:"secondStageEngines"`
	Propellants        string  `json:"propellants"`
	Cycle              string  `json:"cycle"`
	LiftoffMass        float64 `json:"liftoffMass"` // т
	LiftoffThrust      float64 `json:"liftoffThrust"`
	Payload            float64 `json:"payload"` // т

	// Notes — оговорки о том, что в профиле упрощено.
	Notes []string `json:"notes,omitempty"`

	// build возвращает конфигурацию носителя.
	build func() Config
}

// Config возвращает конфигурацию носителя этого профиля.
func (p Profile) Config() Config { return p.build() }

// profiles — реестр доступных носителей.
var profiles = map[string]Profile{
	"falcon9": {
		ID:    "falcon9",
		Title: "Falcon 9 Block 5",
		Summary: "Двухступенчатый носитель среднего класса на керосине " +
			"и жидком кислороде. Двигатели открытой схемы с газогенератором.",
		FirstStageEngines:  9,
		SecondStageEngines: 1,
		Propellants:        "РГ-1 / жидкий кислород",
		Cycle:              "открытая, с газогенератором",
		LiftoffMass:        562,
		LiftoffThrust:      7.6,
		Payload:            12,
		Notes: []string{
			"Программа тангажа первой ступени задана таблицей и подобрана " +
				"под орбиту 400 км: на других высотах выведение будет хуже.",
			"Возврат и посадка первой ступени не моделируются — после " +
				"разделения она летит баллистически и разрушается.",
		},
		build: Falcon9Config,
	},

	"starship": {
		ID:    "starship",
		Title: "Starship / Super Heavy (V3)",
		Summary: "Сверхтяжёлый носитель на метане и жидком кислороде. " +
			"Тридцать три двигателя на первой ступени, шесть на второй, " +
			"замкнутая схема с полной газификацией компонентов. Параметры " +
			"третьего блока: высота 124 м, давление в камере 350 бар.",
		FirstStageEngines:  33,
		SecondStageEngines: 6,
		Propellants:        "жидкий метан / жидкий кислород",
		Cycle:              "замкнутая, полнопоточная",
		LiftoffMass:        5675,
		LiftoffThrust:      79.7,
		Payload:            100,
		Notes: []string{
			"Схема двигателя упрощена: турбина по-прежнему приводится отбором " +
				"рабочего тела, но в замкнутом контуре он возвращается в камеру, " +
				"а не выбрасывается за борт. Двух отдельных дожигательных " +
				"газогенераторов, как у настоящего двигателя, модель не знает.",
			"На второй ступени три двигателя с атмосферными соплами и три " +
				"с вакуумными: именно на атмосферных выполняется посадка.",
			"Первая ступень активно возвращается к площадке на резерве " +
				"топлива (разворотный импульс, спуск под решётчатыми рулями, " +
				"посадочный импульс); перехват стартовым сооружением не " +
				"моделируется — приводнение и последующий завал. Заправка " +
				"на орбите и возврат второй ступени не моделируются.",
			"Программа тангажа подобрана под Falcon 9 и для этой ракеты " +
				"неоптимальна — выведение будет с повышенными потерями.",
			"Открытые данные по третьему блоку расходятся: стартовую массу " +
				"называют от 5250 до 5375 т без резерва на возврат ступени, " +
				"суммарную тягу — от 80.8 до 90.6 МН. Взят внутренне " +
				"согласованный набор, тяга считается моделью из давления " +
				"и геометрии сопла; заправка увеличена на резерв возврата " +
				"первой ступени (см. FuelReserve, config.go).",
		},
		build: StarshipConfig,
	},
}

// Profiles возвращает список профилей в устойчивом порядке.
func Profiles() []Profile {
	out := make([]Profile, 0, len(profiles))
	for _, p := range profiles {
		out = append(out, p)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].LiftoffMass < out[j].LiftoffMass })
	return out
}

// ProfileByID возвращает профиль по идентификатору.
func ProfileByID(id string) (Profile, bool) {
	p, ok := profiles[id]
	return p, ok
}

// DefaultProfile возвращает профиль, используемый по умолчанию.
func DefaultProfile() Profile { return profiles["falcon9"] }

// -----------------------------------------------------------------------------
// Состав двигательных установок ступеней
// -----------------------------------------------------------------------------

// Профиль носителя определяет не только массы и тягу, но и то, какие двигатели
// стоят на ступенях. Раньше это было зашито в двух местах прямыми вызовами
// конфигураций Merlin, и добавить носитель на другом топливе было нельзя.
//
// Различить профили по названию — приём грубый, но честный: конфигурация
// носителя описывает изделие целиком, и подмешивать в неё ссылки на пакет
// двигателей значило бы связать описание с реализацией.

// IsMethalox сообщает, что носитель работает на метане.
func IsMethalox(cfg Config) bool {
	return cfg.FirstStage.MixtureRatio > 3.0
}

// FirstStageEngineConfig возвращает конфигурацию двигателя первой ступени.
func FirstStageEngineConfig(cfg Config) propulsion.EngineConfig {
	if IsMethalox(cfg) {
		return propulsion.RaptorConfig()
	}
	return propulsion.Merlin1DConfig()
}

// SecondStagePrimaryConfig возвращает конфигурацию основной группы двигателей
// второй ступени.
//
// У носителя на метане вторая ступень несёт двигатели двух типов: атмосферные,
// которыми выполняется посадка, и вакуумные — ради удельного импульса. Основной
// считается атмосферная группа.
func SecondStagePrimaryConfig(cfg Config) propulsion.EngineConfig {
	if IsMethalox(cfg) {
		return propulsion.RaptorConfig()
	}
	return propulsion.MerlinVacuumConfig()
}

// SecondStagePrimaryCount возвращает число двигателей основной группы.
func SecondStagePrimaryCount(cfg Config) int {
	if IsMethalox(cfg) {
		return 3
	}
	return int(cfg.SecondStage.EngineCount)
}

// SecondStageGroups возвращает дополнительные группы двигателей второй ступени.
func SecondStageGroups(cfg Config) []EngineGroup {
	if !IsMethalox(cfg) {
		return nil
	}
	return []EngineGroup{{
		Suffix: "V",
		Count:  int(cfg.SecondStage.EngineCount) - 3,
		Config: propulsion.RaptorVacuumConfig(),
	}}
}
