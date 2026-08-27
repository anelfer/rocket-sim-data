package vehicle

import (
	"fmt"
	"math"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/propulsion"
)

// -----------------------------------------------------------------------------
// Геометрия носителя для развёртки.
//
// Интерфейс рисует ракету в разрезе, и ему нужны размеры: где кончается бак
// окислителя, какой высоты юбка, по каким кольцам расставлены двигатели и
// какого диаметра у них сопла. Всё это выводится здесь, из той же конфигурации,
// по которой собирается физическая модель, — чтобы картинка не разошлась
// с расчётом.
//
// Живые величины сюда не попадают: уровни в баках, режимы двигателей и углы
// качания приходят телеметрией. Развёртка — это только обводы и привязки,
// по которым телеметрия раскладывается на места.
// -----------------------------------------------------------------------------

// Layout — обводы носителя.
type Layout struct {
	Title       string  `json:"title"`
	Diameter    float64 `json:"diameter"`    // м
	TotalLength float64 `json:"totalLength"` // м

	// Flaps сообщает, что у верхней ступени есть аэродинамические поверхности.
	//
	// Нужно для рисовки: плавники стоят на корабле с самого старта, просто
	// прижаты к борту. Телеметрия про них молчит до разделения — до него ими
	// никто не управляет, — и без этого признака на выведении корабль
	// рисовался бы голым.
	Flaps bool `json:"flaps"`

	// FlapMounts — посадочные места плавников: где панель сидит на обводе
	// и на какой станции по длине.
	//
	// Живого состояния здесь нет, только привязки: углы отклонения приходят
	// телеметрией. Отдаются они затем, чтобы рисовка перестала выдумывать
	// расстановку — прежде она держала свои числа, и на картинке плавники
	// стояли не там, где их считает модель.
	FlapMounts []FlapMountLayout `json:"flapMounts,omitempty"`

	// Stages идут сверху вниз: сначала верхняя ступень, потом нижняя.
	// Так же они и рисуются.
	Stages []StageLayout `json:"stages"`
}

// FlapMountLayout — посадочное место плавника для рисовки.
type FlapMountLayout struct {
	Name string `json:"name"`

	// Station — расстояние от среза сопел корабля, м.
	Station float64 `json:"station"`

	// Azimuth — положение по обводу, градусы от спины. Спина — подветренная
	// сторона, брюхо с плитками — противоположная. Положительный угол уводит
	// на правый борт.
	Azimuth float64 `json:"azimuth"`

	Area  float64 `json:"area"`  // м²
	Limit float64 `json:"limit"` // ход привода, градусы

	// Span и Chord — размах и средняя хорда панели, м.
	Span  float64 `json:"span"`
	Chord float64 `json:"chord"`

	// StowHalf — половина дуги обвода, закрытой прижатой панелью, градусы.
	// По ней рисовка кладёт панель по хорде и поднимает ось навески над
	// обшивкой ровно настолько, чтобы концы панели легли на борт.
	StowHalf float64 `json:"stowHalf"`
}

// StageLayout — обводы одной ступени.
type StageLayout struct {
	Index  int     `json:"index"`  // 1 — первая ступень
	Name   string  `json:"name"`   // название из конфигурации
	Prefix string  `json:"prefix"` // "S1", "S2" — префикс идентификаторов
	Length float64 `json:"length"` // м

	Sections []SectionLayout `json:"sections"`
	Engines  []EngineLayout  `json:"engines"`
}

// SectionLayout — отсек корпуса.
type SectionLayout struct {
	// Kind — назначение отсека: nose, payload, tank, dry, engines.
	Kind  string `json:"kind"`
	Title string `json:"title"`

	Length   float64 `json:"length"`   // м
	Diameter float64 `json:"diameter"` // м

	// Tank связывает отсек с баком телеметрии: "fuel" или "ox". Пустое
	// значение означает, что заполнять отсек нечем.
	Tank string `json:"tank,omitempty"`

	// Propellant — название компонента для подписи.
	Propellant string `json:"propellant,omitempty"`
}

// EngineLayout — место двигателя на срезе юбки.
type EngineLayout struct {
	ID string `json:"id"`

	// X и Y — координаты оси двигателя, метры от продольной оси.
	X float64 `json:"x"`
	Y float64 `json:"y"`

	// ExitRadius — радиус среза сопла, м.
	ExitRadius float64 `json:"exitRadius"`

	// Kind — "sealevel" или "vacuum".
	Kind string `json:"kind"`

	// Gimbal сообщает, что двигатель качается в подвесе. У сверхтяжёлого
	// носителя качаются только центральные: наружные закреплены жёстко.
	Gimbal bool `json:"gimbal"`

	Ring int `json:"ring"` // 0 — центральное кольцо

	// EngineNumber — стабильный физический номер двигателя (1-based),
	// НЕ зависящий от индекса в массиве/порядка укладки placeRings (Stage
	// 4.10, п.1). Источник истины для propulsion, TVC allocator, landing
	// sequencing и рендерера — все обязаны опираться на него (или на ID,
	// производный от него), а не на позицию в срезе. 0 — не назначен
	// (ступени, где физическая нумерация не вводилась, например корабль).
	EngineNumber int `json:"engineNumber"`
}

// Layout возвращает обводы носителя.
func NewLayout(cfg Config) Layout {
	first := firstStageLayout(cfg)
	second := secondStageLayout(cfg)

	total := cfg.TotalLength
	if total <= 0 {
		total = first.Length + second.Length
	}

	out := Layout{
		Title:       cfg.Name,
		Diameter:    cfg.Diameter,
		TotalLength: total,
		Flaps:       cfg.Flaps,
		Stages:      []StageLayout{second, first},
	}
	if cfg.Flaps {
		out.FlapMounts = flapMountLayout(cfg)
	}
	return out
}

// flapMountLayout переводит привязки плавников в градусы для рисовки.
//
// Берутся они у той же функции, по которой модель строит управляющие
// поверхности: разойтись картинке с расчётом здесь физически негде.
func flapMountLayout(cfg Config) []FlapMountLayout {
	mounts := ShipFlapMounts(secondStageLength(cfg), cfg.Diameter/2)

	out := make([]FlapMountLayout, 0, len(mounts))
	for _, m := range mounts {
		out = append(out, FlapMountLayout{
			Name:     m.Name,
			Station:  m.Station,
			Azimuth:  m.Azimuth / physics.DegToRad,
			Area:     m.Area,
			Limit:    m.Limit / physics.DegToRad,
			Span:     m.Span,
			Chord:    m.Chord,
			StowHalf: m.StowHalf / physics.DegToRad,
		})
	}
	return out
}

// secondStageLength возвращает длину верхней ступени вместе с носовой частью.
func secondStageLength(cfg Config) float64 {
	if cfg.TotalLength > 0 && cfg.FirstStageLength > 0 {
		if l := cfg.TotalLength - cfg.FirstStageLength; l > 0 {
			return l
		}
	}

	// Запасной вариант: баки плюс треть на носовую часть и юбку.
	return (cfg.SecondStage.OxTankHeight + cfg.SecondStage.FuelTankHeight) * 1.35
}

// firstStageLayout раскладывает первую ступень на отсеки.
//
// Порядок сверху вниз: межступенчатый отсек, бак окислителя, бак горючего,
// двигательный отсек. Окислитель наверху у обоих носителей: его втрое больше
// по массе, и такое расположение сдвигает центр масс к носу, что помогает
// устойчивости на активном участке.
func firstStageLayout(cfg Config) StageLayout {
	stage := cfg.FirstStage
	length := cfg.FirstStageLength
	if length <= 0 {
		length = (stage.OxTankHeight + stage.FuelTankHeight) * 1.3
	}

	pair := FirstStageEngineConfig(cfg).Pair

	engineBay := length * 0.11
	interstage := length * 0.06

	sections := []SectionLayout{
		{
			Kind:     "dry",
			Title:    "Межступенчатый отсек",
			Length:   interstage,
			Diameter: cfg.Diameter,
		},
	}
	sections = append(sections,
		SectionLayout{
			Kind:       "tank",
			Title:      "Бак окислителя",
			Length:     stage.OxTankHeight,
			Diameter:   cfg.Diameter,
			Tank:       "ox",
			Propellant: pair.Oxidizer.Name,
		},
		SectionLayout{
			Kind:       "tank",
			Title:      "Бак горючего",
			Length:     stage.FuelTankHeight,
			Diameter:   cfg.Diameter,
			Tank:       "fuel",
			Propellant: pair.Fuel.Name,
		},
		SectionLayout{
			Kind:     "engines",
			Title:    "Двигательный отсек",
			Length:   engineBay,
			Diameter: cfg.Diameter,
		},
	)

	return StageLayout{
		Index:    1,
		Name:     stage.Name,
		Prefix:   "S1",
		Length:   length,
		Sections: fitSections(sections, length),
		Engines:  firstStageEngineLayout(cfg),
	}
}

// fitSections подгоняет отсеки под полную длину ступени.
//
// Высоты баков и длина ступени задаются в конфигурации порознь: первые нужны
// модели для гидростатического напора, вторая — для моментов инерции. Строгого
// равенства между ними никто не обеспечивает, и на чертеже отсеки либо не
// достали бы до низа, либо вылезли за габарит. Поэтому длины пропорционально
// приводятся к полной длине ступени: пропорции сохраняются, общий размер
// становится верным.
func fitSections(sections []SectionLayout, length float64) []SectionLayout {
	total := 0.0
	for _, s := range sections {
		total += s.Length
	}
	if total <= 0 || length <= 0 {
		return sections
	}

	k := length / total
	for i := range sections {
		sections[i].Length *= k
	}
	return sections
}

// secondStageLayout раскладывает верхнюю ступень на отсеки.
//
// Здесь порядок баков другой. У носителя на метане окислитель стоит внизу,
// у самых двигателей, а горючее над ним; у керосинового наоборот. Это не
// произвол: расположение определяется тем, какой компонент подаётся коротким
// трактом, и переставлять баки местами ради единообразия картинки нельзя.
func secondStageLayout(cfg Config) StageLayout {
	stage := cfg.SecondStage
	length := secondStageLength(cfg)

	pair := SecondStagePrimaryConfig(cfg).Pair

	engineBay := length * 0.10
	nose := length * 0.22

	oxSection := SectionLayout{
		Kind:       "tank",
		Title:      "Бак окислителя",
		Length:     stage.OxTankHeight,
		Diameter:   cfg.Diameter,
		Tank:       "ox",
		Propellant: pair.Oxidizer.Name,
	}
	fuelSection := SectionLayout{
		Kind:       "tank",
		Title:      "Бак горючего",
		Length:     stage.FuelTankHeight,
		Diameter:   cfg.Diameter,
		Tank:       "fuel",
		Propellant: pair.Fuel.Name,
	}

	sections := []SectionLayout{}
	if nose > 0 {
		title := "Головной обтекатель"
		kind := "nose"
		if cfg.FairingMass <= 0 {
			// Обтекателя как отдельного сбрасываемого элемента нет:
			// нагрузка едет внутри корпуса под створками.
			title = "Грузовой отсек"
			kind = "payload"
		}
		sections = append(sections, SectionLayout{
			Kind:     kind,
			Title:    title,
			Length:   nose,
			Diameter: cfg.Diameter,
		})
	}

	if IsMethalox(cfg) {
		sections = append(sections, fuelSection, oxSection)
	} else {
		sections = append(sections, oxSection, fuelSection)
	}

	sections = append(sections, SectionLayout{
		Kind:     "engines",
		Title:    "Двигательный отсек",
		Length:   engineBay,
		Diameter: cfg.Diameter,
	})

	return StageLayout{
		Index:    2,
		Name:     stage.Name,
		Prefix:   "S2",
		Length:   length,
		Sections: fitSections(sections, length),
		Engines:  secondStageEngineLayout(cfg),
	}
}

// exitRadius возвращает радиус среза сопла, м.
func exitRadius(c propulsion.EngineConfig) float64 {
	if c.Nozzle.ExitArea <= 0 {
		return 0.4
	}
	return math.Sqrt(c.Nozzle.ExitArea / math.Pi)
}

// engineRings разбивает двигатели ступени по кольцам.
//
// Разбиение не произвольное: сопла нельзя поставить ближе, чем позволяет их
// диаметр, поэтому число мест на кольце определяется его длиной. Девять
// двигателей ложатся в схему «один по центру и восемь по кольцу», тридцать
// три — в три кольца по три, десять и двадцать.
func engineRings(count int) []int {
	switch {
	case count <= 0:
		return nil
	case count == 1:
		return []int{1}
	case count <= 4:
		return []int{count}
	case count == 6:
		return []int{3, 3}
	case count == 9:
		return []int{1, 8}
	case count == 33:
		return []int{3, 10, 20}
	}

	// Общий случай: наружное кольцо самое ёмкое, каждое следующее внутрь
	// вмещает примерно вдвое меньше.
	rings := []int{}
	left := count
	outer := int(math.Round(float64(count) * 0.6))
	for left > 0 {
		if outer < 1 {
			outer = 1
		}
		if outer > left {
			outer = left
		}
		rings = append([]int{outer}, rings...)
		left -= outer
		outer = int(math.Round(float64(outer) / 2))
	}
	return rings
}

// ringSpec — кольцо однотипных двигателей.
type ringSpec struct {
	Count  int
	Nozzle float64 // радиус среза сопла, м
	Suffix string  // обозначение группы в идентификаторе
	Kind   string  // "sealevel" или "vacuum"
	Gimbal bool
}

// placeRings расставляет двигатели ступени по кольцам.
//
// Радиусы не задаются на глаз, а следуют из размеров сопел. Соседние камеры
// на одном кольце не могут стоять ближе, чем позволяет их диаметр, а соседние
// кольца — ближе суммы радиусов своих сопел. Кольца ещё и повёрнуты друг
// относительно друга: иначе внутренние двигатели оказываются точно за
// наружными, и на виде с торца шесть камер корабля выглядели как три.
//
// Если наружное кольцо не влезает в габарит, все радиусы сжимаются
// пропорционально: у сверхтяжёлого носителя двадцать камер наружного кольца
// и в жизни стоят почти вплотную.
func placeRings(prefix string, rings []ringSpec, bodyRadius float64) []EngineLayout {
	n := len(rings)
	if n == 0 {
		return nil
	}

	radii := make([]float64, n)
	phase := make([]float64, n)

	for i, r := range rings {
		// Своё кольцо: камеры не должны задевать соседей по кольцу.
		own := 0.0
		if r.Count > 1 {
			own = r.Nozzle / math.Sin(math.Pi/float64(r.Count)) * 1.02
		}

		// Зазор до предыдущего кольца.
		prev := 0.0
		if i > 0 {
			prev = radii[i-1] + rings[i-1].Nozzle + r.Nozzle
			// Полшага поворота: двигатели встают в промежутки между
			// двигателями соседнего кольца.
			phase[i] = phase[i-1] + math.Pi/float64(maxInt(r.Count, 1))
		}

		radii[i] = math.Max(own, prev)
	}

	// Габарит: край сопла наружного кольца укладывается в корпус. Небольшой
	// выход за обвод допустим — у сверхтяжёлого носителя сопла наружного
	// кольца и в жизни стоят вровень с краем юбки.
	if limit := bodyRadius * 1.02; bodyRadius > 0 {
		edge := radii[n-1] + rings[n-1].Nozzle
		if edge > limit {
			k := limit / edge
			for i := range radii {
				radii[i] *= k
			}
		}
	}

	out := make([]EngineLayout, 0, 8)
	for i, r := range rings {
		for j := 0; j < r.Count; j++ {
			angle := phase[i]
			if r.Count > 1 {
				angle += 2 * math.Pi * float64(j) / float64(r.Count)
			}
			out = append(out, EngineLayout{
				ID:         fmt.Sprintf("%s-%s%d", prefix, r.Suffix, j+1),
				X:          radii[i] * math.Sin(angle),
				Y:          radii[i] * math.Cos(angle),
				ExitRadius: r.Nozzle,
				Kind:       r.Kind,
				Gimbal:     r.Gimbal,
				Ring:       i,
			})
		}
	}
	return out
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

// firstStageEngineLayout расставляет двигатели первой ступени.
//
// Идентификаторы обязаны совпадать с теми, что выдаёт двигательная установка:
// по ним телеметрия каждого двигателя ложится на своё место в развёртке.
func firstStageEngineLayout(cfg Config) []EngineLayout {
	count := int(cfg.FirstStage.EngineCount)
	nozzle := exitRadius(FirstStageEngineConfig(cfg))
	counts := engineRings(count)

	specs := make([]ringSpec, 0, len(counts))
	for i, n := range counts {
		// Качаются все кольца, кроме САМОГО НАРУЖНОГО: у сверхтяжёлого
		// носителя V3 внешние камеры закреплены жёстко (соплам такого
		// размаха качать нечем — как и вакуумным камерам корабля,
		// secondStageEngineLayout), а маневренная группа — это ЦЕНТР плюс
		// СРЕДНЕЕ кольцо разом. При штатной раскладке [3,10,20] это даёт
		// 3+10=13 маневренных камер — ту самую физическую тринадцатку,
		// которую заявляет V3 и на которую опирается посадочная
		// последовательность (boosterLandingHigh, booster.go): раньше здесь
		// стояло "качается только i==0" — то есть только ТРИ центральных
		// камеры, а не тринадцать, хотя вся посадочная логика уже
		// подразумевала 13. TestEngineLayout_ManeuverableCountMatchesLanding
		// (booster_stability_test.go) держит это соответствие явным.
		specs = append(specs, ringSpec{
			Count:  n,
			Nozzle: nozzle,
			Kind:   "sealevel",
			Gimbal: i < len(counts)-1 || len(counts) == 1,
		})
	}

	out := placeRings("S1", specs, cfg.Diameter/2)

	// Физическая нумерация (Stage 4.10, п.1) — сквозная по кольцам (кольцо 0
	// первым, дальше по возрастанию), номера начинаются с 1. Внутри каждого
	// кольца — по порядку укладки (placeRings, по углу), КРОМЕ маневренного
	// среднего кольца из десяти камер (см. innerRingEngineNumberBySlot,
	// там же и почему формулой это не выразить).
	ringBase := make([]int, len(specs))
	base := 1
	for i, s := range specs {
		ringBase[i] = base
		base += s.Count
	}
	ringSeen := make([]int, len(specs))
	for i := range out {
		r := out[i].Ring
		j := ringSeen[r]
		ringSeen[r]++
		offset := j
		if specs[r].Count == len(innerRingEngineNumberBySlot) && r != 0 {
			offset = innerRingEngineNumberBySlot[j]
		}
		out[i].EngineNumber = ringBase[r] + offset
		out[i].ID = fmt.Sprintf("S1-%d", out[i].EngineNumber)
	}
	return out
}

// innerRingEngineNumberBySlot — Stage 4.10, п.1: перестановка номеров внутри
// маневренного кольца из десяти камер (смещения 0..9 от начала диапазона
// кольца), а не сквозной порядок укладки по углу. Нужна, чтобы физические
// номера 6 и 12 (см. landingGroup5EngineNumbers, booster.go) легли РОВНО на
// диаметрально противоположные места (5 позиций из 10 = 180°, слот 0 и
// слот 5 в этой таблице): у десяти позиций на кольце разность номеров
// строго противоположной пары обязана быть ровно 5 при ЛЮБОЙ сквозной
// (по углу, с любой отправной точки) нумерации, а 12−6=6 — подряд идущая
// нумерация 4..13 такую пару в принципе не даёт ни при каком повороте
// отсчёта, отсюда явная перестановка, а не формула.
var innerRingEngineNumberBySlot = [10]int{2, 3, 4, 5, 6, 8, 9, 0, 1, 7}

// secondStageEngineLayout расставляет двигатели верхней ступени.
//
// Основная группа — атмосферные камеры, они стоят в центре и качаются.
// Вакуумные с их огромными соплами вынесены на наружное кольцо и закреплены
// жёстко: качать сопло такого размера нечем.
func secondStageEngineLayout(cfg Config) []EngineLayout {
	primary := SecondStagePrimaryConfig(cfg)
	primaryCount := SecondStagePrimaryCount(cfg)
	groups := SecondStageGroups(cfg)

	specs := []ringSpec{{
		Count:  primaryCount,
		Nozzle: exitRadius(primary),
		Kind:   "sealevel",
		Gimbal: true,
	}}
	for _, g := range groups {
		specs = append(specs, ringSpec{
			Count:  g.Count,
			Nozzle: exitRadius(g.Config),
			Suffix: g.Suffix,
			Kind:   "vacuum",
		})
	}

	return placeRings("S2", specs, cfg.Diameter/2)
}
