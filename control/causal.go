package control

import (
	"math"
	"sort"
	"time"
)

// -----------------------------------------------------------------------------
// Регистрация отклика модели.
//
// После вмешательства интерфейс обязан показать, что именно изменилось. Здесь
// принципиально важно не выдавать желаемое за действительное: связь считается
// установленной только тогда, когда величина действительно сдвинулась в данных
// симуляции. Реестр параметров хранит ожидаемые связи, но они лишь помечают
// отклик как непосредственный, а не подменяют собой измерение.
// -----------------------------------------------------------------------------

// Watch — наблюдаемая величина.
type Watch struct {
	Key   string `json:"key"`
	Title string `json:"title"`
	Unit  string `json:"unit"`

	// Threshold — относительное изменение, начиная с которого отклик
	// считается значимым.
	Threshold float64 `json:"threshold"`

	// Floor — абсолютный порог для величин, близких к нулю.
	Floor float64 `json:"floor"`

	// Group — раздел для интерфейса.
	Group string `json:"group"`
}

// watches — величины, за которыми следит пульт.
//
// Список намеренно совпадает с тем, что перечислено в постановке задачи:
// давление за насосом, расход, соотношение компонентов, давление и температура
// камеры, полнота сгорания, тяга, удельный импульс, вибрация, кавитационный
// запас, температура подшипников, остаток топлива, ускорение и траектория.
var watches = []Watch{
	{"turbopump.rpm", "Обороты вала", "об/мин", 0.01, 200, "Турбонасос"},
	{"turbopump.fuelPumpOutlet", "Давление за насосом горючего", "МПа", 0.02, 0.05, "Турбонасос"},
	{"turbopump.oxPumpOutlet", "Давление за насосом окислителя", "МПа", 0.02, 0.05, "Турбонасос"},
	{"turbopump.fuelPumpInlet", "Давление на входе насоса горючего", "кПа", 0.02, 5, "Турбонасос"},
	{"turbopump.oxPumpInlet", "Давление на входе насоса окислителя", "кПа", 0.02, 5, "Турбонасос"},
	{"turbopump.turbinePower", "Мощность турбины", "МВт", 0.02, 0.05, "Турбонасос"},
	{"turbopump.fuelPumpPower", "Мощность насоса горючего", "МВт", 0.02, 0.05, "Турбонасос"},
	{"turbopump.oxPumpPower", "Мощность насоса окислителя", "МВт", 0.02, 0.05, "Турбонасос"},
	{"turbopump.fuelCavitationMargin", "Кавитационный запас горючего", "м", 0.05, 1, "Турбонасос"},
	{"turbopump.oxCavitationMargin", "Кавитационный запас окислителя", "м", 0.05, 1, "Турбонасос"},
	{"turbopump.vibration", "Вибрация", "g", 0.05, 0.2, "Турбонасос"},
	{"turbopump.bearingTemperature", "Температура подшипников", "К", 0.01, 2, "Турбонасос"},
	{"turbopump.powerMargin", "Запас мощности турбины", "МВт", 0.05, 0.02, "Турбонасос"},
	{"turbopump.mechanicalLoss", "Механические потери", "МВт", 0.03, 0.02, "Турбонасос"},
	{"turbopump.angularAcceleration", "Угловое ускорение вала", "рад/с²", 0.05, 20, "Турбонасос"},

	{"engine.fuelFlow", "Расход горючего", "кг/с", 0.02, 0.5, "Двигатель"},
	{"engine.oxFlow", "Расход окислителя", "кг/с", 0.02, 0.5, "Двигатель"},
	{"engine.thrust", "Тяга", "кН", 0.01, 3, "Двигатель"},
	{"engine.specificImpulse", "Удельный импульс", "с", 0.01, 1, "Двигатель"},

	{"chamber.mixtureRatio", "Соотношение компонентов O/F", "", 0.01, 0.02, "Камера"},
	{"chamber.pressure", "Давление в камере", "МПа", 0.01, 0.05, "Камера"},
	{"chamber.temperature", "Температура в камере", "К", 0.01, 20, "Камера"},
	{"chamber.cstarEfficiency", "Полнота сгорания", "", 0.005, 0.005, "Камера"},
	{"chamber.stabilityMargin", "Запас устойчивости", "", 0.05, 0.01, "Камера"},
	{"chamber.pressureRMS", "Пульсации давления", "кПа", 0.1, 5, "Камера"},
	{"chamber.dominantFrequency", "Частота пульсаций", "Гц", 0.05, 3, "Камера"},
	{"chamber.temperatureSpread", "Температурная неоднородность", "К", 0.05, 10, "Камера"},

	{"valves.fuel", "Клапан горючего", "", 0.01, 0.01, "Клапаны"},
	{"valves.ox", "Клапан окислителя", "", 0.01, 0.01, "Клапаны"},
	{"valves.gasGen", "Клапан газогенератора", "", 0.01, 0.01, "Клапаны"},
	{"valves.bypass", "Перепускной клапан", "", 0.01, 0.01, "Клапаны"},

	{"nozzle.wallMargin", "Запас до предела стенки", "К", 0.03, 15, "Сопло"},
	{"nozzle.wallDamage", "Повреждение стенки", "", 0.01, 0.01, "Сопло"},
	{"engine.thrustDeficit", "Недобор тяги", "доля", 0.02, 0.02, "Двигатель"},
	{"nozzle.wallHeatFlux", "Тепловой поток в стенки", "МВт/м²", 0.03, 0.3, "Сопло"},
	{"nozzle.coolingFlow", "Расход охладителя", "кг/с", 0.03, 1, "Сопло"},
	{"nozzle.coolantOutlet", "Температура охладителя на выходе", "К", 0.02, 5, "Сопло"},
	{"nozzle.throatArea", "Площадь горловины", "см²", 0.002, 0.3, "Сопло"},
	{"nozzle.throatErosion", "Эрозия горловины", "мм", 0.1, 0.02, "Сопло"},
	{"nozzle.coolingDrop", "Потери давления в тракте", "кПа", 0.03, 20, "Сопло"},
	{"nozzle.coolantInlet", "Температура охладителя на входе", "К", 0.02, 3, "Сопло"},

	{"fuelTank.mass", "Остаток горючего", "т", 0.005, 0.3, "Баки"},
	{"oxTank.mass", "Остаток окислителя", "т", 0.005, 0.3, "Баки"},
	{"fuelTank.pressure", "Давление в баке горючего", "кПа", 0.02, 5, "Баки"},
	{"oxTank.pressure", "Давление в баке окислителя", "кПа", 0.02, 5, "Баки"},
	{"fuelTank.sloshAmplitude", "Колебания жидкости, горючее", "м", 0.1, 0.01, "Баки"},
	{"oxTank.sloshAmplitude", "Колебания жидкости, окислитель", "м", 0.1, 0.01, "Баки"},
	{"fuelTank.temperature", "Температура горючего", "К", 0.01, 1, "Баки"},
	{"oxTank.temperature", "Температура окислителя", "К", 0.01, 1, "Баки"},
	{"fuelTank.lineDrop", "Потери в магистрали горючего", "кПа", 0.05, 5, "Баки"},
	{"oxTank.lineDrop", "Потери в магистрали окислителя", "кПа", 0.05, 5, "Баки"},
	{"fuelTank.pressurantFlow", "Расход наддува горючего", "кг/с", 0.05, 0.005, "Баки"},
	{"oxTank.pressurantFlow", "Расход наддува окислителя", "кг/с", 0.05, 0.005, "Баки"},
	{"fuelTank.depletionTime", "Время до выработки горючего", "с", 0.03, 2, "Баки"},
	{"oxTank.depletionTime", "Время до выработки окислителя", "с", 0.03, 2, "Баки"},

	{"vehicle.acceleration", "Продольная перегрузка", "g", 0.02, 0.05, "Носитель"},
	{"vehicle.angleOfAttack", "Полный угол атаки", "°", 0.05, 0.3, "Носитель"},
	{"vehicle.pitchRate", "Угловая скорость тангажа", "°/с", 0.05, 0.05, "Носитель"},
	{"control.gimbalPitch", "Отклонение камер по тангажу", "°", 0.05, 0.02, "Управление"},
	{"control.gimbalDemand", "Потребное отклонение камер", "°", 0.05, 0.02, "Управление"},
	{"control.authority", "Запас управляющего момента", "", 0.05, 0.02, "Управление"},
	{"control.staticMargin", "Запас устойчивости", "калибр", 0.05, 0.05, "Управление"},
	{"vehicle.altitude", "Высота", "км", 0.002, 0.05, "Носитель"},
	{"vehicle.velocity", "Скорость", "м/с", 0.005, 5, "Носитель"},
	{"vehicle.mass", "Масса носителя", "т", 0.005, 0.3, "Носитель"},
}

// watchIndex — быстрый доступ к описанию наблюдаемой величины.
var watchIndex = func() map[string]Watch {
	m := make(map[string]Watch, len(watches))
	for _, w := range watches {
		m[w.Key] = w
	}
	return m
}()

// Watches возвращает список наблюдаемых величин.
func Watches() []Watch { return append([]Watch(nil), watches...) }

// -----------------------------------------------------------------------------
// Наблюдатель за откликом
// -----------------------------------------------------------------------------

// responseWindow — сколько модельного времени пульт следит за откликом.
//
// Восьми секунд достаточно: турбонасос выходит на режим за две-три секунды,
// тепловые процессы медленнее, но их начало заметно уже в этом окне.
const responseWindow = 8.0

type tracker struct {
	commandID string
	param     Param
	start     float64
	before    Sample
	onset     map[string]float64
	extreme   map[string]float64
	last      Sample
}

// track заводит наблюдателя за откликом на команду.
func (b *Board) track(cmd Command, p Param, now float64, sample Sample) {
	t := &tracker{
		commandID: cmd.ID,
		param:     p,
		start:     now,
		before:    make(Sample, len(sample)),
		onset:     make(map[string]float64),
		extreme:   make(map[string]float64),
		last:      make(Sample, len(sample)),
	}
	for _, w := range watches {
		if v, ok := sample[w.Key]; ok {
			t.before[w.Key] = v
			t.extreme[w.Key] = v
			t.last[w.Key] = v
		}
	}
	b.trackers = append(b.trackers, t)
}

// observe продвигает наблюдателей. Вызывается под удержанным mu.
func (b *Board) observe(now float64, sample Sample) {
	if len(b.trackers) == 0 {
		return
	}
	alive := b.trackers[:0]

	for _, t := range b.trackers {
		for _, w := range watches {
			v, ok := sample[w.Key]
			if !ok || math.IsNaN(v) {
				continue
			}
			t.last[w.Key] = v

			base, had := t.before[w.Key]
			if !had {
				t.before[w.Key] = v
				t.extreme[w.Key] = v
				continue
			}
			if math.Abs(v-base) > math.Abs(t.extreme[w.Key]-base) {
				t.extreme[w.Key] = v
			}
			if _, seen := t.onset[w.Key]; !seen && significant(w, base, v) {
				t.onset[w.Key] = now - t.start
			}
		}

		if now-t.start < responseWindow {
			alive = append(alive, t)
			continue
		}
		b.finalize(t, now)
	}
	b.trackers = alive
}

// significant сообщает, вышло ли изменение за порог чувствительности.
func significant(w Watch, before, after float64) bool {
	d := math.Abs(after - before)
	if d < w.Floor {
		return false
	}
	if before == 0 {
		return d > w.Floor
	}
	return d/math.Abs(before) > w.Threshold
}

// Response — отклик одной величины на воздействие.
type Response struct {
	Key   string `json:"key"`
	Title string `json:"title"`
	Unit  string `json:"unit"`
	Group string `json:"group"`

	Before float64 `json:"before"`
	After  float64 `json:"after"`
	Peak   float64 `json:"peak"`

	// Change — относительное изменение к концу окна наблюдения.
	Change float64 `json:"change"`

	// Onset — через сколько секунд модельного времени изменение стало
	// заметным. Отрицательное значение означает, что величина не отреагировала.
	Onset float64 `json:"onset"`

	// Expected сообщает, что реестр относит эту величину к непосредственно
	// связанным с изменённым параметром. Само по себе это не доказательство:
	// в список отклика величина попадает только по данным модели.
	Expected bool `json:"expected"`
}

// Reaction — сводка отклика на воздействие.
type Reaction struct {
	Window float64 `json:"window"`

	// Responses — величины, которые действительно изменились, в порядке
	// появления отклика.
	Responses []Response `json:"responses"`

	// Chain — порядок отклика, наблюдавшийся в данных модели.
	Chain []string `json:"chain"`

	// Silent — величины, которые реестр считал связанными, но которые
	// не отреагировали. Показывать их честнее, чем умалчивать.
	Silent []string `json:"silent,omitempty"`
}

// finalize закрывает наблюдение и дописывает отклик в журнал.
func (b *Board) finalize(t *tracker, now float64) {
	expected := make(map[string]bool, len(t.param.Affects))
	for _, k := range t.param.Affects {
		expected[k] = true
	}

	var responses []Response
	for _, w := range watches {
		onset, moved := t.onset[w.Key]
		if !moved {
			continue
		}
		before := t.before[w.Key]
		after := t.last[w.Key]
		change := 0.0
		if before != 0 {
			change = (after - before) / math.Abs(before)
		}
		responses = append(responses, Response{
			Key: w.Key, Title: w.Title, Unit: w.Unit, Group: w.Group,
			Before: before, After: after, Peak: t.extreme[w.Key],
			Change: change, Onset: onset, Expected: expected[w.Key],
		})
	}

	// Порядок — по времени появления отклика: это измеренная величина.
	//
	// Когда телеметрия приходит редко или симуляция идёт с ускорением, все
	// отклики попадают в одно наблюдение и время не различает их. В этом
	// случае порядок не определён данными, и его задаёт объявленная в реестре
	// топология связей: сперва непосредственно связанные величины, затем
	// остальные. Само попадание в список по-прежнему решают только данные —
	// величина, не сдвинувшаяся в модели, сюда не попадёт.
	sort.Slice(responses, func(i, j int) bool {
		a, b := responses[i], responses[j]
		if a.Onset != b.Onset {
			return a.Onset < b.Onset
		}
		if a.Expected != b.Expected {
			return a.Expected
		}
		return math.Abs(a.Change) > math.Abs(b.Change)
	})

	// Цепочка строится по времени появления отклика, а не по ожиданиям.
	chain := make([]string, 0, 8)
	for _, r := range responses {
		if len(chain) >= 8 {
			break
		}
		direction := "выросло"
		if r.After < r.Before {
			direction = "снизилось"
		}
		chain = append(chain, r.Title+" "+direction)
	}

	var silent []string
	for _, k := range t.param.Affects {
		if _, moved := t.onset[k]; !moved {
			if w, ok := watchIndex[k]; ok {
				silent = append(silent, w.Title)
			}
		}
	}

	reaction := &Reaction{
		Window:    now - t.start,
		Responses: responses,
		Chain:     chain,
		Silent:    silent,
	}
	for i := range b.log {
		if b.log[i].ID == t.commandID {
			b.log[i].Reaction = reaction
			return
		}
	}
}

// -----------------------------------------------------------------------------
// Сравнение с базовым запуском
// -----------------------------------------------------------------------------

// Point — снимок наблюдаемых величин на момент модельного времени.
type Point struct {
	T      float64            `json:"t"`
	Values map[string]float64 `json:"values"`
}

// Baseline — сохранённый запуск для сравнения.
type Baseline struct {
	Label      string    `json:"label"`
	CapturedAt time.Time `json:"capturedAt"`
	Seed       int64     `json:"seed"`
	Points     []Point   `json:"points"`
}

// historyLimit ограничивает длину истории одного прогона.
const historyLimit = 4000

// Record добавляет точку в историю текущего прогона.
func (b *Board) Record(t float64, sample Sample) {
	b.mu.Lock()
	defer b.mu.Unlock()

	values := make(map[string]float64, len(watches))
	for _, w := range watches {
		if v, ok := sample[w.Key]; ok {
			values[w.Key] = v
		}
	}
	b.history = append(b.history, Point{T: t, Values: values})
	if len(b.history) > historyLimit {
		b.history = append([]Point(nil), b.history[len(b.history)-historyLimit:]...)
	}
}

// History возвращает историю текущего прогона.
func (b *Board) History() []Point {
	b.mu.RLock()
	defer b.mu.RUnlock()
	return append([]Point(nil), b.history...)
}

// SetBaseline сохраняет текущий прогон как эталон для сравнения.
func (b *Board) SetBaseline(label string, seed int64) *Baseline {
	b.mu.Lock()
	defer b.mu.Unlock()

	bl := &Baseline{
		Label:      label,
		CapturedAt: time.Now().UTC(),
		Seed:       seed,
		Points:     append([]Point(nil), b.history...),
	}
	b.baseline = bl
	return bl
}

// Baseline возвращает сохранённый эталон.
func (b *Board) Baseline() *Baseline {
	b.mu.RLock()
	defer b.mu.RUnlock()
	return b.baseline
}

// Divergence — расхождение одной величины между прогонами.
type Divergence struct {
	Key   string `json:"key"`
	Title string `json:"title"`
	Unit  string `json:"unit"`
	Group string `json:"group"`

	// Baseline, Current — значения в конце сравниваемого отрезка.
	Baseline float64 `json:"baseline"`
	Current  float64 `json:"current"`

	// Delta — разница в конце отрезка.
	Delta float64 `json:"delta"`

	// MaxDeviation — наибольшее расхождение за отрезок.
	MaxDeviation float64 `json:"maxDeviation"`

	// MaxDeviationAt — модельное время наибольшего расхождения.
	MaxDeviationAt float64 `json:"maxDeviationAt"`

	// DivergedAt — момент, с которого прогоны разошлись заметно.
	// Отрицательное значение означает, что величина не разошлась.
	DivergedAt float64 `json:"divergedAt"`

	// SettledAt — момент, после которого расхождение перестало меняться.
	SettledAt float64 `json:"settledAt"`

	// OutOfRange сообщает, что в текущем прогоне величина вышла
	// за пределы, в которых оставалась в эталонном.
	OutOfRange bool `json:"outOfRange"`
}

// Comparison — результат сравнения с эталоном.
type Comparison struct {
	Available bool         `json:"available"`
	Label     string       `json:"label"`
	Seed      int64        `json:"seed"`
	From      float64      `json:"from"`
	To        float64      `json:"to"`
	Rows      []Divergence `json:"rows"`
}

// Compare сравнивает текущий прогон с сохранённым эталоном.
func (b *Board) Compare() Comparison {
	b.mu.RLock()
	defer b.mu.RUnlock()

	if b.baseline == nil || len(b.baseline.Points) == 0 || len(b.history) == 0 {
		return Comparison{}
	}

	base := indexByTime(b.baseline.Points)
	cur := b.history

	cmp := Comparison{
		Available: true,
		Label:     b.baseline.Label,
		Seed:      b.baseline.Seed,
		From:      cur[0].T,
		To:        cur[len(cur)-1].T,
	}

	type acc struct {
		lastBase, lastCur       float64
		maxDev, maxAt           float64
		divergedAt, settledAt   float64
		haveBase, outOfRange    bool
		baseMin, baseMax, curV  float64
		initialisedRangeTracker bool
	}
	state := make(map[string]*acc, len(watches))
	for _, w := range watches {
		state[w.Key] = &acc{divergedAt: -1, settledAt: -1}
	}

	for _, p := range cur {
		bp, ok := base.at(p.T)
		if !ok {
			continue
		}
		for key, v := range p.Values {
			bv, ok := bp[key]
			if !ok || math.IsNaN(v) || math.IsNaN(bv) {
				continue
			}
			w, known := watchIndex[key]
			if !known {
				continue
			}
			a := state[key]
			a.haveBase = true
			a.lastBase, a.lastCur = bv, v

			if !a.initialisedRangeTracker {
				a.baseMin, a.baseMax = bv, bv
				a.initialisedRangeTracker = true
			}
			a.baseMin = math.Min(a.baseMin, bv)
			a.baseMax = math.Max(a.baseMax, bv)
			if v < a.baseMin-math.Abs(a.baseMin)*0.05-w.Floor ||
				v > a.baseMax+math.Abs(a.baseMax)*0.05+w.Floor {
				a.outOfRange = true
			}

			d := v - bv
			if math.Abs(d) > math.Abs(a.maxDev) {
				a.maxDev, a.maxAt = d, p.T
				a.settledAt = p.T
			}
			if a.divergedAt < 0 && significant(w, bv, v) {
				a.divergedAt = p.T
			}
		}
	}

	for _, w := range watches {
		a := state[w.Key]
		if !a.haveBase || (a.divergedAt < 0 && a.maxDev == 0) {
			continue
		}
		cmp.Rows = append(cmp.Rows, Divergence{
			Key: w.Key, Title: w.Title, Unit: w.Unit, Group: w.Group,
			Baseline: a.lastBase, Current: a.lastCur,
			Delta:        a.lastCur - a.lastBase,
			MaxDeviation: a.maxDev, MaxDeviationAt: a.maxAt,
			DivergedAt: a.divergedAt, SettledAt: a.settledAt,
			OutOfRange: a.outOfRange,
		})
	}

	sort.Slice(cmp.Rows, func(i, j int) bool {
		return relative(cmp.Rows[i]) > relative(cmp.Rows[j])
	})
	return cmp
}

func relative(d Divergence) float64 {
	if d.Baseline == 0 {
		return math.Abs(d.MaxDeviation)
	}
	return math.Abs(d.MaxDeviation / d.Baseline)
}

// timeIndex ускоряет поиск ближайшей по времени точки эталона.
type timeIndex struct {
	points []Point
}

func indexByTime(points []Point) timeIndex { return timeIndex{points: points} }

// at возвращает значения эталона на момент, ближайший к t.
func (idx timeIndex) at(t float64) (map[string]float64, bool) {
	if len(idx.points) == 0 {
		return nil, false
	}
	i := sort.Search(len(idx.points), func(i int) bool { return idx.points[i].T >= t })
	switch {
	case i == 0:
		return idx.points[0].Values, math.Abs(idx.points[0].T-t) < 2
	case i >= len(idx.points):
		last := idx.points[len(idx.points)-1]
		return last.Values, math.Abs(last.T-t) < 2
	}
	lo, hi := idx.points[i-1], idx.points[i]
	if t-lo.T <= hi.T-t {
		return lo.Values, true
	}
	return hi.Values, true
}
