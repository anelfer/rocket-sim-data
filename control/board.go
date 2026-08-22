package control

import (
	"fmt"
	"math"
	"sort"
	"strings"
	"sync"
	"time"
)

// -----------------------------------------------------------------------------
// Пульт воздействий.
//
// Пульт — единственная точка, через которую внешний мир меняет модель. Он
// проверяет команду, помнит, что было до неё, собирает поправки для физики,
// ведёт журнал и по данным самой модели определяет, что именно изменилось
// после вмешательства.
// -----------------------------------------------------------------------------

// Sample — снимок фактических величин в единицах интерфейса.
//
// Ключи совпадают с полем Reads у параметров и с именами в телеметрии.
type Sample map[string]float64

// Board — пульт воздействий.
type Board struct {
	mu sync.RWMutex

	// effects — активные воздействия по ключу «адресат + параметр».
	//
	// На одну пару действует ровно одно воздействие: иначе результат зависел
	// бы от порядка обхода. Адресат позволяет остановить один турбонасос
	// из девяти, не трогая остальные.
	effects map[string]*Effect

	// failures — отказы по тому же ключу. Отказ сосуществует с числовым
	// воздействием: заклинивший клапан всё равно имеет положение.
	failures map[string]Command

	// nominals — штатные значения, откалиброванные по конфигурации двигателя.
	nominals map[string]float64

	// seen — идентификаторы уже применённых команд. Защита от повторного
	// применения при переотправке по разорванному соединению.
	seen map[string]Result

	log      []LogEntry
	trackers []*tracker
	seq      int

	// unsafeMode разрешает выход за допустимые пределы.
	unsafeMode bool

	// directMode разрешает прямую подмену состояния модели.
	directMode bool

	// history — история наблюдаемых величин текущего прогона.
	history []Point

	// baseline — сохранённый эталонный запуск для сравнения.
	baseline *Baseline

	// now — последнее известное модельное время.
	now float64
}

// NewBoard создаёт пустой пульт.
func NewBoard() *Board {
	return &Board{
		effects:  make(map[string]*Effect),
		failures: make(map[string]Command),
		nominals: make(map[string]float64),
		seen:     make(map[string]Result),
	}
}

// Calibrate задаёт штатное значение параметра по конфигурации носителя.
//
// Номинальные обороты вала или площадь горловины зависят от того, какая
// ступень сейчас активна, поэтому в реестре они лишь ориентировочные,
// а точные приходят из модели.
func (b *Board) Calibrate(nominals map[string]float64) {
	b.mu.Lock()
	defer b.mu.Unlock()
	for id, v := range nominals {
		b.nominals[id] = v
	}
}

// nominalOf возвращает штатное значение параметра в единицах модели.
func (b *Board) nominalOf(p Param) float64 {
	if v, ok := b.nominals[p.ID]; ok {
		return p.ToModel(v)
	}
	return p.ToModel(p.Nominal)
}

// SetUnsafeMode включает или выключает режим опасных воздействий.
func (b *Board) SetUnsafeMode(on bool) {
	b.mu.Lock()
	b.unsafeMode = on
	b.mu.Unlock()
}

// SetDirectMode включает или выключает прямую подмену состояния.
func (b *Board) SetDirectMode(on bool) {
	b.mu.Lock()
	b.directMode = on
	b.mu.Unlock()
}

// Modes возвращает состояние особых режимов.
func (b *Board) Modes() (unsafe, direct bool) {
	b.mu.RLock()
	defer b.mu.RUnlock()
	return b.unsafeMode, b.directMode
}

// -----------------------------------------------------------------------------
// Применение команды
// -----------------------------------------------------------------------------

// effectKey возвращает ключ воздействия: адресат и параметр.
//
// Общая команда и адресная — разные воздействия, и снимать их тоже нужно
// раздельно, иначе отмена на одном двигателе снимала бы режим со всей ступени.
func effectKey(engine, parameter string) string {
	return engine + "\x00" + parameter
}

// Apply проверяет и принимает команду.
//
// now — модельное время, sample — фактические величины на этот момент.
func (b *Board) Apply(cmd Command, now float64, sample Sample) Result {
	cmd.Normalize()

	b.mu.Lock()
	defer b.mu.Unlock()
	b.now = now

	// Повторная отправка той же команды не применяется дважды. Ответ берётся
	// из журнала: клиент, потерявший подтверждение, получит тот же результат.
	if cmd.ID != "" {
		if prev, ok := b.seen[cmd.ID]; ok {
			return prev
		}
	}

	res := Result{ID: cmd.ID, Parameter: cmd.Parameter, ModelTime: now}

	if err := cmd.Validate(); err != nil {
		res.Status, res.Reason = StatusRejected, err.Error()
		b.remember(cmd, res, nil)
		return res
	}

	// Снятие воздействия обрабатывается отдельно: параметра может уже
	// не быть среди активных.
	if cmd.Mode == ModeRelease {
		return b.release(cmd, now, res)
	}

	p, ok := registry[cmd.Parameter]
	if !ok {
		res.Status = StatusRejected
		res.Reason = fmt.Sprintf("неизвестный параметр %q", cmd.Parameter)
		b.remember(cmd, res, nil)
		return res
	}
	res.Unit = p.Unit

	if cmd.TTL > 0 && cmd.IssuedAtModelTime > 0 && now-cmd.IssuedAtModelTime > cmd.TTL {
		res.Status = StatusExpired
		res.Reason = fmt.Sprintf("команда устарела на %.1f с модельного времени",
			now-cmd.IssuedAtModelTime-cmd.TTL)
		b.remember(cmd, res, &p)
		return res
	}

	if p.Direct && !cmd.Direct {
		res.Status = StatusRejected
		res.Reason = "прямая подмена состояния требует режима Direct state override"
		b.remember(cmd, res, &p)
		return res
	}
	if cmd.Direct && !b.directMode {
		res.Status = StatusRejected
		res.Reason = "режим Direct state override выключен"
		b.remember(cmd, res, &p)
		return res
	}

	nominal := b.nominalOf(p)

	// Текущее значение берётся из телеметрии — оттуда же, откуда его видит
	// оператор. Если параметра в телеметрии нет, отсчёт идёт от активного
	// воздействия либо от номинала.
	key := effectKey(cmd.Engine, cmd.Parameter)

	current := nominal
	if e, ok := b.effects[key]; ok {
		current = e.Value(now)
	}
	if p.Reads != "" {
		if v, ok := sample[p.Reads]; ok && !math.IsNaN(v) && cmd.Basis == BasisPercentCurrent {
			current = p.ToModel(v)
		}
	}
	res.Previous = p.ToDisplay(current)

	// --- Отказ ---------------------------------------------------------------
	if cmd.Mode == ModeFailure {
		if p.fail == nil {
			res.Status = StatusRejected
			res.Reason = "для этого параметра отказы не определены"
			b.remember(cmd, res, &p)
			return res
		}
		b.failures[key] = cmd
		res.Status = StatusApplied
		res.Target = res.Previous
		res.Warnings = append(res.Warnings, failureConsequence(p, cmd.Failure))
		b.remember(cmd, res, &p)
		b.track(cmd, p, now, sample)
		return res
	}

	// --- Числовое воздействие ------------------------------------------------
	targetDisplay := cmd.resolveTarget(p.ToDisplay(current), p.ToDisplay(nominal))
	if math.IsNaN(targetDisplay) || math.IsInf(targetDisplay, 0) {
		res.Status, res.Reason = StatusRejected, "целевое значение не определено"
		b.remember(cmd, res, &p)
		return res
	}

	unsafe := targetDisplay < p.Min || targetDisplay > p.Max
	if unsafe {
		if !b.unsafeMode || !cmd.Unsafe {
			res.Status = StatusRejected
			res.Reason = fmt.Sprintf(
				"значение %.4g %s вне допустимых пределов %.4g…%.4g %s. %s "+
					"Включите режим Unsafe overrides и подтвердите команду.",
				targetDisplay, p.Unit, p.Min, p.Max, p.Unit, p.Consequence)
			b.remember(cmd, res, &p)
			return res
		}
		res.Warnings = append(res.Warnings,
			fmt.Sprintf("Значение вне допустимых пределов %.4g…%.4g %s. %s",
				p.Min, p.Max, p.Unit, p.Consequence))
	}

	if state := p.Classify(targetDisplay); state != "normal" && !unsafe {
		res.Warnings = append(res.Warnings,
			fmt.Sprintf("Целевое значение попадает в %s диапазон. %s",
				map[string]string{"warning": "предупредительный", "critical": "критический"}[state],
				p.Consequence))
	}
	if cmd.Direct {
		res.Warnings = append(res.Warnings,
			"Прямая подмена состояния: физическая причинность на этой величине "+
				"временно нарушена, значение больше не следует из модели.")
	}

	// Приоритет: воздействие с меньшим приоритетом не вытесняет активное.
	if old, ok := b.effects[key]; ok && old.Command.Priority > cmd.Priority {
		res.Status = StatusRejected
		res.Reason = fmt.Sprintf("параметр занят воздействием %q с приоритетом %d",
			old.Command.ID, old.Command.Priority)
		b.remember(cmd, res, &p)
		return res
	}

	e := &Effect{
		Command:   cmd,
		Param:     p,
		StartTime: now,
		Base:      current,
		Target:    p.ToModel(targetDisplay),
		Nominal:   nominal,
		Unsafe:    unsafe,
		current:   current,
	}
	// Скачок и уставка не имеют переходного участка: значение действует сразу.
	if cmd.Mode == ModeStep || cmd.Mode == ModeSet || cmd.Mode == ModeRelative {
		e.current = e.Target
	}
	b.effects[key] = e

	res.Status = StatusApplied
	res.Target = targetDisplay
	b.remember(cmd, res, &p)
	b.track(cmd, p, now, sample)
	return res
}

// failureConsequence описывает последствия конкретного отказа.
//
// Поле Consequence у параметра говорит о выходе величины за пределы и к отказу
// отношения не имеет: остановленный агрегат и раскрученный сверх предела — это
// разные события с разными последствиями.
func failureConsequence(p Param, f Failure) string {
	switch f {
	case FailureStopped:
		return p.Title + ": агрегат остановлен. Подача прекратится, " +
			"и двигатель погаснет из-за нехватки давления."
	case FailureStuck:
		return p.Title + ": агрегат заклинил. Обороты зафиксированы, " +
			"момент турбины уходит в трение, вибрация резко возрастает."
	case FailureUnpowered:
		return p.Title + ": агрегат обесточен. Рабочее тело на турбину " +
			"не подаётся, вал тормозится нагрузкой насосов."
	case FailureUnresponsive:
		return p.Title + ": привод не отвечает на команды. Система управления " +
			"продолжит выдавать команды, но положение не изменится."
	case FailureFrozen:
		return p.Title + ": показание зафиксировано. Физика продолжает " +
			"меняться, а телеметрия — нет."
	case FailureBadSensor:
		return p.Title + ": измерительный канал отказал. Показания больше " +
			"не отражают состояние агрегата."
	case FailureRuptured:
		return p.Title + ": магистраль разрушена. Компонент уходит за борт, " +
			"подача в двигатель прекращается."
	default:
		return p.Title + ": имитация отказа " + string(f)
	}
}

// release снимает воздействие с параметра или со всех сразу.
func (b *Board) release(cmd Command, now float64, res Result) Result {
	if cmd.Parameter == "" || cmd.Parameter == "*" {
		n := len(b.effects) + len(b.failures)
		b.effects = make(map[string]*Effect)
		b.failures = make(map[string]Command)
		res.Status = StatusApplied
		res.Reason = fmt.Sprintf("снято воздействий: %d", n)
		b.remember(cmd, res, nil)
		return res
	}

	p, ok := registry[cmd.Parameter]
	if !ok {
		res.Status, res.Reason = StatusRejected, "неизвестный параметр"
		b.remember(cmd, res, nil)
		return res
	}
	key := effectKey(cmd.Engine, cmd.Parameter)
	if e, ok := b.effects[key]; ok {
		res.Previous = p.ToDisplay(e.Value(now))
	}
	delete(b.effects, key)
	delete(b.failures, key)

	res.Status = StatusApplied
	res.Unit = p.Unit
	res.Target = p.ToDisplay(b.nominalOf(p))
	res.Reason = "параметр возвращён под управление модели"
	b.remember(cmd, res, &p)
	return res
}

// ReleaseAll снимает все воздействия и выключает особые режимы.
func (b *Board) ReleaseAll() {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.effects = make(map[string]*Effect)
	b.failures = make(map[string]Command)
	b.unsafeMode = false
	b.directMode = false
}

// ReleaseFailure снимает отказ (не числовое воздействие) с одного адресата.
//
// Отказ и числовое воздействие сосуществуют на одном ключе не просто так:
// «двигатель остановлен» и «газ на 20 %» — разные вещи, и снимать их вместе
// нельзя. Зажиганию нужно снять именно отказ: он держит камеру выключенной
// каждый шаг (ov.ForceShutdown применяется в Advance заново, а не один раз),
// и голый Start() без этого потухнет на следующем же такте, оставив
// числовую уставку оператора как есть.
func (b *Board) ReleaseFailure(parameter, engine string) {
	b.mu.Lock()
	defer b.mu.Unlock()
	delete(b.failures, effectKey(engine, parameter))
}

// ReleaseFailuresOf снимает отказы по параметру со всех адресатов сразу —
// общего и каждого поимённого. Нужен массовому зажиганию: команда «зажечь
// все камеры» обязана отменять и общий отказ ступени, и отказы, адресованные
// отдельным двигателям, а не только один из них.
func (b *Board) ReleaseFailuresOf(parameter string) {
	b.mu.Lock()
	defer b.mu.Unlock()
	for key := range b.failures {
		if paramOf(key) == parameter {
			delete(b.failures, key)
		}
	}
}

// Reset полностью очищает пульт: воздействия, журнал и наблюдатели.
//
// Вызывается при сбросе симуляции. Режим Unsafe overrides при этом выключается,
// как и требуется: опасный режим не должен переживать сброс.
func (b *Board) Reset() {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.effects = make(map[string]*Effect)
	b.failures = make(map[string]Command)
	b.seen = make(map[string]Result)
	b.trackers = nil
	b.log = nil
	b.history = nil
	b.seq = 0
	b.unsafeMode = false
	b.directMode = false
	b.now = 0
}

// -----------------------------------------------------------------------------
// Поправки для физической модели
// -----------------------------------------------------------------------------

// Advance пересчитывает воздействия на новое модельное время и возвращает
// поправки, которые физическая модель обязана учесть на этом шаге.
//
// Вызывается на каждом шаге интегрирования: плавное изменение должно идти
// по профилю, а не ступеньками между публикациями телеметрии.
func (b *Board) Advance(now float64) Overrides {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.now = now

	// Сначала собираются общие воздействия — те, что адресованы всей ступени.
	var base Overrides
	targets := make(map[string]bool)

	for key, e := range b.effects {
		v := e.Value(now)
		if e.Done {
			// Временное воздействие отработало: значение уже возвращено,
			// само воздействие снимается.
			delete(b.effects, key)
			b.finishTimed(e, now)
			continue
		}
		if e.Command.Engine != "" {
			targets[e.Command.Engine] = true
			continue
		}
		if e.Param.apply != nil {
			e.Param.apply(&base, v)
		}
		base.Active = true
		if e.Unsafe {
			base.Unsafe = true
		}
		if e.Command.Direct {
			base.DirectOverride = true
		}
	}

	for key, cmd := range b.failures {
		p, ok := registry[paramOf(key)]
		if !ok || p.fail == nil {
			continue
		}
		if cmd.Engine != "" {
			targets[cmd.Engine] = true
			continue
		}
		p.fail(&base, cmd.Failure)
		base.Active = true
	}

	if len(targets) == 0 {
		return base
	}

	// Затем для каждого адресата поверх общих накладываются его собственные.
	// Порядок важен: адресная команда должна перекрывать общую, а не наоборот.
	base.PerEngine = make(map[string]EngineOverrides, len(targets))
	for id := range targets {
		sub := base
		sub.PerEngine = nil

		for _, e := range b.effects {
			if e.Command.Engine != id || e.Param.apply == nil {
				continue
			}
			e.Param.apply(&sub, e.Current())
			base.Active = true
			if e.Unsafe {
				base.Unsafe = true
			}
			if e.Command.Direct {
				base.DirectOverride = true
			}
		}
		for key, cmd := range b.failures {
			if cmd.Engine != id {
				continue
			}
			if p, ok := registry[paramOf(key)]; ok && p.fail != nil {
				p.fail(&sub, cmd.Failure)
				base.Active = true
			}
		}

		base.PerEngine[id] = sub.Engine

		// Воздействия на баки и магистрали адресата не имеют: бак на ступени
		// один. Если адресная команда затронула бак, она применяется ко всей
		// ступени — иначе поправка потерялась бы.
		if sub.FuelTank != base.FuelTank {
			base.FuelTank = sub.FuelTank
		}
		if sub.OxTank != base.OxTank {
			base.OxTank = sub.OxTank
		}
	}

	return base
}

// paramOf извлекает идентификатор параметра из ключа воздействия.
func paramOf(key string) string {
	if i := strings.IndexByte(key, 0); i >= 0 {
		return key[i+1:]
	}
	return key
}

// Observe регистрирует отклик модели на воздействия.
//
// Отделено от Advance намеренно: поправки нужны на каждом шаге интегрирования,
// а сравнивать телеметрию имеет смысл только тогда, когда она обновилась.
func (b *Board) Observe(now float64, sample Sample) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.now = now
	b.observe(now, sample)
}

// finishTimed дописывает в журнал факт возврата параметра.
func (b *Board) finishTimed(e *Effect, now float64) {
	for i := range b.log {
		if b.log[i].ID == e.Command.ID {
			b.log[i].ReleasedAt = now
			b.log[i].Status = StatusReleased
			return
		}
	}
}

// Effects возвращает активные воздействия для интерфейса.
func (b *Board) Effects() []View {
	b.mu.RLock()
	defer b.mu.RUnlock()

	out := make([]View, 0, len(b.effects)+len(b.failures))
	for _, e := range b.effects {
		out = append(out, e.view(b.now))
	}
	for key, cmd := range b.failures {
		id := paramOf(key)
		p := registry[id]
		out = append(out, View{
			ID: cmd.ID, Parameter: id, Engine: cmd.Engine,
			Title: p.Title, Subsystem: p.Subsystem,
			Mode: ModeFailure, Failure: cmd.Failure, Unit: p.Unit,
			Nominal: p.Nominal, StartTime: b.now, Operator: cmd.Operator,
			Source: cmd.Source, Unsafe: true,
		})
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i].Subsystem != out[j].Subsystem {
			return out[i].Subsystem < out[j].Subsystem
		}
		return out[i].Parameter < out[j].Parameter
	})
	return out
}

// -----------------------------------------------------------------------------
// Журнал
// -----------------------------------------------------------------------------

// LogEntry — запись журнала вмешательств.
type LogEntry struct {
	Seq       int       `json:"seq"`
	ID        string    `json:"id"`
	ModelTime float64   `json:"modelTime"`
	RealTime  time.Time `json:"realTime"`

	Operator  string `json:"operator"`
	Source    string `json:"source"`
	Subsystem string `json:"subsystem"`
	Parameter string `json:"parameter"`
	Engine    string `json:"engine,omitempty"`
	Title     string `json:"title"`
	Unit      string `json:"unit"`

	Mode    Mode    `json:"mode"`
	Basis   Basis   `json:"basis,omitempty"`
	Curve   Curve   `json:"curve,omitempty"`
	Failure Failure `json:"failure,omitempty"`

	Previous float64 `json:"previous"`
	Target   float64 `json:"target"`
	Duration float64 `json:"duration,omitempty"`

	Status   Status   `json:"status"`
	Reason   string   `json:"reason,omitempty"`
	Warnings []string `json:"warnings,omitempty"`

	Unsafe bool `json:"unsafe"`
	Direct bool `json:"direct"`

	// ReleasedAt — модельное время снятия воздействия.
	ReleasedAt float64 `json:"releasedAt,omitempty"`

	// Reaction — отклик модели, зарегистрированный после воздействия.
	// Заполняется не сразу: нужно время, чтобы реакция проявилась.
	Reaction *Reaction `json:"reaction,omitempty"`
}

func (b *Board) remember(cmd Command, res Result, p *Param) {
	b.seq++
	entry := LogEntry{
		Seq:       b.seq,
		ID:        cmd.ID,
		ModelTime: res.ModelTime,
		RealTime:  time.Now().UTC(),
		Operator:  cmd.Operator,
		Source:    cmd.Source,
		Parameter: cmd.Parameter,
		Engine:    cmd.Engine,
		Mode:      cmd.Mode,
		Basis:     cmd.Basis,
		Curve:     cmd.Curve,
		Failure:   cmd.Failure,
		Previous:  res.Previous,
		Target:    res.Target,
		Duration:  cmd.Duration,
		Status:    res.Status,
		Reason:    res.Reason,
		Warnings:  res.Warnings,
		Unsafe:    cmd.Unsafe,
		Direct:    cmd.Direct,
	}
	if p != nil {
		entry.Subsystem = p.Subsystem
		entry.Title = p.Title
		entry.Unit = p.Unit
	}
	b.log = append(b.log, entry)

	if cmd.ID != "" {
		b.seen[cmd.ID] = res
	}

	// Журнал не должен расти без предела за долгий полёт.
	const maxLog = 5000
	if len(b.log) > maxLog {
		b.log = append([]LogEntry(nil), b.log[len(b.log)-maxLog:]...)
	}
}

// Log возвращает журнал вмешательств.
func (b *Board) Log() []LogEntry {
	b.mu.RLock()
	defer b.mu.RUnlock()
	out := make([]LogEntry, len(b.log))
	copy(out, b.log)
	return out
}
