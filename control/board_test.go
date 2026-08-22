package control

import (
	"math"
	"testing"
)

// -----------------------------------------------------------------------------
// Тесты пульта воздействий.
//
// Проверяется не физика — она проверяется в пакете propulsion, — а то, что
// команда корректно превращается в поправку, живёт положенное время и честно
// отчитывается о том, что произошло.
// -----------------------------------------------------------------------------

func newTestBoard() *Board {
	b := NewBoard()
	b.Calibrate(map[string]float64{"tp.shaft.speed": 32000})
	return b
}

// Уставка оборотов должна попадать в поправки, а не в телеметрию.
func TestSpeedCommandBecomesOverride(t *testing.T) {
	b := newTestBoard()

	res := b.Apply(Command{
		ID: "c1", Parameter: "tp.shaft.speed", Mode: ModeStep, Value: 28000,
	}, 10, Sample{"turbopump.rpm": 32000})

	if res.Status != StatusApplied {
		t.Fatalf("команда не применена: %s (%s)", res.Status, res.Reason)
	}
	if res.Previous != 32000 {
		t.Errorf("прежнее значение %.0f, ожидалось 32000", res.Previous)
	}

	ov := b.Advance(10)
	if !ov.Engine.Turbopump.SpeedTarget.Active {
		t.Fatal("уставка оборотов не попала в поправки")
	}
	// В поправках уставка выражена в радианах в секунду: физическая модель
	// работает в системе СИ, обороты в минуту — единица интерфейса.
	wantRad := 28000 * 2 * math.Pi / 60
	if got := ov.Engine.Turbopump.SpeedTarget.V; math.Abs(got-wantRad) > 1e-6 {
		t.Errorf("уставка %.4f рад/с, ожидалось %.4f", got, wantRad)
	}
}

// Проценты отсчитываются от фактического значения из телеметрии.
func TestRelativeCommandUsesActualValue(t *testing.T) {
	b := newTestBoard()

	res := b.Apply(Command{
		ID: "c1", Parameter: "tp.shaft.speed", Mode: ModeRelative,
		Basis: BasisPercentCurrent, Value: 10,
	}, 5, Sample{"turbopump.rpm": 30000})

	if res.Status != StatusApplied {
		t.Fatalf("команда не применена: %s", res.Reason)
	}
	if math.Abs(res.Target-33000) > 1 {
		t.Errorf("целевое значение %.0f, ожидалось 33000 (плюс десять процентов "+
			"от фактических 30000, а не от номинальных 32000)", res.Target)
	}
}

// Плавное изменение должно идти по профилю, а не скачком.
func TestRampFollowsProfile(t *testing.T) {
	b := newTestBoard()

	b.Apply(Command{
		ID: "c1", Parameter: "tp.shaft.speed", Mode: ModeRamp,
		Value: 24000, Duration: 4, Curve: CurveLinear,
	}, 0, Sample{"turbopump.rpm": 32000})

	toRPM := func(ov Overrides) float64 {
		return ov.Engine.Turbopump.SpeedTarget.V * 60 / (2 * math.Pi)
	}

	checks := []struct{ at, want float64 }{
		{0, 32000}, {1, 30000}, {2, 28000}, {3, 26000}, {4, 24000}, {6, 24000},
	}
	for _, c := range checks {
		got := toRPM(b.Advance(c.at))
		if math.Abs(got-c.want) > 1 {
			t.Errorf("на %.0f с уставка %.0f, ожидалось %.0f", c.at, got, c.want)
		}
	}
}

// Сглаженный профиль обязан начинаться и заканчиваться без рывка.
func TestSmoothCurveHasNoJump(t *testing.T) {
	b := newTestBoard()
	b.Apply(Command{
		ID: "c1", Parameter: "tp.shaft.speed", Mode: ModeRamp,
		Value: 20000, Duration: 10, Curve: CurveSmooth,
	}, 0, Sample{"turbopump.rpm": 32000})

	prev := 32000.0
	var maxRate float64
	for at := 0.0; at <= 10; at += 0.1 {
		v := b.Advance(at).Engine.Turbopump.SpeedTarget.V * 60 / (2 * math.Pi)
		maxRate = math.Max(maxRate, math.Abs(v-prev)/0.1)
		prev = v
	}
	// Средняя скорость изменения — 1200 об/мин в секунду. Сглаженный профиль
	// даёт максимум в полтора раза выше средней, ступенчатый — бесконечность.
	if maxRate > 2000 {
		t.Errorf("максимальная скорость изменения %.0f об/мин за секунду, "+
			"профиль не сглажен", maxRate)
	}
}

// Временное воздействие должно само сняться и вернуть параметр.
func TestTimedEffectReverts(t *testing.T) {
	b := newTestBoard()

	b.Apply(Command{
		ID: "c1", Parameter: "tp.shaft.speed", Mode: ModeTimed,
		Basis: BasisPercentCurrent, Value: 15, Duration: 5,
		Curve: CurvePulse, After: AfterRevertNominal,
	}, 0, Sample{"turbopump.rpm": 32000})

	if n := len(b.Effects()); n != 1 {
		t.Fatalf("активных воздействий %d, ожидалось одно", n)
	}

	// В середине импульса значение максимально.
	mid := b.Advance(2.5).Engine.Turbopump.SpeedTarget.V * 60 / (2 * math.Pi)
	if math.Abs(mid-36800) > 100 {
		t.Errorf("на середине импульса уставка %.0f, ожидалось около 36800", mid)
	}

	b.Advance(5.1)
	if n := len(b.Effects()); n != 0 {
		t.Errorf("временное воздействие не снялось: осталось %d", n)
	}

	// В журнале должен появиться след снятия.
	log := b.Log()
	if log[0].Status != StatusReleased {
		t.Errorf("статус в журнале %s, ожидался released", log[0].Status)
	}
}

// Опасные значения не применяются без явного разрешения, но и не запрещены
// окончательно: интерфейс существует ради исследования отказов.
func TestUnsafeValueRequiresExplicitMode(t *testing.T) {
	b := newTestBoard()
	cmd := Command{
		ID: "c1", Parameter: "tp.shaft.speed", Mode: ModeStep, Value: 90000,
	}

	res := b.Apply(cmd, 0, nil)
	if res.Status != StatusRejected {
		t.Fatalf("опасная команда принята без подтверждения: %s", res.Status)
	}
	if res.Reason == "" {
		t.Error("отказ без объяснения последствий")
	}

	b.SetUnsafeMode(true)
	cmd.ID, cmd.Unsafe = "c2", true
	res = b.Apply(cmd, 0, nil)
	if res.Status != StatusApplied {
		t.Fatalf("опасная команда отклонена в режиме Unsafe overrides: %s", res.Reason)
	}
	if len(res.Warnings) == 0 {
		t.Error("опасная команда применена без предупреждения")
	}
	if ov := b.Advance(0); !ov.Unsafe {
		t.Error("поправки не отмечены как опасные")
	}
}

// Прямая подмена состояния требует отдельного режима.
func TestDirectOverrideRequiresDebugMode(t *testing.T) {
	b := newTestBoard()
	cmd := Command{
		ID: "c1", Parameter: "chamber.pressure", Mode: ModeStep, Value: 9,
		Direct: true,
	}

	if res := b.Apply(cmd, 0, nil); res.Status != StatusRejected {
		t.Fatal("прямая подмена принята при выключенном отладочном режиме")
	}

	b.SetDirectMode(true)
	cmd.ID = "c2"
	res := b.Apply(cmd, 0, nil)
	if res.Status != StatusApplied {
		t.Fatalf("прямая подмена отклонена: %s", res.Reason)
	}
	if len(res.Warnings) == 0 {
		t.Error("нет предупреждения о нарушении причинности")
	}
	if ov := b.Advance(0); !ov.DirectOverride {
		t.Error("поправки не отмечены как прямая подмена")
	}
}

// Прямая подмена без режима Direct не проходит и обычной командой.
func TestDirectParameterRejectsPhysicalCommand(t *testing.T) {
	b := newTestBoard()
	b.SetDirectMode(true)
	res := b.Apply(Command{
		ID: "c1", Parameter: "tp.shaft.speed_direct", Mode: ModeStep, Value: 30000,
	}, 0, nil)
	if res.Status != StatusRejected {
		t.Error("параметр прямой подмены принят как обычная команда")
	}
}

// Повторная отправка команды не применяется дважды.
func TestCommandIsIdempotent(t *testing.T) {
	b := newTestBoard()
	cmd := Command{ID: "same", Parameter: "engine.throttle", Mode: ModeStep, Value: 0.6}

	first := b.Apply(cmd, 0, nil)
	second := b.Apply(cmd, 3, nil)

	if second.Status != first.Status || second.ModelTime != first.ModelTime {
		t.Error("повторная отправка обработана как новая команда")
	}
	if n := len(b.Log()); n != 1 {
		t.Errorf("в журнале %d записей, ожидалась одна", n)
	}
}

// Воздействие с меньшим приоритетом не вытесняет активное.
func TestPriorityProtectsActiveEffect(t *testing.T) {
	b := newTestBoard()
	b.Apply(Command{
		ID: "hi", Parameter: "engine.throttle", Mode: ModeStep,
		Value: 0.5, Priority: 10,
	}, 0, nil)

	res := b.Apply(Command{
		ID: "lo", Parameter: "engine.throttle", Mode: ModeStep,
		Value: 1, Priority: 1,
	}, 1, nil)

	if res.Status != StatusRejected {
		t.Error("низкоприоритетная команда вытеснила активное воздействие")
	}
	if v := b.Advance(1).Engine.ThrottleOverride.V; v != 0.5 {
		t.Errorf("действует значение %.2f, ожидалось 0.5", v)
	}
}

// Снятие воздействия возвращает параметр под управление модели.
func TestReleaseReturnsControl(t *testing.T) {
	b := newTestBoard()
	b.Apply(Command{ID: "c1", Parameter: "engine.throttle", Mode: ModeStep, Value: 0.4}, 0, nil)
	b.Apply(Command{ID: "c2", Parameter: "engine.throttle", Mode: ModeRelease}, 2, nil)

	if ov := b.Advance(2); ov.Engine.ThrottleOverride.Active {
		t.Error("воздействие не снято")
	}
}

// Сброс обязан выключать режим опасных воздействий.
func TestResetClearsUnsafeMode(t *testing.T) {
	b := newTestBoard()
	b.SetUnsafeMode(true)
	b.SetDirectMode(true)
	b.Reset()

	if unsafe, direct := b.Modes(); unsafe || direct {
		t.Error("особые режимы пережили сброс симуляции")
	}
	if len(b.Log()) != 0 {
		t.Error("журнал не очищен")
	}
}

// Отказ переводит агрегат в нерабочее состояние, а не подменяет показание.
func TestFailureProducesPhysicalState(t *testing.T) {
	b := newTestBoard()
	res := b.Apply(Command{
		ID: "c1", Parameter: "tp.shaft.speed", Mode: ModeFailure, Failure: FailureStuck,
	}, 0, nil)

	if res.Status != StatusApplied {
		t.Fatalf("отказ не применён: %s", res.Reason)
	}
	if ov := b.Advance(0); !ov.Engine.Turbopump.Locked {
		t.Error("заклинивание вала не попало в поправки")
	}

	b.Apply(Command{
		ID: "c2", Parameter: "tp.shaft.speed", Mode: ModeFailure, Failure: FailureStopped,
	}, 1, nil)
	if ov := b.Advance(1); !ov.Engine.Turbopump.Unpowered {
		t.Error("остановка агрегата не попала в поправки")
	}
}

// Отклик регистрируется по данным модели, а не по ожиданиям реестра.
func TestReactionIsMeasuredNotAssumed(t *testing.T) {
	b := newTestBoard()

	before := Sample{
		"turbopump.rpm":            32000,
		"turbopump.fuelPumpOutlet": 15.0,
		"chamber.pressure":         9.7,
		"engine.thrust":            845,
		"engine.fuelFlow":          85,
	}
	b.Apply(Command{
		ID: "c1", Parameter: "tp.shaft.speed", Mode: ModeStep, Value: 35000,
	}, 0, before)

	// Обороты и давление за насосом растут, тяга — тоже, а расход горючего
	// намеренно оставлен неизменным: проверяем, что пульт не припишет ему
	// реакцию только потому, что реестр считает его связанным.
	after := Sample{
		"turbopump.rpm":            35000,
		"turbopump.fuelPumpOutlet": 17.9,
		"chamber.pressure":         11.2,
		"engine.thrust":            975,
		"engine.fuelFlow":          85,
	}
	for at := 1.0; at <= 9; at++ {
		b.Observe(at, after)
	}

	entry := b.Log()[0]
	if entry.Reaction == nil {
		t.Fatal("отклик не зарегистрирован")
	}

	moved := map[string]bool{}
	for _, r := range entry.Reaction.Responses {
		moved[r.Key] = true
	}
	for _, key := range []string{"turbopump.rpm", "turbopump.fuelPumpOutlet",
		"chamber.pressure", "engine.thrust"} {
		if !moved[key] {
			t.Errorf("изменение %s не зарегистрировано", key)
		}
	}
	if moved["engine.fuelFlow"] {
		t.Error("расход горючего не менялся, но попал в отклик")
	}

	var silentFlow bool
	for _, s := range entry.Reaction.Silent {
		if s == "Расход горючего" {
			silentFlow = true
		}
	}
	if !silentFlow {
		t.Error("несработавшая ожидаемая связь должна быть показана честно, "+
			"а не умолчана; получено:", entry.Reaction.Silent)
	}
}

// Сравнение с эталоном показывает расхождение и момент его появления.
func TestBaselineComparison(t *testing.T) {
	b := newTestBoard()

	for i := 0; i <= 20; i++ {
		b.Record(float64(i), Sample{"engine.thrust": 845, "chamber.pressure": 9.7})
	}
	b.SetBaseline("Штатный полёт", 42)

	b.mu.Lock()
	b.history = nil
	b.mu.Unlock()

	for i := 0; i <= 20; i++ {
		thrust := 845.0
		if i >= 10 {
			thrust = 700
		}
		b.Record(float64(i), Sample{"engine.thrust": thrust, "chamber.pressure": 9.7})
	}

	cmp := b.Compare()
	if !cmp.Available {
		t.Fatal("сравнение недоступно при сохранённом эталоне")
	}

	var thrust *Divergence
	for i := range cmp.Rows {
		if cmp.Rows[i].Key == "engine.thrust" {
			thrust = &cmp.Rows[i]
		}
	}
	if thrust == nil {
		t.Fatal("расхождение по тяге не найдено")
	}
	if math.Abs(thrust.Delta+145) > 1 {
		t.Errorf("разница в тяге %.1f, ожидалось −145", thrust.Delta)
	}
	if thrust.DivergedAt != 10 {
		t.Errorf("расхождение отмечено на %.0f с, ожидалось на 10-й", thrust.DivergedAt)
	}
	if !thrust.OutOfRange {
		t.Error("выход за пределы эталонного диапазона не отмечен")
	}
}

// Каждый параметр реестра обязан быть пригодным к использованию.
func TestRegistryIsConsistent(t *testing.T) {
	for _, p := range Params() {
		if p.Title == "" || p.Subsystem == "" {
			t.Errorf("%s: не заполнено описание", p.ID)
		}
		if p.Min >= p.Max {
			t.Errorf("%s: пустой диапазон %.4g…%.4g", p.ID, p.Min, p.Max)
		}
		if p.CritLo > p.WarnLo || p.CritHi < p.WarnHi {
			t.Errorf("%s: предупредительный диапазон шире критического", p.ID)
		}
		if p.Nominal < p.Min || p.Nominal > p.Max {
			t.Errorf("%s: номинал %.4g вне допустимых пределов", p.ID, p.Nominal)
		}
		if p.Step <= 0 {
			t.Errorf("%s: не задан шаг изменения", p.ID)
		}
		if p.Consequence == "" {
			t.Errorf("%s: не описаны последствия выхода за пределы", p.ID)
		}
		if p.apply == nil {
			t.Errorf("%s: параметр ничего не меняет в модели", p.ID)
		}
		for _, key := range p.Affects {
			if _, ok := watchIndex[key]; !ok {
				t.Errorf("%s: связанная величина %q не наблюдается", p.ID, key)
			}
		}
		if p.Reads != "" {
			if _, ok := watchIndex[p.Reads]; !ok {
				t.Errorf("%s: источник значения %q не наблюдается", p.ID, p.Reads)
			}
		}
	}
}
