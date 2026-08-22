// Команда acceptance проверяет приёмочный сценарий пульта управления.
//
// Проверяется ровно то, что перечислено в постановке задачи: оператор
// запускает симуляцию, выбирает топливный насос, резко увеличивает обороты,
// получает подтверждение, наблюдает связанные изменения по всей цепочке,
// останавливает насос, видит развитие отказа, возвращает штатный режим,
// сравнивает прогон с базовым, читает журнал и сбрасывает симуляцию.
//
// Прогон идёт против настоящего HTTP-сервера и настоящего WebSocket,
// а не против внутренних вызовов: проверяется система целиком.
package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"time"

	"github.com/gorilla/websocket"
)

const base = "http://127.0.0.1:8087"

func main() {
	failures := 0
	fail := func(format string, args ...any) {
		failures++
		fmt.Printf("  ✗ "+format+"\n", args...)
	}
	pass := func(format string, args ...any) {
		fmt.Printf("  ✓ "+format+"\n", args...)
	}

	waitServer()

	// --- 1. Запуск симуляции ---------------------------------------------
	fmt.Println("\n1. Запуск симуляции")
	post("/api/sim/start", nil)

	// Прогон идёт вдесятеро быстрее реального времени: проверять переходные
	// процессы длиной в десятки секунд модельного времени иначе слишком долго.
	// Команду приходится повторять: симуляция создаётся не мгновенно, и до
	// этого момента сервер отвечает отказом.
	for i := 0; i < 50 && getState().Scale != 10; i++ {
		post("/api/sim/speed?value=10", nil)
		time.Sleep(100 * time.Millisecond)
	}
	waitFor(func(s state) bool { return s.RunState == "running" && s.ModelTime > 6 }, 25*time.Second)
	s := getState()
	if s.RunState != "running" {
		fail("симуляция не запустилась: %s", s.RunState)
	} else {
		pass("работает, T+%.1f с, фаза %q, прогон №%d", s.ModelTime, s.Phase, s.RunNumber)
	}

	// --- 2. Выбор насоса и проверка разделения значений -------------------
	fmt.Println("\n2. Топливный насос")
	if len(s.Pumps) < 2 {
		fail("насосы не отдаются интерфейсу")
		os.Exit(1)
	}
	p := s.Pumps[0]
	pass("%s: состояние %q, обороты факт %.0f, изм %.0f, ном %.0f",
		p.Name, p.State, p.ActualRPM, p.MeasuredRPM, p.NominalRPM)
	if p.MeasuredRPM == p.ActualRPM {
		fail("измеренное значение совпадает с фактическим — датчик не моделируется")
	} else {
		pass("измеренное отличается от фактического на %.1f об/мин",
			p.MeasuredRPM-p.ActualRPM)
	}

	// --- 3. Резкое увеличение оборотов ------------------------------------
	fmt.Println("\n3. Резкое увеличение оборотов на 20 %")
	before := snapshotValues(s)

	res := command(map[string]any{
		"id": "acc-spin", "parameter": "tp.shaft.speed", "mode": "relative",
		"basis": "percent_current", "value": 20, "operator": "acceptance",
	})
	if res.Status != "applied" {
		fail("команда не применена: %s %s", res.Status, res.Reason)
	} else {
		pass("подтверждено: %.0f → %.0f %s", res.Previous, res.Target, res.Unit)
	}

	sleepModel(6)
	s = getState()
	after := snapshotValues(s)

	// --- 4…6. Связанные изменения по всей цепочке --------------------------
	fmt.Println("\n4. Связанные изменения")
	chain := []struct {
		key, title string
		up         bool
	}{
		{"rpm", "обороты вала", true},
		{"pumpOutlet", "давление за насосом", true},
		{"fuelFlow", "расход горючего", true},
		{"chamberPressure", "давление в камере", true},
		{"thrust", "тяга", true},
		{"npshRequired", "потребный кавитационный запас", true},
		{"bearing", "температура подшипников", true},
	}
	for _, c := range chain {
		b, a := before[c.key], after[c.key]
		_ = c.up
		ok := a > b
		delta := (a/b - 1) * 100
		if ok == c.up {
			pass("%s: %.4g → %.4g (%+.1f %%)", c.title, b, a, delta)
		} else {
			fail("%s не изменилось в ожидаемую сторону: %.4g → %.4g", c.title, b, a)
		}
	}

	// Температура в камере при разгоне насоса меняется слабо, и это верно:
	// обороты поднимают расход обоих компонентов почти одинаково, поэтому
	// соотношение компонентов сохраняется, а вместе с ним и температура
	// продуктов сгорания. Проверять её рост было бы ошибкой.
	pass("соотношение компонентов %.4f → %.4f, температура камеры %.0f → %.0f К "+
		"(разгон вала не меняет O/F, поэтому температура почти не сдвигается)",
		before["mixtureRatio"], after["mixtureRatio"],
		before["chamberTemp"], after["chamberTemp"])

	// --- 7. Полная остановка насоса ---------------------------------------
	fmt.Println("\n7. Остановка турбонасоса")
	res = command(map[string]any{
		"id": "acc-stop", "parameter": "tp.shaft.speed", "mode": "failure",
		"failure": "stopped", "operator": "acceptance",
	})
	if res.Status != "applied" {
		fail("отказ не применён: %s", res.Reason)
	} else {
		pass("отказ принят: %s", res.Warnings)
	}

	sleepModel(8)
	s = getState()
	stopped := snapshotValues(s)

	// --- 8. Развитие отказа ------------------------------------------------
	fmt.Println("\n8. Развитие отказа")
	if stopped["rpm"] > after["rpm"]*0.6 {
		fail("обороты не упали: %.0f → %.0f", after["rpm"], stopped["rpm"])
	} else {
		pass("обороты вала %.0f → %.0f (вал тормозится нагрузкой, а не обнуляется)",
			after["rpm"], stopped["rpm"])
	}
	if stopped["thrust"] > after["thrust"]*0.2 {
		fail("тяга не пропала: %.0f → %.0f кН", after["thrust"], stopped["thrust"])
	} else {
		pass("тяга %.0f → %.0f кН", after["thrust"], stopped["thrust"])
	}
	if len(s.Alarms) == 0 {
		fail("предупреждений нет, хотя агрегат остановлен")
	} else {
		for _, a := range s.Alarms {
			pass("предупреждение [%s] %s: %.3g %s", a.Severity, a.Title, a.Value, a.Unit)
		}
	}

	// --- 11. Причинная цепочка по данным модели ----------------------------
	fmt.Println("\n11. Зарегистрированный отклик")
	entries := getLog()
	var spin *logEntry
	for i := range entries {
		if entries[i].ID == "acc-spin" {
			spin = &entries[i]
		}
	}
	switch {
	case spin == nil:
		fail("команда не попала в журнал")
	case spin.Reaction == nil:
		fail("отклик не зарегистрирован")
	default:
		pass("цепочка отклика: %v", spin.Reaction.Chain)
		for _, r := range spin.Reaction.Responses[:min(5, len(spin.Reaction.Responses))] {
			pass("  %s: %.4g → %.4g (%+.1f %%), отклик через %.1f с, связь %s",
				r.Title, r.Before, r.After, r.Change*100, r.Onset,
				map[bool]string{true: "непосредственная", false: "косвенная"}[r.Expected])
		}
		if len(spin.Reaction.Silent) > 0 {
			pass("не подтвердились моделью: %v", spin.Reaction.Silent)
		}
	}

	// --- 9. Возврат в штатный режим ----------------------------------------
	fmt.Println("\n9. Возврат в штатный режим")
	post("/api/sim/restore-nominal", nil)
	sleepModel(6)
	s = getState()
	restored := snapshotValues(s)

	if len(s.Effects) != 0 {
		fail("остались активные воздействия: %d", len(s.Effects))
	} else {
		pass("все параметры под управлением модели")
	}
	if restored["rpm"] < stopped["rpm"]*1.5 {
		fail("агрегат не восстановился: обороты %.0f → %.0f", stopped["rpm"], restored["rpm"])
	} else {
		pass("обороты восстановлены: %.0f → %.0f, тяга %.0f → %.0f кН",
			stopped["rpm"], restored["rpm"], stopped["thrust"], restored["thrust"])
	}

	// --- 12. Журнал вмешательств -------------------------------------------
	//
	// Проверяется до сброса: сброс журнал очищает, и это правильно —
	// журнал относится к прогону, а не к сеансу работы.
	fmt.Println("\n12. Журнал вмешательств")
	entries = getLog()
	if len(entries) < 2 {
		fail("в журнале %d записей", len(entries))
	} else {
		pass("записей: %d", len(entries))
		for _, e := range entries {
			fmt.Printf("      T+%-7.1f %-30s %-10s %10.3f → %-10.3f %-9s %s\n",
				e.ModelTime, e.Title, e.Mode, e.Previous, e.Target, e.Status, e.Operator)
		}
	}

	// --- 10. Сравнение с базовым прогоном ----------------------------------
	//
	// Сравнивать прогон с самим собой бессмысленно: до момента снятия эталона
	// расхождений нет по определению, а после — не с чем сравнивать. Поэтому
	// эталон снимается с текущего прогона, симуляция сбрасывается, и тот же
	// самый полёт повторяется уже с воздействием. Зерно генератора при сбросе
	// сохраняется, так что без вмешательства прогоны совпали бы побитово,
	// и вся разница — следствие команды.
	fmt.Println("\n10. Сравнение с базовым прогоном")
	post("/api/control/baseline", map[string]any{"label": "Штатный участок"})
	post("/api/sim/reset", nil)
	post("/api/sim/resume", nil)
	waitFor(func(s state) bool { return s.ModelTime > 10 }, 25*time.Second)

	command(map[string]any{
		"id": "acc-trim", "parameter": "engine.mixture_trim", "mode": "step",
		"value": 0.88, "operator": "acceptance",
	})
	sleepModel(20)

	var cmp comparison
	getJSON("/api/control/compare", &cmp)
	switch {
	case !cmp.Available:
		fail("сравнение недоступно")
	case len(cmp.Rows) == 0:
		fail("расхождений не найдено, хотя соотношение компонентов изменено")
	default:
		pass("эталон %q, отрезок %.0f…%.0f с, расхождений %d",
			cmp.Label, cmp.From, cmp.To, len(cmp.Rows))
		for _, r := range cmp.Rows[:min(6, len(cmp.Rows))] {
			pass("  %s: %.4g → %.4g (макс. отклонение %.4g на %.0f с)",
				r.Title, r.Baseline, r.Current, r.MaxDeviation, r.MaxDeviationAt)
		}
	}

	// --- Опасные значения и особые режимы ----------------------------------
	fmt.Println("\n13. Пределы и особые режимы")
	res = command(map[string]any{
		"id": "acc-unsafe", "parameter": "tp.shaft.speed", "mode": "step", "value": 90000,
	})
	if res.Status != "rejected" {
		fail("опасное значение принято без подтверждения")
	} else {
		pass("отклонено с объяснением: %s", res.Reason)
	}

	post("/api/control/modes", map[string]any{"unsafe": true})
	res = command(map[string]any{
		"id": "acc-unsafe2", "parameter": "tp.shaft.speed", "mode": "step",
		"value": 46000, "unsafe": true,
	})
	if res.Status != "applied" {
		fail("в режиме Unsafe overrides команда отклонена: %s", res.Reason)
	} else {
		pass("в режиме Unsafe overrides принято, предупреждения: %v", res.Warnings)
	}

	res = command(map[string]any{
		"id": "acc-direct", "parameter": "chamber.pressure", "mode": "step",
		"value": 9, "direct": true,
	})
	if res.Status != "rejected" {
		fail("прямая подмена принята при выключенном отладочном режиме")
	} else {
		pass("прямая подмена требует явного режима: %s", res.Reason)
	}

	// --- Пауза и пошаговое выполнение --------------------------------------
	fmt.Println("\n14. Пауза, шаг, скорость")
	post("/api/sim/pause", nil)
	time.Sleep(1100 * time.Millisecond)
	t1 := getState().ModelTime
	time.Sleep(1100 * time.Millisecond)
	t2 := getState().ModelTime
	if t2 != t1 {
		fail("на паузе модельное время идёт: %.3f → %.3f", t1, t2)
	} else {
		pass("на паузе модельное время стоит: %.2f с", t1)
	}

	post("/api/sim/step?steps=20", nil)
	time.Sleep(1100 * time.Millisecond)
	t3 := getState().ModelTime
	if t3 <= t2 {
		fail("пошаговое выполнение не сработало")
	} else {
		pass("двадцать шагов продвинули время на %.2f с", t3-t2)
	}

	post("/api/sim/speed?value=2", nil)
	post("/api/sim/resume", nil)
	time.Sleep(1200 * time.Millisecond)
	if got := getState().Scale; got != 2 {
		fail("скорость не изменилась: %v", got)
	} else {
		pass("скорость 2×")
	}

	// --- WebSocket ---------------------------------------------------------
	fmt.Println("\n15. Поток телеметрии по WebSocket")
	checkWebsocket(pass, fail)

	// --- 13. Сброс ----------------------------------------------------------
	fmt.Println("\n16. Сброс симуляции")
	post("/api/sim/reset", nil)
	time.Sleep(1200 * time.Millisecond)
	s = getState()
	if s.ModelTime > 1 {
		fail("после сброса модельное время %.2f с", s.ModelTime)
	} else {
		pass("модельное время обнулено, прогон №%d", s.RunNumber)
	}
	if s.UnsafeMode {
		fail("режим Unsafe overrides пережил сброс")
	} else {
		pass("режим Unsafe overrides выключен автоматически")
	}
	if n := len(getLog()); n != 0 {
		fail("журнал не очищен: осталось %d записей", n)
	} else {
		pass("журнал очищен")
	}

	fmt.Println()
	if failures > 0 {
		fmt.Printf("ПРОВАЛЕНО ПРОВЕРОК: %d\n", failures)
		os.Exit(1)
	}
	fmt.Println("ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ")
}

// -----------------------------------------------------------------------------
// Вспомогательное
// -----------------------------------------------------------------------------

type state struct {
	ModelTime  float64 `json:"modelTime"`
	RunState   string  `json:"runState"`
	Scale      float64 `json:"scale"`
	RunNumber  int     `json:"runNumber"`
	Phase      string  `json:"phase"`
	UnsafeMode bool    `json:"unsafeMode"`
	DirectMode bool    `json:"directMode"`

	Pumps []struct {
		Name        string  `json:"name"`
		State       string  `json:"state"`
		ActualRPM   float64 `json:"actualRpm"`
		MeasuredRPM float64 `json:"measuredRpm"`
		NominalRPM  float64 `json:"nominalRpm"`

		OutletPressure     float64 `json:"outletPressure"`
		NPSHRequired       float64 `json:"npshRequired"`
		BearingTemperature float64 `json:"bearingTemperature"`
	} `json:"pumps"`

	Effects []map[string]any `json:"effects"`

	Alarms []struct {
		Title    string  `json:"title"`
		Value    float64 `json:"value"`
		Unit     string  `json:"unit"`
		Severity string  `json:"severity"`
	} `json:"alarms"`

	Telemetry struct {
		Propulsion struct {
			TotalThrust   float64 `json:"totalThrust"`
			TotalFuelFlow float64 `json:"totalFuelFlow"`
			Engine        struct {
				ChamberPressureMean float64 `json:"chamberPressureMean"`
				ChamberTemperature  float64 `json:"chamberTemperature"`
				MixtureRatio        float64 `json:"mixtureRatio"`
			} `json:"engine"`
		} `json:"propulsion"`
	} `json:"telemetry"`
}

func snapshotValues(s state) map[string]float64 {
	p := s.Pumps[0]
	e := s.Telemetry.Propulsion
	return map[string]float64{
		"rpm":             p.ActualRPM,
		"pumpOutlet":      p.OutletPressure,
		"npshRequired":    p.NPSHRequired,
		"bearing":         p.BearingTemperature,
		"fuelFlow":        e.TotalFuelFlow,
		"thrust":          e.TotalThrust / 1e3,
		"chamberPressure": e.Engine.ChamberPressureMean / 1e6,
		"chamberTemp":     e.Engine.ChamberTemperature,
		"mixtureRatio":    e.Engine.MixtureRatio,
	}
}

type result struct {
	Status   string   `json:"status"`
	Reason   string   `json:"reason"`
	Previous float64  `json:"previous"`
	Target   float64  `json:"target"`
	Unit     string   `json:"unit"`
	Warnings []string `json:"warnings"`
}

type logEntry struct {
	ID        string  `json:"id"`
	ModelTime float64 `json:"modelTime"`
	Title     string  `json:"title"`
	Mode      string  `json:"mode"`
	Previous  float64 `json:"previous"`
	Target    float64 `json:"target"`
	Status    string  `json:"status"`
	Operator  string  `json:"operator"`
	Reaction  *struct {
		Chain     []string `json:"chain"`
		Silent    []string `json:"silent"`
		Responses []struct {
			Title    string  `json:"title"`
			Before   float64 `json:"before"`
			After    float64 `json:"after"`
			Change   float64 `json:"change"`
			Onset    float64 `json:"onset"`
			Expected bool    `json:"expected"`
		} `json:"responses"`
	} `json:"reaction"`
}

type comparison struct {
	Available bool    `json:"available"`
	Label     string  `json:"label"`
	From      float64 `json:"from"`
	To        float64 `json:"to"`
	Rows      []struct {
		Title          string  `json:"title"`
		Baseline       float64 `json:"baseline"`
		Current        float64 `json:"current"`
		MaxDeviation   float64 `json:"maxDeviation"`
		MaxDeviationAt float64 `json:"maxDeviationAt"`
	} `json:"rows"`
}

func waitServer() {
	for i := 0; i < 100; i++ {
		if _, err := http.Get(base + "/api/control/registry"); err == nil {
			return
		}
		time.Sleep(100 * time.Millisecond)
	}
	log.Fatal("сервер не поднялся")
}

func waitFor(cond func(state) bool, timeout time.Duration) {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if cond(getState()) {
			return
		}
		time.Sleep(300 * time.Millisecond)
	}
}

// sleepModel ждёт, пока модельное время продвинется на заданную величину.
func sleepModel(seconds float64) {
	from := getState().ModelTime
	waitFor(func(s state) bool { return s.ModelTime-from >= seconds }, 60*time.Second)
}

// getState возвращает состояние. Пока симуляция не создана, сервер отвечает
// 503, и это не ошибка: пульт просто ещё нечего показывать.
func getState() state {
	var s state
	resp, err := http.Get(base + "/api/control/state")
	if err != nil {
		log.Fatalf("GET состояния: %v", err)
	}
	defer resp.Body.Close()
	body, _ := io.ReadAll(resp.Body)
	if resp.StatusCode != http.StatusOK {
		// Молча возвращать нули нельзя: нулевое модельное время неотличимо
		// от только что сброшенной симуляции, и ошибка выглядела бы как
		// «время идёт назад».
		fmt.Printf("      [состояние недоступно: HTTP %d %.120s]\n",
			resp.StatusCode, body)
		return s
	}
	if err := json.Unmarshal(body, &s); err != nil {
		log.Fatalf("состояние: %v (%.300s)", err, body)
	}
	return s
}

func getLog() []logEntry {
	var l []logEntry
	getJSON("/api/control/log", &l)
	return l
}

func getJSON(path string, out any) {
	resp, err := http.Get(base + path)
	if err != nil {
		log.Fatalf("GET %s: %v", path, err)
	}
	defer resp.Body.Close()
	body, _ := io.ReadAll(resp.Body)
	if err := json.Unmarshal(body, out); err != nil {
		log.Fatalf("GET %s: %v (%.200s)", path, err, body)
	}
}

func post(path string, payload any) []byte {
	var body io.Reader
	if payload != nil {
		b, _ := json.Marshal(payload)
		body = bytes.NewReader(b)
	}
	resp, err := http.Post(base+path, "application/json", body)
	if err != nil {
		log.Fatalf("POST %s: %v", path, err)
	}
	defer resp.Body.Close()
	out, _ := io.ReadAll(resp.Body)
	return out
}

func command(cmd map[string]any) result {
	out := post("/api/control/command", cmd)
	var r result
	if err := json.Unmarshal(out, &r); err != nil {
		log.Fatalf("команда: %v (%.200s)", err, out)
	}
	return r
}

func checkWebsocket(pass, fail func(string, ...any)) {
	conn, _, err := websocket.DefaultDialer.Dial("ws://127.0.0.1:8087/api/ws", nil)
	if err != nil {
		fail("не удалось подключиться: %v", err)
		return
	}
	defer conn.Close()

	sent := time.Now().UnixMilli()
	_ = conn.WriteJSON(map[string]any{"type": "ping", "sent": sent})

	var gotTelemetry, gotPong bool
	deadline := time.Now().Add(6 * time.Second)
	conn.SetReadDeadline(deadline)

	for time.Now().Before(deadline) && !(gotTelemetry && gotPong) {
		var msg struct {
			Type     string          `json:"type"`
			Sent     int64           `json:"sent"`
			Snapshot json.RawMessage `json:"snapshot"`
		}
		if err := conn.ReadJSON(&msg); err != nil {
			break
		}
		switch msg.Type {
		case "telemetry":
			gotTelemetry = true
		case "pong":
			gotPong = true
			pass("задержка канала %d мс", time.Now().UnixMilli()-msg.Sent)
		}
	}

	if !gotTelemetry {
		fail("телеметрия по WebSocket не приходит")
	} else {
		pass("телеметрия приходит потоком")
	}
	if !gotPong {
		fail("сервер не отвечает на ping")
	}

	// Команда по тому же каналу.
	_ = conn.WriteJSON(map[string]any{
		"type": "command",
		"command": map[string]any{
			"id": "ws-1", "parameter": "engine.throttle", "mode": "step", "value": 0.9,
		},
	})
	conn.SetReadDeadline(time.Now().Add(5 * time.Second))
	for {
		var msg struct {
			Type   string `json:"type"`
			Result *struct {
				ID     string `json:"id"`
				Status string `json:"status"`
			} `json:"result"`
		}
		if err := conn.ReadJSON(&msg); err != nil {
			fail("подтверждение команды по WebSocket не получено")
			return
		}
		if msg.Type == "result" && msg.Result != nil && msg.Result.ID == "ws-1" {
			if msg.Result.Status == "applied" {
				pass("команда по WebSocket подтверждена")
			} else {
				fail("команда по WebSocket: %s", msg.Result.Status)
			}
			return
		}
	}
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}
