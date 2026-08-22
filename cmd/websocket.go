package main

import (
	"log"
	"net/http"
	"sync"
	"time"

	"github.com/gorilla/websocket"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator"
)

// -----------------------------------------------------------------------------
// Двусторонний канал телеметрии и команд.
//
// Телеметрия идёт потоком, команды — навстречу. Каждая команда несёт
// идентификатор, по которому клиент сопоставляет ответ и не применяет
// команду дважды при переподключении.
// -----------------------------------------------------------------------------

var upgrader = websocket.Upgrader{
	ReadBufferSize:  4096,
	WriteBufferSize: 65536,

	// Интерфейс раздаётся тем же сервером, но может быть открыт и с другого
	// адреса при разработке. Симулятор не хранит секретов, поэтому проверка
	// источника здесь избыточна.
	CheckOrigin: func(r *http.Request) bool { return true },
}

// clientMessage — сообщение от интерфейса.
type clientMessage struct {
	Type string `json:"type"`

	// Для type = "command".
	Command control.Command `json:"command,omitempty"`

	// Для type = "ping": метка времени клиента, возвращается без изменений.
	Sent int64 `json:"sent,omitempty"`

	// Для type = "action": управление ходом симуляции.
	Action string  `json:"action,omitempty"`
	Value  float64 `json:"value,omitempty"`
}

// serverMessage — сообщение интерфейсу.
type serverMessage struct {
	Type string `json:"type"`

	Snapshot *simulator.ControlSnapshot `json:"snapshot,omitempty"`
	Scene    *simulator.SceneFrame      `json:"scene,omitempty"`
	Result   *control.Result            `json:"result,omitempty"`
	Log      []control.LogEntry         `json:"log,omitempty"`
	Sent     int64                      `json:"sent,omitempty"`
	ServerAt int64                      `json:"serverAt,omitempty"`
	Error    string                     `json:"error,omitempty"`
}

// Периоды отправки. Модель публикует состояние каждые пятьдесят миллисекунд,
// и кадр сцены уходит с тем же темпом: чаще слать нечего.
//
// Полный снимок идёт вдвое реже, и это не экономия ради экономии. Он весит
// восемьдесят килобайт и стоит трети миллисекунды на сборку в JSON — на каждого
// подключённого. Двадцать раз в секунду это полтора мегабайта и почти
// десятая доля ядра на одного зрителя, тогда как приборные значения быстрее
// десяти герц глазом всё равно не читаются. Плавность нужна только картинке,
// и она её получает отдельным лёгким сообщением.
//
// Прежде здесь стояло двести пятьдесят миллисекунд при публикации раз
// в секунду: пульт получал четыре сообщения в секунду, но новые данные
// приходили только в каждом четвёртом.
const (
	sceneInterval     = 50 * time.Millisecond
	telemetryInterval = 100 * time.Millisecond
)

func websocketHandler(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("websocket: не удалось установить соединение: %v", err)
		return
	}
	defer conn.Close()

	// Запись из двух горутин недопустима, поэтому все отправки идут
	// через один мьютекс.
	var writeMu sync.Mutex
	send := func(msg serverMessage) error {
		msg.ServerAt = time.Now().UnixMilli()
		writeMu.Lock()
		defer writeMu.Unlock()
		return conn.WriteJSON(msg)
	}

	done := make(chan struct{})
	var once sync.Once
	closeDone := func() { once.Do(func() { close(done) }) }

	// Поток телеметрии.
	//
	// Таймер один, на частом периоде. Полный снимок уходит через такт, а между
	// ними — кадр сцены. Второй таймер здесь был бы хуже: два независимых
	// периода в пятьдесят и сто миллисекунд расходятся по фазе, и тогда часть
	// кадров приходит слипшимися парами, а часть — с двойным промежутком.
	// Картинка от такой неравномерности дёргается ровно так же, как от нехватки
	// кадров.
	go func() {
		ticker := time.NewTicker(sceneInterval)
		defer ticker.Stop()

		every := int(telemetryInterval / sceneInterval)
		if every < 1 {
			every = 1
		}

		lastLogSeq, tick := 0, 0
		for {
			select {
			case <-done:
				return
			case <-ticker.C:
			}
			tick++

			sim := simulator.GetCurrentSimulation()
			if sim == nil {
				// Сообщать о простое двадцать раз в секунду незачем.
				if tick%every != 0 {
					continue
				}
				if err := send(serverMessage{Type: "idle"}); err != nil {
					closeDone()
					return
				}
				continue
			}

			// На промежуточных тактах — только кадр сцены. Слать его вместе
			// с полным снимком не нужно: тот несёт то же состояние целиком.
			if tick%every != 0 {
				frame := sim.SceneFrame()
				if err := send(serverMessage{Type: "scene", Scene: &frame}); err != nil {
					closeDone()
					return
				}
				continue
			}

			snap := sim.ControlSnapshot()
			if err := send(serverMessage{Type: "telemetry", Snapshot: &snap}); err != nil {
				closeDone()
				return
			}

			// Журнал досылается порциями: отклик на команду появляется
			// не сразу, и запись обновляется задним числом.
			entries := sim.Board().Log()
			if len(entries) > 0 {
				last := entries[len(entries)-1].Seq
				if last != lastLogSeq {
					lastLogSeq = last
					if err := send(serverMessage{Type: "log", Log: entries}); err != nil {
						closeDone()
						return
					}
				}
			}
		}
	}()

	// Приём команд.
	for {
		var msg clientMessage
		if err := conn.ReadJSON(&msg); err != nil {
			closeDone()
			if !websocket.IsCloseError(err, websocket.CloseNormalClosure,
				websocket.CloseGoingAway) {
				log.Printf("websocket: соединение закрыто: %v", err)
			}
			return
		}

		switch msg.Type {
		case "ping":
			// Эхо для оценки задержки: клиент сам считает время оборота.
			_ = send(serverMessage{Type: "pong", Sent: msg.Sent})

		case "command":
			sim := simulator.GetCurrentSimulation()
			if sim == nil {
				_ = send(serverMessage{Type: "result", Result: &control.Result{
					ID:     msg.Command.ID,
					Status: control.StatusRejected,
					Reason: "симуляция не запущена",
				}})
				continue
			}
			res := applyCommand(sim, msg.Command)
			_ = send(serverMessage{Type: "result", Result: &res})

		case "action":
			sim := simulator.GetCurrentSimulation()
			if sim == nil {
				_ = send(serverMessage{Type: "error", Error: "симуляция не запущена"})
				continue
			}
			if err := runAction(sim, msg.Action, msg.Value); err != nil {
				_ = send(serverMessage{Type: "error", Error: err.Error()})
			}

		default:
			_ = send(serverMessage{Type: "error",
				Error: "неизвестный тип сообщения " + msg.Type})
		}
	}
}

// runAction выполняет команду управления ходом симуляции.
func runAction(sim *simulator.Simulation, action string, value float64) error {
	switch action {
	case "pause":
		sim.Pause()
	case "resume":
		sim.Resume()
	case "stop":
		sim.Stop()
	case "reset", "restart":
		sim.ResetRun()
	case "step":
		steps := int(value)
		if steps <= 0 {
			steps = 1
		}
		sim.StepForward(steps)
	case "speed":
		sim.SetScale(value)
	case "ignite":
		return sim.IgniteEngines()
	case "emergency-shutdown":
		sim.EmergencyShutdown()
	case "restore-nominal":
		sim.RestoreNominal()
	default:
		return errUnknownAction{action}
	}
	return nil
}

type errUnknownAction struct{ action string }

func (e errUnknownAction) Error() string {
	return "неизвестное действие " + e.action
}
