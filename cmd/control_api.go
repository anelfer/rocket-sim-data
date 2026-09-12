package main

import (
	"encoding/json"
	"fmt"
	"net/http"
	"strconv"
	"strings"

	"github.com/gorilla/mux"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// API управления симулятором.
//
// Интерфейс не имеет доступа к внутренним переменным модели. Единственный
// путь воздействия — команда, которая проходит проверку на сервере, попадает
// в физическую модель и возвращается с подтверждением. Все физические связи
// считает симулятор; на стороне интерфейса не воспроизводится ни одна формула.
// -----------------------------------------------------------------------------

// registerControlRoutes добавляет маршруты управления.
func registerControlRoutes(r *mux.Router) {
	// Профили носителей.
	r.HandleFunc("/api/profiles", profilesHandler).
		Methods(http.MethodGet, http.MethodOptions)
	r.HandleFunc("/api/profiles/select", selectProfileHandler).
		Methods(http.MethodPost, http.MethodOptions)

	// Задания на полёт.
	r.HandleFunc("/api/missions", missionsHandler).
		Methods(http.MethodGet, http.MethodOptions)
	r.HandleFunc("/api/missions/select", selectMissionHandler).
		Methods(http.MethodPost, http.MethodOptions)

	// Возвращение корабля: сход с орбиты и управление плавниками.
	r.HandleFunc("/api/entry/start", entryStartHandler).
		Methods(http.MethodPost, http.MethodOptions)
	r.HandleFunc("/api/entry/land", entryLandHandler).
		Methods(http.MethodPost, http.MethodOptions)
	r.HandleFunc("/api/entry/flap", flapHandler).
		Methods(http.MethodPost, http.MethodOptions)
	r.HandleFunc("/api/entry/attitude", attitudeHandler).
		Methods(http.MethodPost, http.MethodOptions)

	// Обводы носителя для развёртки и заправка перед пуском.
	r.HandleFunc("/api/vehicle/layout", layoutHandler).
		Methods(http.MethodGet, http.MethodOptions)
	r.HandleFunc("/api/vehicle/load", loadHandler).
		Methods(http.MethodGet, http.MethodPost, http.MethodOptions)

	// Геометрия башни-ловушки: статична, читается сценой один раз.
	r.HandleFunc("/api/catch/tower", towerHandler).
		Methods(http.MethodGet, http.MethodOptions)

	// Зажигание одной конкретной камеры (в отличие от /api/sim/ignite,
	// который запускает всю ступень).
	r.HandleFunc("/api/engines/{id}/ignite", igniteEngineHandler).
		Methods(http.MethodPost, http.MethodOptions)

	// Реестр управляемых параметров и наблюдаемых величин.
	r.HandleFunc("/api/control/registry", registryHandler).
		Methods(http.MethodGet, http.MethodOptions)

	// Полное состояние для интерфейса.
	r.HandleFunc("/api/control/state", controlStateHandler).
		Methods(http.MethodGet, http.MethodOptions)

	// Приём команд.
	r.HandleFunc("/api/control/command", commandHandler).
		Methods(http.MethodPost, http.MethodOptions)
	r.HandleFunc("/api/control/release", releaseHandler).
		Methods(http.MethodPost, http.MethodOptions)
	r.HandleFunc("/api/control/modes", modesHandler).
		Methods(http.MethodPost, http.MethodOptions)

	// Журнал, история и сравнение.
	r.HandleFunc("/api/control/log", logHandler).
		Methods(http.MethodGet, http.MethodOptions)
	r.HandleFunc("/api/control/history", historyHandler).
		Methods(http.MethodGet, http.MethodOptions)
	r.HandleFunc("/api/control/baseline", baselineHandler).
		Methods(http.MethodGet, http.MethodPost, http.MethodOptions)
	r.HandleFunc("/api/control/compare", compareHandler).
		Methods(http.MethodGet, http.MethodOptions)

	// Управление ходом симуляции.
	r.HandleFunc("/api/sim/{action}", simActionHandler).
		Methods(http.MethodPost, http.MethodOptions)

	// Поток телеметрии и команд.
	r.HandleFunc("/api/ws", websocketHandler)
}

// profilesHandler отдаёт каталог носителей.
func profilesHandler(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, map[string]any{
		"profiles": vehicle.Profiles(),
		"selected": currentProfile().ID,
	})
}

// selectProfileHandler выбирает носитель для следующего прогона.
func selectProfileHandler(w http.ResponseWriter, r *http.Request) {
	var payload struct {
		ID string `json:"id"`
	}
	if err := json.NewDecoder(r.Body).Decode(&payload); err != nil {
		http.Error(w, "некорректный формат", http.StatusBadRequest)
		return
	}
	if !setProfile(payload.ID) {
		http.Error(w, "неизвестный профиль "+payload.ID, http.StatusNotFound)
		return
	}

	p := currentProfile()
	writeJSON(w, map[string]any{
		"selected": p.ID,
		"title":    p.Title,
		"note": "Профиль применится при следующем запуске: массы, геометрия " +
			"и состав двигательной установки задаются на старте.",
	})
}

// missionsHandler отдаёт список заданий на полёт.
func missionsHandler(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, map[string]any{
		"missions": vehicle.Missions(),
		"selected": currentMission().ID,
	})
}

// selectMissionHandler выбирает задание для следующего прогона.
func selectMissionHandler(w http.ResponseWriter, r *http.Request) {
	var payload struct {
		ID string `json:"id"`
	}
	if err := json.NewDecoder(r.Body).Decode(&payload); err != nil {
		http.Error(w, "некорректный формат", http.StatusBadRequest)
		return
	}
	if !setMission(payload.ID) {
		http.Error(w, "неизвестное задание "+payload.ID, http.StatusNotFound)
		return
	}

	m := currentMission()
	writeJSON(w, map[string]any{
		"selected": m.ID,
		"title":    m.Title,
		"note": "Задание применится при следующем запуске: по нему наведение " +
			"решает, где выключать двигатели.",
	})
}

// entryStartHandler командует сход с орбиты.
func entryStartHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	if !sim.RequestEntry() {
		msg := "сход с орбиты возможен только с орбиты"
		if currentMission().Suborbital() {
			msg = "задание трансатмосферное: замкнутой орбиты нет, и сходить " +
				"не с чего. Корабль пройдёт верхнюю точку и войдёт в атмосферу сам"
		}
		http.Error(w, msg, http.StatusUnprocessableEntity)
		return
	}

	writeJSON(w, map[string]any{
		"accepted": true,
		"note": "Корабль разворачивается против движения. Двигатели включатся, " +
			"когда ошибка наведения станет меньше пяти градусов.",
	})
}

// entryLandHandler запускает посадочную программу немедленно, из любой фазы
// полёта корабля — например, после того как оператор сам поднял его
// суборбитальным прыжком и хочет отдать управление бортовому компьютеру.
func entryLandHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	if err := sim.RequestLanding(); err != nil {
		http.Error(w, err.Error(), http.StatusUnprocessableEntity)
		return
	}

	writeJSON(w, map[string]any{
		"accepted": true,
		"note": "Двигатели выключены, корабль переходит к возвращению: разворот " +
			"брюхом к потоку, торможение плавниками, посадочный манёвр на подходе " +
			"к земле.",
	})
}

// flapHandler управляет одним плавником.
func flapHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	var payload struct {
		Name    string   `json:"name"`
		Angle   *float64 `json:"angle"`
		Release bool     `json:"release"`
		Deploy  *bool    `json:"deploy"`
	}
	if err := json.NewDecoder(r.Body).Decode(&payload); err != nil {
		http.Error(w, "некорректный формат", http.StatusBadRequest)
		return
	}

	switch {
	case payload.Deploy != nil:
		if !sim.DeployFlaps(*payload.Deploy) {
			http.Error(w, "у носителя нет управляющих поверхностей",
				http.StatusUnprocessableEntity)
			return
		}
	case payload.Release:
		if !sim.ReleaseFlap(payload.Name) {
			http.Error(w, "неизвестная поверхность "+payload.Name, http.StatusNotFound)
			return
		}
	case payload.Angle != nil:
		if !sim.SetFlap(payload.Name, *payload.Angle) {
			http.Error(w, "неизвестная поверхность "+payload.Name, http.StatusNotFound)
			return
		}
	}

	writeJSON(w, sim.Snapshot().Flaps)
}

// attitudeHandler назначает ориентацию корпуса вручную.
//
// Оператор задаёт углы, а не моменты: автопилот продолжает работать, просто
// цель ему назначает человек. Корпус придёт к ней ровно настолько быстро,
// насколько позволят располагаемые органы управления, — в разрежённых слоях
// это единицы градусов в секунду.
func attitudeHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	var payload struct {
		Pitch   float64 `json:"pitch"`
		Yaw     float64 `json:"yaw"`
		Roll    float64 `json:"roll"`
		Release bool    `json:"release"`
	}
	if err := json.NewDecoder(r.Body).Decode(&payload); err != nil {
		http.Error(w, "некорректный формат", http.StatusBadRequest)
		return
	}

	if payload.Release {
		sim.ReleaseManualAttitude()
	} else {
		sim.SetManualAttitude(payload.Pitch, payload.Yaw, payload.Roll)
	}

	t := sim.Snapshot()
	writeJSON(w, map[string]any{
		"manual": !payload.Release,
		"pitch":  t.Pitch,
		"yaw":    t.Yaw,
		"roll":   t.Roll,
		"shield": t.HeatShield,
	})
}

// layoutHandler отдаёт обводы носителя.
//
// Пока ракета летит, чертёж обязан показывать именно её: телеметрия ложится
// на отсеки той машины, которая сейчас в воздухе. Как только прогон окончен,
// чертёж переключается на выбранный профиль — иначе оператор выбирает
// сверхтяжёлый носитель, а на развёртке остаётся прежняя ракета, и разница
// заметна только по числу сопел. Ровно так и выглядел дефект: после пуска
// Falcon 9 при выбранном Starship на первой ступени рисовалось девять
// двигателей вместо тридцати трёх, а на второй — один.
func layoutHandler(w http.ResponseWriter, r *http.Request) {
	if sim := simulator.GetCurrentSimulation(); sim != nil &&
		sim.Started() && !sim.Finished() {

		writeJSON(w, sim.Layout())
		return
	}
	writeJSON(w, vehicle.NewLayout(currentConfig()))
}

// towerHandler отдаёт геометрию башни-ловушки.
//
// Отдельной ручкой, а не полем в кадре сцены: геометрия неизменна за весь
// прогон, и возить её двадцать раз в секунду вместе с положением корпуса
// значило бы платить трафиком за постоянную величину.
func towerHandler(w http.ResponseWriter, r *http.Request) {
	if sim := simulator.GetCurrentSimulation(); sim != nil &&
		sim.Started() && !sim.Finished() {

		writeJSON(w, simulator.CatchTowerFor(sim.Config))
		return
	}
	writeJSON(w, simulator.CatchTowerFor(currentConfig()))
}

// loadHandler читает и меняет заправку носителя перед пуском.
//
// Заправка — это свойство изделия на старте, а не состояние полёта: залить
// компоненты в уже летящую ракету нельзя. Поэтому ползунок меняет паспорт
// следующего пуска, а не бак текущего, и модель получает ракету с той массой,
// которую задал оператор, — со всеми последствиями для тяговооружённости,
// центровки и дальности.
func loadHandler(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodPost {
		var payload struct {
			Stage    int     `json:"stage"`
			Fraction float64 `json:"fraction"`
		}
		if err := json.NewDecoder(r.Body).Decode(&payload); err != nil {
			http.Error(w, "некорректный формат", http.StatusBadRequest)
			return
		}
		if !setFuelLoad(payload.Stage, payload.Fraction) {
			http.Error(w, "заправка вне допустимых пределов", http.StatusUnprocessableEntity)
			return
		}
	}

	writeJSON(w, fuelLoadView())
}

// fuelLoadView описывает заправку в долях и тоннах.
func fuelLoadView() map[string]any {
	nominal, cfg := currentProfile().Config(), currentConfig()
	load := fuelLoads()

	stage := func(i int, planned, full vehicle.StageConfig) map[string]any {
		return map[string]any{
			"stage":    i,
			"name":     full.Name,
			"fraction": load[i-1],
			"mass":     planned.FuelMass,
			"nominal":  full.FuelMass,
			"reserve":  planned.FuelReserve,
		}
	}

	// Стартовая масса и тяговооружённость пересчитываются здесь же: оператор
	// должен видеть, что делает с ракетой недолив, ещё до нажатия «Пуск».
	mass := cfg.FirstStage.DryMass + cfg.FirstStage.FuelMass +
		cfg.SecondStage.DryMass + cfg.SecondStage.FuelMass +
		cfg.PayloadMass + cfg.FairingMass
	thrust := cfg.FirstStage.SeaLevelThust * cfg.FirstStage.EngineCount

	return map[string]any{
		"stages": []map[string]any{
			stage(1, cfg.FirstStage, nominal.FirstStage),
			stage(2, cfg.SecondStage, nominal.SecondStage),
		},
		"liftoffMass": mass,
		"twr":         thrust / (mass * 9.80665),
		"note": "Заправка применится при следующем запуске: залить компоненты " +
			"в летящую ракету нельзя.",
	}
}

// igniteEngineHandler запускает одну конкретную камеру ступени.
func igniteEngineHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	id := mux.Vars(r)["id"]
	if err := sim.IgniteEngine(id); err != nil {
		http.Error(w, err.Error(), http.StatusUnprocessableEntity)
		return
	}
	writeJSON(w, map[string]any{"status": "ignited", "engine": id})
}

func registryHandler(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, map[string]any{
		"parameters": control.Params(),
		"watches":    control.Watches(),
		"modes": []string{
			string(control.ModeSet), string(control.ModeStep),
			string(control.ModeRamp), string(control.ModeRelative),
			string(control.ModeTimed), string(control.ModeFailure),
			string(control.ModeRelease),
		},
		"curves": []string{
			string(control.CurveLinear), string(control.CurveSmooth),
			string(control.CurveStaircase), string(control.CurvePulse),
			string(control.CurvePeriodic),
		},
		"failures": []string{
			string(control.FailureStopped), string(control.FailureStuck),
			string(control.FailureUnpowered), string(control.FailureUnresponsive),
			string(control.FailureFrozen), string(control.FailureBadSensor),
			string(control.FailureRuptured),
		},
		"speeds": []float64{0.1, 0.25, 0.5, 1, 2, 5, 10, 20, 50},
	})
}

func controlStateHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}
	writeJSON(w, sim.ControlSnapshot())
}

// commandHandler принимает команду и возвращает подтверждение.
func commandHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	var cmd control.Command
	if err := json.NewDecoder(r.Body).Decode(&cmd); err != nil {
		http.Error(w, "некорректный формат команды: "+err.Error(), http.StatusBadRequest)
		return
	}

	res := applyCommand(sim, cmd)
	if res.Status == control.StatusRejected {
		w.WriteHeader(http.StatusUnprocessableEntity)
	}
	writeJSON(w, res)
}

// boosterEnginePrefix — префикс идентификатора двигателя бустера, тот же,
// что задаёт NewBooster через StagePrefix у своей PropulsionSystem.
const boosterEnginePrefix = "S1-"

// commandVehicle решает, какому аппарату адресована команда: явное поле
// Vehicle важнее угадывания по идентификатору двигателя, а идентификатор
// двигателя — единственный ориентир для старых клиентов, которые поля
// Vehicle ещё не знают.
func commandVehicle(cmd control.Command) string {
	if cmd.Vehicle != "" {
		return cmd.Vehicle
	}
	if strings.HasPrefix(cmd.Engine, boosterEnginePrefix) {
		return "booster"
	}
	return "ship"
}

// applyCommand — единая точка применения команды.
//
// Ею пользуются и REST, и поток по WebSocket: иначе проверки пришлось бы
// повторять дважды, и они неизбежно разошлись бы. Адресат — корабль или
// бустер — решается по commandVehicle: у каждого свой пульт и свой снимок
// наблюдаемых величин, команда одного не задевает другого.
func applyCommand(sim *simulator.Simulation, cmd control.Command) control.Result {
	if commandVehicle(cmd) == "booster" {
		board := sim.BoosterBoard()
		if board == nil {
			return control.Result{
				ID:     cmd.ID,
				Status: control.StatusRejected,
				Reason: "бустер сейчас не летит: адресовать команду некому",
			}
		}
		return board.Apply(cmd, sim.ModelTime(), sim.BoosterControlSample())
	}
	return sim.Board().Apply(cmd, sim.ModelTime(), sim.ControlSample())
}

func releaseHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	var payload struct {
		Parameter string `json:"parameter"`
		ID        string `json:"id"`
		Engine    string `json:"engine,omitempty"`
		Vehicle   string `json:"vehicle,omitempty"`
	}
	_ = json.NewDecoder(r.Body).Decode(&payload)

	res := applyCommand(sim, control.Command{
		ID:        payload.ID,
		Parameter: payload.Parameter,
		Engine:    payload.Engine,
		Vehicle:   payload.Vehicle,
		Mode:      control.ModeRelease,
	})
	writeJSON(w, res)
}

func modesHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	var payload struct {
		Unsafe *bool `json:"unsafe"`
		Direct *bool `json:"direct"`
	}
	if err := json.NewDecoder(r.Body).Decode(&payload); err != nil {
		http.Error(w, "некорректный формат", http.StatusBadRequest)
		return
	}

	if payload.Unsafe != nil {
		sim.Board().SetUnsafeMode(*payload.Unsafe)
	}
	if payload.Direct != nil {
		sim.Board().SetDirectMode(*payload.Direct)
	}

	unsafe, direct := sim.Board().Modes()
	writeJSON(w, map[string]bool{"unsafe": unsafe, "direct": direct})
}

func logHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}
	writeJSON(w, sim.Board().Log())
}

func historyHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}
	writeJSON(w, map[string]any{
		"current":  sim.Board().History(),
		"baseline": sim.Board().Baseline(),
	})
}

func baselineHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	if r.Method == http.MethodGet {
		writeJSON(w, sim.Board().Baseline())
		return
	}

	var payload struct {
		Label string `json:"label"`
	}
	_ = json.NewDecoder(r.Body).Decode(&payload)
	if payload.Label == "" {
		payload.Label = fmt.Sprintf("Прогон №%d", sim.RunNumber())
	}
	writeJSON(w, sim.Board().SetBaseline(payload.Label, sim.Seed()))
}

func compareHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}
	writeJSON(w, sim.Board().Compare())
}

// simActionHandler выполняет команду управления ходом симуляции.
func simActionHandler(w http.ResponseWriter, r *http.Request) {
	action := mux.Vars(r)["action"]

	// Запуск — единственное действие, доступное до создания симуляции.
	if action == "start" {
		startSimulationHandler(w, r)
		return
	}

	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

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
		steps := 1
		if v := r.URL.Query().Get("steps"); v != "" {
			if n, err := strconv.Atoi(v); err == nil {
				steps = n
			}
		}
		sim.StepForward(steps)
	case "speed":
		v, err := strconv.ParseFloat(r.URL.Query().Get("value"), 64)
		if err != nil {
			http.Error(w, "не указана скорость", http.StatusBadRequest)
			return
		}
		sim.SetScale(v)
	case "ignite":
		if err := sim.IgniteEngines(); err != nil {
			http.Error(w, err.Error(), http.StatusUnprocessableEntity)
			return
		}
	case "emergency-shutdown":
		sim.EmergencyShutdown()
	case "restore-nominal":
		sim.RestoreNominal()
	case "scenario":
		sim.SetScenario(r.URL.Query().Get("name"))
	default:
		http.Error(w, "неизвестное действие "+action, http.StatusNotFound)
		return
	}

	writeJSON(w, map[string]any{
		"action":    action,
		"runState":  sim.RunStatus(),
		"modelTime": sim.ModelTime(),
		"scale":     sim.Scale(),
		"runNumber": sim.RunNumber(),
	})
}
