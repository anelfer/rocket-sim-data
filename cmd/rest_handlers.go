package main

import (
	"encoding/json"
	"log"
	"net/http"

	"github.com/gorilla/mux"

	"rocketTelemetrySim/simulator"
)

// simulationStartChan — канал команды запуска симуляции.
var simulationStartChan = make(chan bool, 1)

func startRESTServer() {
	router := mux.NewRouter()
	router.Use(corsMiddleware)

	router.HandleFunc("/simulation/start", startSimulationHandler).Methods(http.MethodPost, http.MethodOptions)
	router.HandleFunc("/simulation/stop", stopSimulationHandler).Methods(http.MethodPost, http.MethodOptions)
	router.HandleFunc("/simulation/data", getSimulationDataHandler).Methods(http.MethodGet, http.MethodOptions)
	router.HandleFunc("/simulation/separation", separationHandler).Methods(http.MethodPost, http.MethodOptions)
	router.HandleFunc("/engines", getEnginesHandler).Methods(http.MethodGet, http.MethodOptions)
	router.HandleFunc("/engines/stop", stopEnginesHandler).Methods(http.MethodPost, http.MethodOptions)
	router.HandleFunc("/engines/start", startEnginesHandler).Methods(http.MethodPost, http.MethodOptions)
	router.HandleFunc("/engines/{id}", updateEngineHandler).Methods(http.MethodPut, http.MethodOptions)

	// Пульт ручного управления: реестр параметров, приём команд, журнал
	// и поток телеметрии.
	registerControlRoutes(router)

	// Страница «Траектория» — отдельная от пульта, со своими данными.
	registerTrajectoryRoutes(router)

	// Веб-интерфейс центра управления.
	registerUIRoutes(router)

	log.Println("REST API server running on port 8087")
	log.Fatal(http.ListenAndServe(":8087", router))
}

// corsMiddleware добавляет заголовки CORS и отвечает на preflight-запросы.
// Раньше эти пять строк были скопированы в каждый обработчик.
func corsMiddleware(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Access-Control-Allow-Origin", "*")
		w.Header().Set("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
		w.Header().Set("Access-Control-Allow-Headers", "Content-Type, Authorization")
		if r.Method == http.MethodOptions {
			w.WriteHeader(http.StatusOK)
			return
		}
		next.ServeHTTP(w, r)
	})
}

// requireSimulation возвращает активную симуляцию или отвечает 503.
// Прежние обработчики разыменовывали глобальную переменную без проверки —
// запрос до старта симуляции ронял процесс.
func requireSimulation(w http.ResponseWriter) (*simulator.Simulation, bool) {
	sim := simulator.GetCurrentSimulation()
	if sim == nil {
		http.Error(w, "simulation is not running", http.StatusServiceUnavailable)
		return nil, false
	}
	return sim, true
}

func writeJSON(w http.ResponseWriter, payload any) {
	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(payload); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
	}
}

func startSimulationHandler(w http.ResponseWriter, r *http.Request) {
	// Идущий прогон останавливается: без этого команда старта легла бы
	// в буфер канала и сработала бы неизвестно когда — либо не сработала
	// вовсе. Для оператора это выглядело как зависший интерфейс.
	stopRunningSimulation()

	select {
	case simulationStartChan <- true:
		w.WriteHeader(http.StatusAccepted)
		writeJSON(w, map[string]string{"status": "starting"})
	default:
		http.Error(w, "команда старта уже принята и ещё не обработана",
			http.StatusConflict)
	}
}

func stopSimulationHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}
	sim.Stop()
	writeJSON(w, map[string]string{"status": "stopping"})
}

func getSimulationDataHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}
	writeJSON(w, sim.Snapshot())
}

func getEnginesHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}
	writeJSON(w, sim.EngineList())
}

func stopEnginesHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}
	sim.SetAllEnginesRunning(false)
	writeJSON(w, map[string]string{"status": "all engines stopped"})
}

func startEnginesHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}
	sim.SetAllEnginesRunning(true)
	writeJSON(w, map[string]string{"status": "all engines started"})
}

func updateEngineHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	// Идентификатор двигателя — строка вида "S1-3" либо порядковый номер
	// в текущей ступени. Двигатели разных ступеней имеют разные обозначения,
	// поэтому сквозной нумерации больше нет.
	engineID := mux.Vars(r)["id"]
	if engineID == "" {
		http.Error(w, "invalid engine id", http.StatusBadRequest)
		return
	}

	var payload struct {
		Thrust  float64 `json:"thrust"`
		Running bool    `json:"running"`
	}
	if err := json.NewDecoder(r.Body).Decode(&payload); err != nil {
		http.Error(w, "invalid JSON payload", http.StatusBadRequest)
		return
	}

	if err := sim.SetEngineState(engineID, payload.Thrust, payload.Running); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	writeJSON(w, map[string]any{"status": "updated", "engine": engineID})
}

func separationHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}
	sim.ForceStageSeparation()
	writeJSON(w, map[string]string{"status": "stage separated"})
}
