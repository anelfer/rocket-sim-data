package main

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"strconv"

	"github.com/gorilla/mux"
	"rocketTelemetrySim/simulator"
)

// simulationStartChan – канал для команды запуска симуляции
var simulationStartChan = make(chan bool, 1)

func getEnginesHandler(w http.ResponseWriter, r *http.Request) {
	// Добавляем CORS-заголовки
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
	w.Header().Set("Access-Control-Allow-Headers", "Content-Type, Authorization")

	engines := make([]simulator.Engine, 1)
	if simulator.CurrentSimulation != nil {
		engines = simulator.CurrentSimulation.Engines
	}
	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(engines); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
	}
}

func startSimulationHandler(w http.ResponseWriter, r *http.Request) {
	// Добавляем CORS-заголовки
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
	w.Header().Set("Access-Control-Allow-Headers", "Content-Type, Authorization")

	// Обработка предварительного запроса CORS
	if r.Method == "OPTIONS" {
		w.WriteHeader(http.StatusOK)
		return
	}

	select {
	case simulationStartChan <- true:
		simulator.CurrentSimulation.Reset() // Например, сброс состояния симуляции
		w.WriteHeader(http.StatusAccepted)
		fmt.Fprintln(w, "Simulation start command accepted")
	default:
		http.Error(w, "Simulation already running or start command already issued", http.StatusConflict)
	}
}

func updateEngineHandler(w http.ResponseWriter, r *http.Request) {
	// Добавляем CORS-заголовки
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
	w.Header().Set("Access-Control-Allow-Headers", "Content-Type, Authorization")

	// Обработка предварительного запроса CORS
	if r.Method == "OPTIONS" {
		w.WriteHeader(http.StatusOK)
		return
	}

	vars := mux.Vars(r)
	idStr := vars["id"]
	engineID, err := strconv.Atoi(idStr)
	if err != nil {
		http.Error(w, "Invalid engine id", http.StatusBadRequest)
		return
	}

	// Ожидаем JSON с новыми параметрами: новая тяга и состояние двигателя
	var payload struct {
		Thrust  float64 `json:"thrust"`
		Running bool    `json:"running"`
	}
	if err := json.NewDecoder(r.Body).Decode(&payload); err != nil {
		http.Error(w, "Invalid JSON payload", http.StatusBadRequest)
		return
	}

	// Обновляем двигатель в симуляторе
	if err := simulator.UpdateEngine(engineID, payload.Thrust, payload.Running, simulator.CurrentSimulation.Engines); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	w.WriteHeader(http.StatusOK)
	fmt.Fprintf(w, "Engine %d updated successfully", engineID)
}

func getSimulationDataHandler(w http.ResponseWriter, r *http.Request) {
	// Добавляем CORS-заголовки
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
	w.Header().Set("Access-Control-Allow-Headers", "Content-Type, Authorization")

	data := simulator.CurrentSimulation.GetData()
	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(data); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
	}
}

func startRESTServer() {
	router := mux.NewRouter()
	router.HandleFunc("/simulation/start", startSimulationHandler).Methods("POST", "OPTIONS")
	router.HandleFunc("/simulation/data", getSimulationDataHandler).Methods("GET")
	router.HandleFunc("/engines", getEnginesHandler).Methods("GET")
	router.HandleFunc("/engines/{id}", updateEngineHandler).Methods("PUT", "OPTIONS")

	log.Println("REST API server running on port 8087")
	log.Fatal(http.ListenAndServe(":8087", router))
}
