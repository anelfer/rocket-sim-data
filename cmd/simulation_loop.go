package main

import (
	"log"

	"rocketTelemetrySim/simulator"
)

func simulationLoop() {
	for {
		<-simulationStartChan
		log.Println("Starting simulation")
		// Создаем новую симуляцию и сохраняем её глобально
		sim := simulator.NewSimulation()
		simulator.SetCurrentSimulation(sim)
		// Запускаем симуляцию (Run блокируется, пока симуляция не завершится)
		sim.Run()
		log.Println("Simulation ended, waiting for next start command")
	}
}
