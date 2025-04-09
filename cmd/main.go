package main

import (
	"log"
	"net"
	"net/http"

	"github.com/prometheus/client_golang/prometheus/promhttp"
)

func main() {
	// Запуск сервера метрик на порту 8086
	http.Handle("/actuator/prometheus", promhttp.Handler())
	go startMetricsServer()

	// Запуск REST API сервера на порту 8087
	go startRESTServer()

	// Запуск цикла симуляции, который ждёт команды старта
	go simulationLoop()

	// Блокировка main(), чтобы приложение не завершилось
	select {}
}

func startMetricsServer() {
	log.Println("Metrics server listening at 0.0.0.0:8086 v1.12.1")
	ln, err := net.Listen("tcp", "0.0.0.0:8086")
	if err != nil {
		log.Fatal(err)
	}
	http.Serve(ln, nil)
}
