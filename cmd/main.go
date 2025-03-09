package main

import (
	"log"
	"net"
	"net/http"

	"github.com/prometheus/client_golang/prometheus/promhttp"
	"rocketTelemetrySim/simulator"
)

func main() {
	http.Handle("/actuator/prometheus", promhttp.Handler())

	go func() {
		log.Println("Metrics server listening at 0.0.0.0:8086 v1.6.2")
		ln, err := net.Listen("tcp", "0.0.0.0:8086")
		if err != nil {
			log.Fatal(err)
		}
		http.Serve(ln, nil)
	}()

	simulator.RunSimulation()
}
