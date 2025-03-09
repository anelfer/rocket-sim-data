package simulator

import (
	"fmt"
	"log"
	"math"
	"math/rand"
	"sync"
	"time"

	"rocketTelemetrySim/metrics"
)

var CurrentSimulation *Simulation

func SetCurrentSimulation(sim *Simulation) {
	CurrentSimulation = sim
}

type Simulation struct {
	Engines            []Engine
	DryMass            float64
	TotalMass          float64
	FuelMass           float64
	FuelBurnRate       float64
	Velocity           float64
	Altitude           float64
	Gravity            float64
	Area               float64
	DragCoefficient    float64
	AirDensitySeaLevel float64

	rng *rand.Rand
	mu  sync.RWMutex
}

func NewSimulation() *Simulation {
	s := &Simulation{
		Engines:            initEngines(9, 900000, 1.83),
		DryMass:            250000.0,
		TotalMass:          549054.0,
		FuelBurnRate:       400.0,
		Velocity:           0.0,
		Altitude:           0.0,
		Gravity:            9.81,
		DragCoefficient:    0.5,
		AirDensitySeaLevel: 1.225,
		rng:                rand.New(rand.NewSource(time.Now().UnixNano())),
	}
	s.FuelMass = s.TotalMass - s.DryMass
	s.Area = math.Pi * math.Pow(1.83, 2)
	return s
}

func (s *Simulation) Reset() {
	if CurrentSimulation == nil {
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	s.Engines = initEngines(9, 900000, 1.83)
	s.DryMass = 250000.0
	s.TotalMass = 549054.0
	s.FuelMass = s.TotalMass - s.DryMass
	s.FuelBurnRate = 400.0
	s.Velocity = 0.0
	s.Altitude = 0.0
	s.Gravity = 9.81
	s.Area = math.Pi * math.Pow(1.83, 2)
	s.DragCoefficient = 0.5
	s.AirDensitySeaLevel = 1.225
}

// Run запускает цикл симуляции.
func (s *Simulation) Run() {
	s.mu.Lock()
	lowFuelThreshold := s.FuelMass * 0.1
	s.mu.Unlock()

	for s.Altitude >= 0 {
		s.mu.Lock()
		// Если топлива нет, отключаем все двигатели
		if s.FuelMass <= 0 {
			s.FuelMass = 0
			for i := range s.Engines {
				if s.Engines[i].Running {
					s.Engines[i].Running = false
					log.Printf("🚨 Двигатель %d отключён из-за отсутствия топлива на высоте %.2f м\n", i+1, s.Altitude)
					balanceEngines(s.Engines, i, s.rng)
				}
			}
		}

		// Если топлива мало, симулируем непредвиденные ситуации
		if s.FuelMass > 0 && s.FuelMass < lowFuelThreshold {
			for idx, engine := range s.Engines {
				if engine.Running {
					chance := s.rng.Float64()
					if chance < 0.2 {
						s.Engines[idx].Running = false
						log.Printf("🚨 Двигатель %d вышел из строя из-за низкого топлива на высоте %.2f м\n", idx+1, s.Altitude)
						balanceEngines(s.Engines, idx, s.rng)
					} else if chance < 0.3 {
						surgeFactor := 1.5
						s.Engines[idx].Thrust *= surgeFactor
						log.Printf("⚡ Двигатель %d испытал резкий скачок тяги, новая тяга: %.2f\n", idx+1, s.Engines[idx].Thrust)
					} else if chance < 0.4 {
						reductionFactor := 0.5
						s.Engines[idx].Thrust *= reductionFactor
						log.Printf("⚠️ Двигатель %d испытал резкое снижение тяги, новая тяга: %.2f\n", idx+1, s.Engines[idx].Thrust)
					}
				}
			}
		}

		// Вычисляем динамику ракеты
		thrust := totalThrust(s.Engines)
		airDensity := s.AirDensitySeaLevel * math.Exp(-s.Altitude/8500)
		drag := 0.5 * s.DragCoefficient * airDensity * s.Velocity * s.Velocity * s.Area
		currentMass := s.DryMass + s.FuelMass
		acceleration := (thrust - currentMass*s.Gravity - drag) / currentMass
		s.Velocity += acceleration
		s.Altitude += s.Velocity

		// Расход топлива
		running := runningEngines(s.Engines)
		fuelConsumed := s.FuelBurnRate * float64(running)
		if s.FuelMass-fuelConsumed < 0 {
			fuelConsumed = s.FuelMass
		}
		s.FuelMass -= fuelConsumed

		// Случайные аварии
		if s.rng.Float64() < 0.10 && runningEngines(s.Engines) >= 7 {
			randomIndex := s.rng.Intn(len(s.Engines))
			s.Engines[randomIndex].Running = false
			log.Printf("🚨 Двигатель %d отключён из-за неизвестной аварии на высоте %.2f м\n", randomIndex+1, s.Altitude)
			balanceEngines(s.Engines, randomIndex, s.rng)
		}

		// Отправка метрик
		metrics.SendBasicMetrics(s.Altitude, s.Velocity, acceleration, currentMass, drag, airDensity, running)
		for idx, engine := range s.Engines {
			engineID := fmt.Sprintf("%d", idx+1)
			if engine.Running {
				metrics.SetEngineThrust(engineID, engine.Thrust)
			} else {
				metrics.SetEngineThrust(engineID, 0)
			}
		}
		s.mu.Unlock()

		time.Sleep(1 * time.Second)
	}
}
