package simulator

import (
	"fmt"
	"math"
	"math/rand"
)

type Engine struct {
	Thrust, MaxThrust, MinThrust float64
	Running                      bool
	X, Y                         float64
}

func initEngines(count int, thrustPerEngine float64, radius float64) []Engine {
	engines := make([]Engine, count)
	angleStep := 2 * math.Pi / float64(count)

	for i := 0; i < count; i++ {
		angle := angleStep * float64(i) // 🔥 добавлено вычисление текущего угла
		engines[i] = Engine{
			Thrust:    thrustPerEngine,
			MaxThrust: thrustPerEngine + 15000,
			MinThrust: 1000,
			Running:   true,
			X:         math.Cos(angle) * radius,
			Y:         math.Sin(angle) * radius,
		}
	}

	return engines
}

func totalThrust(engines []Engine) float64 {
	total := 0.0
	for _, e := range engines {
		if e.Running {
			total += e.Thrust
		}
	}
	return total
}

func runningEngines(engines []Engine) int {
	count := 0
	for _, e := range engines {
		if e.Running {
			count++
		}
	}
	return count
}

func UpdateEngine(id int, thrust float64, running bool, engines []Engine) error {
	if id < 1 || id > len(engines) {
		return fmt.Errorf("invalid engine id")
	}
	engines[id-1].Thrust = thrust
	engines[id-1].Running = running
	return nil
}

func balanceEngines(engines []Engine, failedIdx int, rng *rand.Rand) {
	failedEngine := engines[failedIdx]
	baseCompensationOpposite := 0.3
	baseCompensationNeighbor := 0.15

	for i := range engines {
		if engines[i].Running && i != failedIdx {
			if IsOpposite(engines[i], failedEngine) {
				adjustmentFactor := baseCompensationOpposite + (rng.Float64()*0.1 - 0.05)
				delta := failedEngine.Thrust * adjustmentFactor
				if engines[i].Thrust+delta > engines[i].MaxThrust {
					delta = engines[i].MaxThrust - engines[i].Thrust
				}
				engines[i].Thrust += delta
				fmt.Printf("⚠️ Engine %d thrust increased by %.2f N (total: %.2f N) for balance\n", i+1, delta, engines[i].Thrust)
			} else if IsNeighbor(engines[i], failedEngine) {
				adjustmentFactor := baseCompensationNeighbor + (rng.Float64()*0.05 - 0.025)
				delta := failedEngine.Thrust * adjustmentFactor
				if engines[i].Thrust-delta < engines[i].MinThrust {
					delta = engines[i].Thrust - engines[i].MinThrust
				}
				engines[i].Thrust -= delta
				fmt.Printf("⚠️ Engine %d thrust decreased by %.2f N (total: %.2f N) for balance\n", i+1, delta, engines[i].Thrust)
			}
		}
	}
}
