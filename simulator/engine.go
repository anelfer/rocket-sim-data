package simulator

import "math"

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
