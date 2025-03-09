package simulator

import (
	"fmt"
	"math"
	"math/rand"
	"rocketTelemetrySim/metrics"
	"time"
)

var rng = rand.New(rand.NewSource(time.Now().UnixNano()))

func RunSimulation() {
	dryMass := 250000.0
	totalMass := 549054.0
	fuelMass := totalMass - dryMass
	fuelBurnRate := 400.0
	velocity := 0.0
	altitude := 0.0
	gravity := 9.81
	area := math.Pi * math.Pow(1.83, 2)
	dragCoefficient := 0.5
	airDensitySeaLevel := 1.225

	engines := initEngines(9, 900000, 1.83)

	lowFuelThreshold := fuelMass * 0.1

	for altitude >= 0 {
		if fuelMass <= 0 {
			fuelMass = 0
			for i := range engines {
				if engines[i].Running {
					engines[i].Running = false
					fmt.Printf("🚨 Двигатель %d отключён из-за отсутствия топлива на высоте %.2f м\n", i+1, altitude)
					balanceEngines(engines, i)
				}
			}
		}

		// Если топлива осталось мало, симулируем проблемы с двигателями
		if fuelMass > 0 && fuelMass < lowFuelThreshold {
			for idx, engine := range engines {
				if engine.Running {
					chance := rand.Float64()
					if chance < 0.2 { // 20% вероятность полного отключения двигателя
						engines[idx].Running = false
						fmt.Printf("🚨 Двигатель %d вышел из строя из-за низкого топлива на высоте %.2f м\n", idx+1, altitude)
						balanceEngines(engines, idx)
					} else if chance < 0.3 { // Дополнительная 10% вероятность резкого увеличения тяги (сурж)
						surgeFactor := 1.5
						engines[idx].Thrust *= surgeFactor
						fmt.Printf("⚡ Двигатель %d испытал резкий скачок тяги из-за низкого топлива на высоте %.2f м, новая тяга: %.2f\n", idx+1, altitude, engines[idx].Thrust)
					} else if chance < 0.4 { // Дополнительная 10% вероятность резкого уменьшения тяги
						reductionFactor := 0.5
						engines[idx].Thrust *= reductionFactor
						fmt.Printf("⚠️ Двигатель %d испытал резкое снижение тяги из-за низкого топлива на высоте %.2f м, новая тяга: %.2f\n", idx+1, altitude, engines[idx].Thrust)
					}
				}
			}
		}

		// Вычисляем суммарную тягу от всех работающих двигателей
		thrust := totalThrust(engines)
		airDensity := airDensitySeaLevel * math.Exp(-altitude/8500)
		drag := 0.5 * dragCoefficient * airDensity * velocity * velocity * area

		// Текущая масса ракеты = сухая масса + оставшееся топливо
		currentMass := dryMass + fuelMass
		acceleration := (thrust - currentMass*gravity - drag) / currentMass
		velocity += acceleration
		altitude += velocity

		// Расход топлива происходит только если двигатель работает
		running := runningEngines(engines)
		fuelConsumed := fuelBurnRate * float64(running)
		if fuelMass-fuelConsumed < 0 {
			fuelConsumed = fuelMass
		}
		fuelMass -= fuelConsumed

		chanceAccident := rand.Float64()
		if chanceAccident < 0.10 && runningEngines(engines) >= 7 {
			randomIndex := rand.Intn(len(engines))
			engines[randomIndex].Running = false
			fmt.Printf("🚨 Двигатель %d отключён из-за неизвестной аварии на высоте %.2f м\n", randomIndex+1, altitude)
			balanceEngines(engines, randomIndex)
		}

		metrics.SendBasicMetrics(altitude, velocity, acceleration, currentMass, drag, airDensity, running)

		for idx, engine := range engines {
			engineID := fmt.Sprintf("%d", idx+1)
			if engine.Running {
				metrics.SetEngineThrust(engineID, engine.Thrust)
			} else {
				metrics.SetEngineThrust(engineID, 0)
			}
		}

		time.Sleep(1 * time.Second)
	}
}

func balanceEngines(engines []Engine, failedIdx int) {
	failedEngine := engines[failedIdx]

	// Коэффициенты компенсации для утраченной тяги:
	// Для противоположного двигателя – примерно 30% утраченной тяги (с небольшим случайным разбросом),
	// для соседнего – около 15% утраченной тяги.
	baseCompensationOpposite := 0.3
	baseCompensationNeighbor := 0.15

	for i := range engines {
		// Рассматриваем только работающие двигатели, кроме отключённого
		if engines[i].Running && i != failedIdx {
			if isOpposite(engines[i], failedEngine) {
				// Противоположный двигатель должен компенсировать потерю большей части тяги
				// Случайное отклонение в диапазоне ±0.05
				adjustmentFactor := baseCompensationOpposite + (rand.Float64()*0.1 - 0.05)
				delta := failedEngine.Thrust * adjustmentFactor

				// Проверка: новая тяга не должна превышать максимальную возможную для двигателя
				if engines[i].Thrust+delta > engines[i].MaxThrust {
					delta = engines[i].MaxThrust - engines[i].Thrust
				}
				engines[i].Thrust += delta
				fmt.Printf("⚠️ Engine %d thrust increased by %.2f N (total: %.2f N) for balance\n", i+1, delta, engines[i].Thrust)
			} else if isNeighbor(engines[i], failedEngine) {
				// Соседний двигатель немного уменьшает тягу, чтобы избежать избыточного крутящего момента
				adjustmentFactor := baseCompensationNeighbor + (rand.Float64()*0.05 - 0.025)
				delta := failedEngine.Thrust * adjustmentFactor

				// Проверка: новая тяга не должна опускаться ниже минимальной рабочей тяги
				if engines[i].Thrust-delta < engines[i].MinThrust {
					delta = engines[i].Thrust - engines[i].MinThrust
				}
				engines[i].Thrust -= delta
				fmt.Printf("⚠️ Engine %d thrust decreased by %.2f N (total: %.2f N) for balance\n", i+1, delta, engines[i].Thrust)
			}
		}
	}
}

func randomThrustAdjustment(min, max float64) float64 {
	value := min + rng.Float64()*(max-min)
	if rng.Intn(2) == 0 {
		return -value
	}
	return value
}

// angleBetweenEngines возвращает угол в градусах между двумя двигателями
func angleBetweenEngines(e1, e2 Engine) float64 {
	return math.Atan2(e2.Y-e1.Y, e2.X-e1.X) * (180 / math.Pi)
}

// Определяет, является ли двигатель противоположным (угол близок к 180°)
func isOpposite(e1, e2 Engine) bool {
	angle := math.Abs(angleBetweenEngines(e1, e2))
	return angle > 150 && angle <= 210
}

// Определяет, является ли двигатель соседним (угол малый, <90°)
func isNeighbor(e1, e2 Engine) bool {
	angle := math.Abs(angleBetweenEngines(e1, e2))
	return angle <= 90
}
