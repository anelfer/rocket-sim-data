package simulator

import (
	"fmt"
	"math"
	"math/rand"
)

type Engine struct {
	Thrust, MaxThrust, MinThrust float64
	ISP                          float64 // Удельный импульс (сек)
	MaxIsp                       float64 // Max Удельный импульс (сек)
	Running                      bool
	X, Y                         float64
	ChamberTemp                  float64
	NozzleTemp                   float64
	WallTemp                     float64
	TurbineTemp                  float64
}

func initEngines(count int, thrustPerEngine float64, radius float64, ambientTemp float64) []Engine {
	engines := make([]Engine, count)
	angleStep := 2 * math.Pi / float64(count)

	for i := 0; i < count; i++ {
		angle := angleStep * float64(i)
		engines[i] = Engine{
			Thrust:    thrustPerEngine,
			MaxThrust: thrustPerEngine + 500,
			MinThrust: 1000,
			ISP:       282.0,
			MaxIsp:    311.0,
			Running:   true,
			X:         math.Cos(angle) * radius,
			Y:         math.Sin(angle) * radius,
		}
		engines[i].initializeTemperature(ambientTemp)
	}

	return engines
}

// InitializeTemperature устанавливает начальные температуры двигателя равными ambientTemp.
func (e *Engine) initializeTemperature(ambientTemp float64) {
	e.ChamberTemp = ambientTemp
	e.NozzleTemp = ambientTemp
	e.WallTemp = ambientTemp
	e.TurbineTemp = ambientTemp
}

// UpdateThermalState обновляет температурные параметры двигателя согласно простой модели теплового баланса.
// Если двигатель работает, температура камеры сгорания растёт за счёт теплового потока от сгорания,
// компенсированного потерями через охлаждение. Если двигатель выключен, температура сходится к ambientTemp.
// Параметры:
//
//	dt                   — шаг симуляции (сек).
//	ambientTemp          — температура окружающей среды (°C).
//	cp                   — удельная теплоёмкость (J/(kg*K)).
//	mChamber             — эффективная масса газовой смеси в камере (кг).
//	Q_factor             — коэффициент, переводящий тягу в тепловой поток (J/(N·s)).
//	coolingCoeff         — коэффициент охлаждения (W/°C).
//	coolingTimeConstant  — временная константа для охлаждения неработающего двигателя (сек).
func (e *Engine) UpdateThermalState(dt, effectiveAmbientTemp, cp, mChamber, Q_factor, coolingCoeff, coolingTimeConstant, airDensity, velocity, shieldingFactor float64) {
	if e.Running {
		// Энергетический баланс: поступающая мощность минус потери охлаждения.
		Q_in := e.Thrust * Q_factor
		Q_out := coolingCoeff * (e.ChamberTemp - effectiveAmbientTemp)
		dT := (Q_in - Q_out) * dt / (mChamber * cp)
		e.ChamberTemp += dT
	} else {
		// Экспоненциальное охлаждение: температура асимптотически стремится к effectiveAmbientTemp.
		offlineCoolingMultiplier := calculateOfflineCoolingMultiplier(airDensity, velocity, shieldingFactor)
		decay := math.Exp(-dt / (coolingTimeConstant * offlineCoolingMultiplier))
		e.ChamberTemp = effectiveAmbientTemp + (e.ChamberTemp-effectiveAmbientTemp)*decay
	}

	// Вычисление температуры выходящих газов (NozzleTemp).
	// Здесь используем коэффициент, отражающий эффект изотермического расширения.
	nozzleFactor := 0.93
	e.NozzleTemp = effectiveAmbientTemp + (e.ChamberTemp-effectiveAmbientTemp)*nozzleFactor

	effectiveNozzleTemp := e.NozzleTemp * (0.2 + 0.3*shieldingFactor)

	// Расчет температуры стенок (WallTemp) с учетом нагрева от горячих газов и активного охлаждения.
	// Целевая температура стенок T_targetWall установлена на 1500°C.
	T_targetWall := 1500.0
	tauHeating := 5.0 // временная константа нагрева стенок
	tauCooling := 2.0 // временная константа охлаждения стенок
	heatTerm := (effectiveNozzleTemp - e.WallTemp) / tauHeating
	coolTerm := (e.WallTemp - T_targetWall) / tauCooling
	e.WallTemp += dt * (heatTerm - coolTerm)

	// Температура турбины (TurbineTemp): ниже, чем в камере, из-за дополнительного охлаждения топлива.
	turbineFactor := 0.3
	e.TurbineTemp = effectiveAmbientTemp + (e.ChamberTemp-effectiveAmbientTemp)*turbineFactor
}

func calculateOfflineCoolingMultiplier(airDensity, velocity, shieldingFactor float64) float64 {
	baseMultiplier := 5.0
	alpha := 0.01
	// Применяем shieldingFactor к convectiveFactor
	convectiveFactor := alpha * airDensity * velocity * shieldingFactor
	multiplier := baseMultiplier / (1 + convectiveFactor)
	if multiplier < 1.0 {
		multiplier = 1.0
	}
	return multiplier
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

func TotalThrust(engines []Engine) float64 {
	total := 0.0
	for _, e := range engines {
		if e.Running {
			total += e.Thrust
		}
	}
	return total
}

func MaxTotalThrust(engines []Engine) float64 {
	total := 0.0
	for _, e := range engines {
		if e.Running {
			total += e.MaxThrust
		}
	}
	return total
}

// AngleBetweenEngines возвращает угол в градусах между двумя двигателями.
func AngleBetweenEngines(e1, e2 Engine) float64 {
	return math.Atan2(e2.Y-e1.Y, e2.X-e1.X) * (180 / math.Pi)
}

// IsOpposite определяет, является ли двигатель противоположным (угол ~180°).
func IsOpposite(e1, e2 Engine) bool {
	angle := math.Abs(AngleBetweenEngines(e1, e2))
	return angle > 150 && angle <= 210
}

// IsNeighbor определяет, является ли двигатель соседним (угол <90°).
func IsNeighbor(e1, e2 Engine) bool {
	angle := math.Abs(AngleBetweenEngines(e1, e2))
	return angle <= 90
}
