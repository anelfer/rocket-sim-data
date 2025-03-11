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

	// Параметры тепловой модели
	AmbientTemp         float64 // °C, температура окружающей среды
	Cp                  float64 // удельная теплоёмкость, J/(kg*K)
	MChamber            float64 // эффективная масса газовой смеси в камере, кг
	QFactor             float64 // коэффициент перевода тяги в тепловой поток, J/(N*s)
	CoolingCoeff        float64 // коэффициент охлаждения, W/°C
	CoolingTimeConstant float64 // временная константа охлаждения, сек.
	Dt                  float64 // шаг симуляции, сек.

	rng *rand.Rand
	mu  sync.RWMutex
}

func NewSimulation() *Simulation {
	s := &Simulation{
		Engines:            initEngines(9, 975000, 1.83, 25.0),
		DryMass:            25600.0,
		TotalMass:          549054.0,
		FuelBurnRate:       400.0,
		Velocity:           0.0,
		Altitude:           0.0,
		Gravity:            9.81,
		DragCoefficient:    0.5,
		AirDensitySeaLevel: 1.225,

		AmbientTemp:         25.0,
		Cp:                  1200.0, // J/(kg·K)
		MChamber:            10.0,   // кг (увеличили с 1 до 10 для большего теплового инерцирования)
		QFactor:             23.5,   // J/(N·s)
		CoolingCoeff:        6250.0, // W/°C
		CoolingTimeConstant: 1.0,    // сек
		Dt:                  1.0,    // сек

		rng: rand.New(rand.NewSource(time.Now().UnixNano())),
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
	s.Engines = initEngines(9, 975000, 1.83, 25.0)
	s.DryMass = 25600.0
	s.TotalMass = 549054.0
	s.FuelMass = s.TotalMass - s.DryMass
	s.FuelBurnRate = 295.0
	s.Velocity = 0.0
	s.Altitude = 0.0
	s.Gravity = 9.81
	s.Area = math.Pi * math.Pow(1.83, 2)
	s.DragCoefficient = 0.5
	s.AirDensitySeaLevel = 1.225
}

func (s *Simulation) Run() {
	s.mu.Lock()
	lowFuelThreshold := 1500.0
	s.mu.Unlock()

	for s.Altitude >= 0 {
		s.mu.Lock()

		effectiveAmbientTemp := calcEffectiveAmbientTemp(s.Altitude, s.AmbientTemp)

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
		if s.rng.Float64() < 0.00015 && runningEngines(s.Engines) >= 7 {
			randomIndex := s.rng.Intn(len(s.Engines))
			s.Engines[randomIndex].Running = false
			log.Printf("🚨 Двигатель %d отключён из-за неизвестной аварии на высоте %.2f м\n", randomIndex+1, s.Altitude)
			balanceEngines(s.Engines, randomIndex, s.rng)
		}

		for i := range s.Engines {
			s.Engines[i].UpdateThermalState(s.Dt, effectiveAmbientTemp, s.Cp, s.MChamber, s.QFactor, s.CoolingCoeff, s.CoolingTimeConstant, airDensity, s.Velocity, 0.5)
		}

		// Отправка метрик
		metrics.SendBasicMetrics(s.Altitude, s.Velocity, acceleration, currentMass, drag, airDensity, running, effectiveAmbientTemp)
		for idx, engine := range s.Engines {
			engineID := fmt.Sprintf("%d", idx+1)
			if engine.Running {
				metrics.SetEngineThrust(engineID, engine.Thrust, engine.ChamberTemp, engine.NozzleTemp, engine.WallTemp, engine.TurbineTemp)
			} else {
				metrics.SetEngineThrust(engineID, 0, engine.ChamberTemp, engine.NozzleTemp, engine.WallTemp, engine.TurbineTemp)
			}
		}
		s.mu.Unlock()

		time.Sleep(1 * time.Second)
	}
}

// calcEffectiveAmbientTemp рассчитывает эффективную температуру окружающей среды (в °C)
// в зависимости от высоты (в метрах) и базовой температуры на уровне моря (seaLevelTemp).
// Модель включает следующие слои:
//   - 0–11 км (тропосфера): температура падает линейно со скоростью 6.5°C/км.
//   - 11–20 км (нижняя стратосфера): температура фиксирована (значение на 11 км).
//   - 20–47 км (мезосфера): температура падает линейно, например, на 30°C от значения на 20 км до 47 км.
//   - 47–50 км (переходный слой): температура растёт от значения на 47 км до -60°C на 50 км.
//   - 50–100 км (нижняя термосфера): линейное увеличение от -60°C (на 50 км) до 100°C (на 100 км).
//   - 100–1000 км (термосфера): резкий экспоненциальный рост от 100°C (на 100 км)
//     до примерно 1500°C (на 1000 км).
//   - 1000–1500 км: температура уменьшается линейно от ~1500°C (на 1000 км) до 100°C (на 1500 км).
//   - 1500–10000 км: температура плавно падает от 100°C до абсолютного нуля (-273.15°C).
//   - Выше 10000 км: температура фиксируется на -273.15°C.
//
// При этом температура никогда не опускается ниже –273.15°C.
func calcEffectiveAmbientTemp(altitude, seaLevelTemp float64) float64 {
	var t float64
	if altitude < 11000 {
		t = seaLevelTemp - 6.5*(altitude/1000.0)
	} else if altitude < 20000 {
		t = seaLevelTemp - 6.5*11.0
	} else if altitude < 47000 {
		T20 := seaLevelTemp - 6.5*11.0
		t = T20 - ((altitude-20000)/(47000-20000))*30.0
	} else if altitude < 50000 {
		T20 := seaLevelTemp - 6.5*11.0
		t47 := T20 - 30.0
		t = t47 + ((altitude-47000)/(50000-47000))*((-60.0)-t47)
	} else if altitude < 100000 {
		t = -60.0 + ((altitude-50000)/(100000-50000))*(100.0-(-60.0))
	} else if altitude < 1000000 {
		ratio := (altitude - 100000) / (1000000 - 100000)
		t = 100.0 * math.Exp(2.708*ratio)
	} else if altitude < 1500000 {
		ratio := (altitude - 1000000) / (1500000 - 1000000)
		t = 1500.0 - ratio*(1500.0-100.0)
	} else if altitude < 10000000 {
		ratio := (altitude - 1500000) / (10000000 - 1500000)
		t = 100.0 - ratio*(100.0-(-273.15))
	} else {
		t = -273.15
	}

	// Добавляем шум: амплитуда зависит от диапазона высот
	var noiseAmp float64
	switch {
	case altitude < 20000:
		noiseAmp = 1.0
	case altitude < 50000:
		noiseAmp = 0.5
	case altitude < 100000:
		noiseAmp = 0.5
	case altitude < 1000000:
		noiseAmp = 1.0
	case altitude < 1500000:
		noiseAmp = 1.0
	case altitude < 10000000:
		noiseAmp = 2.0
	default:
		noiseAmp = 0.0
	}
	t += rand.NormFloat64() * noiseAmp
	if t < -273.15 {
		t = -273.15
	}
	return t
}
