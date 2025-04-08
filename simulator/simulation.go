package simulator

import (
	"fmt"
	"log"
	"math"
	"math/rand"
	"rocketTelemetrySim/simulator/orbit"
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
	VerticalVelocity   float64
	HorizontalVelocity float64
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
		VerticalVelocity:   0.0,
		HorizontalVelocity: orbit.EarthRotationSpeedAtLatitude(0),
		Altitude:           0.0,
		Gravity:            calcGravityAtAltitude(0),
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
	s.VerticalVelocity = 0.0
	s.HorizontalVelocity = orbit.EarthRotationSpeedAtLatitude(0)
	s.Altitude = 0.0
	s.Gravity = calcGravityAtAltitude(0)
	s.Area = math.Pi * math.Pow(1.83, 2)
	s.DragCoefficient = 0.5
	s.AirDensitySeaLevel = 1.225
}

func (s *Simulation) Run() {
	s.mu.Lock()
	lowFuelThreshold := 1500.0
	deltaT := 1.0
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

		// Вычисляем массу и гравитацию
		thrust := totalThrust(s.Engines)

		currentMass := s.DryMass + s.FuelMass
		s.Gravity = calcGravityAtAltitude(s.Altitude)
		gravityForce := currentMass * s.Gravity

		// Получаем текущий pitch
		pitch := orbit.ComputePitchByAltitude(s.Altitude, orbit.OrbitLEO) //s.VerticalVelocity
		pitchRad := pitch * math.Pi / 180

		// 5. Разложим тягу по осям
		thrustX := thrust * math.Cos(pitchRad)
		thrustY := thrust * math.Sin(pitchRad)

		// 6. Сопротивление воздуха
		airDensity := CalcAirDensityUSSA(s.Altitude)
		totalVelocity := math.Hypot(s.HorizontalVelocity, s.VerticalVelocity)
		drag := 0.5 * s.DragCoefficient * airDensity * totalVelocity * totalVelocity * s.Area

		// 7. Направление drag — противоположно вектору скорости
		var unitX, unitY float64
		if totalVelocity > 0 {
			unitX = s.HorizontalVelocity / totalVelocity
			unitY = s.VerticalVelocity / totalVelocity
		}
		dragX := -drag * unitX
		dragY := -drag * unitY

		// 8. Суммарные силы
		forceX := thrustX + dragX
		forceY := thrustY + dragY - gravityForce

		// 9. Ускорения по осям
		accelHorizontal := forceX / currentMass
		accelVertical := forceY / currentMass

		// 10. Обновляем скорости
		s.HorizontalVelocity += accelHorizontal * deltaT
		s.VerticalVelocity += accelVertical * deltaT

		// 11. Обновляем высоту
		s.Altitude += s.VerticalVelocity * deltaT
		if s.Altitude < 0 {
			s.Altitude = 0
			s.VerticalVelocity = 0
		}

		// Расход топлива
		var totalFuelUsed float64
		for _, engine := range s.Engines {
			if engine.Running {
				massFlow := calculateMassFlow(engine, s.Gravity) // кг/с
				totalFuelUsed += massFlow
			}
		}
		if s.FuelMass < totalFuelUsed {
			totalFuelUsed = s.FuelMass
		}
		s.FuelMass -= totalFuelUsed

		running := runningEngines(s.Engines)
		// Случайные аварии
		if s.rng.Float64() < 0.00015 && runningEngines(s.Engines) >= 7 {
			randomIndex := s.rng.Intn(len(s.Engines))
			s.Engines[randomIndex].Running = false
			log.Printf("🚨 Двигатель %d отключён из-за неизвестной аварии на высоте %.2f м\n", randomIndex+1, s.Altitude)
			balanceEngines(s.Engines, randomIndex, s.rng)
		}

		for i := range s.Engines {
			s.Engines[i].UpdateThermalState(s.Dt, effectiveAmbientTemp, s.Cp, s.MChamber, s.QFactor, s.CoolingCoeff, s.CoolingTimeConstant, airDensity, totalVelocity, 0.5)
		}

		// Отправка метрик
		metrics.SendBasicMetrics(s.Altitude, s.VerticalVelocity, s.HorizontalVelocity, accelHorizontal, accelVertical, currentMass, drag, airDensity, running, effectiveAmbientTemp, pitch)
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
// используется USSA-76
func calcEffectiveAmbientTemp(altitude, seaLevelTemp float64) float64 {
	t := KelvinToCelsius(CalcEffectiveAmbientTemp(altitude))

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

func calcGravityAtAltitude(altitude float64) float64 {
	const (
		g0 = 9.80665   // гравитация на уровне моря (м/с²)
		R  = 6371000.0 // радиус Земли в метрах
	)

	return g0 * (R / (R + altitude)) * (R / (R + altitude))
}

// Табличные значения из USSA-76 (высота в м, температура в К, плотность в кг/м³)
var altitudeTable = []float64{
	0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000,
	10000, 11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 19000,
	20000, 21000, 22000, 23000, 24000, 25000, 26000, 27000, 28000, 29000,
	30000, 40000, 50000, 60000, 70000, 80000, 90000, 100000,
}

var temperatureTable = []float64{
	288.15, 281.65, 275.15, 268.65, 262.15, 255.65, 249.15, 242.65, 236.15, 229.65,
	223.15, 216.65, 216.65, 216.65, 216.65, 216.65, 216.65, 216.65, 216.65, 216.65,
	216.65, 218.65, 220.65, 222.65, 224.65, 226.65, 228.65, 230.65, 232.65, 234.65,
	236.65, 250.35, 270.65, 247.02, 219.65, 199.15, 186.87, 195.08,
}

var densityTable = []float64{
	1.225, 1.112, 1.007, 0.9093, 0.8194, 0.7364, 0.6601, 0.59, 0.5258, 0.4671,
	0.4135, 0.3648, 0.3108, 0.2731, 0.2385, 0.2067, 0.1775, 0.1507, 0.1262, 0.1038,
	0.08891, 0.07569, 0.06407, 0.05398, 0.04533, 0.03806, 0.03209, 0.02728, 0.02303, 0.01918,
	0.01569, 0.003996, 0.001027, 0.0003097, 0.0000828, 0.0000185, 0.0000038, 0.00000079,
}

// Линейная интерполяция
func linearInterpolate(x float64, xTable, yTable []float64) float64 {
	n := len(xTable)
	var result float64

	if x <= xTable[0] {
		result = yTable[0]
	} else if x >= xTable[n-1] {
		// Экстраполяция по последнему отрезку
		dx := xTable[n-1] - xTable[n-2]
		dy := yTable[n-1] - yTable[n-2]
		slope := dy / dx
		result = yTable[n-1] + slope*(x-xTable[n-1])
	} else {
		for i := 1; i < n; i++ {
			if x < xTable[i] {
				x0, x1 := xTable[i-1], xTable[i]
				y0, y1 := yTable[i-1], yTable[i]
				t := (x - x0) / (x1 - x0)
				result = y0 + t*(y1-y0)
				break
			}
		}
	}

	// Ограничиваем снизу, чтобы не было отрицательных значений
	return math.Max(0, result)
}

// CalcEffectiveAmbientTemp Возвращает температуру в Кельвинах на заданной высоте (м)
func CalcEffectiveAmbientTemp(altitude float64) float64 {
	return linearInterpolate(altitude, altitudeTable, temperatureTable)
}

// CalcAirDensityUSSA Возвращает плотность воздуха в кг/м³ на заданной высоте (м)
func CalcAirDensityUSSA(altitude float64) float64 {
	return linearInterpolate(altitude, altitudeTable, densityTable)
}

// DragForce рассчитывает аэродинамическое сопротивление (в ньютонах)
func DragForce(altitude, velocity, area float64) float64 {
	// Получаем температуру в Кельвинах
	tempK := CalcEffectiveAmbientTemp(altitude)

	// Скорость звука (м/с)
	a := SpeedOfSound(tempK)

	// Число Маха
	mach := velocity / a

	// Коэффициент сопротивления в зависимости от числа Маха
	Cd := CdByMach(mach)

	// Плотность воздуха (кг/м³)
	rho := CalcAirDensityUSSA(altitude)

	// Формула сопротивления
	return 0.5 * Cd * rho * velocity * velocity * area
}

func calculateMassFlow(engine Engine, gravity float64) float64 {
	if !engine.Running || engine.ISP <= 0 {
		return 0
	}
	return engine.Thrust / (engine.ISP * gravity)
}
