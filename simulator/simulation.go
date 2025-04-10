package simulator

import (
	"fmt"
	"log"
	"math"
	"math/rand"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/utils"
	"sync"
	"time"

	"rocketTelemetrySim/metrics"
)

var CurrentSimulation *Simulation
var Lat = 45.9647
var Lon = 63.3050
var Heading = 90.0 // восток

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

	LastDrag            float64
	LastAccelVertical   float64
	LastAccelHorizontal float64

	// Параметры тепловой модели
	AmbientTemp         float64 // °C, температура окружающей среды
	Cp                  float64 // удельная теплоёмкость, J/(kg*K)
	MChamber            float64 // эффективная масса газовой смеси в камере, кг
	QFactor             float64 // коэффициент перевода тяги в тепловой поток, J/(N*s)
	CoolingCoeff        float64 // коэффициент охлаждения, W/°C
	CoolingTimeConstant float64 // временная константа охлаждения, сек.
	Dt                  float64 // шаг симуляции, сек.

	Lat     float64
	Lon     float64
	Heading float64

	rng *rand.Rand
	mu  sync.RWMutex
}

func NewSimulation() *Simulation {
	s := &Simulation{
		Engines:            initEngines(9, 975000, 1.83, 25.0),
		DryMass:            25600.0,
		TotalMass:          549054.0,
		VerticalVelocity:   0.0,
		HorizontalVelocity: orbit.EarthRotationSpeedAtLatitude(45.9647),
		Altitude:           0.0,
		Gravity:            utils.CalcGravityAtAltitude(0),
		DragCoefficient:    0.5,
		AirDensitySeaLevel: 1.225,

		AmbientTemp:         25.0,
		Cp:                  1200.0, // J/(kg·K)
		MChamber:            10.0,   // кг (увеличили с 1 до 10 для большего теплового инерцирования)
		QFactor:             23.5,   // J/(N·s)
		CoolingCoeff:        6250.0, // W/°C
		CoolingTimeConstant: 1.0,    // сек
		Dt:                  1.0,    // сек

		Lat:     45.9647,
		Lon:     63.3050,
		Heading: 90,

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
	s.Gravity = utils.CalcGravityAtAltitude(0)
	s.Area = math.Pi * math.Pow(1.83, 2)
	s.DragCoefficient = 0.5
	s.AirDensitySeaLevel = 1.225
}

func (s *Simulation) Run() {
	s.mu.Lock()
	lowFuelThreshold := 1500.0
	deltaT := 1.0
	stagesSeparated := false
	noseFairingDestack := false
	timeV := 0.0
	s.mu.Unlock()

	for i, engine := range s.Engines {
		if engine.Running {
			engine.Thrust = 845000
			s.Engines[i] = engine
		}
	}

	// Предварительно можно определить константы для событий при низком уровне топлива:
	const (
		lowFuelFailureProb = 0.2 // вероятность отказа двигателя при низком топливе
		surgeProb          = 0.3 // вероятность резкого скачка тяги
		reductionProb      = 0.4 // вероятность резкого снижения тяги
		smoothingFactor    = 0.1 // коэффициент сглаживания для drag
	)

	orbitType := orbit.OrbitLEO
	targetOrbitAltitude := orbitType.GetTargetOrbitAltitude()
	guidance := orbit.NewGuidanceState(targetOrbitAltitude)

	for {
		s.mu.Lock()

		// 1. Вычисляем эффективную температуру (единожды на итерацию)
		effectiveAmbientTemp := calcEffectiveAmbientTemp(s.Altitude, s.AmbientTemp)

		// 2. Обработка ситуаций, связанных с недостатком топлива
		if s.FuelMass <= 0 {
			// Если топлива нет, отключаем все двигатели
			s.FuelMass = 0
			for i := range s.Engines {
				if s.Engines[i].Running {
					s.Engines[i].Running = false
					log.Printf("🚨 Двигатель %d отключён из-за отсутствия топлива на высоте %.2f м\n", i+1, s.Altitude)
				}
			}
			// Можно выполнить балансировку один раз (если balanceEngines умеет обрабатывать группу)
			//balanceEngines(s.Engines, -1, s.rng)
		} else if s.FuelMass < lowFuelThreshold {
			// При низком топливе — возможны непредвиденные ситуации у работающих двигателей
			for idx, engine := range s.Engines {
				if engine.Running {
					chance := s.rng.Float64()
					if chance < lowFuelFailureProb {
						s.Engines[idx].Running = false
						log.Printf("🚨 Двигатель %d вышел из строя из-за низкого топлива на высоте %.2f м\n", idx+1, s.Altitude)
					} else if chance < surgeProb {
						// Резкий скачок тяги
						surgeFactor := 1.5
						s.Engines[idx].Thrust *= surgeFactor
						log.Printf("⚡ Двигатель %d испытал резкий скачок тяги, новая тяга: %.2f\n", idx+1, s.Engines[idx].Thrust)
					} else if chance < reductionProb {
						// Резкое снижение тяги
						reductionFactor := 0.5
						s.Engines[idx].Thrust *= reductionFactor
						log.Printf("⚠️ Двигатель %d испытал резкое снижение тяги, новая тяга: %.2f\n", idx+1, s.Engines[idx].Thrust)
					}
				}
			}
			// Балансировка для двигателей, отключённых в результате событий низкого топлива
			for idx, engine := range s.Engines {
				if !engine.Running {
					balanceEngines(s.Engines, idx, s.rng)
				}
			}
		}

		// 3. Расчёт массы (текущая масса используется для динамических расчётов)
		currentMass := s.DryMass + s.FuelMass
		s.TotalMass = currentMass

		// 4. Гравитация и центростремительное ускорение
		rawGravity := utils.CalcGravityAtAltitude(s.Altitude)
		radius := orbit.EarthRadius + s.Altitude
		centripetalAccel := (s.HorizontalVelocity * s.HorizontalVelocity) / radius
		effectiveGravity := rawGravity - centripetalAccel
		if effectiveGravity < 0 {
			effectiveGravity = 0
		}
		s.Gravity = effectiveGravity
		gravityForce := currentMass * s.Gravity

		// 5. Определяем текущий угол pitch
		pitch := guidance.ComputePitchAuto(s.Altitude, s.VerticalVelocity, s.HorizontalVelocity)
		pitchRad := pitch * math.Pi / 180

		// 6. Расчёт суммарной тяги по всем работающим двигателям
		totalThrust := TotalThrust(s.Engines)
		thrustX := totalThrust * math.Cos(pitchRad)
		thrustY := totalThrust * math.Sin(pitchRad)

		// 7. Аэродинамические силы
		airDensity := CalcAirDensityUSSA(s.Altitude)
		totalVelocity := math.Hypot(s.HorizontalVelocity, s.VerticalVelocity)
		// Используем ранее вычисленную эффективную температуру
		cD := utils.CdByVelocity(totalVelocity, effectiveAmbientTemp)
		targetDrag := 0.5 * cD * airDensity * totalVelocity * totalVelocity * s.Area
		// Сглаживаем изменение drag для избежания резких скачков
		s.LastDrag += (targetDrag - s.LastDrag) * smoothingFactor
		dragForce := s.LastDrag

		var unitX, unitY float64
		if totalVelocity > 0 {
			unitX = s.HorizontalVelocity / totalVelocity
			unitY = s.VerticalVelocity / totalVelocity
		}
		dragX := -dragForce * unitX
		dragY := -dragForce * unitY

		// 8. Суммарная сила и вычисление ускорений
		forceX := thrustX + dragX
		forceY := thrustY + dragY - gravityForce
		rawAccelHorizontal := forceX / currentMass
		rawAccelVertical := forceY / currentMass
		s.LastAccelHorizontal = rawAccelHorizontal
		s.LastAccelVertical = rawAccelVertical

		// 9. Интегрирование скорости
		s.HorizontalVelocity += rawAccelHorizontal * deltaT
		s.VerticalVelocity += rawAccelVertical * deltaT

		// 10. Интегрирование положения (высоты)
		s.Altitude += s.VerticalVelocity * deltaT
		if s.Altitude < 0 {
			s.Altitude = 0
			s.VerticalVelocity = 0
			for i := range s.Engines {
				s.Engines[i].Running = false
			}
		}

		// Вычисляем новый throttle через Guidance
		twr := totalThrust / (currentMass * s.Gravity)
		newThrottle := guidance.ComputeThrottle(
			s.HorizontalVelocity, s.LastAccelHorizontal,
			s.VerticalVelocity, s.LastAccelVertical,
			s.Altitude, pitch,
			twr, 7670.0, 1.0,
		)
		//newThrottle := guidance.ComputeThrottle(
		//	s.HorizontalVelocity,
		//	s.VerticalVelocity,
		//	s.Altitude, 1.0,
		//)
		// Применяем throttle к работающим двигателям
		for i, engine := range s.Engines {
			if engine.Running {
				thrustNew := engine.MinThrust + newThrottle*(engine.MaxThrust-engine.MinThrust)
				if !stagesSeparated {
					s.Engines[i].Thrust = AdjustThrustByAltitude(s.Altitude)
				} else {
					s.Engines[i].Thrust = math.Max(engine.MinThrust, math.Min(thrustNew, engine.MaxThrust))
				}
			}
		}

		// Обновляем позицию (latitude и longitude)
		s.Lat, s.Lon = orbit.GuidanceUpdatePosition(s.Lat, s.Lon, totalVelocity, pitch, s.Heading)

		// 13. Обработка событий отделения ступеней и сброса носового обтекателя
		if timeV+1 >= 174 && !stagesSeparated {
			stagesSeparated = true
			// Обновление параметров после отделения
			s.DryMass = 7300
			s.FuelMass = 107670
			s.TotalMass = s.DryMass + s.FuelMass

			var engineEnabled bool
			for i := range s.Engines {
				if s.Engines[i].Running {
					if !engineEnabled {
						// Оставляем один двигатель работающим с новыми параметрами
						s.Engines[i].ISP = 348
						s.Engines[i].Thrust = 981000
						s.Engines[i].MaxThrust = 981000 + 500
						engineEnabled = true
					} else {
						s.Engines[i].Running = false
					}
				}
			}
			if !engineEnabled && len(s.Engines) > 0 {
				s.Engines[0].Running = true
				s.Engines[0].ISP = 348
				s.Engines[0].MaxIsp = 348
				s.Engines[0].Thrust = 981000
				s.Engines[0].MaxThrust = 981000 + 500
			}
			fmt.Println("Rocket stage was separated!")
		}

		if timeV+1 >= 195 && !noseFairingDestack {
			noseFairingDestack = true
			s.DryMass -= 1750
			s.TotalMass = s.DryMass + s.FuelMass
			fmt.Println("Nose fairing was destacked!")
		}

		// 14. Расход топлива
		var totalFuelUsed float64
		for i, engine := range s.Engines {
			// Обновляем ISP для двигателя с учетом высоты
			s.Engines[i].ISP = calculateISP(s.Altitude, s.Engines[i].MaxIsp)
			if engine.Running {
				massFlow := calculateMassFlow(engine)
				totalFuelUsed += massFlow
			}
		}
		if s.FuelMass < totalFuelUsed {
			totalFuelUsed = s.FuelMass
		}
		s.FuelMass -= totalFuelUsed
		// Обновляем общую массу после сжигания топлива
		currentMass = s.DryMass + s.FuelMass
		s.TotalMass = currentMass

		// 15. Случайные аварии (если запущено достаточно двигателей и до отделения ступеней)
		if s.rng.Float64() < 0.00015 && runningEngines(s.Engines) >= 7 && !stagesSeparated {
			randomIndex := s.rng.Intn(len(s.Engines))
			s.Engines[randomIndex].Running = false
			log.Printf("🚨 Двигатель %d отключён из-за неизвестной аварии на высоте %.2f м\n", randomIndex+1, s.Altitude)
			balanceEngines(s.Engines, randomIndex, s.rng)
		}

		// 16. Отправка метрик и обновление теплового состояния двигателей
		metrics.SendBasicMetrics(
			s.Altitude, s.VerticalVelocity, s.HorizontalVelocity,
			rawAccelHorizontal, rawAccelVertical, currentMass,
			dragForce, airDensity, runningEngines(s.Engines),
			effectiveAmbientTemp, pitch, s.Gravity,
			totalThrust, timeV, s.Lat, s.Lon,
		)
		for idx, engine := range s.Engines {
			s.Engines[idx].UpdateThermalState(s.Dt, effectiveAmbientTemp, s.Cp, s.MChamber, s.QFactor,
				s.CoolingCoeff, s.CoolingTimeConstant, airDensity, totalVelocity, 0.5)
			engineID := fmt.Sprintf("%d", idx+1)
			if engine.Running {
				metrics.SetEngineThrust(engineID, engine.Thrust, engine.ISP,
					engine.ChamberTemp, engine.NozzleTemp, engine.WallTemp, engine.TurbineTemp)
			} else {
				metrics.SetEngineThrust(engineID, 0, engine.ISP,
					engine.ChamberTemp, engine.NozzleTemp, engine.WallTemp, engine.TurbineTemp)
			}
		}

		timeV++
		s.mu.Unlock()

		time.Sleep(1 * time.Second)
	}
}

// calcEffectiveAmbientTemp рассчитывает эффективную температуру окружающей среды (в °C)
// используется USSA-76
func calcEffectiveAmbientTemp(altitude, seaLevelTemp float64) float64 {
	t := utils.KelvinToCelsius(CalcEffectiveAmbientTemp(altitude))

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
	a := utils.SpeedOfSound(tempK)

	// Число Маха
	mach := velocity / a

	// Коэффициент сопротивления в зависимости от числа Маха
	Cd := utils.CdByMach(mach)

	// Плотность воздуха (кг/м³)
	rho := CalcAirDensityUSSA(altitude)

	// Формула сопротивления
	return 0.5 * Cd * rho * velocity * velocity * area
}

func calculateMassFlow(engine Engine) float64 {
	if !engine.Running || engine.ISP <= 0 {
		return 0
	}
	return engine.Thrust / (engine.ISP * utils.G0)
}

func calculateISP(altitude, maxISP float64) float64 {
	const ISP_sea = 282.0 // для Merlin Vacuum на старте
	const h_max = 77000.0 // после этой высоты считаем ISP = ISP_vac

	// clamp значение от 0 до 1
	ratio := altitude / h_max
	if ratio > 1 {
		ratio = 1
	} else if ratio < 0 {
		ratio = 0
	}

	return ISP_sea + (maxISP-ISP_sea)*ratio
}

func AdjustThrustByAltitude(alt float64) float64 {
	const seaLevelThrust = 845000.0
	const vacuumThrust = 975000.0
	const maxBoostAltitude = 50000.0 // до этой высоты увеличиваем

	if alt >= maxBoostAltitude {
		return vacuumThrust
	}

	t := alt / maxBoostAltitude
	eased := 0.5 * (1 - math.Cos(t*math.Pi)) // от 0 до 1
	return seaLevelThrust + eased*(vacuumThrust-seaLevelThrust)
}

func ShouldSECO(altitude, vVertical, vHorizontal float64) bool {
	targetAltitude := 400_000.0 // м
	requiredVelocity := 7670.0  // м/с

	return altitude >= targetAltitude-5000 && // 5 км допуска
		altitude <= targetAltitude+5000 &&
		vHorizontal >= requiredVelocity &&
		math.Abs(vVertical) < 50 // почти 0
}
