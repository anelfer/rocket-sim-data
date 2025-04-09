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
	s.Gravity = calcGravityAtAltitude(0)
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

	for {
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
		thrust := TotalThrust(s.Engines)

		currentMass := s.DryMass + s.FuelMass
		// 1. Вычисляем гравитацию на текущей высоте
		rawGravity := calcGravityAtAltitude(s.Altitude)

		// 2. Центростремительное ускорение (если есть горизонтальная скорость)
		radius := orbit.EarthRadius + s.Altitude
		centripetalAccel := (s.HorizontalVelocity * s.HorizontalVelocity) / radius

		// 3. Эффективная гравитация: "чистая" сила, которую ещё надо компенсировать
		effectiveGravity := rawGravity - centripetalAccel

		// 4. Обнулим при отрицательных значениях (иначе ракета "вылетит в космос")
		if effectiveGravity < 0 {
			effectiveGravity = 0
		}

		// 5. Запоминаем в симуляцию и считаем силу тяжести
		s.Gravity = effectiveGravity
		gravityForce := currentMass * s.Gravity

		// Получаем текущий pitch
		pitch := orbit.ComputePitchSmart(s.Altitude, s.VerticalVelocity, s.HorizontalVelocity, s.LastAccelVertical, s.LastAccelHorizontal, orbit.OrbitLEO) //s.VerticalVelocity
		pitchRad := pitch * math.Pi / 180

		// 5. Разложим тягу по осям
		thrustX := thrust * math.Cos(pitchRad)
		thrustY := thrust * math.Sin(pitchRad)

		// 6. Сопротивление воздуха
		airDensity := CalcAirDensityUSSA(s.Altitude)
		totalVelocity := math.Hypot(s.HorizontalVelocity, s.VerticalVelocity)
		totalvelocityRel := math.Hypot(s.HorizontalVelocity-orbit.EarthRotationSpeedAtLatitude(s.Lat), s.VerticalVelocity)
		cD := utils.CdByVelocity(totalvelocityRel, CalcEffectiveAmbientTemp(s.Altitude))

		targetDrag := 0.5 * cD * airDensity * totalvelocityRel * totalvelocityRel * s.Area
		s.LastDrag += (targetDrag - s.LastDrag) * 1
		drag := s.LastDrag

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

		// 9. Ускорения по осям (сырые)
		rawAccelHorizontal := forceX / currentMass
		rawAccelVertical := forceY / currentMass

		accelVertical := rawAccelVertical
		accelHorizontal := rawAccelHorizontal

		s.LastAccelVertical = accelVertical
		s.LastAccelHorizontal = accelHorizontal

		// 10. Обновляем скорости
		s.VerticalVelocity += accelVertical * deltaT
		s.HorizontalVelocity += accelHorizontal * deltaT

		// 11. Обновляем высоту
		s.Altitude += s.VerticalVelocity * deltaT
		if s.Altitude < 0 {
			s.Altitude = 0
			s.VerticalVelocity = 0
			for i := range s.Engines {
				s.Engines[i].Running = false
			}
		}

		newThrottle := orbit.ComputeThrottle(s.HorizontalVelocity, s.LastAccelHorizontal, s.VerticalVelocity, s.LastAccelVertical, s.Altitude, pitch, utils.CalculateTWR(thrust, s.TotalMass, s.Gravity), 7670.0, 1)
		orbit.CurrentThrottle = newThrottle
		for i, engine := range s.Engines {
			if engine.Running {
				// Линейная интерполяция между MinThrust и MaxThrust по throttle
				thrustNew := engine.MinThrust + newThrottle*(engine.MaxThrust-engine.MinThrust)

				// Обновляем значение в массиве
				s.Engines[i].Thrust = math.Max(engine.MinThrust, math.Min(thrustNew, engine.MaxThrust))
			}
		}

		//if ShouldSECO(s.Altitude, s.VerticalVelocity, s.HorizontalVelocity) {
		//	for i, engine := range s.Engines {
		//		if engine.Running {
		//			s.Engines[i].Running = false
		//		}
		//	} // SECO
		//}

		s.Lat, s.Lon = orbit.GuidanceUpdatePosition(s.Lat, s.Lon, math.Hypot(s.HorizontalVelocity, s.VerticalVelocity), pitch, s.Heading)

		thrustPerEngine := AdjustThrustByAltitude(s.Altitude)
		for i, engine := range s.Engines {
			if engine.Running && !stagesSeparated {
				engine.Thrust = thrustPerEngine
				s.Engines[i] = engine
			}
		}

		//fmt.Printf("alt=%.0f pitch=%.1f vx=%.1f vy=%.1f aX=%.2f aY=%.2f\n",
		//	s.Altitude, pitch, s.HorizontalVelocity, s.VerticalVelocity,
		//	accelHorizontal, accelVertical)

		//stage separation
		if timeV+1 >= 180 && !stagesSeparated {
			stagesSeparated = true
			s.DryMass = 7300
			s.FuelMass = 107670
			s.TotalMass = s.DryMass + s.FuelMass
			enabled := false

			for i := range s.Engines {
				if s.Engines[i].Running {
					if !enabled {
						// Оставляем включённым и задаём параметры
						s.Engines[i].ISP = 348
						s.Engines[i].Thrust = 981000
						s.Engines[i].MaxThrust = 981000 + 500
						enabled = true
					} else {
						// Выключаем лишние
						s.Engines[i].Running = false
					}
				}
			}

			// Если все были выключены — включаем один и задаём параметры
			if !enabled && len(s.Engines) > 0 {
				s.Engines[0].Running = true
				s.Engines[0].ISP = 348
				s.Engines[0].Thrust = 981000
				s.Engines[0].MaxThrust = 981000 + 500
			}
			fmt.Println("Rocket stage was separated!")
		}

		if timeV+1 >= 195 && !noseFairingDestack {
			noseFairingDestack = true
			s.DryMass = s.DryMass - 1750
			fmt.Println("Nose fairing was destacked!")
		}

		// Расход топлива
		var totalFuelUsed float64
		for i, engine := range s.Engines {
			engine.ISP = calculateISP(s.Altitude)
			s.Engines[i] = engine
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
		if s.rng.Float64() < 0.00015 && runningEngines(s.Engines) >= 7 && !stagesSeparated {
			randomIndex := s.rng.Intn(len(s.Engines))
			s.Engines[randomIndex].Running = false
			log.Printf("🚨 Двигатель %d отключён из-за неизвестной аварии на высоте %.2f м\n", randomIndex+1, s.Altitude)
			balanceEngines(s.Engines, randomIndex, s.rng)
		}

		// Отправка метрик
		metrics.SendBasicMetrics(s.Altitude, s.VerticalVelocity, s.HorizontalVelocity, accelHorizontal, accelVertical, currentMass, drag, airDensity, running, effectiveAmbientTemp, pitch, s.Gravity, TotalThrust(s.Engines), timeV, s.Lat, s.Lon)
		for idx, engine := range s.Engines {
			s.Engines[idx].UpdateThermalState(s.Dt, effectiveAmbientTemp, s.Cp, s.MChamber, s.QFactor, s.CoolingCoeff, s.CoolingTimeConstant, airDensity, totalVelocity, 0.5)
			engineID := fmt.Sprintf("%d", idx+1)
			if engine.Running {
				metrics.SetEngineThrust(engineID, engine.Thrust, engine.ISP, engine.ChamberTemp, engine.NozzleTemp, engine.WallTemp, engine.TurbineTemp)
			} else {
				metrics.SetEngineThrust(engineID, 0, engine.ISP, engine.ChamberTemp, engine.NozzleTemp, engine.WallTemp, engine.TurbineTemp)
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

func calculateMassFlow(engine Engine, gravity float64) float64 {
	if !engine.Running || engine.ISP <= 0 {
		return 0
	}
	return engine.Thrust / (engine.ISP * gravity)
}

func calculateISP(altitude float64) float64 {
	const ISP_sea = 282.0 // для Merlin Vacuum на старте
	const ISP_vac = 311.0 // в вакууме
	const h_max = 77000.0 // после этой высоты считаем ISP = ISP_vac

	// clamp значение от 0 до 1
	ratio := altitude / h_max
	if ratio > 1 {
		ratio = 1
	} else if ratio < 0 {
		ratio = 0
	}

	return ISP_sea + (ISP_vac-ISP_sea)*ratio
}

// Реалистичный расчёт maxJerk в м/с³
func CalculateMaxJerk(thrust float64, mass float64, altitude float64) float64 {
	// 1. Получаем плотность атмосферы
	airDensity := CalcAirDensityUSSA(altitude)

	// 2. Время отклика системы управления тягой
	// Чем больше плотность — тем медленнее реакция
	tau := 0.2 + 5.0*(airDensity/1.225) // от ~0.2с в вакууме до ~5.2с на уровне моря

	// 3. Вычисляем максимально допустимый jerk
	maxJerk := thrust / (mass * tau)

	return maxJerk
}

func limitAcceleration(prevAccel, targetAccel, deltaT, maxJerk float64) float64 {
	deltaA := targetAccel - prevAccel
	maxDeltaA := maxJerk * deltaT

	if deltaA > maxDeltaA {
		return prevAccel + maxDeltaA
	} else if deltaA < -maxDeltaA {
		return prevAccel - maxDeltaA
	}
	return targetAccel
}

func getAccelResponseFactor(mass, area, Cd, altitude, throttleResponse, deltaT float64) float64 {
	// 1. Плотность воздуха
	rho := CalcAirDensityUSSA(altitude) // кг/м³

	// 2. Аэродинамическое демпфирование
	// F_drag ≈ 0.5 * rho * V² * Cd * A
	// τ_аэро ~ масса / (0.5 * rho * Cd * A)
	// пусть V = 1 м/с для нормализации (мы хотим коэффициент)
	tauAero := mass / (0.5*rho*Cd*area + 1e-5) // +ε чтобы не делить на 0

	// 3. Время отклика двигателя (сек)
	tauEngine := throttleResponse // обычно 0.1–0.5 сек

	// 4. Примерная задержка конструкции (структура) — эмпирически
	tauStruct := 0.05 // можно сделать параметром

	// 5. Итоговая tau — реакция всей системы
	tau := tauAero + tauEngine + tauStruct

	// 6. alpha = Δt / τ
	alpha := deltaT / tau

	// Ограничим alpha
	if alpha > 0.25 {
		alpha = 0.25
	}
	if alpha < 0.005 {
		alpha = 0.005
	}
	return alpha
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
