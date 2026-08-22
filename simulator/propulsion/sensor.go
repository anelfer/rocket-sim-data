package propulsion

import (
	"math"
	"math/rand"

	"rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Модель датчика.
//
// Шум, дрейф и запаздывание накладываются ТОЛЬКО на выход измерения и никогда
// не возвращаются в уравнения модели. Это принципиально: физические связи
// считаются по истинным величинам, поэтому если расход упал, то давление
// в камере и тяга упадут согласованно, а не разойдутся из-за разного шума.
//
// Наблюдатель видит зашумлённую картину, но под ней лежит согласованная физика.
// -----------------------------------------------------------------------------

// SensorConfig — характеристики измерительного канала.
type SensorConfig struct {
	// NoiseFraction — относительное среднеквадратичное отклонение шума.
	NoiseFraction float64

	// NoiseFloor — абсолютный уровень шума в единицах величины.
	// Нужен там, где измеряемое значение проходит через ноль.
	NoiseFloor float64

	// DriftRate — скорость дрейфа нуля, доля полной шкалы в секунду.
	DriftRate float64

	// FullScale — полная шкала датчика, для пересчёта дрейфа.
	FullScale float64

	// TimeConstant — постоянная времени измерительного тракта, с.
	// Задаёт запаздывание показаний относительно истинного значения.
	TimeConstant float64

	// UpdateInterval — период обновления показаний, с. Между обновлениями
	// датчик отдаёт последнее измеренное значение.
	UpdateInterval float64

	// OutlierRate — частота одиночных выбросов, 1/с.
	OutlierRate float64

	// OutlierScale — во сколько раз выброс превышает обычный шум.
	OutlierScale float64

	// DropoutRate — частота кратковременных пропаданий связи, 1/с.
	DropoutRate float64

	// DropoutDuration — длительность пропадания, с.
	DropoutDuration float64
}

// DefaultPressureSensor возвращает характеристики датчика давления.
func DefaultPressureSensor(fullScale float64) SensorConfig {
	return SensorConfig{
		NoiseFraction:   0.004,
		DriftRate:       2e-6,
		FullScale:       fullScale,
		TimeConstant:    0.02,
		UpdateInterval:  0.05,
		OutlierRate:     0.004,
		OutlierScale:    12,
		DropoutRate:     0.001,
		DropoutDuration: 0.4,
	}
}

// DefaultTemperatureSensor возвращает характеристики термопары.
//
// Термопара инерционна: спай прогревается заметно медленнее газа, поэтому
// постоянная времени на два порядка больше, чем у датчика давления.
func DefaultTemperatureSensor(fullScale float64) SensorConfig {
	return SensorConfig{
		NoiseFraction:   0.002,
		NoiseFloor:      0.6,
		DriftRate:       5e-6,
		FullScale:       fullScale,
		TimeConstant:    0.8,
		UpdateInterval:  0.1,
		OutlierRate:     0.002,
		OutlierScale:    8,
		DropoutRate:     0.0008,
		DropoutDuration: 0.5,
	}
}

// DefaultFlowSensor возвращает характеристики расходомера.
func DefaultFlowSensor(fullScale float64) SensorConfig {
	return SensorConfig{
		NoiseFraction:   0.008,
		DriftRate:       3e-6,
		FullScale:       fullScale,
		TimeConstant:    0.06,
		UpdateInterval:  0.05,
		OutlierRate:     0.003,
		OutlierScale:    10,
		DropoutRate:     0.001,
		DropoutDuration: 0.3,
	}
}

// DefaultSpeedSensor возвращает характеристики датчика оборотов.
func DefaultSpeedSensor(fullScale float64) SensorConfig {
	return SensorConfig{
		NoiseFraction:   0.0015,
		DriftRate:       0,
		FullScale:       fullScale,
		TimeConstant:    0.03,
		UpdateInterval:  0.02,
		OutlierRate:     0.001,
		OutlierScale:    6,
		DropoutRate:     0.0005,
		DropoutDuration: 0.2,
	}
}

// Sensor — состояние измерительного канала.
type Sensor struct {
	Config SensorConfig

	filtered   float64 // значение после инерционного звена
	held       float64 // последнее выданное значение
	drift      float64 // накопленный дрейф нуля
	sinceRead  float64 // время с последнего обновления
	dropoutFor float64 // остаток времени пропадания связи
	valid      bool
}

// NewSensor создаёт датчик.
func NewSensor(cfg SensorConfig) *Sensor {
	return &Sensor{Config: cfg}
}

// Update принимает истинное значение и возвращает показание датчика.
//
// Второй результат сообщает, есть ли достоверное показание: во время
// кратковременного пропадания связи точка телеметрии отсутствует.
func (s *Sensor) Update(trueValue, dt float64, rng *rand.Rand) (float64, bool) {
	return s.UpdateWith(trueValue, dt, rng, control.SensorOverrides{})
}

// UpdateWith пропускает величину через датчик с учётом ручных воздействий
// на измерительный тракт.
//
// Воздействия на датчик не меняют физику: они меняют только то, что видит
// оператор. Замороженный датчик продолжает отдавать последнее показание,
// пока агрегат живёт своей жизнью, — именно так выглядит настоящий отказ
// измерительного канала.
func (s *Sensor) UpdateWith(trueValue, dt float64, rng *rand.Rand,
	ov control.SensorOverrides) (float64, bool) {

	if ov.Dead {
		return 0, false
	}
	if ov.Frozen {
		if !s.valid {
			return 0, false
		}
		return s.held, true
	}

	cfg := s.Config
	cfg.NoiseFraction *= ov.NoiseFactor.Or(1)
	cfg.NoiseFloor *= ov.NoiseFactor.Or(1)
	cfg.DriftRate *= ov.DriftFactor.Or(1)

	// Инерционное звено первого порядка: показание догоняет истинное значение
	// с постоянной времени датчика.
	if cfg.TimeConstant > 0 {
		decay := math.Exp(-dt / cfg.TimeConstant)
		s.filtered = trueValue + (s.filtered-trueValue)*decay
	} else {
		s.filtered = trueValue
	}

	// Медленный дрейф нуля.
	if cfg.DriftRate != 0 && cfg.FullScale != 0 {
		s.drift += rng.NormFloat64() * cfg.DriftRate * cfg.FullScale * dt
		limit := 0.02 * math.Abs(cfg.FullScale)
		s.drift = math.Max(-limit, math.Min(limit, s.drift))
	}

	// Пропадание связи.
	if s.dropoutFor > 0 {
		s.dropoutFor -= dt
		return 0, false
	}
	if cfg.DropoutRate > 0 && rng.Float64() < cfg.DropoutRate*dt {
		s.dropoutFor = cfg.DropoutDuration
		return 0, false
	}

	// Ограниченная частота обновления: между опросами отдаётся то же значение.
	s.sinceRead += dt
	if s.valid && cfg.UpdateInterval > 0 && s.sinceRead < cfg.UpdateInterval {
		return s.held, true
	}
	s.sinceRead = 0

	value := s.filtered + s.drift

	// Шум измерения.
	sigma := math.Abs(value)*cfg.NoiseFraction + cfg.NoiseFloor
	if sigma > 0 {
		value += rng.NormFloat64() * sigma
	}

	// Редкий одиночный выброс.
	if cfg.OutlierRate > 0 && rng.Float64() < cfg.OutlierRate*math.Max(dt, cfg.UpdateInterval) {
		value += rng.NormFloat64() * sigma * cfg.OutlierScale
	}

	s.held = value
	s.valid = true
	return value, true
}

// Reset возвращает датчик в исходное состояние.
func (s *Sensor) Reset() {
	s.filtered = 0
	s.held = 0
	s.drift = 0
	s.sinceRead = 0
	s.dropoutFor = 0
	s.valid = false
}

// -----------------------------------------------------------------------------
// Набор датчиков двигателя
// -----------------------------------------------------------------------------

// SensorSuite — измерительные каналы двигательной установки.
type SensorSuite struct {
	ChamberPressure  *Sensor
	FuelPumpOutlet   *Sensor
	OxPumpOutlet     *Sensor
	FuelPumpInlet    *Sensor
	OxPumpInlet      *Sensor
	FuelFlow         *Sensor
	OxFlow           *Sensor
	ShaftSpeed       *Sensor
	TurbineTemp      *Sensor
	BearingTemp      *Sensor
	WallTemp         *Sensor
	CoolantOutlet    *Sensor
	TankFuelPressure *Sensor
	TankOxPressure   *Sensor
	Vibration        *Sensor
}

// NewSensorSuite создаёт набор датчиков с типовыми характеристиками.
func NewSensorSuite(nominalChamberPressure float64) *SensorSuite {
	pumpScale := nominalChamberPressure * 1.6
	return &SensorSuite{
		ChamberPressure:  NewSensor(DefaultPressureSensor(nominalChamberPressure)),
		FuelPumpOutlet:   NewSensor(DefaultPressureSensor(pumpScale)),
		OxPumpOutlet:     NewSensor(DefaultPressureSensor(pumpScale)),
		FuelPumpInlet:    NewSensor(DefaultPressureSensor(600000)),
		OxPumpInlet:      NewSensor(DefaultPressureSensor(600000)),
		FuelFlow:         NewSensor(DefaultFlowSensor(120)),
		OxFlow:           NewSensor(DefaultFlowSensor(280)),
		ShaftSpeed:       NewSensor(DefaultSpeedSensor(4000)),
		TurbineTemp:      NewSensor(DefaultTemperatureSensor(1200)),
		BearingTemp:      NewSensor(DefaultTemperatureSensor(600)),
		WallTemp:         NewSensor(DefaultTemperatureSensor(1200)),
		CoolantOutlet:    NewSensor(DefaultTemperatureSensor(700)),
		TankFuelPressure: NewSensor(DefaultPressureSensor(400000)),
		TankOxPressure:   NewSensor(DefaultPressureSensor(400000)),
		Vibration:        NewSensor(DefaultFlowSensor(30)),
	}
}

// Reset сбрасывает все каналы.
func (s *SensorSuite) Reset() {
	for _, sensor := range []*Sensor{
		s.ChamberPressure, s.FuelPumpOutlet, s.OxPumpOutlet,
		s.FuelPumpInlet, s.OxPumpInlet, s.FuelFlow, s.OxFlow,
		s.ShaftSpeed, s.TurbineTemp, s.BearingTemp, s.WallTemp,
		s.CoolantOutlet, s.TankFuelPressure, s.TankOxPressure, s.Vibration,
	} {
		sensor.Reset()
	}
}

// Measurement — показание с признаком достоверности.
type Measurement struct {
	Value float64
	Valid bool
}

// Или возвращает измеренное значение, а при потере связи — запасное.
func (m Measurement) Or(fallback float64) float64 {
	if m.Valid {
		return m.Value
	}
	return fallback
}

// AsFloat возвращает значение либо NaN, если показания нет.
// NaN в Prometheus означает пропуск точки, и Grafana рисует разрыв.
func (m Measurement) AsFloat() float64 {
	if m.Valid {
		return m.Value
	}
	return math.NaN()
}
