// Package sensing моделирует навигационные датчики GNC: положение и скорость
// в ECI, зашумленные так же, как двигательные каналы в simulator/propulsion.
//
// Принцип шума один на всю симуляцию (шум fraction+floor, дрейф нуля,
// запаздывание первого порядка, ограниченная частота обновления, редкие
// пропуски связи и выбросы) — меняется только физическая величина на входе.
// ScalarSensor — это и есть propulsion.Sensor, под навигационным именем;
// VectorSensor — три независимых ScalarSensor-канала по осям X, Y, Z,
// собранные в physics.Vec3. Это не отдельная модель ошибки, а тот же
// скалярный шум на трёх независимых каналах — честное приближение для
// позиции/скорости без матрицы ковариаций и лишней настроечной поверхности.
package sensing

import (
	"math/rand"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/propulsion"
)

// ScalarSensor — датчик одной скалярной величины навигации.
type ScalarSensor = propulsion.Sensor

// ScalarSensorConfig — настройки ScalarSensor.
type ScalarSensorConfig = propulsion.SensorConfig

// NewScalarSensor создаёт скалярный датчик с нулевым начальным показанием.
func NewScalarSensor(cfg ScalarSensorConfig) *ScalarSensor {
	return propulsion.NewSensor(cfg)
}

// NewScalarSensorAt создаёт датчик, уже показывающий заданное значение —
// нужен там, где истинная величина при старте симуляции не близка к нулю
// (см. propulsion.NewSensorAt: без прайминга датчик первые секунды отражал
// бы переходный процесс от нуля к истинному значению, а не реальный отсчёт).
func NewScalarSensorAt(cfg ScalarSensorConfig, initial float64) *ScalarSensor {
	return propulsion.NewSensorAt(cfg, initial)
}

// DefaultPositionSensor возвращает характеристики навигационного канала
// положения (спутниковая навигация с инерциальной коррекцией).
//
// Шум задан абсолютным порогом (NoiseFloor), а не долей значения: координата
// в ECI сама по себе — величина порядка радиуса Земли, и относительный шум
// такого масштаба был бы абсурдно велик. Постоянная времени и частота
// обновления заметно быстрее, чем у датчика остатка топлива (Фаза 2):
// наведение замкнуто непрерывно, а не по разовому порогу, и задержка здесь
// впрямую расшатывает контур.
func DefaultPositionSensor() ScalarSensorConfig {
	return ScalarSensorConfig{
		NoiseFloor:      15,
		DriftRate:       1e-7,
		FullScale:       physics.EarthRadius,
		TimeConstant:    0.3,
		UpdateInterval:  0.1,
		OutlierRate:     0.0005,
		OutlierScale:    8,
		DropoutRate:     0.0005,
		DropoutDuration: 0.6,
	}
}

// DefaultVelocitySensor возвращает характеристики навигационного канала
// скорости. Обоснование то же, что у DefaultPositionSensor.
func DefaultVelocitySensor() ScalarSensorConfig {
	return ScalarSensorConfig{
		NoiseFloor:      1.0,
		DriftRate:       2e-7,
		FullScale:       8000,
		TimeConstant:    0.3,
		UpdateInterval:  0.1,
		OutlierRate:     0.0005,
		OutlierScale:    8,
		DropoutRate:     0.0005,
		DropoutDuration: 0.6,
	}
}

// VectorMeasurement — показание векторного датчика.
type VectorMeasurement struct {
	Value physics.Vec3
	Valid bool // true, только если достоверны все три канала
}

// VectorSensor — три независимых скалярных канала по осям X, Y, Z.
type VectorSensor struct {
	X, Y, Z *ScalarSensor
}

// NewVectorSensor создаёт векторный датчик с нулевым начальным показанием.
func NewVectorSensor(cfg ScalarSensorConfig) *VectorSensor {
	return &VectorSensor{
		X: NewScalarSensor(cfg),
		Y: NewScalarSensor(cfg),
		Z: NewScalarSensor(cfg),
	}
}

// NewVectorSensorAt создаёт датчик, уже показывающий заданный вектор —
// нужен для положения и скорости в ECI: обе величины при старте симуляции
// заведомо не близки к нулю (см. NewScalarSensorAt).
func NewVectorSensorAt(cfg ScalarSensorConfig, initial physics.Vec3) *VectorSensor {
	return &VectorSensor{
		X: NewScalarSensorAt(cfg, initial.X),
		Y: NewScalarSensorAt(cfg, initial.Y),
		Z: NewScalarSensorAt(cfg, initial.Z),
	}
}

// UpdateWith пропускает вектор через три независимых канала.
//
// Показание всего вектора достоверно, только если достоверны все три
// канала: неполный вектор положения или скорости непригоден для наведения,
// даже если два из трёх компонентов в порядке.
func (v *VectorSensor) UpdateWith(trueValue physics.Vec3, dt float64, rng *rand.Rand,
	ov control.SensorOverrides) VectorMeasurement {

	x, okX := v.X.UpdateWith(trueValue.X, dt, rng, ov)
	y, okY := v.Y.UpdateWith(trueValue.Y, dt, rng, ov)
	z, okZ := v.Z.UpdateWith(trueValue.Z, dt, rng, ov)
	return VectorMeasurement{
		Value: physics.Vec3{X: x, Y: y, Z: z},
		Valid: okX && okY && okZ,
	}
}

// Reset сбрасывает все три канала.
func (v *VectorSensor) Reset() {
	v.X.Reset()
	v.Y.Reset()
	v.Z.Reset()
}
