package sensing

import (
	"math"
	"math/rand"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/physics"
)

// DefaultAttitudeSensor возвращает характеристики канала ориентации
// (гироскоп со звёздной/оптической коррекцией).
//
// Точнее и быстрее навигационного VectorSensor (DefaultPositionSensor/
// DefaultVelocitySensor): реальный гироскоп откликается на порядки быстрее,
// чем спутниковая навигация даёт свежую точку. FullScale взят как π:
// показание — вектор поворота, больше половины оборота он не бывает
// по построению.
//
// NoiseFloor здесь — одиночный отсчёт, а не итоговая точность знания
// ориентации: у хорошего кольцевого/волоконно-оптического гироскопа
// угловой случайный шум (ARW) на отдельном отсчёте — единицы угловых
// секунд, а не десятые доли градуса. Основной вклад в долгосрочную ошибку
// даёт именно накопленный дрейф (DriftRate), а не мгновенный шум. Это не
// формальность: контур качания камер обязан оставаться быстрым даже на
// посадочном манёвре (см. attitudeError — там сырое, нефильтрованное
// показание), и при большом моменте инерции корпуса даже небольшой на вид
// шум, умноженный на коэффициент пропорциональности контура, превращается
// в заметный момент — заниженный на порядок NoiseFloor раскачивал контур
// именно там, где скорость реакции важнее всего.
func DefaultAttitudeSensor() ScalarSensorConfig {
	return ScalarSensorConfig{
		NoiseFloor:      0.0001, // рад, ≈20″ — одиночный отсчёт, не итоговая точность
		DriftRate:       1e-6,
		FullScale:       math.Pi,
		TimeConstant:    0.05,
		UpdateInterval:  0.02,
		OutlierRate:     0.0003,
		OutlierScale:    6,
		DropoutRate:     0.0002,
		DropoutDuration: 0.3,
	}
}

// AttitudeMeasurement — показание датчика ориентации.
type AttitudeMeasurement struct {
	Orientation physics.Quaternion
	Valid       bool
}

// AttitudeSensor — датчик ориентации корпуса.
//
// Кватернион нелинеен, и покомпонентный шум на W/X/Y/Z не гарантирует
// единичную норму — такая модель была бы попросту неверной. Вместо этого
// ошибка ориентации представляется малым вектором поворота (три независимых
// скалярных канала — тот же принцип шума, что и у VectorSensor, применённый
// к угловой ошибке, а не к позиции), и итоговый кватернион строится
// композицией: sensedOrientation = trueOrientation · qError. Единица
// кватерниона гарантирована самой конструкцией, без ручной перенормировки.
type AttitudeSensor struct {
	Error *VectorSensor
}

// NewAttitudeSensor создаёт датчик ориентации.
func NewAttitudeSensor(cfg ScalarSensorConfig) *AttitudeSensor {
	return &AttitudeSensor{Error: NewVectorSensor(cfg)}
}

// UpdateWith пропускает истинную ориентацию через датчик.
//
// Канал ошибки всегда получает на вход нулевой истинный вектор: ошибка
// ориентации относительно самой себя равна нулю, а датчик добавляет к этому
// нулю собственные шум и дрейф — так и устроена реальная ошибка гироскопа:
// идеальный прибор показывал бы нуль всегда, настоящий уходит от него сам
// по себе.
func (a *AttitudeSensor) UpdateWith(trueOrientation physics.Quaternion, dt float64,
	rng *rand.Rand, ov control.SensorOverrides) AttitudeMeasurement {

	m := a.Error.UpdateWith(physics.Vec3{}, dt, rng, ov)
	qErr := quaternionFromRotationVector(m.Value)
	return AttitudeMeasurement{
		Orientation: trueOrientation.Multiply(qErr).Normalized(),
		Valid:       m.Valid,
	}
}

// Reset сбрасывает датчик.
func (a *AttitudeSensor) Reset() {
	a.Error.Reset()
}

// quaternionFromRotationVector строит кватернион поворота по вектору
// поворота (ось, умноженная на угол в радианах) — стандартное представление
// малой ошибки ориентации.
func quaternionFromRotationVector(v physics.Vec3) physics.Quaternion {
	angle := v.Norm()
	if angle < 1e-12 {
		return physics.IdentityQuaternion()
	}
	return physics.QuaternionFromAxisAngle(v.Scale(1/angle), angle)
}
