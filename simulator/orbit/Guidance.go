package orbit

import (
	"fmt"
	"math"
)

// -------------------------
// Guidance System
// -------------------------

// GuidanceState хранит внутреннее состояние системы наведения.
// Помимо текущих значений управления, система знает целевую орбитальную высоту.
type GuidanceState struct {
	CurrentPitch        float64 // текущий угол наклона (градусы)
	CurrentThrottle     float64 // текущий уровень тяги (от 0 до maxThrottle)
	TargetOrbitAltitude float64 // целевая орбитальная высота (м)
}

// NewGuidanceState создаёт и инициализирует состояние наведения.
func NewGuidanceState(targetOrbitAltitude float64) *GuidanceState {
	return &GuidanceState{
		CurrentPitch:        90.0,
		CurrentThrottle:     1.0,
		TargetOrbitAltitude: targetOrbitAltitude,
	}
}

// GuidanceUpdatePosition обновляет географическую позицию (latitude, longitude) за 1 секунду,
// используя сферическую модель Земли. Все углы задаются в градусах.
func GuidanceUpdatePosition(lat, lon, velocity, pitch, heading float64) (float64, float64) {
	latRad := lat * math.Pi / 180
	lonRad := lon * math.Pi / 180
	headingRad := heading * math.Pi / 180
	pitchRad := pitch * math.Pi / 180

	// Горизонтальная составляющая скорости.
	horizontalVelocity := velocity * math.Cos(pitchRad)
	// Пройденное расстояние за 1 сек (расстояние по дуге).
	d := horizontalVelocity

	newLat := math.Asin(
		math.Sin(latRad)*math.Cos(d/EarthRadius) +
			math.Cos(latRad)*math.Sin(d/EarthRadius)*math.Cos(headingRad),
	)
	newLon := lonRad + math.Atan2(
		math.Sin(headingRad)*math.Sin(d/EarthRadius)*math.Cos(latRad),
		math.Cos(d/EarthRadius)-math.Sin(latRad)*math.Sin(newLat),
	)
	return newLat * 180 / math.Pi, newLon * 180 / math.Pi
}

// calcGravityAtAltitude вычисляет ускорение свободного падения на заданной высоте (упрощённо).
func calcGravityAtAltitude(altitude float64) float64 {
	return 9.81 * math.Pow(EarthRadius/(EarthRadius+altitude), 2)
}

// ComputePitchAuto вычисляет целевой угол наклона (pitch) для ракеты автоматически.
// Алгоритм использует PD-контроль, где ошибка – разница между целевой орбитальной высотой
// и текущей высотой, а также корректировку по вертикальной скорости.
// Дополнительно учитывается доля набранной горизонтальной скорости для плавного перехода.
func (g *GuidanceState) ComputePitchAuto(
	altitude, vVertical, vHorizontal float64,
) float64 {
	fmt.Printf("altitude: %f, vVertical: %f, vHorizontal: %f\n", altitude, vVertical, vHorizontal)
	// Для очень низкой высоты (например, ниже 500 м) сохраняем вертикальный взлёт.
	minimalTurnAlt := 500.0 // порог в метрах
	if altitude < minimalTurnAlt {
		g.CurrentPitch = 90.0
		return g.CurrentPitch
	}

	// Вычисляем ошибку по высоте:
	altError := g.TargetOrbitAltitude - altitude

	// Контроллер PD: задаёт желаемый угол на основе ошибки и вертикальной скорости.
	// Коэффициенты (k_p, k_d) подбираются экспериментально.
	k_p := 0.0001 // [°/м]
	k_d := 0.05   // [°/(м/с)]
	desiredAngle := k_p*altError - k_d*vVertical

	// Если горизонтальная скорость низкая, ракета ещё не готова к орбитальному полёту:
	requiredOrbitalV := 7670.0
	ratio := math.Min(vHorizontal/requiredOrbitalV, 1.0)
	minTargetPitch := 10.0 // минимальный угол при достижении орбитальной скорости.
	// Плавно смешиваем желаемый угол с минимальным уголом по мере роста горизонтальной скорости.
	targetPitch := (1-ratio)*desiredAngle + ratio*minTargetPitch

	//fmt.Printf("altError: %f, desiredAngle: %f, ratio: %f, targetPitch: %f, CurrentPitch: %f\n",
	//	altError, desiredAngle, ratio, targetPitch, g.CurrentPitch)

	// Ограничиваем целевой угол в допустимых пределах.
	if targetPitch > 90 {
		targetPitch = 90
	} else if targetPitch < -30 {
		targetPitch = -30
	}

	// Плавное обновление через альфа-фильтр (чтобы избежать резких скачков).
	alpha := 0.1 // коэффициент сглаживания (можно увеличить для быстрого отклика)
	g.CurrentPitch = (1-alpha)*g.CurrentPitch + alpha*targetPitch

	return g.CurrentPitch
}

// ComputeThrottle вычисляет желаемый уровень тяги (throttle) для управления набором высоты.
// Функция учитывает горизонтальный дефицит скорости, вертикальный компонент и ошибку по высоте,
// чтобы уменьшить тягу при чрезмерном наборе высоты.
// Параметры:
//   - vHorizontal, aHorizontal: горизонтальная скорость и ускорение.
//   - vVertical, aVertical: вертикальная скорость и ускорение.
//   - altitude: текущая высота (м).
//   - pitchDeg: текущий угол наклона (градусы).
//   - twr: текущий коэффициент тяги к весу.
//   - requiredOrbitalV: требуемая орбитальная скорость (м/с).
//   - maxThrottle: максимальное значение throttle.
func (g *GuidanceState) ComputeThrottle(
	vHorizontal, aHorizontal, vVertical, aVertical, altitude, pitchDeg, twr, requiredOrbitalV, maxThrottle float64,
) float64 {
	baseThrottle := maxThrottle

	// Горизонтальный компонент: если горизонтальная скорость ниже требуемой – добавляем коррекцию.
	speedDeficit := math.Max(0.0, requiredOrbitalV-vHorizontal) / requiredOrbitalV
	pitchRad := pitchDeg * math.Pi / 180.0
	angleFactor := math.Pow(math.Cos(pitchRad), 0.5)
	correctionH := speedDeficit * angleFactor

	// Вертикальный фактор: если вертикальная скорость слишком высока – снижаем тягу,
	// а если ракета начинает снижаться, можно немного увеличить тягу.
	verticalFactor := 1.0
	if vVertical > 300 {
		verticalFactor = 0.7
	} else if vVertical < 0 {
		verticalFactor = 1.1
	}

	// Фактор ошибки по высоте: если ракета уже выше цели, снижаем тягу, если ниже – допускаем чуть больший прирост.
	altError := g.TargetOrbitAltitude - altitude
	k_alt := 0.3
	altFactor := 1.0 + k_alt*(altError/g.TargetOrbitAltitude)
	if altFactor > 1.2 {
		altFactor = 1.2
	} else if altFactor < 0.8 {
		altFactor = 0.8
	}

	// Объединяем коррекции.
	desiredThrottle := baseThrottle * (1 + correctionH) * verticalFactor * altFactor

	// Модерация по TWR: если текущий TWR слишком высок, снижаем throttle,
	// если слишком низкий – устанавливаем минимальное значение.
	targetTWR := 1.5
	if twr > targetTWR {
		desiredThrottle *= targetTWR / twr
	} else if twr < 1.0 {
		desiredThrottle = 1.0
	}

	// Плавное изменение с ограничением скорости изменения (rate limiting).
	rateLimit := 0.02
	if vHorizontal > 7000 {
		rateLimit = 0.005
	}
	diff := desiredThrottle - g.CurrentThrottle
	if math.Abs(diff) < rateLimit {
		g.CurrentThrottle = desiredThrottle
	} else if diff > 0 {
		g.CurrentThrottle += rateLimit
	} else {
		g.CurrentThrottle -= rateLimit
	}

	// Ограничиваем значение throttle в пределах допустимого.
	if g.CurrentThrottle < 0 {
		g.CurrentThrottle = 0
	} else if g.CurrentThrottle > maxThrottle {
		g.CurrentThrottle = maxThrottle
	}

	return g.CurrentThrottle
}
