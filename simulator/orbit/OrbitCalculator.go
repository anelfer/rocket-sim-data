package orbit

import "math"

// import (
//
//	"math"
//
// )
const (
	EarthRadius = 6371000.0
	earthOmega  = 7.2921159e-5
)

// // GuidanceUpdatePosition обновляет позицию ракеты за 1 секунду
// func GuidanceUpdatePosition(
//
//	lat, lon float64, // текущие координаты (градусы)
//	velocity float64, // полная скорость ракеты (м/с)
//	pitch float64, // угол к горизонту (градусы), 90 = вверх
//	heading float64, // азимут (градусы), 90 = восток
//
//	) (float64, float64) {
//		latRad := lat * math.Pi / 180
//		lonRad := lon * math.Pi / 180
//		headingRad := heading * math.Pi / 180
//		pitchRad := pitch * math.Pi / 180
//
//		// Горизонтальная компонента скорости (движение по поверхности)
//		horizontalVelocity := velocity * math.Cos(pitchRad)
//
//		// Расстояние за 1 секунду (горизонтальное перемещение по сфере)
//		d := horizontalVelocity // м/с
//
//		// Обновляем широту
//		newLat := math.Asin(
//			math.Sin(latRad)*math.Cos(d/EarthRadius) +
//				math.Cos(latRad)*math.Sin(d/EarthRadius)*math.Cos(headingRad),
//		)
//
//		// Обновляем долготу
//		newLon := lonRad + math.Atan2(
//			math.Sin(headingRad)*math.Sin(d/EarthRadius)*math.Cos(latRad),
//			math.Cos(d/EarthRadius)-math.Sin(latRad)*math.Sin(newLat),
//		)
//
//		return newLat * 180 / math.Pi, newLon * 180 / math.Pi
//	}
//
// var currentPitch = 90.0 // глобально храним последний pitch, можно обернуть в структуру
//
// func ComputePitchSmart(
//
//	altitude float64,
//	vVertical float64,
//	vHorizontal float64,
//	aVertical float64,
//	aHorizontal float64,
//	mission OrbitType,
//
//	) float64 {
//		startAltitude, endAltitude, targetPitch := mission.GetPitchTurnParams()
//		targetAlt := mission.GetTargetOrbitAltitude()
//		requiredOrbitalV := 7670.0
//		initialPitch := 90.0
//
//		// Фаза старта: вертикальный взлёт
//		if altitude <= startAltitude {
//			currentPitch = initialPitch
//			return currentPitch
//		}
//
//		// Фаза гравити-терна: от 90 к целевому углу
//		if altitude < endAltitude {
//			t := (altitude - startAltitude) / (endAltitude - startAltitude)
//			eased := 0.5 * (1 - math.Cos(t*math.Pi))
//			currentPitch = initialPitch*(1-eased) + targetPitch*eased
//			return currentPitch
//		}
//
//		// === Орбитальная фаза ===
//
//		// Прогноз апогея (по вертикальной скорости)
//		gravity := 9.81 // или твой calcGravityAtAltitude(altitude)
//		predictedApogee := altitude + (vVertical*vVertical)/(2*gravity)
//
//		// Относительные факторы
//		speedRatio := math.Min(vHorizontal/requiredOrbitalV, 1.0)
//		apogeeRatio := predictedApogee / targetAlt
//
//		// Основной расчёт pitch
//		target := targetPitch
//
//		// 🔻 Перелёт по апогею — опускаем нос
//		if apogeeRatio > 1.05 {
//			target = -5.0 // "нырнуть", чтобы не забрасывало выше
//		}
//
//		// 🔼 Недолёт по апогею — чуть выше
//		if apogeeRatio < 0.95 && altitude > endAltitude {
//			target = 20.0
//		}
//
//		// 🧠 При хорошем горизонте, плохом апогее — поднимаем чуть-чуть
//		if speedRatio > 0.8 && apogeeRatio < 1.0 {
//			target = 10.0
//		}
//
//		// 🛬 Если пошёл спад (начало падения)
//		if vVertical < -20 {
//			target = -10.0
//		}
//
//		// ⛔ Защита от бессмысленных превышений
//		if altitude > targetAlt*1.5 {
//			target = -30.0
//		}
//
//		// === Плавное сглаживание (альфа-фильтр)
//		alpha := 0.05
//		currentPitch = (1-alpha)*currentPitch + alpha*target
//
//		return currentPitch // Без ограничений
//	}
//
// var CurrentThrottle float64 = 1.0 // можно обернуть в AutopilotState
//
// func ComputeThrottle(
//
//	vHorizontal, aHorizontal float64,
//	vVertical, aVertical float64,
//	altitude, pitchDeg float64,
//	twr float64,
//	requiredOrbitalV, maxThrottle float64,
//
//	) float64 {
//		targetThrottle := maxThrottle
//
//		// --- ГОРИЗОНТАЛЬНЫЙ ДЕФИЦИТ ---
//		speedDeficit := math.Max(0.0, requiredOrbitalV-vHorizontal) / requiredOrbitalV
//		pitchRad := pitchDeg * math.Pi / 180.0
//		angleFactor := math.Pow(math.Cos(pitchRad), 0.5) // Влияние направления тяги
//		correctionH := speedDeficit * angleFactor
//
//		// --- ВЕРТИКАЛЬНЫЙ ФАКТОР ---
//		verticalFactor := 1.0
//		if vVertical > 0 {
//			if vVertical > 500 {
//				verticalFactor = 0.5
//			} else {
//				verticalFactor = 1.0 - (vVertical/1000.0)*0.5
//			}
//		}
//		// Добавочная тяга при слабом вертикальном ускорении и недоборе высоты
//		if vVertical < 300 && aVertical < 2.0 {
//			verticalFactor += 0.3
//		}
//
//		// --- ВЛИЯНИЕ ВЫСОТЫ (сглаженное) ---
//		altitudeFactor := 1.0 - math.Min((altitude-100_000)/250_000, 1.0)*0.2
//
//		// --- ГОРИЗОНТАЛЬНОЕ УСКОРЕНИЕ (насколько эффективно работаем вбок) ---
//		var aHFactor float64
//		if aHorizontal < 1.0 {
//			aHFactor = 1.2
//		} else if aHorizontal > 10.0 {
//			aHFactor = 0.8
//		} else {
//			scale := (aHorizontal - 1.0) / 9.0
//			aHFactor = 1.2 - scale*0.4
//		}
//
//		// --- КОМБИНИРОВАННАЯ КОРРЕКЦИЯ ТЯГИ ---
//		correction := 1.0 + correctionH + verticalFactor*0.5
//		correction = math.Min(1.5, correction) // Ограничим усиление
//		correction *= altitudeFactor * aHFactor
//
//		targetThrottle *= correction
//
//		// --- TWR-МОДЕРАЦИЯ (мягкая) ---
//		targetTWR := 1.5
//		if twr > targetTWR {
//			scale := targetTWR / twr
//			targetThrottle *= scale
//		} else if twr < 1.0 {
//			targetThrottle = 1.0 // Падаем — надо в пол
//		}
//
//		// --- ПЛАВНОСТЬ РЕАКЦИИ ---
//		// Чем ближе к орбите — тем мягче меняем тягу
//		rateLimit := 0.02
//		if vHorizontal > 7000 {
//			rateLimit = 0.005
//		}
//
//		// Плавное приближение к цели
//		diff := targetThrottle - CurrentThrottle
//		if math.Abs(diff) < rateLimit {
//			CurrentThrottle = targetThrottle
//		} else if diff > 0 {
//			CurrentThrottle += rateLimit
//		} else {
//			CurrentThrottle -= rateLimit
//		}
//
//		// Защита от выхода за пределы
//		return math.Max(0.0, math.Min(CurrentThrottle, maxThrottle))
//	}
func EarthRotationSpeedAtLatitude(latitude float64) float64 {
	const omega = 7.2921159e-5 // рад/с
	const radius = 6371000.0   // м
	latRad := latitude * math.Pi / 180
	return omega * radius * math.Cos(latRad) // в м/с
}
