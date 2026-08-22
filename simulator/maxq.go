package simulator

import "rocketTelemetrySim/simulator/physics"

// -----------------------------------------------------------------------------
// Прохождение области максимального скоростного напора.
//
// Прежняя реализация была конечным автоматом с защёлками HasPassedMaxQ /
// MaxQReached и жёстким окном по высоте 8–18 км. Стоило траектории отличаться
// от заложенной — и защёлка срабатывала не там, где надо. Здесь закон
// дросселирования зависит только от текущего напора, поэтому он самонастраивается
// под любую траекторию, а состояние нужно исключительно для телеметрии.
// -----------------------------------------------------------------------------

// MaxQState хранит статистику прохождения Max-Q.
type MaxQState struct {
	PeakDynamicPressure float64 // максимальный достигнутый напор, Па
	PeakTime            float64 // момент достижения, с
	PeakAltitude        float64 // высота в этот момент, м
	PeakVelocity        float64 // скорость в этот момент, м/с
	Announced           bool    // событие уже записано в журнал
}

const (
	// maxQThrottleDownStart — напор, с которого начинается дросселирование.
	maxQThrottleDownStart = 26000.0 // Па

	// maxQThrottleDownFull — напор, при котором достигается минимальный газ.
	maxQThrottleDownFull = 34000.0 // Па

	// maxQMinThrottle — минимальный уровень газа при прохождении Max-Q.
	maxQMinThrottle = 0.65
)

// Observe обновляет статистику по текущему состоянию.
func (s *MaxQState) Observe(dynamicPressure, altitude, velocity, t float64) {
	if dynamicPressure > s.PeakDynamicPressure {
		s.PeakDynamicPressure = dynamicPressure
		s.PeakTime = t
		s.PeakAltitude = altitude
		s.PeakVelocity = velocity
	}
}

// MaxQThrottle возвращает допустимый уровень газа при заданном скоростном
// напоре. Зависимость непрерывная и монотонная: газ убирается при подходе
// к области максимального напора и возвращается сам, когда напор спадает.
func MaxQThrottle(dynamicPressure float64) float64 {
	if dynamicPressure <= maxQThrottleDownStart {
		return 1.0
	}
	if dynamicPressure >= maxQThrottleDownFull {
		return maxQMinThrottle
	}
	progress := (dynamicPressure - maxQThrottleDownStart) /
		(maxQThrottleDownFull - maxQThrottleDownStart)
	// Сглаживание (smoothstep) — чтобы команда не имела излома по производной.
	smooth := progress * progress * (3 - 2*progress)
	return 1.0 - (1.0-maxQMinThrottle)*smooth
}

// SmoothThrottleTransition ограничивает скорость изменения газа.
func SmoothThrottleTransition(current, target, maxRate, dt float64) float64 {
	maxChange := maxRate * dt
	diff := target - current
	if diff > maxChange {
		return current + maxChange
	}
	if diff < -maxChange {
		return current - maxChange
	}
	return target
}

// DynamicPressure — скоростной напор, Па.
func DynamicPressure(density, speed float64) float64 {
	return physics.DynamicPressure(density, speed)
}
