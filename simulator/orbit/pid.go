package orbit

import (
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// PIDController — ПИД-регулятор с защитой от накопления интеграла.
//
// Раньше в проекте регуляторы создавались, но ComputePID не вызывался нигде:
// наведение работало по жёстко зашитым таблицам «высота → угол». Здесь
// регуляторы действительно замыкают контуры по апоцентру, наклонению
// и радиальной скорости.
type PIDController struct {
	Kp float64
	Ki float64
	Kd float64

	MinOutput float64
	MaxOutput float64

	// IntegralLimit ограничивает вклад интегрального члена в выход
	// (anti-windup в единицах выхода, а не в единицах ошибки).
	IntegralLimit float64

	// DerivativeFilterTime — постоянная времени однополюсного фильтра
	// производной, с. Ноль (по умолчанию) выключает фильтр — производная
	// считается сырой конечной разностью, как и раньше, это сохраняет
	// поведение всех контуров, для которых фильтр явно не включён.
	// Обязателен там, где Kd != 0, а измеряемая величина приходит с
	// датчика: без фильтра шум датчика усиливается пропорционально Kd/dt.
	DerivativeFilterTime float64

	integral           float64
	lastError          float64
	filteredDerivative float64
	initialized        bool
}

// Update вычисляет управляющее воздействие по ошибке за шаг dt.
func (p *PIDController) Update(setpoint, measured, dt float64) float64 {
	if dt <= 0 {
		return physics.Clamp(p.Kp*(setpoint-measured), p.MinOutput, p.MaxOutput)
	}

	err := setpoint - measured

	// Производная по первому шагу не определена — берём нулевую,
	// иначе на старте регулятор выдаёт бросок.
	derivative := 0.0
	if p.initialized {
		raw := (err - p.lastError) / dt
		if p.DerivativeFilterTime > 0 {
			decay := math.Exp(-dt / p.DerivativeFilterTime)
			p.filteredDerivative = raw + (p.filteredDerivative-raw)*decay
			derivative = p.filteredDerivative
		} else {
			derivative = raw
		}
	}
	p.lastError = err
	p.initialized = true

	proportional := p.Kp * err
	p.integral += err * dt
	integralTerm := p.Ki * p.integral

	// Anti-windup: ограничиваем именно вклад в выход и пересчитываем интеграл
	// обратно, чтобы он не «уезжал» в насыщении.
	if p.IntegralLimit > 0 {
		if integralTerm > p.IntegralLimit {
			integralTerm = p.IntegralLimit
			if p.Ki != 0 {
				p.integral = integralTerm / p.Ki
			}
		} else if integralTerm < -p.IntegralLimit {
			integralTerm = -p.IntegralLimit
			if p.Ki != 0 {
				p.integral = integralTerm / p.Ki
			}
		}
	}

	output := proportional + integralTerm + p.Kd*derivative
	if math.IsNaN(output) {
		return 0
	}
	return physics.Clamp(output, p.MinOutput, p.MaxOutput)
}

// Reset сбрасывает внутреннее состояние регулятора.
func (p *PIDController) Reset() {
	p.integral = 0
	p.lastError = 0
	p.filteredDerivative = 0
	p.initialized = false
}
