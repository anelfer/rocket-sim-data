package control

import "math"

// -----------------------------------------------------------------------------
// Активное воздействие.
//
// Команда живёт во времени: скачок действует до отмены, плавное изменение
// проходит по профилю, временное воздействие само снимается и возвращает
// параметр в исходное или номинальное состояние.
//
// Всё считается от модельного времени, а не от реального. Иначе при ускорении
// симуляции в десять раз пятисекундное воздействие длилось бы полсекунды
// модельного времени.
// -----------------------------------------------------------------------------

// Effect — воздействие, применяемое к модели на каждом шаге.
type Effect struct {
	Command Command `json:"command"`
	Param   Param   `json:"-"`

	// StartTime — модельное время начала воздействия, с.
	StartTime float64 `json:"startTime"`

	// Base — значение параметра в момент начала воздействия, единицы модели.
	Base float64 `json:"-"`

	// Target — конечное значение, единицы модели.
	Target float64 `json:"-"`

	// Nominal — штатное значение параметра, единицы модели.
	Nominal float64 `json:"-"`

	// Done означает, что воздействие завершилось и должно быть снято.
	Done bool `json:"done"`

	// Unsafe означает выход за допустимые пределы параметра.
	Unsafe bool `json:"unsafe"`

	// current — значение на последнем расчёте, единицы модели.
	current float64
}

// Value возвращает значение параметра на заданное модельное время.
func (e *Effect) Value(now float64) float64 {
	c := e.Command
	elapsed := now - e.StartTime
	if elapsed < 0 {
		return e.Base
	}

	switch c.Mode {
	case ModeStep, ModeSet, ModeRelative:
		e.current = e.Target
		return e.Target

	case ModeRamp:
		if c.Duration <= 0 || elapsed >= c.Duration {
			e.current = e.Target
			return e.Target
		}
		e.current = e.interpolate(elapsed / c.Duration)
		return e.current

	case ModeTimed:
		if elapsed >= c.Duration {
			// Воздействие отработало. Что дальше — решает поле After.
			e.Done = true
			switch c.After {
			case AfterHold:
				e.current = e.Target
				return e.Target
			case AfterRevertPrevious:
				e.current = e.Base
				return e.Base
			default:
				e.current = e.Nominal
				return e.Nominal
			}
		}
		e.current = e.interpolate(elapsed / c.Duration)
		return e.current
	}

	e.current = e.Target
	return e.Target
}

// interpolate возвращает значение по профилю перехода на доле пути u ∈ [0,1].
func (e *Effect) interpolate(u float64) float64 {
	u = math.Max(0, math.Min(1, u))
	c := e.Command
	span := e.Target - e.Base

	switch c.Curve {
	case CurveSmooth:
		// Классический сглаженный переход: нулевая производная на обоих концах,
		// поэтому изменение начинается и заканчивается без рывка.
		return e.Base + span*u*u*(3-2*u)

	case CurveStaircase:
		steps := 4.0
		if c.Period > 0 && c.Duration > 0 {
			steps = math.Max(1, math.Round(c.Duration/c.Period))
		}
		return e.Base + span*math.Floor(u*steps)/steps

	case CurvePulse:
		// Импульс: подъём до цели к середине интервала и возврат к концу.
		var shape float64
		if u < 0.5 {
			shape = 2 * u
		} else {
			shape = 2 * (1 - u)
		}
		return e.Base + span*shape

	case CurvePeriodic:
		period := c.Period
		if period <= 0 {
			period = math.Max(c.Duration, 1)
		}
		phase := 2 * math.Pi * u * c.Duration / period
		return e.Base + span*math.Sin(phase)

	default: // CurveLinear
		return e.Base + span*u
	}
}

// Current возвращает последнее вычисленное значение в единицах модели.
func (e *Effect) Current() float64 { return e.current }

// View — представление воздействия для интерфейса.
type View struct {
	ID        string  `json:"id"`
	Parameter string  `json:"parameter"`
	Engine    string  `json:"engine,omitempty"`
	Title     string  `json:"title"`
	Subsystem string  `json:"subsystem"`
	Mode      Mode    `json:"mode"`
	Curve     Curve   `json:"curve,omitempty"`
	Failure   Failure `json:"failure,omitempty"`
	Unit      string  `json:"unit"`

	// Значения в единицах интерфейса.
	Base    float64 `json:"base"`
	Target  float64 `json:"target"`
	Current float64 `json:"current"`
	Nominal float64 `json:"nominal"`

	StartTime float64 `json:"startTime"`
	Duration  float64 `json:"duration,omitempty"`
	Remaining float64 `json:"remaining,omitempty"`

	Unsafe   bool   `json:"unsafe"`
	Direct   bool   `json:"direct"`
	Operator string `json:"operator"`
	Source   string `json:"source"`
}

// view строит представление воздействия на заданное модельное время.
func (e *Effect) view(now float64) View {
	p := e.Param
	v := View{
		ID:        e.Command.ID,
		Parameter: e.Command.Parameter,
		Engine:    e.Command.Engine,
		Title:     p.Title,
		Subsystem: p.Subsystem,
		Mode:      e.Command.Mode,
		Curve:     e.Command.Curve,
		Failure:   e.Command.Failure,
		Unit:      p.Unit,
		Base:      p.ToDisplay(e.Base),
		Target:    p.ToDisplay(e.Target),
		Current:   p.ToDisplay(e.current),
		Nominal:   p.ToDisplay(e.Nominal),
		StartTime: e.StartTime,
		Duration:  e.Command.Duration,
		Unsafe:    e.Unsafe,
		Direct:    e.Command.Direct,
		Operator:  e.Command.Operator,
		Source:    e.Command.Source,
	}
	if e.Command.Duration > 0 {
		v.Remaining = math.Max(0, e.StartTime+e.Command.Duration-now)
	}
	return v
}
