package control

import (
	"errors"
	"fmt"
	"math"
	"strings"
)

// -----------------------------------------------------------------------------
// Команда управления
//
// Интерфейс не обращается к внутренним переменным симулятора. Единственный
// способ вмешаться в модель — команда, которая проходит проверку, попадает
// в журнал и превращается в поправку к физической модели.
// -----------------------------------------------------------------------------

// Mode — способ воздействия на параметр.
type Mode string

const (
	// ModeSet задаёт целевое значение. Модель отрабатывает его с той
	// скоростью, на какую способна.
	ModeSet Mode = "set"

	// ModeStep меняет значение скачком.
	ModeStep Mode = "step"

	// ModeRamp меняет значение по заданному профилю за указанное время.
	ModeRamp Mode = "ramp"

	// ModeRelative меняет значение относительно текущего или номинального.
	ModeRelative Mode = "relative"

	// ModeTimed задаёт воздействие на ограниченное время с последующим
	// возвратом.
	ModeTimed Mode = "timed"

	// ModeFailure переводит агрегат в состояние отказа.
	ModeFailure Mode = "failure"

	// ModeRelease снимает ранее наложенное воздействие.
	ModeRelease Mode = "release"
)

// Curve — профиль перехода для ModeRamp и ModeTimed.
type Curve string

const (
	CurveLinear    Curve = "linear"    // равномерно
	CurveSmooth    Curve = "smooth"    // с плавным началом и концом
	CurveStaircase Curve = "staircase" // ступенями
	CurvePulse     Curve = "pulse"     // импульс с возвратом
	CurvePeriodic  Curve = "periodic"  // периодические колебания вокруг базы
)

// Basis — относительно чего отсчитывается значение.
type Basis string

const (
	BasisAbsolute       Basis = "absolute"        // абсолютное значение в единицах параметра
	BasisPercentCurrent Basis = "percent_current" // процент от текущего значения
	BasisPercentNominal Basis = "percent_nominal" // процент от номинального значения
	BasisDelta          Basis = "delta"           // приращение в единицах параметра
)

// Failure — вид отказа для ModeFailure.
type Failure string

const (
	FailureStopped      Failure = "stopped"      // остановлен
	FailureStuck        Failure = "stuck"        // заклинил
	FailureUnpowered    Failure = "unpowered"    // обесточен
	FailureUnresponsive Failure = "unresponsive" // не отвечает на команды
	FailureFrozen       Failure = "frozen"       // работает с фиксированным значением
	FailureBadSensor    Failure = "bad_sensor"   // выдаёт некорректную телеметрию
	FailureRuptured     Failure = "ruptured"     // разрушен, разрыв магистрали
)

// AfterAction — что происходит по завершении временного воздействия.
type AfterAction string

const (
	AfterRevertPrevious AfterAction = "revert_previous" // вернуть прежнее значение
	AfterRevertNominal  AfterAction = "revert_nominal"  // вернуть номинал
	AfterHold           AfterAction = "hold"            // оставить достигнутое
)

// Status — состояние команды в её жизненном цикле.
type Status string

const (
	StatusQueued   Status = "queued"
	StatusSent     Status = "sent"
	StatusAccepted Status = "accepted"
	StatusApplied  Status = "applied"
	StatusRejected Status = "rejected"
	StatusExpired  Status = "expired"
	StatusFailed   Status = "failed"
	StatusReleased Status = "released"
)

// Command — единица воздействия.
type Command struct {
	// ID — идентификатор, назначаемый клиентом. Повторная отправка команды
	// с тем же идентификатором не применяется дважды.
	ID string `json:"id"`

	// Parameter — идентификатор управляемого параметра из реестра.
	Parameter string `json:"parameter"`

	// Engine — идентификатор двигателя, которому адресована команда,
	// например "S1-3". Пустое значение означает всю ступень.
	//
	// Параметры баков адресата не имеют: бак на ступени один, и команда
	// с указанным двигателем к нему всё равно относится целиком.
	Engine string `json:"engine,omitempty"`

	// Vehicle — аппарат, которому адресована команда: "" или "ship" —
	// активная ступень (как раньше, единственный получатель), "booster" —
	// возвращающийся бустер после отделения. У бустера свой пульт
	// (Simulation.BoosterBoard) и своя двигательная установка — команда
	// с этим полем не задевает корабль и наоборот.
	Vehicle string `json:"vehicle,omitempty"`

	Mode  Mode  `json:"mode"`
	Basis Basis `json:"basis,omitempty"`
	Curve Curve `json:"curve,omitempty"`

	// Value — целевое значение в единицах параметра либо процент,
	// в зависимости от Basis.
	Value float64 `json:"value"`

	// Duration — длительность перехода или временного воздействия, с
	// модельного времени.
	Duration float64 `json:"duration,omitempty"`

	// Period — период для CurvePeriodic и шаг для CurveStaircase, с.
	Period float64 `json:"period,omitempty"`

	// Failure — вид отказа для ModeFailure.
	Failure Failure `json:"failure,omitempty"`

	// After — действие по завершении временного воздействия.
	After AfterAction `json:"after,omitempty"`

	// Priority — приоритет. При совпадении параметра выигрывает больший.
	Priority int `json:"priority,omitempty"`

	// Source — источник команды: оператор, сценарий, автоматика.
	Source string `json:"source,omitempty"`

	// Operator — кто отдал команду.
	Operator string `json:"operator,omitempty"`

	// Unsafe разрешает выход за допустимые пределы параметра.
	Unsafe bool `json:"unsafe,omitempty"`

	// Direct включает прямую подмену состояния модели. Причинность при этом
	// нарушается, поэтому режим отмечается отдельно и в команде, и в журнале.
	Direct bool `json:"direct,omitempty"`

	// IssuedAtModelTime — модельное время, на которое рассчитана команда.
	// Ноль означает «немедленно».
	IssuedAtModelTime float64 `json:"issuedAtModelTime,omitempty"`

	// TTL — срок годности команды в секундах модельного времени. Команда,
	// пролежавшая в очереди дольше, отбраковывается как устаревшая.
	TTL float64 `json:"ttl,omitempty"`
}

// Result — ответ на команду.
type Result struct {
	ID     string `json:"id"`
	Status Status `json:"status"`
	Reason string `json:"reason,omitempty"`

	// ModelTime — модельное время применения.
	ModelTime float64 `json:"modelTime"`

	// Parameter — параметр, к которому относится команда.
	Parameter string `json:"parameter"`

	// Previous — значение до воздействия.
	Previous float64 `json:"previous"`

	// Target — вычисленное целевое значение в единицах параметра.
	Target float64 `json:"target"`

	// Unit — единица измерения параметра.
	Unit string `json:"unit"`

	// Warnings — предупреждения, выданные при проверке.
	Warnings []string `json:"warnings,omitempty"`
}

var (
	errNoParameter = errors.New("не указан параметр")
	errNoMode      = errors.New("не указан режим воздействия")
)

// Normalize приводит команду к каноническому виду и подставляет значения
// по умолчанию.
func (c *Command) Normalize() {
	c.Parameter = strings.TrimSpace(c.Parameter)
	c.Engine = strings.TrimSpace(c.Engine)
	if c.Engine == "*" || strings.EqualFold(c.Engine, "all") {
		c.Engine = ""
	}
	if c.Basis == "" {
		c.Basis = BasisAbsolute
	}
	if c.Curve == "" {
		c.Curve = CurveLinear
	}
	if c.After == "" {
		c.After = AfterRevertNominal
	}
	if c.Source == "" {
		c.Source = "operator"
	}
	if c.Operator == "" {
		c.Operator = "operator"
	}
	if c.Mode == ModeStep {
		c.Duration = 0
	}
}

// Validate проверяет команду вне контекста модели.
func (c Command) Validate() error {
	if c.Mode == "" {
		return errNoMode
	}
	if c.Mode != ModeRelease && c.Parameter == "" {
		return errNoParameter
	}
	if math.IsNaN(c.Value) || math.IsInf(c.Value, 0) {
		return fmt.Errorf("недопустимое значение %v", c.Value)
	}
	if c.Duration < 0 {
		return fmt.Errorf("отрицательная длительность %.3f с", c.Duration)
	}
	switch c.Mode {
	case ModeRamp:
		if c.Duration <= 0 {
			return errors.New("плавное изменение требует ненулевой длительности")
		}
	case ModeTimed:
		if c.Duration <= 0 {
			return errors.New("временное воздействие требует ненулевой длительности")
		}
	case ModeFailure:
		if c.Failure == "" {
			return errors.New("не указан вид отказа")
		}
	}
	if c.Curve == CurvePeriodic && c.Period <= 0 {
		return errors.New("периодический профиль требует ненулевого периода")
	}
	return nil
}

// resolveTarget вычисляет целевое значение в единицах параметра.
func (c Command) resolveTarget(current, nominal float64) float64 {
	switch c.Basis {
	case BasisPercentCurrent:
		return current * (1 + c.Value/100)
	case BasisPercentNominal:
		return nominal * c.Value / 100
	case BasisDelta:
		return current + c.Value
	default:
		return c.Value
	}
}
