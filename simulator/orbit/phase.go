// Package orbit содержит систему управления полётом (наведение, навигация,
// стабилизация) и определение фаз полёта.
package orbit

// FlightPhase — фаза полёта.
type FlightPhase int

const (
	PhasePreLaunch       FlightPhase = iota // на стартовом столе
	PhaseFirstStage                         // работа первой ступени
	PhaseCoast1                             // пауза между MECO и запуском второй ступени
	PhaseSecondStage                        // работа второй ступени
	PhaseCoast2                             // баллистический полёт к апоцентру
	PhaseCircularization                    // довыведение в апоцентре
	PhaseOrbital                            // орбитальный полёт
	PhaseDeorbit                            // тормозной импульс схода с орбиты
	PhaseEntry                              // торможение в атмосфере брюхом
	PhaseLandingFlip                        // разворот из горизонтали в вертикаль
	PhaseLandingBurn                        // посадочный импульс
	PhaseLanded                             // на поверхности
)

// String возвращает название фазы.
func (p FlightPhase) String() string {
	switch p {
	case PhasePreLaunch:
		return "Pre-Launch"
	case PhaseFirstStage:
		return "First Stage Burn"
	case PhaseCoast1:
		return "Stage Separation Coast"
	case PhaseSecondStage:
		return "Second Stage Burn"
	case PhaseCoast2:
		return "Coast to Apoapsis"
	case PhaseCircularization:
		return "Circularization Burn"
	case PhaseOrbital:
		return "Orbital Flight"
	case PhaseDeorbit:
		return "Deorbit Burn"
	case PhaseEntry:
		return "Atmospheric Entry"
	case PhaseLandingFlip:
		return "Landing Flip"
	case PhaseLandingBurn:
		return "Landing Burn"
	case PhaseLanded:
		return "Landed"
	default:
		return "Unknown"
	}
}

// Powered сообщает, работают ли в этой фазе двигатели.
func (p FlightPhase) Powered() bool {
	switch p {
	case PhaseFirstStage, PhaseSecondStage, PhaseCircularization,
		PhaseDeorbit, PhaseLandingBurn:
		return true
	default:
		return false
	}
}
