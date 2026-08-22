package env

// -----------------------------------------------------------------------------
// Телеметрия отработавшей ступени.
//
// Эти снимки описывают тело, которое носителю уже не принадлежит: отделившуюся
// ступень, её тепловые узлы и поле обломков. Раньше они лежали вместе
// с телеметрией ракеты, хотя относятся к другому телу и другой части модели.
// -----------------------------------------------------------------------------
// ThermalNodeTelemetry — состояние одного участка конструкции.
type ThermalNodeTelemetry struct {
	Name        string  `json:"name"`
	Temperature float64 `json:"temperature"` // °C
	PeakTemp    float64 `json:"peakTemp"`    // °C
	HeatFlux    float64 `json:"heatFlux"`    // Вт/м²
	MeltingsAt  float64 `json:"meltsAt"`     // °C
}

// SpentStageTelemetry — состояние отработавшей ступени.
type SpentStageTelemetry struct {
	Name string `json:"name"`

	// Attached сообщает, что ступень ещё в составе носителя.
	//
	// До разделения телеметрия ступени повторяет состояние носителя: ступень
	// действительно летит, просто не сама по себе. Но радиосвязь в этот момент
	// не может пропасть по определению — терять связь не с чем, — и показывать
	// уверенное «связь есть» с первой секунды полёта значит сделать индикатор
	// бесполезным. Величины, лишённые смысла до разделения, публикуются
	// как неопределённые.
	Attached bool `json:"attached"`

	Outcome   string  `json:"outcome"`
	Reason    string  `json:"reason"`
	Altitude  float64 `json:"altitude"` // м
	Latitude  float64 `json:"lat"`
	Longitude float64 `json:"lon"`
	Speed     float64 `json:"speed"` // м/с относительно воздуха
	Mach      float64 `json:"mach"`
	HeatFlux  float64 `json:"heatFlux"` // Вт/м²
	DynamicQ  float64 `json:"dynamicQ"` // Па

	// Ориентация и вращение.
	AngleOfAttack float64 `json:"angleOfAttack"` // градусы
	SpinRate      float64 `json:"spinRate"`      // град/с
	Pitch         float64 `json:"pitch"`
	Yaw           float64 `json:"yaw"`
	Roll          float64 `json:"roll"`
	Tumbling      bool    `json:"tumbling"`

	// Тепловые узлы.
	HottestNode float64                `json:"hottestNode"` // °C
	Nodes       []ThermalNodeTelemetry `json:"nodes"`

	// Ударный слой.
	ShockTemp          float64 `json:"shockTemp"` // °C
	IonizationFraction float64 `json:"ionizationFraction"`
	PlasmaFrequency    float64 `json:"plasmaFrequency"` // Гц
	RadioBlackout      bool    `json:"radioBlackout"`
	Luminous           bool    `json:"luminous"`

	MaxAltitude  float64 `json:"maxAltitude"`
	PeakHeatFlux float64 `json:"peakHeatFlux"`
	PeakDynamicQ float64 `json:"peakDynamicQ"`
	PeakSpinRate float64 `json:"peakSpinRate"` // град/с
	OutcomeTime  float64 `json:"outcomeTime"`

	// Debris — зона падения обломков, заполняется после разрушения.
	Debris *DebrisTelemetry `json:"debris,omitempty"`
}

// DebrisTelemetry — сводка по зоне падения обломков.
type DebrisTelemetry struct {
	BreakupLatitude  float64 `json:"breakupLat"`
	BreakupLongitude float64 `json:"breakupLon"`
	BreakupAltitude  float64 `json:"breakupAlt"`

	CenterLatitude  float64 `json:"centerLat"`
	CenterLongitude float64 `json:"centerLon"`
	SemiMajorAxis   float64 `json:"semiMajorAxis"` // м
	SemiMinorAxis   float64 `json:"semiMinorAxis"` // м
	BearingDegrees  float64 `json:"bearing"`

	SurvivingMassFraction float64 `json:"survivingMassFraction"`

	Fragments []FragmentTelemetry `json:"fragments"`
}

// FragmentTelemetry — судьба одного класса обломков.
type FragmentTelemetry struct {
	Name        string  `json:"name"`
	Mass        float64 `json:"mass"`
	Ballistic   float64 `json:"ballisticCoefficient"` // кг/м²
	Reached     bool    `json:"reachedSurface"`
	Latitude    float64 `json:"lat"`
	Longitude   float64 `json:"lon"`
	Downrange   float64 `json:"downrange"` // м
	FlightTime  float64 `json:"flightTime"`
	ImpactSpeed float64 `json:"impactSpeed"`
}
