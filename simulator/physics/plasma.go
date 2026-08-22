package physics

import "math"

// -----------------------------------------------------------------------------
// Ионизация ударного слоя и плазменная оболочка.
//
// При гиперзвуковом входе газ за ударной волной тормозится и разогревается
// до нескольких тысяч кельвинов. Выше примерно 4000 К начинается заметная
// термическая ионизация: молекулы диссоциируют, атомы теряют электроны.
//
// Образующаяся плазма отражает радиоволны ниже плазменной частоты — это и есть
// радиоблэкаут, который наблюдается при входе спускаемых аппаратов.
// -----------------------------------------------------------------------------

const (
	// boltzmann — постоянная Больцмана, Дж/К.
	boltzmann = 1.380649e-23

	// electronMass — масса электрона, кг.
	electronMass = 9.1093837015e-31

	// electronCharge — элементарный заряд, Кл.
	electronCharge = 1.602176634e-19

	// vacuumPermittivity — электрическая постоянная, Ф/м.
	vacuumPermittivity = 8.8541878128e-12

	// planck — постоянная Планка, Дж·с.
	planck = 6.62607015e-34

	// airIonizationEnergy — эффективная энергия ионизации воздуха, Дж.
	// Взвешенное значение для смеси азота (14.5 эВ) и кислорода (12.1 эВ),
	// с учётом того, что кислород диссоциирует и ионизируется первым.
	airIonizationEnergy = 13.6 * electronCharge

	// airMeanMolarMass — средняя молярная масса воздуха, кг/моль.
	airMeanMolarMass = 0.02896

	// avogadro — число Авогадро, 1/моль.
	avogadro = 6.02214076e23

	// ionizationOnsetTemperature — температура, ниже которой ионизацией
	// можно пренебречь, К.
	ionizationOnsetTemperature = 2500.0
)

// PlasmaState — состояние ударного слоя.
type PlasmaState struct {
	// ShockTemperature — температура газа за ударной волной, К.
	ShockTemperature float64

	// IonizationFraction — доля ионизованных частиц, 0…1.
	IonizationFraction float64

	// ElectronDensity — концентрация свободных электронов, 1/м³.
	ElectronDensity float64

	// PlasmaFrequency — плазменная частота, Гц. Радиоволны ниже неё
	// не проходят сквозь оболочку.
	PlasmaFrequency float64

	// RadioBlackout сообщает, что связь в S-диапазоне (2.2 ГГц) невозможна.
	RadioBlackout bool

	// Luminous сообщает, что ударный слой светится видимым светом.
	Luminous bool
}

// ShockTemperature возвращает температуру газа за прямой ударной волной.
//
// При гиперзвуковом обтекании почти вся кинетическая энергия набегающего потока
// переходит в тепловую:
//
//	T₂ ≈ T∞ · (1 + (γ−1)/2 · M²)
//
// При больших числах Маха реальный газ поглощает часть энергии на диссоциацию
// и ионизацию, поэтому температура растёт медленнее идеального предсказания.
// Учитывается поправочным множителем, выходящим на насыщение.
func ShockTemperature(ambientK, mach float64) float64 {
	if ambientK <= 0 || mach <= 1 {
		return ambientK
	}

	ideal := ambientK * (1 + (GammaAir-1)/2*mach*mach)

	// Поправка на реальный газ: при M > 6 заметная часть энергии уходит
	// в диссоциацию, и температура оказывается ниже идеальной в полтора-два раза.
	if mach > 6 {
		excess := math.Min((mach-6)/10, 1)
		ideal /= 1 + excess
	}
	return ideal
}

// IonizationFraction решает уравнение Саха относительно степени ионизации.
//
//	x² / (1 − x²) = (2πm_e·kT)^(3/2) / (h³·n) · exp(−E_и / kT)
//
// где n — концентрация тяжёлых частиц. Решение квадратного уравнения:
//
//	x = √(S / (1 + S)),  S — правая часть.
func IonizationFraction(temperature, density float64) float64 {
	if temperature < ionizationOnsetTemperature || density <= 0 {
		return 0
	}

	// Концентрация частиц из плотности газа.
	n := density * avogadro / airMeanMolarMass
	if n <= 0 {
		return 0
	}

	kT := boltzmann * temperature
	exponent := -airIonizationEnergy / kT

	// При очень низких температурах экспонента обнуляется, при очень высоких
	// показатель мал по модулю — переполнения не возникает.
	if exponent < -700 {
		return 0
	}

	thermal := math.Pow(2*math.Pi*electronMass*kT, 1.5) / (planck * planck * planck)
	s := thermal / n * math.Exp(exponent)

	if math.IsInf(s, 1) {
		return 1
	}
	return math.Sqrt(s / (1 + s))
}

// PlasmaFrequency возвращает плазменную частоту, Гц.
//
//	f_p = 1/(2π) · √(n_e·e² / (ε₀·m_e))
//
// Радиосигнал с частотой ниже f_p отражается от плазмы и не проходит наружу.
func PlasmaFrequency(electronDensity float64) float64 {
	if electronDensity <= 0 {
		return 0
	}
	omega := math.Sqrt(electronDensity * electronCharge * electronCharge /
		(vacuumPermittivity * electronMass))
	return omega / (2 * math.Pi)
}

// AnalyzePlasma рассчитывает состояние ударного слоя по параметрам полёта.
//
// sBandFrequency — несущая частота телеметрии, Гц. Для S-диапазона 2.2 ГГц.
func AnalyzePlasma(ambientK, density, speed, sBandFrequency float64) PlasmaState {
	state := PlasmaState{ShockTemperature: ambientK}

	if density <= 0 || speed <= 0 {
		return state
	}

	sound := SpeedOfSound(ambientK)
	if sound <= 0 {
		return state
	}
	mach := speed / sound

	state.ShockTemperature = ShockTemperature(ambientK, mach)

	// Газ за ударной волной сжат: для сильной волны плотность растёт
	// примерно в (γ+1)/(γ−1) = 6 раз.
	shockDensity := density * (GammaAir + 1) / (GammaAir - 1)

	state.IonizationFraction = IonizationFraction(state.ShockTemperature, shockDensity)

	heavyDensity := shockDensity * avogadro / airMeanMolarMass
	state.ElectronDensity = state.IonizationFraction * heavyDensity
	state.PlasmaFrequency = PlasmaFrequency(state.ElectronDensity)

	state.RadioBlackout = state.PlasmaFrequency > sBandFrequency

	// Ударный слой начинает светиться примерно с 3000 К: возбуждённые
	// молекулы азота и кислорода высвечивают энергию в видимом диапазоне.
	state.Luminous = state.ShockTemperature > 3000

	return state
}

// DefaultTelemetryFrequency — несущая частота телеметрии, Гц (S-диапазон).
const DefaultTelemetryFrequency = 2.2e9
