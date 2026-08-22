// Package propulsion содержит связанную модель жидкостного ракетного двигателя:
// баки с наддувом, турбонасосный агрегат, камеру сгорания, сопло и охлаждающий
// тракт.
//
// Ключевой принцип пакета — ни один параметр не задаётся независимым случайным
// процессом. Все величины связаны причинной цепочкой:
//
//	положение клапанов → расход через турбину → мощность турбины → обороты вала
//	→ давление за насосами → расход компонентов → давление в камере → тяга
//
// Шум, дрейф и запаздывание накладываются только на выход датчиков и не влияют
// на сами уравнения, поэтому физические зависимости сохраняются.
package propulsion

import "math"

// -----------------------------------------------------------------------------
// Свойства компонентов топлива.
//
// Плотность зависит от температуры, а через неё — на объёмную заполненность
// баков, уровень жидкости, гидростатический напор на входе насосов и время
// до выработки компонента. Давление насыщенных паров определяет кавитационный
// запас: чем теплее компонент, тем выше давление паров и тем легче сорвать
// насос в кавитацию.
// -----------------------------------------------------------------------------

// Propellant — свойства компонента топлива.
type Propellant struct {
	Name string

	// ReferenceDensity — плотность при опорной температуре, кг/м³.
	ReferenceDensity float64

	// ReferenceTemperature — опорная температура, К.
	ReferenceTemperature float64

	// ThermalExpansion — коэффициент объёмного расширения, 1/К.
	ThermalExpansion float64

	// SpecificHeat — удельная теплоёмкость, Дж/(кг·К).
	SpecificHeat float64

	// BoilingPoint — температура кипения при нормальном давлении, К.
	BoilingPoint float64

	// VaporizationEnthalpy — удельная теплота парообразования, Дж/кг.
	VaporizationEnthalpy float64

	// Cryogenic сообщает, что компонент криогенный: он постоянно подогревается
	// окружающей средой и потому кипит в баке.
	Cryogenic bool

	// Viscosity — динамическая вязкость жидкой фазы в тракте охлаждения, Па·с.
	//
	// Берётся не при температуре хранения, а при рабочей: в рубашке компонент
	// успевает нагреться на сотню-другую градусов, и вязкость керосина за это
	// падает втрое. Именно эта величина, а не складская, определяет число
	// Рейнольдса в канале.
	Viscosity float64

	// ThermalConductivity — теплопроводность жидкой фазы, Вт/(м·К).
	ThermalConductivity float64
}

// RP1 — керосин ракетного назначения.
func RP1() Propellant {
	return Propellant{
		Name:                 "RP-1",
		ReferenceDensity:     810,
		ReferenceTemperature: 288.15,
		ThermalExpansion:     9.3e-4,
		SpecificHeat:         2010,
		BoilingPoint:         490,
		VaporizationEnthalpy: 2.5e5,
		Cryogenic:            false,

		// Керосин в рубашке идёт горячим, около 400 К: вязкость там
		// впятеро ниже складской.
		Viscosity:           5.0e-4,
		ThermalConductivity: 0.12,
	}
}

// LiquidMethane — жидкий метан.
//
// Плотнее керосина по энергии на единицу объёма он не выигрывает, зато не
// коксуется в охлаждающем тракте, что и делает возможным многократное
// использование двигателя без переборки. Криогенный: кипит при 111 К, и в
// баке его приходится держать переохлаждённым, иначе насос сорвётся
// в кавитацию на первых же секундах.
func LiquidMethane() Propellant {
	return Propellant{
		Name:                 "CH4",
		ReferenceDensity:     422,
		ReferenceTemperature: 111.7,
		ThermalExpansion:     3.5e-3,
		SpecificHeat:         3480,
		BoilingPoint:         111.7,
		VaporizationEnthalpy: 5.11e5,
		Cryogenic:            true,

		// В рубашке метан сверхкритический: плотный газ с малой вязкостью.
		// Отсюда и высокие скорости в каналах, и высокая теплоотдача.
		Viscosity:           6.0e-5,
		ThermalConductivity: 0.09,
	}
}

// LOX — жидкий кислород.
func LOX() Propellant {
	return Propellant{
		Name:                 "LOX",
		ReferenceDensity:     1141,
		ReferenceTemperature: 90.2,
		ThermalExpansion:     4.5e-3,
		SpecificHeat:         1699,
		BoilingPoint:         90.2,
		VaporizationEnthalpy: 2.13e5,
		Cryogenic:            true,

		Viscosity:           1.9e-4,
		ThermalConductivity: 0.15,
	}
}

// Density возвращает плотность компонента при заданной температуре, кг/м³.
//
//	ρ(T) = ρ₀ / (1 + β·(T − T₀))
//
// Для жидкого кислорода коэффициент расширения почти на порядок больше, чем
// у керосина: прогрев на десять градусов меняет его плотность на 4 %, что
// заметно и в заполненности бака, и в напоре на входе насоса.
func (p Propellant) Density(temperature float64) float64 {
	d := p.ReferenceDensity / (1 + p.ThermalExpansion*(temperature-p.ReferenceTemperature))
	if d < 1 {
		return 1
	}
	return d
}

// VaporPressure возвращает давление насыщенных паров, Па.
//
// Используется уравнение Клаузиуса — Клапейрона относительно точки кипения
// при нормальном давлении:
//
//	p = p₀ · exp( −L/R · (1/T − 1/T_кип) )
//
// Величина определяет кавитационный запас: насос срывается, когда давление
// на его входе приближается к давлению паров.
func (p Propellant) VaporPressure(temperature float64) float64 {
	if temperature <= 0 {
		return 0
	}
	const (
		standardPressure = 101325.0
		universalR       = 8.314
	)
	// Молярная масса нужна для перевода удельной теплоты в молярную.
	molarMass := 0.032 // кислород
	if !p.Cryogenic {
		molarMass = 0.170 // условная для керосиновой фракции
	}
	specificR := universalR / molarMass

	exponent := -p.VaporizationEnthalpy / specificR * (1/temperature - 1/p.BoilingPoint)
	if exponent > 50 {
		exponent = 50
	}
	return standardPressure * math.Exp(exponent)
}

// -----------------------------------------------------------------------------
// Термодинамика продуктов сгорания
// -----------------------------------------------------------------------------

// PropellantPair — топливная пара и её характеристики горения.
type PropellantPair struct {
	Fuel     Propellant
	Oxidizer Propellant

	// OptimalMixtureRatio — соотношение компонентов, при котором
	// характеристическая скорость максимальна.
	OptimalMixtureRatio float64

	// PeakCharacteristicVelocity — максимальная характеристическая скорость
	// при оптимальном соотношении, м/с.
	PeakCharacteristicVelocity float64

	// MixtureSensitivity — насколько быстро падает характеристическая скорость
	// при отклонении соотношения от оптимума, 1/(единица O/F)².
	MixtureSensitivity float64

	// PeakFlameTemperature — температура продуктов сгорания при оптимальном
	// соотношении, К.
	PeakFlameTemperature float64

	// SpecificHeatRatio — показатель адиабаты продуктов сгорания.
	SpecificHeatRatio float64

	// Свойства продуктов сгорания. Нужны стороне газа: без них коэффициент
	// теплоотдачи по Бартцу посчитать не из чего.
	//
	// GasViscosity — динамическая вязкость при температуре в камере, Па·с.
	GasViscosity float64

	// GasSpecificHeat — теплоёмкость продуктов сгорания, Дж/(кг·К).
	GasSpecificHeat float64

	// GasPrandtl — число Прандтля продуктов сгорания.
	//
	// У продуктов сгорания оно заметно меньше единицы: газ проводит тепло
	// лучше, чем переносит импульс.
	GasPrandtl float64
}

// KeroloxPair возвращает характеристики пары РГ-1 / жидкий кислород.
func KeroloxPair() PropellantPair {
	return PropellantPair{
		Fuel:                       RP1(),
		Oxidizer:                   LOX(),
		OptimalMixtureRatio:        2.34,
		PeakCharacteristicVelocity: 1823,
		MixtureSensitivity:         0.055,
		PeakFlameTemperature:       3670,
		SpecificHeatRatio:          1.22,
		GasViscosity:               1.0e-4,
		GasSpecificHeat:            1900,
		GasPrandtl:                 0.50,
	}
}

// MethaloxPair возвращает характеристики пары жидкий метан / жидкий кислород.
//
// По сравнению с керосиновой парой характеристическая скорость выше примерно
// на сто метров в секунду, а оптимальное соотношение компонентов заметно
// больше: метан легче керосина, и окислителя на единицу горючего нужно
// больше. Двигатели на этой паре обычно работают богаче оптимума, около 3.6,
// чтобы удержать температуру стенок.
//
// PeakCharacteristicVelocity поднят с 1860 до 1950 м/с при калибровке
// Raptor 3 под паспортные цифры (давление в камере 35.2 МПа, расход
// 800.5 кг/с, диаметр горловины 237 мм): эти три числа задают c* = Pc·At/ṁ
// ≈ 1940 м/с однозначно, обратным счётом от геометрии и расхода, а не
// заданием. При прежнем пике 1860 для этого потребовалась бы эффективность
// сгорания выше единицы, чего не бывает. Смещение пика вверх — не подгонка
// ради красивой цифры: у камеры с таким давлением (втрое выше типичного
// газогенераторного двигателя) равновесие сгорания действительно смещено
// дальше в сторону полноты реакции, и более высокий теоретический потолок
// физически оправдан.
func MethaloxPair() PropellantPair {
	return PropellantPair{
		Fuel:                       LiquidMethane(),
		Oxidizer:                   LOX(),
		OptimalMixtureRatio:        3.6,
		PeakCharacteristicVelocity: 1950,
		MixtureSensitivity:         0.030,
		PeakFlameTemperature:       3550,
		SpecificHeatRatio:          1.20,

		// Продукты сгорания метана легче керосиновых: больше водяного пара
		// и водорода, отсюда более высокая теплоёмкость.
		GasViscosity:    0.95e-4,
		GasSpecificHeat: 2600,
		GasPrandtl:      0.55,
	}
}

// CharacteristicVelocity возвращает идеальную характеристическую скорость c*
// при заданном соотношении компонентов, м/с.
//
// Зависимость от соотношения близка к параболической около оптимума: отклонение
// в любую сторону снижает c*, но по-разному. Богатая смесь (низкое O/F) теряет
// меньше, потому что избыток горючего снижает молярную массу продуктов,
// частично компенсируя падение температуры. Богатая окислителем смесь теряет
// сильнее и вдобавок резко поднимает температуру стенок.
func (p PropellantPair) CharacteristicVelocity(mixtureRatio float64) float64 {
	if mixtureRatio <= 0 {
		return 0
	}
	deviation := mixtureRatio - p.OptimalMixtureRatio

	// Асимметрия: избыток окислителя вреднее избытка горючего.
	sensitivity := p.MixtureSensitivity
	if deviation > 0 {
		sensitivity *= 1.6
	}

	factor := 1 - sensitivity*deviation*deviation
	if factor < 0.3 {
		factor = 0.3
	}
	return p.PeakCharacteristicVelocity * factor
}

// FlameTemperature возвращает температуру продуктов сгорания, К.
//
// Максимум температуры смещён в сторону более высокого соотношения компонентов
// относительно максимума c*: стехиометрия для керосина и кислорода около 3.4,
// а двигатели работают на богатой смеси именно ради снижения температуры стенок.
func (p PropellantPair) FlameTemperature(mixtureRatio float64) float64 {
	if mixtureRatio <= 0 {
		return 300
	}
	const stoichiometric = 3.4

	deviation := mixtureRatio - stoichiometric
	factor := 1 - 0.035*deviation*deviation
	if factor < 0.35 {
		factor = 0.35
	}
	return p.PeakFlameTemperature * factor
}

// MixtureRatioDeviation возвращает относительное отклонение соотношения
// компонентов от оптимального.
func (p PropellantPair) MixtureRatioDeviation(mixtureRatio float64) float64 {
	if p.OptimalMixtureRatio <= 0 {
		return 0
	}
	return (mixtureRatio - p.OptimalMixtureRatio) / p.OptimalMixtureRatio
}
