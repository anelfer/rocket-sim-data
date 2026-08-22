package physics

import "math"

// -----------------------------------------------------------------------------
// Модель атмосферы: US Standard Atmosphere 1976 до 100 км,
// далее — стандартная термосфера/экзосфера до 1000 км.
// -----------------------------------------------------------------------------

// Узлы таблицы по высоте (м).
var altitudeNodes = []float64{
	0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000,
	10000, 11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 19000,
	20000, 21000, 22000, 23000, 24000, 25000, 26000, 27000, 28000, 29000,
	30000, 40000, 50000, 60000, 70000, 80000, 90000, 100000,
}

// Кинетическая температура (К) в узлах таблицы.
var temperatureNodes = []float64{
	288.15, 281.65, 275.15, 268.65, 262.15, 255.65, 249.15, 242.65, 236.15, 229.65,
	223.15, 216.65, 216.65, 216.65, 216.65, 216.65, 216.65, 216.65, 216.65, 216.65,
	216.65, 217.65, 218.65, 219.65, 220.65, 221.65, 222.65, 223.65, 224.65, 225.65,
	226.65, 250.35, 270.65, 247.02, 219.65, 198.64, 186.87, 195.08,
}

// Плотность (кг/м³) в узлах таблицы.
var densityNodes = []float64{
	1.2250, 1.1117, 1.0066, 0.90925, 0.81935, 0.73643, 0.66011, 0.59002, 0.52579, 0.46706,
	0.41351, 0.36480, 0.31194, 0.26660, 0.22786, 0.19475, 0.16647, 0.14230, 0.12165, 0.10400,
	0.088910, 0.075715, 0.064510, 0.055006, 0.046938, 0.040084, 0.034257, 0.029298, 0.025076, 0.021478,
	0.018410, 0.0039957, 0.0010269, 3.0592e-4, 8.2829e-5, 1.8458e-5, 3.4160e-6, 5.6040e-7,
}

// Узлы термосферы и экзосферы (100–1000 км), средняя солнечная активность.
// Плотность падает на девять порядков, поэтому интерполяция ведётся
// по логарифму — линейная интерполяция здесь даёт ошибку в разы.
var thermosphereAltNodes = []float64{
	100000, 110000, 120000, 130000, 150000, 180000, 200000, 250000, 300000,
	350000, 400000, 450000, 500000, 600000, 700000, 800000, 900000, 1000000,
}

var thermosphereDensityNodes = []float64{
	5.6040e-7, 9.7080e-8, 2.2220e-8, 8.1520e-9, 2.0760e-9, 5.1940e-10, 2.7890e-10,
	7.2480e-11, 2.4180e-11, 9.5180e-12, 3.7250e-12, 1.5850e-12, 6.9670e-13,
	1.4540e-13, 3.6140e-14, 1.1700e-14, 5.2450e-15, 3.0190e-15,
}

const (
	// exosphericTemperature — предельная кинетическая температура экзосферы
	// при средней солнечной активности.
	exosphericTemperature = 1000.0 // К

	// thermosphereScale — характерная высота выхода температуры на асимптоту.
	thermosphereScale = 40000.0 // м

	// baseThermosphereTemp — температура на нижней границе термосферы (100 км).
	baseThermosphereTemp = 195.08 // К
)

// AtmosphereState — состояние атмосферы на заданной высоте.
type AtmosphereState struct {
	Density     float64 // кг/м³
	Temperature float64 // К (кинетическая температура газа)
	Pressure    float64 // Па
	SoundSpeed  float64 // м/с
}

// Atmosphere возвращает параметры атмосферы на заданной геометрической высоте.
func Atmosphere(altitude float64) AtmosphereState {
	t := AtmosphericTemperature(altitude)
	rho := AtmosphericDensity(altitude)
	return AtmosphereState{
		Density:     rho,
		Temperature: t,
		Pressure:    rho * RSpecificAir * t,
		SoundSpeed:  SpeedOfSound(t),
	}
}

// AtmosphericTemperature возвращает кинетическую температуру газа (К).
//
// ВНИМАНИЕ: выше ~100 км эта величина НЕ равна температуре, которую примет
// конструкция. Газ там разрежён настолько, что теплообмен с ним пренебрежимо мал,
// и равновесная температура тела определяется радиационным балансом
// (см. RadiativeEquilibriumTemperature).
func AtmosphericTemperature(altitude float64) float64 {
	if altitude <= 0 {
		return temperatureNodes[0]
	}
	if altitude <= 100000 {
		return interpolateLinear(altitude, altitudeNodes, temperatureNodes)
	}
	// Термосфера: температура асимптотически растёт к экзосферной.
	d := altitude - 100000
	return exosphericTemperature -
		(exosphericTemperature-baseThermosphereTemp)*math.Exp(-d/thermosphereScale)
}

// AtmosphericDensity возвращает плотность воздуха (кг/м³).
// Интерполяция ведётся по логарифму плотности — она экспоненциальна по высоте.
func AtmosphericDensity(altitude float64) float64 {
	if altitude <= 0 {
		return densityNodes[0]
	}
	if altitude <= 100000 {
		return interpolateLog(altitude, altitudeNodes, densityNodes)
	}
	if altitude >= 1000000 {
		// Выше 1000 км плотность продолжаем экспоненциально с шкалой высот
		// последнего участка таблицы. Практического влияния уже не оказывает.
		last := len(thermosphereDensityNodes) - 1
		h := thermosphereAltNodes[last] - thermosphereAltNodes[last-1]
		scale := h / math.Log(thermosphereDensityNodes[last-1]/thermosphereDensityNodes[last])
		return thermosphereDensityNodes[last] * math.Exp(-(altitude-1000000)/scale)
	}
	return interpolateLog(altitude, thermosphereAltNodes, thermosphereDensityNodes)
}

// DynamicPressure возвращает скоростной напор q = ½·ρ·v².
func DynamicPressure(density, speed float64) float64 {
	return 0.5 * density * speed * speed
}

// DragCoefficient возвращает коэффициент лобового сопротивления по числу Маха.
// Приближение для тела вращения с большим удлинением (ступень ракеты):
// пологий дозвуковой участок, трансзвуковой пик около M = 1.1
// и медленный спад на сверхзвуке.
func DragCoefficient(mach float64) float64 {
	if math.IsNaN(mach) || mach < 0 {
		return 0.3
	}
	switch {
	case mach < 0.8:
		return 0.20
	case mach < 1.0:
		t := (mach - 0.8) / 0.2
		return 0.20 + t*(0.50-0.20)
	case mach < 1.2:
		t := (mach - 1.0) / 0.2
		return 0.50 - t*(0.50-0.45)
	case mach < 3.0:
		t := (mach - 1.2) / 1.8
		return 0.45 - t*(0.45-0.35)
	case mach < 5.0:
		t := (mach - 3.0) / 2.0
		return 0.35 - t*(0.35-0.31)
	default:
		return 0.31
	}
}

// -----------------------------------------------------------------------------
// Интерполяция
// -----------------------------------------------------------------------------

// interpolateLinear — кусочно-линейная интерполяция с зажимом на краях.
func interpolateLinear(x float64, xs, ys []float64) float64 {
	n := len(xs)
	if x <= xs[0] {
		return ys[0]
	}
	if x >= xs[n-1] {
		return ys[n-1]
	}
	i := searchSegment(x, xs)
	t := (x - xs[i]) / (xs[i+1] - xs[i])
	return ys[i] + t*(ys[i+1]-ys[i])
}

// interpolateLog — интерполяция по логарифму значения (для плотности).
func interpolateLog(x float64, xs, ys []float64) float64 {
	n := len(xs)
	if x <= xs[0] {
		return ys[0]
	}
	if x >= xs[n-1] {
		return ys[n-1]
	}
	i := searchSegment(x, xs)
	t := (x - xs[i]) / (xs[i+1] - xs[i])
	return math.Exp(math.Log(ys[i]) + t*(math.Log(ys[i+1])-math.Log(ys[i])))
}

// searchSegment возвращает индекс левой границы отрезка, содержащего x.
func searchSegment(x float64, xs []float64) int {
	lo, hi := 0, len(xs)-1
	for hi-lo > 1 {
		mid := (lo + hi) / 2
		if x < xs[mid] {
			hi = mid
		} else {
			lo = mid
		}
	}
	return lo
}
