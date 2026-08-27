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
// Осевое сопротивление: нос вперёд против двигателей вперёд.
//
// DragCoefficient(mach) — кривая для тела вращения с БОЛЬШИМ УДЛИНЕНИЕМ,
// то есть для обтекаемой, носовой конфигурации (профиль подъёма — вперёд
// идёт остроконечная носовая часть). Она же до сих пор использовалась и для
// входа двигателями вперёд (ForceModel.dragArea брал |cos α| — модуль,
// стирающий ЗНАК угла атаки, а с ним и информацию о том, какой конец
// корпуса в потоке). Физически это два разных тела: обтекаемый конус —
// и тупой торец с юбкой двигателей, тридцатью тремя соплами и промежутками
// между ними. Один и тот же коэффициент для них подозрителен сам по себе
// и не может быть верным для обеих ориентаций одновременно.
// -----------------------------------------------------------------------------

// FlowOrientation — какой конец корпуса встречает поток: нос или двигатели.
type FlowOrientation int

const (
	// NoseFirst — вперёд идёт нос (штатный подъём).
	NoseFirst FlowOrientation = iota
	// EngineFirst — вперёд идут двигатели (вход, посадочный манёвр
	// первой ступени).
	EngineFirst
)

// OrientationFromAngleOfAttack определяет ориентацию по полному углу атаки,
// градусы (см. AngleOfAttack: 0° — нос точно по направлению движения
// относительно воздуха, то есть скорость корпуса относительно воздуха
// направлена туда же, куда и нос; 180° — вперёд идут двигатели). Функция
// AngleOfAttack по построению неотрицательна (arccos), поэтому единственная
// осмысленная граница — ровно девяносто градусов: меньше половины полного
// разворота — ближе к носу, больше — ближе к двигателям.
//
// Рядом с самой границей осевая проекция площади (|cos α| в
// ForceModel.dragArea) сама стремится к нулю — на этом угле корпус подставлен
// потоку почти исключительно бортом, — поэтому то, какая из двух кривых
// выбрана ровно на переходе, не даёт скачка СИЛЫ: коэффициент домножается на
// величину, уже близкую к нулю. Явная проверка — TestAxialDragContinuousThroughOrientationTransition.
func OrientationFromAngleOfAttack(angleOfAttackDeg float64) FlowOrientation {
	if angleOfAttackDeg > 90 {
		return EngineFirst
	}
	return NoseFirst
}

// engineFirstMachNodes/engineFirstCdNodes — коэффициент осевого сопротивления
// корпуса, летящего ДВИГАТЕЛЬНЫМ ОТСЕКОМ ВПЕРЁД, по числу Маха.
//
// Лётных продувочных данных по Super Heavy в открытом доступе нет. Кривая
// опирается на опубликованные измерения тупых тел БЛИЗКОЙ формы — как на
// ГРАНИЦУ/ориентир, а не точную модель: реальный двигательный отсек сложнее
// гладкого плоского торца (перетекание газа между и вокруг тридцати трёх
// сопел способно СНИЖАТЬ эффективное сопротивление против идеального диска
// той же площади — см. предостережение в конце). Источники по участкам:
//
//   - Дозвук (M<0.8): тупой цилиндр/плоский торец, по Hoerner
//     ("Fluid-Dynamic Drag", гл. 3, "Bodies of Revolution") —
//     Cd≈0.9–1.2 по площади миделя для плоского торца с острой кромкой
//     (для сравнения — обтекаемое тело той же площади миделя даёт ≈0.2,
//     см. DragCoefficient). Взято ≈1.0, середина диапазона.
//   - Трансзвук (M=0.8…1.2): у тупых тел скачок около M=1 выражен слабее,
//     чем у обтекаемых, в ОТНОСИТЕЛЬНЫХ величинах — донное давление уже
//     даёт основной вклад в сопротивление до появления скачков уплотнения,
//     расти особо некуда. Заложен умеренный, не резкий рост (≈1.0→1.25).
//   - Сверхзвук (M=2.49…4.63): NASA TR R-274 (Stallings, R. L., 1967,
//     "Experimentally Determined Local Flow Properties and Drag
//     Coefficients for a Family of Blunt Bodies at Mach Numbers from 2.49
//     to 4.63"), плоский торец цилиндра — Cd≈1.7–1.9 (по модифицированной
//     ньютоновской формуле самого отчёта, скорректированной на ~8% по
//     измерениям). Это ПРЯМОЕ измерение близкой формы, не экстраполяция —
//     кривая ниже проходит внутри этого диапазона на M=3 и M=4.
//   - Гиперзвук (M≥5): выполаживание — Mach number independence principle:
//     давление на тупом теле почти перестаёт зависеть от числа Маха выше
//     ~M5, коэффициент дальше практически не растёт.
//
// Между узлами M=1.2 и M=2.49 (нижняя граница измерений Stallings) прямых
// данных нет — участок линейно интерполирован между трансзвуковым и
// измеренным сверхзвуковым уровнем, без claim на точность там, где
// измерений нет.
var engineFirstMachNodes = []float64{0, 0.8, 1.0, 1.2, 2.0, 3.0, 4.0, 5.0}
var engineFirstCdNodes = []float64{1.00, 1.00, 1.15, 1.25, 1.55, 1.75, 1.85, 1.85}

// EngineFirstDragCoefficient возвращает Cd для входа двигателями вперёд —
// см. описание у engineFirstMachNodes/engineFirstCdNodes выше.
func EngineFirstDragCoefficient(mach float64) float64 {
	if math.IsNaN(mach) || mach < 0 {
		return engineFirstCdNodes[0]
	}
	return interpolateLinear(mach, engineFirstMachNodes, engineFirstCdNodes)
}

// AxialDragCoefficient возвращает коэффициент осевого сопротивления по числу
// Маха и ориентации потока (см. FlowOrientation) — единая точка входа вместо
// разбросанных по вызывающему коду проверок |cos α|.
func AxialDragCoefficient(mach float64, orientation FlowOrientation) float64 {
	if orientation == EngineFirst {
		return EngineFirstDragCoefficient(mach)
	}
	return DragCoefficient(mach)
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
