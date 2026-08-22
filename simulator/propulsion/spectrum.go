package propulsion

import "math"

// -----------------------------------------------------------------------------
// Спектральный анализ пульсаций давления.
//
// Prometheus не умеет считать преобразование Фурье, поэтому спектр вычисляется
// здесь и публикуется готовыми частотными полосами. Это позволяет вывести
// спектральную панель в Grafana обычным графиком по метке частоты.
//
// Размер окна и частота дискретизации подобраны под низкочастотные пульсации
// камеры: интерес представляет диапазон от единиц до сотен герц.
// -----------------------------------------------------------------------------

// spectrumSize — размер окна БПФ. Должен быть степенью двойки.
const spectrumSize = 256

// SpectrumBin — одна частотная полоса спектра.
type SpectrumBin struct {
	Frequency float64 // Гц
	Amplitude float64 // в единицах исходного сигнала
}

// SpectrumAnalyzer — кольцевой буфер отсчётов с преобразованием Фурье.
type SpectrumAnalyzer struct {
	samples    []float64
	index      int
	filled     bool
	sampleRate float64

	spectrum []SpectrumBin
	dirty    bool
}

// NewSpectrumAnalyzer создаёт анализатор.
func NewSpectrumAnalyzer(size int, sampleRate float64) *SpectrumAnalyzer {
	if size <= 0 || size&(size-1) != 0 {
		size = spectrumSize
	}
	return &SpectrumAnalyzer{
		samples:    make([]float64, size),
		sampleRate: sampleRate,
		spectrum:   make([]SpectrumBin, size/2),
	}
}

// Push добавляет отсчёт в буфер.
func (a *SpectrumAnalyzer) Push(v float64) {
	if math.IsNaN(v) || math.IsInf(v, 0) {
		v = 0
	}
	a.samples[a.index] = v
	a.index++
	if a.index >= len(a.samples) {
		a.index = 0
		a.filled = true
	}
	a.dirty = true
}

// Ready сообщает, что буфер заполнен хотя бы один раз.
func (a *SpectrumAnalyzer) Ready() bool { return a.filled }

// RMS возвращает среднеквадратичное значение сигнала в окне.
func (a *SpectrumAnalyzer) RMS() float64 {
	n := a.count()
	if n == 0 {
		return 0
	}
	sum := 0.0
	for i := 0; i < n; i++ {
		v := a.samples[i]
		sum += v * v
	}
	return math.Sqrt(sum / float64(n))
}

// Peak возвращает максимальное по модулю значение в окне.
func (a *SpectrumAnalyzer) Peak() float64 {
	n := a.count()
	peak := 0.0
	for i := 0; i < n; i++ {
		if v := math.Abs(a.samples[i]); v > peak {
			peak = v
		}
	}
	return peak
}

func (a *SpectrumAnalyzer) count() int {
	if a.filled {
		return len(a.samples)
	}
	return a.index
}

// Spectrum возвращает амплитудный спектр сигнала.
//
// Перед преобразованием применяется окно Ханна: без него разрывы на границах
// окна размазывают энергию по всему спектру, и выраженный пик неустойчивого
// горения теряется в утечке.
func (a *SpectrumAnalyzer) Spectrum() []SpectrumBin {
	if !a.dirty {
		return a.spectrum
	}
	a.dirty = false

	n := len(a.samples)
	if !a.filled {
		for i := range a.spectrum {
			a.spectrum[i] = SpectrumBin{Frequency: a.binFrequency(i)}
		}
		return a.spectrum
	}

	re := make([]float64, n)
	im := make([]float64, n)

	// Отсчёты укладываются в хронологическом порядке начиная с самого старого.
	for i := 0; i < n; i++ {
		v := a.samples[(a.index+i)%n]
		window := 0.5 * (1 - math.Cos(2*math.Pi*float64(i)/float64(n-1)))
		re[i] = v * window
	}

	fft(re, im)

	// Нормировка: коэффициент 2/N даёт амплитуду гармоники, поправка 2.0
	// компенсирует потерю энергии в окне Ханна.
	const windowGain = 2.0
	scale := windowGain * 2 / float64(n)

	for i := range a.spectrum {
		a.spectrum[i] = SpectrumBin{
			Frequency: a.binFrequency(i),
			Amplitude: math.Hypot(re[i], im[i]) * scale,
		}
	}
	return a.spectrum
}

func (a *SpectrumAnalyzer) binFrequency(i int) float64 {
	return float64(i) * a.sampleRate / float64(len(a.samples))
}

// DominantFrequency возвращает частоту максимума спектра.
//
// Нулевая полоса пропускается: постоянная составляющая не является пульсацией.
func (a *SpectrumAnalyzer) DominantFrequency() (float64, bool) {
	spec := a.Spectrum()
	if len(spec) < 2 {
		return 0, false
	}
	best, bestIdx := 0.0, -1
	for i := 1; i < len(spec); i++ {
		if spec[i].Amplitude > best {
			best, bestIdx = spec[i].Amplitude, i
		}
	}
	if bestIdx < 0 || best <= 0 {
		return 0, false
	}
	return spec[bestIdx].Frequency, true
}

// Reset очищает буфер.
func (a *SpectrumAnalyzer) Reset() {
	for i := range a.samples {
		a.samples[i] = 0
	}
	a.index = 0
	a.filled = false
	a.dirty = true
}

// -----------------------------------------------------------------------------
// Быстрое преобразование Фурье
// -----------------------------------------------------------------------------

// fft выполняет преобразование на месте по алгоритму Кули — Тьюки
// с прореживанием по времени. Длина обязана быть степенью двойки.
func fft(re, im []float64) {
	n := len(re)
	if n <= 1 {
		return
	}

	// Перестановка отсчётов в порядке обращения битов.
	for i, j := 1, 0; i < n; i++ {
		bit := n >> 1
		for ; j&bit != 0; bit >>= 1 {
			j ^= bit
		}
		j |= bit
		if i < j {
			re[i], re[j] = re[j], re[i]
			im[i], im[j] = im[j], im[i]
		}
	}

	// Бабочки по возрастающим длинам блоков.
	for length := 2; length <= n; length <<= 1 {
		angle := -2 * math.Pi / float64(length)
		wRe, wIm := math.Cos(angle), math.Sin(angle)

		for i := 0; i < n; i += length {
			curRe, curIm := 1.0, 0.0
			half := length / 2
			for j := 0; j < half; j++ {
				uRe, uIm := re[i+j], im[i+j]
				vRe := re[i+j+half]*curRe - im[i+j+half]*curIm
				vIm := re[i+j+half]*curIm + im[i+j+half]*curRe

				re[i+j], im[i+j] = uRe+vRe, uIm+vIm
				re[i+j+half], im[i+j+half] = uRe-vRe, uIm-vIm

				curRe, curIm = curRe*wRe-curIm*wIm, curRe*wIm+curIm*wRe
			}
		}
	}
}
