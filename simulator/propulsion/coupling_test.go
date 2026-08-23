package propulsion

import (
	"math"
	"math/rand"
	"sort"
	"testing"

	"rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Тесты причинных связей.
//
// Главное требование к модели — не независимые графики, а связанные величины.
// Каждый тест меняет ровно одну причину и проверяет, что вся цепочка следствий
// сдвинулась в нужную сторону.
// -----------------------------------------------------------------------------

// nominalInput возвращает штатные условия работы двигателя.
func nominalInput() EngineInput {
	fuel, ox := RP1(), LOX()
	return EngineInput{
		Throttle:          1,
		AmbientPressure:   101325,
		FuelInletPressure: 300000,
		OxInletPressure:   340000,
		FuelDensity:       fuel.Density(288),
		OxDensity:         ox.Density(85),
		FuelTemperature:   288,
		OxTemperature:     85,
		FuelVaporPressure: fuel.VaporPressure(288),
		OxVaporPressure:   ox.VaporPressure(85),
		FuelAvailable:     true,
		OxAvailable:       true,
		Gravity:           9.81,
		MixtureTrim:       1,
		CoolingValve:      1,
	}
}

// runToSteadyState прогоняет двигатель до выхода на установившийся режим.
func runToSteadyState(e *Engine, in EngineInput, seconds float64) {
	const dt = 0.02
	for t := 0.0; t < seconds; t += dt {
		e.Update(dt, in)
	}
}

func newNominalEngine() *Engine {
	e := NewEngine(Merlin1DConfig(), rand.New(rand.NewSource(1)), rand.New(rand.NewSource(2)), 288)
	e.Detailed = true
	e.Start()
	return e
}

// Двигатель должен воспроизводить паспортные характеристики.
func TestNominalOperatingPoint(t *testing.T) {
	e := newNominalEngine()
	runToSteadyState(e, nominalInput(), 12)

	checks := []struct {
		name        string
		got, lo, hi float64
		unit        string
	}{
		{"давление в камере", e.Chamber.MeanPressure / 1e6, 9.0, 10.5, "МПа"},
		{"соотношение компонентов", e.MixtureRatio(), 2.2, 2.5, ""},
		{"удельный импульс", e.SpecificImpulse, 270, 295, "с"},
		{"тяга", e.Thrust / 1000, 780, 920, "кН"},
		{"обороты вала", e.Turbopump.RPM(), 28000, 36000, "об/мин"},
		{"характеристическая скорость", e.Chamber.CharacteristicVelocity, 1700, 1800, "м/с"},
		{"коэффициент тяги", e.Nozzle.ThrustCoefficient, 1.5, 1.75, ""},
		{"температура стенки", e.Nozzle.HottestStation().Temperature, 600, 1000, "К"},
	}
	for _, c := range checks {
		if c.got < c.lo || c.got > c.hi {
			t.Errorf("%s = %.3f %s, ожидалось %.1f…%.1f",
				c.name, c.got, c.unit, c.lo, c.hi)
		}
	}

	if e.Chamber.State != CombustionStable {
		t.Errorf("на номинале горение должно быть устойчивым, получено %v", e.Chamber.State)
	}

	t.Logf("Номинал: Pc=%.2f МПа, O/F=%.3f, тяга=%.0f кН, Isp=%.1f с, "+
		"обороты=%.0f, стенка=%.0f К",
		e.Chamber.MeanPressure/1e6, e.MixtureRatio(), e.Thrust/1000,
		e.SpecificImpulse, e.Turbopump.RPM(),
		e.Nozzle.HottestStation().Temperature)
}

// Баланс мощности турбонасоса: турбина немного мощнее насосов и потерь.
func TestTurbopumpPowerBalance(t *testing.T) {
	e := newNominalEngine()
	runToSteadyState(e, nominalInput(), 12)

	tp := e.Turbopump
	consumed := tp.PumpPower() + tp.MechanicalLoss

	// В установившемся режиме баланс сходится: избыток момента разгонял бы вал.
	// Проверяется именно это равенство, а не запас — запас есть только на
	// переходном участке раскрутки.
	imbalance := (tp.TurbinePower - consumed) / tp.TurbinePower
	if math.Abs(imbalance) > 0.02 {
		t.Errorf("баланс мощности не сошёлся: турбина %.3f МВт, потребление %.3f МВт "+
			"(расхождение %.1f %%)",
			tp.TurbinePower/1e6, consumed/1e6, imbalance*100)
	}

	// Насосы должны потреблять основную часть мощности, а не механические потери.
	if share := tp.PumpPower() / consumed; share < 0.7 {
		t.Errorf("на насосы приходится лишь %.0f %% потребления", share*100)
	}

	// А на разгоне момент турбины обязан превышать сопротивление.
	spinup := newNominalEngine()
	runToSteadyState(spinup, nominalInput(), 0.4)
	if spinup.Turbopump.Torque <= 0 {
		t.Errorf("на раскрутке момент на валу %.1f Н·м, ожидался положительный",
			spinup.Turbopump.Torque)
	}

	margin := imbalance
	t.Logf("Турбина %.2f МВт, насосы %.2f МВт, потери %.2f МВт, запас %.1f %%",
		tp.TurbinePower/1e6, tp.PumpPower()/1e6, tp.MechanicalLoss/1e6, margin*100)
}

// Раскрутка турбонасоса — переходный процесс, а не мгновенный скачок.
func TestStartupIsTransient(t *testing.T) {
	e := newNominalEngine()
	in := nominalInput()

	const dt = 0.02
	var timeTo90 float64
	nominal := e.Config.Chamber.NominalPressure

	for i := 0; i < 1000; i++ {
		e.Update(dt, in)
		now := float64(i+1) * dt

		if i == 0 && e.Chamber.MeanPressure > 0.5*nominal {
			t.Fatalf("давление в камере достигло %.1f %% номинала за один шаг — "+
				"переходного процесса нет", e.Chamber.MeanPressure/nominal*100)
		}
		if timeTo90 == 0 && e.Chamber.MeanPressure > 0.9*nominal {
			timeTo90 = now
		}
	}

	if timeTo90 == 0 {
		t.Fatal("двигатель не вышел на режим")
	}
	if timeTo90 < 0.3 || timeTo90 > 6 {
		t.Errorf("выход на 90 %% давления занял %.2f с, ожидалось 0.3…6 с", timeTo90)
	}
	t.Logf("Выход на режим: 90 %% давления камеры за %.2f с", timeTo90)
}

// Падение давления в баке снижает тягу через всю цепочку.
func TestTankPressureDropReducesThrust(t *testing.T) {
	base := newNominalEngine()
	runToSteadyState(base, nominalInput(), 12)

	starved := newNominalEngine()
	low := nominalInput()
	// Давление наддува просело: наддув исчерпан.
	low.FuelInletPressure = 120000
	low.OxInletPressure = 130000
	runToSteadyState(starved, low, 12)

	if starved.Turbopump.OxPump.CavitationMargin >= base.Turbopump.OxPump.CavitationMargin {
		t.Error("кавитационный запас не уменьшился при падении давления в баке")
	}
	if starved.Chamber.MeanPressure >= base.Chamber.MeanPressure {
		t.Errorf("давление в камере не снизилось: %.2f против %.2f МПа",
			starved.Chamber.MeanPressure/1e6, base.Chamber.MeanPressure/1e6)
	}
	if starved.Thrust >= base.Thrust {
		t.Errorf("тяга не снизилась: %.0f против %.0f кН",
			starved.Thrust/1000, base.Thrust/1000)
	}

	t.Logf("Падение наддува: запас %.1f → %.1f м, Pc %.2f → %.2f МПа, тяга %.0f → %.0f кН",
		base.Turbopump.OxPump.CavitationMargin, starved.Turbopump.OxPump.CavitationMargin,
		base.Chamber.MeanPressure/1e6, starved.Chamber.MeanPressure/1e6,
		base.Thrust/1000, starved.Thrust/1000)
}

// Недостаточная мощность турбины роняет обороты, напор, расход и давление.
func TestTurbinePowerShortfallCascades(t *testing.T) {
	base := newNominalEngine()
	runToSteadyState(base, nominalInput(), 12)

	weakCfg := Merlin1DConfig()
	// Клапан газогенератора приоткрыт лишь наполовину.
	weakCfg.Turbopump.GasGeneratorFlow *= 0.5
	weak := NewEngine(weakCfg, rand.New(rand.NewSource(1)), rand.New(rand.NewSource(2)), 288)
	weak.Start()
	runToSteadyState(weak, nominalInput(), 12)

	chain := []struct {
		name      string
		base, got float64
	}{
		{"обороты вала", base.Turbopump.RPM(), weak.Turbopump.RPM()},
		{"давление за насосом горючего",
			base.Turbopump.FuelPump.OutletPressure, weak.Turbopump.FuelPump.OutletPressure},
		{"расход горючего", base.FuelFlow, weak.FuelFlow},
		{"расход окислителя", base.OxFlow, weak.OxFlow},
		{"давление в камере", base.Chamber.MeanPressure, weak.Chamber.MeanPressure},
		{"тяга", base.Thrust, weak.Thrust},
	}
	for _, c := range chain {
		if c.got >= c.base {
			t.Errorf("%s не снизилось: %.4g против %.4g", c.name, c.got, c.base)
		}
	}
	t.Logf("Половина расхода газогенератора: обороты %.0f → %.0f, Pc %.2f → %.2f МПа, "+
		"тяга %.0f → %.0f кН",
		base.Turbopump.RPM(), weak.Turbopump.RPM(),
		base.Chamber.MeanPressure/1e6, weak.Chamber.MeanPressure/1e6,
		base.Thrust/1000, weak.Thrust/1000)
}

// Прикрытие клапана горючего повышает O/F и меняет характеристики горения.
func TestFuelValveClosingRaisesMixtureRatio(t *testing.T) {
	base := newNominalEngine()
	runToSteadyState(base, nominalInput(), 12)

	trimmed := newNominalEngine()
	lean := nominalInput()
	lean.MixtureTrim = 0.85 // клапан горючего прикрыт
	runToSteadyState(trimmed, lean, 12)

	if trimmed.MixtureRatio() <= base.MixtureRatio() {
		t.Errorf("соотношение компонентов не выросло: %.3f против %.3f",
			trimmed.MixtureRatio(), base.MixtureRatio())
	}
	if trimmed.Chamber.CharacteristicVelocity >= base.Chamber.CharacteristicVelocity {
		t.Error("характеристическая скорость должна упасть при отклонении от оптимума")
	}
	if trimmed.SpecificImpulse >= base.SpecificImpulse {
		t.Errorf("удельный импульс не снизился: %.1f против %.1f",
			trimmed.SpecificImpulse, base.SpecificImpulse)
	}
	if trimmed.Chamber.Temperature <= base.Chamber.Temperature {
		t.Error("обеднение смеси по горючему должно поднять температуру в камере")
	}

	t.Logf("Клапан горючего 85 %%: O/F %.3f → %.3f, c* %.0f → %.0f м/с, "+
		"Isp %.1f → %.1f с, T камеры %.0f → %.0f К",
		base.MixtureRatio(), trimmed.MixtureRatio(),
		base.Chamber.CharacteristicVelocity, trimmed.Chamber.CharacteristicVelocity,
		base.SpecificImpulse, trimmed.SpecificImpulse,
		base.Chamber.Temperature, trimmed.Chamber.Temperature)
}

// Снижение расхода охлаждения греет стенку и ускоряет эрозию горловины.
func TestReducedCoolingAcceleratesErosion(t *testing.T) {
	base := newNominalEngine()
	starved := newNominalEngine()

	poor := nominalInput()
	poor.CoolingValve = 0.35

	// Смотреть только конечное состояние здесь нельзя: недоохлаждённая
	// стенка прогорает, двигатель гаснет и остывает, и к концу прогона
	// оказывается холоднее исправного. Поэтому берутся пиковые значения
	// за прогон — то, что стенка успела пережить до отказа.
	const dt = 0.05
	peakBase, peakPoor := 0.0, 0.0
	for t := 0.0; t < 160; t += dt {
		base.Update(dt, nominalInput())
		starved.Update(dt, poor)

		peakBase = math.Max(peakBase, base.Nozzle.HottestStation().Temperature)
		peakPoor = math.Max(peakPoor, starved.Nozzle.HottestStation().Temperature)
	}

	if peakPoor <= peakBase {
		t.Errorf("стенка не нагрелась сильнее: %.0f против %.0f К", peakPoor, peakBase)
	}
	if starved.Nozzle.ThroatErosion <= base.Nozzle.ThroatErosion {
		t.Errorf("эрозия не ускорилась: %.4g против %.4g мм",
			starved.Nozzle.ThroatErosionMillimeters(),
			base.Nozzle.ThroatErosionMillimeters())
	}
	if base.Nozzle.WallDamage > 0 {
		t.Errorf("исправное охлаждение повредило стенку: %.4f", base.Nozzle.WallDamage)
	}
	if !starved.Nozzle.BurnedThrough {
		t.Errorf("стенка не прогорела при охлаждении 35 %%: повреждение %.3f",
			starved.Nozzle.WallDamage)
	}
	if starved.Running {
		t.Error("прогоревший двигатель продолжает работать")
	}

	t.Logf("Охлаждение 35 %%: пиковая стенка %.0f → %.0f К, эрозия %.4f → %.4f мм, "+
		"повреждение %.2f, прогар %v",
		peakBase, peakPoor,
		base.Nozzle.ThroatErosionMillimeters(), starved.Nozzle.ThroatErosionMillimeters(),
		starved.Nozzle.WallDamage, starved.Nozzle.BurnedThrough)
}

// Рост площади горловины меняет давление в камере и тягу даже при неизменных
// клапанах.
func TestThroatAreaAffectsChamberPressure(t *testing.T) {
	e := newNominalEngine()
	in := nominalInput()
	runToSteadyState(e, in, 12)

	before := e.Chamber.MeanPressure
	beforeThrust := e.Thrust

	// Искусственно увеличиваем горловину на пять процентов.
	e.Nozzle.ThroatArea *= 1.05
	runToSteadyState(e, in, 4)

	if e.Chamber.MeanPressure >= before {
		t.Errorf("давление в камере не снизилось при росте горловины: %.3f против %.3f МПа",
			e.Chamber.MeanPressure/1e6, before/1e6)
	}
	t.Logf("Горловина +5 %%: Pc %.3f → %.3f МПа, тяга %.0f → %.0f кН",
		before/1e6, e.Chamber.MeanPressure/1e6, beforeThrust/1000, e.Thrust/1000)
}

// Кавитация повышает вибрацию и раскачивает камеру.
func TestCavitationRaisesVibrationAndInstability(t *testing.T) {
	base := newNominalEngine()
	runToSteadyState(base, nominalInput(), 12)

	cav := newNominalEngine()
	starved := nominalInput()
	// Давление на входе у самой линии насыщения окислителя.
	starved.OxInletPressure = 110000
	starved.OxTemperature = 90
	starved.OxVaporPressure = LOX().VaporPressure(90)
	runToSteadyState(cav, starved, 12)

	if cav.Turbopump.OxPump.CavitationSeverity <= 0 {
		t.Fatalf("кавитация не наступила: запас %.2f м",
			cav.Turbopump.OxPump.CavitationMargin)
	}
	if cav.Turbopump.Vibration <= base.Turbopump.Vibration {
		t.Errorf("вибрация не выросла: %.2f против %.2f g",
			cav.Turbopump.Vibration, base.Turbopump.Vibration)
	}
	if cav.Chamber.StabilityMargin >= base.Chamber.StabilityMargin {
		t.Errorf("запас устойчивости не снизился: %.3f против %.3f",
			cav.Chamber.StabilityMargin, base.Chamber.StabilityMargin)
	}

	t.Logf("Кавитация: глубина %.2f, вибрация %.1f → %.1f g, запас устойчивости %.3f → %.3f",
		cav.Turbopump.OxPump.CavitationSeverity,
		base.Turbopump.Vibration, cav.Turbopump.Vibration,
		base.Chamber.StabilityMargin, cav.Chamber.StabilityMargin)
}

// При падении запаса устойчивости пульсации нарастают и в спектре появляется
// выраженный пик.
func TestUnstableCombustionShowsSpectralPeak(t *testing.T) {
	e := newNominalEngine()
	runToSteadyState(e, nominalInput(), 8)

	stableRMS := e.Chamber.PressureRMS
	e.Chamber.UpdateSpectrum()

	// Принудительно лишаем систему подачи развязки: запас устойчивости падает.
	e.Chamber.StabilityMargin = 0.02
	for i := 0; i < 4000; i++ {
		e.Chamber.UpdateOscillations(0.002, control.ChamberOverrides{})
	}
	e.Chamber.UpdateStatistics()
	e.Chamber.UpdateSpectrum()

	if e.Chamber.PressureRMS <= stableRMS {
		t.Errorf("пульсации не выросли: %.4g против %.4g Па",
			e.Chamber.PressureRMS, stableRMS)
	}

	// Спектр должен иметь выраженный максимум, а не быть равномерным.
	spec := e.Chamber.Spectrum()
	var peak, sum float64
	for i := 1; i < len(spec); i++ {
		peak = math.Max(peak, spec[i].Amplitude)
		sum += spec[i].Amplitude
	}
	mean := sum / float64(len(spec)-1)
	if peak < 4*mean {
		t.Errorf("спектр равномерный: пик %.4g при среднем %.4g", peak, mean)
	}

	freq := e.Chamber.DominantFrequency
	if freq < 100 || freq > 250 {
		t.Errorf("доминирующая частота %.1f Гц, ожидалась акустическая мода камеры", freq)
	}

	t.Logf("Неустойчивость: RMS %.3g → %.3g Па, пик/среднее = %.1f, частота %.1f Гц",
		stableRMS, e.Chamber.PressureRMS, peak/mean, freq)
}

// Дросселирование снижает давление, тягу и расход согласованно.
func TestThrottlingIsConsistent(t *testing.T) {
	full := newNominalEngine()
	runToSteadyState(full, nominalInput(), 12)

	part := newNominalEngine()
	low := nominalInput()
	low.Throttle = 0.6
	runToSteadyState(part, low, 12)

	ratios := map[string]float64{
		"давление в камере": part.Chamber.MeanPressure / full.Chamber.MeanPressure,
		"суммарный расход":  part.TotalFlow() / full.TotalFlow(),
		"тяга":              part.Thrust / full.Thrust,
	}
	for name, r := range ratios {
		if r >= 1 || r <= 0.2 {
			t.Errorf("%s изменилось в %.3f раза, ожидалось снижение", name, r)
		}
	}

	// Давление, расход и тяга обязаны снизиться согласованно: тяга
	// пропорциональна давлению при том же коэффициенте тяги.
	if math.Abs(ratios["давление в камере"]-ratios["тяга"]) > 0.12 {
		t.Errorf("тяга и давление изменились несогласованно: %.3f против %.3f",
			ratios["тяга"], ratios["давление в камере"])
	}

	t.Logf("Дросселирование до 60 %%: Pc ×%.3f, расход ×%.3f, тяга ×%.3f",
		ratios["давление в камере"], ratios["суммарный расход"], ratios["тяга"])
}

// Тяга растёт с высотой из-за падения противодавления на срезе сопла.
func TestThrustRisesWithAltitude(t *testing.T) {
	sea := newNominalEngine()
	runToSteadyState(sea, nominalInput(), 12)

	vac := newNominalEngine()
	vacuum := nominalInput()
	vacuum.AmbientPressure = 0
	runToSteadyState(vac, vacuum, 12)

	if vac.Thrust <= sea.Thrust {
		t.Errorf("тяга в вакууме %.0f кН не выше, чем у земли %.0f кН",
			vac.Thrust/1000, sea.Thrust/1000)
	}
	gain := vac.Thrust/sea.Thrust - 1
	if gain < 0.05 || gain > 0.2 {
		t.Errorf("прирост тяги в вакууме %.1f %%, ожидалось 5…20 %%", gain*100)
	}
	if vac.SpecificImpulse <= sea.SpecificImpulse {
		t.Error("удельный импульс в вакууме должен быть выше")
	}
	t.Logf("Высота: тяга %.0f → %.0f кН (+%.1f %%), Isp %.1f → %.1f с",
		sea.Thrust/1000, vac.Thrust/1000, gain*100,
		sea.SpecificImpulse, vac.SpecificImpulse)
}

// Шум датчиков не должен разрушать физические связи.
func TestSensorNoisePreservesRelations(t *testing.T) {
	e := newNominalEngine()
	in := nominalInput()
	runToSteadyState(e, in, 12)

	// Истинные величины связаны точно.
	trueRatio := e.Chamber.MeanPressure * e.Nozzle.ThroatArea *
		e.Nozzle.ThrustCoefficient / (e.Thrust - e.GasGeneratorThrust)
	if math.Abs(trueRatio-1) > 1e-9 {
		t.Errorf("тяга не равна Cf·Pc·A: отношение %.9f", trueRatio)
	}

	// Показания датчиков отличаются от истины, но остаются в разумных пределах.
	// Выбросы и потери связи — штатное поведение модели датчика, поэтому
	// проверяется не худшее значение, а типичное: девяносто пятый процентиль.
	// Иначе один выброс, ради которого модель и написана, ронял бы тест.
	var deviations []float64
	var dropouts, outliers int

	for i := 0; i < 2000; i++ {
		e.Update(0.02, in)
		tel := e.Telemetry(0.02, false)
		if !tel.ChamberPressure.Defined() {
			dropouts++
			continue
		}
		d := math.Abs(tel.ChamberPressure.Value()-e.Chamber.Pressure) / e.Chamber.MeanPressure
		if d > 0.2 {
			outliers++
		}
		deviations = append(deviations, d)
	}

	sort.Float64s(deviations)
	typical := deviations[len(deviations)*95/100]
	if typical > 0.05 {
		t.Errorf("типичное отклонение датчика %.1f %%, ожидался шум в пределах 5 %%",
			typical*100)
	}
	// Выбросы должны быть редкими, иначе датчик бесполезен.
	if rate := float64(outliers) / float64(len(deviations)); rate > 0.02 {
		t.Errorf("выбросы составляют %.1f %% показаний", rate*100)
	}

	t.Logf("Датчик давления: отклонение по 95-му процентилю %.2f %%, "+
		"выбросов %d, потерь связи %d из %d",
		typical*100, outliers, dropouts, len(deviations)+dropouts)
}

// tankEngineInput собирает вход двигателя из фактического состояния баков —
// в отличие от nominalInput(), давление на входе не задаётся числом,
// а вычисляется через Tank.OutletPressure(), то есть зависит от осадки
// топлива.
func tankEngineInput(fuel, ox *Tank, axialAccel float64) EngineInput {
	return EngineInput{
		Throttle:          1,
		AmbientPressure:   101325,
		FuelInletPressure: fuel.OutletPressure(axialAccel),
		OxInletPressure:   ox.OutletPressure(axialAccel),
		FuelDensity:       fuel.Density(),
		OxDensity:         ox.Density(),
		FuelTemperature:   fuel.Temperature,
		OxTemperature:     ox.Temperature,
		FuelVaporPressure: fuel.Propellant.VaporPressure(fuel.Temperature),
		OxVaporPressure:   ox.Propellant.VaporPressure(ox.Temperature),
		FuelAvailable:     true,
		OxAvailable:       true,
		FuelGasFraction:   1 - fuel.Settled,
		OxGasFraction:     1 - ox.Settled,
		Gravity:           9.81,
		MixtureTrim:       1,
		CoolingValve:      1,
	}
}

// oxTestTank возвращает наполовину заправленный бак окислителя для тестов.
func oxTestTank() *Tank {
	return NewTank(TankConfig{
		Name:                  "ox",
		Volume:                10,
		Height:                4,
		CrossSection:          2.5,
		InitialMass:           9000,
		InitialTemperature:    85,
		TargetPressure:        3.4e5,
		LineResistance:        5,
		PressurantMass:        50,
		PressurantPressure:    2.1e7,
		PressurantTemperature: 200,
	}, LOX())
}

// В свободном падении топливо отходит от заборника (Tank.updateSettling),
// заборник вместо жидкости захватывает газ наддува, и насос теряет напор
// через ту же кавитационную модель, что и при паровой каверне (см.
// Pump.Update, EngineInput.FuelGasFraction/OxGasFraction). Без единой новой
// проверки в Start() это валит давление в камере — то самое «зажигание
// с перебоями», о котором и был вопрос: полного гашения по ignitionGrace
// не гарантируется (зависит от MinChamberPressureFraction конкретного
// двигателя), но устойчивого номинального режима достичь нельзя.
func TestUnsettledPropellantPreventsIgnition(t *testing.T) {
	const dt = 0.02

	fuel, ox := testTank(), oxTestTank()
	settled := newNominalEngine()
	for i := 0; i < 400; i++ { // 8 с под перегрузкой, топливо всё время осевшее
		env := TankEnvironment{AxialAcceleration: 20}
		fuel.updateSettling(dt, env)
		ox.updateSettling(dt, env)
		settled.Update(dt, tankEngineInput(fuel, ox, 20))
	}
	nominal := settled.Config.Chamber.NominalPressure
	if !settled.Running || settled.Chamber.MeanPressure < 0.9*nominal {
		t.Fatalf("контрольный случай (топливо осело) не вышел на режим: "+
			"Running=%v, Pc=%.2f МПа", settled.Running, settled.Chamber.MeanPressure/1e6)
	}

	fuel, ox = testTank(), oxTestTank()
	unsettled := newNominalEngine()
	for i := 0; i < 400; i++ { // 8 с в свободном падении
		env := TankEnvironment{AxialAcceleration: 0}
		fuel.updateSettling(dt, env)
		ox.updateSettling(dt, env)
		unsettled.Update(dt, tankEngineInput(fuel, ox, 0))
	}

	if cav := unsettled.Turbopump.OxPump.CavitationSeverity; cav < 0.5 {
		t.Errorf("кавитация от газа наддува слишком слабая: глубина %.2f", cav)
	}
	if unsettled.Chamber.MeanPressure >= 0.5*settled.Chamber.MeanPressure {
		t.Errorf("давление в камере не просело без осадки топлива: "+
			"%.2f МПа против осевшего случая %.2f МПа",
			unsettled.Chamber.MeanPressure/1e6, settled.Chamber.MeanPressure/1e6)
	}

	t.Logf("Осевшее топливо: Pc=%.2f МПа, работает=%v. Неосевшее: Pc=%.2f МПа, "+
		"работает=%v, кавитация=%.2f (осадка горючего=%.2f, окислителя=%.2f)",
		settled.Chamber.MeanPressure/1e6, settled.Running,
		unsettled.Chamber.MeanPressure/1e6, unsettled.Running,
		unsettled.Turbopump.OxPump.CavitationSeverity, fuel.Settled, ox.Settled)
}

// Выработка компонентов согласована: соотношение расходов из баков совпадает
// с проектным соотношением заправки.
func TestOverallMixtureRatioMatchesTankFill(t *testing.T) {
	cfg := Merlin1DConfig()
	e := NewEngine(cfg, rand.New(rand.NewSource(1)), rand.New(rand.NewSource(2)), 288)
	e.Start()
	runToSteadyState(e, nominalInput(), 12)

	actual := e.OxFlow / e.FuelFlow
	expected := cfg.OverallMixtureRatio()

	if math.Abs(actual-expected)/expected > 0.06 {
		t.Errorf("фактическое соотношение отбора %.3f расходится с расчётным %.3f",
			actual, expected)
	}
	// Соотношение отбора обязано быть ниже камерного: газогенератор потребляет
	// почти чистое горючее.
	if expected >= e.MixtureRatio() {
		t.Errorf("соотношение отбора %.3f не ниже камерного %.3f",
			expected, e.MixtureRatio())
	}
	t.Logf("Соотношение: в камере %.3f, отбор из баков %.3f (расчёт %.3f)",
		e.MixtureRatio(), actual, expected)
}
