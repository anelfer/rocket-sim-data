package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/env"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Полёт целиком: программа тангажа, ориентация, выход на орбиту, трасса.
//
// Здесь же общие помощники прогонов — runAscent и runUntilPhase, — которыми
// пользуются остальные наборы тестов пакета.
// -----------------------------------------------------------------------------

// testSeed — фиксированное зерно для всех тестов, которым нужен
// воспроизводимый полёт.
const testSeed int64 = 20260728

func approx(t *testing.T, got, want, tol float64, name string) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s: получено %.6g, ожидалось %.6g (допуск %.3g)", name, got, want, tol)
	}
}

// -----------------------------------------------------------------------------
// D2: интегратор
// -----------------------------------------------------------------------------

// На пассивном участке удельная энергия должна сохраняться.
// Явный Эйлер с шагом 1 с давал систематический дрейф энергии,
// из-за которого орбита «раскручивалась» сама по себе.

func TestMaxQThrottleIsContinuousAndReversible(t *testing.T) {
	// Закон должен быть непрерывным и монотонно убывающим по напору.
	prev := MaxQThrottle(0)
	for q := 0.0; q <= 60000; q += 100 {
		v := MaxQThrottle(q)
		if v > prev+1e-9 {
			t.Fatalf("газ вырос при увеличении напора на %.0f Па", q)
		}
		if math.Abs(v-prev) > 0.02 {
			t.Fatalf("скачок газа на %.0f Па: %.4f → %.4f", q, prev, v)
		}
		prev = v
	}

	// Газ возвращается сам, когда напор спадает: защёлок в законе нет.
	approx(t, MaxQThrottle(50000), 0.65, 1e-9, "газ на пике напора")
	approx(t, MaxQThrottle(10000), 1.0, 1e-9, "газ после прохождения Max-Q")
}

// -----------------------------------------------------------------------------
// D4: полный сброс состояния
// -----------------------------------------------------------------------------

func TestResetRestoresInitialState(t *testing.T) {
	sim := NewSimulation()
	before := sim.Snapshot()

	// Прогоняем полторы минуты полёта.
	sim.mu.Lock()
	for i := 0; i < 900; i++ {
		sim.step(0.1)
	}
	sim.telemetry = sim.buildTelemetryLocked()
	sim.mu.Unlock()

	flown := sim.Snapshot()
	if flown.Altitude <= before.Altitude {
		t.Fatal("ракета должна была набрать высоту")
	}

	sim.Reset()
	after := sim.Snapshot()

	approx(t, after.Time, 0, 1e-9, "время после сброса")
	approx(t, after.Altitude, 0, 1.0, "высота после сброса")

	// Заправка сравнивается с исходным снимком, а не с номиналом: у каждого
	// пуска свой разброс, и Reset обязан воспроизвести именно его.
	approx(t, after.FuelMass, before.FuelMass, 1e-9, "топливо после сброса")
	approx(t, after.DryMass, before.DryMass, 1e-9, "сухая масса после сброса")

	if sim.Phase() != orbit.PhasePreLaunch {
		t.Errorf("фаза после сброса = %v, ожидалась PreLaunch", sim.Phase())
	}
	if len(after.Engines) != 9 {
		t.Errorf("после сброса должно быть 9 двигателей, получено %d", len(after.Engines))
	}
	if after.MaxQ != 0 {
		t.Errorf("статистика Max-Q не сброшена: %.1f Па", after.MaxQ)
	}
	for _, e := range after.Engines {
		if e.WallTemp > 100 {
			t.Errorf("тепловое состояние двигателя %s не сброшено: %.1f °C", e.ID, e.WallTemp)
		}
	}
}

// -----------------------------------------------------------------------------
// Идентификаторы двигателей
// -----------------------------------------------------------------------------

// Двигатели разных ступеней обязаны иметь разные идентификаторы.
// Раньше нумерация начиналась с единицы в каждой ступени, и на графиках
// запуск двигателя второй ступени выглядел как повторный запуск первого
// двигателя первой ступени.

func TestRollProgramAlignsWithFlightPlane(t *testing.T) {
	cfg := vehicle.Falcon9Config()

	initial := cfg.InitialRoll()
	if math.Abs(initial) < 5 {
		t.Fatalf("начальный крен = %.1f°, ожидался заметный при азимуте стола 90° "+
			"и наклонении %.1f°", initial, cfg.TargetInclination)
	}

	sim := NewSimulationWithSeed(cfg, DefaultTimeConfig(), testSeed)

	sim.mu.Lock()
	defer sim.mu.Unlock()

	// На столе крен равен ориентации пускового устройства.
	if math.Abs(sim.gnc.Attitude.Roll-initial) > 1e-9 {
		t.Errorf("крен на столе = %.2f°, ожидался %.2f°", sim.gnc.Attitude.Roll, initial)
	}

	// К концу ролл-программы крен должен обнулиться.
	for i := 0; i < 1500; i++ {
		sim.step(0.1)
		if sim.state.Altitude() > sim.gnc.Config.RollProgramAltitude+2000 {
			break
		}
	}

	if math.Abs(sim.gnc.Attitude.Roll) > 1.0 {
		t.Errorf("после ролл-программы крен = %.2f°, ожидался около нуля",
			sim.gnc.Attitude.Roll)
	}
}

// Угол атаки в плотных слоях должен оставаться малым: иначе поперечная
// нагрузка разрушит корпус.

func TestAngleOfAttackStaysSmallInAtmosphere(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), testSeed)

	sim.mu.Lock()
	defer sim.mu.Unlock()

	maxAoA := 0.0
	var worstQ, worstAlt float64

	for i := 0; i < 15000; i++ {
		sim.step(0.1)
		if sim.phase != orbit.PhaseFirstStage {
			break
		}
		nav := sim.navState()
		// Ограничение имеет смысл только там, где есть напор.
		if nav.DynamicPressure < 5000 {
			continue
		}
		if a := math.Abs(sim.gnc.TotalAoA); a > maxAoA {
			maxAoA = a
			worstQ = nav.DynamicPressure
			worstAlt = nav.Altitude
		}
	}

	if maxAoA > 10 {
		t.Errorf("максимальный угол атаки = %.2f° при напоре %.1f кПа на высоте %.1f км, "+
			"ожидалось не более 10°", maxAoA, worstQ/1000, worstAlt/1000)
	}
	t.Logf("Максимальный угол атаки в плотных слоях: %.2f°", maxAoA)
}

// Угловые скорости не должны превышать заданные пределы.

func TestAngularRatesRespectLimits(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), testSeed)
	limits := sim.gnc.Config.RateLimits

	sim.mu.Lock()
	defer sim.mu.Unlock()

	const tolerance = 1e-6
	for i := 0; i < 6000; i++ {
		sim.step(0.1)
		if math.Abs(sim.gnc.PitchRate) > limits.Pitch+tolerance {
			t.Fatalf("скорость по тангажу = %.3f °/с, предел %.3f", sim.gnc.PitchRate, limits.Pitch)
		}
		if math.Abs(sim.gnc.YawRate) > limits.Yaw+tolerance {
			t.Fatalf("скорость по рысканию = %.3f °/с, предел %.3f", sim.gnc.YawRate, limits.Yaw)
		}
		if math.Abs(sim.gnc.RollRate) > limits.Roll+tolerance {
			t.Fatalf("скорость по крену = %.3f °/с, предел %.3f", sim.gnc.RollRate, limits.Roll)
		}
	}
}

// -----------------------------------------------------------------------------
// Перицентр на суборбитальном участке
// -----------------------------------------------------------------------------

// Пока траектория не замкнута, апоцентр и перицентр публикуются как NaN.
// Раньше на графике появлялся перицентр в минус шесть тысяч километров —
// формально верный для баллистической горки, но ломающий масштаб панели.

func TestOrbitParametersAreNaNWhileSuborbital(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), testSeed)

	sim.mu.Lock()
	for i := 0; i < 600; i++ {
		sim.step(0.1)
	}
	sim.telemetry = sim.buildTelemetryLocked()
	sim.mu.Unlock()

	tel := sim.Snapshot()
	if tel.OrbitClosed {
		t.Fatal("на активном участке траектория не может быть замкнутой")
	}

	// Перицентр на активном участке лежит внутри Земли — значение бессмысленно
	// для графика, наружу идёт NaN.
	if tel.Periapsis.Defined() {
		t.Errorf("перицентр = %.1f, ожидался NaN на суборбитальном участке",
			tel.Periapsis.Value())
	}

	// Апоцентр, наоборот, определён и осмыслен: по нему работает наведение,
	// и на графике видно, как он подтягивается к целевой высоте.
	if !tel.Apoapsis.Defined() {
		t.Error("апоцентр должен быть определён на активном участке")
	}
	if tel.Apoapsis.Value() <= 0 {
		t.Errorf("апоцентр = %.1f м, ожидалось положительное значение",
			tel.Apoapsis.Value())
	}
}

// Апоцентр обязан расти по мере выведения: это главный показатель работы
// наведения второй ступени.

func TestApoapsisGrowsDuringAscent(t *testing.T) {
	sim := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), testSeed)

	sim.mu.Lock()
	defer sim.mu.Unlock()

	var samples []float64
	for i := 0; i < 6000; i++ {
		sim.step(0.1)
		if i%500 == 0 {
			tel := sim.buildTelemetryLocked()
			if !tel.Apoapsis.Defined() {
				t.Fatalf("апоцентр не определён на T+%.0f с", tel.Time)
			}
			samples = append(samples, tel.Apoapsis.Value())
		}
	}

	if len(samples) < 3 {
		t.Fatal("недостаточно отсчётов")
	}
	if samples[len(samples)-1] <= samples[0] {
		t.Errorf("апоцентр не вырос за выведение: %.1f км → %.1f км",
			samples[0]/1000, samples[len(samples)-1]/1000)
	}
	t.Logf("Апоцентр за первые 600 с: %.1f км → %.1f км",
		samples[0]/1000, samples[len(samples)-1]/1000)
}

// Телеметрия первой ступени существует с момента старта, а не появляется
// только после разделения.

func TestOrbitParametersBecomeFiniteOnOrbit(t *testing.T) {
	sim, tel := runAscent(t, 2000)
	if sim.Phase() != orbit.PhaseOrbital {
		t.Skip("орбита не достигнута")
	}

	if !tel.OrbitClosed {
		t.Error("на орбите траектория должна быть замкнутой")
	}
	if !tel.Periapsis.Defined() || !tel.Apoapsis.Defined() {
		t.Error("на орбите апоцентр и перицентр должны быть определены")
	}
	if tel.Periapsis.Value() <= 0 {
		t.Errorf("перицентр на орбите = %.1f м", tel.Periapsis.Value())
	}
}

// -----------------------------------------------------------------------------
// Воспроизводимость и разброс
// -----------------------------------------------------------------------------

// Один и тот же seed обязан давать побитово одинаковый полёт.

func TestAngularMomentumConservedInVacuum(t *testing.T) {
	pos := physics.Vec3{X: physics.EarthRadius + 500000}
	vel := physics.Vec3{Y: 3000}

	stage := env.NewSpentStage("test", pos, vel, 25600, 42.6, 1.83, 300, 0,
		physics.Vec3{X: 0.05, Y: 0.2, Z: 0.1}, env.CalmWind())

	initial := stage.Inertia.AngularMomentum(stage.State.AngularVelocity).Norm()
	energy := stage.Inertia.RotationalEnergy(stage.State.AngularVelocity)

	for i := 0; i < 2000; i++ {
		stage.Step(0.1, float64(i)*0.1)
	}

	final := stage.Inertia.AngularMomentum(stage.State.AngularVelocity).Norm()
	finalEnergy := stage.Inertia.RotationalEnergy(stage.State.AngularVelocity)

	if rel := math.Abs(final-initial) / initial; rel > 1e-6 {
		t.Errorf("момент импульса изменился на %.3g в отсутствие момента сил", rel)
	}
	if rel := math.Abs(finalEnergy-energy) / energy; rel > 1e-6 {
		t.Errorf("энергия вращения изменилась на %.3g", rel)
	}
}

// Кватернион обязан оставаться единичным на протяжении всего полёта.

func TestQuaternionStaysNormalized(t *testing.T) {
	pos := physics.Vec3{X: physics.EarthRadius + 120000}
	frame := physics.NewLocalFrame(pos)
	vel := frame.Direction(-20, 90).Scale(2500)

	stage := env.NewSpentStage("test", pos, vel, 25600, 42.6, 1.83, 300, 0,
		physics.Vec3{Y: 0.3, Z: 0.2}, env.CalmWind())

	for i := 0; i < 5000 && stage.Alive(); i++ {
		stage.Step(0.05, float64(i)*0.05)
		if n := stage.State.Orientation.Norm(); math.Abs(n-1) > 1e-9 {
			t.Fatalf("норма кватерниона = %.12f на шаге %d", n, i)
		}
	}
}

// Неуправляемая ступень при входе в атмосферу обязана раскручиваться:
// центр давления впереди центра масс, конфигурация статически неустойчива.

func TestPitchProgramIsMonotoneAndContinuous(t *testing.T) {
	prev := orbit.FirstStagePitchProgram(0)
	for alt := 0.0; alt <= 120000; alt += 100 {
		p := orbit.FirstStagePitchProgram(alt)
		if p > prev+1e-9 {
			t.Fatalf("программа тангажа выросла на высоте %.0f м: %.3f → %.3f",
				alt, prev, p)
		}
		if prev-p > 1.0 {
			t.Fatalf("скачок программы тангажа на высоте %.0f м: %.3f → %.3f",
				alt, prev, p)
		}
		prev = p
	}
	if p := orbit.FirstStagePitchProgram(0); p < 70 || p > 90 {
		t.Errorf("у поверхности программа задаёт %.1f°, ожидалось около 77°", p)
	}
	if p := orbit.FirstStagePitchProgram(120000); p > 30 {
		t.Errorf("к высоте разделения программа задаёт %.1f°, ожидался пологий полёт", p)
	}
}

// -----------------------------------------------------------------------------
// Сериализация телеметрии
// -----------------------------------------------------------------------------

// Телеметрия обязана сериализоваться в JSON на любом участке полёта.
//
// encoding/json возвращает ошибку «unsupported value» на NaN, поэтому
// неопределённые апоцентр и перицентр на суборбитальном участке роняли
// весь ответ REST-эндпоинта в 500. Наружу они уходят как null.

func runAscent(t *testing.T, maxModelSeconds float64) (*Simulation, Telemetry) {
	t.Helper()

	// Фиксированное зерно: иначе тест плавает вместе с разбросом характеристик
	// и профилем ветра, и падение невозможно воспроизвести.
	sim := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), testSeed)

	const dt = 0.1
	steps := int(maxModelSeconds / dt)

	sim.mu.Lock()
	defer sim.mu.Unlock()

	for i := 0; i < steps; i++ {
		sim.step(dt)
		if sim.crashed || sim.phase == orbit.PhaseOrbital {
			break
		}
	}
	sim.telemetry = sim.buildTelemetryLocked()
	return sim, sim.telemetry
}

// runUntilPhase прогоняет симуляцию до достижения заданной фазы.

func runUntilPhase(t *testing.T, phase orbit.FlightPhase, maxModelSeconds float64) (*Simulation, Telemetry) {
	t.Helper()

	sim := NewSimulationWithSeed(vehicle.Falcon9Config(), DefaultTimeConfig(), testSeed)

	const dt = 0.1
	sim.mu.Lock()
	for i := 0; i < int(maxModelSeconds/dt); i++ {
		sim.step(dt)
		if sim.phase >= phase || sim.crashed {
			break
		}
	}
	sim.telemetry = sim.buildTelemetryLocked()
	reached := sim.phase
	sim.mu.Unlock()

	if reached < phase {
		t.Fatalf("фаза %v не достигнута за %.0f с, остановились на %v",
			phase, maxModelSeconds, reached)
	}
	return sim, sim.Snapshot()
}

func TestAscentReachesOrbit(t *testing.T) {
	sim, tel := runAscent(t, 2000)

	if tel.Crashed {
		t.Fatalf("ракета разбилась на T+%.0f с, высота %.1f км", tel.Time, tel.Altitude/1000)
	}
	if sim.Phase() != orbit.PhaseOrbital {
		t.Fatalf("фаза на T+%.0f с = %v, орбита не достигнута (апоцентр %.1f км, перицентр %.1f км)",
			tel.Time, sim.Phase(), tel.Apoapsis.Value()/1000, tel.Periapsis.Value()/1000)
	}

	target := sim.Config.TargetOrbitAltitude

	if tel.Periapsis.Value() < target*0.9 {
		t.Errorf("перицентр = %.1f км, ожидалось не ниже %.1f км",
			tel.Periapsis.Value()/1000, target*0.85/1000)
	}
	// Прямое выведение одним включением всегда даёт апоцентр выше целевого:
	// ступень выключается по достижении перицентра, а к этому моменту верхняя
	// точка орбиты успевает уйти вверх. Двухимпульсная схема дала бы точнее,
	// но потребовала бы отдельного манёвра.
	if tel.Apoapsis.Value() > target*1.35 {
		t.Errorf("апоцентр = %.1f км, ожидалось не выше %.1f км",
			tel.Apoapsis.Value()/1000, target*1.35/1000)
	}
	if tel.Eccentricity > 0.05 {
		t.Errorf("эксцентриситет = %.4f, ожидалось меньше 0.02", tel.Eccentricity)
	}

	// A5/A6: наклонение должно быть результатом динамики и совпасть с целевым.
	if math.Abs(tel.Inclination-sim.Config.TargetInclination) > 2.0 {
		t.Errorf("наклонение = %.2f°, целевое %.2f°",
			tel.Inclination, sim.Config.TargetInclination)
	}

	t.Logf("Орбита достигнута на T+%.0f с: %.1f × %.1f км, e=%.5f, i=%.2f°, топливо %.1f т",
		tel.Time, tel.Periapsis.Value()/1000, tel.Apoapsis.Value()/1000,
		tel.Eccentricity, tel.Inclination, tel.FuelMass/1000)
}

// Max-Q должен приходиться на реалистичную высоту и величину.

func TestMaxQIsRealistic(t *testing.T) {
	_, tel := runAscent(t, 2000)

	if tel.MaxQ < 20000 || tel.MaxQ > 50000 {
		t.Errorf("максимальный скоростной напор = %.1f кПа, ожидалось 20…50 кПа", tel.MaxQ/1000)
	}
	if tel.MaxQAltitude < 8000 || tel.MaxQAltitude > 20000 {
		t.Errorf("высота Max-Q = %.1f км, ожидалось 8…20 км", tel.MaxQAltitude/1000)
	}
	t.Logf("Max-Q = %.1f кПа на T+%.0f с, высота %.1f км",
		tel.MaxQ/1000, tel.MaxQTime, tel.MaxQAltitude/1000)
}

// A2/A3: наземный трек не должен уползать на восток из-за двойного учёта
// вращения Земли. Проверяем, что смещение по долготе согласуется с
// горизонтальной скоростью относительно поверхности.

func TestGroundTrackConsistentWithGroundSpeed(t *testing.T) {
	sim := NewSimulation()

	sim.mu.Lock()
	defer sim.mu.Unlock()

	// Первые секунды на столе: ракета почти неподвижна относительно земли,
	// значит и координаты почти не меняются.
	startLat := sim.Config.LaunchLatitude
	startLon := sim.Config.LaunchLongitude

	for i := 0; i < 30; i++ {
		sim.step(0.1)
	}
	tel := sim.buildTelemetryLocked()

	// За 3 секунды вертикального подъёма смещение по долготе исчезающе мало.
	if math.Abs(tel.Longitude-startLon) > 0.001 {
		t.Errorf("долгота ушла на %.5f° за 3 с вертикального подъёма",
			tel.Longitude-startLon)
	}
	if math.Abs(tel.Latitude-startLat) > 0.001 {
		t.Errorf("широта ушла на %.5f° за 3 с вертикального подъёма",
			tel.Latitude-startLat)
	}

	// Скорость относительно поверхности должна быть заметно меньше
	// инерциальной: разница и есть вращение Земли, учтённое один раз.
	if tel.GroundSpeed >= tel.HorizontalVelocity {
		t.Errorf("путевая скорость (%.1f) не меньше инерциальной (%.1f) — "+
			"вращение Земли учтено неверно", tel.GroundSpeed, tel.HorizontalVelocity)
	}
}

// A4: латч восходящей/нисходящей ветви не должен залипать в точке
// максимальной широты — иначе трек перестаёт быть синусоидой.

func TestGroundTrackCrossesMaxLatitude(t *testing.T) {
	sim, tel := runAscent(t, 2000)
	if sim.Phase() != orbit.PhaseOrbital {
		t.Skip("орбита не достигнута, проверка трека неприменима")
	}

	// Летим ещё полтора витка и следим за широтой.
	period := tel.OrbitPeriod
	if period <= 0 {
		t.Fatal("период орбиты не определён")
	}

	sim.mu.Lock()
	defer sim.mu.Unlock()

	const dt = 1.0
	steps := int(1.5 * period / dt)

	minLat, maxLat := math.Inf(1), math.Inf(-1)
	rising, falling := false, false
	prev := sim.buildTelemetryLocked().Latitude

	for i := 0; i < steps; i++ {
		sim.step(dt)
		lat := physics.ECIToGeodetic(sim.state.Position, sim.elapsed).Latitude
		minLat = math.Min(minLat, lat)
		maxLat = math.Max(maxLat, lat)
		if lat > prev+1e-6 {
			rising = true
		}
		if lat < prev-1e-6 {
			falling = true
		}
		prev = lat
	}

	if !rising || !falling {
		t.Errorf("широта менялась только в одну сторону (рост=%v, спад=%v) — "+
			"признак залипания латча направления", rising, falling)
	}

	target := sim.Config.TargetInclination
	if maxLat < target-5 || maxLat > target+2 {
		t.Errorf("максимальная широта = %.2f°, ожидалась около наклонения %.2f°", maxLat, target)
	}
	if minLat > -(target - 5) {
		t.Errorf("минимальная широта = %.2f°, трек не дошёл до южной ветви", minLat)
	}
	t.Logf("Трек за 1.5 витка: широта от %.2f° до %.2f° при наклонении %.2f°",
		minLat, maxLat, target)
}

// C5: на орбите остаточная атмосфера должна тормозить аппарат.
