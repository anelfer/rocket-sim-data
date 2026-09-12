package simulator

import (
	"log"
	"math"

	"rocketTelemetrySim/metrics"
	"rocketTelemetrySim/simulator/env"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Логика смены фаз полёта.
//
// Все пороги берутся из конфигурации носителя или вычисляются из орбитальных
// элементов. Магических литералов вроде «523454.0 − FuelMass» больше нет:
// такое выражение молча ломалось при любом изменении массы ракеты.
// -----------------------------------------------------------------------------

// updateFlightPhase проверяет условия перехода между фазами.
// Вызывается под удержанным mu.
func (s *Simulation) updateFlightPhase(nav orbit.NavState) {
	switch s.phase {

	case orbit.PhasePreLaunch:
		s.phase = orbit.PhaseFirstStage
		s.startAllEngines()
		log.Printf("🚀 Старт! Стартовая масса %.1f т, тяга %.1f МН, TWR %.2f",
			(s.dryMass+s.state.FuelMass)/1000,
			vehicle.TotalThrust(s.engines)/1e6,
			vehicle.TotalThrust(s.engines)/((s.dryMass+s.state.FuelMass)*physics.G0))

	case orbit.PhaseFirstStage:
		if s.shouldMECO(nav) {
			s.performMECO()
		}

	case orbit.PhaseCoast1:
		if s.elapsed-s.mecoTime >= s.Config.StageSeparationDelay {
			s.performStageSeparation()
		}

	case orbit.PhaseSecondStage:
		if s.shouldSECO(nav) {
			s.performSECO(nav)
		}

	case orbit.PhaseCoast2:
		if s.shouldStartCircularization(nav) {
			s.startCircularization(nav)
		}

	case orbit.PhaseCircularization:
		if s.shouldFinishCircularization(nav) {
			s.finishCircularization(nav)
		}

	case orbit.PhaseOrbital:
		if s.entryRequested {
			s.startDeorbit()
			break
		}
		s.maintainOrbit(nav)

	case orbit.PhaseDeorbit:
		s.updateDeorbit(nav)

	case orbit.PhaseEntry:
		s.updateEntry(nav)

	case orbit.PhaseLandingFlip:
		s.updateLandingFlip(nav)

	case orbit.PhaseLandingBurn:
		s.updateLandingBurn(nav)
	}
}

// -----------------------------------------------------------------------------
// Первая ступень
// -----------------------------------------------------------------------------

// shouldMECO проверяет условия выключения двигателей первой ступени.
func (s *Simulation) shouldMECO(nav orbit.NavState) bool {
	stage := s.Config.FirstStage

	// Топливо израсходовано до резерва, оставленного на возврат ступени.
	//
	// Порог по остатку — это ровно то решение, которое в реальности
	// принимается по показанию датчика, а не по факту в баке: настоящего
	// прямого измерения массы не существует физически (см.
	// DefaultPropellantSensor), и борт не может знать остаток точнее,
	// чем говорит его собственная оценка.
	if s.sensedFuelMass <= stage.FuelReserve {
		return true
	}

	// Либо кончился один из компонентов: дальше двигатель работать не может
	// независимо от того, сколько осталось второго. Это не показание
	// датчика остатка, а физический факт — сорванная подача горючего или
	// окислителя, который сама камера обнаруживает по срыву горения
	// независимо ни от какой оценки массы.
	if s.propulsion != nil {
		if s.propulsion.FuelTank.Empty() || s.propulsion.OxTank.Empty() {
			return true
		}
	}

	// Достигнуты целевые скорость и высота разделения.
	return nav.TransverseVelocity >= s.Config.MECOVelocity &&
		nav.Altitude >= s.Config.MECOMinAltitude
}

// hotStageThrottle — уровень тяги центральных камер на горячем разделении.
//
// Задача этих камер — не разгон, а опора: пока корабль запускает свои
// двигатели и отходит, ступень обязана оставаться под тягой. Иначе связка
// переходит в невесомость, компоненты в баках всплывают, и запуск верхней
// ступени приходится делать на осадке двигателями малой тяги.
const hotStageThrottle = 0.4

// hotStageEngines — сколько камер бустера остаётся работать при разделении.
//
// Пять: три центральные (они же качаются в подвесе, значит связка остаётся
// управляемой до самого расхождения) и две с соседнего кольца — по записям
// реальных пусков Super Heavy V3 остаётся именно пять, а не только
// центральная тройка.
const hotStageEngines = 5

func (s *Simulation) performMECO() {
	s.mecoTime = s.elapsed

	// Горячее разделение.
	//
	// Обычная схема — выключить ступень целиком, разойтись и уже потом
	// запускать верхнюю. Здесь иначе: бустер гасит всё, кроме трёх
	// центральных камер, корабль зажигает свои двигатели прямо в связке,
	// и расхождение идёт под тягой. Выигрыш — потерянные секунды свободного
	// полёта и отсутствие осадки: у сверхтяжёлого носителя это десятки
	// метров в секунду характеристической скорости.
	s.setRunningEngines(hotStageEngines)
	s.stagingThrottle = hotStageThrottle

	s.phase = orbit.PhaseCoast1
	log.Printf("🔴 Дросселирование перед горячим разделением на T+%.1f с: "+
		"высота %.1f км, скорость %.0f м/с, остаётся %d камеры, остаток %.1f т",
		s.elapsed, s.state.Altitude()/1000, s.state.TransverseVelocity(),
		hotStageEngines, s.state.FuelMass/1000)
}

// setRunningEngines оставляет работать первые count камер ступени.
//
// Первыми в списке идут центральные — те, что стоят в подвесе.
func (s *Simulation) setRunningEngines(count int) {
	if count < 0 {
		count = 0
	}
	if count > len(s.engines) {
		count = len(s.engines)
	}

	live := make(map[string]bool, count)
	for i := range s.engines {
		on := i < count
		s.engines[i].Running = on
		if on {
			live[s.engines[i].ID] = true
		} else {
			s.engines[i].Thrust = 0
		}
	}

	if s.propulsion == nil {
		return
	}
	for _, e := range s.propulsion.Engines {
		if live[e.ID] {
			e.Start()
		} else {
			e.Stop()
		}
	}
	s.propulsion.Commissioned = count
}

// performStageSeparation отделяет первую ступень и запускает вторую.
func (s *Simulation) performStageSeparation() {
	cfg := s.Config

	// Отработавшая ступень отделяется с текущим вектором состояния и дальше
	// летит самостоятельно: баллистика, вход в атмосферу, разрушение или удар.
	// Её двигатели больше не публикуются в метрики, поэтому их температуры
	// не «зависают» на последнем значении.
	// Температура обшивки ступени при разделении.
	//
	// Брать её равной температуре торможения потока нельзя: на высоте
	// разделения при числе Маха около девяти та достигает трёх тысяч кельвинов,
	// и ступень «разрушалась» от перегрева в первый же шаг. Обшивка обладает
	// теплоёмкостью и за две минуты работы первой ступени прогревается лишь
	// до нескольких сотен кельвинов.
	const separationSkinLimit = 450.0
	stageSkinTemp := math.Min(separationSkinLimit,
		physics.EffectiveEnvironmentTemperature(
			s.state.Altitude(), s.state.AirRelativeVelocity().Norm()))

	// Носители с активным возвратом (решётчатые рули, посадочный импульс,
	// приводнение в заливе) получают полноценный второй аппарат вместо
	// пассивной баллистики — см. booster.go. Остальные (например Falcon 9
	// в этой модели) ведут себя как прежде.
	if cfg.BoosterReturn {
		s.booster = NewBooster(cfg, s.state.Position, s.state.Velocity,
			s.elapsed, s.rng, s.sensorRng, s.seed, stageStartTemperature(cfg), s.attitude.Orientation)
		s.calibrateBoosterBoard()
	} else {
		s.spentStage = env.NewSpentStage(
			cfg.FirstStage.Name,
			s.state.Position, s.state.Velocity,
			cfg.FirstStage.DryMass+cfg.FirstStage.FuelReserve,
			cfg.FirstStageLength, cfg.Diameter/2,
			stageSkinTemp,
			s.elapsed,
			env.RandomTipOff(s.rng),
			s.wind,
		)
	}

	s.stage = 2

	// Двигательная установка пересобирается под вторую ступень: свои баки,
	// свой запас наддува, свой двигатель с вакуумным соплом. Перенос состояния
	// с первой ступени был бы неверен — это другие агрегаты.
	structure := cfg.SecondStage.DryMass
	if !s.fairingGone {
		structure += cfg.FairingMass
	}

	// Температура, с которой начинает жизнь железо второй ступени.
	//
	// Брать здесь температуру набегающего потока нельзя: на шестидесяти
	// километрах при полутора тысячах метров в секунду она равна полутора
	// тысячам градусов — это температура торможения воздуха, а не металла.
	// Двигатель до запуска стоял под обтекателем рядом с криогенными баками
	// и холоден. Прежде из-за этого камера второй ступени «рождалась»
	// раскалённой до 1446 К, а подшипники — до 1450 К, и первые полторы
	// минуты работы пульт показывал перегрев, которого не было.
	ambientTemp := stageStartTemperature(cfg)

	s.propulsion = NewPropulsionSystem(StageProps{
		StructureMass:      structure,
		PayloadMass:        cfg.PayloadMass,
		PropellantMass:     cfg.SecondStage.FuelMass,
		MixtureRatio:       cfg.SecondStage.MixtureRatio,
		EngineCount:        vehicle.SecondStagePrimaryCount(cfg),
		EngineGroups:       vehicle.SecondStageGroups(cfg),
		StagePrefix:        "S2",
		EngineConfig:       vehicle.SecondStagePrimaryConfig(cfg),
		FuelTankHeight:     cfg.SecondStage.FuelTankHeight,
		OxTankHeight:       cfg.SecondStage.OxTankHeight,
		FuelTankPressure:   cfg.SecondStage.FuelTankPressure,
		OxTankPressure:     cfg.SecondStage.OxTankPressure,
		FuelPressurantMass: cfg.SecondStage.FuelPressurantMass,
		OxPressurantMass:   cfg.SecondStage.OxPressurantMass,
	}, s.rng, s.sensorRng, ambientTemp)

	s.dryMass = s.propulsion.DryMass()
	s.rcsPropellant = cfg.RCSPropellantMass
	s.state.FuelMass = s.propulsion.PropellantMass()
	s.propulsion.StartAll()

	// Корабль, рассчитанный на возвращение, несёт плавники. Пока он идёт
	// на выведение, они прижаты к борту: раскрытые, на активном участке они
	// дали бы неуправляемый момент.
	if cfg.Flaps {
		length := secondStageLength(cfg)
		s.attitude.Surfaces = NewSurfaceSet(vehicle.ShipFlaps(length, cfg.Diameter/2))
		s.heatShield = NewHeatShield(length, cfg.Diameter, stageStartTemperature(cfg))
	}

	// Двигатели первой ступени улетают вместе с ней. Их наборы меток нужно
	// убрать из реестра, иначе Prometheus продолжит отдавать последние
	// записанные значения, и на графике температура камеры будет выглядеть
	// как застывшая горизонтальная линия до конца полёта.
	retired := make([]string, len(s.engines))
	for i := range s.engines {
		retired[i] = s.engines[i].ID
	}
	metrics.RetireEngines(retired)

	// Двигатель второй ступени создаётся заново: перенос теплового состояния
	// с двигателей первой ступени физически неверен, это другое изделие.
	ambient := physics.EffectiveEnvironmentTemperature(
		s.state.Altitude(), s.state.AirRelativeVelocity().Norm(),
	)
	s.engines = vehicle.BuildSecondStageEngine(cfg, ambient)
	s.gnc.Config.MinThrottle = cfg.SecondStage.MinThrottle

	// Номинальные значения второй ступени отличаются от первой: другой
	// турбонасос, другая горловина, другие баки. Пульт нужно перекалибровать,
	// иначе уставки продолжали бы отсчитываться от параметров первой ступени.
	s.calibrateBoard()

	s.stagingThrottle = 0
	s.phase = orbit.PhaseSecondStage

	log.Printf("🔥 Горячее разделение на T+%.1f с: корабль зажёг %d камеры "+
		"(%d обычных и %d вакуумных), бустер отпущен",
		s.elapsed, len(s.engines), vehicle.SecondStagePrimaryCount(cfg),
		len(s.engines)-vehicle.SecondStagePrimaryCount(cfg))
}

// -----------------------------------------------------------------------------
// Вторая ступень
// -----------------------------------------------------------------------------

// circularizationMargin — во сколько раз остаток топлива должен превышать
// потребность манёвра довыведения, чтобы можно было выключать вторую ступень.
const circularizationMargin = 1.25

// pegSecoReadyTgo — порог GNCSystem.PredictedTgo (см. orbit/peg.go), ниже
// которого PEG считается завершившим манёвр прямого выведения.
//
// PEG целится одновременно в апоцентр и в орбитальную скорость, но апоцентр
// (nav.Elements.ApoapsisAltitude) — оскулирующая, по сути энергетическая
// величина: она обычно приходит к целевой высоте раньше, чем вектор
// скорости успевает довернуться к местному горизонту и набрать нужную
// величину. Выключать двигатель по одному лишь апоцентру — значит обрывать
// наведение на середине манёвра, а не после него: получается глубоко
// эллиптическая орбита (перицентр на тысячи километров внутри Земли),
// которую последующее довыведение поднять уже не может — на это и не
// хватало запаса характеристической скорости, хотя PEG отработал бы
// манёвр до конца, дай ему для этого время. Порог взят с небольшим запасом
// над собственным минимальным τ регулятора (ниже него PEG перестаёт решать
// систему наведения и держит последнюю команду) — не нулевой: доля секунды
// точности здесь не нужна, для этого и существует довыведение как
// страховка на случай небольшого остатка.
const pegSecoReadyTgo = 5.0

// remainingDeltaV возвращает запас характеристической скорости по формуле
// Циолковского для текущей ступени.
func (s *Simulation) remainingDeltaV() float64 {
	if s.state.FuelMass <= 0 || s.dryMass <= 0 {
		return 0
	}
	isp := s.currentStage().VacuumISP
	return isp * physics.G0 * math.Log((s.dryMass+s.state.FuelMass)/s.dryMass)
}

// circularizationDeltaV возвращает приращение скорости, необходимое для
// скругления орбиты в апоцентре.
func circularizationDeltaV(el physics.OrbitalElements) float64 {
	if !el.Elliptical {
		return 0
	}
	apoapsisRadius := physics.EarthRadius + el.ApoapsisAltitude
	speedAtApoapsis := physics.VisVivaSpeed(apoapsisRadius, el.SemiMajorAxis)
	return physics.CircularOrbitVelocity(el.ApoapsisAltitude) - speedAtApoapsis
}

// shouldSECO проверяет условия выключения второй ступени.
//
// Критерий состоит из двух частей: апоцентр достиг целевой высоты И оставшегося
// топлива заведомо хватает на довыведение из этого апоцентра.
//
// Одной высоты апоцентра недостаточно. Баллистическая горка с апоцентром 400 км
// и горизонтальной скоростью 1.5 км/с имеет перицентр глубоко внутри Земли, и
// «довыведение» из такой точки требует почти всей орбитальной скорости — именно
// на этом прежняя логика сжигала всё топливо второй ступени. Проверка запаса
// характеристической скорости отсекает такие случаи и при этом, в отличие от
// жёсткого порога по перицентру, не заставляет ступень работать дольше нужного,
// перебрасывая апоцентр далеко за цель.
func (s *Simulation) shouldSECO(nav orbit.NavState) bool {
	if s.state.FuelMass <= 0 {
		return true
	}

	el := nav.Elements
	if !el.Elliptical {
		// Энергии уже больше, чем нужно для замкнутой орбиты.
		return true
	}

	target := s.Config.TargetOrbitAltitude

	// Перицентр задания. Для круговой орбиты он совпадает с апоцентром,
	// для трансатмосферной траектории лежит внутри атмосферы — и тогда
	// ступень выключается заметно раньше, не добрав до первой космической
	// несколько сотен метров в секунду. Это не «недовыведение», а само
	// задание: возвращать такой корабль будет атмосфера, а не двигатели.
	cutoff := s.Config.PeriapsisTarget()
	if cutoff > 0 {
		cutoff *= 0.97
	}
	if el.PeriapsisAltitude >= cutoff {
		return true
	}

	// Дальше — только замкнутая орбита: условие выключения трансатмосферного
	// задания уже проверено.
	if s.Config.Suborbital() {
		return false
	}

	// Прямое выведение: ступень работает непрерывно до тех пор, пока перицентр
	// не поднимется к целевой высоте. Наведение по апоцентру всё это время
	// удерживает верхнюю точку орбиты у цели, поэтому по мере разгона орбита
	// скругляется сама.
	//
	// Прежний критерий выключал ступень, как только апоцентр достигал цели, —
	// в этот момент перицентр ещё лежал глубоко внутри Земли, и на манёвр
	// довыведения требовалось около 800 м/с. Оставшегося топлива на это уже
	// не хватало, и ракета оставалась на орбите с перицентром 150 км.
	// На низкую орбиту носители такого класса выводят одним включением.
	// Двухимпульсная схема. Как только верхняя точка пришла к цели, разгоняться
	// дальше нечем: у самой первой космической каждые десять метров в секунду
	// поднимают апоцентр на десятки километров, и ступень, работающая «до
	// перицентра», успевает перебросить его на сотню километров за цель.
	// Порог стоял на четверти запаса именно поэтому — и именно поэтому корабль
	// выходил на пятьсот километров вместо четырёхсот. Теперь остаток орбиты
	// добирается довыведением в апоцентре, на что уходит около полутора сотен
	// метров в секунду.
	if el.ApoapsisAltitude >= target {
		required := circularizationDeltaV(el)
		if required <= 0 {
			return true
		}
		if s.remainingDeltaV() < required*circularizationMargin {
			return false
		}
		// Запаса на довыведение хватает, но резать манёвр по одному
		// апоцентру нельзя (см. pegSecoReadyTgo) — ждём, пока сам PEG
		// сочтёт прямое выведение законченным.
		return s.gnc.PredictedTgo <= pegSecoReadyTgo
	}

	return false
}

func (s *Simulation) performSECO(nav orbit.NavState) {
	s.secoTime = s.elapsed
	s.shutdownAllEngines()

	el := nav.Elements
	log.Printf("🔴 SECO на T+%.1f с: апоцентр %.1f км, перицентр %.1f км, e = %.4f",
		s.elapsed, el.ApoapsisAltitude/1000, el.PeriapsisAltitude/1000, el.Eccentricity)

	// Трансатмосферное задание: орбиты нет и не будет. Корабль проходит
	// верхнюю точку и входит в атмосферу сам, поэтому дальше он ведёт себя
	// как возвращаемый аппарат — держит угол атаки и ждёт плотных слоёв.
	if s.Config.Suborbital() {
		s.phase = orbit.PhaseEntry
		log.Printf("🌍 Трансатмосферная траектория: апоцентр %.1f км, "+
			"перицентр %.1f км. Вход начнётся сам, тормозной импульс не нужен",
			el.ApoapsisAltitude/1000, el.PeriapsisAltitude/1000)
		return
	}

	// Прямое выведение завершено — орбита уже замкнута, довыведение не нужно.
	if el.Elliptical && el.PeriapsisAltitude >= s.Config.TargetOrbitAltitude*0.9 {
		s.phase = orbit.PhaseOrbital
		s.finishAscent(nav)
		return
	}

	// Иначе переходим к полёту в апоцентр и довыведению.
	s.phase = orbit.PhaseCoast2
}

// finishAscent записывает итог выведения.
func (s *Simulation) finishAscent(nav orbit.NavState) {
	el := nav.Elements
	if el.Elliptical && el.PeriapsisAltitude > 120000 {
		s.orbitReached = true
		log.Printf("✅ Орбита достигнута на T+%.1f с", s.elapsed)
		log.Printf("   Апоцентр %.1f км, перицентр %.1f км, e = %.5f",
			el.ApoapsisAltitude/1000, el.PeriapsisAltitude/1000, el.Eccentricity)
		log.Printf("   Наклонение %.2f° (цель %.2f°), период %.1f мин, остаток %.2f т",
			el.InclinationDegrees(), s.Config.TargetInclination,
			el.Period()/60, s.state.FuelMass/1000)
		return
	}
	log.Printf("❌ Орбита не достигнута на T+%.1f с: перицентр %.1f км",
		s.elapsed, el.PeriapsisAltitude/1000)
}

// -----------------------------------------------------------------------------
// Довыведение в апоцентре
// -----------------------------------------------------------------------------

// estimateBurnDuration оценивает длительность манёвра для приращения deltaV
// по формуле Циолковского.
func (s *Simulation) estimateBurnDuration(deltaV float64) float64 {
	if deltaV <= 0 {
		return 0
	}
	stage := s.currentStage()
	thrust := stage.VacuumThrust * stage.EngineCount
	if thrust <= 0 {
		return 0
	}
	mass := s.dryMass + s.state.FuelMass
	isp := stage.VacuumISP

	// m_f = m_0 · exp(−Δv / (Isp·g₀)); длительность = израсходованная масса / расход.
	massFlow := thrust / (isp * physics.G0)
	burned := mass * (1 - math.Exp(-deltaV/(isp*physics.G0)))
	return burned / massFlow
}

// shouldStartCircularization определяет момент включения двигателя.
//
// Манёвр центрируется на апоцентре: половина длительности до него,
// половина после. Прежняя версия ждала, пока вертикальная скорость станет
// меньше 150 м/с, — к этому моменту оптимальная точка уже была пройдена.
func (s *Simulation) shouldStartCircularization(nav orbit.NavState) bool {
	el := nav.Elements
	if !el.Elliptical {
		return true
	}

	// Апоцентр упал ниже цели — ждать больше нечего, надо разгоняться.
	if el.ApoapsisAltitude < s.Config.TargetOrbitAltitude*0.5 {
		return true
	}

	burnTime := s.estimateBurnDuration(circularizationDeltaV(el))
	return el.TimeToApoapsis() <= burnTime/2
}

func (s *Simulation) startCircularization(nav orbit.NavState) {
	s.circStart = s.elapsed
	s.phase = orbit.PhaseCircularization

	s.startAllEngines()

	required := physics.CircularOrbitVelocity(nav.Altitude)
	log.Printf("🎯 Довыведение на T+%.1f с: высота %.1f км, скорость %.0f м/с, требуется %.0f м/с",
		s.elapsed, nav.Altitude/1000, nav.TransverseVelocity, required)
}

// shouldFinishCircularization проверяет достижение круговой орбиты.
func (s *Simulation) shouldFinishCircularization(nav orbit.NavState) bool {
	if s.state.FuelMass <= 0 {
		return true
	}

	el := nav.Elements
	if !el.Elliptical {
		return true
	}

	// Цель довыведения — поднять перицентр из атмосферы и скруглить орбиту.
	//
	// Требовать при этом ещё и низкого апоцентра нельзя: если ступень уже
	// забросила верхнюю точку выше цели, снизить её этим манёвром невозможно
	// в принципе, и двигатель работал бы до полной выработки топлива.
	target := s.Config.TargetOrbitAltitude
	if el.PeriapsisAltitude >= target*0.95 {
		return true
	}
	return el.Eccentricity < 0.01 && el.PeriapsisAltitude > target*0.9
}

func (s *Simulation) finishCircularization(nav orbit.NavState) {
	s.shutdownAllEngines()
	s.phase = orbit.PhaseOrbital

	el := nav.Elements
	if el.Elliptical && el.PeriapsisAltitude > 120000 {
		s.orbitReached = true
		log.Printf("✅ Орбита достигнута на T+%.1f с", s.elapsed)
		log.Printf("   Апоцентр %.1f км, перицентр %.1f км, e = %.5f",
			el.ApoapsisAltitude/1000, el.PeriapsisAltitude/1000, el.Eccentricity)
		log.Printf("   Наклонение %.2f° (цель %.2f°), период %.1f мин",
			el.InclinationDegrees(), s.Config.TargetInclination, el.Period()/60)
	} else {
		log.Printf("❌ Орбита не достигнута на T+%.1f с: перицентр %.1f км — вход в атмосферу",
			s.elapsed, el.PeriapsisAltitude/1000)
	}
}

// -----------------------------------------------------------------------------
// Поддержание орбиты
// -----------------------------------------------------------------------------

// maintainOrbit включает двигатели для коррекции орбиты, когда это нужно.
//
// Раньше этот код был недостижим: орбитальная ветка основного цикла
// завершалась безусловным continue, и всё, что ниже, никогда не выполнялось.
// Теперь орбитальный полёт интегрируется тем же самым способом, что и
// активный участок, — включая торможение об остатки атмосферы.
//
// Гасит эта функция только то, что сама же и зажгла (флаг orbitCorrectionBurn).
// Раньше решение «пора гасить» принималось по одному признаку — горят ли
// двигатели вообще, — и не различало, кто их зажёг. Оператор, вручную
// запустивший камеры на орбите кнопкой «Зажечь двигатели», получал их
// погашенными обратно в течение одного такта: автоматика видела «горит,
// а коррекция уже не нужна» и тут же объявляла её завершённой, хотя сама
// огня не зажигала и распоряжаться им не должна.
func (s *Simulation) maintainOrbit(nav orbit.NavState) {
	correction := s.gnc.CheckOrbitCorrection(nav)

	burning := vehicle.RunningEngines(s.engines) > 0

	switch {
	case correction.Needed && correction.AtApsis && correction.DeltaV > 5:
		if !burning && s.state.FuelMass > 50 {
			s.startAllEngines()
			s.orbitCorrectionBurn = true
			log.Printf("🔧 Коррекция орбиты на T+%.1f с (%s): Δv = %.1f м/с, перицентр %.1f км",
				s.elapsed, correction.Reason, correction.DeltaV,
				nav.Elements.PeriapsisAltitude/1000)
		}

	case s.orbitCorrectionBurn && burning && (!correction.Needed || correction.DeltaV <= 1):
		s.shutdownAllEngines()
		s.orbitCorrectionBurn = false
		log.Printf("✅ Коррекция завершена на T+%.1f с: e = %.5f, перицентр %.1f км",
			s.elapsed, nav.Elements.Eccentricity, nav.Elements.PeriapsisAltitude/1000)

	case s.orbitCorrectionBurn && burning && s.state.FuelMass <= 0:
		s.shutdownAllEngines()
		s.orbitCorrectionBurn = false
		log.Printf("⚠️ Топливо исчерпано на T+%.1f с, коррекция прервана", s.elapsed)
	}
}

// stageStartTemperature возвращает температуру конструкции ступени к моменту
// её включения, К.
//
// Верхняя ступень до разделения летит внутри носителя: её двигатели греет
// не набегающий поток, а соседство с баками. У криогенного носителя они
// заметно холоднее стартовых условий, у керосинового — близки к ним.
func stageStartTemperature(cfg vehicle.Config) float64 {
	if vehicle.IsMethalox(cfg) {
		return 200
	}
	return 260
}
