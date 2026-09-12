package simulator

import (
	"log"
	"math"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/orbit/gfold"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// План возврата.
//
// Раньше возврат вели три несвязанные части: кинематический поиск высоты
// розжига на пассивном участке, резервный закон на самом импульсе и выпуклая
// задача (G-FOLD), подключавшаяся в последние десятки метров. Каждая
// отвечала на свой вопрос, ни одна — на главный: существует ли вообще
// достижимая траектория до мягкой посадки, и если нет, то когда об этом
// станет известно. Ответ приходил в последние секунды, когда сделать уже
// ничего нельзя.
//
// Здесь один план, который живёт с конца разворотного импульса и
// пересчитывается заново каждую секунду по принципу отступающего горизонта.
// Он состоит из двух моделей, и это не дублирование, а разделение областей
// применимости:
//
//	пассивный участок — баллистика с сопротивлением (predictLanding):
//	  высокие числа Маха, плотность меняется на порядки, аэродинамика
//	  определяет всё. Выпуклая постановка здесь невалидна: у неё постоянная
//	  тяжесть, нет сопротивления в динамике и линеаризованная масса;
//
//	терминальный участок — выпуклая задача (G-FOLD): решается ИЗ
//	  ПРЕДСКАЗАННОЙ точки розжига, а не из текущего состояния. Именно она
//	  отвечает на вопрос «существует ли мягкая посадка», и отвечает задолго
//	  до того, как до розжига дойдёт дело.
//
// Из этих двух и складывается решение: держать курс на башню или уходить.
// -----------------------------------------------------------------------------

// landingDecision — что план говорит делать.
type landingDecision int

const (
	// landingPlanning — план ещё считается, решение не принято.
	landingPlanning landingDecision = iota

	// landingCommitted — полная траектория до мягкой посадки существует,
	// курс на башню подтверждён.
	landingCommitted

	// landingAborted — достижимость потеряна, уход от башни.
	landingAborted
)

func (d landingDecision) String() string {
	switch d {
	case landingCommitted:
		return "курс на башню"
	case landingAborted:
		return "уход от башни"
	}
	return "расчёт"
}

// landingPlanInterval — период пересчёта плана, с модельного времени.
// Тот же порядок, что и у пересчёта самой траектории посадки: план
// меняется медленно и непрерывно, считать его чаще незачем, а стоит он
// дорого — прогноз до касания плюс выпуклая задача.
const landingPlanInterval = 1.0

// landingPlanTerminalFailures — сколько пересчётов подряд терминальная
// задача должна оказаться неразрешимой, чтобы это считалось потерей
// достижимости, а не разовым отказом решателя.
const landingPlanTerminalFailures = 5

// landingPlanFuelMargin — во сколько раз остаток топлива должен превышать
// потребный по плану, чтобы курс на башню считался обеспеченным.
const landingPlanFuelMargin = 1.3

// landingPlan — непрерывно обновляемый план возврата.
type landingPlan struct {
	Valid     bool
	UpdatedAt float64

	// Прогноз пассивного участка.
	IgnitionAltitude float64
	IgnitionSpeed    float64
	IgnitionVertical float64
	ArrivalMass      float64
	FuelRequired     float64
	RequiredDeltaV   float64
	StoppingMargin   float64
	Schedule         []int

	// Проверка терминального участка выпуклой задачей из предсказанной
	// точки розжига.
	TerminalStatus gfold.Status
	TerminalMiss   float64
	TerminalTof    float64
	TerminalOK     bool

	// PredictedMiss — промах баллистического прогноза падения (с
	// сопротивлением) относительно точки прицеливания, м. Это РАЗОМКНУТЫЙ
	// промах: куда ступень придёт, если с этого момента боковой коррекции
	// больше не делать (см. planMissVector).
	PredictedMiss float64

	// MissEast, MissNorth — тот же промах вектором, в местных осях текущей
	// позиции (nav.Frame). Наведение правит не «на столько-то метров», а «в
	// такую-то сторону»: по одной длине нельзя понять, куда отклонять
	// корпус, и поперечная составляющая промаха оставалась неуправляемой.
	MissEast, MissNorth float64

	// MissTime — время до падения по ТОМУ ЖЕ прогнозу, что дал промах, с.
	// Им и только им промах пересчитывается в потребное поперечное
	// ускорение (coastOffsetDemand): взять сюда другую оценку времени
	// значит просить не то ускорение, а ошибка входит в квадрате.
	MissTime float64

	Decision landingDecision
	Reason   landingAbortReason

	terminalFailures int
}

// updateLandingPlan пересчитывает план возврата.
func (b *Booster) updateLandingPlan(nav orbit.NavState) {
	if b.phase != BoosterCoast && b.phase != BoosterLandingBurn {
		return
	}
	if b.plan.Valid && b.elapsed-b.plan.UpdatedAt < landingPlanInterval {
		return
	}
	vz, _ := descentComponents(nav)
	if vz <= 0 {
		// На восходящей ветви прогноз о посадке бессмыслен — план ждёт.
		return
	}

	b.plan.UpdatedAt = b.elapsed
	b.plan.Valid = true

	// --- 1. Пассивный участок: куда и с чем придём -------------------------
	ignition, reachable := b.landingBurnIgnitionAltitude(nav)
	aoaCoast := b.coastAngleOfAttack(nav)
	align := b.ignitionAlignDelay(nav)
	p := b.predictLanding(nav, ignition, aoaCoast, align)

	b.plan.IgnitionAltitude = ignition
	b.plan.IgnitionVertical = p.ArrivalVertical
	b.plan.IgnitionSpeed = math.Hypot(p.ArrivalVertical, p.ArrivalHorizontal)
	b.plan.ArrivalMass = p.ArrivalMass
	b.plan.FuelRequired = p.FuelUsed
	b.plan.StoppingMargin = p.MinMargin
	b.plan.Schedule = p.Schedule
	b.plan.RequiredDeltaV = b.planDeltaV(p)
	b.plan.MissEast, b.plan.MissNorth, b.plan.PredictedMiss, b.plan.MissTime = b.planMissVector(nav)

	// --- 2. Терминальный участок: существует ли мягкая посадка -------------
	//
	// Задача решается ИЗ ПРЕДСКАЗАННОЙ точки розжига. Решать её из текущего
	// состояния на пассивном участке нельзя: там числа Маха под три,
	// сопротивление определяет движение, и выпуклая модель, у которой в
	// динамике нет ни того ни другого, ответила бы про другой аппарат.
	b.plan.TerminalOK = false
	b.plan.TerminalStatus = gfold.StatusFailed
	if reachable && p.Reached && !p.Dry {
		if traj := b.probeTerminal(nav, p); traj != nil {
			b.plan.TerminalStatus = traj.Status
			b.plan.TerminalMiss = traj.Miss
			b.plan.TerminalTof = traj.TimeOfFlight()
			// Достижимость — это не «решатель сошёлся», а «сошёлся НА
			// ПЛОЩАДКЕ». План с промахом в шестнадцать километров
			// разрешим и физичен, но к башне отношения не имеет: это
			// мягкая посадка в стороне, то есть тот же увод, только не
			// объявленный.
			b.plan.TerminalOK = traj.Status == gfold.StatusOptimal &&
				traj.Miss <= gfoldTouchdownTolerance
		}
	}

	if b.plan.TerminalOK {
		b.plan.terminalFailures = 0
	} else {
		b.plan.terminalFailures++
	}

	// --- 3. Решение --------------------------------------------------------
	b.decideLandingPlan(nav, p, reachable)
}

// planDeltaV — характеристическая скорость, которую предстоит потратить на
// посадочный импульс, м/с. Считается по формуле Циолковского из расхода,
// назначенного прогнозом, — то есть из той же траектории, а не отдельной
// оценкой, которая могла бы с ней разойтись.
func (b *Booster) planDeltaV(p landingPrediction) float64 {
	if p.ArrivalMass <= 0 || p.FuelUsed <= 0 {
		return 0
	}
	final := p.ArrivalMass - p.FuelUsed
	if final <= 0 {
		return math.Inf(1)
	}
	return b.Config.FirstStage.VacuumISP * physics.G0 * math.Log(p.ArrivalMass/final)
}

// planMissVector — промах баллистического прогноза падения относительно
// точки прицеливания: составляющие на восток и север в местных осях текущей
// позиции и его длина, м.
//
// Прогноз строится ЧИСТЫМ tail-first, а не по командной ориентации. Это и
// значит «разомкнутый промах»: куда ступень придёт, если с этого момента
// боковой коррекции больше не делать. Иначе получается петля — команда
// отклонения считается по промаху (coastOffsetDemand), а промах считается по
// команде отклонения, и регулятор двигает собственную прогнозируемую цель:
// стоит ему довернуть корпус, как прогноз послушно показывает, что промах
// уже исправлен, и доворот снимается, хотя в действительности не исправлено
// ничего.
//
// Торможение от этого почти не меняется: на рабочих 5…15° от tail-first
// сопротивление отличается от чистого торца меньше чем на десятую долю (см.
// TestBodyAeroSmallOffsetIsNotABrake) — отклонение здесь орган БОКОВОГО
// управления, а не тормоз.
func (b *Booster) planMissVector(nav orbit.NavState) (east, north, miss, flightTime float64) {
	radius := b.Config.Diameter / 2
	impact, ft, ok := predictImpact(nav.Position, nav.Velocity,
		b.dryMass()+b.state.FuelMass, math.Pi*radius*radius,
		b.Config.FirstStageLength*b.Config.Diameter, 180)
	if !ok {
		nan := math.NaN()
		return nan, nan, nan, nan
	}
	target := physics.GeodeticToECEF(b.landingGroundAimPoint())
	d := physics.ECIToECEF(impact, b.elapsed+ft).Sub(target)

	// Раскладка — в местных осях ТЕКУЩЕЙ позиции (там же, где потом
	// строится цель ориентации): восток и север физически те же
	// направления и в ECEF, и в ECI-базисе nav.Frame, поэтому коэффициенты
	// переносятся напрямую (тот же приём, что в boostbackMissVector).
	local := physics.NewLocalFrame(physics.ECIToECEF(nav.Position, b.elapsed))
	east, north, _ = local.Decompose(d)
	return east, north, math.Hypot(east, north), ft
}

// probeTerminal решает терминальную задачу из ПРЕДСКАЗАННОЙ точки розжига.
//
// Синтетическое состояние строится по прогнозу: та же вертикаль местного
// базиса, высота и раскладка скорости на вертикальную и горизонтальную —
// то есть ровно то, что прогноз и предсказывает. Направление горизонтали
// берётся текущее: за оставшийся пассивный участок оно почти не меняется,
// а на разрешимость влияет слабо.
func (b *Booster) probeTerminal(nav orbit.NavState, p landingPrediction) *gfold.Trajectory {
	if !p.HandoverFound || p.HandoverGroup <= 0 || p.HandoverMass <= 0 {
		return nil
	}

	// Положение — не текущее, а ПРЕДСКАЗАННОЕ: точка, куда ступень придёт
	// к моменту передачи управления. Текущее положение здесь было прямой
	// ошибкой: посреди пассивного участка ступень в сотнях километров от
	// площадки, и задача «сесть на площадку с семисот метров, находясь в
	// двухстах километрах» неразрешима при любой тяге. По этой мнимой
	// неразрешимости объявлялся увод, точка прицеливания уезжала на пять
	// километров, и терминальное наведение уже не могло сойтись ни при
	// каких условиях. Измерено: против настоящей площадки та же задача из
	// той же точки решается точно, с нулевым промахом.
	up := nav.Position.Unit()

	horiz := nav.GroundRelativeVelocity
	horiz = horiz.Sub(up.Scale(horiz.Dot(up)))
	dir := physics.Vec3{}
	if n := horiz.Norm(); n > 1e-6 {
		dir = horiz.Scale(1 / n)
	}

	pos := nav.Position.Add(dir.Scale(p.HandoverTravel))
	pos = pos.Unit().Scale(physics.EarthRadius + p.HandoverAltitude)

	up = pos.Unit()
	vel := dir.Scale(p.HandoverHorizontal).
		Sub(up.Scale(p.HandoverVertical)).
		Add(physics.CorotatingVelocity(pos))

	indices := b.landingGroupIndices[b.groupScheduleIndex(p.HandoverGroup)]
	probe := orbit.NewNavState(pos, vel, physics.Vec3{},
		p.HandoverMass, 0, cfgVacuumISP(b.Config), b.elapsed, orbit.PhaseEntry)

	// Время полёта — кинематическая оценка по располагаемому замедлению
	// назначенной прогнозом группы: с чего-то перебор по времени начинать
	// надо, и оценка из той же траектории честнее произвольной константы.
	perEngine := vehicle.ThrustAtAltitude(b.Config.FirstStage,
		physics.Atmosphere(p.HandoverAltitude).Pressure)
	aMax := perEngine*float64(p.HandoverGroup)/p.HandoverMass -
		physics.GravityMagnitudeAtAltitude(p.HandoverAltitude)
	tof := gfoldMinTimeOfFlight
	if aMax > 0 {
		tof = math.Max(gfoldMinTimeOfFlight,
			math.Hypot(p.HandoverVertical, p.HandoverHorizontal)/aMax)
	}

	return b.gfoldSolve(probe, p.HandoverGroup, indices, tof, false)
}

// gfoldMinTimeOfFlight — нижняя оценка времени терминального участка, с.
const gfoldMinTimeOfFlight = 4.0

// groupScheduleIndex — место группы в посадочной последовательности.
func (b *Booster) groupScheduleIndex(group int) int {
	for i, n := range landingEngineSequence {
		if n == group {
			return i
		}
	}
	return 0
}

// decideLandingPlan принимает решение: курс на башню или уход.
//
// Решение — по СОВОКУПНОСТИ физической достижимости, а не по одной высоте:
// существует ли терминальная траектория, хватает ли топлива на неё с
// запасом, остаётся ли положительным запас по тормозному пути, собирается
// ли группа камер. Ни один признак по отдельности не достаточен.
func (b *Booster) decideLandingPlan(nav orbit.NavState, p landingPrediction, reachable bool) {
	if b.plan.Decision == landingAborted {
		return
	}

	if reason := b.abortPlan.confirm(b.elapsed, b.planAbortReason(nav, p, reachable)); reason != landingAbortNone {
		b.plan.Decision = landingAborted
		b.plan.Reason = reason
		b.declareLandingAbort(nav, reason)
		return
	}

	// Курс на башню подтверждается только тогда, когда полная траектория до
	// мягкой посадки уже существует: прогноз пассивного участка сходится к
	// мягкому касанию И выпуклая задача из предсказанной точки розжига
	// разрешима.
	if b.plan.Decision != landingCommitted && reachable && b.plan.TerminalOK {
		b.plan.Decision = landingCommitted
		log.Printf("🎯 Курс на башню подтверждён на T+%.1f с: розжиг %.0f м при %.0f м/с, "+
			"группы %v, требуется %.0f м/с и %.0f т, терминальный промах %.0f м, "+
			"время терминального участка %.1f с",
			b.elapsed, b.plan.IgnitionAltitude, b.plan.IgnitionSpeed, b.plan.Schedule,
			b.plan.RequiredDeltaV, b.plan.FuelRequired/1000,
			b.plan.TerminalMiss, b.plan.TerminalTof)
	}
}

// planNearIgnition — подошла ли ступень к розжигу настолько, что дальше
// улучшать нечего и решение пора принимать.
//
// Горизонт — по ВРЕМЕНИ до розжига, а не по высоте: время и есть та
// величина, которой не хватает на увод. landingPlanCommitHorizon —
// характерное время манёвра увода: развернуться, набрать боковую скорость
// и разойтись с площадкой.
func (b *Booster) planNearIgnition(nav orbit.NavState) bool {
	vz, _ := descentComponents(nav)
	if vz <= 0 {
		return false
	}
	return (nav.Altitude-b.plan.IgnitionAltitude)/vz <= landingPlanCommitHorizon
}

// landingPlanCommitHorizon — за сколько секунд до розжига решение о курсе
// на башню или уводе обязано быть принято, с.
const landingPlanCommitHorizon = 45.0

// planAbortReason — потеряна ли достижимость.
func (b *Booster) planAbortReason(nav orbit.NavState, p landingPrediction, reachable bool) landingAbortReason {
	// Камеры: терминальную группу собрать нечем.
	terminal := landingEngineSequence[len(landingEngineSequence)-1]
	if b.phase == BoosterLandingBurn {
		if live := b.propulsion.Commissioned - b.propulsion.EnginesOut; live < terminal {
			return landingAbortEngines
		}
	}

	// Дальше — признаки, которые считает ПРОГНОЗ РОЗЖИГА, и они имеют
	// смысл только до самого розжига. После него лестница ищет высоту
	// события, которое уже произошло, а прогноз описывает импульс от
	// начальной группы камер, тогда как летим мы уже не на ней.
	//
	// Измерено, чего это стоило: на исправной ступени, которую G-FOLD уже
	// вёл с нулевым промахом плана, увод объявлялся на четырёхстах
	// восьмидесяти метрах — по «недостижимости мягкого касания при любой
	// высоте розжига», хотя розжиг был позади, а посадка шла штатно и
	// закончилась касанием 9.7 м/с в ста шестидесяти метрах от площадки.
	if b.phase != BoosterCoast {
		return landingAbortNone
	}

	// Топливо: прогноз просит больше, чем есть с запасом.
	if p.Dry {
		return landingAbortFuel
	}
	if p.Ran && p.FuelUsed*landingPlanFuelMargin > b.propulsion.PropellantMass() {
		return landingAbortFuel
	}

	// Кинематика: мягкое касание недостижимо ни при какой высоте розжига.
	if !reachable {
		return landingAbortUnreachable
	}

	// Неразрешимость ПРОБЫ поводом для увода не является.
	//
	// Проба решает терминальную задачу из ПРЕДСКАЗАННОГО состояния
	// передачи управления, а предсказание грубое: высота передачи зависит
	// от того, где прогноз назначит даунселект, и гуляет от тысячи трёхсот
	// метров до тридцати трёх. Измерено, что проба даёт ложные отрицания:
	// в момент, когда предсказанная точка отстоит от площадки на
	// восемьдесят метров, она объявляет задачу неразрешимой, — а та же
	// задача, решённая из ФАКТИЧЕСКОГО состояния передачи, сходится точно,
	// с нулевым промахом.
	//
	// Цена ошибки была велика: по мнимой неразрешимости объявлялся увод,
	// точка прицеливания уезжала на пять километров, терминальное
	// наведение после этого не могло сойтись ни при каких условиях, и весь
	// спуск вёл резервный закон. Отменять посадку по признаку, который
	// заведомо ошибается в сторону запрета, нельзя.
	//
	// Поэтому проба ПОДТВЕРЖДАЕТ достижимость (см. decideLandingPlan), но
	// её отказ означает лишь «не подтверждено», а не «невозможно».
	// Настоящие поводы для увода — выше: топливо, камеры и кинематическая
	// недостижимость. На самом импульсе к ним добавляется счётчик отказов
	// ФАКТИЧЕСКОГО наведения (landingAbortReason): там задача решается из
	// настоящего состояния, и её отказ уже что-то значит.
	return landingAbortNone
}
