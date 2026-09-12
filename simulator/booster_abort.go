package simulator

import (
	"log"
	"math"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Увод от площадки.
//
// Посадка на башню — не единственный допустимый исход возврата, и попытка
// довести её любой ценой хуже отказа от неё. Ступень, которой физически не
// хватает высоты, тяги или топлива, приходит к башне не «почти севшей», а
// неуправляемой — с башней рядом. Настоящие носители в этом случае уводят
// аппарат в воду и садятся туда: точка теряется, площадка цела.
//
// Решение принимается не по одному признаку, а по совокупности состояния —
// достижимости мягкого касания при ЛЮБОЙ высоте розжига, остатку топлива
// против потребного по прогнозу, собираемости группы камер и разрешимости
// задачи терминальным наведением. Ни один из них по отдельности не
// достаточен: прогноз огрублён, отказ камеры может быть единственным, а
// наведение не находит решения и на секунду.
// -----------------------------------------------------------------------------

// landingAbortReason — причина отказа от штатной посадки.
type landingAbortReason int

const (
	// landingAbortNone — посадка идёт штатно.
	landingAbortNone landingAbortReason = iota

	// landingAbortUnreachable — мягкое касание недостижимо: либо прогноз
	// даёт удар при любой высоте розжига, либо уже начатый импульс на
	// полном газе не успевает погасить снижение до земли.
	landingAbortUnreachable

	// landingAbortFuel — топлива меньше, чем требует прогноз посадки.
	landingAbortFuel

	// landingAbortEngines — не собрать даже терминальную группу камер.
	landingAbortEngines

	// landingAbortGuidance — терминальное наведение подряд не находит
	// решения: задача посадки в эту точку неразрешима.
	landingAbortGuidance
)

func (r landingAbortReason) String() string {
	switch r {
	case landingAbortUnreachable:
		return "мягкое касание недостижимо при любой высоте розжига"
	case landingAbortFuel:
		return "топлива меньше потребного по прогнозу"
	case landingAbortEngines:
		return "не собрать терминальную группу камер"
	case landingAbortGuidance:
		return "терминальное наведение не находит решения"
	}
	return "штатно"
}

// landingAbortFuelMargin — во сколько раз остаток топлива должен превышать
// потребный по прогнозу, чтобы посадка считалась обеспеченной. Прогноз не
// видит ни бокового выноса наведения, ни ветра, ни разброса тяги; полтора
// раза — тот запас, на котором эти неучтённые статьи ещё помещаются.
const landingAbortFuelMargin = 1.5

// landingAbortGuidanceFailures — сколько ПОДРЯД неудачных решений
// терминального наведения считать отказом от задачи, а не разовым сбоем.
//
// Считать надо именно подряд идущие: общий счётчик отказов (g.failures)
// растёт за весь полёт и никогда не сбрасывается — по нему увод объявлялся
// бы у любой ступени, у которой наведение хоть раз не сошлось.
const landingAbortGuidanceFailures = 12

// landingAbortDivertRange — на сколько метров от площадки уводится точка
// приводнения. Порядок величины — безопасное удаление от стартового
// комплекса при разрушении ступени о воду.
const landingAbortDivertRange = 5000.0

// landingAbortMinAltitude — ниже этой высоты увод уже не объявляется:
// разворачивать задачу в последние секунды поздно, а начатое торможение к
// башне лучше довести, чем бросить.
const landingAbortMinAltitude = 2000.0

// landingAimPoint — точка, в которую целится терминальное наведение:
// площадка при штатной посадке, точка увода после отказа от неё.
//
// Метрика промаха (landingMiss) сознательно остаётся привязанной к
// площадке: увод — это и есть промах, и прятать его за подменой точки
// отсчёта нельзя.
func (b *Booster) landingAimPoint() physics.Geodetic {
	if b.abortReason != landingAbortNone {
		return b.abortTarget
	}
	// Центр зоны захвата: те же координаты площадки, но НА ВЫСОТЕ ЗАХВАТА.
	//
	// Этой точкой задаётся начало отсчёта выпуклой задачи (см.
	// newLandingFrame), а конечное состояние в ней — начало координат.
	// Поэтому одной высоты здесь достаточно, чтобы терминальная задача
	// целилась в просвет между руками, а не в грунт под башней: ни одно
	// из ограничений задачи не нужно переписывать, сдвигается сама
	// система отсчёта.
	target := b.launchTarget
	target.Altitude += b.tower.CatchHeight
	return target
}

// landingGroundAimPoint — та же точка прицеливания, но на уровне грунта.
//
// Нужна там, где сравнивается БАЛЛИСТИЧЕСКИЙ прогноз падения: он по
// построению заканчивается на поверхности, и мерить его промах до
// приподнятой на три десятка метров цели значило бы добавлять к промаху
// постоянную вертикальную составляющую, которой в горизонтальном
// прицеливании нет.
func (b *Booster) landingGroundAimPoint() physics.Geodetic {
	if b.abortReason != landingAbortNone {
		return b.abortTarget
	}
	return b.launchTarget
}

// updateLandingAbort проверяет, остаётся ли штатная посадка физически
// достижимой, и при необходимости объявляет увод.
func (b *Booster) updateLandingAbort(nav orbit.NavState) {
	if b.abortReason != landingAbortNone {
		return
	}
	if b.phase != BoosterCoast && b.phase != BoosterLandingBurn {
		return
	}
	if nav.Altitude <= landingAbortMinAltitude {
		return
	}
	if vz, _ := descentComponents(nav); vz <= 0 {
		return
	}

	reason := b.abortTick.confirm(b.elapsed, b.landingAbortReason(nav))
	if reason == landingAbortNone {
		return
	}
	b.declareLandingAbort(nav, reason)
}

// landingAbortConfirmTime — сколько секунд признак увода обязан держаться
// НЕПРЕРЫВНО, прежде чем увод объявлен, с.
//
// Подтверждение появилось не для мягкости, а потому что решение
// необратимо, а признак считается прогнозом, у которого есть участки, где
// он ничего не значит. Сразу за апогеем снижение ещё единицы метров в
// секунду: прогноз пассивного участка экстраполирует восемьдесят
// километров падения по скорости, которой ещё нет, лестница поиска розжига
// не находит ни одной высоты, расход выходит любым. Замерено: на одном
// зерне из шести увод объявлялся на 82 км при снижении 10 м/с и остатке
// 182 т — после чего розжиг уезжал на 29 км, а промах за полтора
// километра.
//
// Пять секунд — это два-три пересчёта плана (landingPlanInterval) и сотня
// тактов проверки: достаточно, чтобы переходный признак успел пропасть, и
// ничтожно против того, что остаётся до розжига.
const landingAbortConfirmTime = 5.0

// abortConfirm — выдержка одной причины увода по модельному времени.
//
// У каждого источника своя: признаки считают разные проверки с разной
// частотой (потактовая landingAbortReason и посекундная planAbortReason),
// и общая выдержка означала бы, что несогласие одного источника сбрасывает
// отсчёт другому — тогда устойчивый признак не дозрел бы никогда.
type abortConfirm struct {
	reason landingAbortReason
	since  float64
}

// confirm пропускает причину увода, только если она держится непрерывно
// достаточно долго. Потеря камер подтверждения не требует: это не прогноз,
// а свершившийся факт.
func (c *abortConfirm) confirm(now float64, reason landingAbortReason) landingAbortReason {
	if reason == landingAbortNone {
		c.reason = landingAbortNone
		return landingAbortNone
	}
	if reason == landingAbortEngines {
		return reason
	}
	if reason != c.reason {
		c.reason = reason
		c.since = now
		return landingAbortNone
	}
	if now-c.since < landingAbortConfirmTime {
		return landingAbortNone
	}
	return reason
}

// landingAbortReason возвращает первую сработавшую причину увода.
func (b *Booster) landingAbortReason(nav orbit.NavState) landingAbortReason {
	// Камеры. Терминальная группа — последняя в посадочной
	// последовательности; если живых камер меньше, гасить снижение у самой
	// воды будет нечем.
	terminal := landingEngineSequence[len(landingEngineSequence)-1]
	if b.phase == BoosterLandingBurn {
		if live := b.propulsion.Commissioned - b.propulsion.EnginesOut; live < terminal {
			return landingAbortEngines
		}
	}

	// Наведение. Отказы считаются только там, где G-FOLD и ведёт полёт, —
	// на терминальной группе камер (gfoldApplicable). Раньше по этому
	// участку: до розжига и на больших группах терминальная задача
	// неразрешима ПО ПОСТРОЕНИЮ, её решения никто и не ждёт, а счётчик
	// отказов при этом рос — и увод объявлялся ещё на пассивном участке,
	// у полностью исправной ступени. Измерено: увод в первую же секунду
	// посадочного импульса, точка перенесена на пять километров, ступень
	// весь спуск летела к ней и пришла к воде с горизонтальной скоростью
	// в сотню метров в секунду.
	if b.gfoldApplicable(nav) &&
		b.gfold.consecutiveFailures >= landingAbortGuidanceFailures {
		return landingAbortGuidance
	}

	// Кинематика уже начатого импульса. Даунселект необратим: если ТЕКУЩАЯ
	// группа на полном газе больше не успевает погасить снижение до земли,
	// добавить камер уже нельзя, и посадка на башню перестала быть
	// достижимой — независимо от того, что показывает прогноз розжига
	// (он считает импульс от группы 13, а летим мы уже не на ней).
	if b.phase == BoosterLandingBurn && b.propulsion.Commissioned > 0 {
		_, aMax := b.groupAccelEnvelope(nav, b.propulsion.Commissioned)
		if b.landingStoppingMargin(nav, aMax) < 0 {
			return landingAbortUnreachable
		}
	}

	// Дальше — признаки, которые считает прогноз розжига. Он описывает
	// импульс от НАЧАЛЬНОЙ группы камер (landingIgnitionGroup) и потому
	// имеет смысл только до розжига: после него группа уже другая, и тот
	// же прогноз описывал бы не тот полёт, который идёт.
	if b.phase != BoosterCoast {
		return landingAbortNone
	}

	ignition, feasible := b.landingBurnIgnitionAltitude(nav)
	if !feasible {
		return landingAbortUnreachable
	}

	// Топливо. Прогноз считает расход посадочного импульса от той самой
	// высоты розжига, которую сам же и назначил. Пока она не определена
	// (нулевая — ступень ещё не снижается), считать нечего.
	if ignition <= 0 {
		return landingAbortNone
	}
	p := b.predictLanding(nav, ignition, b.coastAngleOfAttack(nav), b.ignitionAlignDelay(nav))
	if p.Dry {
		return landingAbortFuel
	}
	if p.Ran && p.FuelUsed*landingAbortFuelMargin > b.propulsion.PropellantMass() {
		return landingAbortFuel
	}
	return landingAbortNone
}

// declareLandingAbort переносит точку посадки в воду и фиксирует причину.
//
// Направление увода — от площадки в сторону, откуда ступень идёт, то есть
// «не долететь». Это и самый дешёвый по топливу увод (гасить горизонтальную
// скорость приходится меньше, а не больше), и самый безопасный: траектория
// после отказа от посадки не проходит над стартовым комплексом.
func (b *Booster) declareLandingAbort(nav orbit.NavState, reason landingAbortReason) {
	b.abortReason = reason
	b.abortAt = b.elapsed

	pad := physics.GeodeticToECEF(b.launchTarget)

	// Точка увода — та, куда ступень и так летит, а не назначенная в
	// стороне. Гнаться за точкой после того, как погоня объявлена
	// невозможной, — противоречие: именно эта погоня и съедает тягу,
	// которой не хватает на вертикаль. Увод означает «принять, где
	// окажусь», и единственное требование к этому месту — быть не ближе
	// разрешённого удаления от площадки.
	here := physics.ECIToECEF(nav.Position, b.elapsed)
	radius := b.Config.Diameter / 2
	if impact, ft, ok := predictImpact(nav.Position, nav.Velocity,
		b.dryMass()+b.state.FuelMass, math.Pi*radius*radius,
		b.Config.FirstStageLength*b.Config.Diameter,
		b.coastAngleOfAttack(nav)); ok {
		here = physics.ECIToECEF(impact, b.elapsed+ft)
	}

	up := pad.Unit()
	away := here.Sub(pad)
	away = away.Sub(up.Scale(away.Dot(up)))
	if away.Norm() < 1 {
		// Ступень ровно над площадкой — уводить некуда по направлению
		// подхода; уводим по восточной оси местного горизонта.
		away = nav.Frame.East
	}
	// Разрешённая зона — не ближе landingAbortDivertRange от площадки.
	// Если ступень уже дальше, ближайшая точка зоны — прямо под ней, и
	// тянуть её куда-то ещё незачем; если ближе — надо отойти наружу.
	// Обратное (Min вместо Max) означало бы увод К площадке, то есть
	// ровно противоположное уводу.
	divert := pad.Add(away.Unit().Scale(math.Max(away.Norm(), landingAbortDivertRange)))

	b.abortTarget = physics.ECEFToGeodetic(divert)
	b.abortTarget.Altitude = b.launchTarget.Altitude

	b.gfold.valid = false

	log.Printf("🛟 Бустер уходит от площадки на T+%.1f с: %s "+
		"(высота %.0f м, снижение %.0f м/с, остаток %.1f т) — "+
		"точка приводнения перенесена на %.0f м от площадки",
		b.elapsed, reason, nav.Altitude, -nav.RadialVelocity,
		b.propulsion.PropellantMass()/1000,
		math.Min(away.Norm(), landingAbortDivertRange))
}
