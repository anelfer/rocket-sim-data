package simulator

import (
	"math"
	"strconv"
	"testing"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Инварианты ориентации на возврате.
//
// Проверяется не форма кривой и не конкретные числа прогона: они законно
// меняются от любой правки наведения. Проверяются свойства, нарушение
// которых означает неисправность независимо от того, какой получилась
// траектория:
//
//   - разворот идёт в пределах угловой скорости, которую корпус способен
//     отыграть, и не держит приводы в насыщении дольше нескольких секунд;
//   - к концу разворота угловая скорость сходится к малой;
//   - на пассивном участке недостижимая цель НЕ превращается в монотонно
//     растущую угловую скорость: орган ориентации обязан гасить вращение,
//     а не разгонять его;
//   - вырождение углов Эйлера у вертикали не создаёт фиктивных оборотов
//     в накопленном крене.
//
// Числа в порогах — не «как сейчас получилось», а границы физического
// смысла: 25 °/с это вдвое больше потолка опоры (boosterMaxReferenceRate),
// то есть запас на переброс; 5 секунд насыщения — время, за которое
// приводы обязаны выбрать долг.
// -----------------------------------------------------------------------------

// attitudeRun прогоняет возврат и собирает по нему сводку.
type attitudeRun struct {
	boostbackPeakRate  float64 // °/с
	boostbackExitRate  float64 // °/с
	boostbackSatStreak float64 // с, самая длинная непрерывная полоса насыщения
	coastStartRate     float64 // °/с
	coastVacuumMaxRate float64 // °/с, пока напор меньше coastVacuumQ
	coastVacuumExit    float64 // °/с, на границе появления аэродинамики
	coastTrackingMax   float64 // °, отставание корпуса от опоры в вакууме
	coastTrackingExit  float64 // °, отставание на той же границе
	rollIntegral       float64 // °, накопленный физический крен
	rollFromRates      float64 // °, он же, посчитанный независимо
	sawBoostback       bool
	sawCoast           bool
}

// attitudeSeeds — зёрна, на которых проверяются инварианты. Одного мало:
// ветер, разбросы и шум приборов задаются зерном, и свойство, которое
// держится ровно на одном прогоне, — это не свойство управления, а
// совпадение.
var attitudeSeeds = []int64{1, 2, 3}

func seedName(seed int64) string { return "зерно " + strconv.FormatInt(seed, 10) }

// coastVacuumQ — граница «аэродинамики ещё нет», Па. Ниже неё моменты
// потока на три порядка меньше располагаемого момента газоотвода, и всё,
// что происходит с вращением, — дело рук управления, а не воздуха.
const coastVacuumQ = 50.0

func runBoosterAttitude(t *testing.T, seed int64) attitudeRun {
	t.Helper()

	p, ok := vehicle.ProfileByID("starship")
	if !ok {
		t.Fatal("нет профиля starship")
	}
	cfg := p.Config()
	if m, ok := vehicle.MissionByID("transatmospheric"); ok {
		cfg = m.Apply(cfg)
	}
	sim := NewSimulationWithSeed(cfg, DefaultTimeConfig(), seed)

	var out attitudeRun
	var satStreak float64
	const dt = 0.1

	for step := 0; step < 6000; step++ {
		sim.step(dt)
		b := sim.detachedBooster()
		if b == nil {
			continue
		}

		rate := b.attitude.Omega.Norm() * physics.RadToDeg
		out.rollFromRates += b.attitude.Omega.X * physics.RadToDeg * dt
		out.rollIntegral = b.rollIntegral

		switch b.phase {
		case BoosterBoostback:
			out.sawBoostback = true
			if rate > out.boostbackPeakRate {
				out.boostbackPeakRate = rate
			}
			out.boostbackExitRate = rate
			if b.attitude.Saturated {
				satStreak += dt
				if satStreak > out.boostbackSatStreak {
					out.boostbackSatStreak = satStreak
				}
			} else {
				satStreak = 0
			}

		case BoosterCoast:
			if !out.sawCoast {
				out.sawCoast = true
				out.coastStartRate = rate
			}
			q := physics.Atmosphere(b.state.Altitude()).Density *
				0.5 * b.state.Velocity.Norm() * b.state.Velocity.Norm()
			if q < coastVacuumQ {
				if rate > out.coastVacuumMaxRate {
					out.coastVacuumMaxRate = rate
				}
				e := b.referenceTrackingError(b.coastCommandedDir) * physics.RadToDeg
				if e > out.coastTrackingMax {
					out.coastTrackingMax = e
				}
				// Последние значения перед тем, как поток вступит в дело:
				// именно с ними ступень войдёт в плотные слои.
				out.coastVacuumExit, out.coastTrackingExit = rate, e
			}
		}

		if b.phase == BoosterDestroyed || b.phase == BoosterCaught ||
			b.phase == BoosterSplashdown {
			break
		}
	}

	if !out.sawBoostback || !out.sawCoast {
		t.Fatal("прогон не дошёл до возврата бустера")
	}
	return out
}

// Разворот ведётся в пределах, которые корпус способен отыграть.
func TestBoostbackStaysWithinTurnAuthority(t *testing.T) {
	for _, seed := range attitudeSeeds {
		t.Run(seedName(seed), func(t *testing.T) { boostbackAuthority(t, seed) })
	}
}

func boostbackAuthority(t *testing.T, seed int64) {
	r := runBoosterAttitude(t, seed)

	// Порог абсолютный и намеренно НЕ выводится из boosterMaxReferenceRate:
	// тест обязан падать и тогда, когда потолок опоры подняли. 30 °/с —
	// вдвое больше нынешнего потолка и меньше 38.8 °/с, которые корпус
	// набирал, когда опору ничто не ограничивало.
	const peakLimit = 30.0
	if r.boostbackPeakRate > peakLimit {
		t.Errorf("на развороте угловая скорость дошла до %.1f °/с при пределе %.1f °/с",
			r.boostbackPeakRate, peakLimit)
	}
	if r.boostbackSatStreak > 5 {
		t.Errorf("приводы держались в насыщении %.1f с подряд — опора уезжает быстрее, чем контур её отрабатывает",
			r.boostbackSatStreak)
	}
	if r.boostbackExitRate > 3 {
		t.Errorf("разворот закончился с угловой скоростью %.2f °/с — корпус не остановлен на цели",
			r.boostbackExitRate)
	}
}

// На пассивном участке в вакууме управление обязано УДЕРЖИВАТЬ корпус, а не
// раскручивать его. Недостижимая угловая ошибка — не повод набирать
// угловую скорость: погасить её потом будет нечем.
func TestCoastDoesNotSpinUpOnUnreachableTarget(t *testing.T) {
	for _, seed := range attitudeSeeds {
		t.Run(seedName(seed), func(t *testing.T) { coastHoldsAttitude(t, seed) })
	}
}

func coastHoldsAttitude(t *testing.T, seed int64) {
	r := runBoosterAttitude(t, seed)

	if r.coastVacuumMaxRate > 8 {
		t.Errorf("в вакууме на пассивном участке |ω| дошла до %.2f °/с (в начале участка %.2f °/с)",
			r.coastVacuumMaxRate, r.coastStartRate)
	}
	if r.coastTrackingMax > 45 {
		t.Errorf("корпус отстал от опорного направления на %.0f° — опора ведёт туда, куда контур не идёт",
			r.coastTrackingMax)
	}

	// Главная проверка — состояние на границе, за которой начинает
	// работать поток: в плотные слои ступень обязана входить успокоенной
	// и там, куда её вело наведение, а не с накопленной за минуту
	// угловой скоростью и стоградусным отставанием.
	if r.coastVacuumExit > 3 {
		t.Errorf("к появлению аэродинамики |ω| = %.2f °/с — вход в плотные слои с накопленным вращением",
			r.coastVacuumExit)
	}
	if r.coastTrackingExit > 20 {
		t.Errorf("к появлению аэродинамики корпус отстаёт от цели на %.0f°", r.coastTrackingExit)
	}
}

// Накопленный крен — физическая величина, а не сумма скачков вырождения:
// он обязан совпасть с независимо посчитанным интегралом угловой скорости.
func TestRollIntegralHasNoPhantomTurns(t *testing.T) {
	for _, seed := range attitudeSeeds {
		t.Run(seedName(seed), func(t *testing.T) { rollIntegralIsPhysical(t, seed) })
	}
}

func rollIntegralIsPhysical(t *testing.T, seed int64) {
	r := runBoosterAttitude(t, seed)

	if off := math.Abs(r.rollIntegral - r.rollFromRates); off > 1 {
		t.Errorf("накопленный крен %.1f° против независимого интеграла %.1f°",
			r.rollIntegral, r.rollFromRates)
	}
}
