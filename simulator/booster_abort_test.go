package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/env"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// boosterInCoast прогоняет миссию до пассивного участка возврата и отдаёт
// бустер вместе с его навигационным состоянием.
func boosterInCoast(t *testing.T, seed int64) (*Booster, orbit.NavState) {
	t.Helper()

	m, ok := vehicle.MissionByID("transatmospheric")
	if !ok {
		t.Fatal("нет миссии transatmospheric")
	}
	p, ok := vehicle.ProfileByID("starship")
	if !ok {
		t.Fatal("нет профиля starship")
	}
	sim := NewSimulationWithSeed(m.Apply(p.Config()), DefaultTimeConfig(), seed)

	for i := 0; i < 40000; i++ {
		sim.mu.Lock()
		sim.step(0.05)
		b := sim.booster
		// Именно на СНИЖЕНИИ: на восходящей ветви пассивного участка
		// прогноз посадки по построению ничего не значит.
		reached := false
		if b != nil && b.phase == BoosterCoast {
			nav := orbit.NewNavState(b.state.Position, b.state.Velocity,
				b.windVelocity(env.CalmWind(), b.state.Altitude()),
				b.dryMass()+b.state.FuelMass, b.propulsion.TotalThrust,
				cfgVacuumISP(b.Config), b.elapsed, orbit.PhaseEntry)
			vz, _ := descentComponents(nav)
			reached = vz > 100
		}
		sim.mu.Unlock()
		if reached {
			break
		}
	}

	sim.mu.Lock()
	defer sim.mu.Unlock()
	b := sim.booster
	if b == nil || b.phase != BoosterCoast {
		t.Fatalf("бустер не дошёл до снижения на пассивном участке: %v", b)
	}
	nav := orbit.NewNavState(b.state.Position, b.state.Velocity,
		b.windVelocity(env.CalmWind(), b.state.Altitude()),
		b.dryMass()+b.state.FuelMass, b.propulsion.TotalThrust,
		cfgVacuumISP(b.Config), b.elapsed, orbit.PhaseEntry)
	return b, nav
}

// Исправная ступень на пассивном участке уводиться не должна: увод — ответ
// на недостижимость посадки, а не на сам факт возврата.
func TestHealthyBoosterDoesNotAbort(t *testing.T) {
	b, nav := boosterInCoast(t, 1)
	if reason := b.landingAbortReason(nav); reason != landingAbortNone {
		t.Fatalf("увод у исправной ступени: %v", reason)
	}
	// Без увода целимся в ту же площадку — но НЕ в грунт под ней, а в
	// центр зоны захвата над стартовым столом (см. landingAimPoint).
	got := b.landingAimPoint()
	if got.Latitude != b.launchTarget.Latitude || got.Longitude != b.launchTarget.Longitude {
		t.Fatalf("точка прицеливания сдвинута без увода: %+v", got)
	}
	if want := b.launchTarget.Altitude + b.tower.CatchHeight; got.Altitude != want {
		t.Fatalf("цель не на высоте захвата: %.3f м вместо %.3f", got.Altitude, want)
	}
	if ground := b.landingGroundAimPoint(); ground != b.launchTarget {
		t.Fatalf("наземная точка прицеливания сдвинута без увода: %+v", ground)
	}
}

// Пустые баки — увод: посадочный импульс нечем вести.
func TestEmptyTanksForceAbort(t *testing.T) {
	b, nav := boosterInCoast(t, 1)

	b.propulsion.FuelTank.Mass = 0
	b.propulsion.OxTank.Mass = 0
	b.state.FuelMass = 0
	b.ignitionAltValid = false // состояние изменилось — кеш прогноза устарел

	if reason := b.landingAbortReason(nav); reason == landingAbortNone {
		t.Fatal("пустые баки не привели к уводу")
	}
}

// Увод переносит точку приводнения НЕ БЛИЖЕ разрешённого удаления от
// площадки — в этом весь его смысл.
func TestAbortDivertsAwayFromPad(t *testing.T) {
	b, nav := boosterInCoast(t, 1)

	b.declareLandingAbort(nav, landingAbortUnreachable)

	if b.abortReason != landingAbortUnreachable {
		t.Fatalf("причина увода не сохранена: %v", b.abortReason)
	}

	pad := physics.GeodeticToECEF(b.launchTarget)
	aim := physics.GeodeticToECEF(b.landingAimPoint())
	up := pad.Unit()
	off := aim.Sub(pad)
	horizontal := off.Sub(up.Scale(off.Dot(up))).Norm()

	if horizontal < landingAbortDivertRange-1 {
		t.Fatalf("увод на %.0f м, требуется не ближе %.0f м",
			horizontal, landingAbortDivertRange)
	}
	if math.Abs(b.landingAimPoint().Altitude-b.launchTarget.Altitude) > 1 {
		t.Fatalf("высота точки увода %.1f, у площадки %.1f",
			b.landingAimPoint().Altitude, b.launchTarget.Altitude)
	}
}

// Объявленный увод не отменяется и не пересматривается: решение
// однократное.
func TestAbortIsIrreversible(t *testing.T) {
	b, nav := boosterInCoast(t, 1)

	b.declareLandingAbort(nav, landingAbortFuel)
	target := b.abortTarget

	b.updateLandingAbort(nav)

	if b.abortReason != landingAbortFuel || b.abortTarget != target {
		t.Fatalf("увод пересмотрен: %v %+v", b.abortReason, b.abortTarget)
	}
}

// Увод объявляется только по устойчивому признаку, а не по одному
// срабатыванию. Решение необратимо, а признак считается прогнозом, который
// сразу за апогеем ничего не значит: одиночный отказ там стоил площадки.
func TestAbortNeedsConfirmation(t *testing.T) {
	b, _ := boosterInCoast(t, 1)
	start := b.elapsed

	// Признак держится, но время ещё не вышло — увода нет.
	for b.elapsed < start+landingAbortConfirmTime {
		if r := b.abortTick.confirm(b.elapsed, landingAbortFuel); r != landingAbortNone {
			t.Fatalf("увод объявлен через %.1f с из %.1f",
				b.elapsed-start, landingAbortConfirmTime)
		}
		b.elapsed += 0.05
	}
	if r := b.abortTick.confirm(b.elapsed, landingAbortFuel); r != landingAbortFuel {
		t.Errorf("устойчивый признак не привёл к уводу: %v", r)
	}

	// Пропавший признак сбрасывает отсчёт: иначе набежали бы разрозненные
	// отказы за весь пассивный участок.
	b2, _ := boosterInCoast(t, 1)
	for i := 0; i < 20; i++ {
		for j := 0; j < 20; j++ {
			if r := b2.abortTick.confirm(b2.elapsed, landingAbortFuel); r != landingAbortNone {
				t.Fatalf("увод объявлен на чередующемся признаке (шаг %d)", i)
			}
			b2.elapsed += 0.05
		}
		if r := b2.abortTick.confirm(b2.elapsed, landingAbortNone); r != landingAbortNone {
			t.Fatal("исправный прогноз объявил увод")
		}
	}

	// Потеря камер подтверждения не требует: это не прогноз, а факт.
	b3, _ := boosterInCoast(t, 1)
	if r := b3.abortTick.confirm(b3.elapsed, landingAbortEngines); r != landingAbortEngines {
		t.Errorf("потеря камер отложена на подтверждение: %v", r)
	}
}
