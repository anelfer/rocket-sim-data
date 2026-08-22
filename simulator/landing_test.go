package simulator

import (
	"strings"
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Посадка корабля.
//
// Проверяется не «сел или нет», а то, из чего посадка складывается: камеры
// зажигаются на развороте, корпус приходит к вертикали, тяга ведётся по остатку
// высоты и скорости, лишние камеры гасятся, и касание происходит на скорости,
// которую выдерживает конструкция.
// -----------------------------------------------------------------------------

type landingSummary struct {
	landed     bool
	crashed    bool
	touchdown  float64 // вертикальная скорость касания, м/с
	fuelLeft   float64 // остаток компонентов, кг
	ignition   float64 // высота зажигания посадочных камер, м
	minEngine  int     // наименьшее число работающих камер перед касанием
	maxEngine  int     // наибольшее число камер на торможении
	pointing   float64 // ошибка наведения у земли, градусы
	flapOrder  string  // порядок складывания пар плавников
	flipEngine int     // сколько камер работает на развороте
}

func flyToLanding(t *testing.T, mission string) landingSummary {
	t.Helper()

	m, ok := vehicle.MissionByID(mission)
	if !ok {
		t.Fatalf("нет задания %q", mission)
	}
	p, _ := vehicle.ProfileByID("starship")

	sim := NewSimulationWithSeed(m.Apply(p.Config()), DefaultTimeConfig(), 1)
	out := landingSummary{minEngine: 99}

	requested := false
	for i := 0; i < 80000; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		phase := sim.phase
		alt := sim.state.Altitude()
		vr := sim.state.RadialVelocity()
		crashed := sim.crashed
		fuel := sim.state.FuelMass

		running := 0
		for i := range sim.engines {
			if sim.engines[i].Running {
				running++
			}
		}

		nav := sim.navState()
		pointing := sim.attitude.PointingError(sim.entryTarget(nav), nav.Frame)

		// Порядок складывания пар: сложенной считается пара, выведенная
		// из-под автопилота на нулевой угол.
		stowed := func(prefix string) bool {
			set := sim.attitude.Surfaces
			if set == nil {
				return false
			}
			for i, surface := range set.Surfaces {
				if len(surface.Name) >= 3 && surface.Name[:3] == prefix {
					if !set.Manual[i] || set.Command[i] != 0 {
						return false
					}
				}
			}
			return true
		}
		aftStowed, fwdStowed := stowed("aft"), stowed("fwd")
		sim.mu.Unlock()

		for _, pair := range []struct {
			name string
			done bool
		}{{"aft", aftStowed}, {"fwd", fwdStowed}} {
			if !pair.done || contains(out.flapOrder, pair.name) {
				continue
			}
			if out.flapOrder != "" {
				out.flapOrder += ","
			}
			out.flapOrder += pair.name
		}

		// Орбитальному заданию нужна команда на сход: само оно не вернётся.
		if !requested && phase == orbit.PhaseOrbital {
			sim.RequestEntry()
			requested = true
		}

		if phase == orbit.PhaseLandingFlip && out.ignition == 0 {
			out.ignition = alt
		}
		if phase == orbit.PhaseLandingFlip {
			out.flipEngine = running
		}
		if phase == orbit.PhaseLandingFlip || phase == orbit.PhaseLandingBurn {
			// Число камер считается на торможении: на развороте их
			// намеренно меньше, там нужен момент, а не тяга.
			if phase == orbit.PhaseLandingBurn {
				if running > out.maxEngine {
					out.maxEngine = running
				}
				if running > 0 && running < out.minEngine {
					out.minEngine = running
				}
			}
			if alt < 100 {
				out.pointing = pointing
				out.touchdown = -vr
			}
			out.fuelLeft = fuel
		}

		if phase == orbit.PhaseLanded || crashed {
			out.landed = phase == orbit.PhaseLanded
			out.crashed = crashed
			return out
		}
	}
	t.Fatal("прогон не завершился")
	return out
}

// contains сообщает, встречается ли подстрока.
func contains(s, sub string) bool { return strings.Contains(s, sub) }

func TestShipLandsSoftly(t *testing.T) {
	if testing.Short() {
		t.Skip("длинный прогон")
	}

	s := flyToLanding(t, "transatmospheric")

	t.Logf("посадка: касание %.1f м/с, зажигание на %.0f м, на развороте %d камера, "+
		"на торможении от %d до %d, плавники %s, ошибка наведения у земли %.1f°, "+
		"остаток %.1f т",
		s.touchdown, s.ignition, s.flipEngine, s.minEngine, s.maxEngine,
		s.flapOrder, s.pointing, s.fuelLeft/1000)

	if !s.landed {
		t.Fatal("корабль не дошёл до посадки")
	}
	if s.crashed {
		t.Errorf("корабль разбился: касание %.1f м/с", s.touchdown)
	}
	if s.touchdown > 6 {
		t.Errorf("скорость касания %.1f м/с выше предела конструкции", s.touchdown)
	}
	if s.ignition < 200 {
		t.Errorf("камеры зажглись на %.0f м — на разворот и торможение "+
			"такой высоты не хватает", s.ignition)
	}
	if s.maxEngine > 3 {
		t.Errorf("на посадке работают %d камер: вакуумные у земли включать "+
			"нельзя", s.maxEngine)
	}
	// Тормозит корабль на трёх камерах, касается на двух — так же, как это
	// делает настоящий аппарат. Меньше двух не оставляем: на одной камере
	// нечем держать крен у самой земли.
	if s.maxEngine != 3 {
		t.Errorf("на торможении работают %d камер вместо трёх", s.maxEngine)
	}
	if s.minEngine != 2 {
		t.Errorf("касание на %d камерах вместо двух", s.minEngine)
	}
	if s.pointing > 15 {
		t.Errorf("у земли корпус отклонён от вертикали на %.0f°", s.pointing)
	}
	if s.fuelLeft <= 0 {
		t.Error("компоненты кончились до касания")
	}
	if s.flapOrder != "aft,fwd" {
		t.Errorf("плавники складываются в порядке %q: сначала убирается задняя "+
			"пара, следом передняя", s.flapOrder)
	}
}

// Севший корабль стоит, а не продолжает разворачиваться.
func TestLandedShipStandsUpright(t *testing.T) {
	if testing.Short() {
		t.Skip("длинный прогон")
	}

	m, _ := vehicle.MissionByID("transatmospheric")
	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithSeed(m.Apply(p.Config()), DefaultTimeConfig(), 1)

	landedAt := -1.0
	var tiltAtTouchdown, tiltLater float64

	for i := 0; i < 80000; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		sim.telemetry = sim.buildTelemetryLocked()
		phase := sim.phase
		nav := sim.navState()
		upright := physics.Attitude{Pitch: 90, Yaw: sim.attitude.AttitudeIn(nav.Frame).Yaw}
		tilt := sim.attitude.PointingError(upright, nav.Frame)
		crashed := sim.crashed
		sim.mu.Unlock()

		if phase != orbit.PhaseLanded {
			continue
		}
		if landedAt < 0 {
			landedAt = sim.ModelTime()
			tiltAtTouchdown = tilt
			if crashed {
				t.Fatalf("корабль не сел: разрушен на T+%.0f", sim.ModelTime())
			}
		}
		tiltLater = tilt

		// Полминуты после касания достаточно, чтобы заметить сползание.
		if sim.ModelTime()-landedAt > 30 {
			break
		}
	}

	if landedAt < 0 {
		t.Fatal("корабль не дошёл до посадки")
	}

	t.Logf("отклонение от вертикали: при касании %.1f°, через полминуты %.1f°",
		tiltAtTouchdown, tiltLater)

	if tiltLater > tiltAtTouchdown+2 {
		t.Errorf("корабль заваливается после посадки: %.1f° → %.1f°",
			tiltAtTouchdown, tiltLater)
	}
	if tiltLater > 15 {
		t.Errorf("севший корабль отклонён от вертикали на %.1f°", tiltLater)
	}
}

// Баки обязаны пустеть вместе: иначе к посадке остаётся мёртвый груз.
func TestTanksDrainTogether(t *testing.T) {
	if testing.Short() {
		t.Skip("длинный прогон")
	}

	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithSeed(p.Config(), DefaultTimeConfig(), 1)

	worst := 0.0
	for i := 0; i < 20000; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		fuel := sim.propulsion.FuelTank
		ox := sim.propulsion.OxTank
		stage := sim.stage
		sim.mu.Unlock()

		// Смотрим на вторую ступень: её остаток и есть запас на возвращение.
		if stage < 2 || fuel.FillFraction() > 0.9 {
			continue
		}
		if d := fuel.FillFraction() - ox.FillFraction(); d > worst || -d > worst {
			if d < 0 {
				d = -d
			}
			worst = d
		}
		if fuel.FillFraction() < 0.05 || ox.FillFraction() < 0.05 {
			break
		}
	}

	t.Logf("наибольшее расхождение долей остатка: %.2f процентных пункта",
		worst*100)

	if worst > 0.03 {
		t.Errorf("баки расходятся на %.1f процентных пункта — один компонент "+
			"кончится раньше другого, и остаток второго станет мёртвым грузом",
			worst*100)
	}
}
