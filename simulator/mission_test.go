package simulator

import (
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// Трансатмосферный полёт: корабль не выходит на орбиту и входит сам.
func TestTransatmosphericReturnsWithoutBurn(t *testing.T) {
	m, _ := vehicle.MissionByID("transatmospheric")
	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithSeed(m.Apply(p.Config()), DefaultTimeConfig(), 1)

	var seco, peak, entryAlt float64
	var secoPeri, secoSpeed, secoFuel, interfaceFuel float64
	deployed := false

	for i := 0; i < 60000; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		phase := sim.phase
		alt := sim.state.Altitude()
		el := physics.ElementsFromState(sim.state.Position, sim.state.Velocity, sim.elapsed)
		set := sim.attitude.Surfaces
		crashed := sim.crashed
		sim.mu.Unlock()

		if seco == 0 && phase == orbit.PhaseEntry {
			seco = sim.ModelTime()
			secoPeri = el.PeriapsisAltitude
			secoSpeed = sim.state.Velocity.Norm()
			secoFuel = sim.state.FuelMass
		}
		// Запас на подходе к плотным слоям. Между выключением двигателей
		// и входом корабль не должен потратить ни килограмма: возвращает
		// его атмосфера, а не тяга.
		if seco > 0 && interfaceFuel == 0 && alt < 100000 &&
			sim.state.RadialVelocity() < 0 {

			interfaceFuel = sim.state.FuelMass
		}
		if alt > peak {
			peak = alt
		}
		if set != nil && set.Deployed && !deployed {
			deployed = true
			entryAlt = alt
		}
		if phase == orbit.PhaseLandingBurn || phase == orbit.PhaseLanded || crashed {
			t.Logf("прогон окончен на T+%.0f с: %s, высота %.1f км, разрушен %v",
				sim.ModelTime(), phase, alt/1000, crashed)
			break
		}
	}

	t.Logf("SECO T+%.0f с: перицентр %.0f км, скорость %.0f м/с; "+
		"верхняя точка %.0f км; плавники на %.0f км",
		seco, secoPeri/1000, secoSpeed, peak/1000, entryAlt/1000)

	if seco == 0 {
		t.Fatal("корабль не перешёл к возвращению после выключения двигателей")
	}
	// Перицентр обязан лежать в плотных слоях: только тогда траектория
	// сама приводит корабль в атмосферу.
	if secoPeri > 60000 {
		t.Errorf("перицентр %.0f км — такая траектория не входит в атмосферу "+
			"сама, это орбита", secoPeri/1000)
	}
	if interfaceFuel == 0 {
		t.Fatal("корабль не дошёл до плотных слоёв")
	}
	// Пять тонн — это запас на удержание ориентации двигателями малой тяги
	// на получасовом спуске. Тормозной импульс стоил бы десятков тонн,
	// и такой расход эта проверка поймает.
	if spent := secoFuel - interfaceFuel; spent > 5000 {
		t.Errorf("между выключением двигателей и входом истрачено %.1f т — "+
			"трансатмосферной траектории тормозной импульс не нужен", spent/1000)
	}
	if secoSpeed < 7000 {
		t.Errorf("скорость выключения %.0f м/с — до трансатмосферной "+
			"траектории разгон не доведён", secoSpeed)
	}
	if peak < 150000 || peak > 280000 {
		t.Errorf("верхняя точка %.0f км вне задания (200 км)", peak/1000)
	}
	if !deployed {
		t.Error("плавники так и не раскрылись")
	}
}
