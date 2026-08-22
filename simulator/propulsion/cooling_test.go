package propulsion

import (
	"math"
	"math/rand"
	"testing"

	ctrl "rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Теплообмен в камере: сторона газа и сторона охладителя.
//
// Обе стороны считаются по критериальным зависимостям, а не задаются числом,
// поэтому и проверять их надо по порядкам величин, известным для настоящих
// двигателей: поток в стенку, температура огневой стенки, скорость охладителя
// в канале и его нагрев в тракте.
// -----------------------------------------------------------------------------

// engineAt возвращает двигатель, выведенный на установившийся режим.
func engineAt(cfg EngineConfig, in EngineInput, seconds float64) *Engine {
	e := NewEngine(cfg, rand.New(rand.NewSource(1)), rand.New(rand.NewSource(2)), 288)
	e.Detailed = true
	e.Start()

	for t := 0.0; t < seconds; t += 0.05 {
		e.Update(0.05, in)
	}
	return e
}

// methaloxInput возвращает условия работы метанового двигателя.
func methaloxInput() EngineInput {
	fuel, ox := LiquidMethane(), LOX()

	in := nominalInput()
	in.FuelDensity = fuel.Density(107)
	in.OxDensity = ox.Density(85)
	in.FuelTemperature = 107
	in.OxTemperature = 85
	in.FuelVaporPressure = fuel.VaporPressure(107)
	in.OxVaporPressure = ox.VaporPressure(85)
	in.FuelInletPressure = 350000
	in.OxInletPressure = 380000
	return in
}

// Тепловой режим обоих двигателей должен укладываться в известные пределы.
func TestCoolingMatchesRealEngines(t *testing.T) {
	cases := []struct {
		name string
		cfg  EngineConfig
		in   EngineInput

		// Пределы взяты по опубликованным данным для двигателей этого
		// класса: керосиновый на десяти мегапаскалях и метановый
		// на тридцати пяти.
		fluxLo, fluxHi float64 // поток в горловине, МВт/м²
		wallLo, wallHi float64 // температура огневой стенки, К
		speedLo        float64 // скорость охладителя в горловине, м/с
		speedHi        float64
	}{
		{
			name: "Merlin", cfg: Merlin1DConfig(), in: nominalInput(),
			fluxLo: 20, fluxHi: 50,
			wallLo: 650, wallHi: 950,
			speedLo: 30, speedHi: 130,
		},
		{
			// Метановый двигатель на 350 бар — самый теплонапряжённый
			// из существующих: поток в горловине больше сотни мегаватт
			// на квадратный метр, и снимает его сверхкритический метан
			// на высокой скорости.
			name: "Raptor", cfg: RaptorConfig(), in: methaloxInput(),
			fluxLo: 80, fluxHi: 170,
			wallLo: 650, wallHi: 1000,
			speedLo: 80, speedHi: 350,
		},
	}

	for _, c := range cases {
		e := engineAt(c.cfg, c.in, 20)
		nz := e.Nozzle

		var throat *WallStation
		for i := range nz.Stations {
			if nz.Stations[i].Name == "throat" {
				throat = &nz.Stations[i]
			}
		}
		if throat == nil {
			t.Fatalf("%s: участок горловины не найден", c.name)
		}

		flux := throat.HeatFlux / 1e6
		if flux < c.fluxLo || flux > c.fluxHi {
			t.Errorf("%s: поток в горловине %.0f МВт/м² вне ожидаемых %.0f…%.0f",
				c.name, flux, c.fluxLo, c.fluxHi)
		}
		if throat.Temperature < c.wallLo || throat.Temperature > c.wallHi {
			t.Errorf("%s: огневая стенка %.0f К вне ожидаемых %.0f…%.0f",
				c.name, throat.Temperature, c.wallLo, c.wallHi)
		}
		if throat.Margin() <= 0 {
			t.Errorf("%s: на номинале стенка за пределом материала, запас %.0f К",
				c.name, throat.Margin())
		}

		// Скорость охладителя в канале горловины.
		area, hydraulic := nz.channelGeometry(throat)
		density := c.in.FuelDensity
		speed := nz.CoolingFlow / (density * area)

		if speed < c.speedLo || speed > c.speedHi {
			t.Errorf("%s: скорость охладителя в горловине %.0f м/с вне ожидаемых %.0f…%.0f",
				c.name, speed, c.speedLo, c.speedHi)
		}

		// Нагрев охладителя в тракте: слишком малый означает, что тепло
		// уходит в никуда, слишком большой — что компонент вскипит.
		rise := nz.CoolantOutletTemp - nz.CoolantInletTemp
		if rise < 20 || rise > 400 {
			t.Errorf("%s: охладитель нагрелся на %.0f К — вне разумного", c.name, rise)
		}

		reynolds := nz.CoolingFlow / area * hydraulic / c.cfg.Pair.Fuel.Viscosity
		if reynolds < 1e4 {
			t.Errorf("%s: течение в канале не турбулентное, Re = %.0f — "+
				"зависимость Дитуса–Бёльтера к нему неприменима", c.name, reynolds)
		}

		t.Logf("%-7s поток %5.1f МВт/м², стенка %4.0f К (запас %+4.0f), "+
			"охладитель %.0f м/с, Re %.1e, нагрев %.0f К",
			c.name, flux, throat.Temperature, throat.Margin(), speed, reynolds, rise)
	}
}

// Показатель 0.8 в соотношении Бартца — это зависимость от числа Рейнольдса,
// а связь с давлением возникает из запирания горловины. Значит, удвоение
// давления в камере обязано поднимать теплоотдачу ровно в 2^0.8 раза.
func TestBartzScalesWithPressureToThePowerOfPointEight(t *testing.T) {
	e := newNominalEngine()
	diameter := e.Nozzle.ThroatDiameter()

	// Характеристическая скорость от давления не зависит, поэтому массовый
	// поток через горловину меняется ровно как давление.
	e.Chamber.CharacteristicVelocity = 1800

	e.Chamber.MeanPressure = 1.0e7
	low := e.Chamber.GasSideCoefficient(diameter, ctrl.EngineOverrides{}.Chamber)

	e.Chamber.MeanPressure = 2.0e7
	high := e.Chamber.GasSideCoefficient(diameter, ctrl.EngineOverrides{}.Chamber)

	got := high / low
	want := math.Pow(2, 0.8)

	if math.Abs(got-want) > 0.01 {
		t.Errorf("удвоение давления изменило теплоотдачу в %.3f раза, ожидалось %.3f",
			got, want)
	}
	t.Logf("h(20 МПа)/h(10 МПа) = %.3f, 2^0.8 = %.3f", got, want)
}
