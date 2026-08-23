package propulsion

import (
	"testing"

	"rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Осадка топлива.
//
// Под перегрузкой жидкость прижата к днищу и закрывает заборник; в свободном
// падении её ничто не удерживает. Settled — доля объёма, реально закрывающая
// заборник, и именно через неё невесомость должна сказываться на давлении,
// которое видит насос.
// -----------------------------------------------------------------------------

// testTank возвращает наполовину заправленный бак горючего для тестов.
func testTank() *Tank {
	return NewTank(TankConfig{
		Name:                  "test",
		Volume:                10,
		Height:                4,
		CrossSection:          2.5,
		InitialMass:           4000,
		InitialTemperature:    288,
		TargetPressure:        3.0e5,
		LineResistance:        5,
		PressurantMass:        50,
		PressurantPressure:    2.1e7,
		PressurantTemperature: 200,
	}, RP1())
}

// Под ощутимой перегрузкой топливо прижимается обратно к заборнику.
func TestSettledRelaxesUnderThrust(t *testing.T) {
	tank := testTank()
	tank.Settled = 0

	env := TankEnvironment{AxialAcceleration: 20} // около 2g
	for i := 0; i < 500; i++ {
		tank.updateSettling(0.01, env)
	}

	if tank.Settled < 0.95 {
		t.Errorf("осадка под перегрузкой не восстановилась: %.3f", tank.Settled)
	}
}

// В свободном падении топливо быстро отходит от заборника.
func TestSettledDecaysInFreefall(t *testing.T) {
	tank := testTank() // Settled = 1 по умолчанию (заправка на столе)

	env := TankEnvironment{AxialAcceleration: 0}
	for i := 0; i < 200; i++ {
		tank.updateSettling(0.01, env)
	}

	if tank.Settled > 0.05 {
		t.Errorf("осадка не пропала в невесомости: %.3f", tank.Settled)
	}
}

// Расходный бак осел всегда — в этом и смысл его существования.
func TestHeaderFeedAlwaysSettled(t *testing.T) {
	tank := testTank()
	tank.HeaderFeed = true
	tank.Settled = 0

	tank.updateSettling(0.01, TankEnvironment{AxialAcceleration: 0})

	if tank.Settled != 1 {
		t.Errorf("расходный бак не осел: Settled = %.3f", tank.Settled)
	}
}

// Ручная подмена осадки перекрывает расчёт по перегрузке.
func TestSettledDirectOverride(t *testing.T) {
	tank := testTank()

	env := TankEnvironment{
		AxialAcceleration: 0,
		Overrides:         control.TankOverrides{SettledDirect: control.Set(0.7)},
	}
	tank.updateSettling(0.01, env)

	if tank.Settled != 0.7 {
		t.Errorf("подмена осадки не применилась: Settled = %.3f", tank.Settled)
	}
}

// Неосевшее топливо лишает насос гидростатического напора: заборник видит
// наддув, а не жидкость.
func TestOutletPressureCollapsesWhenUnsettled(t *testing.T) {
	settled := testTank()
	settled.Settled = 1

	unsettled := testTank()
	unsettled.Settled = 0

	const axialAccel = 15.0
	pSettled := settled.OutletPressure(axialAccel)
	pUnsettled := unsettled.OutletPressure(axialAccel)

	if pUnsettled >= pSettled {
		t.Errorf("давление на входе насоса не упало при потере осадки: "+
			"%.0f Па против %.0f Па", pUnsettled, pSettled)
	}

	// Без осадки на входе остаётся чистое давление наддува за вычетом
	// потерь в магистрали — никакого гидростатического столба.
	want := unsettled.Pressure - unsettled.LineDrop
	if want < 0 {
		want = 0
	}
	if diff := pUnsettled - want; diff < -1 || diff > 1 {
		t.Errorf("неосевшее давление %.1f Па не совпало с чистым наддувом %.1f Па",
			pUnsettled, want)
	}
}

// Расходный бак не зависит от осадки — он всегда отдаёт полное давление
// вытеснительной подачи.
func TestHeaderFeedIgnoresSettling(t *testing.T) {
	tank := testTank()
	tank.HeaderFeed = true
	tank.Settled = 0

	if p := tank.OutletPressure(0); p <= 0 {
		t.Errorf("расходный бак не подаёт давление при Settled=0: %.0f Па", p)
	}
}
