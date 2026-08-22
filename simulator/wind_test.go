package simulator

import (
	"math"
	"math/rand"
	"testing"

	"rocketTelemetrySim/simulator/env"
	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Ветровой профиль и его влияние на скорость относительно воздуха.
// -----------------------------------------------------------------------------

func TestWindProfileShape(t *testing.T) {
	w := env.NewWindProfile(rand.New(rand.NewSource(42)))

	speedAt := func(alt float64) float64 {
		pos := physics.Vec3{X: physics.EarthRadius + alt}
		return w.Velocity(pos, alt, 0).Norm()
	}

	surface := speedAt(10)
	jet := speedAt(w.JetStreamAltitude)
	high := speedAt(55000)

	if jet <= surface {
		t.Errorf("струйное течение (%.1f м/с) должно быть сильнее приземного ветра (%.1f м/с)",
			jet, surface)
	}
	if high >= jet {
		t.Errorf("на 55 км ветер (%.1f м/с) должен быть слабее струйного течения (%.1f м/с)",
			high, jet)
	}
	if speedAt(80000) != 0 {
		t.Error("выше границы модели ветра быть не должно")
	}

	// Ветер горизонтален: вертикальной составляющей быть не должно.
	pos := physics.Vec3{X: physics.EarthRadius + 11000}
	frame := physics.NewLocalFrame(pos)
	v := w.Velocity(pos, 11000, 0)
	if math.Abs(v.Dot(frame.Up)) > 1e-9 {
		t.Errorf("вертикальная составляющая ветра = %.6f м/с", v.Dot(frame.Up))
	}
}

// Ветер обязан влиять на скорость относительно потока, а значит на напор
// и угол атаки. Без этого он был бы декоративным.

func TestWindAffectsAirRelativeVelocity(t *testing.T) {
	pos := physics.Vec3{X: physics.EarthRadius + 11000}
	state := VehicleState{
		Position: pos,
		Velocity: physics.CorotatingVelocity(pos).Add(physics.Vec3{Y: 300}),
	}

	wind := physics.Vec3{Y: 40}

	noWind := state.AirRelativeVelocity().Norm()
	withWind := state.AirRelativeVelocityWithWind(wind).Norm()

	if math.Abs(noWind-withWind) < 1 {
		t.Errorf("ветер не изменил скорость относительно потока: %.2f и %.2f",
			noWind, withWind)
	}
	approx(t, withWind, math.Abs(300-40), 1e-9, "скорость относительно воздуха")
}

// -----------------------------------------------------------------------------
// Угловая динамика отработавшей ступени
// -----------------------------------------------------------------------------

// В вакууме момента нет, поэтому момент импульса обязан сохраняться.
