package physics

import (
	"math"
	"testing"
)

// Плоская панель поперёк потока должна давать ньютоновский коэффициент
// давления, равный двум.
func TestNewtonianFlatPlateCoefficient(t *testing.T) {
	// Панель с нормалью против потока: поток идёт вдоль -X, нормаль +X.
	s := ControlSurface{
		Name: "test", Area: 2,
		Neutral: Vec3{X: 1}, Hinge: Vec3{Y: 1},
		Arm: Vec3{X: 0},
	}

	const q = 5000
	air := Vec3{X: 100} // корпус летит по +X, поток идёт по -X

	f := s.Force(0, air, q, 10)

	// Cp = 2 при нормальном падении, значит сила = 2·q·A.
	want := 2 * q * s.Area
	if math.Abs(f.Norm()-want) > 1 {
		t.Errorf("сила %.0f Н, ожидалось %.0f Н", f.Norm(), want)
	}

	// Поток должен сносить панель по своему направлению, то есть по -X.
	if f.X >= 0 {
		t.Errorf("сила направлена против потока: %+v", f)
	}
}

// Панель, поставленная ребром к потоку, силы не создаёт.
func TestSurfaceEdgeOnGivesNoForce(t *testing.T) {
	s := ControlSurface{
		Name: "test", Area: 2,
		Neutral: Vec3{Z: 1}, Hinge: Vec3{Y: 1},
	}

	f := s.Force(0, Vec3{X: 100}, 5000, 10)
	if f.Norm() > 1 {
		t.Errorf("панель ребром к потоку даёт силу %.1f Н", f.Norm())
	}
}

// Сила должна падать как квадрат синуса угла к потоку.
func TestNewtonianFollowsSineSquared(t *testing.T) {
	s := ControlSurface{
		Name: "test", Area: 1,
		Neutral: Vec3{X: 1}, Hinge: Vec3{Y: 1},
	}

	const q = 1000
	air := Vec3{X: 100}

	for _, angle := range []float64{15, 30, 45, 60} {
		rad := angle * DegToRad
		got := s.Force(rad, air, q, 10).Norm()

		// Отклонение панели на угол уводит её нормаль от потока,
		// и коэффициент падает как квадрат косинуса этого угла.
		want := 2 * q * s.Area * math.Cos(rad) * math.Cos(rad)
		if math.Abs(got-want) > want*0.01 {
			t.Errorf("при отклонении %.0f° сила %.0f Н, ожидалось %.0f Н",
				angle, got, want)
		}
	}
}

// Момент считается относительно центра масс: панель впереди и панель позади
// при одинаковом отклонении крутят корпус в разные стороны.
func TestForwardAndAftSurfacesOpposeEachOther(t *testing.T) {
	fwd := ControlSurface{
		Name: "fwd", Area: 15, Arm: Vec3{X: 20},
		Neutral: Vec3{Z: -1}, Hinge: Vec3{Y: 1},
	}
	aft := ControlSurface{
		Name: "aft", Area: 15, Arm: Vec3{X: -15},
		Neutral: Vec3{Z: -1}, Hinge: Vec3{Y: 1},
	}

	// Корпус летит брюхом вперёд: поток снизу.
	air := Vec3{Z: -2000}

	mFwd := fwd.Torque(0, air, 12000, 20)
	mAft := aft.Torque(0, air, 12000, 20)

	if mFwd.Y*mAft.Y >= 0 {
		t.Errorf("моменты не противоположны: передний %.0f, задний %.0f Н·м",
			mFwd.Y, mAft.Y)
	}
}
