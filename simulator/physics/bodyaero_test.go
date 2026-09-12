package physics

import (
	"math"
	"testing"
)

// Геометрия первой ступени Starship: удлинение L/d = 8, теория поперечного
// обтекания применима (работает от L/d ≈ 5).
const (
	testFrontal = 63.6  // мидель, м²
	testSide    = 650.7 // борт (планформа), м²
	testMach    = 2.5
)

// Ось корпуса, отклонённая на offset градусов от tail-first в сторону +Y,
// при потоке вдоль +X. Угол атаки при этом равен 180 − offset.
func axisOffTail(offset float64) Vec3 {
	a := (180 - offset) * DegToRad
	return Vec3{X: math.Cos(a), Y: math.Sin(a)}
}

// На торце, борту и носом вперёд поперечной силы нет: одна из двух
// составляющих обращается в ноль, и сила приложена строго против скорости.
// Это те самые три угла, на которых модель поперечного обтекания и модель
// проекции площади обязаны совпадать.
func TestBodyAeroPureDragAtCardinalAngles(t *testing.T) {
	const q = 20000
	for _, aoa := range []float64{0, 90, 180} {
		a := aoa * DegToRad
		axis := Vec3{X: math.Cos(a), Y: math.Sin(a)}
		f := BodyAeroForce(q, testMach, testFrontal, testSide, Vec3{X: 1}, axis)

		if lateral := math.Hypot(f.Y, f.Z); lateral > 1e-6*f.Norm()+1e-9 {
			t.Errorf("угол атаки %.0f°: поперечная сила %.6f Н при полной %.1f", aoa, lateral, f.Norm())
		}
		if f.X >= 0 {
			t.Errorf("угол атаки %.0f°: сила не тормозит, X = %.1f Н", aoa, f.X)
		}
		want := q * BodyDragArea(testMach, aoa, testFrontal, testSide)
		if math.Abs(f.Norm()-want) > 1e-6*want {
			t.Errorf("угол атаки %.0f°: величина %.3f вместо %.3f Н", aoa, f.Norm(), want)
		}
	}
}

// Осевое сопротивление на чистом tail-first — паспортное: коэффициент
// «двигателями вперёд» по одному миделю, без поперечного члена. На этом
// стоит весь расчёт посадочного импульса.
func TestBodyAeroTailFirstIsAxialOnly(t *testing.T) {
	got := BodyDragArea(testMach, 180, testFrontal, testSide)
	want := EngineFirstDragCoefficient(testMach) * testFrontal
	if math.Abs(got-want) > 1e-9 {
		t.Errorf("tail-first: %.3f м² вместо %.3f", got, want)
	}
}

// Полёт бортом — чистое поперечное обтекание по планформе.
func TestBodyAeroBroadsideIsCrossflowOnly(t *testing.T) {
	got := BodyDragArea(testMach, 90, testFrontal, testSide)
	want := CrossflowDragCoefficient * testSide
	if math.Abs(got-want) > 1e-9 {
		t.Errorf("бортом: %.3f м² вместо %.3f", got, want)
	}
}

// Степени при синусе и косинусе следуют из того, что поперечное обтекание
// гонит ПОПЕРЕЧНЫЙ напор q·sin²α, а осевое — осевой q·cos²α. Проверяется
// прямо: величины двух составляющих обязаны совпасть с этой формулой.
func TestBodyAeroTermsFollowVelocityComponents(t *testing.T) {
	const q = 20000
	for _, offset := range []float64{5, 10, 15, 30, 60} {
		aoa := 180 - offset
		a := aoa * DegToRad
		axis := Vec3{X: math.Cos(a), Y: math.Sin(a)}
		f := BodyAeroForce(q, testMach, testFrontal, testSide, Vec3{X: 1}, axis)

		wantAxial := q * EngineFirstDragCoefficient(testMach) * testFrontal * math.Cos(a) * math.Cos(a)
		wantCross := q * CrossflowDragCoefficient * testSide * math.Sin(a) * math.Sin(a)

		// Осевая составляющая — проекция силы на ось корпуса.
		gotAxial := math.Abs(f.Dot(axis))
		gotCross := f.Sub(axis.Scale(f.Dot(axis))).Norm()

		if math.Abs(gotAxial-wantAxial) > 1e-6*wantAxial {
			t.Errorf("отклонение %.0f°: осевая %.1f вместо %.1f Н", offset, gotAxial, wantAxial)
		}
		if math.Abs(gotCross-wantCross) > 1e-6*wantCross {
			t.Errorf("отклонение %.0f°: нормальная %.1f вместо %.1f Н", offset, gotCross, wantCross)
		}
	}
}

// Знак поперечной силы: корпус, отклонённый носом в сторону +Y, подставляет
// поток борту и получает силу в сторону −Y. На этом знаке строится всякое
// боковое наведение.
func TestBodyAeroPushesOppositeToNoseTilt(t *testing.T) {
	const q = 20000
	// Ниже семи градусов знак меняется: там осевой член, тянущий в сторону
	// наклона, ещё больше поперечного. Проверяем рабочий диапазон возврата.
	for _, offset := range []float64{15, 30, 60, 80} {
		f := BodyAeroForce(q, testMach, testFrontal, testSide, Vec3{X: 1}, axisOffTail(offset))
		if f.Y >= 0 {
			t.Errorf("отклонение %.0f°: поперечная сила %.1f Н идёт туда же, куда отклонён нос", offset, f.Y)
		}
		if f.X >= 0 {
			t.Errorf("отклонение %.0f°: сила перестала тормозить, X = %.1f Н", offset, f.X)
		}
	}
}

// Доли для плоских прогнозов обязаны соответствовать той же силе.
func TestBodyAeroSplitMatchesForce(t *testing.T) {
	const q = 20000
	for aoa := 0.0; aoa <= 180; aoa += 15 {
		axial, lateral := BodyAeroSplit(testMach, aoa, testFrontal, testSide)

		a := aoa * DegToRad
		axis := Vec3{X: math.Cos(a), Y: math.Sin(a)}
		f := BodyAeroForce(q, testMach, testFrontal, testSide, Vec3{X: 1}, axis)
		total := q * BodyDragArea(testMach, aoa, testFrontal, testSide)
		if total <= 0 {
			continue
		}
		if got, want := axial, -f.X/total; math.Abs(got-want) > 1e-9 {
			t.Errorf("угол атаки %.0f°: продольная доля %.6f вместо %.6f", aoa, got, want)
		}
		if got, want := lateral, math.Abs(f.Y)/total; math.Abs(got-want) > 1e-9 {
			t.Errorf("угол атаки %.0f°: поперечная доля %.6f вместо %.6f", aoa, got, want)
		}
	}
	if axial, lateral := BodyAeroSplit(testMach, 180, testFrontal, testSide); math.Abs(axial-1) > 1e-9 || lateral > 1e-9 {
		t.Errorf("tail-first: доли %.6f/%.6f вместо 1/0", axial, lateral)
	}
}

// Рабочий диапазон возврата: на 5…15° от tail-first сопротивление почти не
// отличается от чистого торца, а подъёмная сила мала. Угол атаки здесь —
// орган БОКОВОГО УПРАВЛЕНИЯ, а не аэродинамический тормоз, и числа обязаны
// это подтверждать: если бы отклонение заметно добавляло торможения, им бы
// снова стали тормозить.
func TestBodyAeroSmallOffsetIsNotABrake(t *testing.T) {
	tail := BodyDragArea(testMach, 180, testFrontal, testSide)
	for _, offset := range []float64{5, 10, 15} {
		got := BodyDragArea(testMach, 180-offset, testFrontal, testSide)
		if ratio := got / tail; ratio > 1.1 {
			t.Errorf("отклонение %.0f°: сопротивление выросло в %.2f раза — это уже тормоз, а не руль",
				offset, ratio)
		}
		_, lateral := BodyAeroSplit(testMach, 180-offset, testFrontal, testSide)
		if lateral > 0.3 {
			t.Errorf("отклонение %.0f°: поперечная доля %.2f — слишком велика для малого угла", offset, lateral)
		}
	}
}
