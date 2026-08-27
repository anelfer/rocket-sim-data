package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Stage 4.5, п.1: аудит карты двигателей первой ступени — до production
// правок TVC. Подтверждает, что маневренная группа (Gimbal:true) физически
// совпадает с "13 maneuverable engines" посадочной последовательности
// (boosterLandingHigh), а не с историческими тремя центральными.
// -----------------------------------------------------------------------------

func firstStageLayoutForTest(t *testing.T) vehicle.StageLayout {
	t.Helper()
	m, ok := vehicle.MissionByID("transatmospheric")
	if !ok {
		t.Fatal("MissionByID: не найдена миссия transatmospheric")
	}
	p, ok := vehicle.ProfileByID("starship")
	if !ok {
		t.Fatal("ProfileByID: не найден профиль starship")
	}
	layout := vehicle.NewLayout(m.Apply(p.Config()))
	for _, stage := range layout.Stages {
		if stage.Index == 1 {
			return stage
		}
	}
	t.Fatal("первая ступень не найдена в Layout.Stages")
	return vehicle.StageLayout{}
}

// TestEngineLayout_ThirtyThreeEngines — базовая карта: 33 камеры, кольца
// [3,10,20] (центр/среднее/наружное). ID — не сквозной порядок укладки
// (Stage 4.10, п.1: физическая нумерация внутри маневренного кольца
// переставлена, см. vehicle.innerRingEngineNumberBySlot), а
// "S1-"+EngineNumber — стабильный физический номер, источник истины.
func TestEngineLayout_ThirtyThreeEngines(t *testing.T) {
	stage := firstStageLayoutForTest(t)
	if len(stage.Engines) != 33 {
		t.Fatalf("ожидалось 33 камеры, получено %d", len(stage.Engines))
	}
	ringCounts := map[int]int{}
	seenNumbers := make(map[int]bool, 33)
	for i, e := range stage.Engines {
		wantID := "S1-" + itoa(e.EngineNumber)
		if e.ID != wantID {
			t.Errorf("engine[%d].ID=%q, ожидалось %q (по EngineNumber=%d)", i, e.ID, wantID, e.EngineNumber)
		}
		if e.EngineNumber < 1 || e.EngineNumber > 33 {
			t.Errorf("engine[%d].EngineNumber=%d вне диапазона 1..33", i, e.EngineNumber)
		}
		if seenNumbers[e.EngineNumber] {
			t.Errorf("EngineNumber=%d встречается дважды", e.EngineNumber)
		}
		seenNumbers[e.EngineNumber] = true
		ringCounts[e.Ring]++
	}
	if ringCounts[0] != 3 || ringCounts[1] != 10 || ringCounts[2] != 20 {
		t.Errorf("кольца не [3,10,20]: получено %v", ringCounts)
	}
}

// TestEngineLayout_ManeuverableCountMatchesLanding — п.1 Stage 4.5: число
// физически маневренных камер (Gimbal:true в карте) обязано совпадать с
// числом камер, которые посадочная последовательность зажигает на фазе
// "13" (boosterLandingHigh) — обе цифры описывают один и тот же реальный
// факт (13 maneuverable engines V3), и они не должны разойтись молча.
func TestEngineLayout_ManeuverableCountMatchesLanding(t *testing.T) {
	stage := firstStageLayoutForTest(t)

	gimbalCount := 0
	for _, e := range stage.Engines {
		if e.Gimbal {
			gimbalCount++
		}
	}
	if gimbalCount != boosterLandingHigh {
		t.Errorf("маневренных камер по карте: %d, а посадочная группа 13 (boosterLandingHigh=%d) — расходится",
			gimbalCount, boosterLandingHigh)
	}

	// Маневренные камеры обязаны быть ПЕРВЫМИ по индексу (центр+среднее
	// кольцо идут раньше наружного в сквозной нумерации placeRings) — это
	// как раз то, на чём молча полагается setEngineGroup(n): Start()'ит
	// индексы < n. Явная проверка вместо неявного допущения.
	for i, e := range stage.Engines {
		want := i < boosterLandingHigh
		if e.Gimbal != want {
			t.Errorf("engine[%d] (%s): Gimbal=%v, ожидалось %v (маневренные — первые %d по индексу)",
				i, e.ID, e.Gimbal, want, boosterLandingHigh)
		}
	}
}

// TestEngineLayout_PhysicalNumberingMatchesGeometry — Stage 4.10, п.1:
// найденная схема (1-3 центр, 4-13 маневренное кольцо, 14-33 закреплённое
// наружное) действительно соответствует геометрии (Ring, Gimbal), а не
// только диапазонам номеров как таковым.
func TestEngineLayout_PhysicalNumberingMatchesGeometry(t *testing.T) {
	stage := firstStageLayoutForTest(t)
	byNumber := make(map[int]vehicle.EngineLayout, len(stage.Engines))
	for _, e := range stage.Engines {
		byNumber[e.EngineNumber] = e
	}

	for n := 1; n <= 3; n++ {
		e, ok := byNumber[n]
		if !ok {
			t.Fatalf("камера с номером %d не найдена", n)
		}
		if e.Ring != 0 {
			t.Errorf("камера №%d: Ring=%d, ожидалось 0 (центральное кольцо)", n, e.Ring)
		}
		if !e.Gimbal {
			t.Errorf("камера №%d: ожидался маневренный (Gimbal=true)", n)
		}
	}
	for n := 4; n <= 13; n++ {
		e, ok := byNumber[n]
		if !ok {
			t.Fatalf("камера с номером %d не найдена", n)
		}
		if e.Ring != 1 {
			t.Errorf("камера №%d: Ring=%d, ожидалось 1 (маневренное кольцо)", n, e.Ring)
		}
		if !e.Gimbal {
			t.Errorf("камера №%d: ожидался маневренный (Gimbal=true)", n)
		}
	}
	for n := 14; n <= 33; n++ {
		e, ok := byNumber[n]
		if !ok {
			t.Fatalf("камера с номером %d не найдена", n)
		}
		if e.Ring != 2 {
			t.Errorf("камера №%d: Ring=%d, ожидалось 2 (наружное кольцо)", n, e.Ring)
		}
		if e.Gimbal {
			t.Errorf("камера №%d: ожидалась закреплённая (Gimbal=false)", n)
		}
	}
}

// TestEngineLayout_Engines6And12AreDiametricallyOpposite — Stage 4.10,
// п.1-2: Landing5={1,2,3,6,12} опирается на то, что камеры №6 и №12
// внутреннего кольца стоят РОВНО друг напротив друга (180°) — без этого
// пятёрка не была бы симметричной. Проверяется напрямую по геометрии
// (X,Y), а не по номеру: перестановка номеров (innerRingEngineNumberBySlot)
// обязана давать именно такой результат, это не должно молча разойтись.
func TestEngineLayout_Engines6And12AreDiametricallyOpposite(t *testing.T) {
	stage := firstStageLayoutForTest(t)
	var e6, e12 *vehicle.EngineLayout
	for i := range stage.Engines {
		switch stage.Engines[i].EngineNumber {
		case 6:
			e6 = &stage.Engines[i]
		case 12:
			e12 = &stage.Engines[i]
		}
	}
	if e6 == nil || e12 == nil {
		t.Fatal("камеры №6 и/или №12 не найдены")
	}
	const tol = 1e-6
	if math.Abs(e6.X+e12.X) > tol || math.Abs(e6.Y+e12.Y) > tol {
		t.Errorf("камеры №6 (%.4f,%.4f) и №12 (%.4f,%.4f) не диаметрально противоположны "+
			"(ожидалось X6≈-X12, Y6≈-Y12)", e6.X, e6.Y, e12.X, e12.Y)
	}
	r6 := math.Hypot(e6.X, e6.Y)
	r12 := math.Hypot(e12.X, e12.Y)
	if math.Abs(r6-r12) > tol {
		t.Errorf("камеры №6 и №12 на разных радиусах: %.4f против %.4f", r6, r12)
	}
}

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	neg := n < 0
	if neg {
		n = -n
	}
	var buf [20]byte
	i := len(buf)
	for n > 0 {
		i--
		buf[i] = byte('0' + n%10)
		n /= 10
	}
	if neg {
		i--
		buf[i] = '-'
	}
	return string(buf[i:])
}
