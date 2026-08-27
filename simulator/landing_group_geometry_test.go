package simulator

import (
	"math"
	"sort"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Регрессия на состав посадочных групп по физической нумерации (Stage 4.10,
// п.2, наследует Stage 4.9 п.A3).
//
// Раньше группа посадочного импульса — это просто engines[0:n]: первые n
// маневренных камер по порядку укладки (placeRings, layout.go). Для n=5 это
// давало 3 центральных двигателя плюс 2 ФИЗИЧЕСКИ СОСЕДНИЕ камеры среднего
// кольца — Stage 4.8 audit (перебор всех C(13,5)=1287 пятёрок) нашёл, что
// такой набор занимает ранг 859/1287 по min(pitch,yaw) authority и даёт
// заметный паразитный момент при нулевом отклонении камер (постоянная
// "трим-компенсация", съедающая часть располагаемого хода ещё до начала
// реальной коррекции). Stage 4.9 закрыла это geometry-search'ем; Stage 4.10
// заменила search на явную найденную схему физических номеров
// (landingGroup5EngineNumbers={1,2,3,6,12} и т.д., booster.go) — здесь
// проверяется, что landingGroupIndices действительно даёт РОВНО эти номера,
// что последовательность 13→5→3 остаётся вложенной (меньшая группа —
// подмножество большей, иначе даунселект гасил бы уже работающие камеры и
// разжигал вместо них другие, остывшие с самого начала импульса), и что
// найденная пятёрка симметрична там, где старая (index-driven) — нет.
// -----------------------------------------------------------------------------

// landingGroupSymmetry возвращает (symmetry, zeroGimbalMoment) для набора
// камер idx при геометрии/authority данного booster'а — тот же расчёт, что
// bestSymmetricSubset использует при выборе, применённый здесь независимо
// для проверки результата.
func landingGroupSymmetry(t *testing.T, b *Booster, idx []int) (symmetry, zeroMoment float64) {
	t.Helper()
	arm := b.gimbalArm()
	maxGimbal := b.attitude.Config.MaxGimbal
	sub := make([]EngineActuator, len(idx))
	for k, i := range idx {
		e := b.engineLayout[i]
		sub[k] = EngineActuator{ID: e.ID, RadialX: e.X, RadialY: e.Y, Gimbal: e.Gimbal, Thrust: 1}
	}
	forceScale := float64(len(sub))
	torqueScale := forceScale * arm * math.Sin(maxGimbal)
	probe := func(axis physics.Vec3) float64 {
		gimbal := allocateEngineGimbal(sub, arm, physics.Vec3{}, axis.Scale(1e12), forceScale, torqueScale, maxGimbal)
		_, achieved := engineForceTorque(sub, gimbal, arm)
		return math.Abs(achieved.Dot(axis.Unit()))
	}
	pitch, yaw := probe(physics.Vec3{Y: 1}), probe(physics.Vec3{Z: 1})
	if maxAuth := math.Max(pitch, yaw); maxAuth > 0 {
		symmetry = math.Min(pitch, yaw) / maxAuth
	}
	zeroGimbal := make([]EngineGimbalState, len(sub))
	_, zt := engineForceTorque(sub, zeroGimbal, arm)
	return symmetry, zt.Norm()
}

// TestLandingGroupIndices_Group5IsSymmetricNotIndexDriven — главная
// регрессия Stage 4.9 п.A3: группа 5 больше не engines[0:5].
func TestLandingGroupIndices_Group5IsSymmetricNotIndexDriven(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)

	seqIdx := -1
	for i, n := range landingEngineSequence {
		if n == boosterLandingMid {
			seqIdx = i
		}
	}
	if seqIdx < 0 {
		t.Fatal("boosterLandingMid отсутствует в landingEngineSequence")
	}
	group5 := b.landingGroupIndices[seqIdx]
	if len(group5) != boosterLandingMid {
		t.Fatalf("landingGroupIndices[%d] содержит %d камер, ожидалось %d", seqIdx, len(group5), boosterLandingMid)
	}

	oldFirstFive := []int{0, 1, 2, 3, 4}
	same := len(group5) == len(oldFirstFive)
	if same {
		seen := make(map[int]bool, len(group5))
		for _, i := range group5 {
			seen[i] = true
		}
		for _, i := range oldFirstFive {
			if !seen[i] {
				same = false
				break
			}
		}
	}
	if same {
		t.Fatalf("landingGroupIndices для группы 5 совпал со старым index-driven engines[0:5]=%v — "+
			"geometry-driven выбор должен отличаться на реальной 3+10 раскладке", oldFirstFive)
	}

	symmetry, zeroMoment := landingGroupSymmetry(t, b, group5)
	if symmetry < 0.99 {
		t.Errorf("группа 5 (%v) недостаточно симметрична: min/max(pitch,yaw)=%.4f, ожидалось >=0.99", group5, symmetry)
	}
	// zeroMoment здесь — паразитный момент на единичной тяге каждого
	// двигателя (Thrust=1 в landingGroupSymmetry/bestSymmetricSubset), не
	// в ньютонах реальной тяги — порог сравнивается с authority того же
	// масштаба (пример из Stage 4.8/4.9 диагностики: authority ~14, старый
	// набор давал момент ~3.8 на этом масштабе).
	if zeroMoment > 1e-6 {
		t.Errorf("группа 5 (%v) даёт ненулевой паразитный момент при нулевом gimbal: %.3e, ожидалось ~0", group5, zeroMoment)
	}
}

// TestLandingGroupIndices_NestedSequence — 13→5→3 обязана быть вложенной:
// каждая следующая (по убыванию размера) группа — подмножество предыдущей,
// иначе даунселект гасил бы уже работающие камеры новой группы и разжигал
// вместо них другие, остывшие с самого начала посадочного импульса.
func TestLandingGroupIndices_NestedSequence(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)

	for i := 0; i < len(landingEngineSequence)-1; i++ {
		bigger := b.landingGroupIndices[i]
		smaller := b.landingGroupIndices[i+1]
		if len(smaller) >= len(bigger) {
			t.Fatalf("группа %d (%v) не меньше группы %d (%v)",
				landingEngineSequence[i+1], smaller, landingEngineSequence[i], bigger)
		}
		biggerSet := make(map[int]bool, len(bigger))
		for _, idx := range bigger {
			biggerSet[idx] = true
		}
		for _, idx := range smaller {
			if !biggerSet[idx] {
				t.Errorf("камера с индексом %d есть в группе %d, но отсутствует в большей группе %d — "+
					"последовательность не вложена, даунселект перезажёг бы остывшую камеру",
					idx, landingEngineSequence[i+1], landingEngineSequence[i])
			}
		}
	}
}

// TestLandingGroupIndices_MatchFoundScheme — Stage 4.10, п.2: подтверждает
// буквально, что landingGroupIndices соответствует найденной схеме —
// Landing13={1..13}, Landing5={1,2,3,6,12}, Landing3={1,2,3} — по
// ФИЗИЧЕСКИМ номерам (EngineNumber), а не по индексам массива, и заодно
// печатает численное сравнение с прежней (index-driven) неправильной
// пятёркой engines[0:5] (Stage 4.8/4.9 п.1 отчёта: authority/симметрия/
// паразитный момент до и после).
func TestLandingGroupIndices_MatchFoundScheme(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)

	numbersOf := func(idx []int) []int {
		nums := make([]int, len(idx))
		for k, i := range idx {
			nums[k] = b.engineLayout[i].EngineNumber
		}
		sort.Ints(nums)
		return nums
	}

	cases := []struct {
		name string
		n    int
		want []int
	}{
		{"Landing13", boosterLandingHigh, []int{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}},
		{"Landing5", boosterLandingMid, []int{1, 2, 3, 6, 12}},
		{"Landing3", boosterLandingLow, []int{1, 2, 3}},
	}
	for _, c := range cases {
		seqIdx := -1
		for i, n := range landingEngineSequence {
			if n == c.n {
				seqIdx = i
			}
		}
		if seqIdx < 0 {
			t.Fatalf("%s: размер %d отсутствует в landingEngineSequence", c.name, c.n)
		}
		got := numbersOf(b.landingGroupIndices[seqIdx])
		if len(got) != len(c.want) {
			t.Fatalf("%s: получено %v, ожидалось %v", c.name, got, c.want)
		}
		for i := range got {
			if got[i] != c.want[i] {
				t.Errorf("%s: получено %v, ожидалось %v", c.name, got, c.want)
				break
			}
		}
	}

	// Численное сравнение с прежней неправильной пятёркой (engines[0:5] —
	// те же ФИЗИЧЕСКИЕ позиции, что и до Stage 4.9/4.10, номера у них
	// сейчас другие из-за перестановки, но позиции те же самые пять камер).
	oldFive := []int{0, 1, 2, 3, 4}
	newFive := b.landingGroupIndices[1]
	oldSym, oldZero := landingGroupSymmetry(t, b, oldFive)
	newSym, newZero := landingGroupSymmetry(t, b, newFive)
	t.Logf("пятёрка: старая (index-driven, физические номера %v) symmetry=%.4f zeroMoment=%.3e",
		numbersOf(oldFive), oldSym, oldZero)
	t.Logf("пятёрка: новая ({1,2,3,6,12}) symmetry=%.4f zeroMoment=%.3e", newSym, newZero)
	if newSym <= oldSym {
		t.Errorf("новая пятёрка не симметричнее старой: %.4f <= %.4f", newSym, oldSym)
	}
	if newZero >= oldZero {
		t.Errorf("новая пятёрка не даёт меньший паразитный момент: %.3e >= %.3e", newZero, oldZero)
	}
}

// TestLandingGroupIndices_Group3StaysSymmetric — группа 3 (терминальная,
// последняя перед касанием) обязана остаться идеально симметричной, как и
// раньше (центральное кольцо, 120° друг от друга) — Stage 4.9 не должна
// была задеть то, что и так уже было хорошо, гоняясь за фиксом группы 5.
func TestLandingGroupIndices_Group3StaysSymmetric(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)

	seqIdx := -1
	for i, n := range landingEngineSequence {
		if n == boosterLandingLow {
			seqIdx = i
		}
	}
	if seqIdx < 0 {
		t.Fatal("boosterLandingLow отсутствует в landingEngineSequence")
	}
	group3 := b.landingGroupIndices[seqIdx]
	symmetry, zeroMoment := landingGroupSymmetry(t, b, group3)
	if symmetry < 0.999 {
		t.Errorf("группа 3 (%v) потеряла симметрию: min/max(pitch,yaw)=%.4f, ожидалось >=0.999", group3, symmetry)
	}
	if zeroMoment > 1e-6 {
		t.Errorf("группа 3 (%v) даёт ненулевой паразитный момент: %.3e, ожидалось ~0", group3, zeroMoment)
	}
}
