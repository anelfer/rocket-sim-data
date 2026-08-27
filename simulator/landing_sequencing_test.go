package simulator

import (
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Регрессия на посадочную engine-group sequence (13→5→3): раньше группа
// пересчитывалась заново каждый такт из доли оставшейся высоты
// (altitude/landingBurnStartAltitude) — без персистентного состояния и без
// проверки, что переход вообще необходим/достижим. Итог: если посадочный
// импульс на заниженной (MinThrottle-пол) тяге разгонял бустер обратно
// вверх, доля высоты возвращалась к прежним значениям, и все тринадцать
// камер зажигались заново — reboost кормил сам себя.
//
// Тесты здесь бьются на две группы: чистые (без побочных эффектов) проверки
// решающей функции landingGroupShouldDownselect, и интеграционные — на
// реальном Booster, реальном движке (propulsion.Engine.MinThrottle),
// проверяющие, что состояние действительно необратимо.
// -----------------------------------------------------------------------------

// syntheticLandingBooster прогоняет реальную Simulation до момента ignition
// посадочного импульса и возвращает живой *Booster с реально раскрученными,
// реально работающими двигателями — тот же объект, что использует
// production, не изолированный конструктор с нуля.
func syntheticLandingBooster(seed int64) *Booster {
	m, _ := vehicle.MissionByID("transatmospheric")
	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithSeed(m.Apply(p.Config()), DefaultTimeConfig(), seed)
	for i := 0; i < 400000; i++ {
		sim.mu.Lock()
		sim.step(0.1)
		b := sim.booster
		ignited := b != nil && b.Alive() && b.phase == BoosterLandingBurn
		dead := b != nil && !b.Alive()
		sim.mu.Unlock()
		if ignited {
			return b
		}
		if dead {
			return nil
		}
	}
	return nil
}

// landingBoosterOnGroup возвращает booster сразу после ignition посадочного
// импульса (реальные, раскрученные двигатели — см. syntheticLandingBooster
// в предыдущем разборе), принудительно переведённый на заданную группу.
func landingBoosterOnGroup(t *testing.T, seed int64, group int) *Booster {
	t.Helper()
	b := syntheticLandingBooster(seed)
	if b == nil {
		t.Fatal("не удалось получить booster на ignition")
	}
	b.setEngineGroup(group)
	// Один такт пропуска ускорения турбонасоса/клапанов не нужен для этих
	// тестов — landingGroupShouldDownselect работает по Config.MinThrottle
	// (паспортная величина), а не по мгновенному AppliedThrottle.

	// b.attitude.RequestedTorque — снимок РЕАЛЬНОГО момента, запрошенного
	// автопилотом в момент, когда ignition сработал в настоящем прогоне
	// (syntheticLandingBooster), а не что-либо, связанное с синтетическим
	// сценарием, который тест строит ниже (navAt с нулевыми Frame/AirRelative
	// — см. её комментарий). updateLandingEngineGroup использует это поле
	// для одношаговой экстраполяции ПОТРЕБНОГО углового момента (Stage 4.9,
	// п.A2) — оставленное как есть, оно тянет в тест постороннее число,
	// зависящее от того, на какой именно высоте/скорости реальный прогон
	// именно этого зерна решил зажечь посадочный импульс (то есть от физики
	// Boostback/Coast, а не от того, что тест на самом деле проверяет —
	// решение о даунселекте по СВОИМ явно заданным navAt-сценариям).
	// Обнуляем, чтобы тест был самодостаточным и не зависел от того, когда
	// именно реальный прогон зажёг импульс.
	b.attitude.RequestedTorque = physics.Vec3{}

	// Остаток топлива на ignition — тоже посторонняя для этих тестов
	// величина: она определяется тем, сколько разворотный импульс успел
	// сжечь (физика Boostback, не то, что здесь проверяется), и меняется
	// при любой перенастройке Boostback — между разными версиями Boostback
	// в этом же файле колебалась от нескольких сотен тонн до значения,
	// близкого к резерву посадочного импульса. Фиксированное представительное
	// значение делает тест самодостаточным тем же способом, что и обнуление
	// RequestedTorque выше.
	b.state.FuelMass = 400000
	return b
}

// navAt строит минимальный NavState для landingRequiredThrust/
// updateLandingEngineGroup: только высота и скорость снижения — этого
// достаточно, поскольку dragDecel возвращает 0 без DynamicPressure/скорости
// потока, и остальная формула требует только их.
func navAt(altitude, descentSpeed float64) orbit.NavState {
	return orbit.NavState{
		Altitude:       altitude,
		RadialVelocity: -descentSpeed,
	}
}

// TestLandingSequence_A_NominalDownselect — Test A: убывающая по времени
// потребная тяга (типичный ход торможения) обязана провести группу ровно
// 13→5→3, без пропусков и без обратных переходов.
func TestLandingSequence_A_NominalDownselect(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)

	// Скорость снижения, убывающая от "быстро падаем" до "почти у цели",
	// одновременно с расходом топлива (реалистичный ход импульса — масса
	// падает по мере горения, а не остаётся на полной ignition-массе; при
	// полной массе даже 3 двигателя на MinThrottle не смогли бы удержать
	// вес корпуса, и даунселект ниже 5 физически не оправдан — см. Test C).
	const altitude = 1500.0
	seen := []int{b.propulsion.Commissioned}
	descents := []float64{500, 400, 300, 250, 200, 150, 120, 100, 80, 60, 40, 20, 10, 5}
	startFuel := b.state.FuelMass
	for i, descent := range descents {
		b.state.FuelMass = startFuel * (1 - float64(i)/float64(len(descents)))
		nav := navAt(altitude, descent)
		// Несколько тактов на каждой скорости — переход требует dwell.
		for j := 0; j < 20; j++ {
			b.updateLandingEngineGroup(nav, 0.1)
		}
		if len(seen) == 0 || seen[len(seen)-1] != b.propulsion.Commissioned {
			seen = append(seen, b.propulsion.Commissioned)
		}
	}

	want := []int{boosterLandingHigh, boosterLandingMid, boosterLandingLow}
	if len(seen) < 2 {
		t.Fatalf("группа ни разу не уменьшилась за весь профиль: seen=%v", seen)
	}
	// seen обязана быть неубывающей подпоследовательностью [13,5,3] —
	// каждое новое значение обязано быть МЕНЬШЕ предыдущего.
	for i := 1; i < len(seen); i++ {
		if seen[i] >= seen[i-1] {
			t.Errorf("группа не уменьшилась на шаге %d: %v", i, seen)
		}
	}
	for _, s := range seen {
		found := false
		for _, w := range want {
			if s == w {
				found = true
			}
		}
		if !found {
			t.Errorf("группа %d не входит в штатную последовательность %v", s, want)
		}
	}
	t.Logf("последовательность групп: %v", seen)
}

// TestLandingSequence_B_NoReverseAfter13To5 — Test B: после перехода 13→5
// внезапный рост потребной тяги (например, порыв встречного сопротивления)
// не должен вернуть группу на 13 — она обязана остаться на 5, физически
// исчерпав её и оставив дефицит тяги как диагностируемый факт, а не повод
// зажечь заново большую группу.
func TestLandingSequence_B_NoReverseAfter13To5(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)
	const altitude = 1500.0

	// Ведём к 5 небольшой потребной тягой...
	for i := 0; i < 40; i++ {
		b.updateLandingEngineGroup(navAt(altitude, 150), 0.1)
	}
	if b.propulsion.Commissioned != boosterLandingMid {
		t.Fatalf("группа не дошла до 5 перед проверкой: Commissioned=%d", b.propulsion.Commissioned)
	}

	// ...затем резко поднимаем требуемую тягу (огромная скорость снижения).
	for i := 0; i < 40; i++ {
		b.updateLandingEngineGroup(navAt(altitude, 2000), 0.1)
	}
	if b.propulsion.Commissioned != boosterLandingMid {
		t.Errorf("группа откатилась назад при росте required: Commissioned=%d, ожидалось %d (без reverse)",
			b.propulsion.Commissioned, boosterLandingMid)
	}
}

// TestLandingSequence_C_NoReverseAfter5To3 — Test C: то же самое для 5→3.
func TestLandingSequence_C_NoReverseAfter5To3(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingMid)
	const altitude = 200.0

	// К моменту, когда 5→3 вообще оправдан, импульс уже давно горит —
	// на полной ignition-массе вес корпуса превышает даже Tmax(3) (см.
	// TestLandingSequence_ThrustEnvelopeOverlap: Tmax(3)≈7.75 МН против
	// веса, требующего кратно больше при полной массе), и переход
	// физически невозможен. Модельно уменьшаем запас топлива, как это уже
	// произошло бы к этой точке импульса.
	b.state.FuelMass *= 0.05

	for i := 0; i < 40; i++ {
		b.updateLandingEngineGroup(navAt(altitude, 20), 0.1)
	}
	if b.propulsion.Commissioned != boosterLandingLow {
		t.Fatalf("группа не дошла до 3 перед проверкой: Commissioned=%d", b.propulsion.Commissioned)
	}

	for i := 0; i < 40; i++ {
		b.updateLandingEngineGroup(navAt(altitude, 2000), 0.1)
	}
	if b.propulsion.Commissioned != boosterLandingLow {
		t.Errorf("группа откатилась назад при росте required: Commissioned=%d, ожидалось %d (без reverse)",
			b.propulsion.Commissioned, boosterLandingLow)
	}
}

// TestLandingSequence_StoppedEnginesStayStopped — после даунселекта
// выключенные двигатели не должны сами себя перезапускать (см. setEngineGroup
// — раньше он вызывался каждый такт заново из пересчитанной с нуля группы,
// и мог Start() двигатель, который сам же Stop()'нул минуту назад).
func TestLandingSequence_StoppedEnginesStayStopped(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)
	const altitude = 1500.0

	for i := 0; i < 40; i++ {
		b.updateLandingEngineGroup(navAt(altitude, 150), 0.1)
	}
	if b.propulsion.Commissioned != boosterLandingMid {
		t.Fatalf("группа не дошла до 5: Commissioned=%d", b.propulsion.Commissioned)
	}
	// Состав группы 5 — geometry-driven (Stage 4.9, п.A3, см.
	// landingGroupIndices/landingGroupEngineIndices), не обязательно
	// "первые 5 по порядку укладки" — сверяемся с фактически вычисленным
	// для ЭТОГО booster составом, а не с i<boosterLandingMid.
	wantActive := make(map[int]bool, boosterLandingMid)
	for _, i := range b.landingGroupIndices[1] {
		wantActive[i] = true
	}
	if len(wantActive) != boosterLandingMid {
		t.Fatalf("landingGroupIndices[1] содержит %d индексов, ожидалось %d", len(wantActive), boosterLandingMid)
	}
	for i, e := range b.propulsion.Engines {
		want := wantActive[i]
		if e.Running != want {
			t.Errorf("engine %s: Running=%v, ожидалось %v (группа=5, geometry-driven состав)", e.ID, e.Running, want)
		}
	}

	// Требуем огромную тягу — группа НЕ обязана расти, и погашенные камеры
	// обязаны остаться погашенными.
	for i := 0; i < 40; i++ {
		b.updateLandingEngineGroup(navAt(altitude, 2000), 0.1)
	}
	for i, e := range b.propulsion.Engines {
		if i < boosterLandingHigh && !wantActive[i] && e.Running {
			t.Errorf("engine %s (индекс %d, погашен при даунселекте) снова Running", e.ID, i)
		}
	}
}

// TestLandingSequence_D_ShouldDownselectIsPure — Test D/E: детерминированные
// проверки решающей функции на характерных точках (далеко ниже границы,
// точно на границе, далеко выше) — без построения Booster/NavState.
func TestLandingSequence_D_ShouldDownselectPredicate(t *testing.T) {
	cases := []struct {
		name                          string
		required, minCurrent, maxNext float64
		want                          bool
	}{
		{"глубоко ниже обеих границ", 1e6, 5e6, 1e7, true},
		{"выше minCurrent — рано", 6e6, 5e6, 1e7, false},
		{"ровно на minCurrent — не строго меньше, ждём", 5e6, 5e6, 1e7, false},
		{"ниже minCurrent, но выше maxNext — некуда", 4e6, 5e6, 3e6, false},
		{"ровно на maxNext — граница, next едва справляется", 3e6, 5e6, 3e6, true},
		{"required=0 — свободный ход, next легко справится", 0, 5e6, 1e7, true},
	}
	for _, c := range cases {
		got := landingGroupShouldDownselect(c.required, c.minCurrent, c.maxNext)
		if got != c.want {
			t.Errorf("%s: landingGroupShouldDownselect(%.2e,%.2e,%.2e)=%v, ожидалось %v",
				c.name, c.required, c.minCurrent, c.maxNext, got, c.want)
		}
	}
}

// TestLandingSequence_ThrustEnvelopeOverlap — п.19: численно построить
// [Tmin,Tmax] каждой группы на представительной высоте и посмотреть, есть
// ли overlap между соседними группами. Диагностический тест — печатает
// числа и явно отмечает найденный gap, не пытается его скрыть подгонкой.
func TestLandingSequence_ThrustEnvelopeOverlap(t *testing.T) {
	m, _ := vehicle.MissionByID("transatmospheric")
	p, _ := vehicle.ProfileByID("starship")
	cfg := m.Apply(p.Config())

	const altitude = 1500.0 // типичная высота середины импульса
	perEngine := vehicle.ThrustAtAltitude(cfg.FirstStage, physics.Atmosphere(altitude).Pressure)

	// MinThrottle одинаков для всех камер первой ступени (одна конфигурация
	// двигателя) — берём его из реального движка через свежий booster.
	b := syntheticLandingBooster(1)
	if b == nil {
		t.Fatal("не удалось получить booster на ignition")
	}
	minThrottle := b.propulsion.Engines[0].Config.MinThrottle

	type envelope struct {
		group    int
		min, max float64
	}
	envs := []envelope{
		{boosterLandingHigh, perEngine * boosterLandingHigh * minThrottle, perEngine * boosterLandingHigh},
		{boosterLandingMid, perEngine * boosterLandingMid * minThrottle, perEngine * boosterLandingMid},
		{boosterLandingLow, perEngine * boosterLandingLow * minThrottle, perEngine * boosterLandingLow},
	}
	for _, e := range envs {
		t.Logf("group=%d  Tmin=%.2f МН  Tmax=%.2f МН", e.group, e.min/1e6, e.max/1e6)
	}
	for i := 0; i < len(envs)-1; i++ {
		cur, next := envs[i], envs[i+1]
		gap := cur.min - next.max
		if gap > 0 {
			t.Logf("GAP между группами %d и %d: Tmin(%d)=%.2f МН > Tmax(%d)=%.2f МН на %.2f МН (%.0f%%) — "+
				"в этом промежутке НИ ОДНА группа не может дать ровно столько тяги на непрерывном газе",
				cur.group, next.group, cur.group, cur.min/1e6, next.group, next.max/1e6, gap/1e6, 100*gap/cur.min)
		} else {
			t.Logf("overlap между группами %d и %d: %.2f МН — переход можно сделать непрерывным по тяге",
				cur.group, next.group, -gap/1e6)
		}
	}
}
