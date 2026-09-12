package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// recordShort — короткая запись для проверок структуры: полный прогон здесь
// не нужен, а время сборки тестов дорого.
func recordShort(t *testing.T, seed int64) *TrajectoryRun {
	t.Helper()
	m, _ := vehicle.MissionByID("transatmospheric")
	p, _ := vehicle.ProfileByID("starship")
	opt := DefaultTrajectoryOptions()
	opt.MaxDuration = 200
	return RecordTrajectory(m.Apply(p.Config()), seed, opt)
}

// Одно зерно — одна и та же запись. На этом держится смысл выбора прогона
// на странице: оператор, вернувшись к тому же зерну, обязан увидеть ту же
// траекторию, а не «похожую».
func TestTrajectory_SameSeedGivesSameRun(t *testing.T) {
	a := recordShort(t, 7)
	b := recordShort(t, 7)

	if len(a.Ship) != len(b.Ship) || len(a.Booster) != len(b.Booster) {
		t.Fatalf("разное число точек: корабль %d/%d, бустер %d/%d",
			len(a.Ship), len(b.Ship), len(a.Booster), len(b.Booster))
	}
	for i := range a.Ship {
		if a.Ship[i] != b.Ship[i] {
			t.Fatalf("точка %d разошлась: %+v против %+v", i, a.Ship[i], b.Ship[i])
		}
	}
	if len(a.Events) != len(b.Events) {
		t.Fatalf("разное число событий: %d против %d", len(a.Events), len(b.Events))
	}
}

// Разные зёрна обязаны давать разные траектории — иначе выбор прогона
// ничего не выбирает.
func TestTrajectory_DifferentSeedsDiffer(t *testing.T) {
	a := recordShort(t, 1)
	b := recordShort(t, 2)
	if len(a.Ship) == 0 || len(b.Ship) == 0 {
		t.Fatal("пустая траектория корабля")
	}
	same := true
	n := len(a.Ship)
	if len(b.Ship) < n {
		n = len(b.Ship)
	}
	for i := 0; i < n; i++ {
		if a.Ship[i] != b.Ship[i] {
			same = false
			break
		}
	}
	if same {
		t.Fatal("разные зёрна дали идентичную траекторию")
	}
}

// Координаты обязаны быть согласованы сами с собой: ECEF-радиус точки
// должен отвечать её же записанной высоте. Это ловит любую путаницу систем
// координат — ровно ту ошибку, из-за которой трасса поехала бы относительно
// площадки.
func TestTrajectory_CoordinatesMatchAltitude(t *testing.T) {
	run := recordShort(t, 3)
	check := func(name string, pts []TrajectoryPoint) {
		for i, p := range pts {
			r := math.Sqrt(p.X*p.X + p.Y*p.Y + p.Z*p.Z)
			want := physics.EarthRadius + p.Altitude
			if math.Abs(r-want) > 1 {
				t.Fatalf("%s, точка %d: |ECEF|=%.1f, а высота даёт %.1f", name, i, r, want)
			}
		}
	}
	check("корабль", run.Ship)
	check("бустер", run.Booster)
}

// Точка старта записи обязана совпадать со стартовым комплексом: трасса
// начинается там, где стоит ракета, а не «примерно там».
func TestTrajectory_StartsAtPad(t *testing.T) {
	run := recordShort(t, 4)
	if len(run.Ship) == 0 {
		t.Fatal("пустая траектория корабля")
	}
	first := run.Ship[0]
	d := greatCircleDistance(run.Pad.Latitude, run.Pad.Longitude, first.Latitude, first.Longitude)
	if d > 2000 {
		t.Fatalf("первая точка в %.0f м от площадки", d)
	}
	if first.Altitude > 1000 {
		t.Fatalf("первая точка на высоте %.0f м", first.Altitude)
	}
}

// События обязаны указывать на существующие точки своего тела и идти по
// возрастанию времени: метка, привязанная к чужой или несуществующей точке,
// нарисуется не там, где произошла.
func TestTrajectory_EventsReferenceRealPoints(t *testing.T) {
	run := recordShort(t, 5)
	if len(run.Events) == 0 {
		t.Fatal("ни одного события за двести секунд полёта")
	}
	prev := -1.0
	for _, e := range run.Events {
		var pts []TrajectoryPoint
		switch e.Body {
		case "ship":
			pts = run.Ship
		case "booster":
			pts = run.Booster
		default:
			t.Fatalf("неизвестное тело события: %q", e.Body)
		}
		if e.Index < 0 || e.Index >= len(pts) {
			t.Fatalf("событие %q ссылается на точку %d, а их %d", e.Kind, e.Index, len(pts))
		}
		if e.Time < prev {
			t.Fatalf("события не упорядочены по времени: %.2f после %.2f", e.Time, prev)
		}
		prev = e.Time
	}
}

// Отделение и разворотный импульс обязаны быть отмечены — это опорные
// события возврата, ради которых страница и делалась.
func TestTrajectory_MarksSeparationAndBoostback(t *testing.T) {
	run := recordShort(t, 6)
	kinds := map[string]bool{}
	for _, e := range run.Events {
		kinds[e.Kind] = true
	}
	for _, want := range []string{"liftoff", "separation", "boostback-start"} {
		if !kinds[want] {
			t.Errorf("не отмечено событие %q (есть: %v)", want, kinds)
		}
	}
}

// -----------------------------------------------------------------------------
// Запись ИДУЩЕГО прогона (trajectory_live.go).
// -----------------------------------------------------------------------------

// liveAfter прогоняет живую симуляцию до заданного модельного времени и
// возвращает снимок её записанной траектории.
func liveAfter(t *testing.T, seconds float64) (*Simulation, *TrajectoryRun) {
	t.Helper()
	m, _ := vehicle.MissionByID("transatmospheric")
	p, _ := vehicle.ProfileByID("starship")
	sim := NewSimulationWithSeed(m.Apply(p.Config()), DefaultTimeConfig(), 5)
	for sim.elapsed < seconds {
		sim.mu.Lock()
		sim.step(0.05)
		sim.mu.Unlock()
	}
	return sim, sim.LiveTrajectory()
}

// Каждое событие живой записи обязано называть своё тело. Пустое имя тела —
// не мелочь: страница по нему выбирает, на какой из двух трасс рисовать
// метку, и событие корабля ушло бы на трассу бустера.
func TestLiveTrajectory_EventsNameTheirBody(t *testing.T) {
	_, run := liveAfter(t, 130)
	if len(run.Events) == 0 {
		t.Fatal("ни одного события за сто тридцать секунд полёта")
	}
	for _, e := range run.Events {
		if e.Body != "ship" && e.Body != "booster" {
			t.Fatalf("событие %q (T+%.1f) не называет тело: %q", e.Kind, e.Time, e.Body)
		}
		pts := run.Ship
		if e.Body == "booster" {
			pts = run.Booster
		}
		if e.Index < 0 || e.Index >= len(pts) {
			t.Fatalf("событие %q тела %q ссылается на точку %d, а их %d",
				e.Kind, e.Body, e.Index, len(pts))
		}
	}
}

// Живая запись обязана расти по ходу полёта и начинаться со старта.
func TestLiveTrajectory_GrowsWithFlight(t *testing.T) {
	sim, early := liveAfter(t, 30)
	if len(early.Ship) == 0 {
		t.Fatal("за тридцать секунд не записано ни одной точки")
	}
	for sim.elapsed < 90 {
		sim.mu.Lock()
		sim.step(0.05)
		sim.mu.Unlock()
	}
	later := sim.LiveTrajectory()
	if len(later.Ship) <= len(early.Ship) {
		t.Fatalf("трасса не выросла: было %d точек, стало %d", len(early.Ship), len(later.Ship))
	}
	// Уже записанные точки не должны переписываться: трасса дописывается
	// в конец, а не пересобирается.
	for i := range early.Ship {
		if early.Ship[i] != later.Ship[i] {
			t.Fatalf("точка %d изменилась задним числом", i)
		}
	}
	if later.Seed != sim.Seed() {
		t.Fatalf("зерно в записи (%d) не совпадает с зерном прогона (%d)", later.Seed, sim.Seed())
	}
}

// Снимок обязан быть копией: продолжение полёта не имеет права менять уже
// отданные наружу данные.
func TestLiveTrajectory_SnapshotIsIndependent(t *testing.T) {
	sim, snap := liveAfter(t, 40)
	n := len(snap.Ship)
	if n == 0 {
		t.Fatal("пустая запись")
	}
	for sim.elapsed < 70 {
		sim.mu.Lock()
		sim.step(0.05)
		sim.mu.Unlock()
	}
	if len(snap.Ship) != n {
		t.Fatalf("снимок изменился после продолжения полёта: было %d точек, стало %d",
			n, len(snap.Ship))
	}
}

// Возврат прогона к T+0 обязан очищать трассу: она относится к прошлому
// полёту.
func TestLiveTrajectory_ResetClearsTrack(t *testing.T) {
	sim, before := liveAfter(t, 40)
	if len(before.Ship) == 0 {
		t.Fatal("пустая запись до сброса")
	}
	sim.ResetRun()
	after := sim.LiveTrajectory()
	if len(after.Ship) != 0 || len(after.Booster) != 0 || len(after.Events) != 0 {
		t.Fatalf("после сброса трасса не очищена: %d/%d точек, %d событий",
			len(after.Ship), len(after.Booster), len(after.Events))
	}
}

// Координаты живой записи обязаны быть согласованы с высотой — та же
// проверка, что и для записи по зерну: она ловит путаницу систем координат.
func TestLiveTrajectory_CoordinatesMatchAltitude(t *testing.T) {
	_, run := liveAfter(t, 60)
	for i, p := range run.Ship {
		r := math.Sqrt(p.X*p.X + p.Y*p.Y + p.Z*p.Z)
		if math.Abs(r-(physics.EarthRadius+p.Altitude)) > 1 {
			t.Fatalf("точка %d: |ECEF|=%.1f, а высота даёт %.1f",
				i, r, physics.EarthRadius+p.Altitude)
		}
	}
}
