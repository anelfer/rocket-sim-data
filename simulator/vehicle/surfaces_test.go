package vehicle

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Плавники корабля.
//
// Здесь проверяется не аэродинамика, а геометрия: где панель сидит, куда она
// поворачивается и что с ней происходит по мере раскрытия. Ошибиться в этом
// легко и незаметно — картинка продолжает что-то рисовать, а модель что-то
// считать, — поэтому каждое из свойств названо словами.
// -----------------------------------------------------------------------------

const testShipLength = 52.0
const testShipRadius = 4.5

// Расстановка по обводу взята у настоящего корабля второго поколения:
// передняя пара разведена примерно на 140 градусов, задняя — на 180.
func TestFlapsSitWhereTheRealShipHasThem(t *testing.T) {
	mounts := ShipFlapMounts(testShipLength, testShipRadius)
	if len(mounts) != 4 {
		t.Fatalf("плавников %d, ожидалось четыре", len(mounts))
	}

	byName := map[string]FlapMount{}
	for _, m := range mounts {
		byName[m.Name] = m
	}

	fwd := (byName["fwd_right"].Azimuth - byName["fwd_left"].Azimuth) / physics.DegToRad
	aft := (byName["aft_right"].Azimuth - byName["aft_left"].Azimuth) / physics.DegToRad

	if math.Abs(fwd-140) > 1 {
		t.Errorf("передняя пара разведена на %.0f°, у корабля 140°", fwd)
	}
	if math.Abs(aft-180) > 1 {
		t.Errorf("задняя пара разведена на %.0f°, у корабля 180°", aft)
	}

	// Передняя пара обязана сидеть ближе к спине задней: ради этого её
	// и переносили — чтобы вывести из потока.
	if math.Abs(byName["fwd_right"].Azimuth) >= math.Abs(byName["aft_right"].Azimuth) {
		t.Error("передние плавники не ближе к спине, чем задние")
	}

	// Панель вытянута вдоль корпуса, а не поперёк.
	for _, m := range mounts {
		if m.Span >= m.Chord {
			t.Errorf("%s: вылет %.1f м при хорде %.1f м — панель поперёк корпуса",
				m.Name, m.Span, m.Chord)
		}
		if math.Abs(m.Span*m.Chord-m.Area) > 1e-9 {
			t.Errorf("%s: размах на хорду даёт %.2f м² вместо %.2f",
				m.Name, m.Span*m.Chord, m.Area)
		}
	}

	t.Logf("передние ±%.0f°, задние ±%.0f°; вылет %.1f и %.1f м при хордах %.1f и %.1f м",
		byName["fwd_right"].Azimuth/physics.DegToRad,
		byName["aft_right"].Azimuth/physics.DegToRad,
		byName["fwd_right"].Span, byName["aft_right"].Span,
		byName["fwd_right"].Chord, byName["aft_right"].Chord)
}

// Прижатая панель лежит вдоль обшивки, раскрытая — подставлена потоку.
//
// Это то самое свойство, которого модели не хватало: нейтраль в ней была
// привязана к потоку независимо от места навески, и панель «раскрывалась»
// в поток, откуда бы ни росла.
func TestStowedFlapLiesAlongTheHullAndOpensIntoTheFlow(t *testing.T) {
	for _, s := range ShipFlaps(testShipLength, testShipRadius) {
		// Направление наружу в точке навески.
		outward := physics.Vec3{Y: s.Arm.Y, Z: s.Arm.Z}
		if n := outward.Norm(); n > 0 {
			outward = outward.Scale(1 / n)
		}

		stowed := s.Normal(0)
		open := s.Normal(s.MaxDeflection)

		// Прижатая: нормаль смотрит наружу, то есть панель лежит на борту.
		if d := stowed.Dot(outward); d < 0.85 {
			t.Errorf("%s: прижатая панель не лежит вдоль борта, нормаль отклонена "+
				"на %.0f° от радиуса", s.Name, math.Acos(d)/physics.DegToRad)
		}

		// Раскрытая: нормаль смотрит в сторону брюха, навстречу потоку.
		belly := physics.Vec3{Z: 1}
		if d := open.Dot(belly); d < 0.9 {
			t.Errorf("%s: раскрытая панель отклонена от потока на %.0f°",
				s.Name, math.Acos(d)/physics.DegToRad)
		}

		t.Logf("%-10s прижата: наружу %.2f · раскрыта: в поток %.2f",
			s.Name, stowed.Dot(outward), open.Dot(belly))
	}
}

// Раскрытие обязано выносить панель от корпуса, и только от корпуса.
//
// Прежняя рисовка вела панель к неподвижной точке «поперёк борта», а не
// поворачивала её на заданный угол. Путь до этой точки был длиннее хода
// привода, и после пятидесяти градусов панель уходила за перпендикуляр
// и заваливалась обратно: вылет от оси сначала рос, потом падал, и раскрытый
// плавник на картинке выглядел сложенным.
func TestOpeningFlapAlwaysMovesTipAwayFromHull(t *testing.T) {
	for _, m := range ShipFlapMounts(testShipLength, testShipRadius) {
		peak, peakAt := 0.0, 0.0

		for i := 0; i <= 24; i++ {
			deflection := m.Limit * float64(i) / 24
			if reach := flapTipReach(m, testShipRadius, deflection); reach > peak {
				peak, peakAt = reach, deflection
			}
		}

		stowed := flapTipReach(m, testShipRadius, 0)
		open := flapTipReach(m, testShipRadius, m.Limit)

		// Вылет обязан расти до самого упора.
		//
		// Строгого равенства требовать нельзя: панель встаёт точно по радиусу
		// чуть раньше упора, и последние градусы хода уводят её конец обратно
		// на считанные сантиметры. Прежняя же рисовка теряла на этом больше
		// метра — раскрытый плавник выглядел сложенным.
		if open < peak*0.98 {
			t.Errorf("%s: наибольший вылет %.2f м приходится на %.0f°, а на упоре "+
				"%.0f° остаётся %.2f м — раскрытая панель заваливается обратно",
				m.Name, peak, peakAt/physics.DegToRad,
				m.Limit/physics.DegToRad, open)
		}
		if open < stowed*1.4 {
			t.Errorf("%s: раскрытая панель уходит от оси на %.2f м против %.2f м "+
				"прижатой — раскрытие почти не видно", m.Name, open, stowed)
		}

		t.Logf("%-10s вылет от оси: прижата %.2f м, раскрыта %.2f м (наибольший "+
			"%.2f м на %.0f°)", m.Name, stowed, open, peak, peakAt/physics.DegToRad)
	}
}

// Прижатая панель не проваливается внутрь корпуса и не висит над ним.
func TestStowedFlapTouchesTheHull(t *testing.T) {
	for _, m := range ShipFlapMounts(testShipLength, testShipRadius) {
		// Ось навески приподнята, чтобы плоская панель легла на обшивку
		// серединой.
		hinge := testShipRadius / math.Cos(m.StowHalf)

		// Середина прижатой панели обязана оказаться ровно на обводе.
		mid := flapPointReach(m, testShipRadius, 0, m.Span/2)
		if math.Abs(mid-testShipRadius) > 0.01 {
			t.Errorf("%s: середина прижатой панели на %.2f м от оси при радиусе "+
				"%.2f м", m.Name, mid, testShipRadius)
		}

		// А концы — чуть выше, и это не изъян: плоскую панель к цилиндру
		// плотнее не приложить.
		gap := hinge - testShipRadius
		if gap > 0.6 {
			t.Errorf("%s: корень панели поднят над бортом на %.2f м — слишком "+
				"высокий прилив", m.Name, gap)
		}

		t.Logf("%-10s прилив под корнем %.2f м, дуга под панелью %.0f°",
			m.Name, gap, 2*m.StowHalf/physics.DegToRad)
	}
}

// flapPointReach возвращает расстояние от оси корабля до точки панели,
// отстоящей от навески на s метров, при заданном отклонении.
//
// Панель жёсткая и плоская: она поворачивается вокруг оси навески целиком.
// Считается здесь то же построение, по которому её рисует сцена, — иначе
// проверять было бы нечего.
func flapPointReach(m FlapMount, radius, deflection, s float64) float64 {
	hand := 1.0
	if m.Azimuth < 0 {
		hand = -1
	}

	hinge := radius / math.Cos(m.StowHalf)
	hy := hinge * math.Sin(m.Azimuth)
	hz := -hinge * math.Cos(m.Azimuth)

	// Прижатое положение — касательная в середине закрытой дуги.
	mid := m.Azimuth - hand*m.StowHalf
	stowed := mid
	if hand > 0 {
		stowed += math.Pi
	}
	dir := stowed + hand*deflection

	y := hy + s*math.Cos(dir)
	z := hz + s*math.Sin(dir)
	return math.Hypot(y, z)
}

func flapTipReach(m FlapMount, radius, deflection float64) float64 {
	return flapPointReach(m, radius, deflection, m.Span)
}
