package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// Центр зоны захвата обязан стоять РОВНО НАД центром стартового стола: это и
// есть весь смысл выноса рук — башня рядом, а ловит она над столом.
func TestCatchTargetSitsAbovePad(t *testing.T) {
	tower := newCatchTower(vehicle.StarshipConfig())

	padECI := physics.ECEFToECI(physics.GeodeticToECEF(tower.Pad), 0)
	along, across, up := tower.decompose(padECI, 0)

	tAlong, tAcross, _ := tower.Target()
	if math.Abs(along-tAlong) > 0.01 || math.Abs(across-tAcross) > 0.01 {
		t.Errorf("центр стола не под зоной захвата: вдоль %.3f (цель %.3f), поперёк %.3f (цель %.3f)",
			along, tAlong, across, tAcross)
	}
	if math.Abs(up) > 0.01 {
		t.Errorf("центр стола не на уровне основания фермы: %.3f м", up)
	}
}

// Оси башни обязаны быть правой ортонормированной тройкой: иначе «поперёк»
// окажется не поперёк, и допуск в два метра начнёт мерить не ту величину.
func TestCatchTowerAxesAreOrthonormal(t *testing.T) {
	tower := newCatchTower(vehicle.StarshipConfig())

	for _, c := range []struct {
		name string
		v    physics.Vec3
	}{{"вдоль", tower.along}, {"поперёк", tower.across}, {"вверх", tower.up}} {
		if math.Abs(c.v.Norm()-1) > 1e-9 {
			t.Errorf("ось «%s» не единичная: %.12f", c.name, c.v.Norm())
		}
	}
	if d := tower.along.Dot(tower.across); math.Abs(d) > 1e-9 {
		t.Errorf("вдоль и поперёк не перпендикулярны: %.12f", d)
	}
	if d := tower.along.Dot(tower.up); math.Abs(d) > 1e-9 {
		t.Errorf("вдоль и вверх не перпендикулярны: %.12f", d)
	}
	if d := tower.along.Cross(tower.across).Dot(tower.up); d < 0.999 {
		t.Errorf("тройка левая: вдоль × поперёк · вверх = %.6f", d)
	}
}

// Попадание точно в центр зоны — нулевой промах по всем трём осям.
func TestCatchExactHitIsZeroMiss(t *testing.T) {
	tower := newCatchTower(vehicle.StarshipConfig())
	tAlong, tAcross, tUp := tower.Target()

	centre := tower.base.
		Add(tower.along.Scale(tAlong)).
		Add(tower.across.Scale(tAcross)).
		Add(tower.up.Scale(tUp))

	along, across, up := tower.decompose(physics.ECEFToECI(centre, 0), 0)
	if math.Abs(along-tAlong) > 0.01 || math.Abs(across-tAcross) > 0.01 || math.Abs(up-tUp) > 0.01 {
		t.Errorf("центр зоны не сходится сам с собой: %.3f/%.3f/%.3f против %.3f/%.3f/%.3f",
			along, across, up, tAlong, tAcross, tUp)
	}
}

// Условие захвата обязано отвергать проход мимо просвета — это удар о руку,
// а не «слегка неточная посадка».
func TestCatchRejectsMissedGap(t *testing.T) {
	tower := newCatchTower(vehicle.StarshipConfig())
	tAlong, _, tUp := tower.Target()

	good := catchPass{
		Valid: true, Crossed: true,
		Along: tAlong, Across: 0, Up: tUp,
		Vertical: 1, Horizontal: 0.5, Tilt: 1, AngularRate: 0.5,
	}
	if !tower.caught(good) {
		t.Fatal("точный подход в центр зоны не признан захватом")
	}

	// Половина просвета минус радиус корпуса — дальше корпус физически
	// не помещается между руками.
	clearance := tower.ArmGap/2 - tower.BodyRadius
	bad := good
	bad.Across = clearance + 0.5
	if tower.caught(bad) {
		t.Errorf("проход мимо просвета (%.2f м при зазоре %.2f м) признан захватом",
			bad.Across, clearance)
	}

	// Не пересёк плоскость — захвата нет ни при каких числах.
	notYet := good
	notYet.Crossed = false
	if tower.caught(notYet) == false && notYet.Success {
		t.Error("успех выставлен без пересечения плоскости")
	}
}

// Скорость и наклон входят в условие наравне с положением: попасть в просвет
// на сорока метрах в секунду — это не захват, а удар.
func TestCatchRejectsExcessiveState(t *testing.T) {
	tower := newCatchTower(vehicle.StarshipConfig())
	tAlong, _, tUp := tower.Target()

	base := catchPass{
		Valid: true, Crossed: true,
		Along: tAlong, Up: tUp,
		Vertical: 1, Horizontal: 0.5, Tilt: 1, AngularRate: 0.5,
	}
	cases := []struct {
		name string
		mod  func(*catchPass)
	}{
		{"скорость снижения", func(p *catchPass) { p.Vertical = tower.MaxVertical + 1 }},
		{"боковая скорость", func(p *catchPass) { p.Horizontal = tower.MaxHorizontal + 1 }},
		{"наклон", func(p *catchPass) { p.Tilt = tower.MaxTilt + 1 }},
		{"угловая скорость", func(p *catchPass) { p.AngularRate = tower.MaxAngularRate + 1 }},
		{"высота", func(p *catchPass) { p.Up = tUp + tower.CatchWindow + 1 }},
		{"вдоль рук", func(p *catchPass) { p.Along = tAlong + tower.CorridorAlong + 1 }},
	}
	for _, c := range cases {
		p := base
		c.mod(&p)
		if tower.caught(p) {
			t.Errorf("превышение «%s» признано захватом", c.name)
		}
	}
}

// Телеметрия до подхода — NaN, а не ноль: ноль здесь означал бы «промах
// нулевой» (см. CLAUDE.md про неопределённые величины).
func TestCatchTelemetryUndefinedIsNaN(t *testing.T) {
	tower := newCatchTower(vehicle.StarshipConfig())
	tel := catchTelemetry(tower, catchPass{})

	for name, v := range map[string]Float{
		"catchMissX":          tel.MissX,
		"catchMissY":          tel.MissY,
		"catchMissZ":          tel.MissZ,
		"catchMissHorizontal": tel.MissHorizontal,
		"catchMiss3D":         tel.Miss3D,
		"catchTilt":           tel.Tilt,
	} {
		if !math.IsNaN(float64(v)) {
			t.Errorf("%s до подхода равно %v, а должно быть NaN", name, float64(v))
		}
	}
	if tel.TargetZ != tower.CatchHeight {
		t.Errorf("высота цели %.1f вместо %.1f", tel.TargetZ, tower.CatchHeight)
	}
}

// Наведение обязано вести ступень к ПЛОСКОСТИ ЗАХВАТА, а не к поверхности.
//
// Проверяется тот отказ, ради которого правка и делалась: снижение погашено,
// но плоскость рук ещё выше — отпускать тягу нельзя. Прежний закон в этом
// месте честно возвращал ноль (для посадки на поверхность это означало «уже
// сели»), ступень падала свободно оставшиеся десятки метров и приходила в
// зону захвата на сорока метрах в секунду.
func TestGuidanceHoldsDescentAboveCatchPlane(t *testing.T) {
	b, nav := boosterInCoast(t, 1)

	// Ставим ступень над самой зоной захвата с уже погашенным снижением.
	tower := b.tower
	up := nav.Position.Unit()
	radius := physics.EarthRadius + tower.CatchHeight + 20
	nav.Position = up.Scale(radius)
	nav.Frame = physics.NewLocalFrame(nav.Position)
	nav.Altitude = tower.CatchHeight + 20
	nav.RadialVelocity = -1.0
	nav.Velocity = physics.CorotatingVelocity(nav.Position).Add(nav.Frame.Up.Scale(-1.0))
	nav.AirRelativeVelocity = nav.Frame.Up.Scale(-1.0)

	if got := b.landingRequiredThrust(nav); got <= 0 {
		t.Fatalf("над плоскостью захвата тяга отпущена: %.3f Н", got)
	}

	// А ниже плоскости уже нечего делать: либо поймали, либо нет.
	nav.Altitude = tower.CatchHeight - 1
	nav.Position = up.Scale(physics.EarthRadius + nav.Altitude)
	nav.Frame = physics.NewLocalFrame(nav.Position)
	if got := b.landingRequiredThrust(nav); got != 0 {
		t.Errorf("ниже плоскости захвата тяга не отпущена: %.3f Н", got)
	}
}

// Остаток пути считается до плоскости захвата, а не до поверхности.
func TestCatchAltitudeIsMeasuredFromCatchPlane(t *testing.T) {
	b, _ := boosterInCoast(t, 1)
	if got, want := b.catchAltitude(1000), 1000-b.tower.CatchHeight; got != want {
		t.Errorf("высота над зоной захвата %.3f вместо %.3f", got, want)
	}
}

// Предел наклона у самой плоскости захвата обязан закрываться, и он не должен
// раскрываться настежь оттого, что вертикальная скорость прошла через ноль:
// именно так корпус и заваливался набок.
func TestCatchTiltLimitClosesNearPlane(t *testing.T) {
	b, nav := boosterInCoast(t, 1)
	up := nav.Position.Unit()

	at := func(height, radial float64) float64 {
		nav.Altitude = b.tower.CatchHeight + height
		nav.Position = up.Scale(physics.EarthRadius + nav.Altitude)
		nav.Frame = physics.NewLocalFrame(nav.Position)
		nav.RadialVelocity = radial
		return b.catchTiltLimit(nav)
	}

	far := at(400, -60)
	near := at(5, -2)
	if !(near < far) {
		t.Errorf("предел наклона у плоскости (%.3f рад) не строже дальнего (%.3f рад)", near, far)
	}
	if rising := at(5, +2); rising > far {
		t.Errorf("на подъёме предел наклона раскрылся: %.3f рад против %.3f", rising, far)
	}
}
