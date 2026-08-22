package physics

import (
	"math"
	"testing"
)

const eps = 1e-9

func approx(t *testing.T, got, want, tol float64, name string) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s: получено %.6g, ожидалось %.6g (допуск %.3g)", name, got, want, tol)
	}
}

// -----------------------------------------------------------------------------
// A1: скорость звука считается от температуры в Кельвинах.
// Прежний код передавал сюда градусы Цельсия, и выше тропопаузы получался NaN.
// -----------------------------------------------------------------------------

func TestSpeedOfSoundUsesKelvin(t *testing.T) {
	approx(t, SpeedOfSound(288.15), 340.29, 0.1, "скорость звука на уровне моря")
	approx(t, SpeedOfSound(216.65), 295.07, 0.1, "скорость звука в тропопаузе")

	// Отрицательная и нулевая температура не должны давать NaN.
	if v := SpeedOfSound(-10); v != 0 {
		t.Errorf("при отрицательной температуре ожидался 0, получено %v", v)
	}
	if math.IsNaN(SpeedOfSound(0)) {
		t.Error("скорость звука не должна быть NaN")
	}
}

func TestSoundSpeedFiniteThroughAtmosphere(t *testing.T) {
	for h := 0.0; h <= 300000; h += 1000 {
		a := Atmosphere(h)
		if math.IsNaN(a.SoundSpeed) || a.SoundSpeed <= 0 {
			t.Fatalf("на высоте %.0f м скорость звука = %v", h, a.SoundSpeed)
		}
	}
}

// Коэффициент сопротивления должен показывать трансзвуковой пик.
// Раньше из-за NaN в числе Маха всегда возвращалось значение по умолчанию 0.31.
func TestDragCoefficientTransonicPeak(t *testing.T) {
	subsonic := DragCoefficient(0.5)
	peak := DragCoefficient(1.0)
	hypersonic := DragCoefficient(6.0)

	if !(peak > subsonic && peak > hypersonic) {
		t.Errorf("ожидался трансзвуковой пик: M0.5=%.3f, M1.0=%.3f, M6=%.3f",
			subsonic, peak, hypersonic)
	}
	approx(t, subsonic, 0.20, 1e-9, "Cd на M=0.5")
	approx(t, peak, 0.50, 1e-9, "Cd на M=1.0")

	if got := DragCoefficient(math.NaN()); math.IsNaN(got) {
		t.Error("Cd не должен быть NaN при NaN на входе")
	}
}

// -----------------------------------------------------------------------------
// Атмосфера
// -----------------------------------------------------------------------------

func TestAtmosphereMonotonicDensity(t *testing.T) {
	prev := math.Inf(1)
	for h := 0.0; h <= 1000000; h += 500 {
		rho := AtmosphericDensity(h)
		if rho <= 0 {
			t.Fatalf("на высоте %.0f м плотность = %v", h, rho)
		}
		if rho > prev {
			t.Fatalf("плотность выросла с высотой на %.0f м: %.4g > %.4g", h, rho, prev)
		}
		prev = rho
	}
}

// Логарифмическая интерполяция должна попадать в табличные значения
// USSA-76 между узлами. Линейная давала здесь ошибку в десятки процентов.
func TestAtmosphericDensityAccuracy(t *testing.T) {
	cases := []struct {
		altitude float64
		expected float64
		relTol   float64
	}{
		{5000, 0.73643, 0.01},
		{11000, 0.36480, 0.01},
		{85000, 8.22e-6, 0.10},
		{200000, 2.789e-10, 0.01},
		{400000, 3.725e-12, 0.01},
	}
	for _, c := range cases {
		got := AtmosphericDensity(c.altitude)
		rel := math.Abs(got-c.expected) / c.expected
		if rel > c.relTol {
			t.Errorf("плотность на %.0f м: получено %.4g, ожидалось %.4g (отклонение %.1f%%)",
				c.altitude, got, c.expected, rel*100)
		}
	}
}

// B5: на орбитальных высотах кинетическая температура выходит на экзосферную,
// а не растёт линейно до бесконечности, как при прежней экстраполяции.
func TestThermosphereTemperature(t *testing.T) {
	t200 := AtmosphericTemperature(200000)
	t400 := AtmosphericTemperature(400000)
	t1000 := AtmosphericTemperature(1000000)

	if t200 < 700 || t200 > 1000 {
		t.Errorf("температура на 200 км = %.1f К, ожидалось 700…1000 К", t200)
	}
	if t400 < 950 || t400 > 1000 {
		t.Errorf("температура на 400 км = %.1f К, ожидалось 950…1000 К", t400)
	}
	if t1000 > exosphericTemperature+eps {
		t.Errorf("температура на 1000 км = %.1f К, превышает экзосферную", t1000)
	}
}

// -----------------------------------------------------------------------------
// B3/B5: нагрев
// -----------------------------------------------------------------------------

func TestRecoveryTemperatureRisesWithMach(t *testing.T) {
	ambient := 220.0
	if RecoveryTemperature(ambient, 0) != ambient {
		t.Error("при M=0 температура восстановления равна температуре среды")
	}
	tr := RecoveryTemperature(ambient, 8)
	if tr < 2000 {
		t.Errorf("при M=8 ожидалась температура восстановления > 2000 К, получено %.0f К", tr)
	}
}

func TestStagnationHeatFluxScaling(t *testing.T) {
	q1 := StagnationHeatFlux(1e-4, 2000, 1.0)
	q2 := StagnationHeatFlux(1e-4, 4000, 1.0)
	// Поток пропорционален кубу скорости.
	approx(t, q2/q1, 8.0, 0.01, "зависимость теплового потока от скорости")

	if StagnationHeatFlux(0, 5000, 1) != 0 {
		t.Error("в вакууме конвективный поток должен быть нулевым")
	}
}

// Равновесная температура в вакууме должна быть в разумном диапазоне
// для аппарата на низкой орбите, а не 1000 К «температуры газа».
func TestRadiativeEquilibriumTemperature(t *testing.T) {
	temp := RadiativeEquilibriumTemperature(400000)
	// Для покрытия с отношением α/ε ≈ 0.35 (светлая эмаль) равновесие
	// приходится примерно на −25 °C. Главное — что это НЕ кинетическая
	// температура газа в 1000 К, которую выдавала прежняя модель.
	if temp < 220 || temp > 320 {
		t.Errorf("равновесная температура на 400 км = %.1f К, ожидалось 220…320 К", temp)
	}
	if kinetic := AtmosphericTemperature(400000); math.Abs(temp-kinetic) < 300 {
		t.Errorf("равновесная температура (%.0f К) не должна быть близка к кинетической (%.0f К)",
			temp, kinetic)
	}
}

func TestEffectiveEnvironmentTemperatureRegimes(t *testing.T) {
	// У поверхности при нулевой скорости — температура воздуха.
	surface := EffectiveEnvironmentTemperature(0, 0)
	approx(t, surface, 288.15, 1.0, "эффективная температура у поверхности")

	// На орбите — радиационное равновесие, а не кинетическая температура газа.
	orbital := EffectiveEnvironmentTemperature(400000, 7700)
	radiative := RadiativeEquilibriumTemperature(400000)
	approx(t, orbital, radiative, 1.0, "эффективная температура на орбите")
}

// -----------------------------------------------------------------------------
// C2: преобразования систем координат обратимы при одном и том же времени.
// Прежний код брал разные отсчёты времени в прямом и обратном преобразовании,
// из-за чего в момент выхода на орбиту координаты скакали.
// -----------------------------------------------------------------------------

func TestECIECEFRoundTrip(t *testing.T) {
	original := Vec3{X: 4000000, Y: 3000000, Z: 4500000}
	for _, tm := range []float64{0, 1, 600, 5400, 86164} {
		back := ECEFToECI(ECIToECEF(original, tm), tm)
		if back.Sub(original).Norm() > 1e-6 {
			t.Errorf("t=%.0f: обратное преобразование дало смещение %.6g м",
				tm, back.Sub(original).Norm())
		}
	}
}

func TestGeodeticRoundTrip(t *testing.T) {
	cases := []Geodetic{
		{45.9647, 63.3050, 0},
		{-33.5, -170.2, 400000},
		{0, 0, 200000},
		{51.6, 179.9, 550000},
	}
	for _, g := range cases {
		back := ECEFToGeodetic(GeodeticToECEF(g))
		approx(t, back.Latitude, g.Latitude, 1e-9, "широта")
		approx(t, back.Longitude, g.Longitude, 1e-9, "долгота")
		approx(t, back.Altitude, g.Altitude, 1e-6, "высота")
	}
}

// Стартовая площадка не должна «уезжать» относительно поверхности:
// точка, движущаяся вместе с Землёй, сохраняет широту и долготу.
func TestCorotatingPointKeepsGroundPosition(t *testing.T) {
	launch := Geodetic{Latitude: 45.9647, Longitude: 63.3050, Altitude: 0}
	r := ECEFToECI(GeodeticToECEF(launch), 0)
	v := CorotatingVelocity(r)

	// Аналитически вращаем точку на час вперёд и сравниваем координаты.
	const dt = 3600.0
	theta := EarthRotationAngle(dt)
	rotated := Vec3{
		X: r.X*math.Cos(theta) - r.Y*math.Sin(theta),
		Y: r.X*math.Sin(theta) + r.Y*math.Cos(theta),
		Z: r.Z,
	}
	geo := ECIToGeodetic(rotated, dt)

	approx(t, geo.Latitude, launch.Latitude, 1e-9, "широта стартовой площадки")
	approx(t, geo.Longitude, launch.Longitude, 1e-7, "долгота стартовой площадки")

	// Линейная скорость точки старта: ω·R·cos(φ).
	expected := EarthAngularVelocity * EarthRadius * math.Cos(launch.Latitude*math.Pi/180)
	approx(t, v.Norm(), expected, 1e-6, "скорость вращения на широте старта")
}

// -----------------------------------------------------------------------------
// Локальный базис
// -----------------------------------------------------------------------------

func TestLocalFrameOrthonormal(t *testing.T) {
	positions := []Vec3{
		{EarthRadius, 0, 0},
		{4000000, 3000000, 4500000},
		{0, 0, EarthRadius},
	}
	for _, p := range positions {
		f := NewLocalFrame(p)
		for _, v := range []Vec3{f.East, f.North, f.Up} {
			approx(t, v.Norm(), 1, 1e-12, "единичная длина базисного вектора")
		}
		approx(t, f.East.Dot(f.North), 0, 1e-12, "восток·север")
		approx(t, f.East.Dot(f.Up), 0, 1e-12, "восток·вверх")
		approx(t, f.North.Dot(f.Up), 0, 1e-12, "север·вверх")
	}
}

func TestLocalFrameDirectionRoundTrip(t *testing.T) {
	f := NewLocalFrame(Vec3{4000000, 3000000, 4500000})
	cases := []struct{ pitch, azimuth float64 }{
		{90, 0}, {0, 90}, {45, 63.3}, {-10, 270}, {30, 180},
	}
	for _, c := range cases {
		dir := f.Direction(c.pitch, c.azimuth)
		approx(t, dir.Norm(), 1, 1e-12, "единичная длина направления")

		pitch, azimuth := f.PitchAzimuth(dir)
		approx(t, pitch, c.pitch, 1e-9, "восстановленный угол места")
		if math.Abs(math.Cos(c.pitch*math.Pi/180)) > 1e-9 {
			approx(t, azimuth, NormalizeAzimuth(c.azimuth), 1e-9, "восстановленный азимут")
		}
	}
}

// -----------------------------------------------------------------------------
// Гравитация
// -----------------------------------------------------------------------------

func TestGravityMagnitude(t *testing.T) {
	approx(t, GravityMagnitudeAtAltitude(0), 9.82, 0.02, "гравитация у поверхности")
	approx(t, GravityMagnitudeAtAltitude(400000), 8.68, 0.02, "гравитация на 400 км")

	// Векторная гравитация направлена к центру.
	r := Vec3{4000000, 3000000, 4500000}
	g := GravityAcceleration(r)
	if g.Dot(r) >= 0 {
		t.Error("вектор гравитации должен быть направлен к центру Земли")
	}
	// Отличие от ньютоновской величины из-за J2 не превышает промилле в LEO.
	rel := math.Abs(g.Norm()-GM/r.Dot(r)) / (GM / r.Dot(r))
	if rel > 2e-3 {
		t.Errorf("вклад J2 = %.4f, ожидалось меньше 0.002", rel)
	}
}

// -----------------------------------------------------------------------------
// Орбитальные элементы
// -----------------------------------------------------------------------------

func TestCircularOrbitElements(t *testing.T) {
	altitude := 400000.0
	r := EarthRadius + altitude
	speed := CircularOrbitVelocity(altitude)

	// Круговая орбита с наклонением 51.6°.
	inc := 51.6 * math.Pi / 180
	pos := Vec3{r, 0, 0}
	vel := Vec3{0, speed * math.Cos(inc), speed * math.Sin(inc)}

	el := ElementsFromState(pos, vel, 0)

	if !el.Elliptical {
		t.Fatal("круговая орбита должна распознаваться как замкнутая")
	}
	approx(t, el.Eccentricity, 0, 1e-9, "эксцентриситет круговой орбиты")
	approx(t, el.SemiMajorAxis, r, 1e-3, "большая полуось")
	approx(t, el.InclinationDegrees(), 51.6, 1e-9, "наклонение")
	approx(t, el.ApoapsisAltitude, altitude, 1e-3, "высота апоцентра")
	approx(t, el.PeriapsisAltitude, altitude, 1e-3, "высота перицентра")
	// Для сферической модели с R = 6371 км период равен 5544.9 с.
	approx(t, el.Period(), 5544.9, 1.0, "период обращения")
}

func TestEllipticalOrbitElements(t *testing.T) {
	// Гомановская орбита: перицентр 200 км, апоцентр 400 км.
	rp := EarthRadius + 200000
	ra := EarthRadius + 400000
	a := (rp + ra) / 2
	vp := VisVivaSpeed(rp, a)

	pos := Vec3{rp, 0, 0}
	vel := Vec3{0, vp, 0}
	el := ElementsFromState(pos, vel, 0)

	approx(t, el.PeriapsisAltitude, 200000, 1.0, "высота перицентра")
	approx(t, el.ApoapsisAltitude, 400000, 1.0, "высота апоцентра")
	approx(t, el.Eccentricity, (ra-rp)/(ra+rp), 1e-9, "эксцентриситет")

	// В перицентре истинная аномалия равна нулю, до апоцентра — полпериода.
	approx(t, el.TimeToApoapsis(), el.Period()/2, 1.0, "время до апоцентра")
}

func TestHyperbolicOrbitIsNotElliptical(t *testing.T) {
	r := EarthRadius + 400000
	escape := math.Sqrt(2*GM/r) * 1.1

	el := ElementsFromState(Vec3{r, 0, 0}, Vec3{0, escape, 0}, 0)
	if el.Elliptical {
		t.Error("гиперболическая траектория не должна считаться замкнутой орбитой")
	}
	if !math.IsInf(el.ApoapsisAltitude, 1) {
		t.Errorf("у гиперболы апоцентра нет, получено %.4g", el.ApoapsisAltitude)
	}
	if el.Eccentricity <= 1 {
		t.Errorf("эксцентриситет гиперболы должен быть > 1, получено %.4f", el.Eccentricity)
	}
}

// -----------------------------------------------------------------------------
// Кватернионы
// -----------------------------------------------------------------------------

func TestQuaternionRotationRoundTrip(t *testing.T) {
	q := QuaternionFromAxisAngle(Vec3{1, 2, 3}, 1.1)
	v := Vec3{4, -5, 6}

	back := q.RotateInverse(q.Rotate(v))
	if back.Sub(v).Norm() > 1e-12 {
		t.Errorf("обратный поворот дал невязку %.3g", back.Sub(v).Norm())
	}
	approx(t, q.Rotate(v).Norm(), v.Norm(), 1e-12, "длина при повороте")
	approx(t, q.Norm(), 1, 1e-12, "норма кватерниона")
}

func TestQuaternionFromBasisRecoversAxes(t *testing.T) {
	forward := Vec3{1, 1, 0}.Unit()
	right := Vec3{0, 0, -1}
	up := forward.Cross(right).Unit()

	q := QuaternionFromBasis(forward, right, up)
	body := q.Body()

	if body.Forward.Sub(forward).Norm() > 1e-9 {
		t.Errorf("продольная ось не восстановлена: невязка %.3g",
			body.Forward.Sub(forward).Norm())
	}
	if body.Right.Sub(right).Norm() > 1e-9 {
		t.Errorf("поперечная ось не восстановлена: невязка %.3g",
			body.Right.Sub(right).Norm())
	}
}

// Интегрирование кватерниона при постоянной угловой скорости должно давать
// поворот на ω·t.
func TestQuaternionIntegration(t *testing.T) {
	omega := Vec3{X: 0.5} // рад/с вокруг продольной оси
	q := IdentityQuaternion()

	const dt = 0.001
	const steps = 2000 // 2 секунды

	for i := 0; i < steps; i++ {
		q = q.Add(q.Derivative(omega).Scale(dt)).Normalized()
	}

	expected := QuaternionFromAxisAngle(Vec3{X: 1}, 0.5*float64(steps)*dt)
	// Кватернионы q и −q описывают один поворот, сравниваем по модулю
	// скалярного произведения.
	dot := math.Abs(q.W*expected.W + q.X*expected.X + q.Y*expected.Y + q.Z*expected.Z)
	if math.Abs(dot-1) > 1e-6 {
		t.Errorf("интегрирование дало другой поворот: |q·q_ожид| = %.9f", dot)
	}
}

// -----------------------------------------------------------------------------
// Динамика вращения
// -----------------------------------------------------------------------------

func TestInertiaTensorForCylinder(t *testing.T) {
	// Тонкостенный цилиндр: продольный момент равен m·r².
	inertia := CylinderInertia(1000, 2, 10)
	approx(t, inertia.Ixx, 1000*4, 1e-9, "продольный момент инерции")
	if inertia.Iyy <= inertia.Ixx {
		t.Error("у вытянутого тела поперечный момент должен превышать продольный")
	}
	approx(t, inertia.Iyy, inertia.Izz, 1e-9, "симметрия поперечных моментов")
}

// Свободное вращение сохраняет момент импульса и энергию.
func TestFreeRotationConservesMomentum(t *testing.T) {
	inertia := CylinderInertia(25600, 1.83, 42.6)
	omega := Vec3{X: 0.3, Y: 0.2, Z: 0.1}

	h0 := inertia.AngularMomentum(omega).Norm()
	e0 := inertia.RotationalEnergy(omega)

	const dt = 0.0005
	for i := 0; i < 200000; i++ {
		// Классический RK4 для уравнений Эйлера без внешнего момента.
		k1 := inertia.AngularAcceleration(omega, Vec3{})
		k2 := inertia.AngularAcceleration(omega.Add(k1.Scale(dt/2)), Vec3{})
		k3 := inertia.AngularAcceleration(omega.Add(k2.Scale(dt/2)), Vec3{})
		k4 := inertia.AngularAcceleration(omega.Add(k3.Scale(dt)), Vec3{})
		omega = omega.Add(k1.Add(k2.Scale(2)).Add(k3.Scale(2)).Add(k4).Scale(dt / 6))
	}

	h1 := inertia.AngularMomentum(omega).Norm()
	e1 := inertia.RotationalEnergy(omega)

	if rel := math.Abs(h1-h0) / h0; rel > 1e-9 {
		t.Errorf("момент импульса изменился на %.3g", rel)
	}
	if rel := math.Abs(e1-e0) / e0; rel > 1e-9 {
		t.Errorf("энергия вращения изменилась на %.3g", rel)
	}
}

// Эффективная площадь цилиндра максимальна при поперечном обтекании.
func TestEffectiveAreaDependsOnAngle(t *testing.T) {
	shape := DefaultStageShape(42.6, 1.83)

	axial := shape.EffectiveArea(0)
	broadside := shape.EffectiveArea(math.Pi / 2)
	oblique := shape.EffectiveArea(math.Pi / 4)

	approx(t, axial, shape.ReferenceArea(), 1e-9, "площадь при осевом обтекании")
	approx(t, broadside, shape.SideArea(), 1e-9, "площадь при поперечном обтекании")

	if oblique <= axial || oblique >= broadside {
		t.Errorf("промежуточная площадь %.1f вне диапазона %.1f…%.1f",
			oblique, axial, broadside)
	}
}

// Демпфирующий момент обязан гасить вращение, а не раскручивать.
func TestAerodynamicDampingOpposesRotation(t *testing.T) {
	shape := DefaultStageShape(42.6, 1.83)

	// Поток строго вдоль оси: восстанавливающего момента нет, остаётся
	// только демпфирование.
	flow := Vec3{X: -1}
	omega := Vec3{Y: 0.5}

	torque := shape.AerodynamicTorque(20000, flow, omega, 3)

	if torque.Y >= 0 {
		t.Errorf("демпфирующий момент по оси Y = %.3g, должен быть отрицательным", torque.Y)
	}
}

// -----------------------------------------------------------------------------
// Ионизация и плазма
// -----------------------------------------------------------------------------

func TestShockTemperatureRisesWithMach(t *testing.T) {
	ambient := 250.0

	if ShockTemperature(ambient, 0.5) != ambient {
		t.Error("на дозвуке ударной волны нет")
	}

	t3 := ShockTemperature(ambient, 3)
	t8 := ShockTemperature(ambient, 8)
	t20 := ShockTemperature(ambient, 20)

	if !(t3 < t8 && t8 < t20) {
		t.Errorf("температура за скачком не растёт монотонно: %.0f, %.0f, %.0f", t3, t8, t20)
	}
	// При M = 8 без учёта реального газа получилось бы около 3400 К;
	// поправка на диссоциацию должна снизить результат.
	ideal := ambient * (1 + (GammaAir-1)/2*64)
	if t8 >= ideal {
		t.Errorf("поправка на реальный газ не применена: %.0f против идеальных %.0f", t8, ideal)
	}
}

// Степень ионизации растёт с температурой и падает с плотностью.
func TestIonizationFractionBehaviour(t *testing.T) {
	if f := IonizationFraction(1000, 1e-4); f != 0 {
		t.Errorf("при 1000 К ионизации быть не должно, получено %.3g", f)
	}

	low := IonizationFraction(5000, 1e-4)
	high := IonizationFraction(9000, 1e-4)
	if !(low > 0 && high > low) {
		t.Errorf("ионизация не растёт с температурой: %.3g → %.3g", low, high)
	}

	// В более разрежённом газе ионизация выше при той же температуре.
	dense := IonizationFraction(7000, 1e-3)
	rare := IonizationFraction(7000, 1e-6)
	if rare <= dense {
		t.Errorf("в разрежённом газе ионизация (%.3g) должна превышать плотный (%.3g)",
			rare, dense)
	}

	if f := IonizationFraction(50000, 1e-8); f > 1 {
		t.Errorf("степень ионизации не может превышать единицу: %.3g", f)
	}
}

// Радиоблэкаут наступает на гиперзвуке и отсутствует на умеренных скоростях.
func TestRadioBlackoutOnlyAtHypersonicSpeeds(t *testing.T) {
	atm := Atmosphere(60000)

	slow := AnalyzePlasma(atm.Temperature, atm.Density, 1500, DefaultTelemetryFrequency)
	if slow.RadioBlackout {
		t.Error("на 1.5 км/с плазменной оболочки быть не должно")
	}

	fast := AnalyzePlasma(atm.Temperature, atm.Density, 6000, DefaultTelemetryFrequency)
	if !fast.RadioBlackout {
		t.Errorf("на 6 км/с ожидался радиоблэкаут, плазменная частота %.3g Гц",
			fast.PlasmaFrequency)
	}
	if !fast.Luminous {
		t.Error("при 6 км/с ударный слой должен светиться")
	}

	// В вакууме плазмы нет независимо от скорости.
	vacuum := AnalyzePlasma(1000, 0, 8000, DefaultTelemetryFrequency)
	if vacuum.RadioBlackout || vacuum.IonizationFraction != 0 {
		t.Error("в вакууме плазменной оболочки быть не может")
	}
}

// Плазменная частота растёт как корень из концентрации электронов.
func TestPlasmaFrequencyScaling(t *testing.T) {
	f1 := PlasmaFrequency(1e16)
	f4 := PlasmaFrequency(4e16)
	approx(t, f4/f1, 2, 1e-9, "зависимость плазменной частоты от концентрации")

	if PlasmaFrequency(0) != 0 {
		t.Error("без свободных электронов плазменной частоты нет")
	}
}

// -----------------------------------------------------------------------------
// A4: азимут пуска
// -----------------------------------------------------------------------------

func TestLaunchAzimuth(t *testing.T) {
	// Байконур, наклонение как у МКС.
	az, reachable := LaunchAzimuthForInclination(45.9647, 51.6, true)
	if !reachable {
		t.Error("наклонение 51.6° достижимо с широты 46°")
	}
	approx(t, az, 63.3, 0.5, "азимут пуска на восходящей ветви")

	// На нисходящей ветви — зеркально относительно направления на восток.
	azDesc, _ := LaunchAzimuthForInclination(45.9647, 51.6, false)
	approx(t, azDesc, 180-63.3, 0.5, "азимут на нисходящей ветви")

	// Наклонение меньше широты недостижимо — азимут упирается в восток.
	az, reachable = LaunchAzimuthForInclination(51.6, 28.5, true)
	if reachable {
		t.Error("наклонение 28.5° недостижимо с широты 51.6°")
	}
	approx(t, az, 90, 1e-9, "азимут при недостижимом наклонении")

	// В точке максимальной широты азимут строго восточный.
	az, _ = LaunchAzimuthForInclination(51.6, 51.6, true)
	approx(t, az, 90, 1e-6, "азимут в точке максимальной широты")
}

// Наклонение орбиты, полученной пуском под расчётным азимутом, должно
// совпасть с целевым.
func TestLaunchAzimuthProducesTargetInclination(t *testing.T) {
	lat := 45.9647
	targetInc := 51.6

	az, _ := LaunchAzimuthForInclination(lat, targetInc, true)

	g := Geodetic{Latitude: lat, Longitude: 0, Altitude: 200000}
	pos := ECEFToECI(GeodeticToECEF(g), 0)
	frame := NewLocalFrame(pos)
	vel := frame.Direction(0, az).Scale(CircularOrbitVelocity(200000))

	el := ElementsFromState(pos, vel, 0)
	approx(t, el.InclinationDegrees(), targetInc, 0.01, "полученное наклонение")
}
