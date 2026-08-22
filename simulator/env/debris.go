package env

import (
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Поле обломков.
//
// «Разрушилась» не означает «исчезла»: при разрушении на высоте порядка 25 км
// конструкция распадается на фрагменты, и часть из них достигает поверхности.
// Разлёт определяется в первую очередь баллистическим коэффициентом
//
//	β = m / (Cd · A)
//
// Лёгкие панели с малым β тормозятся почти сразу и падают практически под
// точкой разрушения. Плотные фрагменты — двигатели, силовые шпангоуты — теряют
// скорость медленно и улетают на десятки километров вперёд.
//
// Каждый фрагмент интегрируется отдельно только для нескольких представительных
// классов: этого достаточно, чтобы получить границы зоны падения, и на порядки
// дешевле, чем моделировать сотни обломков.
// -----------------------------------------------------------------------------

// FragmentClass — представительный класс обломков.
type FragmentClass struct {
	Name string

	// Mass — масса фрагмента, кг.
	Mass float64

	// Area — площадь миделя, м².
	Area float64

	// DragCoefficient — коэффициент сопротивления кувыркающегося обломка.
	DragCoefficient float64

	// MeltAltitude — высота, ниже которой фрагмент считается разрушенным
	// от нагрева. Ноль означает, что фрагмент долетает до земли.
	Demises bool
}

// BallisticCoefficient возвращает β = m/(Cd·A), кг/м².
func (f FragmentClass) BallisticCoefficient() float64 {
	if f.Area <= 0 || f.DragCoefficient <= 0 {
		return 0
	}
	return f.Mass / (f.DragCoefficient * f.Area)
}

// DefaultFragments возвращает представительные классы обломков ступени.
//
// Значения соответствуют тому, что реально находят после разрушения ступеней:
// фрагменты обечайки баков сгорают или падают рядом, а двигатели и днища
// баков как компактные плотные объекты улетают дальше всех.
func DefaultFragments() []FragmentClass {
	return []FragmentClass{
		{Name: "обшивка бака", Mass: 15, Area: 4.0, DragCoefficient: 1.3, Demises: true},
		{Name: "секция обечайки", Mass: 250, Area: 6.0, DragCoefficient: 1.2},
		{Name: "днище бака", Mass: 900, Area: 8.0, DragCoefficient: 1.0},
		{Name: "двигательный отсек", Mass: 6000, Area: 8.5, DragCoefficient: 0.9},
	}
}

// FragmentImpact — результат полёта одного класса обломков.
type FragmentImpact struct {
	Class FragmentClass

	Reached      bool    // достиг ли поверхности
	Latitude     float64 // точка падения
	Longitude    float64
	FlightTime   float64 // время падения от разрушения, с
	Downrange    float64 // удаление от точки разрушения, м
	ImpactSpeed  float64 // скорость у поверхности, м/с
	PeakHeatFlux float64
}

// DebrisField — оценка зоны падения обломков.
type DebrisField struct {
	// Точка разрушения.
	BreakupLatitude  float64
	BreakupLongitude float64
	BreakupAltitude  float64

	Impacts []FragmentImpact

	// Эллипс рассеяния.
	CenterLatitude  float64
	CenterLongitude float64
	SemiMajorAxis   float64 // вдоль трассы, м
	SemiMinorAxis   float64 // поперёк трассы, м
	BearingDegrees  float64 // ориентация большой оси, градусы от севера

	// Доля массы, достигшей поверхности.
	SurvivingMassFraction float64
}

// EstimateDebrisField оценивает зону падения обломков.
//
// Для каждого представительного класса интегрируется отдельная траектория
// с собственным баллистическим коэффициентом. Эллипс строится по крайним
// точкам падения: большая ось — вдоль трассы между ближайшим и дальнейшим
// фрагментом, малая — поперечный разброс от скорости разлёта при разрушении.
func EstimateDebrisField(position, velocity physics.Vec3, wind WindModel, t float64) *DebrisField {
	breakup := physics.ECIToGeodetic(position, t)

	field := &DebrisField{
		BreakupLatitude:  breakup.Latitude,
		BreakupLongitude: breakup.Longitude,
		BreakupAltitude:  breakup.Altitude,
	}

	fragments := DefaultFragments()
	frame := physics.NewLocalFrame(position)

	// Направление трассы: горизонтальная составляющая скорости.
	horizontal := velocity.Sub(frame.Up.Scale(velocity.Dot(frame.Up)))
	if horizontal.Norm() > 1 {
		_, field.BearingDegrees = frame.PitchAzimuth(horizontal)
	}

	var totalMass, survivedMass float64
	minRange, maxRange := math.Inf(1), math.Inf(-1)
	var maxFlightTime float64

	for _, class := range fragments {
		totalMass += class.Mass
		impact := propagateFragment(class, position, velocity, wind, t)
		field.Impacts = append(field.Impacts, impact)

		if !impact.Reached {
			continue
		}
		survivedMass += class.Mass
		minRange = math.Min(minRange, impact.Downrange)
		maxRange = math.Max(maxRange, impact.Downrange)
		maxFlightTime = math.Max(maxFlightTime, impact.FlightTime)
	}

	if totalMass > 0 {
		field.SurvivingMassFraction = survivedMass / totalMass
	}

	if math.IsInf(minRange, 1) {
		// Ничего не долетело.
		field.CenterLatitude = breakup.Latitude
		field.CenterLongitude = breakup.Longitude
		return field
	}

	field.SemiMajorAxis = math.Max((maxRange-minRange)/2, 500)

	// Поперечный разброс: скорость разлёта при разрушении порядка 10 м/с,
	// умноженная на время падения, плюс снос ветром.
	const fragmentationVelocity = 10.0
	field.SemiMinorAxis = math.Max(fragmentationVelocity*maxFlightTime*0.5, 500)

	// Центр эллипса — середина между крайними точками падения.
	center := (minRange + maxRange) / 2
	field.CenterLatitude, field.CenterLongitude = offsetAlongBearing(
		breakup.Latitude, breakup.Longitude, center, field.BearingDegrees,
	)

	return field
}

// propagateFragment интегрирует падение одного фрагмента.
func propagateFragment(
	class FragmentClass,
	position, velocity physics.Vec3,
	wind WindModel,
	t float64,
) FragmentImpact {
	impact := FragmentImpact{Class: class}

	beta := class.BallisticCoefficient()
	if beta <= 0 {
		return impact
	}

	pos, vel := position, velocity
	start := position
	elapsed := 0.0

	const dt = 0.5
	const maxTime = 3600.0

	for elapsed < maxTime {
		altitude := pos.Norm() - physics.EarthRadius
		if altitude <= 0 {
			impact.Reached = true
			break
		}

		atm := physics.Atmosphere(altitude)

		air := physics.CorotatingVelocity(pos)
		if wind != nil {
			air = air.Add(wind.Velocity(pos, altitude, t+elapsed))
		}
		vRel := vel.Sub(air)
		speed := vRel.Norm()

		acc := physics.GravityAcceleration(pos)
		if speed > 1e-3 && atm.Density > 0 {
			// a = −q/β · v̂, где q = ½ρv²
			q := physics.DynamicPressure(atm.Density, speed)
			acc = acc.Add(vRel.Unit().Scale(-q / beta))

			if flux := physics.StagnationHeatFlux(atm.Density, speed, 0.2); flux > impact.PeakHeatFlux {
				impact.PeakHeatFlux = flux
			}
		}

		// Лёгкие фрагменты с малым баллистическим коэффициентом сгорают:
		// вся кинетическая энергия уходит в тепло, а рассеять его нечем.
		if class.Demises && impact.PeakHeatFlux > 150000 {
			impact.Reached = false
			return impact
		}

		// Полушаговый метод: точности достаточно для оценки зоны падения.
		vel = vel.Add(acc.Scale(dt))
		pos = pos.Add(vel.Scale(dt))
		elapsed += dt
		impact.ImpactSpeed = vel.Sub(physics.CorotatingVelocity(pos)).Norm()
	}

	if !impact.Reached {
		return impact
	}

	geo := physics.ECIToGeodetic(pos, t+elapsed)
	impact.Latitude = geo.Latitude
	impact.Longitude = geo.Longitude
	impact.FlightTime = elapsed

	// Удаление считается по дуге большого круга от точки разрушения
	// в связанной с Землёй системе: иначе в него войдёт вращение планеты.
	startGeo := physics.ECIToGeodetic(start, t)
	impact.Downrange = greatCircleDistance(
		startGeo.Latitude, startGeo.Longitude, geo.Latitude, geo.Longitude,
	)

	return impact
}

// greatCircleDistance возвращает расстояние по дуге большого круга, м.
func greatCircleDistance(lat1, lon1, lat2, lon2 float64) float64 {
	const deg = math.Pi / 180
	p1, p2 := lat1*deg, lat2*deg
	dp := (lat2 - lat1) * deg
	dl := (lon2 - lon1) * deg

	a := math.Sin(dp/2)*math.Sin(dp/2) +
		math.Cos(p1)*math.Cos(p2)*math.Sin(dl/2)*math.Sin(dl/2)
	return 2 * physics.EarthRadius * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
}

// offsetAlongBearing возвращает точку на заданном удалении по азимуту.
func offsetAlongBearing(lat, lon, distance, bearing float64) (float64, float64) {
	const deg = math.Pi / 180
	d := distance / physics.EarthRadius
	b := bearing * deg
	p1 := lat * deg
	l1 := lon * deg

	p2 := math.Asin(math.Sin(p1)*math.Cos(d) + math.Cos(p1)*math.Sin(d)*math.Cos(b))
	l2 := l1 + math.Atan2(
		math.Sin(b)*math.Sin(d)*math.Cos(p1),
		math.Cos(d)-math.Sin(p1)*math.Sin(p2),
	)
	return p2 / deg, physics.NormalizeLongitude(l2 / deg)
}
