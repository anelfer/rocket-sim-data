package physics

import "math"

// Физические константы. Все величины в СИ.
const (
	// G0 — стандартное ускорение свободного падения (используется только
	// в определении удельного импульса, а НЕ как локальная гравитация).
	G0 = 9.80665 // м/с²

	// GM — гравитационный параметр Земли (WGS-84).
	GM = 3.986004418e14 // м³/с²

	// EarthRadius — средний радиус Земли (сферическая модель).
	EarthRadius = 6371000.0 // м

	// EarthEquatorialRadius — экваториальный радиус, нужен для J2.
	EarthEquatorialRadius = 6378137.0 // м

	// J2 — второй зональный гармонический коэффициент гравитационного поля.
	// Отвечает за сплюснутость Земли и прецессию восходящего узла орбиты.
	J2 = 1.08262668e-3

	// SiderealDay — звёздные сутки: период вращения Земли относительно звёзд.
	SiderealDay = 86164.0905 // с

	// EarthAngularVelocity — угловая скорость вращения Земли.
	EarthAngularVelocity = 2 * 3.141592653589793 / SiderealDay // ≈7.2921159e-5 рад/с

	// GammaAir — показатель адиабаты для воздуха.
	GammaAir = 1.4

	// RSpecificAir — удельная газовая постоянная сухого воздуха.
	RSpecificAir = 287.05 // Дж/(кг·К)

	// StefanBoltzmann — постоянная Стефана — Больцмана (радиационный баланс).
	StefanBoltzmann = 5.670374419e-8 // Вт/(м²·К⁴)

	// SolarConstant — солнечная постоянная на орбите Земли.
	SolarConstant = 1361.0 // Вт/м²

	// EarthIRFlux — среднее инфракрасное излучение Земли в космос.
	EarthIRFlux = 239.0 // Вт/м²

	// KelvinOffset — смещение шкалы Цельсия относительно Кельвина.
	KelvinOffset = 273.15
)

// EarthRotationVector — вектор угловой скорости Земли в ECI (вдоль оси Z).
var EarthRotationVector = Vec3{X: 0, Y: 0, Z: EarthAngularVelocity}

// KelvinToCelsius переводит Кельвины в градусы Цельсия.
func KelvinToCelsius(k float64) float64 { return k - KelvinOffset }

// CelsiusToKelvin переводит градусы Цельсия в Кельвины.
func CelsiusToKelvin(c float64) float64 { return c + KelvinOffset }

// -----------------------------------------------------------------------------
// Перевод углов
// -----------------------------------------------------------------------------

const (
	// DegToRad переводит градусы в радианы.
	DegToRad = math.Pi / 180

	// RadToDeg переводит радианы в градусы.
	RadToDeg = 180 / math.Pi
)
