package physics

import "math"

// -----------------------------------------------------------------------------
// Аэродинамический и радиационный нагрев.
//
// Раньше нагрев конструкции не моделировался вообще: плотность и скорость
// передавались в тепловую модель, но использовались только для расчёта скорости
// остывания выключенного двигателя. Здесь появляются оба реальных механизма —
// конвективный нагрев в плотных слоях и радиационный баланс в вакууме.
// -----------------------------------------------------------------------------

const (
	// suttonGravesConstant — коэффициент формулы Саттона — Грейвса
	// для воздуха в системе СИ: q = k·√(ρ/Rn)·V³.
	suttonGravesConstant = 1.7415e-4

	// recoveryFactor — коэффициент восстановления температуры для
	// турбулентного пограничного слоя (≈ Pr^(1/3) для воздуха).
	recoveryFactor = 0.89

	// referenceWallTemp — опорная температура «холодной стенки», при которой
	// определена формула Саттона — Грейвса. Нужна для пересчёта потока
	// в коэффициент теплоотдачи.
	referenceWallTemp = 300.0 // К

	// surfaceEmissivity — степень черноты обшивки.
	surfaceEmissivity = 0.85

	// surfaceAbsorptivity — коэффициент поглощения солнечного излучения.
	surfaceAbsorptivity = 0.30

	// earthAlbedo — среднее альбедо Земли.
	earthAlbedo = 0.30

	// sunlitFraction — доля витка на освещённой стороне для типичной LEO.
	sunlitFraction = 0.60
)

// RecoveryTemperature возвращает температуру восстановления (К) — ту температуру,
// к которой стремится теплоизолированная поверхность в набегающем потоке.
//
//	T_r = T∞ · (1 + r·(γ−1)/2 · M²)
//
// При M = 8 и T∞ = 220 К это ≈ 2600 К — именно поэтому носовой обтекатель
// на активном участке греется, а не остывает.
func RecoveryTemperature(ambientK, mach float64) float64 {
	if ambientK <= 0 {
		return 0
	}
	return ambientK * (1 + recoveryFactor*(GammaAir-1)/2*mach*mach)
}

// StagnationHeatFlux возвращает конвективный тепловой поток в критической точке
// по формуле Саттона — Грейвса (Вт/м²) для холодной стенки.
//
//	q = k·√(ρ/Rn)·V³
//
// noseRadius — эффективный радиус затупления, м.
func StagnationHeatFlux(density, speed, noseRadius float64) float64 {
	if density <= 0 || speed <= 0 || noseRadius <= 0 {
		return 0
	}
	return suttonGravesConstant * math.Sqrt(density/noseRadius) * speed * speed * speed
}

// ConvectiveHeatTransferCoefficient возвращает эффективный коэффициент
// теплоотдачи (Вт/(м²·К)) от набегающего потока к поверхности.
//
// Получается пересчётом потока Саттона — Грейвса на разность температур:
// это даёт правильный поток при холодной стенке и корректно обнуляет нагрев,
// когда стенка прогрелась до температуры восстановления.
//
// bodyFactor — понижающий множитель для поверхностей вне критической точки
// (боковая обшивка греется в несколько раз слабее носка).
func ConvectiveHeatTransferCoefficient(density, speed, noseRadius, ambientK, mach, bodyFactor float64) float64 {
	tr := RecoveryTemperature(ambientK, mach)
	deltaT := tr - referenceWallTemp
	if deltaT <= 1 {
		return 0
	}
	q := StagnationHeatFlux(density, speed, noseRadius)
	return bodyFactor * q / deltaT
}

// AeroHeatFlux возвращает результирующий конвективный поток (Вт/м²) к стенке
// с температурой wallK. Отрицательное значение означает, что поток охлаждает
// стенку — так бывает, когда двигатель разогрел конструкцию сильнее потока.
func AeroHeatFlux(density, speed, noseRadius, ambientK, mach, bodyFactor, wallK float64) float64 {
	h := ConvectiveHeatTransferCoefficient(density, speed, noseRadius, ambientK, mach, bodyFactor)
	if h == 0 {
		return 0
	}
	return h * (RecoveryTemperature(ambientK, mach) - wallK)
}

// RadiativeEquilibriumTemperature возвращает равновесную температуру (К)
// поверхности в вакууме на заданной высоте.
//
// В космосе температура конструкции определяется не «температурой воздуха»
// (на 400 км кинетическая температура газа около 1000 К, но плотность
// 3·10⁻¹² кг/м³, и теплообмена с ним практически нет), а балансом:
//
//	поглощение Солнца + альбедо Земли + ИК Земли = собственное излучение
func RadiativeEquilibriumTemperature(altitude float64) float64 {
	r := EarthRadius + altitude
	// Угловой коэффициент облучённости от Земли для вращающегося тела.
	viewFactor := 0.5 * (EarthRadius / r) * (EarthRadius / r)

	// Солнце: для вращающегося тела освещена в среднем четверть поверхности.
	solar := SolarConstant / 4 * sunlitFraction
	// Отражённое Землёй солнечное излучение.
	albedo := SolarConstant * earthAlbedo * viewFactor * sunlitFraction
	// Собственное ИК-излучение Земли (есть и в тени).
	infrared := EarthIRFlux * viewFactor

	absorbed := surfaceAbsorptivity*(solar+albedo) + surfaceEmissivity*infrared
	return math.Pow(absorbed/(surfaceEmissivity*StefanBoltzmann), 0.25)
}

// RadiativeFlux возвращает нетто-поток излучения (Вт/м²) для стенки с
// температурой wallK на заданной высоте: положительный — нагрев, отрицательный —
// охлаждение излучением в космос.
func RadiativeFlux(altitude, wallK float64) float64 {
	eq := RadiativeEquilibriumTemperature(altitude)
	return surfaceEmissivity * StefanBoltzmann * (math.Pow(eq, 4) - math.Pow(wallK, 4))
}

// EffectiveEnvironmentTemperature возвращает температуру, к которой в
// действительности стремится конструкция на заданной высоте: в плотных слоях —
// температура восстановления набегающего потока, в вакууме — радиационное
// равновесие. Переход между режимами плавный по плотности.
//
// Именно эта величина имеет смысл как «температура окружающей среды» в
// телеметрии; кинетическая температура газа на орбитальных высотах вводит
// в заблуждение.
func EffectiveEnvironmentTemperature(altitude, relativeSpeed float64) float64 {
	atm := Atmosphere(altitude)
	radiative := RadiativeEquilibriumTemperature(altitude)

	if atm.Density <= 0 || atm.SoundSpeed <= 0 {
		return radiative
	}

	mach := relativeSpeed / atm.SoundSpeed
	recovery := RecoveryTemperature(atm.Temperature, mach)

	// Вес аэродинамического режима: на уровне моря ≈1, к 100 км → 0.
	// Порог 1e-5 кг/м³ примерно соответствует высоте 85 км.
	w := atm.Density / (atm.Density + 1e-5)
	if w > 1 {
		w = 1
	}
	return w*recovery + (1-w)*radiative
}
