package vehicle

import (
	"fmt"
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Конфигурация ракеты-носителя.
//
// Все характеристики собраны в одном месте и связаны между собой формулами,
// а не разбросаны магическими числами по условиям смены фаз. Раньше, например,
// в проверке MECO стоял литерал 523454.0 — это масса топлива первой ступени
// из конструктора; при изменении массы ракеты логика молча ломалась.
// -----------------------------------------------------------------------------

// StageConfig — параметры одной ступени.
type StageConfig struct {
	Name string

	DryMass  float64 // сухая масса ступени, кг
	FuelMass float64 // масса топлива, кг

	EngineCount   float64 // число двигателей
	VacuumThrust  float64 // тяга одного двигателя в вакууме, Н
	SeaLevelThust float64 // тяга одного двигателя на уровне моря, Н
	VacuumISP     float64 // удельный импульс в вакууме, с
	SeaLevelISP   float64 // удельный импульс на уровне моря, с

	// MinThrottle — минимальный уровень дросселирования. Для Merlin 1D
	// это примерно 40 %: спуститься до 0.1 % от номинала, как позволяла
	// прежняя модель (MinThrust = 1000 Н при MaxThrust = 975 кН), нельзя.
	MinThrottle float64

	// FuelReserve — остаток топлива, который нельзя расходовать
	// (для возврата и посадки первой ступени), кг.
	FuelReserve float64

	// MixtureRatio — проектное соотношение компонентов при заправке.
	// Определяет, как масса топлива делится между баками горючего
	// и окислителя.
	MixtureRatio float64

	// TankPressure — поддерживаемое давление наддува баков, Па.
	FuelTankPressure float64
	OxTankPressure   float64

	// PressurantMass — запас газа наддува на бак, кг.
	FuelPressurantMass float64
	OxPressurantMass   float64

	// TankHeight — высота баков, м.
	FuelTankHeight float64
	OxTankHeight   float64
}

// Config — конфигурация носителя целиком.
type Config struct {
	Name string

	FirstStage  StageConfig
	SecondStage StageConfig

	// PayloadMass — масса полезной нагрузки, кг.
	//
	// Двенадцать тонн — это то, что модель выводит на орбиту 400 км с
	// наклонением 51.6° при программе тангажа, заданной таблицей. Реальный
	// носитель этого класса поднимает больше: его программа выведения
	// оптимизирована численно, а здесь профиль подобран вручную, и потери
	// на неоптимальность траектории выше.
	PayloadMass float64

	// FairingMass — масса головного обтекателя, кг.
	FairingMass float64

	// FairingJettisonAltitude — высота сброса обтекателя, м.
	FairingJettisonAltitude float64

	// Diameter — диаметр корпуса, м (для площади миделя).
	Diameter float64

	// EngineRingRadius — радиус кольца двигателей, м.
	EngineRingRadius float64

	// FirstStageLength — длина первой ступени, м. Нужна для расчёта моментов
	// инерции и аэродинамических моментов при неуправляемом входе.
	FirstStageLength float64

	// RCSMoment — располагаемый момент двигателей ориентации, Н·м.
	//
	// Величина конструктивная: тяга блока умноженная на плечо. У среднего
	// носителя это единицы килоньютон-метров, у сверхтяжёлого корабля —
	// сотни: развернуть триста тонн за разумное время меньшим моментом
	// невозможно, и с прежним общим значением корабль просто кувыркался,
	// выдавая тормозной импульс куда попало.
	RCSMoment float64

	// Flaps сообщает, что верхняя ступень несёт аэродинамические плавники
	// и рассчитана на возвращение в атмосферу.
	Flaps bool

	// TotalLength — длина носителя в сборе, м. В физике не участвует: нужна
	// развёртке, чтобы отсеки верхней ступени сошлись с реальным габаритом.
	// Ноль означает, что длину верхней ступени придётся оценить по бакам.
	TotalLength float64

	// LaunchLatitude, LaunchLongitude — координаты стартового комплекса.
	LaunchLatitude  float64
	LaunchLongitude float64

	// LaunchPadAzimuth — азимут, по которому ориентировано пусковое устройство,
	// градусы от севера. Стартовое сооружение стоит неподвижно, а азимут пуска
	// зависит от целевого наклонения, поэтому после отрыва носитель
	// разворачивается вокруг продольной оси на разницу между этими азимутами.
	// Именно этот манёвр и называется ролл-программой.
	LaunchPadAzimuth float64

	// TargetOrbitAltitude — целевая высота круговой орбиты, м.
	TargetOrbitAltitude float64

	// TargetPeriapsisAltitude — целевая высота перицентра, м.
	//
	// Ноль означает «как апоцентр», то есть круговую орбиту: так ведут себя
	// все конфигурации, которые о задании ничего не знают. Отрицательная
	// величина означает трансатмосферную траекторию — перицентр внутри
	// атмосферы, возвращение без тормозного импульса.
	TargetPeriapsisAltitude float64

	// TargetInclination — целевое наклонение орбиты, градусы.
	TargetInclination float64

	// MECOVelocity — горизонтальная скорость, при которой выключается
	// первая ступень, м/с.
	MECOVelocity float64

	// MECOMinAltitude — минимальная высота выключения первой ступени, м.
	MECOMinAltitude float64

	// StageSeparationDelay — пауза между MECO и запуском второй ступени, с.
	StageSeparationDelay float64

	// PitchProgramStretch — растяжение программы тангажа по высоте.
	//
	// Единица соответствует носителю со стартовой тяговооружённостью около
	// 1.4 и обычным для среднего класса темпом набора высоты. Более тяжёлому
	// носителю, который на той же высоте идёт медленнее, программу надо
	// растягивать, иначе он ляжет в горизонт раньше, чем разгонится,
	// и пойдёт под большим углом атаки.
	PitchProgramStretch float64
}

// CrossSectionArea возвращает площадь миделя, м².
func (c Config) CrossSectionArea() float64 {
	r := c.Diameter / 2
	return math.Pi * r * r
}

// LiftoffMass возвращает стартовую массу, кг.
func (c Config) LiftoffMass() float64 {
	return c.FirstStage.DryMass + c.FirstStage.FuelMass +
		c.SecondStage.DryMass + c.SecondStage.FuelMass +
		c.PayloadMass + c.FairingMass
}

// FirstStageDryMass возвращает сухую массу, которую «везёт» первая ступень:
// собственная конструкция плюс вся вторая ступень с топливом, нагрузкой
// и обтекателем.
func (c Config) FirstStageDryMass() float64 {
	return c.FirstStage.DryMass +
		c.SecondStage.DryMass + c.SecondStage.FuelMass +
		c.PayloadMass + c.FairingMass
}

// SecondStageDryMass возвращает сухую массу связки «вторая ступень + нагрузка».
func (c Config) SecondStageDryMass() float64 {
	return c.SecondStage.DryMass + c.PayloadMass + c.FairingMass
}

// Falcon9Config возвращает конфигурацию, приближённую к Falcon 9 Block 5.
func Falcon9Config() Config {
	return Config{
		Name: "Falcon 9 Block 5",

		FirstStage: StageConfig{
			Name:          "Stage 1 (9 × Merlin 1D)",
			DryMass:       25600,
			FuelMass:      411000,
			EngineCount:   9,
			VacuumThrust:  934000,
			SeaLevelThust: 845000,
			VacuumISP:     311,
			SeaLevelISP:   282,
			MinThrottle:   0.40,
			FuelReserve:   18000,

			MixtureRatio:       2.34,
			FuelTankPressure:   3.0e5,
			OxTankPressure:     3.4e5,
			FuelPressurantMass: 30,
			OxPressurantMass:   75,
			FuelTankHeight:     13.5,
			OxTankHeight:       25.0,
		},

		SecondStage: StageConfig{
			Name:          "Stage 2 (1 × Merlin 1D Vacuum)",
			DryMass:       4000,
			FuelMass:      107500,
			EngineCount:   1,
			VacuumThrust:  981000,
			SeaLevelThust: 981000, // вакуумный двигатель, противодавления нет
			VacuumISP:     348,
			SeaLevelISP:   348,
			MinThrottle:   0.39,
			FuelReserve:   0,

			MixtureRatio:       2.34,
			FuelTankPressure:   3.0e5,
			OxTankPressure:     3.4e5,
			FuelPressurantMass: 12,
			OxPressurantMass:   28,
			FuelTankHeight:     5.5,
			OxTankHeight:       9.0,
		},

		PayloadMass:             12000,
		FairingMass:             1900,
		FairingJettisonAltitude: 110000,

		RCSMoment: 4000,

		Diameter:         3.66,
		EngineRingRadius: 1.30,
		FirstStageLength: 42.6,
		TotalLength:      70.0,

		LaunchLatitude:   45.9647, // Байконур
		LaunchLongitude:  63.3050,
		LaunchPadAzimuth: 90, // пусковое устройство ориентировано на восток

		TargetOrbitAltitude: 400000,
		TargetInclination:   51.6,

		MECOVelocity:         2350,
		MECOMinAltitude:      60000,
		StageSeparationDelay: 4.0,
	}
}

// BuildFirstStageEngines собирает блок двигателей первой ступени:
// один центральный и восемь по кольцу (компоновка Octaweb).
func BuildFirstStageEngines(cfg Config, ambientTempK float64) []Engine {
	stage := cfg.FirstStage
	count := int(stage.EngineCount)
	engines := make([]Engine, count)

	ringCount := count - 1
	for i := 0; i < count; i++ {
		thermal := DefaultEngineThermalConfig()

		e := Engine{
			ID:        fmt.Sprintf("S1-%d", i+1),
			MaxThrust: stage.SeaLevelThust,
			MinThrust: stage.SeaLevelThust * stage.MinThrottle,
			Thrust:    stage.SeaLevelThust,
			VacIsp:    stage.VacuumISP,
			SeaIsp:    stage.SeaLevelISP,
			ISP:       stage.SeaLevelISP,
			Running:   true,
			Thermal:   thermal,
		}

		if i == 0 {
			// Центральный двигатель.
			e.AngleRad = 0
			e.Radius = 0
		} else {
			e.AngleRad = 2 * math.Pi * float64(i-1) / float64(ringCount)
			e.Radius = cfg.EngineRingRadius
		}

		e.SetTemperatures(ambientTempK)
		engines[i] = e
	}
	return engines
}

// BuildSecondStageEngine собирает двигатель второй ступени.
// Тепловое состояние задаётся заново: после разделения это физически другой
// двигатель, и переносить на него прогрев первой ступени неправильно.
// BuildSecondStageEngine собирает набор двигателей второй ступени.
//
// Число двигателей берётся из конфигурации носителя: у среднего класса он один,
// у сверхтяжёлого — шесть. Прежде здесь всегда создавался ровно один двигатель,
// и суммарная тяга второй ступени в телеметрии оказывалась вшестеро меньше
// фактической.
func BuildSecondStageEngine(cfg Config, ambientTempK float64) []Engine {
	stage := cfg.SecondStage

	thermal := DefaultEngineThermalConfig()
	// У вакуумного сопла степень расширения намного выше, газ на срезе холоднее.
	thermal.ExitPressureRatio = 0.0004
	thermal.ExternalArea = 9.0 // большое сопло с радиационным охлаждением

	total := int(stage.EngineCount)
	if total < 1 {
		total = 1
	}

	// Идентификаторы обязаны совпадать с теми, что раздаёт двигательная
	// установка (PropulsionSystem.addEngineGroup), а не просто идти подряд.
	// У методановой ступени это не сквозная нумерация: основная группа —
	// "S2-1"…"S2-3", вакуумная — "S2-V1"…"S2-V3". Раньше здесь всегда были
	// "S2-1"…"S2-6", и телеметрия трёх вакуумных камер либо синхронизировалась
	// по чужому имени (индексом, а не ID), либо не находила себе камеры вовсе —
	// в счётчике проходили несуществующие "S2-4", "S2-5", "S2-6".
	primary := total
	var groups []EngineGroup
	if IsMethalox(cfg) {
		primary = SecondStagePrimaryCount(cfg)
		groups = SecondStageGroups(cfg)
	}

	type slot struct {
		suffix string
		index  int
	}
	slots := make([]slot, 0, total)
	for i := 0; i < primary; i++ {
		slots = append(slots, slot{"", i})
	}
	for _, g := range groups {
		for i := 0; i < g.Count; i++ {
			slots = append(slots, slot{g.Suffix, i})
		}
	}

	engines := make([]Engine, len(slots))
	for idx, sl := range slots {
		e := Engine{
			ID:        fmt.Sprintf("S2-%s%d", sl.suffix, sl.index+1),
			MaxThrust: stage.VacuumThrust,
			MinThrust: stage.VacuumThrust * stage.MinThrottle,
			Thrust:    stage.VacuumThrust,
			VacIsp:    stage.VacuumISP,
			SeaIsp:    stage.SeaLevelISP,
			ISP:       stage.VacuumISP,
			Running:   true,
			Thermal:   thermal,
		}
		if len(slots) > 1 {
			// Двигатели расставлены по кольцу вокруг оси.
			e.AngleRad = 2 * math.Pi * float64(idx) / float64(len(slots))
			e.Radius = cfg.EngineRingRadius * 0.5
		}
		e.SetTemperatures(ambientTempK)
		engines[idx] = e
	}
	return engines
}

// InitialRoll возвращает крен на стартовом столе относительно плоскости
// выведения, градусы.
//
// Пусковое устройство ориентировано жёстко, а азимут пуска определяется целевым
// наклонением. Разница между ними и есть угол, на который носитель должен
// довернуться вокруг продольной оси после отрыва.
func (c Config) InitialRoll() float64 {
	launchAzimuth, _ := physics.LaunchAzimuthForInclination(
		c.LaunchLatitude, c.TargetInclination, true,
	)
	return physics.ShortestAngle(c.LaunchPadAzimuth - launchAzimuth)
}

// ThrustAtAltitude возвращает тягу одного двигателя ступени с поправкой
// на противодавление.
//
//	F(h) = F_вак − (F_вак − F_мор) · p(h)/p₀
//
// Зависимость от давления, а не косинусная интерполяция по высоте до 50 км,
// как было раньше.
func ThrustAtAltitude(stage StageConfig, ambientPressure float64) float64 {
	const seaLevelPressure = 101325.0
	ratio := physics.Clamp(ambientPressure/seaLevelPressure, 0, 1)
	return stage.VacuumThrust - (stage.VacuumThrust-stage.SeaLevelThust)*ratio
}

// StarshipConfig возвращает конфигурацию сверхтяжёлого носителя.
//
// Масштаб здесь другой во всём: стартовая масса впятеро больше, тяга вдесятеро,
// диаметр девять метров против трёх с половиной. Тридцать три двигателя на
// первой ступени и шесть на второй, причём вторая ступень несёт двигатели двух
// типов — три с атмосферными соплами и три с вакуумными.
//
// Числа соответствуют третьей версии носителя (2026): высота 124.4 м,
// стартовая масса около 5300 т, двигатели третьего поколения с давлением
// в камере 350 бар. Открытые источники расходятся между собой: стартовую
// массу называют и 5250, и 5300 т, суммарную тягу — от 80.8 до 90.6 МН,
// запас топлива первой ступени — от 3400 до 4050 т. Здесь взят внутренне
// согласованный набор: тяга и удельный импульс считаются моделью из
// давления, геометрии сопла и расхода, а не задаются отдельно, поэтому
// подгонять их под конкретную публикацию бессмысленно.
//
// Обе ступени рассчитаны на возврат, но возврат не моделируется: после
// разделения первая ступень летит баллистически. Поэтому резерв топлива на
// посадку заложен в конфигурацию, но израсходован не будет.
func StarshipConfig() Config {
	return Config{
		Name: "Starship / Super Heavy",

		FirstStage: StageConfig{
			Name:    "Super Heavy (33 × Raptor 3)",
			DryMass: 275000,

			// Три тысячи четыреста тонн — величина, которую называют для
			// третьего блока. Верхняя оценка в четыре тысячи относится
			// к предельной вместимости баков, а не к рабочей заправке.
			FuelMass:    3400000,
			EngineCount: 33,

			// Паспортные величины сверены с установившимся режимом модели:
			// у земли двигатель даёт 2.56 МН при удельном импульсе 321 с,
			// в вакууме — 2.70 МН при 344 с. Публикуемые для третьего
			// поколения 280 тс и 350 с относятся к вакууму и отличаются
			// от расчёта на пару процентов.
			VacuumThrust:  2700000,
			SeaLevelThust: 2560000,
			VacuumISP:     344,
			SeaLevelISP:   321,
			MinThrottle:   0.40,

			// Резерв на возврат и перехват стартовой башней: разворотный
			// импульс, торможение при входе и посадочный импульс. В модели
			// он просто не расходуется, поскольку возврат не реализован.
			FuelReserve: 250000,

			MixtureRatio:       3.6,
			FuelTankPressure:   3.5e5,
			OxTankPressure:     3.8e5,
			FuelPressurantMass: 2600,
			OxPressurantMass:   5200,
			FuelTankHeight:     20.0,
			OxTankHeight:       32.0,
		},

		SecondStage: StageConfig{
			Name: "Starship (3 × Raptor 3 + 3 × Raptor Vacuum)",

			// Полторы тысячи тонн топлива при сухой массе около ста тонн.
			// С прежними значениями (120 / 1200 т) характеристической
			// скорости не хватало примерно на семьсот метров в секунду,
			// и корабль со стотонной нагрузкой не замыкал орбиту —
			// оставался на баллистической дуге с апоцентром пятьсот
			// километров.
			DryMass:     100000,
			FuelMass:    1500000,
			EngineCount: 6,

			// Три двигателя с атмосферным соплом и три с вакуумным.
			// Паспортные величины здесь усреднены по шести двигателям:
			// модель считает каждый отдельно, по своей геометрии сопла.
			VacuumThrust:  2820000,
			SeaLevelThust: 2560000,
			VacuumISP:     358,
			SeaLevelISP:   321,
			MinThrottle:   0.40,
			FuelReserve:   0,

			MixtureRatio:       3.6,
			FuelTankPressure:   3.5e5,
			OxTankPressure:     3.8e5,
			FuelPressurantMass: 900,
			OxPressurantMass:   1800,
			FuelTankHeight:     14.0,
			OxTankHeight:       21.0,
		},

		// Полезная нагрузка внутри корабля, обтекателя как отдельного
		// сбрасываемого элемента нет: створки открываются и закрываются.
		PayloadMass:             100000,
		FairingMass:             0,
		FairingJettisonAltitude: 1e9, // сброса не происходит

		Flaps:     true,
		RCSMoment: 2.5e5,

		Diameter:         9.0,
		EngineRingRadius: 3.6,
		FirstStageLength: 72.3,  // третий блок выше прежнего на метр с лишним
		TotalLength:      124.4, // самая высокая ракета из летавших

		LaunchLatitude:   25.997, // Бока-Чика
		LaunchLongitude:  -97.155,
		LaunchPadAzimuth: 95,

		TargetOrbitAltitude: 400000,
		TargetInclination:   30.0,

		MECOVelocity:         2300,
		MECOMinAltitude:      62000,
		StageSeparationDelay: 3.0,

		// Стартовая тяговооружённость около 1.45 против 1.38 у среднего
		// носителя, но масса впятеро больше и разгон идёт положе: программу
		// приходится растягивать почти вдвое.
		PitchProgramStretch: 1.9,
	}
}

// PeriapsisTarget возвращает целевую высоту перицентра.
//
// Незаданное значение означает круговую орбиту: перицентр совпадает
// с апоцентром.
func (c Config) PeriapsisTarget() float64 {
	if c.TargetPeriapsisAltitude == 0 {
		return c.TargetOrbitAltitude
	}
	return c.TargetPeriapsisAltitude
}

// Suborbital сообщает, что задание не предполагает замкнутой орбиты:
// перицентр лежит в плотных слоях, и корабль вернётся сам.
func (c Config) Suborbital() bool {
	return c.PeriapsisTarget() < atmosphereInterface
}
