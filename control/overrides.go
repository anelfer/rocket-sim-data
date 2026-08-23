// Package control описывает ручные воздействия на физическую модель.
//
// Пакет намеренно не зависит ни от simulator, ни от propulsion: он описывает
// только команды и их результат — набор поправок, которые физическая модель
// обязана учесть при расчёте. Направление зависимости одностороннее,
// propulsion импортирует control, но не наоборот.
//
// Главный принцип: воздействие не подменяет телеметрию. Оно меняет вход
// физической модели, а всё остальное модель считает сама. Команда «поднять
// обороты» открывает клапан газогенератора, а не пишет новое число в поле
// оборотов; давление за насосом, расход, соотношение компонентов, давление
// в камере и тяга изменяются потому, что так работает модель.
package control

import "math"

// -----------------------------------------------------------------------------
// Необязательное значение
// -----------------------------------------------------------------------------

// Opt — величина, которая может быть не задана.
//
// Нулевое значение означает «не задано», поэтому пустая структура поправок
// не меняет поведение модели ни в одном месте. Это важнее краткости: забытое
// поле не должно молча включать воздействие.
type Opt struct {
	Active bool
	V      float64
}

// Set возвращает заданное значение.
func Set(v float64) Opt { return Opt{Active: true, V: v} }

// Or возвращает заданное значение либо штатное, если воздействия нет.
func (o Opt) Or(auto float64) float64 {
	if !o.Active || math.IsNaN(o.V) {
		return auto
	}
	return o.V
}

// Scale умножает штатное значение на коэффициент, если он задан.
func (o Opt) Scale(nominal float64) float64 {
	if !o.Active || math.IsNaN(o.V) {
		return nominal
	}
	return nominal * o.V
}

// -----------------------------------------------------------------------------
// Поправки к модели
// -----------------------------------------------------------------------------

// PumpOverrides — воздействия на отдельный насос.
type PumpOverrides struct {
	// HeadFactor — множитель развиваемого напора. Износ или повреждение
	// крыльчатки снижает напор при тех же оборотах.
	HeadFactor Opt

	// EfficiencyFactor — множитель гидравлического КПД. Падение КПД
	// увеличивает потребляемую мощность и нагружает вал.
	EfficiencyFactor Opt

	// InletPressureBias — добавка к давлению на входе, Па. Отрицательная
	// величина имитирует падение наддува или засорение магистрали.
	InletPressureBias Opt

	// SuctionFactor — множитель потребного кавитационного запаса.
	// Повреждение шнека повышает потребный запас и вводит насос в кавитацию
	// при штатном давлении наддува.
	SuctionFactor Opt

	// GasIngestion — доля газа на входе, 0…1. Прорыв газа наддува в тракт
	// срывает работу насоса так же, как кавитация.
	GasIngestion Opt

	// SpeedDirect — прямая подмена оборотов (отладочный режим), рад/с.
	SpeedDirect Opt
}

// TurbopumpOverrides — воздействия на турбонасосный агрегат.
type TurbopumpOverrides struct {
	FuelPump PumpOverrides
	OxPump   PumpOverrides

	// SpeedTarget — заданные обороты вала, рад/с.
	//
	// Это не подмена оборотов, а уставка регулятора: он открывает или
	// прикрывает клапан газогенератора, добиваясь заданных оборотов.
	// Фактические обороты определяются балансом мощности и могут не совпасть
	// с уставкой, если турбине не хватает мощности или сработало ограничение.
	SpeedTarget Opt

	// SpinRateLimit — ограничение углового ускорения вала, рад/с².
	SpinRateLimit Opt

	// Locked означает заклинивший вал: обороты фиксируются, момент турбины
	// уходит в трение.
	Locked bool

	// Unpowered означает обесточенный или остановленный агрегат: газ на
	// турбину не подаётся, вал тормозится нагрузкой насосов.
	Unpowered bool

	// TurbineTorqueFactor — множитель момента турбины.
	TurbineTorqueFactor Opt

	// MechanicalLoadFactor — множитель механических потерь.
	MechanicalLoadFactor Opt

	// BearingCoolingFactor — множитель теплоотвода от подшипников.
	BearingCoolingFactor Opt

	// VibrationBias — добавка к вибрации, g.
	VibrationBias Opt

	// Прямые подмены состояния (отладочный режим).
	SpeedDirect       Opt
	BearingTempDirect Opt

	Valves ValveOverrides
}

// ValveOverrides — воздействия на клапаны агрегата.
type ValveOverrides struct {
	Fuel     ValveOverride
	Ox       ValveOverride
	GasGen   ValveOverride
	Bypass   ValveOverride
	Cooling  ValveOverride
	FuelPrs  ValveOverride
	OxPrs    ValveOverride
	RateOpt  Opt // общий множитель скорости перекладки
	AllStuck bool
}

// ValveOverride — воздействие на один клапан.
type ValveOverride struct {
	// Command — принудительное командное положение, 0…1.
	Command Opt

	// MaxOpen — ограничение максимального открытия, 0…1.
	MaxOpen Opt

	// RateFactor — множитель скорости перекладки.
	RateFactor Opt

	// Stuck фиксирует фактическое положение: привод заклинил.
	Stuck bool

	// Delay — задержка реакции привода, с.
	Delay Opt

	// SensorBias — смещение показаний датчика положения. На физику не влияет,
	// расходится только показание — именно так выглядит отказ датчика.
	SensorBias Opt
}

// TankOverrides — воздействия на бак и магистраль.
type TankOverrides struct {
	// PressurantValve — принудительное положение клапана наддува, 0…1.
	PressurantValve Opt

	// PressurantFlowFactor — множитель расхода газа наддува.
	PressurantFlowFactor Opt

	// LeakRate — расход утечки компонента, кг/с.
	LeakRate Opt

	// LineBlockage — доля перекрытия магистрали, 0…1. Потери давления растут
	// как квадрат сужения.
	LineBlockage Opt

	// LineRuptured означает разрыв магистрали: подача прекращается.
	LineRuptured bool

	// SloshExcitationFactor, SloshDampingFactor — множители возбуждения
	// и демпфирования колебаний жидкости.
	SloshExcitationFactor Opt
	SloshDampingFactor    Opt

	// Прямые подмены состояния (отладочный режим).
	MassDirect        Opt
	PressureDirect    Opt
	TemperatureDirect Opt

	// SettledDirect — принудительная доля осадки топлива, 0…1. Не сам манёвр
	// осадки (импульс РСУ перед повторным зажигом), а способ задать его
	// результат — реальной модели ullage burn в симуляторе нет.
	SettledDirect Opt
}

// ChamberOverrides — воздействия на камеру сгорания.
type ChamberOverrides struct {
	// InjectorFuelAreaFactor, InjectorOxAreaFactor — множители площади
	// форсунок. Засорение уменьшает площадь, поднимает перепад и меняет O/F.
	InjectorFuelAreaFactor Opt
	InjectorOxAreaFactor   Opt

	// CStarEfficiencyFactor — множитель полноты сгорания.
	CStarEfficiencyFactor Opt

	// StabilityMarginBias — добавка к запасу устойчивости. Отрицательная
	// величина раскачивает камеру.
	StabilityMarginBias Opt

	// OscillationFactor — множитель амплитуды пульсаций.
	OscillationFactor Opt

	// OscillationFrequency — принудительная частота пульсаций, Гц.
	OscillationFrequency Opt

	// TemperatureSpreadBias — добавка к температурной неоднородности, К.
	TemperatureSpreadBias Opt

	// HeatFluxFactor — множитель теплового потока в стенки.
	HeatFluxFactor Opt

	// Extinguish принудительно гасит двигатель.
	Extinguish bool

	// Прямые подмены состояния (отладочный режим).
	PressureDirect    Opt
	TemperatureDirect Opt
}

// NozzleOverrides — воздействия на сопло и охлаждающий тракт.
type NozzleOverrides struct {
	// CoolingFlowFactor — множитель расхода охладителя.
	CoolingFlowFactor Opt

	// CoolingConductanceFactor — множитель теплоотдачи в тракт. Засорение
	// каналов снижает теплоотвод, не меняя расход через форсунки.
	CoolingConductanceFactor Opt

	// CoolingDropFactor — множитель потерь давления в тракте.
	CoolingDropFactor Opt

	// CoolantInletTemp — принудительная температура охладителя на входе, К.
	CoolantInletTemp Opt

	// ErosionRateFactor — множитель скорости эрозии горловины.
	ErosionRateFactor Opt

	// ThroatAreaDirect — прямая подмена площади горловины, м².
	ThroatAreaDirect Opt

	// HotSpotStation — участок стенки с местным ухудшением охлаждения.
	HotSpotStation string

	// HotSpotFactor — множитель теплоотвода на этом участке.
	HotSpotFactor Opt
}

// SensorOverrides — воздействия на измерительный тракт.
//
// Датчики не влияют на физику: они лишь искажают то, что видит оператор.
// Разделение фактического и измеренного значения — часть требований.
type SensorOverrides struct {
	// NoiseFactor — множитель шума.
	NoiseFactor Opt

	// DriftFactor — множитель дрейфа нуля.
	DriftFactor Opt

	// Frozen замораживает показания на последнем значении.
	Frozen bool

	// Dead означает отсутствие данных.
	Dead bool
}

// EngineOverrides — полный набор поправок для одного двигателя.
type EngineOverrides struct {
	Turbopump TurbopumpOverrides
	Chamber   ChamberOverrides
	Nozzle    NozzleOverrides
	Sensors   SensorOverrides

	// ThrottleOverride — принудительный уровень тяги, 0…1.
	ThrottleOverride Opt

	// MixtureTrimOverride — принудительная поправка соотношения компонентов.
	MixtureTrimOverride Opt

	// ForceShutdown немедленно выключает двигатель.
	ForceShutdown bool

	// ForceIgnition принудительно запускает двигатель.
	ForceIgnition bool
}

// ControlOverrides — воздействия на рулевой тракт носителя.
//
// Управление ориентацией — единственное, что удерживает статически
// неустойчивый корпус на траектории. Возможность его сломать нужна именно
// поэтому: отказ привода на активном участке разворачивает ракету поперёк
// потока за секунды, и увидеть это на модели важнее, чем на бумаге.
type ControlOverrides struct {
	// GimbalLimitFactor — множитель предельного хода привода.
	GimbalLimitFactor Opt

	// GimbalRateFactor — множитель скорости перекладки.
	GimbalRateFactor Opt

	// BandwidthFactor — множитель полосы пропускания автопилота.
	BandwidthFactor Opt

	// DampingFactor — множитель демпфирования автопилота.
	DampingFactor Opt

	// GimbalBias — постоянное смещение привода, рад. Заклинивший в отклонённом
	// положении привод создаёт непрерывный уводящий момент.
	GimbalBias Opt

	// RCSFactor — множитель момента двигателей ориентации.
	RCSFactor Opt

	// Frozen фиксирует привод в текущем положении: команды не отрабатываются.
	Frozen bool

	// Dead обесточивает рулевой тракт: управляющего момента нет вовсе.
	Dead bool
}

// Overrides — поправки ко всей двигательной установке ступени.
type Overrides struct {
	// Engine — воздействия, действующие на все двигатели ступени.
	Engine EngineOverrides

	// PerEngine — воздействия, адресованные конкретным двигателям.
	//
	// Каждый двигатель блока — отдельное изделие со своим турбонасосом,
	// поэтому остановить один агрегат из девяти физически возможно, и модель
	// обязана это различать. Значение уже включает в себя общие воздействия:
	// адресная команда перекрывает общую для того же параметра.
	PerEngine map[string]EngineOverrides

	FuelTank TankOverrides
	OxTank   TankOverrides

	// Control — воздействия на рулевой тракт. Общие для ступени: автопилот
	// один на носитель, а не по одному на двигатель.
	Control ControlOverrides

	// Active сообщает, есть ли хотя бы одно активное воздействие.
	Active bool

	// Unsafe сообщает, что среди активных воздействий есть выходящие
	// за допустимые пределы.
	Unsafe bool

	// DirectOverride сообщает, что применена прямая подмена состояния
	// и физическая причинность на соответствующих величинах нарушена.
	DirectOverride bool
}

// ForEngine возвращает поправки для двигателя с заданным идентификатором.
func (o Overrides) ForEngine(id string) EngineOverrides {
	if e, ok := o.PerEngine[id]; ok {
		return e
	}
	return o.Engine
}

// Targets возвращает идентификаторы двигателей, которым адресованы
// персональные воздействия.
func (o Overrides) Targets() []string {
	out := make([]string, 0, len(o.PerEngine))
	for id := range o.PerEngine {
		out = append(out, id)
	}
	return out
}
