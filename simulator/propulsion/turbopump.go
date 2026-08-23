package propulsion

import (
	"math"

	"rocketTelemetrySim/control"
)

// -----------------------------------------------------------------------------
// Турбонасосный агрегат.
//
// Обороты вала — не задаваемая величина, а результат баланса мощности:
//
//	I·ω·dω/dt = P_турбины − P_насосов − P_потерь
//
// Пока турбина мощнее, вал разгоняется; когда мощности не хватает, обороты
// падают, вслед за ними падает напор насосов, расходы компонентов, давление
// в камере и тяга. Именно поэтому раскрутка при запуске занимает секунды,
// а не происходит мгновенно.
// -----------------------------------------------------------------------------

// PumpConfig — параметры одного насоса.
type PumpConfig struct {
	Name string

	// DesignHead — напор при номинальных оборотах, м столба жидкости.
	DesignHead float64

	// DesignSpeed — номинальные обороты, рад/с.
	DesignSpeed float64

	// DesignFlow — номинальный объёмный расход, м³/с.
	DesignFlow float64

	// Efficiency — гидравлический КПД на номинале.
	Efficiency float64

	// SuctionSpecificSpeed — коэффициент, задающий потребный кавитационный
	// запас: NPSHr растёт как квадрат оборотов.
	SuctionSpecificSpeed float64

	// ImpellerArea — площадь крыльчатки, м². Нужна для расчёта осевого усилия.
	ImpellerArea float64
}

// Pump — состояние насоса.
type Pump struct {
	Config PumpConfig

	// InletPressure — давление на входе, Па.
	InletPressure float64

	// OutletPressure — давление на выходе, Па.
	OutletPressure float64

	// Head — развиваемый напор, м.
	Head float64

	// MassFlow — массовый расход, кг/с.
	MassFlow float64

	// Power — потребляемая мощность, Вт.
	Power float64

	// NPSHAvailable — располагаемый кавитационный запас, м.
	NPSHAvailable float64

	// NPSHRequired — потребный кавитационный запас, м.
	NPSHRequired float64

	// CavitationMargin — превышение располагаемого запаса над потребным, м.
	// Отрицательное значение означает кавитацию.
	CavitationMargin float64

	// CavitationSeverity — глубина кавитации, 0…1.
	CavitationSeverity float64

	// EffectiveEfficiency — КПД с учётом кавитации.
	EffectiveEfficiency float64
}

// Update пересчитывает состояние насоса.
//
// Напор подчиняется законам подобия: H ∝ N². При кавитации часть крыльчатки
// работает в паровой каверне, напор и КПД падают.
//
// gasFraction — доля объёма на входе, занятая газом, а не жидкостью (0…1).
// Источник — неосевшее в невесомости топливо (Tank.Settled, см. tank.go):
// заборник вместо жидкости захватывает газ наддува. Складывается с ручным
// GasIngestion, а не подменяет его — это два разных источника одного явления.
func (p *Pump) Update(speed, inletPressure, density, vaporPressure, gravity, gasFraction float64,
	ov control.PumpOverrides) {

	cfg := p.Config

	// Ручное смещение давления на входе — это не подмена показания, а модель
	// падения наддува или сужения магистрали. Дальше оно проходит через весь
	// расчёт: кавитационный запас, напор, расход, давление в камере.
	inletPressure += ov.InletPressureBias.Or(0)
	if inletPressure < 0 {
		inletPressure = 0
	}
	p.InletPressure = inletPressure

	if density <= 0 || cfg.DesignSpeed <= 0 {
		p.Head, p.OutletPressure, p.Power = 0, inletPressure, 0
		return
	}

	// Ускорение свободного падения используется только как масштаб перевода
	// напора в давление; для кавитационного запаса берётся стандартное
	// значение, чтобы величина оставалась сравнимой в полёте и на земле.
	g := gravity
	if g < 1 {
		g = 9.80665
	}

	// Располагаемый кавитационный запас: насколько давление на входе выше
	// давления насыщенных паров компонента.
	p.NPSHAvailable = (inletPressure - vaporPressure) / (density * g)

	// Потребный запас растёт как квадрат оборотов. Повреждение шнека повышает
	// потребный запас, не меняя ни оборотов, ни давления наддува.
	speedRatio := speed / cfg.DesignSpeed
	p.NPSHRequired = cfg.SuctionSpecificSpeed * speedRatio * speedRatio *
		ov.SuctionFactor.Or(1)

	p.CavitationMargin = p.NPSHAvailable - p.NPSHRequired

	// Глубина кавитации: ноль при положительном запасе, единица при полном
	// срыве. Переход плавный — насос деградирует постепенно.
	switch {
	case p.CavitationMargin >= 0:
		p.CavitationSeverity = 0
	case p.NPSHRequired > 0:
		p.CavitationSeverity = math.Min(1, -p.CavitationMargin/math.Max(p.NPSHRequired, 1))
	default:
		p.CavitationSeverity = 0
	}

	// Газ, попавший на вход, действует на насос так же, как паровая каверна:
	// крыльчатка работает в двухфазной среде и теряет напор.
	if gas := math.Max(0, gasFraction) + ov.GasIngestion.Or(0); gas > 0 {
		p.CavitationSeverity = math.Min(1, p.CavitationSeverity+gas)
	}

	// Кавитация срезает и напор, и КПД.
	headFactor := 1 - 0.85*p.CavitationSeverity
	p.EffectiveEfficiency = cfg.Efficiency * (1 - 0.5*p.CavitationSeverity) *
		ov.EfficiencyFactor.Or(1)
	if p.EffectiveEfficiency < 0.02 {
		p.EffectiveEfficiency = 0.02
	}

	p.Head = cfg.DesignHead * speedRatio * speedRatio * headFactor *
		ov.HeadFactor.Or(1)
	if p.Head < 0 {
		p.Head = 0
	}

	p.OutletPressure = inletPressure + p.Head*density*g

	// Потребляемая мощность: полезная работа, делённая на КПД.
	if p.MassFlow > 0 {
		p.Power = p.MassFlow * (p.OutletPressure - p.InletPressure) /
			(density * p.EffectiveEfficiency)
	} else {
		// Насос на закрытую задвижку всё равно потребляет мощность
		// на перемешивание жидкости.
		p.Power = 0.08 * cfg.DesignFlow * density * g * p.Head
	}
	if p.Power < 0 {
		p.Power = 0
	}
}

// -----------------------------------------------------------------------------
// Турбина и вал
// -----------------------------------------------------------------------------

// TurbopumpConfig — параметры агрегата.
type TurbopumpConfig struct {
	// RotorInertia — момент инерции ротора, кг·м².
	RotorInertia float64

	// DesignSpeed — номинальные обороты, рад/с.
	DesignSpeed float64

	// MaxSpeed — предельно допустимые обороты, рад/с.
	MaxSpeed float64

	// GasGeneratorFlow — расход через газогенератор при полностью открытом
	// клапане, кг/с.
	GasGeneratorFlow float64

	// TurbineEnthalpyDrop — располагаемый теплоперепад на турбине, Дж/кг.
	TurbineEnthalpyDrop float64

	// TurbineEfficiency — КПД турбины на номинале.
	TurbineEfficiency float64

	// MechanicalLossCoefficient — коэффициент механических потерь;
	// потери растут как куб оборотов.
	MechanicalLossCoefficient float64

	// BearingHeatCoefficient — тепловыделение в подшипниках.
	BearingHeatCoefficient float64

	// BearingHeatCapacity — теплоёмкость подшипникового узла, Дж/К.
	BearingHeatCapacity float64

	// BearingCooling — теплоотвод от подшипников, Вт/К.
	BearingCooling float64

	// RotorImbalance — остаточный дисбаланс ротора; задаёт штатную вибрацию.
	RotorImbalance float64

	// AxialStiffness — жёсткость упорного подшипника, Н/м.
	AxialStiffness float64

	// ValveRate — скорость перекладки клапанов, доля хода в секунду.
	ValveRate float64

	// GasGenStroke — полный ход клапана газогенератора в долях штатного
	// положения.
	//
	// Единица соответствует номинальному режиму, но механически клапан
	// открывается дальше: у любого двигателя есть запас по форсированию,
	// иначе поднять обороты выше паспортных было бы физически невозможно.
	// Без этого запаса регулятор оборотов упирался бы в полностью открытый
	// клапан и уставка выше номинала не отрабатывалась бы вовсе.
	GasGenStroke float64

	FuelPump PumpConfig
	OxPump   PumpConfig
}

// Turbopump — состояние турбонасосного агрегата.
type Turbopump struct {
	Config TurbopumpConfig

	FuelPump Pump
	OxPump   Pump

	// Speed — угловая скорость вала, рад/с.
	Speed float64

	// AngularAcceleration — угловое ускорение вала, рад/с².
	AngularAcceleration float64

	// TurbinePower — мощность турбины, Вт.
	TurbinePower float64

	// MechanicalLoss — механические потери, Вт.
	MechanicalLoss float64

	// Torque — результирующий момент на валу, Н·м.
	Torque float64

	// TurbineFlow — расход рабочего тела через турбину, кг/с.
	TurbineFlow float64

	// TurbineTemperature — температура на входе турбины, К.
	TurbineTemperature float64

	// BearingTemperature — температура подшипников, К.
	BearingTemperature float64

	// Vibration — вибрация подшипников, g.
	Vibration float64

	// AxialDisplacement — осевое смещение ротора, м.
	AxialDisplacement float64

	// Положения клапанов, 0…1.
	FuelValve   float64
	OxValve     float64
	GasGenValve float64
	BypassValve float64

	// governorIntegral — накопленная составляющая регулятора оборотов.
	//
	// Уставка оборотов отрабатывается клапаном газогенератора, а не подменой
	// оборотов: именно так регулируется тяга настоящего двигателя. Обороты
	// остаются результатом баланса мощности и могут не дойти до уставки,
	// если турбине не хватает мощности.
	governorIntegral float64

	// governorActive сообщает, что оборотами управляет уставка, а не команда
	// на уровень тяги.
	governorActive bool

	// lastValidMeasuredSpeed — последнее достоверное показание датчика
	// оборотов, рад/с. Держится на пропуске связи: регулятор не должен ни
	// тайно подсматривать в истинные обороты, ни пугаться нулевого
	// показания и дёргать клапан.
	lastValidMeasuredSpeed float64

	// valveDelay — накопленное запаздывание приводов, с.
	valveDelay [4]float64
}

// GovernorActive сообщает, работает ли регулятор оборотов по уставке.
func (tp *Turbopump) GovernorActive() bool { return tp.governorActive }

// GovernorCommand возвращает положение клапана газогенератора, потребное для
// выхода на заданные обороты.
//
// Регулятор пропорционально-интегральный: пропорциональная часть отрабатывает
// рассогласование, интегральная удерживает режим. Интегратор ограничен ходом
// клапана, иначе после длительного насыщения он не успевал бы разрядиться
// и обороты проскакивали бы уставку.
//
// measured — показание датчика оборотов (рад/с), а не истинные tp.Speed:
// регулятор реального двигателя знает только то, что ему сказал тахометр.
// На пропуске связи (measured.Valid == false) регулятор держит последнее
// достоверное показание и не двигает интеграл — иначе он либо тайно
// подсмотрел бы в истинные обороты, либо испугался бы нулевого показания
// и рвано задёргал клапаном.
func (tp *Turbopump) GovernorCommand(measured Measurement, target, dt float64) float64 {
	tp.governorActive = true
	if tp.Config.DesignSpeed <= 0 {
		return 0
	}
	const kp, ki = 4.0, 2.0

	stroke := tp.Config.GasGenStroke
	if stroke < 1 {
		stroke = 1
	}

	speed := tp.lastValidMeasuredSpeed
	if measured.Valid {
		speed = measured.Value
		tp.lastValidMeasuredSpeed = speed
	}

	err := (target - speed) / tp.Config.DesignSpeed
	if measured.Valid {
		tp.governorIntegral += ki * err * dt
		tp.governorIntegral = math.Max(0, math.Min(stroke, tp.governorIntegral))
	}

	return clampTo(kp*err+tp.governorIntegral, stroke)
}

// ReleaseGovernor возвращает управление оборотами команде на уровень тяги.
func (tp *Turbopump) ReleaseGovernor() {
	if tp.governorActive {
		tp.governorActive = false
		// Интегратор оставляется равным текущему положению клапана: иначе
		// возврат в штатный режим сопровождался бы скачком тяги.
		tp.governorIntegral = tp.GasGenValve
	}
}

// NewTurbopump создаёт агрегат в остановленном состоянии.
func NewTurbopump(cfg TurbopumpConfig, ambientTemperature float64) *Turbopump {
	return &Turbopump{
		Config:             cfg,
		FuelPump:           Pump{Config: cfg.FuelPump},
		OxPump:             Pump{Config: cfg.OxPump},
		BearingTemperature: ambientTemperature,
		TurbineTemperature: ambientTemperature,
	}
}

// RPM возвращает обороты вала в оборотах в минуту.
func (tp *Turbopump) RPM() float64 { return tp.Speed * 60 / (2 * math.Pi) }

// SpeedFraction возвращает долю от номинальных оборотов.
func (tp *Turbopump) SpeedFraction() float64 {
	if tp.Config.DesignSpeed <= 0 {
		return 0
	}
	return tp.Speed / tp.Config.DesignSpeed
}

// PumpPower возвращает суммарную мощность, потребляемую насосами, Вт.
func (tp *Turbopump) PumpPower() float64 {
	return tp.FuelPump.Power + tp.OxPump.Power
}

// ValveCommand — команды на клапаны.
type ValveCommand struct {
	Fuel   float64
	Ox     float64
	GasGen float64
	Bypass float64
}

// TrackValves отрабатывает команды с конечной скоростью перекладки.
//
// Клапан не переставляется мгновенно: именно это ограничение задаёт темп
// изменения тяги и делает запуск двигателя переходным процессом.
func (tp *Turbopump) TrackValves(cmd ValveCommand, dt float64, ov control.ValveOverrides) {
	rate := tp.Config.ValveRate
	if rate <= 0 {
		rate = 2.0
	}
	rate *= ov.RateOpt.Or(1)

	ggStroke := tp.Config.GasGenStroke
	if ggStroke < 1 {
		ggStroke = 1
	}

	targets := [4]float64{cmd.Fuel, cmd.Ox, cmd.GasGen, cmd.Bypass}
	actual := [4]*float64{&tp.FuelValve, &tp.OxValve, &tp.GasGenValve, &tp.BypassValve}
	over := [4]control.ValveOverride{ov.Fuel, ov.Ox, ov.GasGen, ov.Bypass}
	stroke := [4]float64{1, 1, ggStroke, 1}

	for i := range targets {
		o := over[i]

		// Заклинивший привод остаётся там, где был. Команда при этом
		// продолжает поступать — расхождение команды и факта и есть признак
		// отказа.
		if o.Stuck || ov.AllStuck {
			continue
		}

		target := clampTo(o.Command.Or(targets[i]), stroke[i])
		if max := o.MaxOpen.Or(stroke[i]); target > max {
			target = clampTo(max, stroke[i])
		}

		// Запаздывание привода: команда начинает отрабатываться не сразу.
		if delay := o.Delay.Or(0); delay > 0 {
			if math.Abs(target-*actual[i]) > 1e-9 {
				tp.valveDelay[i] += dt
				if tp.valveDelay[i] < delay {
					continue
				}
			} else {
				tp.valveDelay[i] = 0
			}
		} else {
			tp.valveDelay[i] = 0
		}

		step := rate * o.RateFactor.Or(1) * dt
		*actual[i] = approach(*actual[i], target, step)
	}
}

// UpdateShaft интегрирует уравнение вращения вала.
//
// turbineInletTemperature — температура газа перед турбиной, К.
func (tp *Turbopump) UpdateShaft(dt, turbineInletTemperature float64,
	ov control.TurbopumpOverrides) {

	cfg := tp.Config

	// Расход через турбину задаётся клапаном газогенератора за вычетом
	// перепуска мимо турбины.
	tp.TurbineFlow = cfg.GasGeneratorFlow * tp.GasGenValve * (1 - 0.8*tp.BypassValve)

	fraction := tp.SpeedFraction()
	tp.TurbineTemperature = turbineInletTemperature

	// Момент турбины считается напрямую, а не как мощность, делённая на
	// обороты. Деление на скорость даёт бесконечный момент при запуске,
	// тогда как реальная турбина развивает конечный пусковой момент —
	// примерно вдвое больше номинального.
	designTorque := 0.0
	if cfg.DesignSpeed > 0 {
		nominalPower := cfg.GasGeneratorFlow * cfg.TurbineEnthalpyDrop * cfg.TurbineEfficiency
		designTorque = nominalPower / cfg.DesignSpeed
	}
	flowFraction := 0.0
	if cfg.GasGeneratorFlow > 0 {
		flowFraction = tp.TurbineFlow / cfg.GasGeneratorFlow
	}
	turbineTorque := designTorque * flowFraction * turbineTorqueFactor(fraction) *
		ov.TurbineTorqueFactor.Or(1)

	// Обесточенный или остановленный агрегат газ на турбину не получает.
	if ov.Unpowered {
		turbineTorque = 0
		tp.TurbineFlow = 0
	}

	tp.TurbinePower = turbineTorque * tp.Speed

	// Механические потери: трение в подшипниках и уплотнениях, растут как куб
	// оборотов.
	tp.MechanicalLoss = cfg.MechanicalLossCoefficient * math.Pow(math.Abs(tp.Speed), 3) *
		ov.MechanicalLoadFactor.Or(1)

	// Сопротивляющий момент насосов и потерь.
	const minSpeed = 50.0
	speed := math.Max(tp.Speed, minSpeed)
	resistTorque := (tp.PumpPower() + tp.MechanicalLoss) / speed

	tp.Torque = turbineTorque - resistTorque

	inertia := cfg.RotorInertia
	if inertia <= 0 {
		inertia = 1
	}

	// Обороты считаются полунеявно, а не явным приращением.
	//
	// Контур вала быстрый: постоянная времени ротора — сотые доли секунды,
	// а шаг интегрирования полёта в разы больше. Явная схема при таком
	// соотношении не просто теряет точность — она расходится: ротор
	// проскакивает равновесие, следующий шаг возвращает его с перелётом
	// в другую сторону, и качание нарастает, пока двигатель не погаснет.
	// Именно так и вело себя изделие с давлением в камере в тридцать пять
	// мегапаскалей: обороты метались между двадцатью тремя и сорока тремя
	// тысячами, а тяга — между единицей и ста тридцатью меганьютонами.
	//
	// Момент падает с ростом оборотов по двум причинам сразу: турбина при
	// раскрутке отдаёт меньше, а насосы забирают больше. Наклон этой
	// зависимости и есть жёсткость контура; подставленная в знаменатель,
	// она делает схему устойчивой при любом шаге и не меняет положение
	// равновесия — в установившемся режиме момент равен нулю, и поправка
	// обращается в ноль вместе с ним.
	stiffness := 2 * math.Abs(resistTorque) / speed
	if cfg.DesignSpeed > 0 {
		stiffness += math.Abs(turbineTorque) / cfg.DesignSpeed
	}

	tp.AngularAcceleration = tp.Torque / (inertia + dt*stiffness)

	// Ограничение темпа раскрутки, если оно задано оператором.
	if limit := ov.SpinRateLimit.Or(0); limit > 0 {
		tp.AngularAcceleration = math.Max(-limit, math.Min(limit, tp.AngularAcceleration))
	}

	// Заклинивший вал не вращается: весь момент турбины уходит в трение.
	// Насосы при этом перестают развивать напор, и подача прекращается —
	// последствия считает модель, а не интерфейс.
	if ov.Locked {
		tp.AngularAcceleration = 0
		tp.MechanicalLoss += math.Abs(tp.TurbinePower)
		tp.Torque = 0
		return
	}

	tp.Speed += tp.AngularAcceleration * dt
	if tp.Speed < 0 {
		tp.Speed = 0
		tp.AngularAcceleration = 0
	}

	maxSpeed := cfg.MaxSpeed
	if maxSpeed > 0 && tp.Speed > maxSpeed {
		tp.Speed = maxSpeed
		tp.AngularAcceleration = 0
	}

	// Прямая подмена оборотов. Баланс мощности при этом не выполняется,
	// и режим отмечен в интерфейсе как отладочный.
	if ov.SpeedDirect.Active {
		tp.Speed = math.Max(0, ov.SpeedDirect.V)
		tp.AngularAcceleration = 0
	}
}

// turbineTorqueFactor возвращает относительный момент турбины в зависимости
// от доли номинальных оборотов.
//
// Момент максимален на заторможенном роторе и линейно спадает по мере
// раскрутки: чем быстрее вращаются лопатки, тем меньше относительная скорость
// набегающего газа. При двукратном превышении номинальных оборотов момент
// обращается в ноль — это естественный предел раскрутки.
//
// Пусковой момент вдвое выше номинального, что и обеспечивает раскрутку
// агрегата за секунды, а не мгновенно и не бесконечно долго.
func turbineTorqueFactor(fraction float64) float64 {
	if fraction < 0 {
		fraction = 0
	}
	f := 2 - fraction
	if f < 0 {
		return 0
	}
	if f > 2 {
		return 2
	}
	return f
}

// UpdateMechanical считает вибрацию, температуру подшипников и осевое смещение.
func (tp *Turbopump) UpdateMechanical(dt, coolantTemperature float64,
	ov control.TurbopumpOverrides) {

	cfg := tp.Config

	// Штатная вибрация от остаточного дисбаланса растёт как квадрат оборотов.
	fraction := tp.SpeedFraction()
	base := cfg.RotorImbalance * fraction * fraction

	// Кавитация добавляет вибрацию скачком: схлопывание паровых каверн бьёт
	// по крыльчатке. Это главный диагностический признак кавитации.
	cavitation := math.Max(tp.FuelPump.CavitationSeverity, tp.OxPump.CavitationSeverity)
	tp.Vibration = base + 25*cavitation*cavitation + ov.VibrationBias.Or(0)

	// Заклинивший вал бьёт по опорам сильнее любой кавитации.
	if ov.Locked {
		tp.Vibration += 40
	}

	// Температура подшипников: трение против отвода тепла компонентом.
	heat := cfg.BearingHeatCoefficient * fraction * fraction * (1 + 0.3*tp.Vibration)
	capacity := cfg.BearingHeatCapacity
	if capacity <= 0 {
		capacity = 1000
	}
	cooling := cfg.BearingCooling
	if cooling <= 0 {
		cooling = 50
	}
	cooling *= ov.BearingCoolingFactor.Or(1)
	if cooling < 1 {
		cooling = 1
	}

	equilibrium := coolantTemperature + heat/cooling
	tau := capacity / cooling
	tp.BearingTemperature = equilibrium + (tp.BearingTemperature-equilibrium)*math.Exp(-dt/tau)

	if ov.BearingTempDirect.Active {
		tp.BearingTemperature = ov.BearingTempDirect.V
	}

	// Осевое усилие от неуравновешенного перепада давления на крыльчатках.
	stiffness := cfg.AxialStiffness
	if stiffness <= 0 {
		stiffness = 1e8
	}
	force := (tp.OxPump.OutletPressure-tp.OxPump.InletPressure)*cfg.OxPump.ImpellerArea -
		(tp.FuelPump.OutletPressure-tp.FuelPump.InletPressure)*cfg.FuelPump.ImpellerArea
	tp.AxialDisplacement = force / stiffness
}

// CavitationFlowRipple возвращает относительную амплитуду пульсаций расхода,
// вызванных кавитацией.
//
// Схлопывание паровых каверн модулирует расход через насос, а через него —
// давление в камере и тягу. Так кавитация из чисто насосного явления
// превращается в пульсации всей двигательной установки.
func (tp *Turbopump) CavitationFlowRipple() float64 {
	severity := math.Max(tp.FuelPump.CavitationSeverity, tp.OxPump.CavitationSeverity)
	return 0.25 * severity
}

func approach(current, target, step float64) float64 {
	if step <= 0 {
		return target
	}
	d := target - current
	if math.Abs(d) <= step {
		return target
	}
	if d > 0 {
		return current + step
	}
	return current - step
}

func clamp01(v float64) float64 { return clampTo(v, 1) }

// clampTo ограничивает величину диапазоном от нуля до полного хода.
func clampTo(v, stroke float64) float64 {
	if v < 0 {
		return 0
	}
	if v > stroke {
		return stroke
	}
	return v
}
