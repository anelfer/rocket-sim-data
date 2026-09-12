package simulator

import (
	"math/rand"

	"rocketTelemetrySim/control"
	"rocketTelemetrySim/simulator/env"
	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/orbit/sensing"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/propulsion"
)

// -----------------------------------------------------------------------------
// Навигация первой ступени по показаниям приборов.
//
// Ровно та же схема, что уже работает у корабля (Simulation.step,
// sensedNavState), перенесённая на бустер: канал положения и канал скорости
// в ECI, канал ориентации и канал остатка топлива. Настройки каналов взяты
// БЕЗ изменений (sensing.DefaultPositionSensor и соседние) — это то же самое
// приборное хозяйство того же носителя, и заводить бустеру собственные,
// более удобные характеристики значило бы подгонять условия задачи под
// ответ.
//
// Что остаётся истинным и почему:
//
//   - масса, удельный импульс, число зажжённых камер, время — это не
//     измерения, а внутренний учёт борта (заправка минус проинтегрированный
//     расход, паспорт двигателя, собственная команда, бортовые часы);
//   - вся физика — интегрирование, аэродинамический момент, проверка
//     касания воды, промах в точке касания. Промах особенно: он и есть
//     оценка работы наведения, и считать его по показаниям того же
//     наведения значило бы позволить ему поставить себе оценку самому.
// -----------------------------------------------------------------------------

// boosterSensorStreamSalt — произвольная, но ФИКСИРОВАННАЯ добавка, которой
// поток шума приборов бустера разводится с потоком приборов корабля из
// одного и того же зерна. Значение роли не играет, важно лишь чтобы оно не
// менялось: от него зависит воспроизводимость прогонов.
const boosterSensorStreamSalt = 0x9E3779B97F4A7C15

// initSensors создаёт измерительные каналы бустера в момент отделения.
func (b *Booster) initSensors(seed int64, position, velocity physics.Vec3,
	orientation physics.Quaternion) {

	// Свой поток шума, разведённый от потока корабля тем же перемешиванием,
	// что Simulation разводит физику и приборы (см. splitMix64).
	//
	// Разводится именно от ЗЕРНА, а не броском общего генератора. Броском
	// было бы короче на строку, но он сдвигает общую последовательность —
	// и на том же зерне меняются показания приборов КОРАБЛЯ, который в
	// этот момент ещё летит. Его траектория поехала бы от появления
	// бустера, то есть по причине, к нему не относящейся, а
	// воспроизводимость прогонов по зерну — то, на чём здесь держится
	// вся диагностика. Ровно то же соображение записано и у самой
	// Simulation, где sensorRng разводится от seed перемешиванием, а не
	// броском s.rng.
	b.sensorRng = rand.New(rand.NewSource(int64(splitMix64(uint64(seed) ^ boosterSensorStreamSalt))))

	// Каналы положения и скорости обязаны стартовать УЖЕ показывающими
	// текущее значение: в момент отделения модуль положения порядка радиуса
	// Земли, скорости — километры в секунду, и канал, начатый с нуля, первые
	// секунды показывал бы не отсчёт, а собственный переходный процесс
	// (см. propulsion.NewSensorAt).
	b.positionSensor = sensing.NewVectorSensorAt(sensing.DefaultPositionSensor(), position)
	b.velocitySensor = sensing.NewVectorSensorAt(sensing.DefaultVelocitySensor(), velocity)
	b.sensedPosition = position
	b.sensedVelocity = velocity
	b.navPosition = position
	b.navVelocity = velocity

	// Каналу ориентации прайминг не нужен: его вход — вектор ошибки
	// относительно самой себя, то есть тождественный ноль (см.
	// sensing.AttitudeSensor.UpdateWith).
	b.attitudeSensor = sensing.NewAttitudeSensor(sensing.DefaultAttitudeSensor())
	b.sensedOrientation = orientation

	prop := b.propulsion.PropellantMass()
	b.fuelSensor = propulsion.NewSensorAt(propulsion.DefaultPropellantSensor(prop), prop)
	b.sensedFuelMass = prop
}

// updateSensors снимает показания всех каналов — РОВНО ОДИН РАЗ за такт.
//
// Не внутри вспомогательных функций наведения: часть из них вызывается по
// два-три раза за такт по независимым причинам (landingAuthorityConstrainedThrust
// — и из выбора группы камер, и из расчёта дросселя), и повторный опрос
// сдвигал бы внутреннее состояние канала несколько раз за один и тот же
// интервал времени, искажая и постоянную времени, и статистику выбросов.
func (b *Booster) updateSensors(dt float64) {
	if b.sensorRng == nil {
		return
	}
	none := control.SensorOverrides{}

	if v, valid := b.fuelSensor.UpdateWith(b.propulsion.PropellantMass(), dt, b.sensorRng, none); valid {
		b.sensedFuelMass = v
	}
	if m := b.positionSensor.UpdateWith(b.state.Position, dt, b.sensorRng, none); m.Valid {
		b.sensedPosition = m.Value
	}
	if m := b.velocitySensor.UpdateWith(b.state.Velocity, dt, b.sensorRng, none); m.Valid {
		b.sensedVelocity = m.Value
	}
	if m := b.attitudeSensor.UpdateWith(b.attitude.Orientation, dt, b.sensorRng, none); m.Valid {
		b.sensedOrientation = m.Orientation
	}
}

// sensedNavState собирает состояние навигации из ПОКАЗАНИЙ — то, чем
// распоряжается наведение бустера.
//
// Ветер вычисляется в точке показанного датчиком положения, а не истинного:
// бортовая модель атмосферы видит обстановку там, где, по её мнению,
// находится ступень. Тот же приём и то же обоснование, что в
// Simulation.sensedNavState.
//
// Масса, располагаемая тяга, удельный импульс, время и фаза берутся из
// истинного состояния — они не измеряются навигационными датчиками (см.
// заголовок файла). Остальные поля NewNavState выводит из положения и
// скорости сама, поэтому высота, скорости, местные оси, скоростной напор и
// элементы орбиты становятся «показанными» автоматически.
func (b *Booster) sensedNavState(trueNav orbit.NavState, wind env.WindModel) orbit.NavState {
	position := b.navPosition
	velocity := b.navVelocity
	altitude := position.Norm() - physics.EarthRadius

	var windVel physics.Vec3
	if wind != nil {
		windVel = wind.Velocity(position, altitude, b.elapsed)
	}
	return orbit.NewNavState(position, velocity, windVel,
		trueNav.Mass, trueNav.AvailableThrust, trueNav.ISP, trueNav.Time, trueNav.Phase)
}

// sensedThrustDirection — направление тяги, каким его ВИДИТ борт: по
// показанию гироскопа, а не по истинной ориентации корпуса.
//
// Отклонение камер здесь не учитывается намеренно: угол качания — это
// собственная команда автопилота (он её знает точно), но применена она к
// ориентации, которую борт знает лишь с точностью гироскопа, и определяющая
// неопределённость здесь именно вторая. Величина используется как ПРИЗНАК
// («помогает ли тяга коррекции промаха», см. updatePhase), а не как точный
// вектор, и для признака этой точности достаточно.
func (b *Booster) sensedThrustDirection() physics.Vec3 {
	return b.sensedOrientation.Rotate(physics.Vec3{X: 1})
}

// sensedAttitudeIn — углы корпуса в местных осях по показанию гироскопа.
func (b *Booster) sensedAttitudeIn(frame physics.LocalFrame) physics.Attitude {
	return b.sensedOrientation.AttitudeIn(frame)
}

// -----------------------------------------------------------------------------
// Навигационный фильтр.
//
// Отдавать наведению СЫРОЕ показание датчика неправильно, и дело не в
// аккуратности. Канал положения шумит на пятнадцать метров, канал скорости
// на метр в секунду, оба отстают на 0.3 с, обновляются раз в 0.1 с и
// изредка пропадают на полсекунды. Контур посадки работает с постоянной
// времени в доли секунды, и такой вход он отрабатывает как реальное
// движение: измерено — скорость касания выросла с 2.3 до 45 м/с при
// НЕИЗМЕННОМ законе наведения, просто от подстановки сырых показаний
// вместо истины.
//
// На настоящем борту между приборами и наведением всегда стоит навигация:
// счисление по инерциальным измерениям, поправляемое редкими и шумными
// внешними отсчётами. Здесь то же самое в простейшей честной форме —
// фильтр с постоянными коэффициентами (комплементарный):
//
//   - счисление: положение и скорость продвигаются по ИЗВЕСТНОМУ борту
//     ускорению — тяга (борт знает её по давлению в камерах и знает, куда
//     повёрнут корпус) плюс притяжение. Это быстро и без шума, но копит
//     ошибку;
//   - коррекция: результат подтягивается к показаниям приборов с
//     коэффициентами, заданными полосой фильтра. Это медленно и шумно, но
//     не копит ошибку.
//
// Сумма даёт то, чего нет ни у одного из слагаемых по отдельности:
// гладкую оценку без накопления ошибки. Ветра и сопротивления счисление не
// знает — их невязку и снимает коррекция.
// -----------------------------------------------------------------------------

// navFilterBandwidth — полоса навигационного фильтра, рад/с.
//
// Выбрана из характеристик самих каналов, а не подбором: канал положения
// обновляется раз в UpdateInterval=0.1 с и имеет постоянную времени 0.3 с,
// то есть содержит осмысленную информацию до частот порядка 1/0.3 ≈ 3 рад/с.
// Фильтр берёт от приборов долю ниже этой границы, оставляя более быстрые
// движения счислению. Половина рад/с — с запасом ниже и одновременно
// заметно выше полосы контура наведения (см. gfoldTrack, ω ≤ Bandwidth/5),
// так что навигация не становится самым медленным звеном.
const navFilterBandwidth = 0.5

// navFilterDamping — относительное демпфирование фильтра. Единица —
// критическое: оценка сходится к показаниям без перерегулирования, которое
// в противном случае само стало бы источником колебаний.
const navFilterDamping = 1.0

// updateNavigation продвигает навигационное решение на такт.
//
// specificForce — кажущееся ускорение в осях ECI: то, что измеряет
// акселерометр, — ВСЕ силы, кроме тяготения, делённые на массу. И тяга, и
// аэродинамика; притяжение — нет, свободно падающий акселерометр
// показывает ноль.
//
// Включать сюда аэродинамику обязательно. Счисление по одной лишь тяге
// систематически ошибается на всё сопротивление, а на входе в плотные слои
// это десятки м/с²; фильтр такую ошибку не убирает, а лишь размазывает —
// установившаяся ошибка оценки скорости при постоянной невязке ускорения
// равна ей же, делённой на полосу фильтра, то есть те же десятки м/с.
func (b *Booster) updateNavigation(dt float64, specificForce physics.Vec3) {
	if b.sensorRng == nil || dt <= 0 {
		return
	}

	// Счисление: известное ускорение плюс притяжение в оценённой точке.
	accel := specificForce.Add(physics.GravityAcceleration(b.navPosition))
	b.navPosition = b.navPosition.
		Add(b.navVelocity.Scale(dt)).
		Add(accel.Scale(dt * dt / 2))
	b.navVelocity = b.navVelocity.Add(accel.Scale(dt))

	// Коррекция по приборам. Коэффициенты — стандартные для фильтра
	// второго порядка: kp = 2ζω, kv = ω².
	w := navFilterBandwidth
	kp := 2 * navFilterDamping * w * dt
	kv := w * w * dt
	if kp > 1 {
		kp = 1
	}
	if kv > 1 {
		kv = 1
	}

	posErr := b.sensedPosition.Sub(b.navPosition)
	velErr := b.sensedVelocity.Sub(b.navVelocity)
	b.navPosition = b.navPosition.Add(posErr.Scale(kp))
	b.navVelocity = b.navVelocity.
		Add(posErr.Scale(kv)).
		Add(velErr.Scale(kp))

	if !b.navPosition.IsFinite() || !b.navVelocity.IsFinite() {
		// Расходящаяся оценка бесполезна и опасна: возвращаемся к
		// последнему достоверному показанию, а не продолжаем считать по
		// нечислам.
		b.navPosition, b.navVelocity = b.sensedPosition, b.sensedVelocity
	}
}
