package simulator

import (
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Управляющие поверхности как исполнительный орган.
//
// Плавники и решётчатые рули отличаются от качания камер тем, что работают
// от потока, а не от тяги. На активном участке они бесполезны — там нечему
// создавать напор, — зато на входе в атмосферу это единственный орган
// управления: двигатели выключены, а тормозить и держать угол атаки надо.
//
// Отсюда и особенность контура: располагаемый момент здесь не постоянен,
// а растёт со скоростным напором. В разрежённых слоях раскрытые плавники
// почти ничего не дают, ниже — становятся мощнее любого другого органа
// управления.
//
// Угол отсчитывается от прижатого положения: ноль — плавник сложен вдоль
// борта, предел — раскрыт поперёк потока.
// -----------------------------------------------------------------------------

// SurfaceSet — набор управляющих поверхностей и их приводы.
type SurfaceSet struct {
	Surfaces []physics.ControlSurface

	// Deflection — фактические углы отклонения, рад.
	Deflection []float64

	// Command — потребные углы, рад.
	Command []float64

	// Manual отмечает поверхности, которыми управляет оператор.
	// Автопилот их не трогает.
	Manual []bool

	// Jammed отмечает заклинившие приводы: команда не отрабатывается.
	Jammed []bool

	// Torque — суммарный момент от всех поверхностей, Н·м.
	Torque physics.Vec3

	// Force — суммарная аэродинамическая сила от всех поверхностей в связанных
	// осях, Н. Та же физическая сила, из которой строится Torque (r×F, см.
	// physics.ControlSurface.ForceAndTorque) — не отдельная, заново
	// придуманная величина. У плавников/рулей, которые до сих пор влияли
	// только на вращение (см. Torque), это поле даёт вызывающей стороне то
	// же самое воздействие для поступательного движения, вместо того чтобы
	// заново решать, "сколько" оно тормозит.
	Force physics.Vec3

	// Authority — потребный момент к располагаемому по тангажу, доля.
	Authority float64

	// Deployed сообщает, что поверхности выпущены в поток.
	//
	// На активном участке плавники прижаты к борту: раскрытые, они дали бы
	// огромный момент на максимальном скоростном напоре и опрокинули бы
	// носитель. Выпускают их перед входом в атмосферу.
	Deployed bool
}

// NewSurfaceSet собирает набор поверхностей.
func NewSurfaceSet(surfaces []physics.ControlSurface) *SurfaceSet {
	n := len(surfaces)
	return &SurfaceSet{
		Surfaces:   surfaces,
		Deflection: make([]float64, n),
		Command:    make([]float64, n),
		Manual:     make([]bool, n),
		Jammed:     make([]bool, n),
	}
}

// Count возвращает число поверхностей.
func (s *SurfaceSet) Count() int {
	if s == nil {
		return 0
	}
	return len(s.Surfaces)
}

// IndexOf возвращает номер поверхности по имени.
func (s *SurfaceSet) IndexOf(name string) int {
	if s == nil {
		return -1
	}
	for i, surface := range s.Surfaces {
		if surface.Name == name {
			return i
		}
	}
	return -1
}

// SetManual задаёт ручной угол для одной поверхности.
//
// Пока поверхность под ручным управлением, автопилот её не двигает: ровно
// как настоящий пульт с переключателем «автомат — ручное».
func (s *SurfaceSet) SetManual(index int, angle float64) {
	if s == nil || index < 0 || index >= len(s.Surfaces) {
		return
	}
	surface := s.Surfaces[index]
	s.Manual[index] = true
	s.Command[index] = math.Max(surface.MinDeflection, math.Min(surface.MaxDeflection, angle))
}

// ReleaseManual возвращает поверхность автопилоту.
func (s *SurfaceSet) ReleaseManual(index int) {
	if s == nil || index < 0 || index >= len(s.Manual) {
		return
	}
	s.Manual[index] = false
}

// ringHingeThreshold отделяет поверхности с осевым шарниром (плавники
// корабля, Hinge вдоль X — пара нос/корма) от поверхностей с радиальным
// шарниром (решётчатые рули бустера, Hinge лежит в плоскости Y-Z, X=0).
// Значения устройства ровно 0 и ±1, порог нужен только как надёжная граница
// между ними.
const ringHingeThreshold = 0.5

// surfaceWeights возвращает вклад поверхности в тангаж/рыскание/крен —
// то, на что домножается потребная доля хода при раскладке (Allocate) и
// при оценке располагаемого момента (PitchAuthority).
//
// Для плавников (Hinge вдоль X) это прежняя схема: знак плеча вдоль корпуса
// задаёт роль передней/задней пары, знак плеча поперёк — левой/правой.
//
// Для решётчатых рулей эта схема не годится в принципе: все рули сидят
// на одной станции по длине (плечо вдоль корпуса не задаёт пар "перед/зад"),
// расставлены по кольцу, а шарнир радиальный — отклонение поворачивает
// нормаль панели не в плоскости Y-Z, а в сторону оси X. Момент по тангажу
// и рысканию от одного руля пропорционален не знаку его положения, а
// синусу/косинусу его азимута на кольце: то же самое, что и в управлении
// циклическим шагом несущего винта, где отклонение лопасти зависит от её
// текущего угла, а не от того, "спереди" она или "сзади". При одинаковом
// (равномерном) отклонении всех рулей эти вклады по кольцу с симметричной
// расстановкой взаимно гасятся — момент по тангажу и рысканию выходит
// нулевым независимо от того, сколько рулей в кольце (3 или 4), — и раньше
// именно так и было: раскладка трогала все рули поровну, реального
// управления по тангажу/рысканию решётчатые рули не создавали, и на входе
// в плотные слои, где аэродинамический момент неустойчивого корпуса велик,
// а движки ориентации на него уже не хватает, бустер начинал раскачиваться
// и кувыркаться. Крен, наоборот, у кольца получается именно равномерным
// отклонением: за счёт того же поворота нормали к оси X момент по крену
// от каждого руля почти не зависит от азимута и складывается, а не гасится.
func surfaceWeights(surface physics.ControlSurface) (pitch, yaw, roll float64) {
	if math.Abs(surface.Hinge.X) < ringHingeThreshold {
		theta := math.Atan2(surface.Arm.Y, surface.Arm.Z)
		sinT, cosT := math.Sin(theta), math.Cos(theta)
		return -sinT, -cosT, 1
	}

	fore := 1.0
	if surface.Arm.X < 0 {
		fore = -1
	}
	side := 1.0
	if surface.Arm.Y < 0 {
		side = -1
	}
	return fore, side, side
}

// hasDynamicLever сообщает, что у набора есть хотя бы одна поверхность с
// PositionFromNose (см. physics.ControlSurface) — то есть это решётчатые
// рули бустера (GridFins), а не плавники корабля (ShipFlaps).
func (s *SurfaceSet) hasDynamicLever() bool {
	for _, surface := range s.Surfaces {
		if surface.PositionFromNose != 0 {
			return true
		}
	}
	return false
}

// Allocate раскладывает запрошенный трёхмерный момент mcmd (Н·м, связанные
// оси: X — крен, Y — тангаж, Z — рыскание) по поверхностям.
//
// Для решётчатых рулей бустера (PositionFromNose задан — см.
// hasDynamicLever) — allocateOptimal: полноценный МНК-solver на реальной
// физике. Для плавников корабля (PositionFromNose не задан) — прежняя схема
// с фиксированными долями (allocateLegacy): solver на этой геометрии (две
// пары плавников, ход до 120°, разбалансировка пары на больших углах атаки)
// пока не проверен и не показал улучшения, а старая схема годами держит
// вход корабля в атмосферу. Смешивать доказанный фикс бустера с непроверенным
// изменением поведения корабля в одном коммите не стал — тест
// TestFlapsTakeOverFromThrustersInDenseAir поймал именно это при первой
// попытке unified solver.
func (s *SurfaceSet) Allocate(mcmd, airBody physics.Vec3, dynamicPressure, mach, comFromNose float64) {
	if s == nil || len(s.Surfaces) == 0 {
		return
	}

	if !s.Deployed {
		for i := range s.Surfaces {
			if !s.Manual[i] {
				s.Command[i] = 0
			}
		}
		return
	}

	if !s.hasDynamicLever() {
		s.allocateLegacy(mcmd, airBody, dynamicPressure, mach, comFromNose)
		return
	}
	s.allocateOptimal(mcmd, airBody, dynamicPressure, mach, comFromNose)
}

// allocateLegacy — прежняя схема (фиксированные доли pitch/roll/yaw),
// сохранена без изменений поведения для плавников корабля. mcmd переводится
// в ту же нормированную "потребность", что вычислял старый demand() в
// attitude_dynamics.go: want/PitchAuthority, — так численно ничего не
// меняется по сравнению с тем, что уже работало.
func (s *SurfaceSet) allocateLegacy(mcmd, airBody physics.Vec3, dynamicPressure, mach, comFromNose float64) {
	authority := s.PitchAuthority(airBody, dynamicPressure, mach, comFromNose)
	demand := func(want float64) float64 {
		if authority < 1 {
			return 0
		}
		return clampAbs(want/authority, 1)
	}
	pitch, yaw, roll := demand(mcmd.Y), demand(mcmd.Z), demand(mcmd.X)

	for i, surface := range s.Surfaces {
		if s.Manual[i] {
			continue
		}

		// span/mid — по факту границ поверхности (MinDeflection/
		// MaxDeflection), а не жёстко [0,MaxDeflection]: для ShipFlaps
		// (MinDeflection=0) числа совпадают с прежними один в один, но
		// путь теперь не полагается на скрытое допущение диапазона.
		span := surface.MaxDeflection - surface.MinDeflection
		mid := (surface.MinDeflection + surface.MaxDeflection) / 2

		wPitch, wYaw, wRoll := surfaceWeights(surface)

		const (
			pitchShare = 1.0
			rollShare  = 0.30
			yawShare   = 0.15
		)

		angle := mid +
			wPitch*pitch*span*pitchShare +
			wRoll*roll*span*rollShare +
			wYaw*yaw*span*yawShare
		s.Command[i] = math.Max(surface.MinDeflection, math.Min(surface.MaxDeflection, angle))
	}
}

// allocateOptimal — solver для решётчатых рулей бустера. Раньше запрос
// сначала схлопывался в одно число (demand = want/PitchAuthority), а затем
// размазывался по рулям фиксированными долями (pitchShare=1.0,
// rollShare=0.30, yawShare=0.15) — у N рулей N степеней свободы (по одному
// углу на руль), а схема с одним общим числом и жёстко зашитыми долями в
// принципе не может подобрать комбинацию, реализующую все три компоненты
// момента одновременно: она задаёт НАПРАВЛЕНИЕ раскладки заранее, не глядя,
// как эта конкретная геометрия рулей в действительности отвечает на свой
// угол при текущих q/Mach/CoM. Диагностика (canonical harness, четыре
// режима controller×allocator) показала: именно это, а не привод и не
// контроллер, определяло, разрушится ли бустер — при физически одинаковой
// authority схема с честной раскладкой удерживала аппарат, а старая — нет.
//
// Устройство — покоординатный перебор (coordinateDescentPolish), а не
// градиентный Гаусс-Ньютон, которым решение начиналось: на сильно
// нелинейной Cp=2cos²θ (особенно у решётки, только приоткрытой от
// кромочного положения) локальная линеаризация регулярно сходилась в один
// и тот же fin_2-доминантный аттрактор НЕЗАВИСИМО от стартовой точки —
// измерено на T-раскладке: даже старт ровно в найденном поодиночке хорошем
// угле для yaw один демпфированный ньютоновский шаг уводил обратно к тому
// же плохому решению (raw loss ≈3.04e14 против ≈1.08e14 у найденной вручную
// комбинации двух рулей). Прямой перебор одной координаты за раз (при
// зафиксированных остальных, с сужением сетки по проходам) не может
// застрять в ложном градиенте, потому что не использует градиент вовсе —
// и с ним же решается задача, которую раньше решал якобиан: п.1 первой
// версии фикса (нормировка невязки по осям через directional authority,
// ниже) остаётся нужна независимо от метода поиска.
//
// Directional authority (target ниже) — самостоятельная, не менее важная
// часть фикса, актуальная независимо от границ хода конкретной поверхности:
// нормировать САМУ невязку на "сколько вообще есть авторитета" по
// симметричной оценке (например, норма столбца якобиана) — неверно, если
// авторитет разный в двух направлениях одной оси (у ShipFlaps так и есть
// физически — см. physics.ControlSurface.MinDeflection; у GridFins так
// БЫЛО по ошибке модели актуатора, пока диапазон не стал [-Max,Max] —
// см. история в комментарии у vehicle.GridFins). Симметричная оценка на
// таком дисбалансе занижает масштаб недостижимой стороны, отчего вес её
// невязки в объективе становится ЕЩЁ БОЛЬШЕ, а не меньше, — solver жертвует
// достижимой осью ради бесплодной погони за принципиально недостижимой.
// Правильная мера — не взвешивание невязки, а обрезка САМОЙ ЦЕЛИ: если
// ось недостижима в свою сторону почти совсем, значит и спрашивать с
// solver'а нечего — задача становится честной (все три компоненты цели
// в принципе достижимы по отдельности), и обычный перебор на неё уже не
// жертвует другими осями ради constrained-to-zero компонента.
func (s *SurfaceSet) allocateOptimal(mcmd, airBody physics.Vec3, dynamicPressure, mach, comFromNose float64) {
	cols := make([]int, 0, len(s.Surfaces))
	current := make([]float64, len(s.Surfaces))
	for i := range s.Surfaces {
		if s.Manual[i] {
			current[i] = s.Command[i]
			continue
		}
		current[i] = s.Deflection[i]
		cols = append(cols, i)
	}
	if len(cols) == 0 {
		return
	}

	achieved := func(angles []float64) physics.Vec3 {
		total := physics.Vec3{}
		for i, surface := range s.Surfaces {
			total = total.Add(surface.Torque(angles[i], airBody, dynamicPressure, mach, comFromNose))
		}
		return total
	}

	// --- Directional authority per axis: сколько РЕАЛЬНО достижимо в
	// ЗАПРОШЕННУЮ (со своим знаком) сторону каждой оси в отдельности — не
	// magic-константа и не симметричное приближение, а прямое измерение
	// (тот же перебор, что решает и основную задачу ниже, просто с целью
	// "максимум по этой одной оси"). Порядок проходов/сетки здесь заметно
	// грубее основного решения — это только оценка масштаба для обрезки
	// цели, а не финальный ответ.
	zero := make([]float64, len(s.Surfaces))
	axisScale := [3]float64{}
	mcmdArr := [3]float64{mcmd.X, mcmd.Y, mcmd.Z}
	for k := 0; k < 3; k++ {
		sign := 1.0
		if mcmdArr[k] < 0 {
			sign = -1
		}
		var probeTarget physics.Vec3
		switch k {
		case 0:
			probeTarget = physics.Vec3{X: sign * probeHuge}
		case 1:
			probeTarget = physics.Vec3{Y: sign * probeHuge}
		case 2:
			probeTarget = physics.Vec3{Z: sign * probeHuge}
		}

		x := coordinateDescentSearch(zero, cols, s.Surfaces, achieved, probeTarget, probePasses, probeGridSteps)
		a := achieved(x)
		var got float64
		switch k {
		case 0:
			got = a.X
		case 1:
			got = a.Y
		case 2:
			got = a.Z
		}
		// Знак важен: для ShipFlaps (MinDeflection=0, физически
		// односторонний упор — см. TestAllocateLegacy_UnaffectedByGridFinFix)
		// перебор, не способный уйти в запрошенную сторону, может осесть на
		// решении с достигнутым значением ПРОТИВОПОЛОЖНОГО знака (то, что
		// физически легко даётся при побочном участии рулей в других осях) —
		// считается только совпадающий по знаку результат; отрицательный
		// (или нулевой) вклада в authority не даёт. У GridFins
		// (симметричный диапазон) это тоже страхует на случай остаточной
		// геометрической асимметрии между осями.
		if gotSigned := got * sign; gotSigned > 0 {
			axisScale[k] = gotSigned
		}
	}

	target := physics.Vec3{
		X: clampMagnitude(mcmd.X, axisScale[0]),
		Y: clampMagnitude(mcmd.Y, axisScale[1]),
		Z: clampMagnitude(mcmd.Z, axisScale[2]),
	}

	// Два старта, не один: current — минимальное смещение хода в типичном
	// такте (secondary cost — те же соображения, что и раньше), zero —
	// нейтральная, неадверсариальная точка, которая на практике надёжно
	// сходится к хорошему решению независимо от того, где случайно
	// оказался Deflection на предыдущем такте (см. TestAllocateOptimal_
	// StartRobustness — только current, стартующий из "плохих" положений
	// вроде [max,max,max] или [max,0,0], не всегда успевал выбраться за
	// отведённые проходы; zero подстраховывает без полного возврата к
	// дорогому мульти-старту, который и был первопричиной непозволительной
	// стоимости на такт).
	best := coordinateDescentSearch(current, cols, s.Surfaces, achieved, target, mainPasses, mainGridSteps)
	bestLoss := lossVec(achieved(best), target)

	fromZero := coordinateDescentSearch(zero, cols, s.Surfaces, achieved, target, mainPasses, mainGridSteps)
	if l := lossVec(achieved(fromZero), target); l < bestLoss {
		best = fromZero
	}

	for _, i := range cols {
		s.Command[i] = best[i]
	}
}

// lossVec — квадрат нормы невязки achieved-target, Н²·м².
func lossVec(achieved, target physics.Vec3) float64 {
	d := achieved.Sub(target)
	return d.Dot(d)
}

// probeHuge — заведомо недостижимый в реальности момент для directional-
// authority проб, Н·м (см. TestAllocateOptimal_SaturationIsWellBehaved).
// probePasses/probeGridSteps — грубый перебор (оценка масштаба для обрезки
// цели, не финальный ответ). mainPasses/mainGridSteps — перебор основного
// решения, точнее. Все четыре — стоимость перебора (проходы×шаги на руль)
// напрямую определяют, сколько раз считается achieved() за один вызов
// Allocate — а он вызывается на каждом подшаге контура ориентации (100 Гц),
// так что перебор должен быть настолько грубым, насколько ещё находит
// разумное решение, а не насколько точным можно было бы сделать offline.
const (
	probeHuge      = 1e12
	probePasses    = 2
	probeGridSteps = 8
	mainPasses     = 3
	mainGridSteps  = 10
)

// coordinateDescentSearch ищет x, минимизирующий |achieved(x)-target|²,
// покоординатным перебором: несколько проходов, на каждом — для каждого
// активного руля перебирается сетка углов в [MinDeflection, MaxDeflection]
// этой конкретной поверхности (не жёстко [0,Max] — см.
// physics.ControlSurface.MinDeflection: у решётчатых рулей нижняя граница
// отрицательна) при ЗАФИКСИРОВАННЫХ остальных, берётся то значение, что
// минимизирует невязку; сетка следующего прохода сужается вокруг найденного.
// Не использует градиент вовсе — поэтому не может застрять в ложном
// градиенте локальной линеаризации (см. комментарий у allocateOptimal).
func coordinateDescentSearch(x0 []float64, cols []int, surfaces []physics.ControlSurface,
	achieved func([]float64) physics.Vec3, target physics.Vec3, passes, gridSteps int) []float64 {

	x := append([]float64{}, x0...)
	lossAt := func() float64 {
		d := achieved(x).Sub(target)
		return d.Dot(d)
	}

	for pass := 0; pass < passes; pass++ {
		// Сетка сужается с каждым проходом вокруг текущего значения —
		// первый проход широкий (весь диапазон), последующие всё точнее.
		spanFraction := 1.0 / math.Pow(2, float64(pass))
		for _, i := range cols {
			minD, maxD := surfaces[i].MinDeflection, surfaces[i].MaxDeflection
			span := maxD - minD
			if span <= 0 {
				continue
			}
			center := x[i]
			lo := math.Max(minD, center-span*spanFraction)
			hi := math.Min(maxD, center+span*spanFraction)
			if pass == 0 {
				lo, hi = minD, maxD
			}

			bestAngle := x[i]
			bestLoss := lossAt()
			for step := 0; step <= gridSteps; step++ {
				candidate := lo + (hi-lo)*float64(step)/float64(gridSteps)
				prev := x[i]
				x[i] = candidate
				l := lossAt()
				if l < bestLoss {
					bestLoss = l
					bestAngle = candidate
				}
				x[i] = prev
			}
			x[i] = bestAngle
		}
	}
	return x
}

// clampMagnitude ограничивает |v| значением maxAbs, сохраняя знак v.
func clampMagnitude(v, maxAbs float64) float64 {
	if v > maxAbs {
		return maxAbs
	}
	if v < -maxAbs {
		return -maxAbs
	}
	return v
}

// Advance перекладывает приводы к потребным углам.
func (s *SurfaceSet) Advance(dt float64) {
	if s == nil {
		return
	}
	for i := range s.Surfaces {
		if s.Jammed[i] {
			continue
		}
		step := s.Surfaces[i].Rate * dt
		s.Deflection[i] = approach(s.Deflection[i], s.Command[i], step)
	}
}

// UpdateTorque считает суммарные силу и момент от поверхностей.
//
// Момент и поступательная сила — проекции ОДНОЙ физической аэродинамической
// силы каждой панели (physics.ControlSurface.ForceAndTorque считает её один
// раз и отдаёт оба): здесь оба слагаемых накапливаются в одном проходе, без
// двойного счёта и без отдельной, заново придуманной "силы сопротивления
// рулей". Результат момента возвращается, как и раньше (вызывающая сторона
// подставляет его в уравнение вращения); суммарная сила остаётся в s.Force —
// её забирает вызывающая сторона, которой нужно и поступательное движение
// (см. booster.go:Step).
//
// comFromNose — текущее положение центра масс от носа, м (см.
// physics.ControlSurface.ForceAndTorque) — поверхностям без PositionFromNose
// (плавники корабля) это значение не нужно и не влияет на результат.
func (s *SurfaceSet) UpdateTorque(airBody physics.Vec3, dynamicPressure, mach, comFromNose float64) physics.Vec3 {
	if s == nil {
		return physics.Vec3{}
	}

	totalForce := physics.Vec3{}
	totalTorque := physics.Vec3{}
	for i, surface := range s.Surfaces {
		force, torque := surface.ForceAndTorque(s.Deflection[i], airBody, dynamicPressure, mach, comFromNose)
		totalForce = totalForce.Add(force)
		totalTorque = totalTorque.Add(torque)
	}
	s.Force = totalForce
	s.Torque = totalTorque
	return totalTorque
}

// PitchAuthority возвращает располагаемый момент по тангажу при полном
// перекладывании поверхностей, Н·м.
//
// Нужен, чтобы честно показывать оператору, чем он вообще располагает:
// в разрежённых слоях эта величина близка к нулю, и никакое отклонение
// плавников корпус не развернёт.
func (s *SurfaceSet) PitchAuthority(airBody physics.Vec3, dynamicPressure, mach, comFromNose float64) float64 {
	if s == nil {
		return 0
	}

	// Власть — это разность между крайними положениями привода на ВСЁМ его
	// физическом ходу (MinDeflection…MaxDeflection), а не момент в одном
	// крайнем положении относительно нуля: для решётчатых рулей ноль —
	// середина хода, а не упор (см. physics.ControlSurface.MinDeflection),
	// и добрая половина располагаемой власти лежит именно в отрицательной
	// части хода. Для плавников корабля (MinDeflection=0) формула не
	// меняется: прижатый плавник по-прежнему нижняя граница.
	full := 0.0
	for _, surface := range s.Surfaces {
		wPitch, _, _ := surfaceWeights(surface)

		open := surface.Torque(surface.MaxDeflection, airBody, dynamicPressure, mach, comFromNose)
		shut := surface.Torque(surface.MinDeflection, airBody, dynamicPressure, mach, comFromNose)
		full += wPitch * (open.Y - shut.Y)
	}
	return math.Abs(full)
}

// approach двигает значение к цели не быстрее заданного шага.
func approach(current, target, step float64) float64 {
	if step <= 0 {
		return current
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
