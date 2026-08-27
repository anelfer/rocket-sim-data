package simulator

import (
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Stage 4.5: per-engine TVC.
//
// Раньше управляющий момент носителя считался от ОДНОГО агрегатного угла
// качания (VehicleAttitude.GimbalPitch/Yaw/Roll, attitude_dynamics.go) —
// как будто все камеры блока качаются синхронно одной осью, а крен и вовсе
// получал отдельный, ничем не связанный с этим отклонением "магический"
// момент (Thrust*sin(GimbalRoll)*EngineRingRadius). У сверхтяжёлого
// носителя маневренных камер тринадцать (см. vehicle.EngineLayout,
// firstStageEngineLayout), каждая в своей точке юбки — и физически
// единственная сила, которую они создают, это F_i = T_i·d_i в этой самой
// точке; сумма этих сил двигает бустер, а их r_i×F_i одновременно его
// вращает. Никакого отдельного "момента качания" в природе нет — есть
// только тяга в конкретных точках подвеса.
//
// Здесь: физическая модель одной камеры (engineDirectionBody,
// engineForceTorque — реальная, нелинейная, та же, что попадёт в
// интегратор) и allocator, ищущий отклонения подвеса КАЖДОЙ маневренной
// камеры, которые одновременно приближают суммарную силу к желаемой и
// суммарный момент — к желаемому (см. allocateEngineGimbal).
// -----------------------------------------------------------------------------

// EngineActuator — вход allocator'а для одного двигателя на такте: его
// неизменная геометрия (позиция в связанных осях крепления, способность
// качаться) и текущая тяга (0, если камера не работает — погашенные камеры
// не участвуют, см. п.20 Stage 4.5).
type EngineActuator struct {
	ID string

	// Radial — положение оси двигателя в плоскости среза юбки, м, ДО
	// поворота в связанные оси корпуса (в тех же X/Y, что
	// vehicle.EngineLayout — координаты от продольной оси, не связанные
	// оси корпуса напрямую).
	RadialX, RadialY float64

	// Gimbal сообщает, что камера физически способна качаться — совпадает
	// с vehicle.EngineLayout.Gimbal этого двигателя.
	Gimbal bool

	// Thrust — тяга ЭТОЙ камеры прямо сейчас, Н (0, если не работает).
	Thrust float64
}

// EngineGimbalState — фактическое (после ограничения скорости привода)
// отклонение подвеса одного двигателя, рад.
type EngineGimbalState struct {
	Pitch, Yaw float64
}

// enginePosition возвращает положение двигателя относительно ЦМ в связанных
// осях корпуса (X — Forward/продольная, Y — Right, Z — Down), м.
//
// Собственная (X,Y) позиция двигателя в плоскости среза (EngineActuator.
// RadialX/RadialY) кладётся в поперечные оси Right/Down — выбор направления
// не заимствован ни из какой физики рендера (там те же X/Y используются
// только для картинки в сечении, см. vehicle/layout.go), это первое
// использование этих координат в физике, и оно самосогласовано: главное,
// чтобы одна и та же (X,Y)→(Right,Down) раскладка применялась и к позиции,
// и к решётке допустимых отклонений — что здесь и есть. armAxial —
// расстояние от ЦМ до плоскости среза камер вдоль продольной оси, м (то же
// GimbalArm, что уже строит booster.go/updateAttitude).
func enginePosition(e EngineActuator, armAxial float64) physics.Vec3 {
	return physics.Vec3{X: -armAxial, Y: e.RadialX, Z: e.RadialY}
}

// engineDirectionBody возвращает направление тяги одной камеры в связанных
// осях при отклонении подвеса (pitch,yaw), рад — та же формула и те же
// знаки, что VehicleAttitude.ThrustDirection (агрегатная версия) применяет
// ко всему блоку разом: pitch отклоняет тягу к Down (нос поднимается —
// см. её комментарий), yaw — к -Right.
func engineDirectionBody(pitch, yaw float64) physics.Vec3 {
	cp, sp := math.Cos(pitch), math.Sin(pitch)
	cy, sy := math.Cos(yaw), math.Sin(yaw)
	dir := physics.Vec3{X: cp * cy, Y: -sy, Z: sp}
	if n := dir.Norm(); n > 1e-9 {
		return dir.Scale(1 / n)
	}
	return physics.Vec3{X: 1}
}

// engineForceTorque — РЕАЛЬНАЯ (нелинейная) сила и момент от всего блока
// камер при заданных gimbal, связанные оси. Та же самая функция служит и
// источником истины для интегратора (см. integrate()), и для оценки
// решения allocator'а (п.17 Stage 4.5: "final evaluation — только real
// model", линеаризация допустима только ВНУТРИ solver'а).
func engineForceTorque(engines []EngineActuator, gimbal []EngineGimbalState, armAxial float64) (force, torque physics.Vec3) {
	for i, e := range engines {
		if e.Thrust <= 0 {
			continue
		}
		p, y := 0.0, 0.0
		if e.Gimbal && i < len(gimbal) {
			p, y = gimbal[i].Pitch, gimbal[i].Yaw
		}
		f := engineDirectionBody(p, y).Scale(e.Thrust)
		force = force.Add(f)
		r := enginePosition(e, armAxial)
		torque = torque.Add(r.Cross(f))
	}
	return force, torque
}

// allocateGimbalRegularization — вес вторичной цели allocator'а (минимальное
// отклонение подвеса, п.15 Stage 4.5 "secondary cost — минимальные движения
// gimbal"), безразмерная величина в той же нормированной шкале, что и веса
// силы/момента ниже — не физическая константа, а параметр решателя: budget
// на то, чтобы задача с избытком степеней свободы (до 26 переменных против
// 6 целевых) не оседала на решении с бессмысленно большими встречными
// отклонениями разных камер, компенсирующими друг друга.
const allocateGimbalRegularization = 1e-3

// allocateEngineGimbal ищет отклонения подвеса КАЖДОЙ маневренной камеры,
// приближающие суммарную силу И суммарный момент к желаемым разом — п.14
// Stage 4.5 ("не решать только torque, забыв translation").
//
// Малоугловая линеаризация используется ТОЛЬКО внутри решателя (реальная,
// нелинейная оценка результата — отдельно, engineForceTorque, применяется
// снаружи; см. п.17: "линеаризацию можно использовать внутри solver'а,
// final evaluation — только real model"): для малых δ вклад камеры i в силу
// и момент линеен по (δpitch_i,δyaw_i), и задача сводится к взвешенному
// МНК с регуляризацией — точное решение нормальных уравнений, без
// итеративного перебора.
//
// Перебор (как у решётчатых рулей, coordinateDescentSearch) здесь
// сознательно не используется: там 2-4 поверхности и выраженная
// нелинейность делают грубый перебор дешёвым и надёжным; здесь до
// 26 переменных на 13 маневренных камерах, и на подшаге контура ориентации
// (100 Гц, см. surfaces.go/probePasses) перебор такого масштаба был бы
// непозволительно дорог. Малоугловая аппроксимация для качания в ±5°
// (текущий MaxGimbal) даёт погрешность порядка (5°)²/2 ≈ 0.2% — пренебрежимо
// рядом с ценой полного перебора 2N-мерной сетки.
//
// forceScale/torqueScale — нормировка невязки (п.16 Stage 4.5): "не
// минимизировать сырые Н²+(Н·м)², а привести к одному масштабу". Берутся
// из располагаемой authority ТЕКУЩЕЙ группы камер (суммарная тяга и
// тяга·плечо), а не подбираются на глаз — см. вызывающий код (integrate).
func allocateEngineGimbal(engines []EngineActuator, armAxial float64,
	targetForce, targetTorque physics.Vec3, forceScale, torqueScale, maxGimbal float64) []EngineGimbalState {

	result := make([]EngineGimbalState, len(engines))
	if forceScale <= 0 {
		forceScale = 1
	}
	if torqueScale <= 0 {
		torqueScale = 1
	}

	type column struct {
		engineIdx int
		isPitch   bool
		v         [6]float64
	}
	var cols []column
	var baselineForce, baselineTorque physics.Vec3

	for i, e := range engines {
		if e.Thrust <= 0 {
			continue
		}
		axial := physics.Vec3{X: e.Thrust}
		baselineForce = baselineForce.Add(axial)
		r := enginePosition(e, armAxial)
		baselineTorque = baselineTorque.Add(r.Cross(axial))
		if !e.Gimbal {
			continue
		}

		// ∂F/∂δpitch = T·Down, ∂F/∂δyaw = -T·Right — та же линеаризация
		// (sin δ≈δ, cos δ≈1), что engineDirectionBody для малых δ.
		dDown := physics.Vec3{Z: e.Thrust}
		dNegRight := physics.Vec3{Y: -e.Thrust}
		tPitch := r.Cross(dDown)
		tYaw := r.Cross(dNegRight)

		cols = append(cols, column{i, true, [6]float64{dDown.X, dDown.Y, dDown.Z, tPitch.X, tPitch.Y, tPitch.Z}})
		cols = append(cols, column{i, false, [6]float64{dNegRight.X, dNegRight.Y, dNegRight.Z, tYaw.X, tYaw.Y, tYaw.Z}})
	}

	n := len(cols)
	if n == 0 {
		return result
	}

	fw := 1 / (forceScale * forceScale)
	mw := 1 / (torqueScale * torqueScale)
	w := [6]float64{fw, fw, fw, mw, mw, mw}

	b := [6]float64{
		targetForce.X - baselineForce.X,
		targetForce.Y - baselineForce.Y,
		targetForce.Z - baselineForce.Z,
		targetTorque.X - baselineTorque.X,
		targetTorque.Y - baselineTorque.Y,
		targetTorque.Z - baselineTorque.Z,
	}

	// Нормальные уравнения взвешенного МНК с тихоновской регуляризацией:
	// (AᵀWA + λI)x = AᵀWb. Матрица n×n (n≤26) — прямое решение дешевле
	// любого перебора и точно (в пределах линеаризации), а не приближённо.
	normal := make([][]float64, n)
	rhs := make([]float64, n)
	for i := 0; i < n; i++ {
		normal[i] = make([]float64, n)
		for j := 0; j < n; j++ {
			sum := 0.0
			for k := 0; k < 6; k++ {
				sum += cols[i].v[k] * w[k] * cols[j].v[k]
			}
			normal[i][j] = sum
		}
		normal[i][i] += allocateGimbalRegularization
		sum := 0.0
		for k := 0; k < 6; k++ {
			sum += cols[i].v[k] * w[k] * b[k]
		}
		rhs[i] = sum
	}

	x := solveLinearSystem(normal, rhs)
	for i, c := range cols {
		v := physics.Clamp(x[i], -maxGimbal, maxGimbal)
		if c.isPitch {
			result[c.engineIdx].Pitch = v
		} else {
			result[c.engineIdx].Yaw = v
		}
	}
	return result
}

// solveLinearSystem решает A·x=b методом Гаусса с выбором главного элемента
// по столбцу. Общего назначения, без предположения о структуре A — здесь
// используется на симметричной положительно определённой (с учётом
// регуляризации) матрице нормальных уравнений, но сам метод этого не
// требует. Вырожденная/близкая к вырожденной строка (нулевой ведущий
// элемент) даёт нулевую компоненту решения, а не панику или NaN.
func solveLinearSystem(a [][]float64, b []float64) []float64 {
	n := len(b)
	m := make([][]float64, n)
	for i := range a {
		m[i] = append([]float64{}, a[i]...)
	}
	x := append([]float64{}, b...)

	for col := 0; col < n; col++ {
		pivot := col
		best := math.Abs(m[col][col])
		for r := col + 1; r < n; r++ {
			if v := math.Abs(m[r][col]); v > best {
				best = v
				pivot = r
			}
		}
		if best < 1e-15 {
			continue // вырождено по этому столбцу — соответствующая переменная остаётся 0
		}
		m[col], m[pivot] = m[pivot], m[col]
		x[col], x[pivot] = x[pivot], x[col]

		diag := m[col][col]
		for r := col + 1; r < n; r++ {
			factor := m[r][col] / diag
			if factor == 0 {
				continue
			}
			for c := col; c < n; c++ {
				m[r][c] -= factor * m[col][c]
			}
			x[r] -= factor * x[col]
		}
	}

	out := make([]float64, n)
	for i := n - 1; i >= 0; i-- {
		if math.Abs(m[i][i]) < 1e-15 {
			out[i] = 0
			continue
		}
		sum := x[i]
		for j := i + 1; j < n; j++ {
			sum -= m[i][j] * out[j]
		}
		out[i] = sum / m[i][i]
	}
	return out
}
