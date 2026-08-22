package env

import (
	"math"
	"math/rand"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Ветер.
//
// Раньше атмосфера считалась жёстко вращающейся вместе с Землёй, то есть
// неподвижной относительно поверхности. На самом деле в тропосфере и нижней
// стратосфере есть устойчивые течения со скоростями до 60 м/с, а главное —
// сдвиг ветра по высоте. Именно сдвиг создаёт угол атаки на участке
// максимального скоростного напора и определяет поперечную нагрузку на корпус.
// -----------------------------------------------------------------------------

// WindModel возвращает скорость воздуха относительно вращающейся Земли
// в инерциальных осях.
type WindModel interface {
	// Velocity возвращает добавку к скорости воздуха в точке position
	// на высоте altitude в момент t, в осях ECI.
	Velocity(position physics.Vec3, altitude, t float64) physics.Vec3
}

// -----------------------------------------------------------------------------
// Профиль ветра
// -----------------------------------------------------------------------------

// WindProfile — модель ветра со струйным течением и порывами.
type WindProfile struct {
	// JetStreamAltitude — высота максимума струйного течения, м.
	JetStreamAltitude float64

	// JetStreamSpeed — скорость в максимуме, м/с.
	JetStreamSpeed float64

	// JetStreamWidth — характерная полуширина струи по высоте, м.
	JetStreamWidth float64

	// JetStreamDirection — направление, КУДА дует ветер, градусы от севера.
	JetStreamDirection float64

	// SurfaceSpeed — скорость приземного ветра, м/с.
	SurfaceSpeed float64

	// SurfaceDirection — направление приземного ветра, градусы от севера.
	SurfaceDirection float64

	// ShearAmplitude — амплитуда случайных слоёв сдвига, м/с.
	ShearAmplitude float64

	// TopAltitude — высота, выше которой ветра нет, м.
	TopAltitude float64

	// layers — заранее разыгранные слои сдвига. Профиль детерминирован
	// на весь полёт: ветер не должен меняться от шага к шагу, иначе
	// он превращается в шум, а не в поле скоростей.
	layers []shearLayer
}

type shearLayer struct {
	altitude  float64 // центр слоя, м
	thickness float64 // толщина, м
	speed     float64 // амплитуда, м/с
	direction float64 // направление, градусы от севера
}

// NewWindProfile создаёт профиль ветра со случайными слоями сдвига.
//
// Все случайные величины разыгрываются один раз при создании, поэтому профиль
// воспроизводим при одном и том же seed и постоянен в течение полёта.
func NewWindProfile(rng *rand.Rand) *WindProfile {
	w := &WindProfile{
		JetStreamAltitude:  11000,
		JetStreamSpeed:     25 + rng.NormFloat64()*12,
		JetStreamWidth:     4000,
		JetStreamDirection: 90 + rng.NormFloat64()*40,
		SurfaceSpeed:       math.Abs(4 + rng.NormFloat64()*3),
		SurfaceDirection:   rng.Float64() * 360,
		ShearAmplitude:     6,
		TopAltitude:        60000,
	}
	if w.JetStreamSpeed < 0 {
		w.JetStreamSpeed = 0
	}

	// Несколько слоёв сдвига в тропосфере и стратосфере.
	count := 3 + rng.Intn(3)
	w.layers = make([]shearLayer, count)
	for i := range w.layers {
		w.layers[i] = shearLayer{
			altitude:  2000 + rng.Float64()*25000,
			thickness: 800 + rng.Float64()*2500,
			speed:     rng.NormFloat64() * w.ShearAmplitude,
			direction: rng.Float64() * 360,
		}
	}
	return w
}

// CalmWind возвращает модель без ветра.
func CalmWind() WindModel { return calmWind{} }

type calmWind struct{}

func (calmWind) Velocity(physics.Vec3, float64, float64) physics.Vec3 {
	return physics.Vec3{}
}

// Velocity возвращает скорость ветра в инерциальных осях.
func (w *WindProfile) Velocity(position physics.Vec3, altitude, t float64) physics.Vec3 {
	if altitude < 0 || altitude > w.TopAltitude {
		return physics.Vec3{}
	}

	frame := physics.NewLocalFrame(position)

	// Приземный слой: логарифмический профиль, затухающий к 2 км.
	surface := w.SurfaceSpeed
	if altitude > 0 {
		surface *= math.Log1p(altitude/10) / math.Log1p(200)
	}
	if altitude > 2000 {
		surface *= math.Exp(-(altitude - 2000) / 3000)
	}

	// Струйное течение: гауссов профиль по высоте.
	d := (altitude - w.JetStreamAltitude) / w.JetStreamWidth
	jet := w.JetStreamSpeed * math.Exp(-d*d)

	total := horizontalWind(frame, surface, w.SurfaceDirection).
		Add(horizontalWind(frame, jet, w.JetStreamDirection))

	// Слои сдвига.
	for _, layer := range w.layers {
		dl := (altitude - layer.altitude) / layer.thickness
		amplitude := layer.speed * math.Exp(-dl*dl)
		total = total.Add(horizontalWind(frame, amplitude, layer.direction))
	}

	// Выше 30 км ветер быстро слабеет.
	if altitude > 30000 {
		total = total.Scale(math.Exp(-(altitude - 30000) / 8000))
	}

	return total
}

// horizontalWind строит горизонтальный вектор ветра по скорости и направлению.
// Направление задаётся как азимут, КУДА дует ветер.
func horizontalWind(frame physics.LocalFrame, speed, directionDeg float64) physics.Vec3 {
	if speed == 0 {
		return physics.Vec3{}
	}
	rad := directionDeg * math.Pi / 180
	return frame.North.Scale(speed * math.Cos(rad)).
		Add(frame.East.Scale(speed * math.Sin(rad)))
}

// SpeedAt возвращает модуль скорости ветра на заданной высоте, м/с.
// Вспомогательный метод для телеметрии и тестов.
func (w *WindProfile) SpeedAt(altitude float64) float64 {
	// Локальный базис в произвольной точке: модуль от точки не зависит.
	frame := physics.NewLocalFrame(physics.Vec3{X: physics.EarthRadius + altitude})
	return w.Velocity(physics.Vec3{X: physics.EarthRadius + altitude}, altitude, 0).
		Sub(physics.Vec3{}).Norm() * boolToOne(frame.Up.Norm() > 0)
}

func boolToOne(b bool) float64 {
	if b {
		return 1
	}
	return 0
}
