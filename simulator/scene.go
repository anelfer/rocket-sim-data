package simulator

import (
	"math"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Состояние для трёхмерной сцены.
//
// Пульту нужна не «ориентация» вообще, а тройка связанных осей в понятной ему
// системе. Углы места, азимута и крена для этого не годятся: по ним сцена
// обязана заново собирать матрицу поворота, повторяя соглашения модели, и любое
// расхождение в порядке поворотов даст корабль, летящий боком.
//
// Поэтому наружу отдаются сами оси — вперёд, вправо, вниз — уже пересчитанные
// в местный горизонт (восток, север, верх). Сцена ставит их в матрицу как есть.
// Тем же способом отдаётся направление набегающего потока: угол атаки на
// картинке обязан быть тем же, по которому модель считает сопротивление.
// -----------------------------------------------------------------------------

// SceneVector — вектор в местном горизонте: восток, север, верх.
type SceneVector struct {
	East  float64 `json:"e"`
	North float64 `json:"n"`
	Up    float64 `json:"u"`
}

// SceneTelemetry — то, что нужно трёхмерной сцене и не выводится из остальных
// полей телеметрии.
type SceneTelemetry struct {
	// Оси корпуса в местном горизонте.
	Forward SceneVector `json:"forward"`
	Right   SceneVector `json:"right"`
	Down    SceneVector `json:"down"`

	// Airflow — направление набегающего потока в местном горизонте.
	// Нулевой вектор означает, что потока нет: в вакууме обдувать нечем.
	Airflow SceneVector `json:"airflow"`

	// Velocity — направление движения ОТНОСИТЕЛЬНО ПОВЕРХНОСТИ.
	//
	// Раньше здесь была инерциальная (ECI) скорость, и это ломало картинку.
	// Инерциальная скорость включает переносную скорость вращения Земли —
	// около 400 м/с на широте площадки. Пока аппарат идёт быстро, вклад
	// теряется в общей величине; на возврате и посадке, где своя скорость
	// сравнима с переносной, направления расходятся. Измерено на живом
	// прогоне: 51.7° между инерциальным и относительно поверхности при
	// |v_ECI|=469 м/с и |v_отн|=497 м/с.
	//
	// Рядом в той же сцене рисуется набегающий поток (Airflow), который
	// считается относительно ВОЗДУХА. Две стрелки в разных системах отсчёта
	// расходились на полсотни градусов без всякой физической причины — и
	// это выглядело как «корпус, направление полёта и векторы не сходятся».
	// Здесь та же система отсчёта, что у потока: расхождение между ними
	// теперь означает ровно то, чем оно и является физически — снос ветром.
	//
	// Тот же переход и по той же причине уже сделан для скорости бустера в
	// телеметрии (Stage 4.12.5, BoosterTelemetry.TotalVelocity).
	Velocity SceneVector `json:"velocity"`

	// Downrange — удаление от стартовой площадки по дуге большого круга, м.
	Downrange float64 `json:"downrange"`

	// Tower — вектор ОТ аппарата К основанию башни-ловушки в том же местном
	// горизонте (восток, север, верх), м.
	//
	// Якорь, по которому сцена ставит башню относительно корпуса. Один
	// вектор вместо широт и долгот: картинка не должна повторять
	// преобразования координат — на десятках километров плоская земля
	// расходится с моделью на заметную величину, и башня уезжала бы из-под
	// садящейся ступени.
	Tower SceneVector `json:"tower"`
}

// -----------------------------------------------------------------------------
// Кадр сцены.
//
// Полный снимок для пульта весит восемьдесят килобайт, и почти весь этот вес —
// подробности двигательной установки: тридцать три камеры, у каждой турбонасос,
// станции стенки и спектр пульсаций. Собрать его в JSON стоит треть
// миллисекунды, и это самая дорогая операция во всём потоке телеметрии.
//
// Трёхмерной сцене из этого не нужно ничего. Ей нужны оси корпуса, положение,
// плавники и температура сторон — полтора килобайта. Поэтому кадр сцены ходит
// отдельным сообщением и вдвое чаще полного снимка: плавность картинки стоит
// сорока килобайт в секунду вместо полутора мегабайт.
//
// Имена полей совпадают с именами в полной телеметрии: сцена читает те же
// ключи и не различает, из какого сообщения пришло состояние.
// -----------------------------------------------------------------------------

// SceneFrame — состояние для трёхмерной сцены и ничего сверх него.
type SceneFrame struct {
	Time  float64 `json:"time"`
	Phase string  `json:"phase"`
	Stage int     `json:"stage"`

	Altitude         float64 `json:"altitude"`
	Latitude         float64 `json:"lat"`
	Longitude        float64 `json:"lon"`
	TotalVelocity    float64 `json:"totalVelocity"`
	VerticalVelocity float64 `json:"verticalVelocity"`

	Mach            float64 `json:"mach"`
	TotalAoA        float64 `json:"totalAoA"`
	DynamicPressure float64 `json:"dynamicPressure"`
	HeatFlux        float64 `json:"heatFlux"`

	Throttle       float64 `json:"throttle"`
	EnginesRunning int     `json:"enginesRunning"`

	Scene      SceneTelemetry       `json:"scene"`
	Flaps      []FlapTelemetry      `json:"flaps,omitempty"`
	HeatShield *HeatShieldTelemetry `json:"heatShield,omitempty"`
	Landing    *LandingTelemetry    `json:"landing,omitempty"`

	// Booster — облегчённое состояние возвращающегося бустера, для плавной
	// одновременной отрисовки второго тела в 3D между полными снимками.
	Booster *BoosterSceneFrame `json:"booster,omitempty"`
}

// BoosterSceneFrame — минимум, нужный сцене для второго тела: оси корпуса,
// положение, решётчатые рули. Тот же принцип, что и у основного SceneFrame —
// не тащить сюда двигательную установку целиком ради плавности картинки.
type BoosterSceneFrame struct {
	Phase     string  `json:"phase"`
	Altitude  float64 `json:"altitude"`
	Latitude  float64 `json:"lat"`
	Longitude float64 `json:"lon"`

	Scene    SceneTelemetry  `json:"scene"`
	GridFins []FlapTelemetry `json:"gridFins,omitempty"`

	// Tower, Catch — башня-ловушка глазами сцены: куда её поставить
	// относительно корпуса и что показать о проходе зоны захвата.
	// Полтысячи байт на кадр — цена того, чтобы промах относительно рук
	// рисовался с той же частотой, что и сам корпус, а не рывками по
	// полному снимку.
	// Catch — что произошло (или ещё нет) на высоте захвата. Полтораста
	// байт на кадр — цена того, чтобы промах относительно рук рисовался с
	// той же частотой, что и сам корпус, а не рывками по полному снимку.
	Catch CatchTelemetry `json:"catch"`

	Destroyed bool `json:"destroyed"`
}

// SceneFrame возвращает кадр сцены по последнему опубликованному снимку.
func (s *Simulation) SceneFrame() SceneFrame {
	t := s.Snapshot()

	return SceneFrame{
		Time:             t.Time,
		Phase:            t.Phase,
		Stage:            t.Stage,
		Altitude:         t.Altitude,
		Latitude:         t.Latitude,
		Longitude:        t.Longitude,
		TotalVelocity:    t.TotalVelocity,
		VerticalVelocity: t.VerticalVelocity,
		Mach:             t.Mach,
		TotalAoA:         t.TotalAoA,
		DynamicPressure:  t.DynamicPressure,
		HeatFlux:         t.HeatFlux,
		Throttle:         t.Throttle,
		EnginesRunning:   t.EnginesRunning,
		Scene:            t.Scene,
		Flaps:            t.Flaps,
		HeatShield:       t.HeatShield,
		Landing:          t.Landing,
		Booster:          boosterSceneFrame(t.Booster),
	}
}

// boosterSceneFrame сжимает полную телеметрию бустера до кадра сцены.
func boosterSceneFrame(b *BoosterTelemetry) *BoosterSceneFrame {
	if b == nil {
		return nil
	}
	return &BoosterSceneFrame{
		Phase:     b.Phase,
		Altitude:  b.Altitude,
		Latitude:  b.Latitude,
		Longitude: b.Longitude,
		Scene:     b.Scene,
		GridFins:  b.GridFins,
		Catch:     b.Catch,
		Destroyed: b.Destroyed,
	}
}

// greatCircleDistance возвращает расстояние по дуге большого круга, м.
func greatCircleDistance(lat1, lon1, lat2, lon2 float64) float64 {
	const d = physics.DegToRad

	p1, p2 := lat1*d, lat2*d
	dp, dl := (lat2-lat1)*d, (lon2-lon1)*d

	a := math.Sin(dp/2)*math.Sin(dp/2) +
		math.Cos(p1)*math.Cos(p2)*math.Sin(dl/2)*math.Sin(dl/2)

	return 2 * physics.EarthRadius * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
}

// toScene переводит вектор из ECI в местный горизонт.
func toScene(f physics.LocalFrame, v physics.Vec3) SceneVector {
	return SceneVector{
		East:  v.Dot(f.East),
		North: v.Dot(f.North),
		Up:    v.Dot(f.Up),
	}
}

// buildSceneTelemetry собирает состояние для трёхмерной сцены.
// Вызывается под удержанным mu.
func (s *Simulation) buildSceneTelemetry(air physics.Vec3, geo physics.Geodetic) SceneTelemetry {
	frame := physics.NewLocalFrame(s.state.Position)
	body := s.attitude.Orientation.Body()

	out := SceneTelemetry{
		Forward: toScene(frame, body.Forward),
		Right:   toScene(frame, body.Right),
		Down:    toScene(frame, body.Down),
		Downrange: greatCircleDistance(
			s.Config.LaunchLatitude, s.Config.LaunchLongitude,
			geo.Latitude, geo.Longitude),
		Tower: s.tower.Offset(s.state.Position, s.elapsed),
	}

	// Поток идёт навстречу движению относительно воздуха.
	if speed := air.Norm(); speed > 1 {
		out.Airflow = toScene(frame, air.Scale(-1/speed))
	}
	ground := s.state.Velocity.Sub(physics.CorotatingVelocity(s.state.Position))
	if speed := ground.Norm(); speed > 1 {
		out.Velocity = toScene(frame, ground.Scale(1/speed))
	}
	return out
}
