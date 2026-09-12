package simulator

import (
	"log"
	"math"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Башня-ловушка.
//
// До сих пор цель возврата бустера была ТОЧКОЙ НА ЗЕМЛЕ: наведение целилось в
// координаты стартового комплекса, а успехом считалось касание поверхности с
// малой скоростью. Настоящая же цель Super Heavy — не земля: ступень обязана
// прийти в ПРОСВЕТ МЕЖДУ РУКАМИ башни на высоте нескольких десятков метров над
// стартовым столом, с подходящими скоростью и ориентацией, и быть пойманной.
// Разница не косметическая: горизонтальный промах, безобидный для посадки на
// опоры, здесь означает удар о руку или о ферму, а «мягкое касание» на нулевой
// высоте означает, что ступень пролетела мимо зоны захвата насквозь.
//
// Здесь описана геометрия башни и зона захвата, и считается промах ОТНОСИТЕЛЬНО
// ЦЕНТРА ЭТОЙ ЗОНЫ, а не относительно основания башни.
//
// Что здесь НАМЕРЕННО не делается: наведение не переключается на новую цель.
// Сначала нужно увидеть, какова настоящая ошибка относительно зоны захвата —
// поэтому башня, цель и диагностика прохождения плоскости рук появляются
// раньше, чем терминальная задача узнает про них. Иначе выпуклая задача будет
// переписана вслепую, под цель, ошибку относительно которой ещё никто не мерил.
// -----------------------------------------------------------------------------

// CatchTower — башня-ловушка (Mechazilla) и её зона захвата.
//
// Система координат башни, в которой считается всё остальное:
//
//	X — вдоль рук, от фермы наружу (к центру захвата);
//	Y — поперёк рук, то есть ПОПЕРЁК ПРОСВЕТА между ними;
//	Z — вверх от грунта у основания фермы.
//
// Разделение на «вдоль» и «поперёк» — не украшение. Допуск вдоль рук велик:
// руки длинные, и точка подхвата вдоль них смещается почти безнаказанно.
// Допуск поперёк — это половина просвета минус радиус корпуса, полтора-два
// метра: мимо него ступень бьётся о руку.
type CatchTower struct {
	// Pad — центр стартового стола. Он же точка прицеливания возврата
	// (launchTarget): центр зоны захвата стоит РОВНО НАД ним, поэтому
	// горизонтальная цель наведения не меняется, а вертикальная появляется.
	Pad physics.Geodetic `json:"pad"`

	// ArmAzimuth — азимут оси «ферма → просвет», градусы от севера к востоку.
	// Совпадает с азимутом пуска: бустер возвращается с того же направления,
	// куда уходил, и потому приходит ВДОЛЬ рук, а не поперёк.
	ArmAzimuth float64 `json:"armAzimuth"`

	// ArmReach — вынос центра захвата от оси фермы, м. Ферма стоит рядом со
	// столом, а не над ним.
	ArmReach float64 `json:"armReach"`

	TowerHeight float64 `json:"towerHeight"` // полная высота решётчатой фермы, м
	TowerWidth  float64 `json:"towerWidth"`  // сторона фермы в плане, м
	TowerBays   int     `json:"towerBays"`   // число секций фермы

	TableHeight float64 `json:"tableHeight"` // высота стартового стола, м
	TableRadius float64 `json:"tableRadius"` // радиус стола, м

	ArmLength    float64 `json:"armLength"`    // длина руки, м
	ArmGap       float64 `json:"armGap"`       // просвет между руками, м
	ArmThickness float64 `json:"armThickness"` // толщина руки по вертикали, м
	ArmWidth     float64 `json:"armWidth"`     // ширина руки в плане, м

	// ArmHeight — высота рук над грунтом, м. На этом уровне руки смыкаются
	// вокруг корпуса, и на нём же они рисуются.
	ArmHeight float64 `json:"armHeight"`

	// CatchHeight — высота ОПОРНОЙ ТОЧКИ корпуса (среза сопел, от которого
	// отсчитывается высота в телеметрии) в момент захвата, м. Именно её
	// пересечение и есть момент истины: горизонтальная ошибка снимается
	// здесь, а не после падения или заваливания корпуса.
	//
	// Она заметно НИЖЕ рук, и это не описка: ступень висит на цапфах под
	// решётчатыми рулями, то есть почти у самого верха корпуса, а срез
	// сопел при этом оказывается на три десятка метров над столом. Мерить
	// проход зоны по высоте рук значило бы сравнивать высоту корпуса с
	// высотой точки, в которой корпуса нет.
	CatchHeight float64 `json:"catchHeight"`

	// LugHeight — высота цапф над срезом сопел, м.
	LugHeight float64 `json:"lugHeight"`

	// Коридор захвата: допуски по осям башни, м.
	CorridorAlong  float64 `json:"corridorAlong"`
	CorridorAcross float64 `json:"corridorAcross"`
	CatchWindow    float64 `json:"catchWindow"` // полувысота окна захвата, м

	// Допуски на состояние в момент захвата.
	MaxVertical    float64 `json:"maxVertical"`    // скорость снижения, м/с
	MaxHorizontal  float64 `json:"maxHorizontal"`  // боковая скорость, м/с
	MaxTilt        float64 `json:"maxTilt"`        // наклон корпуса, град
	MaxAngularRate float64 `json:"maxAngularRate"` // угловая скорость, град/с

	// BodyRadius — радиус корпуса, который надо провести между руками.
	// Хранится здесь, а не берётся из конфигурации на месте: зона захвата
	// без него не определена, а сцене нужна та же величина.
	BodyRadius float64 `json:"bodyRadius"`

	// Орты и основание в ECEF считаются один раз при постройке: площадка
	// не движется, а раскладывать положение по осям башни приходится
	// каждый такт посадочного участка.
	along, across, up physics.Vec3
	base              physics.Vec3
}

// Геометрия башни Starbase, округлённая до целых метров.
//
// Числа не выдуманы: ферма — 146 м на девяти секциях со стороной около
// двенадцати; стартовый стол — двадцатиметровая тумба; руки — сорокапятиметровые
// балки с просветом, в который девятиметровый корпус проходит с зазором около
// двух с половиной метров на сторону. Высота захвата взята по точкам подхвата
// на уровне решётчатых рулей: пойманная ступень висит срезом сопел заметно выше
// стола, а не стоит на нём.
const (
	catchTowerHeight  = 146.0
	catchTowerWidth   = 12.0
	catchTowerBays    = 9
	catchTableHeight  = 20.0
	catchTableRadius  = 18.0
	catchArmLength    = 45.0
	catchArmGap       = 14.0
	catchArmThickness = 3.0
	catchArmWidth     = 4.0
	catchArmReach     = 40.0
	catchArmHeight    = 96.0

	// catchLugFraction — где по длине корпуса стоят цапфы: под решётчатыми
	// рулями, у самого верха ступени.
	catchLugFraction = 0.93
)

// Допуски захвата.
//
// Поперечный — половина просвета минус радиус корпуса, и ещё полметра на то,
// чтобы руки успели сомкнуться: пройти в притык и быть пойманным — разные
// события. Продольный — свободный ход точки подхвата вдоль рук. Скорости и
// углы — те же порядки, что у стыковки: подходить к неподвижной конструкции
// быстрее нескольких метров в секунду не во что.
const (
	catchCorridorAlong  = 6.0
	catchCorridorAcross = 2.0
	catchWindowHalf     = 5.0
	catchMaxVertical    = 3.0
	catchMaxHorizontal  = 2.0
	catchMaxTilt        = 5.0
	catchMaxAngularRate = 3.0
)

// CatchTowerFor — та же башня для внешних вызывающих (REST отдаёт геометрию
// сцене один раз, а не двадцать раз в секунду в каждом кадре).
func CatchTowerFor(cfg vehicle.Config) CatchTower { return newCatchTower(cfg) }

// newCatchTower строит башню для площадки из конфигурации носителя.
func newCatchTower(cfg vehicle.Config) CatchTower {
	azimuth, ok := physics.LaunchAzimuthForInclination(
		cfg.LaunchLatitude, cfg.TargetInclination, true)
	if !ok {
		azimuth = cfg.LaunchPadAzimuth
	}

	t := CatchTower{
		Pad: physics.Geodetic{
			Latitude:  cfg.LaunchLatitude,
			Longitude: cfg.LaunchLongitude,
		},
		ArmAzimuth:     azimuth,
		ArmReach:       catchArmReach,
		TowerHeight:    catchTowerHeight,
		TowerWidth:     catchTowerWidth,
		TowerBays:      catchTowerBays,
		TableHeight:    catchTableHeight,
		TableRadius:    catchTableRadius,
		ArmLength:      catchArmLength,
		ArmGap:         catchArmGap,
		ArmThickness:   catchArmThickness,
		ArmWidth:       catchArmWidth,
		ArmHeight:      catchArmHeight,
		LugHeight:      catchLugFraction * cfg.FirstStageLength,
		CatchHeight:    catchArmHeight - catchLugFraction*cfg.FirstStageLength,
		CorridorAlong:  catchCorridorAlong,
		CorridorAcross: catchCorridorAcross,
		CatchWindow:    catchWindowHalf,
		MaxVertical:    catchMaxVertical,
		MaxHorizontal:  catchMaxHorizontal,
		MaxTilt:        catchMaxTilt,
		MaxAngularRate: catchMaxAngularRate,
		BodyRadius:     cfg.Diameter / 2,
	}
	t.buildAxes()
	return t
}

// buildAxes считает орты системы башни в ECEF и основание фермы.
// Ферма стоит не над столом, а рядом: смещение от центра стола назад
// вдоль рук на ArmReach.
func (t *CatchTower) buildAxes() {
	padECEF := physics.GeodeticToECEF(t.Pad)
	local := physics.NewLocalFrame(padECEF)

	a := t.ArmAzimuth * physics.DegToRad
	t.along = local.East.Scale(math.Sin(a)).Add(local.North.Scale(math.Cos(a))).Unit()
	t.up = local.Up
	// Правая тройка: вдоль × поперёк = вверх.
	t.across = t.up.Cross(t.along).Unit()
	t.base = padECEF.Sub(t.along.Scale(t.ArmReach))
}

// Target возвращает центр зоны захвата в осях башни, м.
//
// Это и есть та самая точка, ради которой всё: середина между руками на высоте
// захвата, ровно над центром стартового стола.
func (t CatchTower) Target() (along, across, up float64) {
	return t.ArmReach, 0, t.CatchHeight
}

// decompose раскладывает точку (ECI на момент elapsedAt) по осям башни.
func (t CatchTower) decompose(positionECI physics.Vec3, elapsedAt float64) (along, across, up float64) {
	d := physics.ECIToECEF(positionECI, elapsedAt).Sub(t.base)
	return d.Dot(t.along), d.Dot(t.across), d.Dot(t.up)
}

// Offset возвращает вектор ОТ заданной точки К основанию фермы, разложенный
// в местном горизонте этой точки (восток, север, верх), — якорь, по которому
// трёхмерная сцена ставит башню относительно корпуса. Один вектор вместо
// широт и долгот: сцена не должна повторять преобразования координат, а на
// плоской земле кривизна на десятках километров даёт заметную ошибку.
func (t CatchTower) Offset(positionECI physics.Vec3, elapsedAt float64) SceneVector {
	baseECI := physics.ECEFToECI(t.base, elapsedAt)
	return toScene(physics.NewLocalFrame(positionECI), baseECI.Sub(positionECI))
}

// -----------------------------------------------------------------------------
// Прохождение плоскости захвата
// -----------------------------------------------------------------------------

// catchPass — что произошло на высоте захвата.
//
// Записывается РОВНО ОДИН РАЗ за полёт, в момент пересечения плоскости рук
// сверху вниз, и больше не меняется. Так и задумано: горизонтальную ошибку
// наведения нельзя оценивать по тому, где корпус оказался после того, как
// прошёл зону насквозь, упал и завалился, — это уже другая величина.
type catchPass struct {
	Valid   bool // плоскость пересечена, числа ниже осмысленны
	Crossed bool // пересечена сверху вниз (а не «ближайший подход»)

	Along, Across, Up float64 // положение в осях башни, м
	Vertical          float64 // скорость снижения, м/с
	Horizontal        float64 // боковая скорость, м/с
	Tilt              float64 // наклон корпуса от вертикали, град
	AngularRate       float64 // модуль угловой скорости, град/с

	Downrange  float64 // продольный промах вдоль курса отлёта, м
	Crossrange float64 // боковой промах поперёк него, м

	Success bool
}

// catchWatchAltitude — с какой высоты над плоскостью захвата начинается
// слежение за ближайшим подходом, м.
const catchWatchAltitude = 5000.0

// updateCatch следит за пересечением плоскости захвата.
//
// prev — состояние ДО шага интегрирования: момент пересечения ищется между
// двумя тактами и берётся линейной интерполяцией. Без неё на скорости в
// десяток метров в секунду и такте в пятьдесят миллисекунд точка прохода
// уезжала бы на полметра — соизмеримо с самим допуском поперёк рук.
func (b *Booster) updateCatch(prev VehicleState, prevElapsed float64) {
	if b.catch.Valid && b.catch.Crossed {
		return
	}

	h := b.tower.CatchHeight
	_, _, wasUp := b.tower.decompose(prev.Position, prevElapsed)
	_, _, nowUp := b.tower.decompose(b.state.Position, b.elapsed)

	// Ближайший подход — на случай, если плоскость так и не будет пересечена
	// (ступень зависла выше, разрушилась, ушла в сторону). Тогда промах по
	// вертикали не ноль, и это честнее, чем не показать ничего.
	//
	// Следить начинаем не с отделения, а с последних километров: снимок
	// состояния стоит пары преобразований координат, а «ближайший подход»
	// с восьмидесяти километров — это просто текущая высота, никому не
	// нужная.
	if nowUp < h+catchWatchAltitude && (!b.catch.Valid || nowUp < b.catch.Up) {
		b.recordCatch(b.state, b.elapsed, false)
	}

	if !(wasUp > h && nowUp <= h) {
		return
	}

	f := 0.0
	if d := wasUp - nowUp; d > 1e-9 {
		f = (wasUp - h) / d
	}
	at := VehicleState{
		Position: prev.Position.Add(b.state.Position.Sub(prev.Position).Scale(f)),
		Velocity: prev.Velocity.Add(b.state.Velocity.Sub(prev.Velocity).Scale(f)),
		FuelMass: b.state.FuelMass,
	}
	b.recordCatch(at, prevElapsed+(b.elapsed-prevElapsed)*f, true)

	if b.catch.Success {
		b.captureByTower(at)
	}
}

// captureByTower останавливает ступень в руках башни.
//
// Это единственное штатное окончание возврата Super Heavy. Ступень не
// касается ни воды, ни грунта: руки смыкаются на цапфах, и дальше она висит
// неподвижно относительно площадки — поэтому скорость приводится к
// переносной (вместе с Землёй), а не к нулю в инерциальных осях, где
// «неподвижно» означало бы падение.
func (b *Booster) captureByTower(at VehicleState) {
	b.state.Position = at.Position
	b.state.Velocity = physics.CorotatingVelocity(at.Position)
	b.phase = BoosterCaught
	b.propulsion.StopAll()
	b.freezePosition()

	log.Printf("🤝 Бустер пойман башней на T+%.1f с: промах %.1f м "+
		"(вдоль %.1f, поперёк %.1f), снижение %.1f м/с, снос %.1f м/с, "+
		"наклон %.1f°, вращение %.1f°/с",
		b.elapsed, math.Hypot(b.catch.Along-b.tower.ArmReach, b.catch.Across),
		b.catch.Along-b.tower.ArmReach, b.catch.Across,
		b.catch.Vertical, b.catch.Horizontal, b.catch.Tilt, b.catch.AngularRate)
}

// recordCatch снимает состояние в точке сравнения.
func (b *Booster) recordCatch(st VehicleState, at float64, crossed bool) {
	along, across, up := b.tower.decompose(st.Position, at)

	frame := physics.NewLocalFrame(st.Position)
	ground := st.Velocity.Sub(physics.CorotatingVelocity(st.Position))
	vertical := -ground.Dot(frame.Up)
	horizontal := ground.Sub(frame.Up.Scale(ground.Dot(frame.Up))).Norm()

	tilt := 0.0
	if f := b.attitude.Orientation.Rotate(physics.Vec3{X: 1}); f.Norm() > 1e-9 {
		tilt = math.Acos(physics.Clamp(f.Unit().Dot(frame.Up), -1, 1)) * physics.RadToDeg
	}

	_, downrange, crossrange := b.landingMiss(st.Position, at)

	p := catchPass{
		Valid:       true,
		Crossed:     crossed,
		Along:       along,
		Across:      across,
		Up:          up,
		Vertical:    vertical,
		Horizontal:  horizontal,
		Tilt:        tilt,
		AngularRate: b.attitude.Omega.Norm() * physics.RadToDeg,
		Downrange:   downrange,
		Crossrange:  crossrange,
	}
	p.Success = crossed && b.tower.caught(p)
	b.catch = p
}

// caught — условие успешного захвата.
//
// Мимо просвета пройти нельзя: корпус радиуса BodyRadius должен уместиться
// между руками, и это требование ЖЁСТЧЕ коридора наведения — коридор задаёт,
// с какой точностью хотелось бы прийти, а просвет задаёт, что физически
// помещается. Проверяются оба: первый — качество наведения, второй — сам факт
// отсутствия удара о руку.
func (t CatchTower) caught(p catchPass) bool {
	tAlong, tAcross, tUp := t.Target()

	if math.Abs(p.Along-tAlong) > t.CorridorAlong {
		return false
	}
	if math.Abs(p.Across-tAcross) > t.CorridorAcross {
		return false
	}
	if math.Abs(p.Across-tAcross) > t.ArmGap/2-t.BodyRadius {
		return false
	}
	if math.Abs(p.Up-tUp) > t.CatchWindow {
		return false
	}
	if p.Vertical > t.MaxVertical || p.Vertical < -t.MaxVertical {
		return false
	}
	if p.Horizontal > t.MaxHorizontal {
		return false
	}
	if p.Tilt > t.MaxTilt || p.AngularRate > t.MaxAngularRate {
		return false
	}
	return true
}

// -----------------------------------------------------------------------------
// Телеметрия захвата
// -----------------------------------------------------------------------------

// CatchTelemetry — цель, факт и промах в осях башни.
//
// Величины публикуются как NaN, пока сравнивать не с чем: ноль здесь —
// осмысленное «промах нулевой», и подменять им «ещё не долетели» нельзя.
type CatchTelemetry struct {
	// Геометрия — чтобы сцена и пульт не повторяли константы.
	TargetX float64 `json:"catchTargetX"`
	TargetY float64 `json:"catchTargetY"`
	TargetZ float64 `json:"catchTargetZ"`

	ActualX Float `json:"catchActualX"`
	ActualY Float `json:"catchActualY"`
	ActualZ Float `json:"catchActualZ"`

	MissX Float `json:"catchMissX"`
	MissY Float `json:"catchMissY"`
	MissZ Float `json:"catchMissZ"`

	MissHorizontal Float `json:"catchMissHorizontal"`
	Miss3D         Float `json:"catchMiss3D"`

	Downrange  Float `json:"catchMissDownrange"`
	Crossrange Float `json:"catchMissCrossrange"`

	VerticalVelocity   Float `json:"catchVerticalVelocity"`
	HorizontalVelocity Float `json:"catchHorizontalVelocity"`
	Tilt               Float `json:"catchTilt"`
	AngularRate        Float `json:"catchAngularRate"`

	// Crossed — плоскость захвата пересечена сверху вниз. Пока false,
	// числа выше описывают ближайший подход, а не проход зоны.
	Crossed bool `json:"catchCrossed"`
	Success bool `json:"catchSuccess"`
}

// catchTelemetry собирает телеметрию захвата.
func catchTelemetry(t CatchTower, p catchPass) CatchTelemetry {
	tx, ty, tz := t.Target()
	out := CatchTelemetry{
		TargetX: tx, TargetY: ty, TargetZ: tz,
		ActualX: Float(math.NaN()), ActualY: Float(math.NaN()), ActualZ: Float(math.NaN()),
		MissX: Float(math.NaN()), MissY: Float(math.NaN()), MissZ: Float(math.NaN()),
		MissHorizontal: Float(math.NaN()), Miss3D: Float(math.NaN()),
		Downrange: Float(math.NaN()), Crossrange: Float(math.NaN()),
		VerticalVelocity:   Float(math.NaN()),
		HorizontalVelocity: Float(math.NaN()),
		Tilt:               Float(math.NaN()),
		AngularRate:        Float(math.NaN()),
	}
	if !p.Valid {
		return out
	}

	dx, dy, dz := p.Along-tx, p.Across-ty, p.Up-tz
	out.ActualX, out.ActualY, out.ActualZ = Float(p.Along), Float(p.Across), Float(p.Up)
	out.MissX, out.MissY, out.MissZ = Float(dx), Float(dy), Float(dz)
	out.MissHorizontal = Float(math.Hypot(dx, dy))
	out.Miss3D = Float(math.Sqrt(dx*dx + dy*dy + dz*dz))
	out.Downrange, out.Crossrange = Float(p.Downrange), Float(p.Crossrange)
	out.VerticalVelocity = Float(p.Vertical)
	out.HorizontalVelocity = Float(p.Horizontal)
	out.Tilt = Float(p.Tilt)
	out.AngularRate = Float(p.AngularRate)
	out.Crossed = p.Crossed
	out.Success = p.Success
	return out
}
