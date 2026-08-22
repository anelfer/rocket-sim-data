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
	limit := s.Surfaces[index].MaxDeflection
	s.Manual[index] = true
	s.Command[index] = math.Max(-limit, math.Min(limit, angle))
}

// ReleaseManual возвращает поверхность автопилоту.
func (s *SurfaceSet) ReleaseManual(index int) {
	if s == nil || index < 0 || index >= len(s.Manual) {
		return
	}
	s.Manual[index] = false
}

// Allocate раскладывает потребные моменты по поверхностям.
//
// Схема та же, что на корабле: тангаж отрабатывается разностью между парой
// передних плавников и парой задних, крен — разностью между левыми и правыми.
// Средний угол раскрытия держится посередине хода, чтобы запас оставался
// в обе стороны.
func (s *SurfaceSet) Allocate(pitch, yaw, roll float64) {
	if s == nil || len(s.Surfaces) == 0 {
		return
	}

	for i, surface := range s.Surfaces {
		if s.Manual[i] {
			continue
		}

		// Не выпущены — прижаты к борту, то есть на нуле хода.
		if !s.Deployed {
			s.Command[i] = 0
			continue
		}

		limit := surface.MaxDeflection
		mid := limit * 0.5

		// Расположение поверхности определяет её роль: знак плеча вдоль
		// корпуса задаёт вклад в тангаж, знак поперёк — в крен.
		fore := 1.0
		if surface.Arm.X < 0 {
			fore = -1
		}
		side := 1.0
		if surface.Arm.Y < 0 {
			side = -1
		}

		// Чтобы поднять нос, передние плавники раскрывают, а задние прижимают:
		// момент создаёт разность их сил. Раскрыть — значит увеличить угол,
		// отсюда прямой знак.
		//
		// Каналы неравноправны. Главный — тангаж: им держат угол атаки,
		// от которого зависит и торможение, и нагрев. Крен и рыскание
		// получают остаток хода, иначе они забирают весь ход приводов
		// себе, тангаж остаётся без власти, и корпус опрокидывается.
		const (
			pitchShare = 1.0
			rollShare  = 0.30
			yawShare   = 0.15
		)

		angle := mid +
			fore*pitch*limit*pitchShare +
			side*roll*limit*rollShare +
			side*yaw*limit*yawShare
		s.Command[i] = math.Max(0, math.Min(limit, angle))
	}
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

// UpdateTorque считает суммарный момент от поверхностей.
func (s *SurfaceSet) UpdateTorque(airBody physics.Vec3, dynamicPressure, mach float64) physics.Vec3 {
	if s == nil {
		return physics.Vec3{}
	}

	total := physics.Vec3{}
	for i, surface := range s.Surfaces {
		total = total.Add(surface.Torque(s.Deflection[i], airBody, dynamicPressure, mach))
	}
	s.Torque = total
	return total
}

// PitchAuthority возвращает располагаемый момент по тангажу при полном
// перекладывании поверхностей, Н·м.
//
// Нужен, чтобы честно показывать оператору, чем он вообще располагает:
// в разрежённых слоях эта величина близка к нулю, и никакое отклонение
// плавников корпус не развернёт.
func (s *SurfaceSet) PitchAuthority(airBody physics.Vec3, dynamicPressure, mach float64) float64 {
	if s == nil {
		return 0
	}

	// Власть — это разность между крайними положениями привода, а не момент
	// в крайнем положении. Прижатый плавник стоит почти по потоку и не даёт
	// почти ничего; работает именно разница между раскрытым и прижатым.
	full := 0.0
	for _, surface := range s.Surfaces {
		fore := 1.0
		if surface.Arm.X < 0 {
			fore = -1
		}

		open := surface.Torque(surface.MaxDeflection, airBody, dynamicPressure, mach)
		shut := surface.Torque(0, airBody, dynamicPressure, mach)
		full += fore * (open.Y - shut.Y)
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
