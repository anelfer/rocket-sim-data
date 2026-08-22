package metrics

import (
	"math"
	"testing"

	"github.com/prometheus/client_golang/prometheus"
)

// gatheredNames возвращает имена метрик, которые сейчас отдаёт реестр.
func gatheredNames(t *testing.T) map[string]bool {
	t.Helper()
	families, err := prometheus.DefaultGatherer.Gather()
	if err != nil {
		t.Fatalf("не удалось собрать метрики: %v", err)
	}
	names := make(map[string]bool, len(families))
	for _, f := range families {
		names[f.GetName()] = true
	}
	return names
}

// До первой публикации телеметрии метрик быть не должно.
//
// Зарегистрированный gauge отдаёт ноль, пока в него не записали значение.
// Prometheus собирал такие нули раз в секунду, и на карте Grafana появлялась
// точка с координатами (0°, 0°) в Гвинейском заливе, от которой в момент
// старта шёл скачок на стартовую площадку.
func TestNoMetricsBeforeFirstSample(t *testing.T) {
	if Registered() {
		t.Skip("метрики уже зарегистрированы другим тестом")
	}

	names := gatheredNames(t)
	for _, name := range []string{
		"rocket_latitude",
		"rocket_longitude",
		"rocket_altitude_meters",
		"rocket_mass_kg",
		"rocket_total_thrust_newton",
	} {
		if names[name] {
			t.Errorf("метрика %s отдаётся до старта симуляции", name)
		}
	}
}

// После первой публикации метрики появляются и содержат переданные значения.
func TestMetricsAppearAfterFirstSample(t *testing.T) {
	SetFlight(FlightSample{
		Altitude:  0,
		Latitude:  45.9647,
		Longitude: 63.3050,
		Mass:      565000,
	})

	if !Registered() {
		t.Fatal("после публикации метрики должны быть зарегистрированы")
	}

	names := gatheredNames(t)
	for _, name := range []string{"rocket_latitude", "rocket_longitude", "rocket_altitude_meters"} {
		if !names[name] {
			t.Errorf("метрика %s не появилась после публикации", name)
		}
	}

	// Координаты должны быть стартовой площадкой, а не нулями.
	families, _ := prometheus.DefaultGatherer.Gather()
	for _, f := range families {
		if f.GetName() != "rocket_latitude" {
			continue
		}
		got := f.GetMetric()[0].GetGauge().GetValue()
		if got < 45.9 || got > 46.0 {
			t.Errorf("широта = %.4f, ожидалась широта стартовой площадки", got)
		}
	}
}

// gaugeValue возвращает значение метрики по имени.
func gaugeValue(t *testing.T, name string) (float64, bool) {
	t.Helper()
	families, err := prometheus.DefaultGatherer.Gather()
	if err != nil {
		t.Fatalf("не удалось собрать метрики: %v", err)
	}
	for _, f := range families {
		if f.GetName() != name {
			continue
		}
		m := f.GetMetric()
		if len(m) == 0 {
			return 0, false
		}
		return m[0].GetGauge().GetValue(), true
	}
	return 0, false
}

// Координаты, для которых данных ещё нет, обязаны быть NaN, а не нулём.
//
// Ноль — это осмысленная точка на карте: (0°, 0°) в Гвинейском заливе.
// Зона падения обломков появляется только после разрушения ступени, и до
// этого момента Grafana рисовала отметку в центре мира, а затем линию через
// всю карту к реальной точке.
func TestUndefinedCoordinatesAreNaN(t *testing.T) {
	// Публикация любой метрики регистрирует весь набор.
	SetFlight(FlightSample{Latitude: 45.9647, Longitude: 63.3050})

	for _, name := range []string{
		"rocket_debris_center_latitude",
		"rocket_debris_center_longitude",
		"rocket_debris_semi_major_axis_meters",
		"rocket_debris_semi_minor_axis_meters",
		"rocket_debris_surviving_mass_fraction",
	} {
		v, ok := gaugeValue(t, name)
		if !ok {
			t.Errorf("метрика %s не зарегистрирована", name)
			continue
		}
		if !math.IsNaN(v) {
			t.Errorf("%s = %v до появления данных, ожидался NaN", name, v)
		}
	}
}

// После публикации данных обломков координаты становятся конечными.
func TestDebrisCoordinatesBecomeDefined(t *testing.T) {
	SetDebris(DebrisSample{
		CenterLatitude:  48.45,
		CenterLongitude: 70.76,
		SemiMajorAxis:   4000,
		SemiMinorAxis:   2700,
		SurvivingMass:   1.0,
	})

	v, ok := gaugeValue(t, "rocket_debris_center_latitude")
	if !ok || math.IsNaN(v) {
		t.Fatal("после публикации координаты обломков должны быть определены")
	}
	if math.Abs(v-48.45) > 1e-9 {
		t.Errorf("широта = %v, ожидалось 48.45", v)
	}
}

// Повторные публикации не должны приводить к повторной регистрации:
// prometheus.MustRegister паникует при дубликате.
func TestRepeatedPublishDoesNotPanic(t *testing.T) {
	for i := 0; i < 5; i++ {
		SetFlight(FlightSample{Altitude: float64(i)})
		SetOrbit(OrbitSample{Apoapsis: float64(i)})
		SetEngine(EngineSample{ID: "S1-1", Thrust: float64(i)})
	}
}
