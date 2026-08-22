package metrics

import (
	"strings"
	"testing"

	"github.com/prometheus/client_golang/prometheus"
	dto "github.com/prometheus/client_model/go"
)

// -----------------------------------------------------------------------------
// Разметка метрик двигательной установки.
//
// Двигателей на ступени девять, и у каждого свой турбонасос. Без метки
// engine_id в Grafana попадал бы только один из них, и отказ одного агрегата
// был бы неотличим от отказа всей ступени.
// -----------------------------------------------------------------------------

// collect собирает текущие значения метрики по наборам меток.
func collect(t *testing.T, c prometheus.Collector) []*dto.Metric {
	t.Helper()

	ch := make(chan prometheus.Metric, 256)
	go func() {
		c.Collect(ch)
		close(ch)
	}()

	var out []*dto.Metric
	for m := range ch {
		d := &dto.Metric{}
		if err := m.Write(d); err != nil {
			t.Fatalf("не удалось прочитать метрику: %v", err)
		}
		out = append(out, d)
	}
	return out
}

// labelOf возвращает значение метки.
func labelOf(m *dto.Metric, name string) string {
	for _, l := range m.Label {
		if l.GetName() == name {
			return l.GetValue()
		}
	}
	return ""
}

// Метрики агрегата обязаны различать двигатели.
func TestPropulsionMetricsAreLabelledByEngine(t *testing.T) {
	ids := []string{"S1-1", "S1-2", "S1-3"}
	for i, id := range ids {
		SetEngineDetail(id, EngineDetailSample{
			ShaftRPM:        32000 + float64(i)*100,
			ChamberPressure: 9.7e6,
			WallStationName: []string{"throat"},
			WallStationTemp: []float64{1100},
		})
	}

	got := make(map[string]float64)
	for _, m := range collect(t, tpRPM) {
		got[labelOf(m, "engine_id")] = m.GetGauge().GetValue()
	}

	for i, id := range ids {
		want := 32000 + float64(i)*100
		if v, ok := got[id]; !ok {
			t.Errorf("метрика оборотов не содержит двигатель %s", id)
		} else if v != want {
			t.Errorf("%s: обороты %.0f, ожидалось %.0f", id, v, want)
		}
	}

	// Участки стенки размечены и двигателем, и сечением.
	var found bool
	for _, m := range collect(t, wallStationTemp) {
		if labelOf(m, "engine_id") == "S1-2" && labelOf(m, "station") == "throat" {
			found = true
		}
	}
	if !found {
		t.Error("температура стенки не размечена двигателем и участком одновременно")
	}
}

// После разделения ступеней метки улетевших двигателей должны исчезать.
func TestRetireEnginesClearsPropulsionMetrics(t *testing.T) {
	SetEngineDetail("S1-9", EngineDetailSample{
		ShaftRPM:        31000,
		WallStationName: []string{"throat"},
		WallStationTemp: []float64{900},
	})

	present := func(c prometheus.Collector, id string) bool {
		for _, m := range collect(t, c) {
			if labelOf(m, "engine_id") == id {
				return true
			}
		}
		return false
	}

	if !present(tpRPM, "S1-9") {
		t.Fatal("метрика не опубликована")
	}

	RetireEngines([]string{"S1-9"})

	if present(tpRPM, "S1-9") {
		t.Error("обороты отделившегося двигателя остались на графике")
	}
	if present(wallStationTemp, "S1-9") {
		t.Error("температура стенки отделившегося двигателя осталась на графике")
	}
}

// Величины ступени метки двигателя иметь не должны: бак на ступени один.
func TestStageMetricsHaveNoEngineLabel(t *testing.T) {
	SetFuelTank(TankSample{Mass: 1000, Pressure: 3e5})

	for _, m := range collect(t, tankFuelPressure) {
		if id := labelOf(m, "engine_id"); id != "" {
			t.Errorf("давление в баке размечено двигателем %q, хотя бак общий", id)
		}
	}
}

// Каждая метрика двигателя должна быть перечислена в списке снимаемых:
// забытая метрика переживёт разделение ступеней и застынет на графике.
func TestAllEngineMetricsAreRetirable(t *testing.T) {
	retirable := make(map[string]bool)
	for _, v := range enginePropulsionVecs() {
		retirable[describeName(t, v)] = true
	}

	for _, c := range propulsionCollectors() {
		v, ok := c.(*prometheus.GaugeVec)
		if !ok {
			continue
		}
		name := describeName(t, v)
		// Метрики с дополнительными метками снимаются частичным совпадением.
		switch name {
		case "chamber_pressure_spectrum_pa", "nozzle_wall_temperature_k",
			"nozzle_wall_heat_flux_w_m2", "nozzle_wall_station_margin_k":
			continue
		}
		if !retirable[name] {
			t.Errorf("метрика %s размечена двигателем, но не снимается при разделении",
				name)
		}
	}
}

// describeName возвращает имя метрики из её описания.
func describeName(t *testing.T, c prometheus.Collector) string {
	t.Helper()

	ch := make(chan *prometheus.Desc, 4)
	go func() {
		c.Describe(ch)
		close(ch)
	}()

	for d := range ch {
		// Описание имеет вид Desc{fqName: "имя", ...}.
		s := d.String()
		const marker = `fqName: "`
		i := strings.Index(s, marker)
		if i < 0 {
			continue
		}
		s = s[i+len(marker):]
		if j := strings.IndexByte(s, '"'); j >= 0 {
			return s[:j]
		}
	}
	return ""
}
