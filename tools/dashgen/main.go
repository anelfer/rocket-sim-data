// Команда dashgen выгружает реестр параметров двигательной установки в JSON.
//
// Пороговые значения панелей Grafana берутся из этого реестра, а не задаются
// в дашборде вручную: иначе подпись на графике и поведение модели неизбежно
// разойдутся.
package main

import (
	"encoding/json"
	"fmt"
	"math"
	"os"

	"rocketTelemetrySim/simulator/propulsion"
)

type entry struct {
	Metric     string   `json:"metric"`
	Title      string   `json:"title"`
	Unit       string   `json:"unit"`
	Initial    *float64 `json:"initial"`
	NominalMin *float64 `json:"nominalMin"`
	NominalMax *float64 `json:"nominalMax"`
	WarningMin *float64 `json:"warningMin"`
	WarningMax *float64 `json:"warningMax"`
	Derivation string   `json:"derivation"`
	Related    []string `json:"related"`
	PanelType  string   `json:"panelType"`
}

func finite(v float64) *float64 {
	if math.IsInf(v, 0) || math.IsNaN(v) {
		return nil
	}
	return &v
}

func main() {
	reg := propulsion.ParameterRegistry()
	out := make([]entry, 0, len(reg.All()))
	for _, p := range reg.All() {
		out = append(out, entry{
			Metric: p.Metric, Title: p.Title, Unit: p.Unit,
			Initial:    finite(p.Initial),
			NominalMin: finite(p.Nominal.Min), NominalMax: finite(p.Nominal.Max),
			WarningMin: finite(p.Warning.Min), WarningMax: finite(p.Warning.Max),
			Derivation: p.Derivation, Related: p.Related, PanelType: p.PanelType,
		})
	}
	enc := json.NewEncoder(os.Stdout)
	enc.SetIndent("", "  ")
	if err := enc.Encode(out); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
