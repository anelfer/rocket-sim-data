package vehicle

import (
	"math"
	"testing"
)

// Каждый профиль должен быть цельным: массы, тяга и геометрия согласованы.
func TestProfilesAreConsistent(t *testing.T) {
	for _, p := range Profiles() {
		cfg := p.Config()

		if p.Title == "" || p.Summary == "" {
			t.Errorf("%s: не заполнено описание", p.ID)
		}
		if cfg.FirstStage.EngineCount != float64(p.FirstStageEngines) {
			t.Errorf("%s: в каталоге %d двигателей первой ступени, в конфигурации %.0f",
				p.ID, p.FirstStageEngines, cfg.FirstStage.EngineCount)
		}
		if cfg.SecondStage.EngineCount != float64(p.SecondStageEngines) {
			t.Errorf("%s: в каталоге %d двигателей второй ступени, в конфигурации %.0f",
				p.ID, p.SecondStageEngines, cfg.SecondStage.EngineCount)
		}

		// Стартовая тяговооружённость обязана быть больше единицы,
		// иначе ракета не оторвётся от стола.
		mass := cfg.FirstStage.DryMass + cfg.FirstStage.FuelMass +
			cfg.SecondStage.DryMass + cfg.SecondStage.FuelMass +
			cfg.PayloadMass + cfg.FairingMass
		thrust := cfg.FirstStage.SeaLevelThust * cfg.FirstStage.EngineCount
		twr := thrust / (mass * 9.80665)
		if twr < 1.15 || twr > 2.0 {
			t.Errorf("%s: стартовая тяговооружённость %.2f вне разумного диапазона",
				p.ID, twr)
		}

		if math.Abs(mass/1000-p.LiftoffMass) > p.LiftoffMass*0.05 {
			t.Errorf("%s: в каталоге стартовая масса %.0f т, в конфигурации %.0f т",
				p.ID, p.LiftoffMass, mass/1000)
		}
		t.Logf("%-10s %d + %d двигателей, %.0f т, %.1f МН, тяговооружённость %.2f",
			p.ID, p.FirstStageEngines, p.SecondStageEngines,
			mass/1000, thrust/1e6, twr)
	}
}
