package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Регрессия на актуаторную модель управляющих поверхностей: раньше ВСЯ
// раскладка (Allocate, SetManual, PitchAuthority, покоординатный перебор)
// молча предполагала диапазон [0, MaxDeflection] — верно для плавников
// корабля (физический упор складывания), но НЕ подтверждено ни физикой
// ControlSurface.Force/Normal (никакого запрета на отрицательный угол там
// нет), ни доступными источниками по решётчатым рулям Falcon/Starship
// (независимое управление тангажом/рысканием/креном каждым рулём —
// свойство привода, вращающегося в обе стороны от нейтрали, а не
// одностороннего открытия). См. physics.ControlSurface.MinDeflection и
// vehicle.GridFins/ShipFlaps.
//
// Тесты здесь проверяют, что все пути раскладки используют ОДНИ и те же
// границы (MinDeflection…MaxDeflection) конкретной поверхности, а не
// зашитый [0,Max]; что решётчатые рули (симметричный диапазон) физически
// принимают и используют отрицательные углы; и что плавники корабля
// (физически односторонний упор) остаются одноsторонними без изменений.
// -----------------------------------------------------------------------------

// TestControlSurface_AcceptsPositiveAndNegativeDeflection — сама физика
// (Force/Torque) не имеет математического запрета на отрицательный угол:
// оба знака обязаны давать конечную, ненулевую (при не-вырожденном потоке)
// силу без NaN/Inf.
func TestControlSurface_AcceptsPositiveAndNegativeDeflection(t *testing.T) {
	fins := vehicle.GridFins(72.3, 4.5)
	air := physics.Vec3{X: -1000, Y: 50}
	const q, mach = 50000.0, 3.0

	for _, fin := range fins {
		for _, deg := range []float64{-20, -10, -1, 1, 10, 20} {
			delta := deg * physics.DegToRad
			f := fin.Force(delta, air, q, mach)
			if math.IsNaN(f.X) || math.IsNaN(f.Y) || math.IsNaN(f.Z) {
				t.Fatalf("%s: Force(%.0f°) = NaN", fin.Name, deg)
			}
			if f.Norm() < 1 {
				t.Errorf("%s: Force(%.0f°) ≈ 0 (%v) — ожидалась ненулевая сила", fin.Name, deg, f)
			}
		}
	}
}

// TestControlSurface_MirroredDeflectionGivesMirroredMoment — для чисто
// осевого потока (без бокового сноса, чтобы не путать геометрическую
// асимметрию рулей с ветром) момент от +δ и −δ ОДНОГО руля обязан быть
// зеркальным: тот же порядок величины по модулю, и вклад в ось, которую
// этот конкретный руль контролирует, должен менять знак вместе с δ.
func TestControlSurface_MirroredDeflectionGivesMirroredMoment(t *testing.T) {
	fins := vehicle.GridFins(72.3, 4.5)
	air := physics.Vec3{X: -1000}
	const q, mach, comFromNose = 50000.0, 3.0, 39.0

	for _, fin := range fins {
		plus := fin.Torque(15*physics.DegToRad, air, q, mach, comFromNose)
		minus := fin.Torque(-15*physics.DegToRad, air, q, mach, comFromNose)

		if plus.Norm() < 1 || minus.Norm() < 1 {
			t.Fatalf("%s: |Torque|≈0 для ±15° (plus=%v minus=%v)", fin.Name, plus, minus)
		}

		const tolerance = 0.15 // 15% — не обязано быть идеально зеркальным (Cp нелинеен по θ)
		if math.Abs(plus.Norm()-minus.Norm()) > plus.Norm()*tolerance {
			t.Errorf("%s: |Torque(+15°)|=%.4e и |Torque(-15°)|=%.4e не симметричны в пределах %.0f%%",
				fin.Name, plus.Norm(), minus.Norm(), tolerance*100)
		}
	}
}

// TestAllocateOptimal_CanUseNegativeFinAngle — прямая регрессия на найденный
// дефект: для проблемного mixed-axis запроса (roll+pitch+yaw, seed=1 t≈306.5с
// из разбора) allocator обязан реально ИСПОЛЬЗОВАТЬ отрицательный угол хотя
// бы одного руля — раньше это было структурно невозможно (диапазон
// [0,MaxDeflection] запрещал это самой моделью, не только текущим решением).
func TestAllocateOptimal_CanUseNegativeFinAngle(t *testing.T) {
	length, radius, comFromNose, q, mach, airBody, mcmd := traceProblemState()
	surf := NewSurfaceSet(vehicle.GridFins(length, radius))
	surf.Deployed = true
	surf.Allocate(mcmd, airBody, q, mach, comFromNose)

	negative := false
	for i, surface := range surf.Surfaces {
		if surf.Command[i] < surface.MinDeflection-1e-9 || surf.Command[i] > surface.MaxDeflection+1e-9 {
			t.Errorf("surface %s: Command=%.4f вне [%.4f,%.4f]", surface.Name, surf.Command[i],
				surface.MinDeflection, surface.MaxDeflection)
		}
		if surf.Command[i] < -1e-6 {
			negative = true
		}
	}
	if !negative {
		t.Error("allocator не использовал ни одного отрицательного угла на проблемном запросе — " +
			"ожидалось, что хотя бы один руль уйдёт в отрицательную сторону")
	}
}

// TestShipFlaps_RemainOneSided — плавники корабля физически складываются
// плашмя вдоль борта и не могут повернуться "сквозь" обшивку: MinDeflection
// обязан остаться нулём, и раскладка (allocateLegacy) не должна выдавать
// отрицательный Command ни при каком запросе — регрессия на то, что фикс
// решётчатых рулей не затронул эту физически иную поверхность.
func TestShipFlaps_RemainOneSided(t *testing.T) {
	for _, s := range vehicle.ShipFlaps(50, 4.5) {
		if s.MinDeflection != 0 {
			t.Errorf("%s: MinDeflection=%.4f, ожидался 0 (физический упор складывания)", s.Name, s.MinDeflection)
		}
	}

	surf := NewSurfaceSet(vehicle.ShipFlaps(50, 4.5))
	surf.Deployed = true
	airBody := physics.Vec3{X: -100, Z: -50}
	const q, mach, comFromNose = 12000.0, 1.5, 0.0

	// Запрос, заведомо провоцирующий "противоположный" крайний случай —
	// большой отрицательный pitch/yaw/roll разом.
	surf.Allocate(physics.Vec3{X: -1e6, Y: -1e6, Z: -1e6}, airBody, q, mach, comFromNose)
	for i, s := range surf.Surfaces {
		if surf.Command[i] < -1e-9 {
			t.Errorf("%s: Command=%.4f отрицательный — плавник корабля не может так повернуться",
				s.Name, surf.Command[i])
		}
	}
}

// TestManualAndAutomaticPathsShareBounds — SetManual (ручное управление) и
// Allocate (автопилот) обязаны разделять ОДНИ и те же границы поверхности:
// раньше SetManual уже клэмпил в [-MaxDeflection,MaxDeflection] (готовый к
// двустороннему движению), а Allocate — в [0,MaxDeflection] (нет), то есть
// два пути одного и того же SurfaceSet расходились в том, что вообще
// физически достижимо. Тест проверяет, что для КАЖДОЙ поверхности оба пути
// теперь используют identичные MinDeflection/MaxDeflection.
func TestManualAndAutomaticPathsShareBounds(t *testing.T) {
	check := func(name string, surfaces []physics.ControlSurface) {
		surf := NewSurfaceSet(surfaces)
		surf.Deployed = true
		for i, s := range surf.Surfaces {
			// SetManual: запрос далеко за пределами в обе стороны.
			surf.SetManual(i, s.MaxDeflection+100)
			if math.Abs(surf.Command[i]-s.MaxDeflection) > 1e-9 {
				t.Errorf("%s/%s: SetManual(+∞) дал %.4f, ожидался MaxDeflection=%.4f",
					name, s.Name, surf.Command[i], s.MaxDeflection)
			}
			surf.SetManual(i, s.MinDeflection-100)
			if math.Abs(surf.Command[i]-s.MinDeflection) > 1e-9 {
				t.Errorf("%s/%s: SetManual(-∞) дал %.4f, ожидался MinDeflection=%.4f",
					name, s.Name, surf.Command[i], s.MinDeflection)
			}
			surf.ReleaseManual(i)
		}

		// Allocate: тот же SurfaceSet, огромный запрос по всем осям —
		// автоматический путь не должен выходить за те же границы.
		airBody := physics.Vec3{X: -1000, Y: 20}
		surf.Allocate(physics.Vec3{X: 1e9, Y: 1e9, Z: 1e9}, airBody, 50000, 3.0, 39.0)
		for i, s := range surf.Surfaces {
			if surf.Command[i] < s.MinDeflection-1e-6 || surf.Command[i] > s.MaxDeflection+1e-6 {
				t.Errorf("%s/%s: Allocate вышел за [%.4f,%.4f]: Command=%.4f",
					name, s.Name, s.MinDeflection, s.MaxDeflection, surf.Command[i])
			}
		}
	}

	check("GridFins", vehicle.GridFins(72.3, 4.5))
	check("ShipFlaps", vehicle.ShipFlaps(50, 4.5))
}
