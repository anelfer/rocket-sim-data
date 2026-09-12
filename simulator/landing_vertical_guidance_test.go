package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/orbit"
	"rocketTelemetrySim/simulator/physics"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Регрессия на Stage 4.3 (state-based vertical powered descent): required
// thrust строится из текущего кинематического состояния (высота, вертикальная
// скорость, масса, аэродинамика) через явный, сглаженный по времени эталон
// замедления (landingAccelBudget/updateLandingAccelBudget) — а не из заранее
// придуманной кривой газа. Тесты здесь проверяют свойства этой зависимости
// аналитически на синтетических состояниях (см. Stage 4.3, п.5,17,18), без
// полного прогона миссии.
// -----------------------------------------------------------------------------

// primeRequiredThrust "прогревает" b.landingAccelBudgetSmoothed до сырого
// (несглаженного, dt<=0 внутри updateLandingAccelBudget) значения для
// заданного nav, и сразу возвращает landingRequiredThrust на этом же nav —
// так каждый синтетический сценарий видит закон в чистом виде, без
// переходного процесса, унаследованного от предыдущего сценария теста.
func primeRequiredThrust(b *Booster, nav orbit.NavState) float64 {
	b.landingAccelBudgetSmoothed = 0
	b.updateLandingAccelBudget(nav, 0)
	return b.landingRequiredThrust(nav)
}

// TestVerticalGuidance_HighSpeedNeedsMoreThrust — п.17,18 Stage 4.3: два
// состояния на ОДНОЙ высоте, но с разной вертикальной скоростью, не могут
// требовать одинаковую тягу только потому что высота совпала. Более
// отрицательная (быстрее падающая) Vz обязана требовать БОЛЬШУЮ тягу.
func TestVerticalGuidance_HighSpeedNeedsMoreThrust(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingMid)
	const altitude = 1500.0

	slow := primeRequiredThrust(b, navAt(altitude, 100))
	fast := primeRequiredThrust(b, navAt(altitude, 400))

	if !(fast > slow) {
		t.Errorf("быстрое снижение (Vz=-400) на той же высоте не требует больше тяги, "+
			"чем медленное (Vz=-100): fast=%.0f Н, slow=%.0f Н", fast, slow)
	}
}

// TestVerticalGuidance_LowSpeedNeedsLessThrust — обратная сторона того же
// свойства: чем ближе скорость к цели, тем меньше требуемая тяга (п.18
// "Low downward speed").
func TestVerticalGuidance_LowSpeedNeedsLessThrust(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingMid)
	const altitude = 500.0

	fast := primeRequiredThrust(b, navAt(altitude, 200))
	slow := primeRequiredThrust(b, navAt(altitude, 20))

	if !(slow < fast) {
		t.Errorf("медленное снижение (Vz=-20) требует не меньше тяги, чем быстрое "+
			"(Vz=-200) на той же высоте: slow=%.0f Н, fast=%.0f Н", slow, fast)
	}
}

// TestVerticalGuidance_NearTargetNoOvershoot — п.18 "Near target": вблизи
// целевой скорости касания требуемая тяга обязана оставаться в разумных
// пределах (не улетать в бесконечность/отрицательное значение и не
// провоцировать скачок ускорения вверх — reboost), а не только быть
// формально конечным числом.
func TestVerticalGuidance_NearTargetNoOvershoot(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingLow)
	const altitude = 50.0

	required := primeRequiredThrust(b, navAt(altitude, landingTouchdownSpeed))

	mass := b.dryMass() + b.state.FuelMass
	weight := mass * 9.8
	if required > 2*weight {
		t.Errorf("на целевой скорости требуемая тяга %.0f Н больше чем 2×вес (%.0f Н) — "+
			"профиль просит гораздо больше зависания там, где цель уже достигнута", required, weight)
	}
	if math.IsNaN(required) || math.IsInf(required, 0) || required < 0 {
		t.Fatalf("требуемая тяга не является конечным неотрицательным числом: %v", required)
	}
}

// TestVerticalGuidance_HeavierNeedsMoreThrust — п.18 "Different mass": более
// тяжёлый бустер на ТОЙ ЖЕ высоте и скорости обязан требовать БОЛЬШУЮ тягу.
//
// Состояние взято ВЫШЕ тормозного профиля (на 1200 м снижение 400 м/с
// требует около четырёх километров тормозного пути) — там, где тяга
// действительно нужна. Прежде здесь стояло 150 м/с, и проверка проходила
// по недоразумению: на 1200 м погасить 150 м/с можно на шестистах метрах,
// тяга не нужна ни при какой массе, — а закон всё равно возвращал
// пропорциональную массе величину, потому что имел пол в размере веса
// (want = g). Пол снят как запрещавший снижение (см. landingRequiredThrust),
// и в том состоянии закон честно отвечает нулём для обеих масс. Свойство
// «тяжелее — больше» от этого никуда не делось, но проверять его надо там,
// где тяга запрошена.
func TestVerticalGuidance_HeavierNeedsMoreThrust(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingMid)
	const altitude, descent = 1200.0, 400.0

	b.state.FuelMass = 50000
	light := primeRequiredThrust(b, navAt(altitude, descent))

	b.state.FuelMass = 200000
	heavy := primeRequiredThrust(b, navAt(altitude, descent))

	if !(heavy > light) {
		t.Errorf("более тяжёлый бустер (топливо 200 т) не требует больше тяги, чем лёгкий "+
			"(50 т) на той же высоте и скорости: heavy=%.0f Н, light=%.0f Н", heavy, light)
	}
}

// TestVerticalGuidance_StoppingDistanceAnalytic — п.5 Stage 4.3: чистая
// кинематическая проверка landingStoppingDistance на нескольких вручную
// посчитанных состояниях, без построения Booster/NavState.
func TestVerticalGuidance_StoppingDistanceAnalytic(t *testing.T) {
	cases := []struct {
		name       string
		vz, aDecel float64
		want       float64
		wantInf    bool
	}{
		// (100²-3²)/(2·10) = 9991/20 = 499.55
		{"100 м/с, замедление 10 м/с²", 100, 10, 499.55, false},
		// уже на целевой скорости — тормозной путь нулевой.
		{"на целевой скорости", landingTouchdownSpeed, 10, 0, false},
		// ниже целевой скорости — тоже нулевой (нечего гасить).
		{"ниже целевой скорости", 1, 10, 0, false},
		// группа не может тормозить (замедление <= 0) — явно +Inf, не NaN
		// и не выдуманное конечное число (см. п.18 "Impossible state").
		{"нулевое замедление", 50, 0, 0, true},
		{"отрицательное замедление", 50, -5, 0, true},
	}
	for _, c := range cases {
		got := landingStoppingDistance(c.vz, c.aDecel)
		if c.wantInf {
			if !math.IsInf(got, 1) {
				t.Errorf("%s: landingStoppingDistance(%.1f,%.1f)=%v, ожидалось +Inf", c.name, c.vz, c.aDecel, got)
			}
			continue
		}
		if math.Abs(got-c.want) > 0.5 {
			t.Errorf("%s: landingStoppingDistance(%.1f,%.1f)=%.2f, ожидалось %.2f", c.name, c.vz, c.aDecel, got, c.want)
		}
	}
}

// TestVerticalGuidance_ImpossibleStateIsNegativeMargin — п.18 "Impossible
// state": если даже Tmax текущей группы не обеспечивает stopping
// feasibility, landingStoppingMargin обязан явно и конечно уйти в минус —
// не в NaN и не в положительное число, которое замаскировало бы проблему.
func TestVerticalGuidance_ImpossibleStateIsNegativeMargin(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingLow)
	// Огромная скорость снижения у самой земли — три камеры на максимуме
	// газа заведомо не успеют погасить её на оставшихся метрах.
	nav := navAt(5.0, 500.0)

	_, aMax := b.groupAccelEnvelope(nav, b.propulsion.Commissioned)
	if math.IsNaN(aMax) || math.IsInf(aMax, 0) {
		t.Fatalf("groupAccelEnvelope вернул не конечное aMax: %v", aMax)
	}

	margin := b.landingStoppingMargin(nav, aMax)
	if math.IsNaN(margin) {
		t.Fatalf("landingStoppingMargin вернул NaN вместо явного отрицательного запаса")
	}
	if margin >= 0 {
		t.Errorf("заведомо недостижимое состояние (Vz=-500 м/с, высота 5 м) даёт "+
			"неотрицательный запас %.0f м — должно быть явно отрицательным", margin)
	}
}

// TestVerticalGuidance_GroupAccelEnvelopeOrdering — меньше камер не может
// давать больше максимального ускорения при той же массе/высоте/
// сопротивлении: aMax(13) > aMax(5) > aMax(3) (п.8 Stage 4.3).
func TestVerticalGuidance_GroupAccelEnvelopeOrdering(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)
	nav := navAt(1500.0, 150.0)

	_, aMax13 := b.groupAccelEnvelope(nav, boosterLandingHigh)
	_, aMax5 := b.groupAccelEnvelope(nav, boosterLandingMid)
	_, aMax3 := b.groupAccelEnvelope(nav, boosterLandingLow)

	if !(aMax13 > aMax5 && aMax5 > aMax3) {
		t.Errorf("acceleration envelope не убывает с числом камер: aMax(13)=%.2f aMax(5)=%.2f aMax(3)=%.2f",
			aMax13, aMax5, aMax3)
	}
}

// TestVerticalGuidance_ExcessGroupTriggersDownselect — защита от вечного
// висения.
//
// Прежде здесь стояла проверка «идя медленнее профиля, наведение обязано
// просить МЕНЬШЕ веса»: тогда закон гнался за опорным профилем, и
// оказаться ниже него означало «есть запас, можно падать быстрее». Закон
// профиля больше не имеет — он считает потребное замедление прямо
// (landingRequiredThrust), и на двух километрах при снижении в тридцать
// м/с честно просит вес плюс две десятых процента: этого ровно хватает,
// чтобы прийти к земле со скоростью касания. Требовать от него меньшего
// значило бы требовать не тормозить там, где тормозить и не нужно.
//
// Опасность, от которой та проверка защищала, никуда не делась: если
// потребная тяга ниже того, что группа камер может выдать на минимальном
// газе, ступень не снижается, а зависает — и на избыточной группе висение
// переходит в набор высоты. Защита теперь в другом месте: такое состояние
// обязано быть признаком даунселекта. Это и проверяется.
func TestVerticalGuidance_ExcessGroupTriggersDownselect(t *testing.T) {
	b := landingBoosterOnGroup(t, 1, boosterLandingHigh)

	// Высоко и медленно: тормозить почти нечего, потребная тяга близка к
	// весу и заведомо ниже минимума тринадцати камер.
	nav := navAt(2000, 30)
	required := primeRequiredThrust(b, nav)

	perEngine := vehicle.ThrustAtAltitude(b.Config.FirstStage,
		physics.Atmosphere(nav.Altitude).Pressure)
	minCurrent := perEngine * float64(boosterLandingHigh) * b.minThrottle()
	maxNext := perEngine * float64(boosterLandingMid)

	if !(required < minCurrent) {
		t.Fatalf("потребная тяга %.2f МН не ниже минимума группы из %d камер (%.2f МН) — "+
			"проверка не о том состоянии", required/1e6, boosterLandingHigh, minCurrent/1e6)
	}
	if !landingGroupShouldDownselect(required, minCurrent, maxNext) {
		t.Fatalf("группа из %d камер избыточна (нужно %.2f МН, минимум группы %.2f МН), "+
			"а даунселект не назначен — ступень будет висеть и уйдёт вверх",
			boosterLandingHigh, required/1e6, minCurrent/1e6)
	}
	if math.IsNaN(required) || required < 0 {
		t.Fatalf("требуемая тяга не является конечной неотрицательной: %v", required)
	}
}
