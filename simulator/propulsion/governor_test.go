package propulsion

import (
	"math"
	"testing"
)

// -----------------------------------------------------------------------------
// Регулятор оборотов на показаниях датчика.
//
// Регулятор реального двигателя не может подсмотреть в истинные обороты
// вала — он знает только то, что сказал тахометр. Эти тесты проверяют, что
// переход на показание датчика не сломал ни штатную отработку уставки, ни
// поведение на пропуске связи.
// -----------------------------------------------------------------------------

// Регулятор реагирует на показание датчика так же, как раньше реагировал
// на истинные обороты: при заниженной уставке относительно текущих оборотов
// клапан газогенератора приоткрывается.
func TestGovernorTracksSensedSpeed(t *testing.T) {
	e := engineAt(RaptorConfig(), methaloxInput(), 6)
	tp := e.Turbopump

	// Цель заведомо выше текущих оборотов — ошибка положительна однозначно,
	// независимо от того, на каком именно установившемся режиме встал двигатель.
	target := tp.Speed * 1.1
	measured := Measurement{Value: tp.Speed, Valid: true}

	cmd := tp.GovernorCommand(measured, target, 0.05)
	if cmd <= 0 {
		t.Fatalf("регулятор не потребовал открытия клапана при заниженных оборотах: %.4f", cmd)
	}
}

// На пропуске связи регулятор держит последнее достоверное показание и не
// двигает интеграл — а не подсматривает в истинные обороты вала и не
// пугается нулевого показания.
func TestGovernorHoldsLastValidSpeedThroughDropout(t *testing.T) {
	e := engineAt(RaptorConfig(), methaloxInput(), 6)
	tp := e.Turbopump

	target := tp.Config.DesignSpeed
	lastGood := Measurement{Value: tp.Speed, Valid: true}

	cmdBefore := tp.GovernorCommand(lastGood, target, 0.05)
	integralBefore := tp.governorIntegral

	// Пропуск связи: несколько тактов подряд без достоверного показания —
	// как раз то, что реально задаёт DefaultSpeedSensor (0.2 с на пропуск).
	dropout := Measurement{Valid: false}
	var cmdDuring float64
	for i := 0; i < 20; i++ {
		cmdDuring = tp.GovernorCommand(dropout, target, 0.05)
	}

	if tp.governorIntegral != integralBefore {
		t.Errorf("интеграл сдвинулся на пропуске связи: %.6f -> %.6f",
			integralBefore, tp.governorIntegral)
	}
	if math.Abs(cmdDuring-cmdBefore) > 1e-9 {
		t.Errorf("команда клапану изменилась на пропуске связи без нового "+
			"показания: %.6f -> %.6f", cmdBefore, cmdDuring)
	}

	// Связь восстановилась на новом, отличном от предыдущего, показании —
	// регулятор обязан снова реагировать на реальные обороты, а не
	// оставаться замороженным навсегда.
	recovered := Measurement{Value: tp.Speed * 0.5, Valid: true}
	cmdAfter := tp.GovernorCommand(recovered, target, 0.05)
	if cmdAfter <= cmdDuring {
		t.Errorf("регулятор не отреагировал на восстановленное показание: "+
			"%.6f -> %.6f", cmdDuring, cmdAfter)
	}
}

// Истинное давление в камере и истинные обороты вала публикуются в
// телеметрии наравне с показаниями датчика, а не теряются после того, как
// их место в отчёте займёт зашумлённое значение.
func TestTelemetryPublishesTrueAlongsideSensed(t *testing.T) {
	e := engineAt(RaptorConfig(), methaloxInput(), 6)

	in := methaloxInput()
	var sawDivergence bool
	for i := 0; i < 200; i++ {
		e.Update(0.02, in)
		tel := e.Telemetry(0.02, false)

		if math.IsNaN(tel.ChamberPressureTrue) || tel.ChamberPressureTrue <= 0 {
			t.Fatalf("истинное давление в камере не посчитано: %v", tel.ChamberPressureTrue)
		}
		if tel.ShaftRPMTrue <= 0 {
			t.Fatalf("истинные обороты вала не посчитаны: %v", tel.ShaftRPMTrue)
		}
		if tel.ChamberPressure.Defined() && tel.ChamberPressure.Value() != tel.ChamberPressureTrue {
			sawDivergence = true
		}
	}
	if !sawDivergence {
		t.Error("показание датчика давления камеры ни разу не разошлось с " +
			"истинным значением за 4 секунды — шум датчика не действует")
	}
}
