package simulator

import (
	"math"
	"testing"

	"rocketTelemetrySim/simulator/physics"
)

// -----------------------------------------------------------------------------
// Stage 4.5, п.5,10,11: проверка знаков/frame conventions и базовые
// регрессии на симметричной карте камер, без production allocator'а —
// вручную заданные позиции/gimbal, r×F посчитан кодом (physics.Vec3.Cross),
// сверен с ожиданием на бумаге.
// -----------------------------------------------------------------------------

// TestEngineDirectionBody_SignConventions — п.5: pitch отклоняет тягу к Down
// (Z+), yaw — к -Right (Y-), при нулевом отклонении тяга строго по Forward
// (X+) — та же конвенция, что VehicleAttitude.ThrustDirection.
func TestEngineDirectionBody_SignConventions(t *testing.T) {
	d0 := engineDirectionBody(0, 0)
	if math.Abs(d0.X-1) > 1e-9 || math.Abs(d0.Y) > 1e-9 || math.Abs(d0.Z) > 1e-9 {
		t.Errorf("нулевое отклонение: направление = %+v, ожидалось {1,0,0}", d0)
	}

	const angle = 5 * physics.DegToRad
	dp := engineDirectionBody(angle, 0)
	if dp.Z <= 0 {
		t.Errorf("положительный pitch не отклонил тягу к Down (Z+): %+v", dp)
	}
	if math.Abs(dp.Y) > 1e-9 {
		t.Errorf("чистый pitch дал ненулевую Y-составляющую: %+v", dp)
	}

	dy := engineDirectionBody(0, angle)
	if dy.Y >= 0 {
		t.Errorf("положительный yaw не отклонил тягу к -Right (Y-): %+v", dy)
	}
	if math.Abs(dy.Z) > 1e-9 {
		t.Errorf("чистый yaw дал ненулевую Z-составляющую: %+v", dy)
	}
}

// TestEnginePosition_TorqueSign — п.5: вручную посчитанный r×F для одного
// известного двигателя (позиция справа от оси, RadialX>0) с чистым
// downward-отклонением тяги, сверенный с ручным расчётом.
func TestEnginePosition_TorqueSign(t *testing.T) {
	const arm, radial, thrust = 30.0, 2.0, 1e6
	e := EngineActuator{RadialX: radial, RadialY: 0, Gimbal: true, Thrust: thrust}
	r := enginePosition(e, arm)
	wantR := physics.Vec3{X: -arm, Y: radial, Z: 0}
	if r != wantR {
		t.Fatalf("enginePosition = %+v, ожидалось %+v", r, wantR)
	}

	// Тяга отклонена чисто по pitch (к Down): F = T·{0,0,1} (в пределе
	// малых углов engineDirectionBody(δ,0)→{~1,0,~δ}, здесь берём точный
	// download-вектор для однозначности).
	f := physics.Vec3{Z: thrust}
	torque := r.Cross(f)
	// r×F = {-arm,radial,0} × {0,0,thrust}
	//     = { radial*thrust - 0, 0 - (-arm*thrust), 0 }
	//     = { radial*thrust, arm*thrust, 0 }
	want := physics.Vec3{X: radial * thrust, Y: arm * thrust, Z: 0}
	if math.Abs(torque.X-want.X) > 1 || math.Abs(torque.Y-want.Y) > 1 || math.Abs(torque.Z-want.Z) > 1 {
		t.Errorf("r×F = %+v, ручной расчёт даёт %+v", torque, want)
	}
	// Знак: положительный Y-момент (pitch) от тяги, отклонённой к Down —
	// та же полярность, что и агрегатная модель (side*sin(GimbalPitch),
	// см. attitude_dynamics.go integrate) — камера, отклонённая вниз,
	// поднимает нос (положительный pitch torque).
	if torque.Y <= 0 {
		t.Errorf("отклонение тяги к Down не дало положительный pitch-момент: %+v", torque)
	}
}

// symmetricRing13 строит 13 маневренных камер по кольцевой раскладке,
// зеркально симметричной по X и Y — центр (нулевой радиус) не годится
// (не создаёт torque ни при каком отклонении), берём просто равномерное
// кольцо из 13, без центра, для чистоты симметричного теста.
func symmetricRing13(thrust float64) []EngineActuator {
	const n = 13
	const radius = 4.0
	out := make([]EngineActuator, n)
	for i := 0; i < n; i++ {
		angle := 2 * math.Pi * float64(i) / float64(n)
		out[i] = EngineActuator{
			RadialX: radius * math.Cos(angle),
			RadialY: radius * math.Sin(angle),
			Gimbal:  true,
			Thrust:  thrust,
		}
	}
	return out
}

// TestEngineForceTorque_ZeroGimbalSymmetric — п.10 Stage 4.5: 13 камер,
// одинаковый throttle, нулевой gimbal, симметричная карта → лёгкое (боковая
// сила и все три момента около нуля), суммарная осевая тяга совпадает с
// T_i×n.
func TestEngineForceTorque_ZeroGimbalSymmetric(t *testing.T) {
	const thrust, arm = 2e6, 30.0
	engines := symmetricRing13(thrust)
	gimbal := make([]EngineGimbalState, len(engines))

	force, torque := engineForceTorque(engines, gimbal, arm)

	wantAxial := thrust * float64(len(engines))
	if math.Abs(force.X-wantAxial) > 1 {
		t.Errorf("суммарная осевая тяга = %.0f, ожидалось %.0f", force.X, wantAxial)
	}
	if math.Abs(force.Y) > 1e-6 || math.Abs(force.Z) > 1e-6 {
		t.Errorf("нулевой gimbal на симметричной карте дал боковую силу: %+v", force)
	}
	if torque.Norm() > 1e-3 {
		t.Errorf("нулевой gimbal на симметричной карте дал ненулевой момент: %+v", torque)
	}
}

// TestEngineForceTorque_CommonGimbal — п.11 Stage 4.5: все 13 камер
// отклонены ОДИНАКОВО по одной оси (pitch) — общий (common-mode) gimbal
// на симметричной группе в основном меняет НАПРАВЛЕНИЕ суммарной тяги, а
// НЕ обязательно создаёт большой attitude torque (момент возникает от
// плеча armAxial, но лежит в плоскости pitch — по определению не
// разбрасывается по roll/yaw на симметричной карте).
func TestEngineForceTorque_CommonGimbal(t *testing.T) {
	const thrust, arm, angle = 2e6, 30.0, 5 * physics.DegToRad
	engines := symmetricRing13(thrust)
	gimbal := make([]EngineGimbalState, len(engines))
	for i := range gimbal {
		gimbal[i] = EngineGimbalState{Pitch: angle}
	}

	force, torque := engineForceTorque(engines, gimbal, arm)

	wantZ := thrust * float64(len(engines)) * math.Sin(angle)
	if math.Abs(force.Z-wantZ) > wantZ*0.01+1 {
		t.Errorf("боковая (Z) составляющая при общем pitch-gimbal = %.0f, ожидалось %.0f", force.Z, wantZ)
	}
	if math.Abs(force.Y) > 1e-6 {
		t.Errorf("чистый common pitch дал ненулевую Y-составляющую силы: %.3f", force.Y)
	}
	// Момент по крену/рысканию у симметричной группы при чистом common
	// pitch должен остаться пренебрежимо мал — вся асимметрия только в
	// pitch-плоскости.
	if math.Abs(torque.X) > math.Abs(torque.Y)*0.01+1 {
		t.Errorf("common pitch на симметричной карте дал заметный roll-момент: torque=%+v", torque)
	}
	if math.Abs(torque.Z) > math.Abs(torque.Y)*0.01+1 {
		t.Errorf("common pitch на симметричной карте дал заметный yaw-момент: torque=%+v", torque)
	}
	if torque.Y <= 0 {
		t.Errorf("common pitch-gimbal (тяга к Down) не дал положительный pitch-момент: %+v", torque)
	}
}

// -----------------------------------------------------------------------------
// allocateEngineGimbal
// -----------------------------------------------------------------------------

// TestAllocateEngineGimbal_PurePitchTorque — чистая цель по pitch-моменту
// (без силовой составляющей) должна быть достижима близко к запрошенному
// значению, если оно в пределах physical authority (не на упоре).
func TestAllocateEngineGimbal_PurePitchTorque(t *testing.T) {
	const thrust, arm, maxGimbal = 2e6, 30.0, 5 * physics.DegToRad
	engines := symmetricRing13(thrust)
	forceScale := thrust * float64(len(engines))
	torqueScale := forceScale * arm * math.Sin(maxGimbal)

	targetTorque := physics.Vec3{Y: torqueScale * 0.3} // 30% authority — заведомо достижимо
	gimbal := allocateEngineGimbal(engines, arm, physics.Vec3{}, targetTorque, forceScale, torqueScale, maxGimbal)

	achievedF, achievedM := engineForceTorque(engines, gimbal, arm)
	if math.Abs(achievedM.Y-targetTorque.Y) > targetTorque.Y*0.1 {
		t.Errorf("achieved pitch-момент = %.3e, запрошено %.3e (>10%% расхождение)", achievedM.Y, targetTorque.Y)
	}
	if math.Abs(achievedM.X) > math.Abs(achievedM.Y)*0.05 {
		t.Errorf("чистая pitch-цель дала заметный roll-момент: achieved=%+v", achievedM)
	}
	if math.Abs(achievedM.Z) > math.Abs(achievedM.Y)*0.05 {
		t.Errorf("чистая pitch-цель дала заметный yaw-момент: achieved=%+v", achievedM)
	}
	// Осевая тяга не должна заметно проседать при умеренном запросе.
	wantAxial := thrust * float64(len(engines))
	if achievedF.X < wantAxial*0.99 {
		t.Errorf("осевая тяга просела при умеренном torque-запросе: %.0f от %.0f", achievedF.X, wantAxial)
	}
}

// TestAllocateEngineGimbal_ForceAndTorqueTogether — п.14 Stage 4.5: цель
// одновременно по силе (боковая Z, т.е. "хотим наклонить суммарный вектор
// тяги") и по моменту (pitch) — оба должны получить какой-то отклик,
// allocator не должен полностью игнорировать один ради другого.
func TestAllocateEngineGimbal_ForceAndTorqueTogether(t *testing.T) {
	const thrust, arm, maxGimbal = 2e6, 30.0, 5 * physics.DegToRad
	engines := symmetricRing13(thrust)
	forceScale := thrust * float64(len(engines))
	torqueScale := forceScale * arm * math.Sin(maxGimbal)

	targetForce := physics.Vec3{Z: forceScale * 0.2}
	targetTorque := physics.Vec3{Y: torqueScale * 0.2}
	gimbal := allocateEngineGimbal(engines, arm, targetForce, targetTorque, forceScale, torqueScale, maxGimbal)

	achievedF, achievedM := engineForceTorque(engines, gimbal, arm)
	if achievedF.Z <= 0 {
		t.Errorf("силовая цель полностью проигнорирована: achievedF.Z=%.0f", achievedF.Z)
	}
	if achievedM.Y <= 0 {
		t.Errorf("моментная цель полностью проигнорирована: achievedM.Y=%.3e", achievedM.Y)
	}
}

// TestAllocateEngineGimbal_RespectsBounds — насыщение: заведомо
// недостижимая (за пределом физики) цель не должна давать отклонения за
// maxGimbal ни у одной камеры.
func TestAllocateEngineGimbal_RespectsBounds(t *testing.T) {
	const thrust, arm, maxGimbal = 2e6, 30.0, 5 * physics.DegToRad
	engines := symmetricRing13(thrust)

	huge := physics.Vec3{Y: 1e12}
	gimbal := allocateEngineGimbal(engines, arm, physics.Vec3{}, huge, 1, 1, maxGimbal)

	for i, g := range gimbal {
		if math.Abs(g.Pitch) > maxGimbal+1e-9 || math.Abs(g.Yaw) > maxGimbal+1e-9 {
			t.Errorf("engine[%d]: gimbal=%+v превышает предел %.4f рад", i, g, maxGimbal)
		}
	}
}

// TestAllocateEngineGimbal_StoppedEnginesGetNoCommand — п.20 Stage 4.5:
// двигатель с Thrust=0 (погашен) не должен получать ненулевую команду —
// allocator строит столбцы только для работающих камер.
func TestAllocateEngineGimbal_StoppedEnginesGetNoCommand(t *testing.T) {
	const thrust, arm, maxGimbal = 2e6, 30.0, 5 * physics.DegToRad
	engines := symmetricRing13(thrust)
	engines[3].Thrust = 0 // погашена

	gimbal := allocateEngineGimbal(engines, arm, physics.Vec3{}, physics.Vec3{Y: 1e6}, thrust*13, thrust*13*arm*math.Sin(maxGimbal), maxGimbal)
	if gimbal[3].Pitch != 0 || gimbal[3].Yaw != 0 {
		t.Errorf("погашенный двигатель[3] получил команду: %+v", gimbal[3])
	}
}

// TestAllocateEngineGimbal_DifferentialRollAuthorityIsSmall — п.4 Stage 4.5:
// если реальная per-engine TVC почти не даёт roll authority, это важный
// физический результат, а не повод возвращать magic torque. Здесь
// численно сравниваем максимально достижимый (в пределах allocator'а)
// roll-момент с pitch/yaw — ожидаем на порядки меньше (плечо
// EngineRingRadius на порядок меньше armAxial, а вклад в roll идёт только
// от differential, а не common, составляющей).
func TestAllocateEngineGimbal_DifferentialRollAuthorityIsSmall(t *testing.T) {
	const thrust, arm, maxGimbal = 2e6, 30.0, 5 * physics.DegToRad
	engines := symmetricRing13(thrust)
	forceScale := thrust * float64(len(engines))
	torqueScale := forceScale * arm * math.Sin(maxGimbal)

	rollTarget := physics.Vec3{X: torqueScale * 10} // заведомо больше, чем реально достижимо — прощупываем потолок
	gimbal := allocateEngineGimbal(engines, arm, physics.Vec3{}, rollTarget, forceScale, torqueScale, maxGimbal)
	_, achievedRoll := engineForceTorque(engines, gimbal, arm)

	pitchTarget := physics.Vec3{Y: torqueScale * 10}
	gimbalP := allocateEngineGimbal(engines, arm, physics.Vec3{}, pitchTarget, forceScale, torqueScale, maxGimbal)
	_, achievedPitch := engineForceTorque(engines, gimbalP, arm)

	t.Logf("достижимый roll-момент (насыщение) = %.3e Н·м, pitch-момент (насыщение) = %.3e Н·м, отношение = %.3f",
		achievedRoll.X, achievedPitch.Y, math.Abs(achievedRoll.X)/math.Abs(achievedPitch.Y))

	if math.Abs(achievedRoll.X) >= math.Abs(achievedPitch.Y) {
		t.Errorf("roll authority (%.3e) не меньше pitch authority (%.3e) — неожиданно для чистого gimbal-based TVC",
			achievedRoll.X, achievedPitch.Y)
	}
}
