package main

import (
	"log"
	"sync"
	"time"

	"rocketTelemetrySim/simulator"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Жизненный цикл прогона.
//
// Каждый запуск создаёт новый экземпляр симуляции, поэтому состояние
// предыдущего полёта не может «просочиться» в следующий. Настройки оператора —
// скорость течения времени и номер прогона — наоборот, переносятся: сбрасывать
// их при каждом старте значило бы терять контекст работы.
// -----------------------------------------------------------------------------

// selectedProfile — профиль носителя для следующего прогона.
//
// Менять носитель на лету нельзя: от него зависят массы, геометрия, азимут
// пуска и состав двигательной установки. Выбор запоминается и применяется
// при следующем запуске.
// Заправка тоже задаётся до пуска и хранится долей от штатной: в долях она
// переживает смену носителя, а в тоннах пришлось бы пересчитывать её при
// каждом переключении профиля.
var (
	profileMu sync.RWMutex
	profileID = vehicle.DefaultProfile().ID
	missionID = vehicle.DefaultMission().ID
	fuelLoad  = [2]float64{1, 1}
)

// SetProfile запоминает профиль носителя.
func setProfile(id string) bool {
	if _, ok := vehicle.ProfileByID(id); !ok {
		return false
	}
	profileMu.Lock()
	profileID = id

	// Новый носитель заправляется штатно: доля от прежнего к его бакам
	// отношения не имеет.
	fuelLoad = [2]float64{1, 1}
	profileMu.Unlock()
	return true
}

// setMission запоминает задание на полёт.
//
// Задание не меняет ни грамма железа: оно говорит наведению, куда выводить.
// Поэтому менять его можно вместе с носителем — любой из них полетит и по
// трансатмосферной траектории, и на замкнутую орбиту.
func setMission(id string) bool {
	if _, ok := vehicle.MissionByID(id); !ok {
		return false
	}
	profileMu.Lock()
	missionID = id
	profileMu.Unlock()
	return true
}

// currentMission возвращает выбранное задание.
func currentMission() vehicle.Mission {
	profileMu.RLock()
	id := missionID
	profileMu.RUnlock()

	if m, ok := vehicle.MissionByID(id); ok {
		return m
	}
	return vehicle.DefaultMission()
}

// setFuelLoad запоминает заправку ступени долей от штатной.
func setFuelLoad(stage int, fraction float64) bool {
	if stage < 1 || stage > 2 || fraction < 0 || fraction > 1.2 {
		return false
	}
	profileMu.Lock()
	fuelLoad[stage-1] = fraction
	profileMu.Unlock()
	return true
}

// fuelLoads возвращает заданную заправку обеих ступеней.
func fuelLoads() [2]float64 {
	profileMu.RLock()
	defer profileMu.RUnlock()
	return fuelLoad
}

// currentProfile возвращает выбранный профиль.
func currentProfile() vehicle.Profile {
	profileMu.RLock()
	id := profileID
	profileMu.RUnlock()

	if p, ok := vehicle.ProfileByID(id); ok {
		return p
	}
	return vehicle.DefaultProfile()
}

// currentConfig возвращает конфигурацию носителя для следующего прогона.
//
// Заправка входит в неё так же, как входила бы в паспорт изделия: модель
// получает уже заправленную ракету и дальше считает всё сама — стартовую массу,
// тяговооружённость, момент выработки баков и дальность. Отдельного «уровня
// топлива» в обход физики не существует.
func currentConfig() vehicle.Config {
	cfg := currentMission().Apply(currentProfile().Config())
	load := fuelLoads()

	cfg.FirstStage.FuelMass *= load[0]
	cfg.SecondStage.FuelMass *= load[1]

	// Резерв на возврат не может превышать оставшийся запас: при неполной
	// заправке ступень обязана отдать под работу двигателей хоть что-то.
	if r := cfg.FirstStage.FuelReserve; r > cfg.FirstStage.FuelMass*0.9 {
		cfg.FirstStage.FuelReserve = cfg.FirstStage.FuelMass * 0.9
	}
	return cfg
}

// simulationLoop ждёт команду старта и запускает симуляцию.
func simulationLoop() {
	for {
		<-simulationStartChan

		// Настройки предыдущего прогона переносятся на новый.
		scale, run := 1.0, 0
		if old := simulator.GetCurrentSimulation(); old != nil {
			scale, run = old.Scale(), old.RunNumber()
		}

		profile, cfg := currentProfile(), currentConfig()
		sim := simulator.NewSimulationWithConfig(cfg, simulator.DefaultTimeConfig())
		sim.SetScale(scale)
		sim.SetRunNumber(run + 1)

		simulator.SetCurrentSimulation(sim)
		log.Printf("Starting simulation: %s, задание «%s», заправка %.0f/%.0f т, "+
			"прогон №%d, скорость %g×",
			profile.Title, currentMission().Title, cfg.FirstStage.FuelMass/1000,
			cfg.SecondStage.FuelMass/1000, run+1, scale)

		sim.Run()

		log.Println("Simulation ended, waiting for next start command")

		// Значение, положенное в канал во время прогона, здесь намеренно
		// не выбрасывается: раньше именно это молча съедало команду оператора,
		// и повторный старт «не срабатывал». Очередь разберёт следующая
		// итерация цикла.
	}
}

// stopRunningSimulation останавливает текущий прогон и дожидается выхода
// из цикла.
//
// Кнопка «Start» означает «начать заново», а не «поставить в очередь»:
// пока предыдущий прогон не завершился, новый начаться не может, и команда
// оператора просто терялась бы. Именно так и происходило после остановки
// турбонасоса: ракета ещё минуту падала, старт уходил в буфер канала,
// а по завершении прогона выбрасывался.
func stopRunningSimulation() {
	sim := simulator.GetCurrentSimulation()
	if sim == nil || !sim.Started() || sim.Finished() {
		return
	}

	sim.Stop()
	if !sim.Wait(3 * time.Second) {
		log.Println("Предыдущий прогон не завершился за три секунды")
	}
}
