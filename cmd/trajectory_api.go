package main

import (
	"net/http"
	"sort"
	"strconv"
	"sync"

	"github.com/gorilla/mux"

	"rocketTelemetrySim/simulator"
	"rocketTelemetrySim/simulator/vehicle"
)

// -----------------------------------------------------------------------------
// Выдача записанных траекторий странице «Траектория».
//
// Запись — это отдельный прогон модели от старта до конца (см.
// simulator.RecordTrajectory), а не выборка из живой симуляции пульта. Так
// сделано намеренно: страница показывает ПРОГОН ЦЕЛИКОМ, включая то, что
// уже произошло, и должна открываться в любой момент, а не только пока
// вкладка была открыта с самого старта.
//
// Прогон стоит несколько секунд машинного времени, поэтому результат
// запоминается: одно и то же зерно даёт одну и ту же запись (это проверено
// тестом), значит считать её второй раз незачем.
// -----------------------------------------------------------------------------

// trajectoryCacheLimit — сколько записей держать в памяти.
//
// Запись одного прогона — порядка мегабайта. Полтора десятка записей это
// пара десятков мегабайт: незаметно на фоне самой модели и с запасом
// покрывает перебор зёрен, ради которого выбор прогона и сделан.
const trajectoryCacheLimit = 16

type trajectoryKey struct {
	seed    int64
	profile string
	mission string
}

type trajectoryCacheEntry struct {
	run  *simulator.TrajectoryRun
	used int64
}

var (
	trajectoryMu    sync.Mutex
	trajectoryCache = map[trajectoryKey]*trajectoryCacheEntry{}
	trajectoryClock int64

	// trajectoryBuild сериализует сами прогоны. Без него десяток
	// одновременных запросов (оператор дёргает список зёрен подряд)
	// запустил бы десяток полных симуляций разом и занял бы все ядра
	// машины, на которой в это время идёт настоящий прогон пульта.
	trajectoryBuild sync.Mutex
)

// registerTrajectoryRoutes добавляет маршруты страницы «Траектория».
func registerTrajectoryRoutes(r *mux.Router) {
	r.HandleFunc("/api/trajectory", trajectoryHandler).
		Methods(http.MethodGet, http.MethodOptions)
	r.HandleFunc("/api/trajectory/runs", trajectoryRunsHandler).
		Methods(http.MethodGet, http.MethodOptions)
	r.HandleFunc("/api/trajectory/live", trajectoryLiveHandler).
		Methods(http.MethodGet, http.MethodOptions)
}

// trajectoryRunsHandler отдаёт список того, что вообще можно посмотреть:
// доступные носители, задания и уже посчитанные (то есть мгновенно
// доступные) зёрна.
func trajectoryRunsHandler(w http.ResponseWriter, r *http.Request) {
	profiles := make([]map[string]string, 0)
	for _, p := range vehicle.Profiles() {
		profiles = append(profiles, map[string]string{"id": p.ID, "title": p.Title})
	}
	missions := make([]map[string]string, 0)
	for _, m := range vehicle.Missions() {
		missions = append(missions, map[string]string{"id": m.ID, "title": m.Title})
	}

	trajectoryMu.Lock()
	cached := make([]map[string]any, 0, len(trajectoryCache))
	for k := range trajectoryCache {
		cached = append(cached, map[string]any{
			"seed": k.seed, "profile": k.profile, "mission": k.mission,
		})
	}
	trajectoryMu.Unlock()
	sort.Slice(cached, func(i, j int) bool {
		return cached[i]["seed"].(int64) < cached[j]["seed"].(int64)
	})

	writeJSON(w, map[string]any{
		"profiles": profiles,
		"missions": missions,
		"cached":   cached,
		"current": map[string]any{
			"profile": currentProfile().ID,
			"mission": currentMission().ID,
		},
	})
}

// trajectoryLiveHandler отдаёт траекторию ИДУЩЕГО прогона — того самого, за
// которым оператор смотрит на пульте.
//
// Это не то же самое, что запись по зерну. Зерно задаёт ветер, разбросы и
// шум приборов, но не действия оператора: стоило один раз тронуть тягу или
// створки — и повтор по зерну разойдётся с тем, что было. Живая трасса
// пишется по ходу полёта и содержит всё, что с прогоном на самом деле
// происходило (см. simulator/trajectory_live.go).
func trajectoryLiveHandler(w http.ResponseWriter, r *http.Request) {
	sim, ok := requireSimulation(w)
	if !ok {
		return
	}

	run := sim.LiveTrajectory()
	run.Profile = currentProfile().ID
	run.Mission = currentMission().ID

	writeJSON(w, run)
}

// trajectoryHandler считает (или достаёт из памяти) запись одного прогона.
func trajectoryHandler(w http.ResponseWriter, r *http.Request) {
	q := r.URL.Query()

	seed := int64(1)
	if v := q.Get("seed"); v != "" {
		parsed, err := strconv.ParseInt(v, 10, 64)
		if err != nil {
			http.Error(w, "seed: "+err.Error(), http.StatusBadRequest)
			return
		}
		seed = parsed
	}

	profileID := q.Get("profile")
	if profileID == "" {
		profileID = currentProfile().ID
	}
	profile, ok := vehicle.ProfileByID(profileID)
	if !ok {
		http.Error(w, "неизвестный носитель: "+profileID, http.StatusBadRequest)
		return
	}

	missionID := q.Get("mission")
	if missionID == "" {
		missionID = currentMission().ID
	}
	mission, ok := vehicle.MissionByID(missionID)
	if !ok {
		http.Error(w, "неизвестное задание: "+missionID, http.StatusBadRequest)
		return
	}

	key := trajectoryKey{seed: seed, profile: profile.ID, mission: mission.ID}

	if run := trajectoryFromCache(key); run != nil {
		writeJSON(w, run)
		return
	}

	// Прогоны идут по одному (см. trajectoryBuild). После ожидания запись
	// могла появиться — проверяем ещё раз, чтобы не считать то же самое
	// дважды подряд.
	trajectoryBuild.Lock()
	defer trajectoryBuild.Unlock()
	if run := trajectoryFromCache(key); run != nil {
		writeJSON(w, run)
		return
	}

	opt := simulator.DefaultTrajectoryOptions()
	if v := q.Get("duration"); v != "" {
		d, err := strconv.ParseFloat(v, 64)
		if err != nil || d <= 0 || d > 6000 {
			http.Error(w, "duration: ожидается число секунд от 0 до 6000", http.StatusBadRequest)
			return
		}
		opt.MaxDuration = d
	}

	run := simulator.RecordTrajectory(mission.Apply(profile.Config()), seed, opt)
	run.Profile = profile.ID
	run.Mission = mission.ID

	trajectoryStore(key, run)
	writeJSON(w, run)
}

func trajectoryFromCache(key trajectoryKey) *simulator.TrajectoryRun {
	trajectoryMu.Lock()
	defer trajectoryMu.Unlock()
	entry, ok := trajectoryCache[key]
	if !ok {
		return nil
	}
	trajectoryClock++
	entry.used = trajectoryClock
	return entry.run
}

// trajectoryStore кладёт запись в память, вытесняя самую давно не
// востребованную, когда их накопилось больше предела.
func trajectoryStore(key trajectoryKey, run *simulator.TrajectoryRun) {
	trajectoryMu.Lock()
	defer trajectoryMu.Unlock()

	trajectoryClock++
	trajectoryCache[key] = &trajectoryCacheEntry{run: run, used: trajectoryClock}

	for len(trajectoryCache) > trajectoryCacheLimit {
		var oldestKey trajectoryKey
		var oldest int64
		first := true
		for k, e := range trajectoryCache {
			if first || e.used < oldest {
				oldestKey, oldest, first = k, e.used, false
			}
		}
		delete(trajectoryCache, oldestKey)
	}
}
