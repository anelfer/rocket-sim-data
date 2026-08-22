package main

import (
	"embed"
	"io/fs"
	"net/http"

	"github.com/gorilla/mux"
)

// -----------------------------------------------------------------------------
// Раздача веб-интерфейса.
//
// Интерфейс вшит в исполняемый файл: контейнеру не нужен ни отдельный том,
// ни сборщик пакетов, ни доступ в сеть. Один бинарник — весь центр управления.
// -----------------------------------------------------------------------------

//go:embed web
var webFS embed.FS

// registerUIRoutes добавляет маршруты веб-интерфейса.
func registerUIRoutes(r *mux.Router) {
	sub, err := fs.Sub(webFS, "web")
	if err != nil {
		panic(err)
	}
	files := http.FileServer(http.FS(sub))

	// Интерфейс раздаётся с корня: адрес пульта совпадает с адресом API,
	// поэтому никаких настроек в браузере не требуется.
	r.PathPrefix("/").Handler(noCache(files))
}

// noCache запрещает кеширование интерфейса.
//
// Пульт правится часто, а обновлённый файл, застрявший в кеше браузера,
// стоит дороже, чем повторная загрузка сотни килобайт.
func noCache(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Cache-Control", "no-store")
		next.ServeHTTP(w, r)
	})
}
