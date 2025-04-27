package main

import (
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"runtime/pprof"
	"strings"

	_ "github.com/lintang-b-s/navigatorx/docs"
	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/engine/heuristics"
	"github.com/lintang-b-s/navigatorx/pkg/engine/riderdrivermatching"
	"github.com/lintang-b-s/navigatorx/pkg/engine/routingalgorithm"
	"github.com/lintang-b-s/navigatorx/pkg/kv"
	"github.com/lintang-b-s/navigatorx/pkg/server/rest"
	"github.com/lintang-b-s/navigatorx/pkg/server/rest/service"
	bolt "go.etcd.io/bbolt"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	httpSwagger "github.com/swaggo/http-swagger"

	_ "net/http/pprof"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
	"github.com/go-chi/cors"
	mymiddleware "github.com/lintang-b-s/navigatorx/pkg/server/middleware"
)

var (
	listenAddr   = flag.String("listenaddr", ":5000", "server listen address")
	mapFile      = flag.String("f", "solo_jogja.osm.pbf", "openstreeetmap file buat road network graphnya")
	cpuprofile   = flag.String("cpuprofile", "", "write cpu profile to file")
	memprofile   = flag.String("memprofile", "", "write memory profile to this file")
	useRateLimit = flag.Bool("ratelimit", false, "use rate limit")
)

//	@title			navigatorx lintangbs API
//	@version		1.0
//	@description	simple openstreetmap routing engine in go

//	@contact.name	lintang birda saputra
//	@description 	simple openstreetmap routing engine in go. Using Contraction Hierarchies for preprocessing and Bidirectioanl Dijkstra for shortest path query

//	@license.name	GNU Affero General Public License v3.0
//	@license.url	https://www.gnu.org/licenses/gpl-3.0.en.html

// @host		localhost:5000
// @BasePath	/api
// @schemes	http
func main() {
	flag.Parse()

	ch := contractor.NewContractedGraph()
	err := ch.LoadGraph(false)
	if err != nil {
		log.Fatal(err)
	}

	recordMemProfile(memprofile, "load_contracted_graph")

	db, err := bolt.Open("./navigatox.db", 0600, nil)
	if err != nil {
		panic(err)
	}
	defer db.Close()

	err = db.Update(func(tx *bolt.Tx) error {
		_, err := tx.CreateBucketIfNotExists([]byte(kv.H3_BOLTBUCKET))
		return err
	})
	if err != nil {
		panic(err)
	}

	kvDB := kv.NewKVDB(db)
	defer kvDB.Close()

	reg := prometheus.NewRegistry()
	m := rest.NewMetrics(reg)

	r := chi.NewRouter()

	r.Use(middleware.Logger)

	r.Use(rest.PromeHttpMiddleware(m)) // prometheus http middleware
	r.Use(cors.Handler(cors.Options{
		AllowedOrigins:   []string{"https://*", "http://*"},
		AllowedMethods:   []string{"GET", "POST", "PUT", "DELETE", "OPTIONS"},
		AllowedHeaders:   []string{"Accept", "Authorization", "Content-Type", "X-CSRF-Token"},
		ExposedHeaders:   []string{"Link"},
		AllowCredentials: false,
		MaxAge:           300,
	}))

	if *useRateLimit {
		r.Use(mymiddleware.Limit)
	}

	r.Mount("/debug", middleware.Profiler())

	r.Handle("/metrics", promhttp.HandlerFor(reg, promhttp.HandlerOpts{}))

	r.Get("/swagger/*", httpSwagger.Handler(
		httpSwagger.URL("http://localhost:5000/swagger/doc.json"), //The url pointing to API definition
	))

	routingAlgorithm := routingalgorithm.NewRouteAlgorithm(ch)
	hungarian := riderdrivermatching.NewHungarian(routingAlgorithm)

	heuristic := heuristics.NewHeuristics(routingAlgorithm, ch)
	alternativeRouter := routingalgorithm.NewAlternativeRouteXCHV(3, routingAlgorithm)

	navigatorSvc := service.NewNavigationService(ch, kvDB, hungarian, routingAlgorithm, heuristic, alternativeRouter)
	recordMemProfile(memprofile, "service_init")

	rest.NavigatorRouter(r, navigatorSvc, m)

	fmt.Printf("\n Contraction Hieararchies + Bidirectional Dijkstra Ready!!")
	fmt.Printf("\nserver started at %s\n", *listenAddr)

	log.Fatal(http.ListenAndServe(*listenAddr, r))
}

func recordMemProfile(memprofile *string, name string) {
	if *memprofile != "" {
		*memprofile = strings.Replace(*memprofile, ".mprof", fmt.Sprintf("%s.mprof", name), -1)
		f, err := os.Create(*memprofile)
		if err != nil {
			log.Fatal(err)
		}
		pprof.WriteHeapProfile(f)
		f.Close()
	}

}

// use log middleware below if u want to use elk for logging
// logFile, err := os.OpenFile("./logs/navigatorx.log", os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0777)
// if err != nil {
// 	log.Fatal(err)
// }
// logger := httplog.NewLogger("navigatorx", httplog.Options{
// 	Writer:   io.MultiWriter(os.Stdout, logFile),
// 	LogLevel: slog.LevelDebug,
// 	JSON:     true,
// 	Concise:  true,
// 	// RequestHeaders:   true,
// 	// ResponseHeaders:  true,
// 	MessageFieldName: "message",
// 	LevelFieldName:   "severity",
// 	TimeFieldFormat:  time.RFC3339,
// 	Tags: map[string]string{
// 		"version": "v1.0",
// 		"env":     "dev",
// 	},
// 	QuietDownRoutes: []string{
// 		"/metrics",
// 	},
// 	QuietDownPeriod: 10 * time.Second,
// })
// r.Use(httplog.RequestLogger(logger, []string{}))
