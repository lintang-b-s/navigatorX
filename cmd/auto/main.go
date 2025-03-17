package main

import (
	"flag"
	_ "lintang/navigatorx/docs"

	_ "net/http/pprof"
)

var (
	listenAddr    = flag.String("listenaddr", ":5000", "server listen address")
	mapFile       = flag.String("f", "solo_jogja.osm.pbf", "openstreeetmap file buat road network graphnya")
	isMapMatching = flag.Bool("mapmatching", false, "apakah mau dijalankan map matching atau tidak")
)

//	@title			navigatorx lintangbs API
//	@version		1.0
//	@description	simple openstreetmap routing engine in go

//	@contact.name	lintang birda saputra
//	@description 	simple openstreetmap routing engine in go. Using Contraction Hierarchies for preprocessing and Bidirectioanl Dijkstra for shortest path query

//	@license.name	GNU Affero General Public License v3.0
//	@license.url	https://www.gnu.org/licenses/gpl-3.0.en.html

// // @host		localhost:5000
// // @BasePath	/api
// // @schemes	http
// func main() {
// 	flag.Parse()
// 	ch := contractor.NewContractedGraph()
// 	osmParser := osmparser.NewOSMParser(ch)
// 	_, nodeIdxMap, graphEdges, _ := osmParser.BikinGraphFromOpenstreetmap(*mapFile, *isMapMatching)

// 	db, err := badger.Open(badger.DefaultOptions("./navigatorx_db"))
// 	if err != nil {
// 		log.Fatal(err)
// 	}

// 	kvDB := kv.NewKVDB(db)
// 	defer kvDB.Close()

// 	go func() {
// 		kvDB.CreateStreetKV(graphEdges, nodeIdxMap, *listenAddr, false)
// 	}()

// 	reg := prometheus.NewRegistry()
// 	m := rest.NewMetrics(reg)
// 	// alg.BikinRtreeStreetNetwork(graphEdges, ch, nodeIdxMap)

// 	r := chi.NewRouter()

// 	r.Use(middleware.Logger)

// 	r.Use(rest.PromeHttpMiddleware(m)) // prometheus http middleware
// 	r.Use(cors.Handler(cors.Options{
// 		AllowedOrigins:   []string{"https://*", "http://*"},
// 		AllowedMethods:   []string{"GET", "POST", "PUT", "DELETE", "OPTIONS"},
// 		AllowedHeaders:   []string{"Accept", "Authorization", "Content-Type", "X-CSRF-Token"},
// 		ExposedHeaders:   []string{"Link"},
// 		AllowCredentials: false,
// 		MaxAge:           300,
// 	}))
// 	r.Mount("/debug", middleware.Profiler())

// 	r.Handle("/metrics", promhttp.HandlerFor(reg, promhttp.HandlerOpts{}))

// 	r.Get("/swagger/*", httpSwagger.Handler(
// 		httpSwagger.URL("http://localhost:5000/swagger/doc.json"), //The url pointing to API definition
// 	))

// 	routingAlgorithm := routingalgorithm.NewRouteAlgorithm(osmParser.CH)
// 	hungarian := riderdrivermatching.NewHungarian(routingAlgorithm)

// 	heuristic := heuristics.NewHeuristics(routingAlgorithm, osmParser.CH)
// 	mapMatching := matching.NewHMMMapMatching(osmParser.CH, kvDB, routingAlgorithm)

// 	navigatorSvc := service.NewNavigationService(osmParser.CH, kvDB, hungarian, routingAlgorithm, mapMatching, heuristic)
// 	rest.NavigatorRouter(r, navigatorSvc, m)

// 	go func() {
// 		osmParser.CH.Contraction()
// 		osmParser.CH.SetCHReady()
// 		runtime.GC()
// 		runtime.GC() // run garbage collection biar heap size nya ngurang
// 		fmt.Printf("\n Contraction Hieararchies + Bidirectional Dijkstra Ready!!")
// 		fmt.Printf("\nserver started at %s\n", *listenAddr)
// 	}()

// 	log.Fatal(http.ListenAndServe(*listenAddr, r))
// }

// // use log middleware below if u want to use elk for logging
// // logFile, err := os.OpenFile("./logs/navigatorx.log", os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0777)
// // if err != nil {
// // 	log.Fatal(err)
// // }
// // logger := httplog.NewLogger("navigatorx", httplog.Options{
// // 	Writer:   io.MultiWriter(os.Stdout, logFile),
// // 	LogLevel: slog.LevelDebug,
// // 	JSON:     true,
// // 	Concise:  true,
// // 	// RequestHeaders:   true,
// // 	// ResponseHeaders:  true,
// // 	MessageFieldName: "message",
// // 	LevelFieldName:   "severity",
// // 	TimeFieldFormat:  time.RFC3339,
// // 	Tags: map[string]string{
// // 		"version": "v1.0",
// // 		"env":     "dev",
// // 	},
// // 	QuietDownRoutes: []string{
// // 		"/metrics",
// // 	},
// // 	QuietDownPeriod: 10 * time.Second,
// // })
// // r.Use(httplog.RequestLogger(logger, []string{}))
