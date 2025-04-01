package main

import (
	"flag"
	"log"
	"net/http"

	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/engine/matching"
	"github.com/lintang-b-s/navigatorx/pkg/kv"
	mmrest "github.com/lintang-b-s/navigatorx/pkg/server/mm_rest"
	"github.com/lintang-b-s/navigatorx/pkg/server/mm_rest/service"
	"github.com/lintang-b-s/navigatorx/pkg/snap"

	badger "github.com/dgraph-io/badger/v4"
	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
	"github.com/go-chi/cors"
)

var (
	listenAddr = flag.String("listenaddr", ":5000", "server listen address")
)

func main() {
	flag.Parse()

	ch := contractor.NewContractedGraph()
	err := ch.LoadGraph()

	if err != nil {
		log.Fatal(err)
	}

	db, err := badger.Open(badger.DefaultOptions("./navigatorx_db"))
	if err != nil {
		log.Fatal(err)
	}

	kvDB := kv.NewKVDB(db)
	defer kvDB.Close()

	rtree := datastructure.NewRtree(25, 50, 2)
	roadSnapper := snap.NewRoadSnapper(rtree)

	roadSnapper.BuildRoadSnapper(ch)

	mapMatching := matching.NewHMMMapMatching(ch, kvDB)

	mmSvc := service.NewMapMatchingService(mapMatching, roadSnapper, kvDB, ch)

	// server

	r := chi.NewRouter()

	r.Use(middleware.Logger)

	r.Use(cors.Handler(cors.Options{
		AllowedOrigins:   []string{"https://*", "http://*"},
		AllowedMethods:   []string{"GET", "POST", "PUT", "DELETE", "OPTIONS"},
		AllowedHeaders:   []string{"Accept", "Authorization", "Content-Type", "X-CSRF-Token"},
		ExposedHeaders:   []string{"Link"},
		AllowCredentials: false,
		MaxAge:           300,
	}))
	r.Mount("/debug", middleware.Profiler())

	mmrest.MapMatchingRouter(r, mmSvc)
	log.Printf(" Map Matching ready!!!")
	log.Printf("server started at %s\n", *listenAddr)
	log.Fatal(http.ListenAndServe(*listenAddr, r))
}
