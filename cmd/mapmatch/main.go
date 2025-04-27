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

	bolt "go.etcd.io/bbolt"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
	"github.com/go-chi/cors"
	mymiddleware "github.com/lintang-b-s/navigatorx/pkg/server/middleware"
)

var (
	listenAddr   = flag.String("listenaddr", ":5050", "server listen address")
	useRateLimit = flag.Bool("ratelimit", false, "use rate limit")
)

func main() {
	flag.Parse()

	ch := contractor.NewContractedGraph()
	err := ch.LoadGraph(true)

	if err != nil {
		log.Fatal(err)
	}

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
	if *useRateLimit {
		r.Use(mymiddleware.Limit)
	}
	r.Mount("/debug", middleware.Profiler())

	mmrest.MapMatchingRouter(r, mmSvc)
	log.Printf(" Map Matching ready!!!")
	log.Printf("server started at %s\n", *listenAddr)
	log.Fatal(http.ListenAndServe(*listenAddr, r))
}
