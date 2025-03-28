package main

import (
	"context"
	"flag"
	"fmt"
	_ "lintang/navigatorx/docs"
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/kv"
	"lintang/navigatorx/pkg/osmparser"
	"log"
	"os"
	"runtime/pprof"
	"strings"
	"sync"

	_ "net/http/pprof"

	badger "github.com/dgraph-io/badger/v4"
)

var (
	listenAddr = flag.String("listenaddr", ":5000", "server listen address")
	mapFile    = flag.String("f", "solo_jogja.osm.pbf", "openstreeetmap file buat road network graphnya")
	cpuprofile = flag.String("cpuprofile", "", "write cpu profile to file")
	memprofile = flag.String("memprofile", "", "write memory profile to this file")
	mapmatch   = flag.Bool("mapmatch", false, "enable map matching")
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
	if *cpuprofile != "" {
		// https://go.dev/blog/pprof
		// ./bin/osm-search-indexer -f "jabodetabek_big.osm.pbf" -cpuprofile=osmsearchcpu.prof -memprofile=osmsearchmem.mprof
		f, err := os.Create(*cpuprofile)
		if err != nil {
			log.Fatal(err)
		}
		defer f.Close()

		pprof.StartCPUProfile(f)
		defer pprof.StopCPUProfile()
	}

	log.Printf("reading osm file %s", *mapFile)
	osmParser := osmparser.NewOSMParserV2()
	processedNodes, processedEdges, streetDirection,
		edgesExtraInfo := osmParser.Parse(*mapFile)

	ch := contractor.NewContractedGraph()

	db, err := badger.Open(badger.DefaultOptions("./navigatorx_db"))
	if err != nil {
		log.Fatal(err)
	}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	kvDB := kv.NewKVDB(db)
	defer kvDB.Close()

	recordMemProfile(memprofile, "parsing_osm_data")
	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		err = kvDB.BuildH3IndexedEdges(ctx, processedEdges, edgesExtraInfo)
		if err != nil {
			panic(err)
		}
	}()

	ch.InitCHGraph(processedNodes, processedEdges, streetDirection, osmParser.GetTagStringIdMap(),
		edgesExtraInfo)

	if !*mapmatch {
		ch.Contraction()
	}

	log.Printf("Saving Contracted Graph to a file...")
	err = ch.SaveToFile()
	if err != nil {
		panic(err)
	}

	wg.Wait()
	recordMemProfile(memprofile, "finish_contracting_graph")

	fmt.Printf("\n Contraction Hieararchies + Bidirectional Dijkstra Ready!!")

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
