package main

import (
	"flag"
	"fmt"
	_ "lintang/navigatorx/docs"
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/kv"
	"lintang/navigatorx/pkg/osmparser"
	"log"

	_ "net/http/pprof"

	"github.com/cockroachdb/pebble"
)

var (
	listenAddr = flag.String("listenaddr", ":5000", "server listen address")
	mapFile    = flag.String("f", "solo_jogja.osm.pbf", "openstreeetmap file buat road network graphnya")
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
	osmParser := osmparser.NewOSMParser(ch)
	hmmEdges, nodeIdxMap, graphEdges := osmParser.BikinGraphFromOpenstreetmap(*mapFile)

	db, err := pebble.Open("navigatorxDB", &pebble.Options{})
	if err != nil {
		log.Fatal(err)
	}

	kvDB := kv.NewKVDB(db)
	defer kvDB.Close()

	go func() {
		kvDB.CreateStreetKV(graphEdges, hmmEdges, nodeIdxMap, *listenAddr, true)
	}()

	osmParser.CH.Contraction()
	osmParser.CH.SetCHReady()
	err = osmParser.SaveToFile()
	if err != nil {
		log.Fatal(err)
	}

	fmt.Printf("\n Contraction Hieararchies + Bidirectional Dijkstra Ready!!")

}
