package osmparser

import (
	"context"
	"fmt"
	"io"
	"lintang/navigatorx/pkg/datastructure"
	"log"
	"os"
	"strings"
	"sync"

	"github.com/k0kubun/go-ansi"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
	"github.com/schollz/progressbar/v3"
)

type nodeMapContainer struct {
	nodeMap map[osm.NodeID]*osm.Node
	mu      sync.Mutex
}

type ContractedGraph interface {
	InitCHGraph(nodes []datastructure.Node, edgeCount int, streetDirections map[string][2]bool,
		streetExtraInfo map[string]datastructure.StreetExtraInfo) map[int64]int32
	GetFirstOutEdge(nodeIDx int32) []int32
	GetFirstInEdge(nodeIDx int32) []int32
	GetNode(nodeIDx int32) datastructure.CHNode2
	GetOutEdge(edgeIDx int32) datastructure.EdgeCH
	GetInEdge(edgeIDx int32) datastructure.EdgeCH
	GetNumNodes() int
	IsChReady() bool
	Contraction() (err error)
	SetCHReady()
	SnapLocationToRoadNetworkNodeH3(ways []datastructure.SmallWay, wantToSnap []float64) int32
	SnapLocationToRoadNetworkNodeH3ForMapMatching(ways []datastructure.SmallWay, wantToSnap []float64) []datastructure.State
	SaveToFile() error
	LoadGraph() error
	GetStreetDirection(streetName int) [2]bool
	GetStreetInfo(streetName int) datastructure.StreetExtraInfo
	GetStreetNameFromID(streetName int) string
	GetRoadClassFromID(roadClass int) string
	GetRoadClassLinkFromID(roadClassLink int) string
}

type OSMParser struct {
	CH ContractedGraph
}

func NewOSMParser(ch ContractedGraph) *OSMParser {
	return &OSMParser{
		CH: ch,
	}
}
func (op *OSMParser) LoadGraph() error {
	return op.CH.LoadGraph()
}
func (op *OSMParser) BikinGraphFromOpenstreetmap(mapFile string) ([]datastructure.SurakartaWay, map[int64]int32, []datastructure.SurakartaWay) {
	f, err := os.Open(mapFile)

	if err != nil {
		log.Fatal(err)
	}
	defer f.Close()

	scanner := osmpbf.New(context.Background(), f, 3)
	defer scanner.Close()

	count := 0
	var typeSet = make(map[osm.Type]int)

	ctr := nodeMapContainer{
		nodeMap: make(map[osm.NodeID]*osm.Node),
	}

	someWayCount := 0
	ways := []*osm.Way{}

	bar := progressbar.NewOptions(450000,
		progressbar.OptionSetWriter(ansi.NewAnsiStdout()),
		progressbar.OptionEnableColorCodes(true),
		progressbar.OptionShowBytes(true),
		progressbar.OptionSetWidth(15),
		progressbar.OptionSetDescription("[cyan][1/7][reset] memproses openstreetmap way..."),
		progressbar.OptionSetTheme(progressbar.Theme{
			Saucer:        "[green]=[reset]",
			SaucerHead:    "[green]>[reset]",
			SaucerPadding: " ",
			BarStart:      "[",
			BarEnd:        "]",
		}))
	nodeCount := 0
	wayNodesMap := make(map[osm.NodeID]bool)
	for scanner.Scan() {
		o := scanner.Object()
		// do something
		tipe := o.ObjectID().Type()
		typeSet[tipe] = typeSet[tipe] + 1
		if count%50000 == 0 {
			bar.Add(50000)
		}

		if tipe != "way" {
			continue
		}
		tag := o.(*osm.Way).TagMap()

		if !isOsmWayUsedByCars(tag) {
			continue
		}

		if tipe == osm.TypeWay {
			ways = append(ways, o.(*osm.Way))
			someWayCount++
			for _, node := range o.(*osm.Way).Nodes {
				wayNodesMap[node.ID] = true
			}
		}
		count++
	}

	f.Seek(0, io.SeekStart)
	if err != nil {
		log.Fatal(err)
	}
	scanner = osmpbf.New(context.Background(), f, 3)
	defer scanner.Close()

	for scanner.Scan() {
		o := scanner.Object()
		if o.ObjectID().Type() == osm.TypeNode {
			node := o.(*osm.Node)
			if _, ok := wayNodesMap[node.ID]; ok {
				ctr.nodeMap[o.(*osm.Node).ID] = o.(*osm.Node)
				nodeCount++
			}
		}
	}

	fmt.Println("")

	scanErr := scanner.Err()
	if scanErr != nil {
		panic(scanErr)
	}
	fmt.Println("jumlah osm nodes: " + fmt.Sprint(nodeCount))

	trafficLightNodeMap := make(map[string]int64)
	var trafficLightNodeIDMap = make(map[osm.NodeID]bool)

	fmt.Println("jumlah osm way: " + fmt.Sprint(someWayCount))
	for idx, way := range ways {
		for i := 0; i < len(way.Nodes); i++ {
			fromNodeID := way.Nodes[i].ID
			ways[idx].Nodes[i].Lat = ctr.nodeMap[fromNodeID].Lat
			ways[idx].Nodes[i].Lon = ctr.nodeMap[fromNodeID].Lon
			for _, tag := range ctr.nodeMap[fromNodeID].Tags {
				if strings.Contains(tag.Value, "traffic_signals") {
					trafficLightNodeMap[tag.Key+"="+tag.Value]++
					trafficLightNodeIDMap[way.Nodes[i].ID] = true
				}
			}
		}
	}

	hmmEdges, surakartaNodes, graphEdges, streetDirections, streetExtraInfo := InitGraph(ways, trafficLightNodeIDMap)

	nodeIdxMap := op.CH.InitCHGraph(surakartaNodes, len(ways), streetDirections, streetExtraInfo)
	convertOSMNodeIDToGraphID(graphEdges, nodeIdxMap)

	return hmmEdges, nodeIdxMap, graphEdges
}
func convertOSMNodeIDToGraphID(surakartaWays []datastructure.SurakartaWay, nodeIDxMap map[int64]int32) {
	for i := range surakartaWays {
		way := &surakartaWays[i]
		for i, nodeID := range way.IntersectionNodesID {

			way.IntersectionNodesID[i] = int64(nodeIDxMap[nodeID])
		}
	}
}
func isOsmWayUsedByCars(tagMap map[string]string) bool {
	_, ok := tagMap["junction"]
	if ok {
		return true
	}

	route, ok := tagMap["route"]
	if ok && route == "ferry" {
		return true
	}

	ferry, ok := tagMap["ferry"]
	if ok && ferry == "yes" {
		return true
	}

	highway, okHW := tagMap["highway"]
	if !okHW {
		return false
	}

	motorcar, ok := tagMap["motorcar"]
	if ok && motorcar == "no" {
		return false
	}

	motorVehicle, ok := tagMap["motor_vehicle"]
	if ok && motorVehicle == "no" {
		return false
	}

	access, ok := tagMap["access"]
	if ok {
		if !(access == "yes" || access == "permissive" || access == "designated" || access == "delivery" || access == "destination") {
			return false
		}
	}

	if okHW && (highway == "motorway" ||
		highway == "trunk" ||
		highway == "primary" ||
		highway == "secondary" ||
		highway == "tertiary" ||
		highway == "unclassified" ||
		highway == "residential" ||
		highway == "living_street" ||
		highway == "service" ||
		highway == "motorway_link" ||
		highway == "trunk_link" ||
		highway == "primary_link" ||
		highway == "secondary_link" ||
		highway == "tertiary_link") {
		return true
	}

	if highway == "bicycle_road" {
		motorcar, ok := tagMap["motorcar"]
		if ok {
			if motorcar == "yes" {
				return true
			}
		}
		return false
	}

	if highway == "construction" ||
		highway == "path" ||
		highway == "footway" ||
		highway == "cycleway" ||
		highway == "bridleway" ||
		highway == "pedestrian" ||
		highway == "bus_guideway" ||
		highway == "raceway" ||
		highway == "escape" ||
		highway == "steps" ||
		highway == "proposed" ||
		highway == "conveying" {
		return false
	}

	oneway, ok := tagMap["oneway"]
	if ok {
		if oneway == "reversible" || oneway == "alternating" {
			return false
		}
	}

	_, ok = tagMap["maxspeed"]
	if ok {
		return true
	}

	return false
}

func (op *OSMParser) SaveToFile() error {
	return op.CH.SaveToFile()
}
