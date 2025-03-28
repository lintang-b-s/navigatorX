package service

import (
	"context"
	"fmt"
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/engine/matching"
	"lintang/navigatorx/pkg/kv"
	"lintang/navigatorx/pkg/osmparser"
	"lintang/navigatorx/pkg/snap"
	"lintang/navigatorx/pkg/util"
	"log"
	"testing"

	"github.com/dgraph-io/badger/v4"
	"github.com/stretchr/testify/assert"
)

type simplenode struct {
	lat float64
	lon float64
	id  int32
}

func buildGraph() (*contractor.ContractedGraph, []datastructure.CHNode, []datastructure.EdgeCH) {
	edges := [][2]simplenode{
		[2]simplenode{{47.615248, -122.320817, 0},
			{47.615248, -122.321466, 1}},
		[2]simplenode{{47.615248, -122.321466, 1},
			{47.615157, -122.321464, 2}},
		[2]simplenode{{47.615157, -122.321464, 2},
			{47.614111, -122.321455, 3}},
		[2]simplenode{{47.615248, -122.321466, 1}, {47.615233, -122.322150, 4}},
		[2]simplenode{{47.615233, -122.322150, 4}, {47.614087, -122.322129, 5}},
		[2]simplenode{{47.614087, -122.322129, 5}, {47.614094, -122.323413, 6}},
		[2]simplenode{{47.614087, -122.322129, 5}, {47.612984, -122.322126, 7}},
		[2]simplenode{{47.612984, -122.322126, 7}, {47.612995, -122.323384, 8}},
		[2]simplenode{{47.612995, -122.323384, 8}, {47.612999, -122.324449, 9}},
		[2]simplenode{{47.612999, -122.324449, 9}, {47.613014, -122.325511, 10}},
		[2]simplenode{{47.613014, -122.325511, 10}, {47.612950, -122.326863, 11}},
		[2]simplenode{{47.614094, -122.323413, 6}, {47.614091, -122.324470, 12}},
		[2]simplenode{{47.614091, -122.324470, 12}, {47.614113, -122.325522, 13}},
		[2]simplenode{{47.614113, -122.325522, 13}, {47.614084, -122.326798, 14}},
		[2]simplenode{{47.614084, -122.326798, 14}, {47.612950, -122.326863, 11}},
		[2]simplenode{{47.612950, -122.326863, 11}, {47.613939, -122.327796, 15}},
		[2]simplenode{{47.613939, -122.327796, 15}, {47.613704, -122.328349, 16}},
		[2]simplenode{{47.613704, -122.328349, 16}, {47.613483, -122.328907, 17}},
		[2]simplenode{{47.613704, -122.328349, 16}, {47.614221, -122.328810, 18}},
		[2]simplenode{{47.613483, -122.328907, 17}, {47.614453, -122.329792, 19}},
		[2]simplenode{{47.614453, -122.329792, 19}, {47.615458, -122.330661, 20}},
		[2]simplenode{{47.615458, -122.330661, 20}, {47.616615, -122.327904, 21}},
		[2]simplenode{{47.609166, -122.332517, 22}, {47.614393, -122.329811, 23}},
		[2]simplenode{{47.614393, -122.329811, 23}, {47.616264, -122.329006, 24}},
		[2]simplenode{{47.616264, -122.329006, 24}, {47.618475, -122.328539, 25}},
	}

	nodes := make([]datastructure.CHNode, 0, len(edges))
	edgesCH := make([]datastructure.EdgeCH, len(edges))
	isAdded := make(map[int32]bool)

	speed := 40.0
	copyEdges := make([][2]simplenode, len(edges))
	copy(copyEdges, edges)

	edgesExtraInfo := make([]datastructure.EdgeExtraInfo, 0)

	for i, e := range copyEdges {
		if !isAdded[e[0].id] {
			nodes = append(nodes, datastructure.NewCHNodePlain(e[0].lat, e[0].lon, e[0].id))
		}
		isAdded[e[0].id] = true

		if !isAdded[e[1].id] {
			nodes = append(nodes, datastructure.NewCHNodePlain(e[1].lat, e[1].lon, e[1].id))
		}
		isAdded[e[1].id] = true

		dist := datastructure.HaversineDistance(e[0].lat, e[0].lon, e[1].lat, e[1].lon)
		eta := dist / speed

		pointsInBetween := make([]datastructure.Coordinate, 0)
		pointsInBetween = append(pointsInBetween, datastructure.Coordinate{Lat: e[0].lat, Lon: e[0].lon})
		pointsInBetween = append(pointsInBetween, datastructure.Coordinate{Lat: e[1].lat, Lon: e[1].lon})
		edgesExtraInfo = append(edgesExtraInfo, datastructure.EdgeExtraInfo{
			PointsInBetween: pointsInBetween,
		})

		edgesCH[i] = datastructure.NewEdgeCHPlain(int32(i), eta, dist, e[1].id, e[0].id)
	}

	ch := contractor.NewContractedGraph()

	nodes = util.QuickSortG(nodes, func(a, b datastructure.CHNode) int {
		return int(a.ID - b.ID)
	})

	ch.InitCHGraph(nodes, edgesCH, make(map[string][2]bool), util.NewIdMap(),
		edgesExtraInfo, datastructure.NewNodeInfo())

	return ch, nodes, edgesCH
}

func TestMapMatching(t *testing.T) {
	graph, nodes, edges := buildGraph()
	gps := []datastructure.Coordinate{
		{47.615298, -122.320881},
		{47.615177, -122.321351},
		{47.615208, -122.321858},
		{47.615201, -122.322223},
		{47.614704, -122.322190},
		{47.614348, -122.322212},
		{47.614120, -122.322064},
		{47.614132, -122.322730},
		{47.614055, -122.323381},
		{47.614073, -122.323867},
		{47.614064, -122.324264},
		{47.614083, -122.324653},
		{47.614055, -122.325034},
		{47.614078, -122.325527},
		{47.614065, -122.325656},
		{47.614067, -122.325964},
		{47.614064, -122.326214},
		{47.614071, -122.326530},
		{47.614060, -122.326823},
		{47.613675, -122.326841},
		{47.613536, -122.326841},
		{47.613259, -122.326857},
		{47.613022, -122.326855},
		{47.613955, -122.327794},
		{47.613780, -122.328376},
		{47.613498, -122.328912},
		{47.613885, -122.329293},
		{47.614441, -122.329781},
		{47.615483, -122.330688},
		{47.616347, -122.328714},
		{47.616441, -122.328247},
	}

	db, err := badger.Open(badger.DefaultOptions("./navigatorx_db"))
	if err != nil {
		t.Error(err)
	}

	kvDB := kv.NewKVDB(db)
	defer kvDB.Close()

	roadSnapper := snap.NewRoadSnapper(datastructure.NewRtree(25, 50, 2))
	roadSnapper.BuildRoadSnapper(graph)

	mapMatcher := matching.NewHMMMapMatching(graph, kvDB)
	mmSvc := NewMapMatchingService(mapMatcher, roadSnapper, kvDB, graph)

	actualPolyline, actualCoords, actualEdges, obsPath, err := mmSvc.MapMatch(context.Background(), gps)
	if err != nil {
		t.Error(err)
	}

	_, _, _ = actualCoords, actualEdges, obsPath
	_, _ = nodes, edges

	expectedPath := []simplenode{
		{47.615248, -122.320817, 0},
		{47.615248, -122.321466, 1},
		{47.615233, -122.322150, 4},
		{47.614087, -122.322129, 5},
		{47.614094, -122.323413, 6},
		{47.614091, -122.324470, 12},
		{47.614113, -122.325522, 13},
		{47.614084, -122.326798, 14},
		{47.612950, -122.326863, 11},
		{47.613939, -122.327796, 15},
		{47.613704, -122.328349, 16},
		{47.613483, -122.328907, 17},
		{47.614453, -122.329792, 19},
		{47.615458, -122.330661, 20},
		{47.616615, -122.327904, 21},
	}

	expectedCoords := make([]datastructure.Coordinate, len(expectedPath))
	for i, n := range expectedPath {
		expectedCoords[i] = datastructure.Coordinate{Lat: n.lat, Lon: n.lon}
	}
	expectedPolyline := datastructure.CreatePolyline(expectedCoords)

	fmt.Printf("expected polyline: %s\n", expectedPolyline)
	fmt.Printf("actual polyline: %s\n", actualPolyline)

	actualPath := make([]simplenode, 0)
	visitedpath := make(map[int32]struct{})
	for _, e := range actualEdges {

		ePointsInBetween := graph.GetEdgePointsInBetween(e.EdgeID)

		visitedpath[e.FromNodeID] = struct{}{}
		actualPath = append(actualPath, simplenode{lat: ePointsInBetween[0].Lat,
			lon: ePointsInBetween[0].Lon, id: e.FromNodeID})

		actualPath = append(actualPath, simplenode{lat: ePointsInBetween[1].Lat,
			lon: ePointsInBetween[1].Lon, id: e.ToNodeID})
	}
	actualPath = removeDuplicate(actualPath)
	assert.Equal(t, expectedPath, actualPath)
}

func removeDuplicate(edges []simplenode) []simplenode {
	seen := make(map[int32]struct{})
	result := make([]simplenode, 0)
	for _, e := range edges {
		if _, ok := seen[e.id]; !ok {
			result = append(result, e)
			seen[e.id] = struct{}{}
		}
	}
	return result
}

func Test_IsReverse(t *testing.T) {
	osmParser := osmparser.NewOSMParserV2()
	processedNodes, processedEdges, streetDirection,
		edgesExtraInfo, nodeInfo := osmParser.Parse("wa-microsoft.osm.pbf")

	ch := contractor.NewContractedGraph()

	ch.InitCHGraph(processedNodes, processedEdges, streetDirection, osmParser.GetTagStringIdMap(),
		edgesExtraInfo, nodeInfo)

	ch.Contraction()

	for _, edgeID := range ch.GetOutEdges() {
		outEdge := ch.GetOutEdge(edgeID.EdgeID)
		inEdge := ch.GetInEdge(edgeID.EdgeID)

		if outEdge.FromNodeID != inEdge.ToNodeID &&
			outEdge.ToNodeID != inEdge.FromNodeID {
			log.Printf("FAIL: edgeID: %d, outEdge: %v, inEdge: %v\n", edgeID.EdgeID, outEdge, inEdge)
			t.Errorf("edgeID: %d, outEdge: %v, inEdge: %v\n", edgeID.EdgeID, outEdge, inEdge)
		}
	}

	log.Printf("success")
}
