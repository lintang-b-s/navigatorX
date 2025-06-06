package routingalgorithm

import (
	"testing"

	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"

	"github.com/stretchr/testify/assert"
)

/*
dari https://jlazarsfeld.github.io/ch.150.project/sections/8-contraction/
p=0, v=1, q=2, w=3, r=4, f=5

	 p
	  \
	   \
	    10
	     \
		  v -----3----- r
		 /            /
		6            5
	   /    		/
	  q ---5----- w ----15---- f

semua edge bidirectional
*/
func NewGraph() *contractor.ContractedGraph {
	chGraph := contractor.NewContractedGraph()
	nodeP := datastructure.NewCHNode(47.58677, -122.18003, 0, 0)
	nodeV := datastructure.NewCHNode(47.5788, -122.2332, 0, 1)
	nodeQ := datastructure.NewCHNode(47.64029, -122.17226, 0, 2)
	nodeW := datastructure.NewCHNode(47.62734, -122.14634, 0, 3)
	nodeR := datastructure.NewCHNode(47.60350, -122.18170, 0, 4)
	nodeF := datastructure.NewCHNode(47.57074, -122.16883, 0, 5)

	edgePv := datastructure.NewEdgePlain(0, 10, 10, nodeP.ID, nodeV.ID)

	edgeVp := datastructure.NewEdgePlain(1, 10, 10, nodeV.ID, nodeP.ID)

	edgeVr := datastructure.NewEdgePlain(2, 3, 3, nodeV.ID, nodeR.ID)

	edgeRv := datastructure.NewEdgePlain(3, 3, 3, nodeR.ID, nodeV.ID)

	edgeVq := datastructure.NewEdgePlain(4, 6, 6, nodeV.ID, nodeQ.ID)

	edgeQv := datastructure.NewEdgePlain(5, 6, 6, nodeQ.ID, nodeV.ID)

	edgeQw := datastructure.NewEdgePlain(6, 5, 5, nodeQ.ID, nodeW.ID)

	edgeWq := datastructure.NewEdgePlain(7, 5, 5, nodeW.ID, nodeQ.ID)

	edgeWr := datastructure.NewEdgePlain(8, 5, 5, nodeW.ID, nodeR.ID)

	edgeRw := datastructure.NewEdgePlain(9, 5, 5, nodeR.ID, nodeW.ID)

	edgeWf := datastructure.NewEdgePlain(10, 15, 15, nodeW.ID, nodeF.ID)

	edgeFw := datastructure.NewEdgePlain(11, 15, 15, nodeF.ID, nodeW.ID)

	edgesExtraInfo := make([]datastructure.EdgeExtraInfo, 0)
	for i := 0; i < 12; i++ {
		edgesExtraInfo = append(edgesExtraInfo, datastructure.EdgeExtraInfo{
			StreetName:    i,
			RoadClass:     uint8(i + 1),
			RoadClassLink: uint8(i + 2),
		})
	}

	edges := []datastructure.Edge{edgePv, edgeVp, edgeVr, edgeRv, edgeVq, edgeQv, edgeQw, edgeWq, edgeWr, edgeRw, edgeWf, edgeFw}
	graphStorage := datastructure.NewGraphStorage()
	for _, edge := range edges {
		graphStorage.AppendEdgeStorage(
			edge,
		)
	}

	streetDirections := make(map[string][2]bool)
	nodes := []datastructure.CHNode{nodeP, nodeV, nodeQ, nodeW, nodeR, nodeF}
	graphStorage.NodeTrafficLight = append(graphStorage.NodeTrafficLight, make([]int32, 6)...)
	graphStorage.GlobalPoints = make([]datastructure.Coordinate, 10)
	graphStorage.MapEdgeInfo = append(graphStorage.MapEdgeInfo, edgesExtraInfo...)
	chGraph.InitCHGraph(nodes, graphStorage, streetDirections, util.NewIdMap())

	return chGraph
}

func TestShortestPathBidirectionalDijkstraCH(t *testing.T) {
	ch := NewGraph()
	ch.Contraction()
	rt := NewRouteAlgorithm(ch)

	from := int32(0)
	to := int32(5)

	path, edgePath, eta, dist := rt.ShortestPathBiDijkstraCH(from, to)
	assert.Equal(t, 5, len(path))
	assert.Equal(t, 33.0, eta)
	assert.Equal(t, 33.0, dist*1000)

	// shortest path nya:  P(0) -> V(1) -> R(4) -> W(3) -> F(5)
	assert.Equal(t, 47.58677, path[0].Lat)
	assert.Equal(t, 47.5788, path[1].Lat)
	assert.Equal(t, 47.60350, path[2].Lat)
	assert.Equal(t, 47.62734, path[3].Lat)
	assert.Equal(t, 47.57074, path[4].Lat)

	assert.Equal(t, int32(0), edgePath[0].FromNodeID)
	assert.Equal(t, int32(1), edgePath[0].ToNodeID)
	assert.Equal(t, int32(4), edgePath[1].ToNodeID)
	assert.Equal(t, int32(3), edgePath[2].ToNodeID)
	assert.Equal(t, int32(5), edgePath[3].ToNodeID)
}
