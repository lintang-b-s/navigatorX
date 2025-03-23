package routingalgorithm

import (
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/util"
	"testing"

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
	nodeP := datastructure.NewCHNode(0, 0, 0, 0, false)
	nodeV := datastructure.NewCHNode(1, 1, 0, 1, false)
	nodeQ := datastructure.NewCHNode(2, 2, 0, 2, false)
	nodeW := datastructure.NewCHNode(3, 3, 0, 3, false)
	nodeR := datastructure.NewCHNode(4, 4, 0, 4, false)
	nodeF := datastructure.NewCHNode(5, 5, 0, 5, false)

	edgePv := datastructure.NewEdgeCH(0, 10, 10, nodeP.ID, nodeV.ID, 0, 0)

	edgeVp := datastructure.NewEdgeCH(1, 10, 10, nodeV.ID, nodeP.ID, 0, 0)

	edgeVr := datastructure.NewEdgeCH(2, 3, 3, nodeV.ID, nodeR.ID, 0, 0)

	edgeRv := datastructure.NewEdgeCH(3, 3, 3, nodeR.ID, nodeV.ID, 0, 0)

	edgeVq := datastructure.NewEdgeCH(4, 6, 6, nodeV.ID, nodeQ.ID, 0, 0)

	edgeQv := datastructure.NewEdgeCH(5, 6, 6, nodeQ.ID, nodeV.ID, 0, 0)

	edgeQw := datastructure.NewEdgeCH(6, 5, 5, nodeQ.ID, nodeW.ID, 0, 0)

	edgeWq := datastructure.NewEdgeCH(7, 5, 5, nodeW.ID, nodeQ.ID, 0, 0)

	edgeWr := datastructure.NewEdgeCH(8, 5, 5, nodeW.ID, nodeR.ID, 0, 0)

	edgeRw := datastructure.NewEdgeCH(9, 5, 5, nodeR.ID, nodeW.ID, 0, 0)

	edgeWf := datastructure.NewEdgeCH(10, 15, 15, nodeW.ID, nodeF.ID, 0, 0)

	edgeFw := datastructure.NewEdgeCH(11, 15, 15, nodeF.ID, nodeW.ID, 0, 0)

	edgesExtraInfo := make([]datastructure.EdgeExtraInfo, 0)
	for i := 0; i < 12; i++ {
		edgesExtraInfo = append(edgesExtraInfo, datastructure.EdgeExtraInfo{
			PointsInBetween: make([]datastructure.Coordinate, 0),
		})
	}

	edges := []datastructure.EdgeCH{edgePv, edgeVp, edgeVr, edgeRv, edgeVq, edgeQv, edgeQw, edgeWq, edgeWr, edgeRw, edgeWf, edgeFw}
	streetDirections := make(map[string][2]bool)
	nodes := []datastructure.CHNode{nodeP, nodeV, nodeQ, nodeW, nodeR, nodeF}
	chGraph.InitCHGraph(nodes, edges, streetDirections, util.NewIdMap(),
		edgesExtraInfo)

	return chGraph
}

func TestShortestPathBidirectionalDijkstra(t *testing.T) {
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
	assert.Equal(t, float64(0), path[0].Lat) // lat nya ku samain sama id nodenya
	assert.Equal(t, float64(1), path[1].Lat)
	assert.Equal(t, float64(4), path[2].Lat)
	assert.Equal(t, float64(3), path[3].Lat)
	assert.Equal(t, float64(5), path[4].Lat)

	assert.Equal(t, int32(0), edgePath[0].FromNodeID)
	assert.Equal(t, int32(1), edgePath[0].ToNodeID)
	assert.Equal(t, int32(4), edgePath[1].ToNodeID)
	assert.Equal(t, int32(3), edgePath[2].ToNodeID)
	assert.Equal(t, int32(5), edgePath[3].ToNodeID)
}
