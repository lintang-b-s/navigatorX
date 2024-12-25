package routingalgorithm

import (
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/datastructure"
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
	maxSpeed := 0.06
	nodeP := datastructure.NewNode(0, 0, 0, "P", false, 2)
	nodeV := datastructure.NewNode(1, 1, 1, "V", false, 2)
	nodeQ := datastructure.NewNode(2, 2, 2, "Q", false, 2)
	nodeW := datastructure.NewNode(3, 3, 3, "W", false, 2)
	nodeR := datastructure.NewNode(4, 4, 4, "R", false, 2)
	nodeF := datastructure.NewNode(5, 5, 5, "F", false, 2)

	edgePv := datastructure.NewEdge(nodeP, nodeV, 10, "p->v", maxSpeed, false, "primary", "primary", 2, nil)
	nodeP.AddEdge(edgePv)

	edgeVp := datastructure.NewEdge(nodeV, nodeP, 10, "v->p", maxSpeed, false, "primary", "primary", 2, nil)
	nodeV.AddEdge(edgeVp)

	edgeVr := datastructure.NewEdge(nodeV, nodeR, 3, "v->r", maxSpeed, false, "primary", "primary", 2, nil)
	nodeV.AddEdge(edgeVr)

	edgeRv := datastructure.NewEdge(nodeR, nodeV, 3, "r->v", maxSpeed, false, "primary", "primary", 2, nil)
	nodeR.AddEdge(edgeRv)

	edgeVq := datastructure.NewEdge(nodeV, nodeQ, 6, "v->q", maxSpeed, false, "primary", "primary", 2, nil)
	nodeV.AddEdge(edgeVq)

	edgeQv := datastructure.NewEdge(nodeQ, nodeV, 6, "q->v", maxSpeed, false, "primary", "primary", 2, nil)
	nodeQ.AddEdge(edgeQv)

	edgeQw := datastructure.NewEdge(nodeQ, nodeW, 5, "q->w", maxSpeed, false, "primary", "primary", 2, nil)
	nodeQ.AddEdge(edgeQw)

	edgeWq := datastructure.NewEdge(nodeW, nodeQ, 5, "w->q", maxSpeed, false, "primary", "primary", 2, nil)
	nodeW.AddEdge(edgeWq)

	edgeWr := datastructure.NewEdge(nodeW, nodeR, 5, "w->r", maxSpeed, false, "primary", "primary", 2, nil)
	nodeW.AddEdge(edgeWr)

	edgeRw := datastructure.NewEdge(nodeR, nodeW, 5, "r->w", maxSpeed, false, "primary", "primary", 2, nil)
	nodeR.AddEdge(edgeRw)

	edgeWf := datastructure.NewEdge(nodeW, nodeF, 15, "w->f", maxSpeed, false, "primary", "primary", 2, nil)
	nodeW.AddEdge(edgeWf)

	edgeFw := datastructure.NewEdge(nodeF, nodeW, 15, "f->w", maxSpeed, false, "primary", "primary", 2, nil)
	nodeF.AddEdge(edgeFw)

	streetDirections := make(map[string][2]bool)
	streetExtraInfo := make(map[string]datastructure.StreetExtraInfo)
	nodes := []datastructure.Node{*nodeP, *nodeV, *nodeQ, *nodeW, *nodeR, *nodeF}
	chGraph.InitCHGraph(nodes, 12, streetDirections, streetExtraInfo)

	return chGraph
}

func TestShortestPathBidirectionalDijkstra(t *testing.T) {
	ch := NewGraph()
	ch.Contraction()
	rt := NewRouteAlgorithm(ch)

	from := int32(0)
	to := int32(5)

	path, edgePath, eta, dist := rt.ShortestPathBiDijkstra(from, to)
	assert.Equal(t, 5, len(path))
	assert.Equal(t, 33.0, eta)
	assert.Equal(t, 33.0, dist*1000)

	// shortest path nya:  P(0) -> V(1) -> R(4) -> W(3) -> F(5)
	assert.Equal(t, float64(0), path[0].Lat) // lat nya ku samain sama id nodenya
	assert.Equal(t, float64(1), path[1].Lat)
	assert.Equal(t, float64(4), path[2].Lat)
	assert.Equal(t, float64(3), path[3].Lat)
	assert.Equal(t, float64(5), path[4].Lat)

	assert.Equal(t, int32(0), edgePath[0].BaseNodeIDx)
	assert.Equal(t, int32(1), edgePath[0].ToNodeIDX)
	assert.Equal(t, int32(4), edgePath[1].ToNodeIDX)
	assert.Equal(t, int32(3), edgePath[2].ToNodeIDX)
	assert.Equal(t, int32(5), edgePath[3].ToNodeIDX)
}
