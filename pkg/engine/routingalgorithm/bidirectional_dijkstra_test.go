package routingalgorithm

import (
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

func TestShortestPathBidirectionalDijkstraWithoutCH(t *testing.T) {
	ch := NewGraph()
	rt := NewRouteAlgorithm(ch)

	from := int32(0)
	to := int32(5)

	dummyEdgeFilter := func(edge datastructure.EdgeCH) bool {
		return true
	}

	path, edgePath, eta, dist := rt.ShortestPathBiDijkstra(from, to, dummyEdgeFilter, dummyEdgeFilter)
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
