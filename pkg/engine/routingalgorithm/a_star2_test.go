package routingalgorithm

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestShortestPathAstar(t *testing.T) {
	ch := NewGraph()
	rt := NewRouteAlgorithm(ch)

	from := int32(0)
	to := int32(5)

	path, edgePath, eta, dist := rt.ShortestPathAStar(from, to)
	assert.Equal(t, 5, len(path))
	assert.Equal(t, 33.0, eta)
	assert.InDelta(t, 33.0, dist*1000, 0.01)

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
