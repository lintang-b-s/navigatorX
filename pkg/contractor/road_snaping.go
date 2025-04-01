package contractor

import (
	"sort"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/geo"
)

func (ch *ContractedGraph) SnapLocationToRoadNetworkNodeH3(edges []datastructure.KVEdge, wantToSnap []float64) int32 {
	sort.Slice(edges, func(i, j int) bool {
		a := edges[i]
		b := edges[j]

		fromNodeA := ch.GetNode(a.FromNodeID)
		toNodeA := ch.GetNode(a.ToNodeID)

		fromNodeB := ch.GetNode(b.FromNodeID)
		toNodeB := ch.GetNode(b.ToNodeID)

		projectionA := geo.ProjectPointToLineCoord(
			geo.NewCoordinate(fromNodeA.Lat, fromNodeA.Lon),
			geo.NewCoordinate(toNodeA.Lat, toNodeA.Lon),
			geo.NewCoordinate(wantToSnap[0], wantToSnap[1]),
		)

		projectionB := geo.ProjectPointToLineCoord(
			geo.NewCoordinate(fromNodeB.Lat, fromNodeB.Lon),
			geo.NewCoordinate(toNodeB.Lat, toNodeB.Lon),
			geo.NewCoordinate(wantToSnap[0], wantToSnap[1]),
		)

		distA := geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], projectionA.Lat, projectionA.Lon)
		distB := geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], projectionB.Lat, projectionB.Lon)

		return distA < distB
	},
	)

	bestEdge := edges[0]

	fromNode := ch.GetNode(bestEdge.FromNodeID)
	toNode := ch.GetNode(bestEdge.ToNodeID)

	distFrom := geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], fromNode.Lat, fromNode.Lon)
	distTo := geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], toNode.Lat, toNode.Lon)

	if distFrom < distTo {
		return bestEdge.FromNodeID
	}
	return bestEdge.ToNodeID
}
