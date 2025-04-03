package contractor

import (
	"sort"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/geo"
)

func (ch *ContractedGraph) SnapLocationToRoadNetworkNodeH3WithSccAnalysis(edgesFrom, edgesTo []datastructure.KVEdge,
	wantToSnapFrom, wantToSnapTo []float64) (int32, int32) {
	ch.sortEdges(edgesFrom, wantToSnapFrom)
	ch.sortEdges(edgesTo, wantToSnapTo)

	var (
		bestEdgeFrom datastructure.KVEdge
		bestEdgeTo   datastructure.KVEdge
		bestNodeFrom int32
		bestNodeTo   int32
	)

	found := false
	for _, edgeFrom := range edgesFrom {
		for _, edgeTo := range edgesTo {
			sccIDFrom := ch.GetNodeSCCID(edgeFrom.FromNodeID)
			sccIDTo := ch.GetNodeSCCID(edgeTo.FromNodeID)

			if sccIDFrom == sccIDTo  {
				bestEdgeFrom = edgeFrom
				bestEdgeTo = edgeTo
				found = true
				break
			}
		}
		if found {
			break
		}
	}

	fromNode := ch.GetNode(bestEdgeFrom.FromNodeID)
	toNode := ch.GetNode(bestEdgeFrom.ToNodeID)

	distFrom := geo.CalculateHaversineDistance(wantToSnapFrom[0], wantToSnapFrom[1], fromNode.Lat, fromNode.Lon)
	distTo := geo.CalculateHaversineDistance(wantToSnapFrom[0], wantToSnapFrom[1], toNode.Lat, toNode.Lon)

	if distFrom < distTo {
		bestNodeFrom = bestEdgeFrom.FromNodeID
	} else {
		bestNodeFrom = bestEdgeFrom.ToNodeID
	}

	fromNode = ch.GetNode(bestEdgeTo.FromNodeID)
	toNode = ch.GetNode(bestEdgeTo.ToNodeID)
	distFrom = geo.CalculateHaversineDistance(wantToSnapTo[0], wantToSnapTo[1], fromNode.Lat, fromNode.Lon)
	distTo = geo.CalculateHaversineDistance(wantToSnapTo[0], wantToSnapTo[1], toNode.Lat, toNode.Lon)

	if distFrom < distTo {
		bestNodeTo = bestEdgeTo.FromNodeID
	} else {
		bestNodeTo = bestEdgeTo.ToNodeID
	}

	return bestNodeFrom, bestNodeTo
}

func (ch *ContractedGraph) sortEdges(edges []datastructure.KVEdge, wantToSnap []float64) {
	sort.Slice(edges, func(i, j int) bool {
		a := edges[i]
		b := edges[j]

		fromNodeA := ch.GetNode(a.FromNodeID)
		toNodeA := ch.GetNode(a.ToNodeID)

		fromNodeB := ch.GetNode(b.FromNodeID)
		toNodeB := ch.GetNode(b.ToNodeID)

		projectionA := geo.ProjectPointToLineCoord(
			datastructure.NewCoordinate(fromNodeA.Lat, fromNodeA.Lon),
			datastructure.NewCoordinate(toNodeA.Lat, toNodeA.Lon),
			datastructure.NewCoordinate(wantToSnap[0], wantToSnap[1]),
		)

		projectionB := geo.ProjectPointToLineCoord(
			datastructure.NewCoordinate(fromNodeB.Lat, fromNodeB.Lon),
			datastructure.NewCoordinate(toNodeB.Lat, toNodeB.Lon),
			datastructure.NewCoordinate(wantToSnap[0], wantToSnap[1]),
		)

		distA := geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], projectionA.Lat, projectionA.Lon)
		distB := geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], projectionB.Lat, projectionB.Lon)

		return distA < distB
	},
	)
}

func (ch *ContractedGraph) SnapLocationToRoadNetworkNodeH3(edges []datastructure.KVEdge, wantToSnap []float64) int32 {
	sort.Slice(edges, func(i, j int) bool {
		a := edges[i]
		b := edges[j]

		fromNodeA := ch.GetNode(a.FromNodeID)
		toNodeA := ch.GetNode(a.ToNodeID)

		fromNodeB := ch.GetNode(b.FromNodeID)
		toNodeB := ch.GetNode(b.ToNodeID)

		projectionA := geo.ProjectPointToLineCoord(
			datastructure.NewCoordinate(fromNodeA.Lat, fromNodeA.Lon),
			datastructure.NewCoordinate(toNodeA.Lat, toNodeA.Lon),
			datastructure.NewCoordinate(wantToSnap[0], wantToSnap[1]),
		)

		projectionB := geo.ProjectPointToLineCoord(
			datastructure.NewCoordinate(fromNodeB.Lat, fromNodeB.Lon),
			datastructure.NewCoordinate(toNodeB.Lat, toNodeB.Lon),
			datastructure.NewCoordinate(wantToSnap[0], wantToSnap[1]),
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
