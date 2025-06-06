package contractor

import (
	"fmt"
	"sort"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/geo"
)

const (
	sameSccIDWeight               = 100.0
	condensedGraphConnectedWeight = 5.0
)

type edgePairWithPriority struct {
	edgeFrom       datastructure.KVEdge
	edgeTo         datastructure.KVEdge
	projectionFrom datastructure.Coordinate
	projectionTo   datastructure.Coordinate
	priority       float64
}

type edgeSlice struct {
	entries    []datastructure.KVEdge
	dists      []float64
	projection []datastructure.Coordinate
}

func (s edgeSlice) Len() int { return len(s.entries) }

func (s edgeSlice) Swap(i, j int) {
	s.entries[i], s.entries[j] = s.entries[j], s.entries[i]
	s.dists[i], s.dists[j] = s.dists[j], s.dists[i]
	s.projection[i], s.projection[j] = s.projection[j], s.projection[i]
}

func (s edgeSlice) Less(i, j int) bool {
	return s.dists[i] < s.dists[j]
}

const (
	k = 30
)

func (ch *ContractedGraph) SnapLocationToRoadSegmentNodeH3WithSccAnalysis(edgesFrom, edgesTo []datastructure.KVEdge,
	wantToSnapFrom, wantToSnapTo []float64) (int32, int32, datastructure.Coordinate, datastructure.Coordinate,
	datastructure.Edge, error) {

	// sort the edgeSlice by the perpendicular distance to the line

	distsFrom := make([]float64, len(edgesFrom))
	distsTo := make([]float64, len(edgesTo))
	projectionFrom := make([]datastructure.Coordinate, len(edgesFrom))
	projectionTo := make([]datastructure.Coordinate, len(edgesTo))

	for i, edgeFrom := range edgesFrom {
		fromNode := ch.GetNode(edgeFrom.FromNodeID)
		toNode := ch.GetNode(edgeFrom.ToNodeID)
		projection := geo.ProjectPointToLineCoord(
			datastructure.NewCoordinate(fromNode.Lat, fromNode.Lon),
			datastructure.NewCoordinate(toNode.Lat, toNode.Lon),
			datastructure.NewCoordinate(wantToSnapFrom[0], wantToSnapFrom[1]),
		)
		distsFrom[i] = geo.CalculateHaversineDistance(wantToSnapFrom[0], wantToSnapFrom[1], projection.Lat, projection.Lon)
		projectionFrom[i] = projection
	}
	for i, edgeTo := range edgesTo {
		fromNode := ch.GetNode(edgeTo.FromNodeID)
		toNode := ch.GetNode(edgeTo.ToNodeID)
		projection := geo.ProjectPointToLineCoord(
			datastructure.NewCoordinate(fromNode.Lat, fromNode.Lon),
			datastructure.NewCoordinate(toNode.Lat, toNode.Lon),
			datastructure.NewCoordinate(wantToSnapTo[0], wantToSnapTo[1]),
		)
		distsTo[i] = geo.CalculateHaversineDistance(wantToSnapTo[0], wantToSnapTo[1], projection.Lat, projection.Lon)
		projectionTo[i] = projection
	}

	edgeSliceFrom := edgeSlice{
		entries:    edgesFrom,
		dists:      distsFrom,
		projection: projectionFrom,
	}
	edgeSliceTo := edgeSlice{
		entries:    edgesTo,
		dists:      distsTo,
		projection: projectionTo,
	}
	sort.Sort(edgeSliceFrom)
	sort.Sort(edgeSliceTo)

	edgeSliceFrom.entries = edgeSliceFrom.entries[:min(len(edgeSliceFrom.entries), k)]
	edgeSliceTo.entries = edgeSliceTo.entries[:min(len(edgeSliceTo.entries), k)]

	for i, edgeFrom := range edgeSliceFrom.entries {
		for j, edgeTo := range edgeSliceTo.entries {

			fromSccID := ch.GetNodeSCCID(edgeFrom.ToNodeID)
			toSccID := ch.GetNodeSCCID(edgeTo.FromNodeID)
			if fromSccID == toSccID || ch.IsCondensationGraphFromToConnected(edgeFrom.ToNodeID, edgeTo.FromNodeID) {
				bestEdgeFrom := edgeFrom
				bestEdgeTo := edgeTo
				bestProjectionFrom := projectionFrom[i]
				bestProjectionTo := projectionTo[j]

				snappedEdge := datastructure.Edge{}

				for _, edgeID := range ch.GetNodeFirstOutEdges(bestEdgeFrom.FromNodeID) {
					edge := ch.GetOutEdge(edgeID)
					if edge.ToNodeID == bestEdgeFrom.ToNodeID {
						snappedEdge = edge
					}
				}

				return bestEdgeFrom.ToNodeID, bestEdgeTo.FromNodeID, bestProjectionFrom, bestProjectionTo, snappedEdge, nil
			}
		}
	}

	return -1, -1, datastructure.Coordinate{}, datastructure.Coordinate{}, datastructure.Edge{},
		fmt.Errorf("no path found from %v,%v to %v,%v", wantToSnapFrom[0], wantToSnapFrom[1],
			wantToSnapTo[0], wantToSnapTo[1])
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func (ch *ContractedGraph) SnapLocationToRoadSegmentNodeH3(edges []datastructure.KVEdge, wantToSnap []float64) int32 {
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
