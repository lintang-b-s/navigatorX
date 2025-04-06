package contractor

import (
	"fmt"
	"math"
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
	k = 20
)

func (ch *ContractedGraph) SnapLocationToRoadNetworkNodeH3WithSccAnalysis(edgesFrom, edgesTo []datastructure.KVEdge,
	wantToSnapFrom, wantToSnapTo []float64) (int32, int32, datastructure.Coordinate, datastructure.Coordinate, error) {

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

	var (
		bestEdgeFrom datastructure.KVEdge
		bestEdgeTo   datastructure.KVEdge
	)

	edgePriorities := make([]edgePairWithPriority, 0, len(edgesFrom)*len(edgesTo))

	for i, edgeFrom := range edgeSliceFrom.entries {
		for j, edgeTo := range edgeSliceTo.entries {

			fromSccID := ch.GetNodeSCCID(edgeFrom.ToNodeID)
			toSccID := ch.GetNodeSCCID(edgeTo.FromNodeID)
			if fromSccID == toSccID {
				edgePriorities = append(edgePriorities, edgePairWithPriority{
					edgeFrom:       edgeFrom,
					edgeTo:         edgeTo,
					priority:       snapObjectiveFunction(sameSccIDWeight, float64(i), float64(j)),
					projectionFrom: edgeSliceFrom.projection[i],
					projectionTo:   edgeSliceTo.projection[j],
				})
			} else if ch.IsCondensationGraphFromToConnected(edgeFrom.ToNodeID, edgeTo.FromNodeID) {
				edgePriorities = append(edgePriorities, edgePairWithPriority{
					edgeFrom:       edgeFrom,
					edgeTo:         edgeTo,
					priority:       snapObjectiveFunction(condensedGraphConnectedWeight, float64(i), float64(j)),
					projectionFrom: edgeSliceFrom.projection[i],
					projectionTo:   edgeSliceTo.projection[j],
				})
			}
		}
	}

	if len(edgePriorities) == 0 {
		return -1, -1, datastructure.Coordinate{}, datastructure.Coordinate{}, fmt.Errorf("no path found from %v,%v to %v,%v", wantToSnapFrom[0], wantToSnapFrom[1], wantToSnapTo[0], wantToSnapTo[1])
	}

	sort.Slice(edgePriorities, func(i, j int) bool {
		a := edgePriorities[i]
		b := edgePriorities[j]
		return a.priority > b.priority
	})

	bestEdgeFrom = edgePriorities[0].edgeFrom
	bestEdgeTo = edgePriorities[0].edgeTo
	bestProjectionFrom := edgePriorities[0].projectionFrom
	bestProjectionTo := edgePriorities[0].projectionTo

	return bestEdgeFrom.ToNodeID, bestEdgeTo.FromNodeID, bestProjectionFrom, bestProjectionTo, nil
}

func snapObjectiveFunction(weight, i, j float64) float64 {
	return weight * (1 / math.Pow(i+1, 3)) * (1 / math.Pow(j+1, 3))
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
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
