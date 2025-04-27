package matching

import "github.com/lintang-b-s/navigatorx/pkg/datastructure"

type ContractedGraph interface {
	GetOutEdges() []datastructure.Edge
	GetInEdges() []datastructure.Edge

	GetNodes() []datastructure.CHNode
	GetFirstInEdges() [][]int32
	GetFirstOutEdges() [][]int32
	AddEdge(newEdge datastructure.Edge)
	SetEdgeInfo(edgeInfo datastructure.EdgeExtraInfo)
	RemoveEdge(edgeID int32, fromNodeID int32, toNodeID int32)
	AddNode(node datastructure.CHNode)

	SnapLocationToRoadSegmentNodeH3(ways []datastructure.KVEdge, wantToSnap []float64) int32

	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32

	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.Edge
	GetInEdge(edgeID int32) datastructure.Edge
	IsRoundabout(edgeID int32) bool

	GetOutEdgesLen() int

	GetEdgeInfo(edgeID int32) datastructure.EdgeExtraInfo

	IsShortcut(edgeID int32) bool
	GetEdgePointsInBetween(edgeID int32) []datastructure.Coordinate
	IsTrafficLight(nodeID int32) bool

	SaveToFile(mapmatch bool) error
	LoadGraph(mapmatch bool) error
	GetStreetDirection(streetName int) [2]bool

	GetStreetNameFromID(streetName int) string
	GetRoadClassFromID(roadClass uint8) string
	GetRoadClassLinkFromID(roadClassLink uint8) string
	SetPointsInBetween(edgeID int32, points []datastructure.Coordinate) (int, int)
}

type RouteAlgorithm interface {
	ShortestPathBiDijkstraCH(from, to int32) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64)
	ShortestPathBiDijkstra(from, to int32, fromEdgeFilter, toEdgeFilter func(edge datastructure.Edge) bool) ([]datastructure.Coordinate, []datastructure.Edge,
		float64, float64)
}
type kvdb interface {
	GetNearestRoadSegmentsFromPointCoord(lat, lon float64) ([]datastructure.KVEdge, error)
}
