package service

import "github.com/lintang-b-s/navigatorx/pkg/datastructure"

type RouteAlgorithm interface {
	ShortestPathBiDijkstraCH(from, to int32) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64)
}

type KVDB interface {
	GetNearestRoadSegmentsFromPointCoord(lat, lon float64) ([]datastructure.KVEdge, error)
}

type Matching interface {
	MapMatch(gps []datastructure.StateObservationPair, nextStateID *int) ([]datastructure.Coordinate, []datastructure.Edge, []datastructure.Coordinate)
}

type RoadSnapper interface {
	SnapToRoads(p datastructure.Point) []datastructure.OSMObject
	SnapToRoadsWithinRadius(p datastructure.Point, radius float64, k int) []datastructure.OSMObject
}
type ContractedGraph interface {
	SnapLocationToRoadSegmentNodeH3(ways []datastructure.KVEdge, wantToSnap []float64) int32

	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32
	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.Edge
	GetInEdge(edgeID int32) datastructure.Edge
	GetOutEdges() []datastructure.Edge
	GetInEdges() []datastructure.Edge
	GetNumNodes() int
	Contraction() (err error)

	SaveToFile() error
	LoadGraph() error
	GetStreetDirection(streetName int) [2]bool
	GetStreetNameFromID(streetName int) string
	GetRoadClassFromID(roadClass uint8) string
	GetRoadClassLinkFromID(roadClassLink uint8) string
	GetEdgePointsInBetween(edgeID int32) []datastructure.Coordinate
}

type OnlineMapMatcher interface {
	OnlineMapMatch(currGps datastructure.StateObservationPair, nextStateID int) []datastructure.Coordinate
}
