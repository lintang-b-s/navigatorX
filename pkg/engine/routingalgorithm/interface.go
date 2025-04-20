package routingalgorithm

import "github.com/lintang-b-s/navigatorx/pkg/datastructure"

type ContractedGraph interface {
	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32

	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.Edge
	GetInEdge(edgeID int32) datastructure.Edge

	IsShortcut(edgeID int32) bool
	GetEdgePointsInBetween(edgeID int32) []datastructure.Coordinate
	IsTrafficLight(nodeID int32) bool

	GetStreetDirection(streetName int) [2]bool
	GetStreetNameFromID(streetName int) string
	GetRoadClassFromID(roadClass uint8) string
	GetRoadClassLinkFromID(roadClassLink uint8) string

	IsRoundabout(edgeID int32) bool

	GetEdgeInfo(edgeID int32) datastructure.EdgeExtraInfo
	GetNodesLen() int32
}
