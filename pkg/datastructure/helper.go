package datastructure

import (
	"github.com/twpayne/go-polyline"
)

type SPSingleResultResult struct {
	Source   int32
	Dest     int32
	Paths    []Coordinate
	EdgePath []EdgeCH
	Dist     float64
	Eta      float64
}

type StateObservationPair struct {
	Observation CHNode
	State       []*State
}

type StatePosition int

const (
	// StatePosition is the position of the hidden state in a nearest road segment.
	// GraphNode if query point (observation/gps point) have distance below 3m to the graph node (Edge source node/target node).
	// VirtualNode if query point have distance higher than 3m to the graph node.
	GraphNode StatePosition = iota
	VirtualNode
)

/*
State. hidden state / road segment candidate for a gps point/observation.

Have virtual edge & virtual node between the road segment if the observation is not close enough to the graph node.
virtual node is the projection of the observation to the nearest road segment.
virtual edge is the edge between the virtual node and the nearest graph node or other virtual node (gps to road segment projection).

each virtual node must only have 2 virtual edges. incoming virtual edge and outgoing virtual edge.
that incoming virtual edge and outgoing virtual edge used to start the bidirectional dijkstra search.

ProjectionID can be graph node ID or virtual node ID.
*/

type State struct {
	StateID int

	Dist            float64
	EdgeID          int32
	PointsInBetween []Coordinate
	EdgeFromNodeID  int32
	EdgeToNodeID    int32
	EdgeWeight      float64
	ProjectionLoc   [2]float64
	ProjectionID    int32

	IncomingVirtualEdgeID int32
	OutgoingVirtualEdgeID int32
	Type                  StatePosition
	ObservationID         int
}

func (s *State) SetOutgoingVirtualEdge(id int32) {
	s.OutgoingVirtualEdgeID = id
}

func (s *State) SetIncomingVirtualEdge(id int32) {
	s.IncomingVirtualEdgeID = id
}

func (s *State) SetType(t StatePosition) {
	s.Type = t
}

func (s *State) GetType() StatePosition {
	return s.Type
}

func (s *State) ID() int {
	return s.StateID
}

type MapMatchOsmWay struct {
	ID               int
	PointsInBetween  []Coordinate
	NodeIDsInBetween []int32
	FromNodeID       int32
	ToNodeID         int32
	Speed            float64
	Dist             float64
	IsMotorway       bool
}

func NewMathMatchOsmWay(id int, points []Coordinate, nodeIDs []int32, from, to int32, speed, dist float64, isMotorway bool) MapMatchOsmWay {
	return MapMatchOsmWay{
		ID:               id,
		PointsInBetween:  points,
		NodeIDsInBetween: nodeIDs,
		FromNodeID:       from,
		ToNodeID:         to,
		Speed:            speed,
		Dist:             dist,
		IsMotorway:       isMotorway,
	}
}

type KVEdge struct {
	CenterLoc  [2]float64
	ToNodeID   int32
	FromNodeID int32
}

func CreatePolyline(path []Coordinate) string {
	s := ""
	coords := make([][]float64, 0)
	for _, p := range path {
		pT := p
		coords = append(coords, []float64{pT.Lat, pT.Lon})
	}
	s = string(polyline.EncodeCoords(coords))
	return s
}
