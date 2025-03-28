package datastructure

// contracted graph
type CHNode struct {
	Lat      float64
	Lon      float64
	OrderPos int64
	ID       int32
}

func NewCHNode(lat, lon float64, orderPos int64, idx int32) CHNode {
	return CHNode{
		Lat:      lat,
		Lon:      lon,
		OrderPos: orderPos,
		ID:       idx,
	}
}

type NodeInfo struct {
	TrafficLight map[int32]bool
}

func NewNodeInfo() *NodeInfo {
	return &NodeInfo{
		TrafficLight: make(map[int32]bool, 0),
	}
}

func (ni *NodeInfo) SetTrafficLight(nodeID int32) {
	ni.TrafficLight[nodeID] = true
}

func NewCHNodePlain(lat, lon float64, idx int32) CHNode {
	return CHNode{
		Lat: lat,
		Lon: lon,
		ID:  idx,
	}
}

type EdgeCH struct {
	EdgeID         int32
	Weight         float64 // minute
	Dist           float64 // meter
	ToNodeID       int32
	FromNodeID     int32
	RemovedEdgeOne int32
	RemovedEdgeTwo int32
}

func NewEdgeCH(edgeID int32, weight, dist float64, toNodeID, fromNodeID int32, removedEdgeOne, removedEdgeTwo int32) EdgeCH {
	return EdgeCH{
		EdgeID:         edgeID,
		Weight:         weight,
		Dist:           dist,
		ToNodeID:       toNodeID,
		FromNodeID:     fromNodeID,
		RemovedEdgeOne: removedEdgeOne,
		RemovedEdgeTwo: removedEdgeTwo,
	}
}

func NewEdgeCHPlain(edgeID int32, weight, dist float64, toNodeID, fromNodeID int32,
) EdgeCH {
	return EdgeCH{
		EdgeID:         edgeID,
		Weight:         weight,
		Dist:           dist,
		ToNodeID:       toNodeID,
		FromNodeID:     fromNodeID,
		RemovedEdgeOne: -1,
		RemovedEdgeTwo: -1,
	}
}

type EdgeExtraInfo struct {
	StreetName      int
	RoadClass       int
	RoadClassLink   int
	Lanes           int
	PointsInBetween []Coordinate
	Roundabout      bool
	IsShortcut      bool
}

func NewEdgeExtraInfo(streetName, roadClass, lanes, roadClassLink int, pointsInBetween []Coordinate, roundabout bool, shortcut bool) EdgeExtraInfo {
	return EdgeExtraInfo{
		StreetName:      streetName,
		RoadClass:       roadClass,
		PointsInBetween: pointsInBetween,
		RoadClassLink:   roadClassLink,
		Lanes:           lanes,
		Roundabout:      roundabout,
		IsShortcut:      shortcut,
	}
}

type EdgeInfo struct {
	StreetName      map[int32]int
	RoadClass       map[int32]int
	RoadClassLink   map[int32]int
	Lanes           map[int32]int
	PointsInBetween map[int32][]Coordinate
	Roundabout      map[int32]bool
	IsShortcut      map[int32]bool
}

func (ex *EdgeInfo) SetEdgeInfo(edgeID int32, streetName, roadClass, lanes, roadClassLink int, pointsInBetween []Coordinate, roundabout bool, shortcut bool) {
	ex.StreetName[edgeID] = streetName
	ex.RoadClass[edgeID] = roadClass
	ex.RoadClassLink[edgeID] = roadClassLink
	ex.Lanes[edgeID] = lanes
	ex.PointsInBetween[edgeID] = pointsInBetween
	if roundabout {
		ex.Roundabout[edgeID] = roundabout
	}
	if shortcut {
		ex.IsShortcut[edgeID] = shortcut
	}
}

func (ex *EdgeInfo) SetShortcut(edgeID int32) {
	ex.IsShortcut[edgeID] = true
}

func NewEdgeEdgeInfo() *EdgeInfo {
	return &EdgeInfo{
		StreetName:      make(map[int32]int, 0),
		RoadClass:       make(map[int32]int, 0),
		RoadClassLink:   make(map[int32]int, 0),
		Lanes:           make(map[int32]int, 0),
		PointsInBetween: make(map[int32][]Coordinate, 0),
		Roundabout:      make(map[int32]bool, 0),
		IsShortcut:      make(map[int32]bool, 0),
	}
}
