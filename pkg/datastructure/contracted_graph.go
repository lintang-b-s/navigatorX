package datastructure

// contracted graph
type CHNode struct {
	Lat          float64
	Lon          float64
	OrderPos     int64
	ID           int32
	TrafficLight bool
}

func NewCHNode(lat, lon float64, orderPos int64, idx int32, trafficLight bool) CHNode {
	return CHNode{
		Lat:          lat,
		Lon:          lon,
		OrderPos:     orderPos,
		ID:           idx,
		TrafficLight: trafficLight,
	}
}

type EdgeCH struct {
	EdgeID          int32
	Weight          float64 // minute
	Dist            float64 // meter
	ToNodeID        int32
	FromNodeID      int32
	IsShortcut      bool
	RemovedEdgeOne  int32
	RemovedEdgeTwo  int32
	StreetName      int
	Roundabout      bool
	RoadClass       int
	RoadClassLink   int
	Lanes           int
	PointsInBetween []Coordinate
}

func NewEdgeCH(edgeID int32, weight, dist float64, toNodeID, fromNodeID int32,
	isShortcut bool, removedEdgeOne, removedEdgeTwo int32, streetName int, roundabout bool,
	roadClass, roadClassLink int, lanes int, pointsInBetween []Coordinate) EdgeCH {
	return EdgeCH{
		EdgeID:          edgeID,
		Weight:          weight,
		Dist:            dist,
		ToNodeID:        toNodeID,
		FromNodeID:      fromNodeID,
		IsShortcut:      isShortcut,
		RemovedEdgeOne:  removedEdgeOne,
		RemovedEdgeTwo:  removedEdgeTwo,
		StreetName:      streetName,
		Roundabout:      roundabout,
		RoadClass:       roadClass,
		RoadClassLink:   roadClassLink,
		Lanes:           lanes,
		PointsInBetween: pointsInBetween,
	}
}

type StreetExtraInfo struct {
	Destination      string
	DestinationRef   string
	MotorwayJunction string
}
