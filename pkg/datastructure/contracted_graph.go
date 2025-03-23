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
	StreetName       int
	RoadClass        int
	RoadClassLink    int
	Lanes            int
	PointsInBetween  []Coordinate
	Destination      string
	DestinationRef   string
	MotorwayJunction string
	Roundabout       bool
	IsShortcut       bool
}

func NewEdgeExtraInfo(streetName, roadClass, lanes, roadClassLink int, pointsInBetween []Coordinate, roundabout bool) EdgeExtraInfo {
	return EdgeExtraInfo{
		StreetName:      streetName,
		RoadClass:       roadClass,
		PointsInBetween: pointsInBetween,
		RoadClassLink:   roadClassLink,
		Lanes:           lanes,
		Roundabout:      roundabout,
		IsShortcut:      false,
	}
}
