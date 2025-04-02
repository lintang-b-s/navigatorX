package datastructure

import (
	"encoding/binary"
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/util"
)

// contracted graph
type CHNode struct {
	Lat      float64
	Lon      float64
	OrderPos int32
	ID       int32
}

func NewCHNode(lat, lon float64, orderPos int32, idx int32) CHNode {
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
		TrafficLight: make(map[int32]bool),
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
	EdgeID     int32
	Weight     float64 // minute
	Dist       float64 // meter
	ToNodeID   int32
	FromNodeID int32

	ViaNodeID int32
}

func NewEdgeCH(edgeID int32, weight, dist float64, toNodeID, fromNodeID int32, viaNodeID int32) EdgeCH {
	return EdgeCH{
		EdgeID:     edgeID,
		Weight:     weight,
		Dist:       dist,
		ToNodeID:   toNodeID,
		FromNodeID: fromNodeID,
		ViaNodeID:  viaNodeID,
	}
}

func NewEdgeCHPlain(edgeID int32, weight, dist float64, toNodeID, fromNodeID int32,
) EdgeCH {
	return EdgeCH{
		EdgeID:     edgeID,
		Weight:     weight,
		Dist:       dist,
		ToNodeID:   toNodeID,
		FromNodeID: fromNodeID,
		ViaNodeID:  -1,
	}
}

func (e *EdgeCH) Serialize() []byte {
	// 4byte*5 + 8byte*2 = 36byte

	buf := make([]byte, 36)

	// edgeID
	binary.LittleEndian.PutUint32(buf[0:4], uint32(e.EdgeID))
	// weight
	binary.LittleEndian.PutUint64(buf[4:12], math.Float64bits(e.Weight))
	// dist
	binary.LittleEndian.PutUint64(buf[12:20], math.Float64bits(e.Dist))
	// toNodeID
	binary.LittleEndian.PutUint32(buf[20:24], uint32(e.ToNodeID))
	// fromNodeID
	binary.LittleEndian.PutUint32(buf[24:28], uint32(e.FromNodeID))

	binary.LittleEndian.PutUint32(buf[28:32], uint32(e.ViaNodeID))

	return buf
}

func DeserializeEdgeCH(buf []byte) EdgeCH {
	// 4byte*5 + 8byte*2 = 36byte

	edgeID := int32(binary.LittleEndian.Uint32(buf[0:4]))
	weight := math.Float64frombits(binary.LittleEndian.Uint64(buf[4:12]))
	dist := math.Float64frombits(binary.LittleEndian.Uint64(buf[12:20]))
	toNodeID := int32(binary.LittleEndian.Uint32(buf[20:24]))
	fromNodeID := int32(binary.LittleEndian.Uint32(buf[24:28]))

	viaNodeID := int32(binary.LittleEndian.Uint32(buf[28:32]))

	return NewEdgeCH(edgeID, weight, dist, toNodeID, fromNodeID, viaNodeID)
}

// EdgeExtraInfo is a struct that contains information about each edge in the graph
// {from -> to -> edgeInfo (streetName, roadClass, lanes, roadClassLink, pointsInBetween, roundabout, isShortcut)}
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

/*
edgeIInfo:

	| StreetName | RoadClass | RoadClassLink | Lanes | Roundabout | IsShourtcut | PointsInBetween |
		4 byte		4 byte			4 byte		4 byte	1 byte		 1 byte			dynamic

	use bitpacking for edgeInfo
	https://wiki.openstreetmap.org/wiki/Map_features

	maxRoadclass is 120 so its 7 bits  (2^7-1=127)
	maxRoadClassLink is 5 so its 3 bits
	maxLanes is just 8 so its 4 bits
	roundabout is 1 bit
	shortcut is 1 bit

	bitpackedEdgeInfoField: (RoadClass, RoadClassLink, Lanes, Roundabout, IsShortcut)


	bitpackedEdgeInfo:
	| Streetname | bitpackedEdgeInfoField | PointsInBetween |
		4 byte			4 byte				dynamic
*/
func (ex *EdgeExtraInfo) Serialize() ([]byte, error) {

	buf := make([]byte, 8)

	// streetName
	binary.LittleEndian.PutUint32(buf[0:4], uint32(ex.StreetName))

	bitpackedEdgeInfoField := int32(ex.RoadClass)
	bitpackedEdgeInfoField = util.BitPackInt(bitpackedEdgeInfoField, int32(ex.RoadClassLink), 8)
	bitpackedEdgeInfoField = util.BitPackInt(bitpackedEdgeInfoField, int32(ex.Lanes), 14)
	bitpackedEdgeInfoField = util.BitPackIntBool(bitpackedEdgeInfoField, ex.Roundabout, 20)
	bitpackedEdgeInfoField = util.BitPackIntBool(bitpackedEdgeInfoField, ex.IsShortcut, 21)

	binary.LittleEndian.PutUint32(buf[4:8], uint32(bitpackedEdgeInfoField))

	coordBuf, err := serializeCoordinates(ex.PointsInBetween)
	if err != nil {
		return nil, err
	}

	buf = append(buf, coordBuf...)

	return buf, nil
}

func DeserializeEdgeExtraInfo(buf []byte) (EdgeExtraInfo, error) {

	streetName := int(binary.LittleEndian.Uint32(buf[0:4]))

	bitpackedEdgeInfoField := int32(binary.LittleEndian.Uint32(buf[4:8]))

	bitpackedEdgeInfoField, isShortcut := util.BitUnpackIntBool(bitpackedEdgeInfoField, 21)
	bitpackedEdgeInfoField, roundabout := util.BitUnpackIntBool(bitpackedEdgeInfoField, 20)
	bitpackedEdgeInfoField, lanes := util.BitUnpackInt(bitpackedEdgeInfoField, 14)
	bitpackedEdgeInfoField, roadClassLink := util.BitUnpackInt(bitpackedEdgeInfoField, 8)
	roadClass := int(bitpackedEdgeInfoField)

	coordBuf := buf[8:]
	pointsInBetween, err := deserializeCoordinates(coordBuf)

	return NewEdgeExtraInfo(streetName, roadClass,
		int(lanes), int(roadClassLink), pointsInBetween, roundabout, isShortcut), err
}

type MapEdgeInfo map[int32]map[int32]EdgeExtraInfo

func NewMapEdgeInfo() MapEdgeInfo {
	return make(map[int32]map[int32]EdgeExtraInfo)
}

func (ex *MapEdgeInfo) GetEdgeInfo(fromNodeID, toNodeID int32, reverse bool) EdgeExtraInfo {

	if !reverse {
		return (*ex)[fromNodeID][toNodeID]
	}
	return (*ex)[toNodeID][fromNodeID]

}

func (ex *EdgeExtraInfo) GetEdgeInfoSize() (int, error) {
	size := 4 * 2 // 2 int32 (bitpackedField  & streetname)
	// slice of Coordinate
	polylineBuf, err := serializeCoordinates(ex.PointsInBetween)
	polylineSize := len(polylineBuf)
	size += polylineSize

	return size, err
}

func (ex *MapEdgeInfo) AppendEdgeInfo(from, to int32, shortcut bool) {
	if _, ok := (*ex)[from]; !ok {
		(*ex)[from] = make(map[int32]EdgeExtraInfo)
	}
	(*ex)[from][to] = NewEdgeExtraInfo(0, 0, 0, 0, nil, false, shortcut)
}

func NewEdgeEdgeInfo(streetName int, roadClass int, lanes int, roadClassLink int, pointsInBetween []Coordinate, roundabout bool, shortcut bool) EdgeExtraInfo {
	return EdgeExtraInfo{
		StreetName:      streetName,
		RoadClass:       roadClass,
		RoadClassLink:   roadClassLink,
		Lanes:           lanes,
		PointsInBetween: pointsInBetween,
		Roundabout:      roundabout,
		IsShortcut:      shortcut,
	}
}
