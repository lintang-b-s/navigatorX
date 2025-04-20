package datastructure

import (
	"math"
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

func NewCHNodePlain(lat, lon float64, idx int32) CHNode {
	return CHNode{
		Lat: lat,
		Lon: lon,
		ID:  idx,
	}
}

type GraphStorage struct {
	GlobalPoints []Coordinate
	EdgeStorage  []Edge // main info (weight, dist, edgeID, toNodeID, fromNodeID, viaNodeID)

	/*
		32 bit -> 32 boolean flag for roundabout & trafficlight

		idx in flag array = floor(edgeID/32)
		idx in flag = edgeID % 32
	*/
	RoundaboutFlag   []int32
	NodeTrafficLight []int32

	StartShortcutID int32

	MapEdgeInfo []EdgeExtraInfo
}

func NewGraphStorage() *GraphStorage {
	return &GraphStorage{}
}

type Edge struct {
	Weight     float64 // minute
	Dist       float64 // meter
	EdgeID     int32
	ToNodeID   int32
	FromNodeID int32
	ViaNodeID  int32
}

func NewEdge(edgeID, toNodeID, fromNodeID, viaNodeID int32, weight, dist float64) Edge {
	return Edge{
		Weight:     weight,
		Dist:       dist,
		EdgeID:     edgeID,
		ToNodeID:   toNodeID,
		FromNodeID: fromNodeID,
		ViaNodeID:  viaNodeID,
	}
}

func (e *Edge) GetEdgeSpeed() float64 {

	return e.Dist / e.Weight
}

func NewEdgePlain(edgeID int32, weight, dist float64, toNodeID, fromNodeID int32) Edge {
	return Edge{
		EdgeID:     edgeID,
		ToNodeID:   toNodeID,
		FromNodeID: fromNodeID,
		Weight:     weight,
		Dist:       dist,
	}
}

func (gs *GraphStorage) SetRoundabout(edgeID int32, isRoundabout bool) {
	index := int(math.Floor(float64(edgeID) / 32))
	if len(gs.RoundaboutFlag) <= int(index) {
		gs.RoundaboutFlag = append(gs.RoundaboutFlag, make([]int32, edgeID-int32(len(gs.RoundaboutFlag))+1)...)
	}
	if isRoundabout {
		gs.RoundaboutFlag[index] |= 1 << (edgeID % 32)
	}
}

func (gs *GraphStorage) SetTrafficLight(nodeID int32) {
	index := int(math.Floor(float64(nodeID) / 32))

	if len(gs.NodeTrafficLight) <= int(index) {
		gs.NodeTrafficLight = append(gs.NodeTrafficLight, make([]int32, nodeID-int32(len(gs.NodeTrafficLight))+1)...)
	}

	gs.NodeTrafficLight[index] |= 1 << (nodeID % 32)
}

func (gs *GraphStorage) GetTrafficLight(nodeID int32) bool {
	index := int(math.Floor(float64(nodeID) / 32))

	return (gs.NodeTrafficLight[index] & (1 << (nodeID % 32))) != 0
}

func (gs *GraphStorage) GetEdgeInfo(edgeID int32, reverse bool) Edge {
	if !reverse {
		return gs.EdgeStorage[edgeID]
	}

	edge := gs.EdgeStorage[edgeID]
	edge.FromNodeID, edge.ToNodeID = edge.ToNodeID, edge.FromNodeID
	return edge
}

func (gs *GraphStorage) GetOutEdge(edgeID int32) Edge {
	return gs.GetEdgeInfo(edgeID, false)
}

func (gs *GraphStorage) GetInEdge(edgeID int32) Edge {
	return gs.GetEdgeInfo(edgeID, true)
}

func (gs *GraphStorage) UpdateEdge(edgeID int32, weight, dist float64, vianode int32) {
	edge := gs.EdgeStorage[edgeID]
	edge.Weight = weight
	edge.Dist = dist
	edge.ViaNodeID = vianode
	gs.EdgeStorage[edgeID] = edge
}

type EdgeExtraInfo struct {
	StartPointsIndex uint32
	EndPointsIndex   uint32
	StreetName       int
	RoadClass        uint8
	RoadClassLink    uint8
	Lanes            uint8
}

func NewEdgeExtraInfo(streetName int, roadClass, lanes, roadClassLink uint8, StartPointsIdx, EndPointsIdx uint32) EdgeExtraInfo {
	return EdgeExtraInfo{
		StreetName:       streetName,
		RoadClass:        roadClass,
		RoadClassLink:    roadClassLink,
		Lanes:            lanes,
		StartPointsIndex: StartPointsIdx,
		EndPointsIndex:   EndPointsIdx,
	}
}

func (gs *GraphStorage) GetPointsInbetween(edgeID int32) []Coordinate {
	edge := gs.MapEdgeInfo[edgeID]
	var (
		edgePoints []Coordinate
	)
	startIndex := edge.StartPointsIndex
	endIndex := edge.EndPointsIndex
	if startIndex <= endIndex {
		edgePoints = gs.GlobalPoints[startIndex:endIndex]

		return edgePoints
	}

	for i := startIndex - 1; i >= endIndex; i-- {
		edgePoints = append(edgePoints, gs.GlobalPoints[i])
	}

	return edgePoints
}

func (gs *GraphStorage) IsShortcut(edgeID int32) bool {

	if edgeID < gs.StartShortcutID {
		return false
	}

	return true
}

// return edgeExtraInfo, isRoundabout
func (gs *GraphStorage) GetEdgeExtraInfo(edgeID int32, reverse bool) (EdgeExtraInfo, bool) {

	if edgeID < gs.StartShortcutID {
		index := int(math.Floor(float64(edgeID) / 32))
		roundabout := (gs.RoundaboutFlag[index] & (1 << (edgeID % 32))) != 0
		return gs.MapEdgeInfo[edgeID], roundabout
	}

	return gs.MapEdgeInfo[edgeID], false
}

func (gs *GraphStorage) UpdateEdgePoints(edgeID int32, startIdx, endIdx uint32) {
	edge := gs.MapEdgeInfo[edgeID]
	edge.StartPointsIndex = startIdx
	edge.EndPointsIndex = endIdx
	gs.MapEdgeInfo[edgeID] = edge
}

func (gs *GraphStorage) AppendGlobalPoints(edgePoints []Coordinate) {
	gs.GlobalPoints = append(gs.GlobalPoints, edgePoints...)
}

func (gs *GraphStorage) AppendMapEdgeInfo(edgeInfo EdgeExtraInfo) {
	gs.MapEdgeInfo = append(gs.MapEdgeInfo, edgeInfo)
}

func (gs *GraphStorage) AppendEdgeStorage(edgeInfo Edge) {
	gs.EdgeStorage = append(gs.EdgeStorage, edgeInfo)
}

func (gs *GraphStorage) SetStartShortcutID(edgeid int32) {
	gs.StartShortcutID = edgeid
}

func (gs *GraphStorage) GetOutEdges() []Edge {
	var edges []Edge
	for i := range gs.EdgeStorage {
		edges = append(edges, gs.EdgeStorage[i])

	}

	return edges
}

func (gs *GraphStorage) GetOutEdgesLen() int {
	return len(gs.EdgeStorage)
}
func (gs *GraphStorage) GetInEdges() []Edge {
	var edges []Edge
	for _, edge := range gs.EdgeStorage {
		edge.FromNodeID, edge.ToNodeID = edge.ToNodeID, edge.FromNodeID
		edges = append(edges, edge)
	}

	return edges
}
