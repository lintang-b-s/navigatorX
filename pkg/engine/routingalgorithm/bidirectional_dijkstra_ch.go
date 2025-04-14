package routingalgorithm

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"
)

const (
	trafficLightAdditionalWeight = 1.2
)

type cameFromPair struct {
	Edge   datastructure.Edge
	NodeID int32
}

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
}

type RouteAlgorithm struct {
	ch ContractedGraph
}

func NewRouteAlgorithm(ch ContractedGraph) *RouteAlgorithm {
	return &RouteAlgorithm{ch: ch}
}

func (rt *RouteAlgorithm) ShortestPathBiDijkstraCH(from, to int32) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64) {
	if from == to {
		return []datastructure.Coordinate{}, []datastructure.Edge{}, 0, 0
	}
	forwQ := contractor.NewMinHeap[int32]()
	backQ := contractor.NewMinHeap[int32]()

	df := make(map[int32]float64)
	db := make(map[int32]float64)
	df[from] = 0.0
	db[to] = 0.0

	fromNode := contractor.PriorityQueueNode[int32]{Rank: 0, Item: from}
	toNode := contractor.PriorityQueueNode[int32]{Rank: 0, Item: to}

	forwQ.Insert(fromNode)
	backQ.Insert(toNode)

	estimate := math.MaxFloat64

	bestCommonVertex := int32(0)

	cameFromf := make(map[int32]cameFromPair)
	cameFromf[from] = cameFromPair{datastructure.Edge{}, -1}

	cameFromb := make(map[int32]cameFromPair)
	cameFromb[to] = cameFromPair{datastructure.Edge{}, -1}

	forwardProcessed := make(map[int32]struct{})
	backwardProcessed := make(map[int32]struct{})

	isForward := true

	// https://publikationen.bibliothek.kit.edu/1000028701/142973925 (algorithm 1)
	for (forwQ.Size() != 0 || backQ.Size() != 0) && estimate > min(forwQ.GetMinRank(), backQ.GetMinRank()) {

		if isForward {
			if backQ.Size() != 0 {
				isForward = false
			}
		} else {
			if forwQ.Size() != 0 {
				isForward = true
			}
		}

		if isForward {

			rt.search(forwQ, df, db, cameFromf, cameFromb, isForward,
				forwardProcessed, backwardProcessed, &estimate, &bestCommonVertex)
		} else {

			rt.search(backQ, df, db, cameFromf, cameFromb, isForward,
				forwardProcessed, backwardProcessed, &estimate, &bestCommonVertex)
		}
	}

	if estimate == math.MaxFloat64 {
		return []datastructure.Coordinate{}, []datastructure.Edge{}, -1, -1
	}

	path, edgePath, eta, dist := rt.createPath(bestCommonVertex, from, to, cameFromf, cameFromb)
	return path, edgePath, eta, dist
}

func (rt *RouteAlgorithm) search(frontier *contractor.MinHeap[int32], df, db map[int32]float64,
	cameFromf, cameFromb map[int32]cameFromPair, turnF bool, forwardProcessed, backwardProcessed map[int32]struct{},
	estimate *float64, bestCommonVertex *int32) {
	node, _ := frontier.ExtractMin()
	if turnF {

		for _, arc := range rt.ch.GetNodeFirstOutEdges(node.Item) {

			edge := rt.ch.GetOutEdge(arc)

			toNID := edge.ToNodeID
			cost := edge.Weight

			if rt.ch.GetNode(node.Item).OrderPos < rt.ch.GetNode(toNID).OrderPos {
				// upward graph
				newCost := cost + df[node.Item]
				_, ok := df[toNID]
				// relax edge
				if !ok {
					df[toNID] = newCost

					neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
					frontier.Insert(neighborNode)
					cameFromf[toNID] = cameFromPair{edge, node.Item}
				} else if newCost < df[toNID] {
					df[toNID] = newCost

					neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
					frontier.DecreaseKey(neighborNode)

					cameFromf[toNID] = cameFromPair{edge, node.Item}
				}

			}
		}

		forwardProcessed[node.Item] = struct{}{}

		_, ok := backwardProcessed[node.Item]
		if ok {
			pathDistance := df[node.Item] + db[node.Item]
			if pathDistance < *estimate {
				*estimate = pathDistance
				*bestCommonVertex = node.Item
			}
		}

	} else {

		for _, arc := range rt.ch.GetNodeFirstInEdges(node.Item) {

			edge := rt.ch.GetInEdge(arc)

			toNID := edge.ToNodeID
			cost := edge.Weight

			if rt.ch.GetNode(node.Item).OrderPos < rt.ch.GetNode(toNID).OrderPos {
				// downward graph
				newCost := cost + db[node.Item]
				_, ok := db[toNID]
				if !ok {
					db[toNID] = newCost

					neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
					frontier.Insert(neighborNode)
					cameFromb[toNID] = cameFromPair{edge, node.Item}
				}
				if newCost < db[toNID] {
					db[toNID] = newCost

					neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
					frontier.DecreaseKey(neighborNode)

					cameFromb[toNID] = cameFromPair{edge, node.Item}
				}

			}
		}

		backwardProcessed[node.Item] = struct{}{}
		_, ok := forwardProcessed[node.Item]
		if ok {
			pathDistance := df[node.Item] + db[node.Item]
			if pathDistance < *estimate {
				*estimate = pathDistance
				*bestCommonVertex = node.Item
			}
		}
	}
}

func min(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}

func (rt *RouteAlgorithm) createPath(commonVertex int32, from, to int32,
	cameFromf, cameFromb map[int32]cameFromPair) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64) {

	fPath := make([]datastructure.Coordinate, 0)
	fedgePath := make([]datastructure.Edge, 0)
	eta := 0.0
	dist := 0.0
	v := commonVertex
	if rt.ch.IsTrafficLight(v) {
		eta += trafficLightAdditionalWeight
	}
	ok := true
	for ok && cameFromf[v].NodeID != -1 {
		isShortcut := rt.ch.IsShortcut(cameFromf[v].Edge.EdgeID)

		if isShortcut {

			rt.unpackBackward(cameFromf[v].Edge, &fPath, &fedgePath, &eta, &dist)
		} else {

			if cameFromf[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromf[v].NodeID) {
				eta += trafficLightAdditionalWeight
			}
			eta += cameFromf[v].Edge.Weight
			dist += cameFromf[v].Edge.Dist

			pointsInBetween := make([]datastructure.Coordinate, 0)
			if cameFromf[v].Edge.Weight != 0 {
				fedgePath = append(fedgePath, cameFromf[v].Edge)
				pointsInBetween = rt.ch.GetEdgePointsInBetween(cameFromf[v].Edge.EdgeID)

				pointsInBetween = util.ReverseG(pointsInBetween)
			}

			nodeV := rt.ch.GetNode(cameFromf[v].NodeID)

			fPath = append(fPath, pointsInBetween...)
			fPath = append(fPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))

		}
		_, ok = cameFromf[v]
		v = cameFromf[v].NodeID

	}

	bPath := make([]datastructure.Coordinate, 0)
	bEdgePath := make([]datastructure.Edge, 0)
	v = commonVertex
	ok = true
	for ok && cameFromb[v].NodeID != -1 {

		isShortcut := rt.ch.IsShortcut(cameFromb[v].Edge.EdgeID)
		if isShortcut {

			rt.unpackForward(cameFromb[v].Edge, &bPath, &bEdgePath, &eta, &dist)

		} else {

			if cameFromb[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromb[v].NodeID) {
				eta += trafficLightAdditionalWeight
			}
			eta += cameFromb[v].Edge.Weight
			dist += cameFromb[v].Edge.Dist

			pointsInBetween := make([]datastructure.Coordinate, 0)
			if cameFromb[v].Edge.Weight != 0 {
				bEdgePath = append(bEdgePath, cameFromb[v].Edge)
				pointsInBetween = rt.ch.GetEdgePointsInBetween(cameFromb[v].Edge.EdgeID)
			}

			nodeV := rt.ch.GetNode(cameFromb[v].NodeID)

			bPath = append(bPath, pointsInBetween...)
			bPath = append(bPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))

		}
		_, ok = cameFromb[v]
		v = cameFromb[v].NodeID
	}

	fPath = util.ReverseG(fPath)

	path := make([]datastructure.Coordinate, 0)
	path = append(path, fPath...)

	vianode := rt.ch.GetNode(commonVertex)
	path = append(path, datastructure.NewCoordinate(vianode.Lat, vianode.Lon))
	path = append(path, bPath...)

	edgePath := make([]datastructure.Edge, 0)
	fedgePath = util.ReverseG(fedgePath)

	for i := 0; i < len(bEdgePath); i++ {
		curr := bEdgePath[i]
		// harus dibalik buat backward edge path nya
		// karena base node arah dari target ke common vertex, sedangkan di driving instruction butuhnya dari common ke target
		toNodeID := curr.FromNodeID
		fromNodeID := curr.ToNodeID
		bEdgePath[i].FromNodeID = fromNodeID
		bEdgePath[i].ToNodeID = toNodeID
	}
	edgePath = append(edgePath, fedgePath...)

	edgePath = append(edgePath, bEdgePath...)

	return path, edgePath, eta, dist / 1000
}

// buat forward dijkstra
// dari common vertex ke source vertex
func (rt *RouteAlgorithm) unpackBackward(edge datastructure.Edge, path *[]datastructure.Coordinate, ePath *[]datastructure.Edge,
	eta, dist *float64) {

	isShortcut := rt.ch.IsShortcut(edge.EdgeID)

	if !isShortcut {
		if rt.ch.IsTrafficLight(edge.FromNodeID) {
			*eta += trafficLightAdditionalWeight
		}
		*eta += edge.Weight
		*dist += edge.Dist

		pointsInBetween := rt.ch.GetEdgePointsInBetween(edge.EdgeID)

		pointsInBetween = util.ReverseG(pointsInBetween)

		fromNode := rt.ch.GetNode(edge.FromNodeID)

		*path = append(*path, pointsInBetween...)
		*path = append(*path, datastructure.NewCoordinate(fromNode.Lat, fromNode.Lon))

		*ePath = append(*ePath, edge)
	} else {
		var (
			edgeTwo datastructure.Edge
			edgeOne datastructure.Edge
		)

		fromNodeID := edge.FromNodeID
		toNodeID := edge.ToNodeID

		for _, arcID := range rt.ch.GetNodeFirstOutEdges(fromNodeID) {
			arc := rt.ch.GetOutEdge(arcID)
			if arc.ToNodeID == edge.ViaNodeID {
				edgeOne = arc
			}
		}

		for _, arcID := range rt.ch.GetNodeFirstOutEdges(edge.ViaNodeID) {
			arc := rt.ch.GetOutEdge(arcID)
			if arc.ToNodeID == toNodeID {
				edgeTwo = arc
			}
		}

		rt.unpackBackward(edgeTwo, path, ePath, eta, dist)
		rt.unpackBackward(edgeOne, path, ePath, eta, dist)
	}
}

// dari common vertex ke target vertex
func (rt *RouteAlgorithm) unpackForward(edge datastructure.Edge, path *[]datastructure.Coordinate, ePath *[]datastructure.Edge,
	eta, dist *float64) {

	isShortcut := rt.ch.IsShortcut(edge.EdgeID)

	if !isShortcut {
		if rt.ch.IsTrafficLight(edge.FromNodeID) {
			*eta += trafficLightAdditionalWeight
		}
		*eta += edge.Weight
		*dist += edge.Dist

		pointsInBetween := rt.ch.GetEdgePointsInBetween(edge.EdgeID)
		fromNode := rt.ch.GetNode(edge.FromNodeID)

		*path = append(*path, pointsInBetween...)

		*path = append(*path, datastructure.NewCoordinate(fromNode.Lat, fromNode.Lon))

		*ePath = append(*ePath, edge)
	} else {

		var (
			edgeTwo datastructure.Edge
			edgeOne datastructure.Edge
		)

		fromNodeID := edge.FromNodeID
		toNodeID := edge.ToNodeID

		for _, arcID := range rt.ch.GetNodeFirstInEdges(edge.ViaNodeID) {
			arc := rt.ch.GetInEdge(arcID)
			if arc.ToNodeID == toNodeID {
				edgeOne = arc
			}
		}

		for _, arcID := range rt.ch.GetNodeFirstInEdges(fromNodeID) {
			arc := rt.ch.GetInEdge(arcID)
			if arc.ToNodeID == edge.ViaNodeID {
				edgeTwo = arc
			}
		}

		rt.unpackForward(edgeOne, path, ePath, eta, dist)
		rt.unpackForward(edgeTwo, path, ePath, eta, dist)

	}
}

func (rt *RouteAlgorithm) GetGraph() ContractedGraph {
	return rt.ch
}
