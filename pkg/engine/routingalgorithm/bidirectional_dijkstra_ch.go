package routingalgorithm

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"
)

type cameFromPair struct {
	Edge   datastructure.EdgeCH
	NodeID int32
}

type ContractedGraph interface {
	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32

	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.EdgeCH
	GetInEdge(edgeID int32) datastructure.EdgeCH
	GetNumNodes() int
	IsShortcut(fromNodeID, toNodeID int32, reverse bool) bool
	GetEdgePointsInBetween(fromNodeID, toNodeID int32, reverse bool) []datastructure.Coordinate
	IsTrafficLight(nodeID int32) bool

	// csr
	GetNodeOutEdgesCsr2(nodeID int32) ([]datastructure.EdgeCH, error)
	GetNodeInEdgesCsr2(nodeID int32) ([]datastructure.EdgeCH, error)
	IsShortcutCsr(fromNodeID, toNodeID int32, reverse bool) (bool, error)
	IsRoundaboutCsr(fromNodeID, toNodeID int32) (bool, error)
	GetEdgePointsInBetweenCsr(fromNodeID, toNodeID int32, reverse bool) ([]datastructure.Coordinate, error)

	GetStreetNameCsr(fromNodeID, toNodeID int32) (int, error)
	GetRoadClassCsr(nodeID int32, edgeID int32) (int, error)
	GetRoadClassLinkCsr(nodeID int32, edgeID int32) (int, error)
	GetLanesCsr(nodeID int32, edgeID int32) (int, error)
	IsTrafficLightCsr(nodeID int32) (bool, error)
}

type RouteAlgorithm struct {
	ch ContractedGraph
}

func NewRouteAlgorithm(ch ContractedGraph) *RouteAlgorithm {
	return &RouteAlgorithm{ch: ch}
}

func (rt *RouteAlgorithm) ShortestPathBiDijkstraCH(from, to int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64) {
	if from == to {
		return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, 0, 0
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

	visitedF := make(map[int32]struct{})
	visitedB := make(map[int32]struct{})

	estimate := math.MaxFloat64

	bestCommonVertex := int32(0)

	cameFromf := make(map[int32]cameFromPair)
	cameFromf[from] = cameFromPair{datastructure.EdgeCH{}, -1}

	cameFromb := make(map[int32]cameFromPair)
	cameFromb[to] = cameFromPair{datastructure.EdgeCH{}, -1}

	frontFinished := false
	backFinished := false

	frontier := forwQ
	otherFrontier := backQ
	turnF := true
	for {
		if frontier.Size() == 0 {
			frontFinished = true
		}
		if otherFrontier.Size() == 0 {
			backFinished = true
		}

		if frontFinished && backFinished {
			// stop pencarian jika kedua priority queue kosong
			break
		}

		ff := *frontier
		if ff.Size() == 0 {
			return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1
		}
		smallestFront, _ := ff.GetMin()
		if smallestFront.Rank >= estimate {
			// bidirectional search di stop ketika smallest node saat ini costnya >=  cost current best candidate path.
			if turnF {
				frontFinished = true
			} else {
				backFinished = true
			}
		} else {
			node, _ := frontier.ExtractMin()
			if node.Rank >= estimate {
				break
			}
			if turnF {

				for _, arc := range rt.ch.GetNodeFirstOutEdges(node.Item) {

					edge := rt.ch.GetOutEdge(arc)

					if _, ok := visitedF[edge.ToNodeID]; ok {
						continue
					}

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

						_, ok = db[toNID]
						if ok {
							pathDistance := newCost + db[toNID]
							if pathDistance < estimate {
								// jika toNID visited di backward search & d(s,toNID) + d(t,toNID) < cost best candidate path, maka update best candidate path
								estimate = pathDistance
								bestCommonVertex = edge.ToNodeID

							}
						}
					}
				}

				visitedF[node.Item] = struct{}{}

			} else {

				for _, arc := range rt.ch.GetNodeFirstInEdges(node.Item) {

					edge := rt.ch.GetInEdge(arc)

					if _, ok := visitedB[edge.ToNodeID]; ok {
						continue
					}

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

						_, ok = df[toNID]
						if ok {
							pathDistance := newCost + df[toNID]
							if pathDistance < estimate {
								estimate = pathDistance
								bestCommonVertex = edge.ToNodeID

							}
						}
					}
				}

				visitedB[node.Item] = struct{}{}

			}

		}

		otherFinished := false

		if turnF {
			if backFinished {
				otherFinished = true
			}
		} else {
			if frontFinished {
				otherFinished = true
			}

		}
		if !otherFinished {
			tmpFrontier := frontier
			frontier = otherFrontier
			otherFrontier = tmpFrontier
			turnF = !turnF
		}
	}

	if estimate == math.MaxFloat64 {
		return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1
	}
	// estimate dari bidirectional dijkstra pake shortcut edge jadi lebih cepet eta nya & gak akurat
	path, edgePath, eta, dist := rt.createPath(bestCommonVertex, from, to, cameFromf, cameFromb)
	return path, edgePath, eta, dist
}

func (rt *RouteAlgorithm) createPath(commonVertex int32, from, to int32,
	cameFromf, cameFromb map[int32]cameFromPair) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64) {

	fPath := make([]datastructure.Coordinate, 0)
	fedgePath := make([]datastructure.EdgeCH, 0)
	eta := 0.0
	dist := 0.0
	v := commonVertex
	if rt.ch.IsTrafficLight(v) {
		eta += 3.0
	}
	ok := true
	for ok && v != -1 {
		isShortcut := rt.ch.IsShortcut(cameFromf[v].Edge.FromNodeID, cameFromf[v].Edge.ToNodeID, false)

		if isShortcut {

			rt.unpackBackward(cameFromf[v].Edge, &fPath, &fedgePath, &eta, &dist)
		} else {

			if cameFromf[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromf[v].NodeID) {
				eta += 3.0
			}
			eta += cameFromf[v].Edge.Weight
			dist += cameFromf[v].Edge.Dist

			pointsInBetween := make([]datastructure.Coordinate, 0)
			if cameFromf[v].Edge.Weight != 0 {
				fedgePath = append(fedgePath, cameFromf[v].Edge)
				pointsInBetween = rt.ch.GetEdgePointsInBetween(cameFromf[v].Edge.FromNodeID, cameFromf[v].Edge.ToNodeID, false)

				pointsInBetween = util.ReverseG(pointsInBetween)
			}

			nodeV := rt.ch.GetNode(v)

			fPath = append(fPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))
			fPath = append(fPath, pointsInBetween...)
		}
		_, ok = cameFromf[v]
		v = cameFromf[v].NodeID

	}

	bPath := make([]datastructure.Coordinate, 0)
	bEdgePath := make([]datastructure.EdgeCH, 0)
	v = commonVertex
	ok = true
	for ok && v != -1 {

		isShortcut := rt.ch.IsShortcut(cameFromb[v].Edge.FromNodeID, cameFromb[v].Edge.ToNodeID, true)
		if isShortcut {

			rt.unpackForward(cameFromb[v].Edge, &bPath, &bEdgePath, &eta, &dist)

		} else {

			if cameFromb[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromb[v].NodeID) {
				eta += 3.0
			}
			eta += cameFromb[v].Edge.Weight
			dist += cameFromb[v].Edge.Dist

			pointsInBetween := make([]datastructure.Coordinate, 0)
			if cameFromb[v].Edge.Weight != 0 {
				bEdgePath = append(bEdgePath, cameFromb[v].Edge)
				pointsInBetween = rt.ch.GetEdgePointsInBetween(cameFromb[v].Edge.FromNodeID, cameFromb[v].Edge.ToNodeID, true)
			}

			nodeV := rt.ch.GetNode(v)

			bPath = append(bPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))
			bPath = append(bPath, pointsInBetween...)

		}
		_, ok = cameFromb[v]
		v = cameFromb[v].NodeID
	}

	fPath = util.ReverseG(fPath)
	fPath = fPath[:len(fPath)-1]
	path := make([]datastructure.Coordinate, 0)
	path = append(path, fPath...)
	path = append(path, bPath...)

	edgePath := make([]datastructure.EdgeCH, 0)
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
func (rt *RouteAlgorithm) unpackBackward(edge datastructure.EdgeCH, path *[]datastructure.Coordinate, ePath *[]datastructure.EdgeCH,
	eta, dist *float64) {
	isShortcut := rt.ch.IsShortcut(edge.FromNodeID, edge.ToNodeID, false)

	if !isShortcut {
		if rt.ch.IsTrafficLight(edge.FromNodeID) {
			*eta += 3.0
		}
		*eta += edge.Weight
		*dist += edge.Dist

		pointsInBetween := rt.ch.GetEdgePointsInBetween(edge.FromNodeID, edge.ToNodeID, false)

		pointsInBetween = util.ReverseG(pointsInBetween)

		fromNode := rt.ch.GetNode(edge.FromNodeID)

		*path = append(*path, pointsInBetween...)
		*path = append(*path, datastructure.NewCoordinate(fromNode.Lat, fromNode.Lon))

		*ePath = append(*ePath, edge)
	} else {
		var (
			edgeTwo datastructure.EdgeCH
			edgeOne datastructure.EdgeCH
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
func (rt *RouteAlgorithm) unpackForward(edge datastructure.EdgeCH, path *[]datastructure.Coordinate, ePath *[]datastructure.EdgeCH,
	eta, dist *float64) {
	isShortcut := rt.ch.IsShortcut(edge.FromNodeID, edge.ToNodeID, true)

	if !isShortcut {
		if rt.ch.IsTrafficLight(edge.ToNodeID) {
			*eta += 3.0
		}
		*eta += edge.Weight
		*dist += edge.Dist

		pointsInBetween := rt.ch.GetEdgePointsInBetween(edge.FromNodeID, edge.ToNodeID, true)
		toNode := rt.ch.GetNode(edge.ToNodeID)

		*path = append(*path, datastructure.NewCoordinate(toNode.Lat, toNode.Lon))
		*path = append(*path, pointsInBetween...)

		*ePath = append(*ePath, edge)
	} else {

		var (
			edgeTwo datastructure.EdgeCH
			edgeOne datastructure.EdgeCH
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
