package routingalgorithm

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"
)

type cameFromPairXCHV struct {
	Edge   datastructure.Edge
	NodeID int32
	Weight   float64
}

func (rt *RouteAlgorithm) ShortestPathBiDijkstraXCHV(from, to int32) ([]datastructure.CHNode,
	[]datastructure.Coordinate, []datastructure.Edge, float64, float64,
	map[int32]cameFromPairXCHV, map[int32]cameFromPairXCHV, int32) {
	if from == to {
		return []datastructure.CHNode{}, []datastructure.Coordinate{}, []datastructure.Edge{}, 0, 0, nil, nil, -1
	}
	forwQ := contractor.NewMinHeap[int32]()
	backQ := contractor.NewMinHeap[int32]()

	df := make(map[int32]float64)
	db := make(map[int32]float64)
	df[from] = 0.0
	db[to] = 0.0

	distf := make(map[int32]float64)
	distb := make(map[int32]float64)
	distf[from] = 0.0
	distb[to] = 0.0

	fromNode := contractor.PriorityQueueNode[int32]{Rank: 0, Item: from}
	toNode := contractor.PriorityQueueNode[int32]{Rank: 0, Item: to}

	forwQ.Insert(fromNode)
	backQ.Insert(toNode)

	visitedF := make(map[int32]struct{})
	visitedB := make(map[int32]struct{})

	estimate := math.MaxFloat64

	bestCommonVertex := int32(0)

	cameFromf := make(map[int32]cameFromPairXCHV)
	cameFromf[from] = cameFromPairXCHV{datastructure.Edge{}, -1, 0}

	cameFromb := make(map[int32]cameFromPairXCHV)
	cameFromb[to] = cameFromPairXCHV{datastructure.Edge{}, -1, 0}

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
			return []datastructure.CHNode{}, []datastructure.Coordinate{}, []datastructure.Edge{}, -1, -1, nil, nil, -1
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
						newDist := edge.Dist + distf[node.Item]

						_, ok := df[toNID]

						if !ok {
							df[toNID] = newCost

							distf[toNID] = newDist

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
							frontier.Insert(neighborNode)
							cameFromf[toNID] = cameFromPairXCHV{edge, node.Item, newCost}
						} else if newCost < df[toNID] {
							df[toNID] = newCost

							distf[toNID] = newDist

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
							frontier.DecreaseKey(neighborNode)

							cameFromf[toNID] = cameFromPairXCHV{edge, node.Item, newCost}
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

						newDist := edge.Dist + distb[node.Item]
						_, ok := db[toNID]
						if !ok {
							db[toNID] = newCost

							distb[toNID] = newDist

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
							frontier.Insert(neighborNode)
							cameFromb[toNID] = cameFromPairXCHV{edge, node.Item, newCost}
						}
						if newCost < db[toNID] {
							db[toNID] = newCost

							distb[toNID] = newDist

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
							frontier.DecreaseKey(neighborNode)

							cameFromb[toNID] = cameFromPairXCHV{edge, node.Item, newCost}
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
		return []datastructure.CHNode{}, []datastructure.Coordinate{}, []datastructure.Edge{}, -1, -1, nil, nil, -1
	}

	cameFromFCopy := make(map[int32]cameFromPairXCHV)
	for k, v := range cameFromf {
		cameFromFCopy[k] = v
	}
	cameFromBCopy := make(map[int32]cameFromPairXCHV)
	for k, v := range cameFromb {
		cameFromBCopy[k] = v
	}
	path, coordPath, edgePath, eta, dist := rt.createPathXCHV(bestCommonVertex, from, to, cameFromf, cameFromb,
		cameFromFCopy, cameFromBCopy)
	return path, coordPath, edgePath, eta, dist, cameFromf, cameFromb, bestCommonVertex
}

func (rt *RouteAlgorithm) createPathXCHV(commonVertex int32, from, to int32,
	cameFromf, cameFromb map[int32]cameFromPairXCHV, cameFromfCopy, cameFrombCopy map[int32]cameFromPairXCHV) ([]datastructure.CHNode,
	[]datastructure.Coordinate, []datastructure.Edge, float64, float64) {

	fCoordPath := make([]datastructure.Coordinate, 0)

	fPath := make([]datastructure.CHNode, 0)
	fedgePath := make([]datastructure.Edge, 0)
	eta := 0.0
	dist := 0.0
	v := commonVertex
	if rt.ch.IsTrafficLight(v) {
		eta += 3.0
	}
	ok := true

	for ok && v != -1 {
		isShortcut := rt.ch.IsShortcut(cameFromf[v].Edge.EdgeID)

		if isShortcut {

			rt.unpackBackwardXCHV(cameFromf[v].Edge, &fPath, &fedgePath, &eta, &dist, cameFromfCopy,
				&fCoordPath)
		} else {

			if cameFromf[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromf[v].NodeID) {
				eta += 3.0
			}
			eta += cameFromf[v].Edge.Weight
			dist += cameFromf[v].Edge.Dist

			pointsInBetween := make([]datastructure.Coordinate, 0)
			if cameFromf[v].Edge.Weight != 0 {
				fedgePath = append(fedgePath, cameFromf[v].Edge)
				pointsInBetween = rt.ch.GetEdgePointsInBetween(cameFromf[v].Edge.EdgeID)
				pointsInBetween = util.ReverseG(pointsInBetween)
			}

			nodeV := rt.ch.GetNode(v)

			fPath = append(fPath, nodeV)
			// fCoordPath = append(fCoordPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))
			fCoordPath = append(fCoordPath, pointsInBetween...)
		}
		_, ok = cameFromf[v]
		v = cameFromf[v].NodeID
	}

	bCoordPath := make([]datastructure.Coordinate, 0)
	bPath := make([]datastructure.CHNode, 0)
	bEdgePath := make([]datastructure.Edge, 0)
	v = commonVertex
	ok = true
	for ok && v != -1 {

		isShortcut := rt.ch.IsShortcut(cameFromb[v].Edge.EdgeID)
		if isShortcut {

			rt.unpackForwardXCHV(cameFromb[v].Edge, &bPath, &bEdgePath, &eta, &dist, cameFrombCopy,
				&bCoordPath)

		} else {

			if cameFromb[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromb[v].NodeID) {
				eta += 3.0
			}
			eta += cameFromb[v].Edge.Weight
			dist += cameFromb[v].Edge.Dist

			pointsInBetween := make([]datastructure.Coordinate, 0)
			if cameFromb[v].Edge.Weight != 0 {
				bEdgePath = append(bEdgePath, cameFromb[v].Edge)

				pointsInBetween = rt.ch.GetEdgePointsInBetween(cameFromb[v].Edge.EdgeID)
			}

			nodeV := rt.ch.GetNode(v)

			bPath = append(bPath, nodeV)
			bCoordPath = append(bCoordPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))
			bCoordPath = append(bCoordPath, pointsInBetween...)

		}
		_, ok = cameFromb[v]
		v = cameFromb[v].NodeID
	}

	fPath = util.ReverseG(fPath)
	fCoordPath = util.ReverseG(fCoordPath)
	if bPath[0].ID != commonVertex {
		fPath = append(fPath, rt.ch.GetNode(commonVertex))
	}

	commonVertexN := rt.ch.GetNode(commonVertex)
	if bCoordPath[0].Lat != commonVertexN.Lat && bCoordPath[0].Lon != commonVertexN.Lon &&
		fCoordPath[len(fCoordPath)-1].Lat != commonVertexN.Lat && fCoordPath[len(fCoordPath)-1].Lon != commonVertexN.Lon {
		fCoordPath = append(fCoordPath, datastructure.NewCoordinate(commonVertexN.Lat, commonVertexN.Lon))
	}

	path := make([]datastructure.CHNode, 0)
	path = append(path, fPath...)
	path = append(path, bPath...)

	coordPath := make([]datastructure.Coordinate, 0)
	coordPath = append(coordPath, fCoordPath...)
	coordPath = append(coordPath, bCoordPath...)

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

	return path, coordPath, edgePath, eta, dist
}

// buat forward dijkstra
// dari common vertex ke source vertex
func (rt *RouteAlgorithm) unpackBackwardXCHV(edge datastructure.Edge, path *[]datastructure.CHNode, ePath *[]datastructure.Edge,
	eta, dist *float64, cameFromf map[int32]cameFromPairXCHV, fCoordPath *[]datastructure.Coordinate) {

	isShortcut := rt.ch.IsShortcut(edge.EdgeID)

	if !isShortcut {
		if rt.ch.IsTrafficLight(edge.FromNodeID) {
			*eta += 3.0
		}
		*eta += edge.Weight
		*dist += edge.Dist

		fromNode := rt.ch.GetNode(edge.FromNodeID)

		pointsInBetween := rt.ch.GetEdgePointsInBetween(edge.EdgeID)
		pointsInBetween = util.ReverseG(pointsInBetween)

		*fCoordPath = append(*fCoordPath, pointsInBetween...)

		*path = append(*path, fromNode)

		*ePath = append(*ePath, edge)

		// for calculating plateau we must update the camefrom to, because all edge in camefrom is a shorcut
		cameFromf[edge.ToNodeID] = cameFromPairXCHV{edge, edge.FromNodeID, *eta}
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

		rt.unpackBackwardXCHV(edgeTwo, path, ePath, eta, dist, cameFromf, fCoordPath)
		rt.unpackBackwardXCHV(edgeOne, path, ePath, eta, dist, cameFromf, fCoordPath)
	}
}

// dari common vertex ke target vertex
func (rt *RouteAlgorithm) unpackForwardXCHV(edge datastructure.Edge, path *[]datastructure.CHNode, ePath *[]datastructure.Edge,
	eta, dist *float64, cameFromb map[int32]cameFromPairXCHV, bCoordPath *[]datastructure.Coordinate) {

	isShortcut := rt.ch.IsShortcut(edge.EdgeID)

	if !isShortcut {
		if rt.ch.IsTrafficLight(edge.FromNodeID) {
			*eta += 3.0
		}
		*eta += edge.Weight
		*dist += edge.Dist

		fromNode := rt.ch.GetNode(edge.FromNodeID)

		pointsInBetween := rt.ch.GetEdgePointsInBetween(edge.EdgeID)

		*bCoordPath = append(*bCoordPath, pointsInBetween...)

		*path = append(*path, fromNode)

		*ePath = append(*ePath, edge)

		cameFromb[edge.ToNodeID] = cameFromPairXCHV{edge, edge.FromNodeID, *eta}
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

		rt.unpackForwardXCHV(edgeOne, path, ePath, eta, dist, cameFromb, bCoordPath)
		rt.unpackForwardXCHV(edgeTwo, path, ePath, eta, dist, cameFromb, bCoordPath)

	}
}
