package routingalgorithm

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"
)

const (
	maxVisitedNodes = 20 // prevState->nextState should be less than 20 node visit. idk
)

// https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

func (rt *RouteAlgorithm) ShortestPathBiDijkstra(from, to int32, fromEdgeFilter, toEdgeFilter func(edge datastructure.Edge) bool) ([]datastructure.Coordinate, []datastructure.Edge,
	float64, float64) {
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

	visitedF := make(map[int32]struct{})
	visitedB := make(map[int32]struct{})

	frontFinished := false
	backFinished := false

	visitedCount := 0
	frontier := forwQ
	otherFrontier := backQ
	turnF := true
	for visitedCount < maxVisitedNodes {
		if frontier.Size() == 0 {
			frontFinished = true
		}
		if otherFrontier.Size() == 0 {
			backFinished = true
		}

		if frontFinished && backFinished {
			// stop search if both front and back search finished  (forw and backw priority queue empty)
			break
		}

		ff := *frontier
		if ff.Size() == 0 {
			return []datastructure.Coordinate{}, []datastructure.Edge{}, -1, -1
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
				_, okb := visitedB[node.Item]
				if okb {
					// The algorithm
					// terminates when the search in one directing selects a
					// vertex that has been scanned in the other direction.
					break
				}

				for _, arc := range rt.ch.GetNodeFirstOutEdges(node.Item) {

					edge := rt.ch.GetOutEdge(arc)
					if _, ok := visitedF[edge.ToNodeID]; ok {
						continue
					}

					if !fromEdgeFilter(edge) {
						// if the source node is virtual node,  must start the search from outgoing virtual edge of source node.
						continue
					} else {
						fromEdgeFilter = func(_ datastructure.Edge) bool {
							return true
						}
					}
					toNID := edge.ToNodeID
					cost := edge.Weight

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
							// If µ > ds(v) + `(v, w) + dt(w),
							//we have found a shorter path than those seen before, so
							// we update µ and its path accordingly.
							estimate = pathDistance
							bestCommonVertex = edge.ToNodeID

						}
					}

				}

				visitedF[node.Item] = struct{}{}

			} else {
				_, okf := visitedF[node.Item]
				if okf {
					// The algorithm
					// terminates when the search in one directing selects a
					// vertex that has been scanned in the other direction.
					break
				}

				for _, arc := range rt.ch.GetNodeFirstInEdges(node.Item) {

					edge := rt.ch.GetInEdge(arc)
					if _, ok := visitedB[edge.ToNodeID]; ok {
						continue
					}

					if !toEdgeFilter(edge) {
						// if the target node node is virtual node,  must start the search from incoming virtual edge of target node.
						continue
					} else {
						toEdgeFilter = func(_ datastructure.Edge) bool {
							return true
						}
					}
					toNID := edge.ToNodeID
					cost := edge.Weight

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
							// If µ > ds(v) + `(v, w) + dt(w),
							//we have found a shorter path than those seen before, so
							// we update µ and its path accordingly
							estimate = pathDistance
							bestCommonVertex = edge.ToNodeID

						}

					}
				}

				visitedB[node.Item] = struct{}{}

			}

		}

		visitedCount++

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

	if estimate == math.MaxFloat64 || visitedCount >= maxVisitedNodes {
		return []datastructure.Coordinate{}, []datastructure.Edge{}, -1, -1
	}

	path, edgePath, eta, dist := rt.createPathPlain(bestCommonVertex, from, to, cameFromf, cameFromb)
	return path, edgePath, eta, dist
}

func (rt *RouteAlgorithm) createPathPlain(commonVertex int32, from, to int32,
	cameFromf, cameFromb map[int32]cameFromPair) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64) {

	fPath := make([]datastructure.Coordinate, 0)
	fedgePath := make([]datastructure.Edge, 0)
	eta := 0.0
	dist := 0.0
	v := commonVertex
	if rt.ch.IsTrafficLight(v) {
		eta += 1.5
	}
	ok := true
	for ok && v != -1 {

		if cameFromf[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromf[v].NodeID) {
			eta += 1.5
		}
		eta += cameFromf[v].Edge.Weight
		dist += cameFromf[v].Edge.Dist

		if cameFromf[v].Edge.Weight != 0 {
			fedgePath = append(fedgePath, cameFromf[v].Edge)

		}

		nodeV := rt.ch.GetNode(v)

		fPath = append(fPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))

		_, ok = cameFromf[v]
		v = cameFromf[v].NodeID

	}

	bPath := make([]datastructure.Coordinate, 0)
	bEdgePath := make([]datastructure.Edge, 0)
	v = commonVertex
	ok = true
	for ok && v != -1 {

		if cameFromb[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromb[v].NodeID) {
			eta += 1.5
		}
		eta += cameFromb[v].Edge.Weight
		dist += cameFromb[v].Edge.Dist

		if cameFromb[v].Edge.Weight != 0 {
			bEdgePath = append(bEdgePath, cameFromb[v].Edge)

		}

		nodeV := rt.ch.GetNode(v)

		bPath = append(bPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))

		_, ok = cameFromb[v]
		v = cameFromb[v].NodeID
	}

	fPath = util.ReverseG(fPath)
	fPath = fPath[:len(fPath)-1]
	path := make([]datastructure.Coordinate, 0)
	path = append(path, fPath...)
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
