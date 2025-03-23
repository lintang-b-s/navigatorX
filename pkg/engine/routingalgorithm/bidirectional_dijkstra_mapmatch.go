package routingalgorithm

import (
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/datastructure"
	"math"
)

const (
	maxVisitedNodes = 25  // prevState->nextState should be less than 25 node visit. idk
)

func (rt *RouteAlgorithm) ShortestPathBiDijkstra(from, to int32, fromEdgeFilter, toEdgeFilter func(edge datastructure.EdgeCH) bool) ([]datastructure.Coordinate, []datastructure.EdgeCH,
	float64, float64) {
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

	estimate := math.MaxFloat64

	bestCommonVertex := int32(0)

	cameFromf := make(map[int32]cameFromPair)
	cameFromf[from] = cameFromPair{datastructure.EdgeCH{}, -1}

	cameFromb := make(map[int32]cameFromPair)
	cameFromb[to] = cameFromPair{datastructure.EdgeCH{}, -1}

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
					if !fromEdgeFilter(edge) {
						// if the source node is virtual node,  must start the search from outgoing virtual edge of source node.
						continue
					} else {
						fromEdgeFilter = func(_ datastructure.EdgeCH) bool {
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
							// jika toNID visited di backward search & d(s,toNID) + d(t,toNID) < cost best candidate path, maka update best candidate path
							estimate = pathDistance
							bestCommonVertex = edge.ToNodeID

						}
					}

				}

			} else {
				for _, arc := range rt.ch.GetNodeFirstInEdges(node.Item) {
					edge := rt.ch.GetInEdge(arc)

					if !toEdgeFilter(edge) {
						// if the target node node is virtual node,  must start the search from incoming virtual edge of target node.
						continue
					} else {
						toEdgeFilter = func(_ datastructure.EdgeCH) bool {
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
							estimate = pathDistance
							bestCommonVertex = edge.ToNodeID

						}

					}
				}

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
		return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1
	}
	// estimate dariway bidirectional dijkstra pake shortcut edge jadi lebih cepet eta nya & gak akurat
	path, edgePath, eta, dist := rt.createPath(bestCommonVertex, from, to, cameFromf, cameFromb)
	return path, edgePath, eta, dist
}
