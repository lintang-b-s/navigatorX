package routingalgorithm

import (
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/util"
	"math"
)

type cameFromPair struct {
	Edge    datastructure.EdgeCH
	NodeIDx int32
}

type ContractedGraph interface {
	GetFirstOutEdge(nodeIDx int32) []int32
	GetFirstInEdge(nodeIDx int32) []int32
	GetNode(nodeIDx int32) datastructure.CHNode2
	GetOutEdge(edgeIDx int32) datastructure.EdgeCH
	GetInEdge(edgeIDx int32) datastructure.EdgeCH
	GetNumNodes() int
}

type RouteAlgorithm struct {
	ch ContractedGraph
}

func NewRouteAlgorithm(ch ContractedGraph) *RouteAlgorithm {
	return &RouteAlgorithm{ch: ch}
}

func (rt *RouteAlgorithm) ShortestPathBiDijkstra(from, to int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64) {

	forwQ := contractor.NewMinHeap[int32]()
	backQ := contractor.NewMinHeap[int32]()
	forwVisited := make(map[int32]bool)
	backVisited := make(map[int32]bool)

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
				forwVisited[node.Item] = true

				for _, arc := range rt.ch.GetFirstOutEdge(node.Item) {
					edge := rt.ch.GetOutEdge(arc)
					toNIDx := edge.ToNodeIDX
					cost := edge.Weight
					if forwVisited[toNIDx] {
						continue
					}
					if rt.ch.GetNode(node.Item).OrderPos < rt.ch.GetNode(toNIDx).OrderPos {
						// upward graph
						newCost := cost + df[node.Item]
						_, ok := df[toNIDx]
						// relax edge
						if !ok {
							df[toNIDx] = newCost

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNIDx}
							frontier.Insert(neighborNode)
							cameFromf[toNIDx] = cameFromPair{edge, node.Item}
						} else if newCost < df[toNIDx] {
							df[toNIDx] = newCost

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNIDx}
							frontier.DecreaseKey(neighborNode)

							cameFromf[toNIDx] = cameFromPair{edge, node.Item}
						}

						_, ok = db[toNIDx]
						if ok {
							pathDistance := newCost + db[toNIDx]
							if pathDistance < estimate {
								// jika toNIDx visited di backward search & d(s,toNIDx) + d(t,toNIDx) < cost best candidate path, maka update best candidate path
								estimate = pathDistance
								bestCommonVertex = edge.ToNodeIDX

							}
						}
					}
				}

			} else {
				backVisited[node.Item] = true
				for _, arc := range rt.ch.GetFirstInEdge(node.Item) {

					edge := rt.ch.GetInEdge(arc)
					toNIDx := edge.ToNodeIDX
					cost := edge.Weight
					if backVisited[toNIDx] {
						continue
					}
					if rt.ch.GetNode(node.Item).OrderPos < rt.ch.GetNode(toNIDx).OrderPos {
						// downward graph
						newCost := cost + db[node.Item]
						_, ok := db[toNIDx]
						if !ok {
							db[toNIDx] = newCost

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNIDx}
							frontier.Insert(neighborNode)
							cameFromb[toNIDx] = cameFromPair{edge, node.Item}
						}
						if newCost < db[toNIDx] {
							db[toNIDx] = newCost

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNIDx}
							frontier.DecreaseKey(neighborNode)

							cameFromb[toNIDx] = cameFromPair{edge, node.Item}
						}

						_, ok = df[toNIDx]
						if ok {
							pathDistance := newCost + df[toNIDx]
							if pathDistance < estimate {
								estimate = pathDistance
								bestCommonVertex = edge.ToNodeIDX

							}
						}
					}
				}

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

	fPath := []datastructure.Coordinate{}
	fedgePath := []datastructure.EdgeCH{}
	eta := 0.0
	dist := 0.0
	v := commonVertex
	if rt.ch.GetNode(v).TrafficLight {
		eta += 3.0
	}
	ok := true
	for ok && v != -1 {

		if cameFromf[v].Edge.IsShortcut {

			rt.unpackBackward(cameFromf[v].Edge, &fPath, &fedgePath, &eta, &dist)
		} else {

			if cameFromf[v].NodeIDx != -1 && rt.ch.GetNode(cameFromf[v].NodeIDx).TrafficLight {
				eta += 3.0
			}
			eta += cameFromf[v].Edge.Weight
			dist += cameFromf[v].Edge.Dist
			if cameFromb[v].Edge.Weight != 0 {
				fedgePath = append(fedgePath, cameFromb[v].Edge)
			}

			nodesInBetween := make([]datastructure.Coordinate, len(cameFromf[v].Edge.NodesInBetween))
			copy(nodesInBetween, cameFromf[v].Edge.NodesInBetween)
			util.ReverseG(nodesInBetween)

			nodeV := rt.ch.GetNode(v)

			fPath = append(fPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))
			fPath = append(fPath, nodesInBetween...)
		}
		_, ok = cameFromf[v]
		v = cameFromf[v].NodeIDx

	}

	bPath := []datastructure.Coordinate{}
	bEdgePath := []datastructure.EdgeCH{}
	v = commonVertex
	ok = true
	for ok && v != -1 {

		if cameFromb[v].Edge.IsShortcut {

			rt.unpackForward(cameFromb[v].Edge, &bPath, &bEdgePath, &eta, &dist)

		} else {

			if cameFromb[v].NodeIDx != -1 && rt.ch.GetNode(cameFromb[v].NodeIDx).TrafficLight {
				eta += 3.0
			}
			eta += cameFromb[v].Edge.Weight
			dist += cameFromb[v].Edge.Dist
			if cameFromb[v].Edge.Weight != 0 {
				bEdgePath = append(bEdgePath, cameFromb[v].Edge)
			}

			nodeV := rt.ch.GetNode(v)

			bPath = append(bPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))
			bPath = append(bPath, cameFromb[v].Edge.NodesInBetween...)
		}
		_, ok = cameFromb[v]
		v = cameFromb[v].NodeIDx
	}

	util.ReverseG(fPath)
	fPath = fPath[:len(fPath)-1]
	path := []datastructure.Coordinate{}
	path = append(path, fPath...)
	path = append(path, bPath...)

	edgePath := []datastructure.EdgeCH{}
	util.ReverseG(fedgePath)

	for i := 0; i < len(bEdgePath); i++ {
		curr := bEdgePath[i]
		// harus dibalik buat backward edge path nya
		// karena base node arah dari target ke common vertex, sedangkan di driving instruction butuhnya dari common ke target
		toNodeIDx := curr.BaseNodeIDx
		baseNodeIDx := curr.ToNodeIDX
		bEdgePath[i].BaseNodeIDx = baseNodeIDx
		bEdgePath[i].ToNodeIDX = toNodeIDx
	}
	if len(fedgePath) > 0 {
		edgePath = append(edgePath, fedgePath[1:]...)
	}
	if len(bEdgePath) > 0 {
		edgePath = append(edgePath, bEdgePath[:len(bEdgePath)-1]...)
	}

	return path, edgePath, eta, dist / 1000
}

// buat forward dijkstra
// dari common vertex ke source vertex
func (rt *RouteAlgorithm) unpackBackward(edge datastructure.EdgeCH, path *[]datastructure.Coordinate, ePath *[]datastructure.EdgeCH,
	eta, dist *float64) {
	if !edge.IsShortcut {
		if rt.ch.GetNode(edge.ToNodeIDX).TrafficLight {
			*eta += 3.0
		}
		*eta += edge.Weight
		*dist += edge.Dist

		nodesInBetween := make([]datastructure.Coordinate, len(edge.NodesInBetween))
		copy(nodesInBetween, edge.NodesInBetween)
		util.ReverseG(nodesInBetween)

		toNode := rt.ch.GetNode(edge.ToNodeIDX)

		*path = append(*path, datastructure.NewCoordinate(toNode.Lat, toNode.Lon))
		*path = append(*path, nodesInBetween...)

		*ePath = append(*ePath, edge)
	} else {
		rt.unpackBackward(rt.ch.GetOutEdge(edge.RemovedEdgeTwo), path, ePath, eta, dist)
		rt.unpackBackward(rt.ch.GetOutEdge(edge.RemovedEdgeOne), path, ePath, eta, dist)
	}
}

// dari common vertex ke target vertex
func (rt *RouteAlgorithm) unpackForward(edge datastructure.EdgeCH, path *[]datastructure.Coordinate, ePath *[]datastructure.EdgeCH,
	eta, dist *float64) {
	if !edge.IsShortcut {
		if rt.ch.GetNode(edge.ToNodeIDX).TrafficLight {
			*eta += 3.0
		}
		*eta += edge.Weight
		*dist += edge.Dist

		nodesInBetween := edge.NodesInBetween
		toNode := rt.ch.GetNode(edge.ToNodeIDX)

		*path = append(*path, datastructure.NewCoordinate(toNode.Lat, toNode.Lon))
		*path = append(*path, nodesInBetween...)

		*ePath = append(*ePath, edge)
	} else {
		rt.unpackForward(rt.ch.GetInEdge(edge.RemovedEdgeOne), path, ePath, eta, dist)
		rt.unpackForward(rt.ch.GetInEdge(edge.RemovedEdgeTwo), path, ePath, eta, dist)

	}
}
