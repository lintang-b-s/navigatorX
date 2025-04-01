package routingalgorithm

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"
)

func (rt *RouteAlgorithm) ShortestPathBiDijkstraCHCSR(from, to int32) ([]datastructure.Coordinate, []datastructure.EdgeCH,
	float64, float64, error) {
	if from == to {
		return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, 0, 0, nil
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
			return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1, nil
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

				outEdges, err := rt.ch.GetNodeOutEdgesCsr2(node.Item)
				if err != nil {
					return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1, nil
				}
				for _, arc := range outEdges {

					toNID := arc.ToNodeID
					cost := arc.Weight

					if _, ok := visitedF[toNID]; ok {
						continue
					}

					if rt.ch.GetNode(node.Item).OrderPos < rt.ch.GetNode(toNID).OrderPos {
						// upward graph
						newCost := cost + df[node.Item]
						_, ok := df[toNID]
						// relax edge
						if !ok {
							df[toNID] = newCost

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
							frontier.Insert(neighborNode)
							cameFromf[toNID] = cameFromPair{arc, node.Item}
						} else if newCost < df[toNID] {
							df[toNID] = newCost

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
							frontier.DecreaseKey(neighborNode)

							cameFromf[toNID] = cameFromPair{arc, node.Item}
						}

						_, ok = db[toNID]
						if ok {
							pathDistance := newCost + db[toNID]
							if pathDistance < estimate {
								// jika toNID visited di backward search & d(s,toNID) + d(t,toNID) < cost best candidate path, maka update best candidate path
								estimate = pathDistance
								bestCommonVertex = arc.ToNodeID

							}
						}
					}
				}

				visitedF[node.Item] = struct{}{}

			} else {

				inEdges, err := rt.ch.GetNodeInEdgesCsr2(node.Item)
				if err != nil {
					return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1, nil
				}
				for _, arc := range inEdges {

					toNID := arc.ToNodeID
					cost := arc.Weight

					if _, ok := visitedB[toNID]; ok {
						continue
					}

					if rt.ch.GetNode(node.Item).OrderPos < rt.ch.GetNode(toNID).OrderPos {
						// downward graph
						newCost := cost + db[node.Item]
						_, ok := db[toNID]
						if !ok {
							db[toNID] = newCost

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
							frontier.Insert(neighborNode)
							cameFromb[toNID] = cameFromPair{arc, node.Item}
						}
						if newCost < db[toNID] {
							db[toNID] = newCost

							neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
							frontier.DecreaseKey(neighborNode)

							cameFromb[toNID] = cameFromPair{arc, node.Item}
						}

						_, ok = df[toNID]
						if ok {
							pathDistance := newCost + df[toNID]
							if pathDistance < estimate {
								estimate = pathDistance
								bestCommonVertex = arc.ToNodeID

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
		return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1, nil
	}
	// estimate dari bidirectional dijkstra pake shortcut edge jadi lebih cepet eta nya & gak akurat
	path, edgePath, eta, dist, err := rt.createPathCsr(bestCommonVertex, from, to, cameFromf, cameFromb)
	return path, edgePath, eta, dist, err
}

const (
	tolerance = 0.00001
)

func (rt *RouteAlgorithm) createPathCsr(commonVertex int32, from, to int32,
	cameFromf, cameFromb map[int32]cameFromPair) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64, error) {

	fPath := make([]datastructure.Coordinate, 0)
	fedgePath := make([]datastructure.EdgeCH, 0)
	eta := 0.0
	dist := 0.0
	v := commonVertex
	if ok, _ := rt.ch.IsTrafficLightCsr(v); ok {
		eta += 3.0
	}
	ok := true
	for ok && v != -1 {

		isShortcut, err := rt.ch.IsShortcutCsr(cameFromf[v].Edge.FromNodeID, cameFromf[v].Edge.ToNodeID, false)
		if err != nil {
			return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1, err
		}
		if isShortcut {

			err = rt.unpackBackwardCsr(cameFromf[v].Edge, &fPath, &fedgePath, &eta, &dist)
			if err != nil {
				return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1, err
			}
		} else {

			if ok, _ := rt.ch.IsTrafficLightCsr(cameFromf[v].NodeID); cameFromf[v].NodeID != -1 && ok {
				eta += 3.0
			}
			eta += cameFromf[v].Edge.Weight
			dist += cameFromf[v].Edge.Dist

			pointsInBetween := make([]datastructure.Coordinate, 0)
			if cameFromf[v].Edge.Weight != 0 {
				fedgePath = append(fedgePath, cameFromf[v].Edge)
				pointsInBetween, err = rt.ch.GetEdgePointsInBetweenCsr(cameFromf[v].Edge.FromNodeID, cameFromf[v].Edge.ToNodeID, false)
				if err != nil {
					return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1, err
				}

				fromNode := rt.ch.GetNode(cameFromf[v].Edge.FromNodeID)
				if math.Abs(pointsInBetween[0].Lat-fromNode.Lat) <= tolerance &&
					math.Abs(pointsInBetween[0].Lon-fromNode.Lon) <= tolerance {
					// pointsInBetween[0] must be toNode of outEdge
					pointsInBetween = util.ReverseG(pointsInBetween)
				}
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

		isShortcut, err := rt.ch.IsShortcutCsr(cameFromb[v].Edge.FromNodeID, cameFromb[v].Edge.ToNodeID, true)
		if err != nil {
			return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1, err
		}
		if isShortcut {

			err = rt.unpackForwardCsr(cameFromb[v].Edge, &bPath, &bEdgePath, &eta, &dist)
			if err != nil {
				return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1, err
			}
		} else {

			if ok, _ := rt.ch.IsTrafficLightCsr(cameFromb[v].NodeID); cameFromb[v].NodeID != -1 && ok {
				eta += 3.0
			}
			eta += cameFromb[v].Edge.Weight
			dist += cameFromb[v].Edge.Dist

			pointsInBetween := make([]datastructure.Coordinate, 0)
			if cameFromb[v].Edge.Weight != 0 {
				bEdgePath = append(bEdgePath, cameFromb[v].Edge)
				pointsInBetween, err = rt.ch.GetEdgePointsInBetweenCsr(cameFromb[v].Edge.FromNodeID, cameFromb[v].Edge.ToNodeID, true)
				if err != nil {
					return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, -1, -1, err
				}

				toNode := rt.ch.GetNode(cameFromb[v].Edge.ToNodeID)
				if math.Abs(pointsInBetween[0].Lat-toNode.Lat) > tolerance && math.Abs(pointsInBetween[0].Lon-toNode.Lon) > tolerance {
					// pointsInBetween[0] must be fromNode of inEdge
					pointsInBetween = util.ReverseG(pointsInBetween)
				}

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

	return path, edgePath, eta, dist / 1000, nil
}

// buat forward dijkstra
// dari common vertex ke source vertex
func (rt *RouteAlgorithm) unpackBackwardCsr(edge datastructure.EdgeCH, path *[]datastructure.Coordinate, ePath *[]datastructure.EdgeCH,
	eta, dist *float64) error {
	if edge.FromNodeID == 0 && edge.ToNodeID == 0 {
		return nil
	}

	isShortcut, err := rt.ch.IsShortcutCsr(edge.FromNodeID, edge.ToNodeID, false)
	if err != nil {
		return err
	}
	if !isShortcut {
		if ok, _ := rt.ch.IsTrafficLightCsr(edge.FromNodeID); ok {
			*eta += 3.0
		}
		*eta += edge.Weight
		*dist += edge.Dist

		pointsInBetween, err := rt.ch.GetEdgePointsInBetweenCsr(edge.FromNodeID, edge.ToNodeID, false)

		if err != nil {
			return err
		}

		fromNode := rt.ch.GetNode(edge.FromNodeID)
		if math.Abs(pointsInBetween[0].Lat-fromNode.Lat) <= tolerance && math.Abs(pointsInBetween[0].Lon-fromNode.Lon) <= tolerance {
			pointsInBetween = util.ReverseG(pointsInBetween)
		}

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

		outEdges, err := rt.ch.GetNodeOutEdgesCsr2(fromNodeID)
		if err != nil {
			return err
		}
		for _, arc := range outEdges {

			if arc.ToNodeID == edge.ViaNodeID {
				edgeOne = arc
				break
			}
		}

		outEdges, err = rt.ch.GetNodeOutEdgesCsr2(edge.ViaNodeID)
		if err != nil {
			return err
		}
		for _, arc := range outEdges {

			if arc.ToNodeID == toNodeID {
				edgeTwo = arc
				break
			}
		}

		err = rt.unpackBackwardCsr(edgeTwo, path, ePath, eta, dist)
		if err != nil {
			return err
		}
		err = rt.unpackBackwardCsr(edgeOne, path, ePath, eta, dist)
		if err != nil {
			return err
		}
	}

	return nil
}

// dari common vertex ke target vertex
func (rt *RouteAlgorithm) unpackForwardCsr(edge datastructure.EdgeCH, path *[]datastructure.Coordinate, ePath *[]datastructure.EdgeCH,
	eta, dist *float64) error {
	if edge.FromNodeID == 0 && edge.ToNodeID == 0 {
		return nil
	}

	isShortcut, err := rt.ch.IsShortcutCsr(edge.FromNodeID, edge.ToNodeID, true)
	if err != nil {
		return err
	}
	if !isShortcut {
		if ok, _ := rt.ch.IsTrafficLightCsr(edge.ToNodeID); ok {
			*eta += 3.0
		}
		*eta += edge.Weight
		*dist += edge.Dist

		pointsInBetween, err := rt.ch.GetEdgePointsInBetweenCsr(edge.FromNodeID, edge.ToNodeID, true)
		if err != nil {
			return err
		}

		toNode := rt.ch.GetNode(edge.ToNodeID)

		if math.Abs(pointsInBetween[0].Lat-toNode.Lat) > tolerance && math.Abs(pointsInBetween[0].Lon-toNode.Lon) > tolerance {
			pointsInBetween = util.ReverseG(pointsInBetween)
		}

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

		inEdges, err := rt.ch.GetNodeInEdgesCsr2(edge.ViaNodeID)
		if err != nil {
			return err
		}
		for _, arc := range inEdges {
			if arc.ToNodeID == toNodeID {
				edgeOne = arc
				break
			}
		}

		inEdges, err = rt.ch.GetNodeInEdgesCsr2(fromNodeID)
		if err != nil {
			return err
		}
		for _, arc := range inEdges {
			if arc.ToNodeID == edge.ViaNodeID {
				edgeTwo = arc
				break
			}
		}

		err = rt.unpackForwardCsr(edgeOne, path, ePath, eta, dist)
		if err != nil {
			return err
		}
		err = rt.unpackForwardCsr(edgeTwo, path, ePath, eta, dist)
		if err != nil {
			return err
		}
	}
	return nil
}
