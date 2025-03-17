package routingalgorithm

// saat ini support contraction hierarchies saja, di commit sebelumnya masih support (karena driving instruction yang masih cuma baru support CH).
// func (rt *RouteAlgorithm) AStar(from, to int32) (pathN []datastructure.CHNode, path string, eta float64, found bool, dist float64) {
// 	cameFromf := make(map[int32]cameFromPair)
// 	cameFromf[from] = cameFromPair{datastructure.EdgeCH{}, -1}
// 	heap := contractor.NewMinHeap[int32]()

// 	fromNode := contractor.PriorityQueueNode[int32]{Rank: 0, Item: from}
// 	heap.Insert(fromNode)

// 	costSoFar := make(map[int32]float64)

// 	costSoFar[rt.ch.GetAstarNode(from).ID] = 0.0
// 	distSoFar := make(map[int32]float64)
// 	distSoFar[rt.ch.GetAstarNode(from).ID] = 0.0

// 	cameFrom := make(map[int32]datastructure.CHNode)
// 	cameFrom[rt.ch.GetAstarNode(from).ID] = datastructure.CHNode{}

// 	for {
// if heap.Size() == 0 {
// 	return
// }

// 		current, _ := heap.ExtractMin()
// 		if current.Item == to {
// 			s := ""
// 			etaTraffic := 0.0

// 			path := []datastructure.CHNode{}
// 			curr := current
// 			for curr.Rank != 0 {
// 				currNode := rt.ch.GetAstarNode(curr.Item)
// 				if currNode.TrafficLight {
// 					etaTraffic += 3.0
// 				}

// 				path = append(path, currNode)

// 				curr = heap.GetItem(cameFrom[curr.Item].ID)
// 			}
// 			path = append(path, rt.ch.GetAstarNode(from))
// 			util.ReverseG(path)
// 			coords := make([][]float64, 0)

// 			pathN := []datastructure.CHNode{}
// 			edgePath := []datastructure.EdgeCH{}
// 			for _, p := range path {
// 				pathN = append(pathN, p)
// 				coords = append(coords, []float64{p.Lat, p.Lon})
// 			}
// 			s = string(polyline.EncodeCoords(coords))

// 			return pathN, s, costSoFar[current.Item] + etaTraffic, true, distSoFar[current.Item] / 1000
// 		}
// 		for _, neighbor := range rt.ch.GetOutEdgesAstar(current.Item) {
// 	newCost := costSoFar[current.Item] + neighbor.Weight
// 	dist := distSoFar[current.Item] + neighbor.Dist
// 	neighborP := rt.ch.GetAstarNode(neighbor.ToNodeID)
// 	neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: neighborP.ID}
// 	_, ok := costSoFar[neighborP.ID]
// 	if !ok {
// 		costSoFar[neighborP.ID] = newCost
// 		distSoFar[neighborP.ID] = dist

// 		heap.Insert(neighborNode)
// 		cameFromf[toNID] = cameFromPair{edge, node.Item}
// 	} else if newCost < costSoFar[neighborP.ID] {
// 		costSoFar[neighborP.ID] = newCost
// 		distSoFar[neighborP.ID] = dist
// 		cameFrom[neighborP.ID] = rt.ch.GetAstarNode(current.Item)
// 		priority := newCost + neighborP.PathEstimatedCostETA(rt.ch.GetAstarNode(to))
// 		neighborNode.Rank = priority
// 		heap.DecreaseKey(neighborNode)
// 		cameFromf[toNID] = cameFromPair{edge, node.Item}
// 	}
// }
// 	}
// }
