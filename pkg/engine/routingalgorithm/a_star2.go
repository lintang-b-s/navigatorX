package routingalgorithm

import (
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"lintang/navigatorx/pkg/util"
)

// https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

func (rt *RouteAlgorithm) ShortestPathAStar(from, to int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64) {
	if from == to {
		return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, 0, 0
	}

	pq := contractor.NewMinHeap[int32]()

	costSoFar := make(map[int32]float64)

	costSoFar[from] = 0.0

	distSoFar := make(map[int32]float64)
	distSoFar[from] = 0.0

	fromNode := contractor.PriorityQueueNode[int32]{Rank: 0, Item: from}

	pq.Insert(fromNode)

	cameFrom := make(map[int32]cameFromPair)

	cameFrom[from] = cameFromPair{datastructure.EdgeCH{}, -1}

	visited := make(map[int32]struct{})

	for {
		if pq.Size() == 0 {
			return []datastructure.Coordinate{}, []datastructure.EdgeCH{}, 0, 0
		}

		current, _ := pq.ExtractMin()
		if current.Item == to {
			pathCoords := []datastructure.Coordinate{}
			pathEdges := []datastructure.EdgeCH{}
			etaTraffic := 0.0

			currNode := rt.ch.GetNode(current.Item)
			for cameFrom[currNode.ID].NodeID != -1 {

				if currNode.TrafficLight {
					etaTraffic += 3.0
				}

				pathCoords = append(pathCoords, datastructure.Coordinate{Lat: currNode.Lat, Lon: currNode.Lon})
				pathEdges = append(pathEdges, cameFrom[currNode.ID].Edge)
				currNode = rt.ch.GetNode(cameFrom[currNode.ID].NodeID)
			}

			fromNode := rt.ch.GetNode(from)
			pathCoords = append(pathCoords, datastructure.Coordinate{Lat: fromNode.Lat, Lon: fromNode.Lon})
			pathCoords = util.ReverseG(pathCoords)
			pathEdges = util.ReverseG(pathEdges)

			return pathCoords, pathEdges, costSoFar[current.Item] + etaTraffic, distSoFar[current.Item] / 1000
		}

		for _, edgeID := range rt.ch.GetNodeFirstOutEdges(current.Item) {

			if _, ok := visited[rt.ch.GetOutEdge(edgeID).ToNodeID]; ok {
				continue
			}

			edge := rt.ch.GetOutEdge(edgeID)
			newCost := costSoFar[current.Item] + edge.Weight

			dist := distSoFar[current.Item] + edge.Dist // in meter
			neighborP := rt.ch.GetNode(edge.ToNodeID)

			_, ok := costSoFar[neighborP.ID]
			if !ok {
				priority := newCost + rt.pathEstimatedCostETA(neighborP, rt.ch.GetNode(to)) // add heuristic
				neighborNode := contractor.PriorityQueueNode[int32]{Rank: priority, Item: neighborP.ID}

				costSoFar[neighborP.ID] = newCost
				distSoFar[neighborP.ID] = dist

				pq.Insert(neighborNode)
				cameFrom[edge.ToNodeID] = cameFromPair{edge, current.Item}
			} else if newCost < costSoFar[neighborP.ID] {
				priority := newCost + rt.pathEstimatedCostETA(neighborP, rt.ch.GetNode(to)) // add heuristic
				neighborNode := contractor.PriorityQueueNode[int32]{Rank: priority, Item: neighborP.ID}

				costSoFar[neighborP.ID] = newCost
				distSoFar[neighborP.ID] = dist

				pq.DecreaseKey(neighborNode)
				cameFrom[edge.ToNodeID] = cameFromPair{edge, current.Item}
			}
		}

		visited[current.Item] = struct{}{}
	}
}

func (rt *RouteAlgorithm) pathEstimatedCostETA(from, to datastructure.CHNode) float64 {

	dist := geo.CalculateHaversineDistance(from.Lat, from.Lon, to.Lat, to.Lon) // in km

	toEdgeSpeed := 40.0 // km/h

	eta := dist / toEdgeSpeed // dist = km, speed = km/h , eta = h
	eta *= 60                 // in minutes
	return eta
}
