package contractor

import "math"

/*
dijkstraWitnessSearch
misal kita kontraksi node v (ignoreNodeID), kita harus cari shortest path dari node u ke w yang meng ignore node v, dimana u adalah salah satu node yang terhubung ke v dan edge (u,v) \in E, dan w adalah  salah satu node yang terhubung dari v dan edge (v,w) \in E.
search dihentikan jika current visited node costnya > acceptedWeight atau ketika sampai di node w & cost target <= acceptedWeight.
*/
func (ch *ContractedGraph) dijkstraWitnessSearch(fromNodeID, targetNodeID int32, ignoreNodeID int32,
	acceptedWeight float64, maxSettledNodes int, pMax float64, contracted []bool) float64 {

	visited := make(map[int32]bool)

	cost := make(map[int32]float64)
	pq := NewMinHeap[int32]()
	fromNode := PriorityQueueNode[int32]{Rank: 0, Item: fromNodeID}
	pq.Insert(fromNode)

	cost[fromNodeID] = 0.0
	settledNodes := 0
	for {

		smallest, _ := pq.GetMin()
		if pq.Size() == 0 || smallest.Rank > acceptedWeight {
			return math.MaxFloat64
		}

		_, ok := cost[targetNodeID]
		if ok && cost[targetNodeID] <= acceptedWeight {
			// kita found path ke target node,  bukan yang shortest, tapi cost nya <= acceptedWeight, bisa return & gak tambahkan shortcut (u,w)
			return cost[targetNodeID]
		}

		currItem, _ := pq.ExtractMin()

		if contracted[currItem.Item] {
			continue
		}

		if currItem.Item == targetNodeID {
			// found shortest path ke target node
			return cost[currItem.Item]
		}

		if currItem.Rank > pMax {
			// rank dari current node > maximum cost path dari node u ke w , dimana u adalah semua node yang terhubung ke v & (u,v) \in E dan w adalah semua node yang terhubung ke v & (v, w) \in E, kita stop search
			out := cost[targetNodeID]
			if out != math.MaxFloat64 {
				return out
			}
			return math.MaxFloat64
		}

		visited[currItem.Item] = true
		for _, outID := range ch.ContractedFirstOutEdge[currItem.Item] {
			neighbor := ch.ContractedOutEdges[outID]
			if visited[neighbor.ToNodeID] || neighbor.ToNodeID == ignoreNodeID ||
				contracted[neighbor.ToNodeID] {
				continue
			}

			newCost := cost[currItem.Item] + neighbor.Weight
			neighborNode := PriorityQueueNode[int32]{Rank: newCost, Item: neighbor.ToNodeID}

			_, ok := cost[neighbor.ToNodeID]
			if !ok {
				cost[neighbor.ToNodeID] = newCost
				pq.Insert(neighborNode)

			} else if newCost < cost[neighbor.ToNodeID] {
				cost[neighbor.ToNodeID] = newCost

				neighborNode.Rank = newCost
				pq.DecreaseKey(neighborNode)
			}
		}

		settledNodes++
	}
}
