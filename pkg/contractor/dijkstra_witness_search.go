package contractor

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
)

/*
dijkstraWitnessSearch
misal kita kontraksi node v (ignoreNodeID), kita harus cari shortest path dari node u ke w yang meng ignore node v, dimana u adalah salah satu node yang terhubung ke v dan edge (u,v) \in E, dan w adalah  salah satu node yang terhubung dari v dan edge (v,w) \in E.
search dihentikan jika current visited node costnya > acceptedWeight atau ketika sampai di node w & cost target <= acceptedWeight.

O((V+E)logV) if use binary heap
O(VlogV+E) if use fibonacci heap
*/
func (ch *ContractedGraph) dijkstraWitnessSearch(fromNodeID, targetNodeID int32, ignoreNodeID int32,
	acceptedWeight float64, maxSettledNodes int, pMax float64, contracted []bool) float64 {

	visited := make(map[int32]bool)

	cost := make(map[int32]float64)

	entryMap := make(map[int32]*datastructure.Entry[int32])

	pq := datastructure.NewFibonacciHeap[int32]()
	entryMap[fromNodeID] = pq.Insert(fromNodeID, 0.0)

	cost[fromNodeID] = 0.0
	settledNodes := 0
	for settledNodes < maxSettledNodes {

		smallest := pq.GetMin()
		if pq.Size() == 0 || smallest.GetPriority() > acceptedWeight {
			return math.MaxFloat64
		}

		_, ok := cost[targetNodeID]
		if ok && cost[targetNodeID] <= acceptedWeight {
			// kita found path ke target node,  bukan yang shortest, tapi cost nya <= acceptedWeight, bisa return & gak tambahkan shortcut (u,w)
			return cost[targetNodeID]
		}

		currItem := pq.ExtractMin()

		if contracted[currItem.GetElem()] {
			continue
		}

		if currItem.GetElem() == targetNodeID {
			// found shortest path ke target node
			return cost[currItem.GetElem()]
		}

		if currItem.GetPriority() > pMax {
			// rank dari current node > maximum cost path dari node u ke w , dimana u adalah semua node yang terhubung ke v & (u,v) \in E dan w adalah semua node yang terhubung ke v & (v, w) \in E, kita stop search
			out := cost[targetNodeID]
			if out != math.MaxFloat64 {
				return out
			}
			return math.MaxFloat64
		}

		visited[currItem.GetElem()] = true
		for _, outID := range ch.ContractedFirstOutEdge[currItem.GetElem()] {
			neighbor := ch.GetOutEdge(outID)
			if visited[neighbor.ToNodeID] || neighbor.ToNodeID == ignoreNodeID ||
				contracted[neighbor.ToNodeID] {
				continue
			}

			newCost := cost[currItem.GetElem()] + neighbor.Weight

			_, ok := cost[neighbor.ToNodeID]
			if !ok {
				cost[neighbor.ToNodeID] = newCost
				entryMap[neighbor.ToNodeID] = pq.Insert(neighbor.ToNodeID, newCost)

			} else if newCost < cost[neighbor.ToNodeID] {
				cost[neighbor.ToNodeID] = newCost

				pq.DecreaseKey(entryMap[neighbor.ToNodeID], newCost)
			}
		}

		settledNodes++
	}
	return math.MaxFloat64
}
