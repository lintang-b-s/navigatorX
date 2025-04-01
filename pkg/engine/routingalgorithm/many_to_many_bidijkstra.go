package routingalgorithm

import (
	"github.com/lintang-b-s/navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
)

func (rt *RouteAlgorithm) CallBidirectionalDijkstra(spMap []int32) datastructure.SPSingleResultResult {

	path, edgePath, eta, dist := rt.ShortestPathBiDijkstraCH(spMap[0], spMap[1])

	return datastructure.SPSingleResultResult{spMap[0], spMap[1], path, edgePath, dist, eta}
}

func (rt *RouteAlgorithm) ShortestPathManyToManyBiDijkstraWorkers(from []int32, to []int32) map[int32]map[int32]datastructure.SPSingleResultResult {
	spPair := [][]int32{}
	for i := 0; i < len(from); i++ {
		for j := 0; j < len(to); j++ {

			spPair = append(spPair, []int32{from[i], to[j]})
		}
	}
	workers := concurrent.NewWorkerPool[[]int32, datastructure.SPSingleResultResult](3, len(spPair))

	for i := 0; i < len(spPair); i++ {
		workers.AddJob(spPair[i])
	}
	workers.Close()
	spMap := make(map[int32]map[int32]datastructure.SPSingleResultResult)

	workers.Start(rt.CallBidirectionalDijkstra)

	workers.Wait()

	for i := 0; i < len(spPair); i++ {
		spMap[spPair[i][0]] = make(map[int32]datastructure.SPSingleResultResult)
	}

	for curr := range workers.CollectResults() {

		spMap[curr.Source][curr.Dest] = curr
	}

	return spMap
}
func (rt *RouteAlgorithm) CreateDistMatrix(spPair [][]int32) map[int32]map[int32]datastructure.SPSingleResultResult {
	workers := concurrent.NewWorkerPool[[]int32, datastructure.SPSingleResultResult](10, len(spPair))

	for i := 0; i < len(spPair); i++ {
		workers.AddJob(spPair[i])
	}

	workers.Close()

	spMap := make(map[int32]map[int32]datastructure.SPSingleResultResult)

	workers.Start(rt.CallBidirectionalDijkstra)

	workers.Wait()

	for i := 0; i < len(spPair); i++ {
		spMap[spPair[i][0]] = make(map[int32]datastructure.SPSingleResultResult)
	}

	for curr := range workers.CollectResults() {

		spMap[curr.Source][curr.Dest] = curr
	}

	return spMap
}
