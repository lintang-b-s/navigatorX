package alg

type priorityQueueBiDijkstra []*bidijkstraNode

func (pq priorityQueueBiDijkstra) Len() int {
	return len(pq)
}

func (pq priorityQueueBiDijkstra) Less(i, j int) bool {
	return pq[i].rank < pq[j].rank
}

func (pq priorityQueueBiDijkstra) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}

func (pq *priorityQueueBiDijkstra) Push(x interface{}) {
	n := len(*pq)
	no := x.(*bidijkstraNode)
	no.index = n
	*pq = append(*pq, no)
}

func (pq *priorityQueueBiDijkstra) Pop() interface{} {
	old := *pq
	n := len(old)
	no := old[n-1]
	no.index = -1
	*pq = old[0 : n-1]
	return no
}