package routingalgorithm

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"
)

const (
	maxVisitedNodes = 30 // prevState->nextState should be less than 20 node visit. idk
)

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

	forwardProcessed := make(map[int32]struct{})
	backwardProcessed := make(map[int32]struct{})

	visitedCount := 0

	for visitedCount < maxVisitedNodes && (forwQ.Size() != 0 && backQ.Size() != 0) {

		u, _ := forwQ.ExtractMin()
		v, _ := backQ.ExtractMin()

		rt.searchPlain(forwQ, df, db, cameFromf, cameFromb, true,
			forwardProcessed, backwardProcessed, &estimate, &bestCommonVertex, fromEdgeFilter,
			toEdgeFilter, u)

		forwardProcessed[u.Item] = struct{}{}
		rt.searchPlain(backQ, df, db, cameFromf, cameFromb, false,
			forwardProcessed, backwardProcessed, &estimate, &bestCommonVertex, fromEdgeFilter,
			toEdgeFilter, v)

		backwardProcessed[v.Item] = struct{}{}

		if df[u.Item]+db[v.Item] >= estimate {
			break
		}

		visitedCount++
	}

	if estimate == math.MaxFloat64 || visitedCount >= maxVisitedNodes {
		return []datastructure.Coordinate{}, []datastructure.Edge{}, -1, -1
	}

	path, edgePath, eta, dist := rt.createPathPlain(bestCommonVertex, from, to, cameFromf, cameFromb)
	return path, edgePath, eta, dist
}

func (rt *RouteAlgorithm) searchPlain(frontier *contractor.MinHeap[int32], df, db map[int32]float64,
	cameFromf, cameFromb map[int32]cameFromPair, turnF bool, forwardProcessed, backwardProcessed map[int32]struct{},
	estimate *float64, bestCommonVertex *int32, fromEdgeFilter, toEdgeFilter func(edge datastructure.Edge) bool, node contractor.PriorityQueueNode[int32]) {
	if turnF {

		for _, arc := range rt.ch.GetNodeFirstOutEdges(node.Item) {

			edge := rt.ch.GetOutEdge(arc)
			if _, ok := forwardProcessed[edge.ToNodeID]; ok {
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

			_, ok = backwardProcessed[toNID]
			if ok {
				pathDistance := newCost + db[toNID]
				if pathDistance < *estimate {
					*estimate = pathDistance
					*bestCommonVertex = node.Item
				}
			}
		}

	} else {

		for _, arc := range rt.ch.GetNodeFirstInEdges(node.Item) {

			edge := rt.ch.GetInEdge(arc)
			if _, ok := backwardProcessed[edge.ToNodeID]; ok {
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

			_, ok = forwardProcessed[toNID]
			if ok {
				pathDistance := newCost + df[toNID]
				if pathDistance < *estimate {
					*estimate = pathDistance
					*bestCommonVertex = node.Item
				}
			}

		}

	}

}

func (rt *RouteAlgorithm) createPathPlain(commonVertex int32, from, to int32,
	cameFromf, cameFromb map[int32]cameFromPair) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64) {

	fPath := make([]datastructure.Coordinate, 0)
	fedgePath := make([]datastructure.Edge, 0)
	eta := 0.0
	dist := 0.0
	v := commonVertex
	if rt.ch.IsTrafficLight(v) {
		eta += trafficLightAdditionalWeight
	}
	ok := true
	for ok && cameFromf[v].NodeID != -1 {

		if cameFromf[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromf[v].NodeID) {
			eta += trafficLightAdditionalWeight
		}
		eta += cameFromf[v].Edge.Weight
		dist += cameFromf[v].Edge.Dist

		if cameFromf[v].Edge.Weight != 0 {
			fedgePath = append(fedgePath, cameFromf[v].Edge)

		}

		nodeV := rt.ch.GetNode(cameFromf[v].NodeID)

		fPath = append(fPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))

		_, ok = cameFromf[v]
		v = cameFromf[v].NodeID

	}

	bPath := make([]datastructure.Coordinate, 0)
	bEdgePath := make([]datastructure.Edge, 0)
	v = commonVertex
	ok = true
	for ok && cameFromb[v].NodeID != -1 {

		if cameFromb[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromb[v].NodeID) {
			eta += trafficLightAdditionalWeight
		}
		eta += cameFromb[v].Edge.Weight
		dist += cameFromb[v].Edge.Dist

		if cameFromb[v].Edge.Weight != 0 {
			bEdgePath = append(bEdgePath, cameFromb[v].Edge)

		}

		nodeV := rt.ch.GetNode(cameFromb[v].NodeID)

		bPath = append(bPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))

		_, ok = cameFromb[v]
		v = cameFromb[v].NodeID
	}

	fPath = util.ReverseG(fPath)

	path := make([]datastructure.Coordinate, 0)
	path = append(path, fPath...)
	vianode := rt.ch.GetNode(commonVertex)
	path = append(path, datastructure.NewCoordinate(vianode.Lat, vianode.Lon))
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
