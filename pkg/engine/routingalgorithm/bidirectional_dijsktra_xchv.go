package routingalgorithm

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"
)

type cameFromPairXCHV struct {
	Edge   datastructure.Edge
	NodeID int32
	Dist   float64
	Weight float64
}

const (
	epsilon = 0.25
)

func (rt *RouteAlgorithm) ShortestPathBiDijkstraXCHV(from, to int32, k int, pruneRadius bool, lOpt float64) ([]datastructure.CHNode,
	[]datastructure.Coordinate, []datastructure.Edge, float64, float64,
	map[int32]cameFromPairXCHV, map[int32]cameFromPairXCHV, int32) {
	if from == to {
		return []datastructure.CHNode{}, []datastructure.Coordinate{}, []datastructure.Edge{}, 0, 0, nil, nil, -1
	}
	forwQ := contractor.NewMinHeap[int32]()
	backQ := contractor.NewMinHeap[int32]()

	df := make(map[int32]float64)
	db := make(map[int32]float64)
	df[from] = 0.0
	db[to] = 0.0

	distf := make(map[int32]float64)
	distb := make(map[int32]float64)
	distf[from] = 0.0
	distb[to] = 0.0

	fromNode := contractor.PriorityQueueNode[int32]{Rank: 0, Item: from}
	toNode := contractor.PriorityQueueNode[int32]{Rank: 0, Item: to}

	forwQ.Insert(fromNode)
	backQ.Insert(toNode)

	estimate := math.MaxFloat64

	bestCommonVertex := int32(0)

	cameFromf := make(map[int32]cameFromPairXCHV)
	cameFromf[from] = cameFromPairXCHV{datastructure.Edge{}, -1, 0, 0}

	cameFromb := make(map[int32]cameFromPairXCHV)
	cameFromb[to] = cameFromPairXCHV{datastructure.Edge{}, -1, 0, 0}

	forwardProcessed := make(map[int32]struct{})
	backwardProcessed := make(map[int32]struct{})
	isForward := true

	stopSearchForward := false
	stopSearchBackward := false

	// https://publikationen.bibliothek.kit.edu/1000028701/142973925 (algorithm 1)
	for (forwQ.Size() != 0 || backQ.Size() != 0) && estimate > min(forwQ.GetMinRank(), backQ.GetMinRank()) &&
		(!stopSearchForward || !stopSearchBackward) {

		if isForward && !stopSearchBackward {
			if backQ.Size() != 0 {
				isForward = false
			}
		} else if !stopSearchForward  {
			if forwQ.Size() != 0 {
				isForward = true
			}
		}



		if isForward  {

			stopSearchForward = rt.searchXCHV(forwQ, df, db, cameFromf, cameFromb, isForward,
				forwardProcessed, backwardProcessed, &estimate, &bestCommonVertex, distf, distb, k, lOpt, pruneRadius)

		} else   {

			stopSearchBackward = rt.searchXCHV(backQ, df, db, cameFromf, cameFromb, isForward,
				forwardProcessed, backwardProcessed, &estimate, &bestCommonVertex, distf, distb, k, lOpt, pruneRadius)
		}
	}

	if estimate == math.MaxFloat64 {
		return []datastructure.CHNode{}, []datastructure.Coordinate{}, []datastructure.Edge{}, -1, -1, nil, nil, -1
	}

	cameFromFCopy := make(map[int32]cameFromPairXCHV)
	for k, v := range cameFromf {
		cameFromFCopy[k] = v
	}
	cameFromBCopy := make(map[int32]cameFromPairXCHV)
	for k, v := range cameFromb {
		cameFromBCopy[k] = v
	}
	path, coordPath, edgePath, eta, dist := rt.createPathXCHV(bestCommonVertex, from, to, cameFromf, cameFromb,
		cameFromFCopy, cameFromBCopy)
	return path, coordPath, edgePath, eta, dist, cameFromf, cameFromb, bestCommonVertex
}

func (rt *RouteAlgorithm) searchXCHV(frontier *contractor.MinHeap[int32], df, db map[int32]float64,
	cameFromf, cameFromb map[int32]cameFromPairXCHV, turnF bool, forwardProcessed, backwardProcessed map[int32]struct{},
	estimate *float64, bestCommonVertex *int32, distf, distb map[int32]float64, k int, lOpt float64, pruneRadius bool) bool {
	node, _ := frontier.ExtractMin()

	if pruneRadius && node.Rank > (1+epsilon)*lOpt {
		return true
	}

	if rt.pruneBD(node.Rank, lOpt, pruneRadius, db, node.Item) {
		return false
	}

	if turnF {

		for _, arc := range rt.ch.GetNodeFirstOutEdges(node.Item) {

			edge := rt.ch.GetOutEdge(arc)

			toNID := edge.ToNodeID
			cost := edge.Weight

			// upward graph
			newCost := cost + df[node.Item]
			newDist := edge.Dist + distf[node.Item]

			if !rt.pruneRelaxedCH(cameFromf, node.Item, toNID, k) {

				_, ok := df[toNID]

				if !ok {
					df[toNID] = newCost

					distf[toNID] = newDist

					neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
					frontier.Insert(neighborNode)
					cameFromf[toNID] = cameFromPairXCHV{edge, node.Item, newDist, newCost}
				} else if newCost < df[toNID] {
					df[toNID] = newCost

					distf[toNID] = newDist

					neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
					frontier.DecreaseKey(neighborNode)

					cameFromf[toNID] = cameFromPairXCHV{edge, node.Item, newDist, newCost}
				}

			}
		}

		forwardProcessed[node.Item] = struct{}{}

		_, ok := backwardProcessed[node.Item]
		if ok {
			pathDistance := df[node.Item] + db[node.Item]
			if pathDistance < *estimate {
				*estimate = pathDistance
				*bestCommonVertex = node.Item
			}
		}

	} else {

		for _, arc := range rt.ch.GetNodeFirstInEdges(node.Item) {

			edge := rt.ch.GetInEdge(arc)

			toNID := edge.ToNodeID
			cost := edge.Weight

			// downward graph
			newCost := cost + db[node.Item]
			newDist := edge.Dist + distb[node.Item]

			if !rt.pruneRelaxedCH(cameFromb, node.Item, toNID, k) {

				_, ok := db[toNID]
				if !ok {
					db[toNID] = newCost

					distb[toNID] = newDist

					neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
					frontier.Insert(neighborNode)
					cameFromb[toNID] = cameFromPairXCHV{edge, node.Item, newDist, newCost}
				}
				if newCost < db[toNID] {
					db[toNID] = newCost

					distb[toNID] = newDist

					neighborNode := contractor.PriorityQueueNode[int32]{Rank: newCost, Item: toNID}
					frontier.DecreaseKey(neighborNode)

					cameFromb[toNID] = cameFromPairXCHV{edge, node.Item, newDist, newCost}
				}

			}
		}

		backwardProcessed[node.Item] = struct{}{}
		_, ok := forwardProcessed[node.Item]
		if ok {
			pathDistance := df[node.Item] + db[node.Item]
			if pathDistance < *estimate {
				*estimate = pathDistance
				*bestCommonVertex = node.Item
			}
		}
	}

	return false
}

func (rt *RouteAlgorithm) pruneBD(newEta, lOpt float64, pruneRadius bool, db map[int32]float64, u int32) bool {

	if pruneRadius {
		distVFromTo, ok := db[u]
		if ok && newEta+distVFromTo > (1+epsilon)*lOpt {
			return true
		}
	}
	return false
}

/*
https://renatowerneck.wordpress.com/wp-content/uploads/2016/06/adgw13-alternatives.pdf
5.3. Relaxed Contraction Hierarchies
The details are as follows. During a query, let pi(u) be the ith ancestor of u in
the search tree: p1(u) is u’s parent, and pi (u) is the parent of pi−1(u) (for i > 1).
The k-relaxed variant of CHV prunes an edge (u, v) only if v precedes all vertices u,
p1 (u), . . . , pk (u) in the CH order. If u has fewer than k ancestors, (u, v) is never pruned

Return true if node edge (u,v) pruned, else return false
*/
func (rt *RouteAlgorithm) pruneRelaxedCH(cameFrom map[int32]cameFromPairXCHV, u int32, v int32, k int,
) bool {

	if rt.ch.GetNode(v).OrderPos > rt.ch.GetNode(u).OrderPos {
		// v not precedes u in the ch order
		return false
	}
	vOrder := rt.ch.GetNode(v).OrderPos

	for i := 1; i <= k; i++ {
		if cameFrom[u].NodeID == -1 {
			// u has fewer than k ancestors, so prune this edge (u,v)
			return true
		}
		u = cameFrom[u].NodeID
		uOrder := rt.ch.GetNode(u).OrderPos
		if vOrder > uOrder {
			// v not precedes all vertices p1(u),p2(u),....,pk(u) in the ch order
			return false
		}
	}

	// v precedes all u,p1(u),...pk(u) in ch order
	return true
}

func (rt *RouteAlgorithm) createPathXCHV(commonVertex int32, from, to int32,
	cameFromf, cameFromb map[int32]cameFromPairXCHV, cameFromfCopy, cameFrombCopy map[int32]cameFromPairXCHV) ([]datastructure.CHNode,
	[]datastructure.Coordinate, []datastructure.Edge, float64, float64) {

	fCoordPath := make([]datastructure.Coordinate, 0)

	fPath := make([]datastructure.CHNode, 0)
	fedgePath := make([]datastructure.Edge, 0)
	eta := 0.0
	dist := 0.0
	v := commonVertex
	if rt.ch.IsTrafficLight(v) {
		eta += trafficLightAdditionalWeight
	}
	ok := true

	for ok && cameFromf[v].NodeID != -1 {
		isShortcut := rt.ch.IsShortcut(cameFromf[v].Edge.EdgeID)

		if isShortcut {

			rt.unpackBackwardXCHV(cameFromf[v].Edge, &fPath, &fedgePath, &eta, &dist, cameFromfCopy,
				&fCoordPath)
		} else {

			if cameFromf[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromf[v].NodeID) {
				eta += trafficLightAdditionalWeight
			}
			eta += cameFromf[v].Edge.Weight
			dist += cameFromf[v].Edge.Dist

			pointsInBetween := make([]datastructure.Coordinate, 0)
			if cameFromf[v].Edge.Weight != 0 {
				fedgePath = append(fedgePath, cameFromf[v].Edge)
				pointsInBetween = rt.ch.GetEdgePointsInBetween(cameFromf[v].Edge.EdgeID)
				pointsInBetween = util.ReverseG(pointsInBetween)
			}

			nodeV := rt.ch.GetNode(cameFromf[v].NodeID)

			fPath = append(fPath, nodeV)
			fCoordPath = append(fCoordPath, pointsInBetween...)
			fCoordPath = append(fCoordPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))
		}
		_, ok = cameFromf[v]
		v = cameFromf[v].NodeID
	}

	bCoordPath := make([]datastructure.Coordinate, 0)
	bPath := make([]datastructure.CHNode, 0)
	bEdgePath := make([]datastructure.Edge, 0)
	v = commonVertex
	ok = true
	for ok && cameFromb[v].NodeID != -1 {

		isShortcut := rt.ch.IsShortcut(cameFromb[v].Edge.EdgeID)
		if isShortcut {

			rt.unpackForwardXCHV(cameFromb[v].Edge, &bPath, &bEdgePath, &eta, &dist, cameFrombCopy,
				&bCoordPath)

		} else {

			if cameFromb[v].NodeID != -1 && rt.ch.IsTrafficLight(cameFromb[v].NodeID) {
				eta += trafficLightAdditionalWeight
			}
			eta += cameFromb[v].Edge.Weight
			dist += cameFromb[v].Edge.Dist

			pointsInBetween := make([]datastructure.Coordinate, 0)
			if cameFromb[v].Edge.Weight != 0 {
				bEdgePath = append(bEdgePath, cameFromb[v].Edge)

				pointsInBetween = rt.ch.GetEdgePointsInBetween(cameFromb[v].Edge.EdgeID)
			}

			nodeV := rt.ch.GetNode(cameFromb[v].NodeID)

			bPath = append(bPath, nodeV)
			bCoordPath = append(bCoordPath, pointsInBetween...)
			bCoordPath = append(bCoordPath, datastructure.NewCoordinate(nodeV.Lat, nodeV.Lon))
		}
		_, ok = cameFromb[v]
		v = cameFromb[v].NodeID
	}

	fPath = util.ReverseG(fPath)
	path := make([]datastructure.CHNode, 0)
	path = append(path, fPath...)
	vianode := rt.ch.GetNode(commonVertex)
	path = append(path, vianode)
	path = append(path, bPath...)

	fCoordPath = util.ReverseG(fCoordPath)

	coordPath := make([]datastructure.Coordinate, 0)
	coordPath = append(coordPath, fCoordPath...)
	coordPath = append(coordPath, datastructure.NewCoordinate(vianode.Lat, vianode.Lon))
	coordPath = append(coordPath, bCoordPath...)

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

	return path, coordPath, edgePath, eta, dist
}

// buat forward dijkstra
// dari common vertex ke source vertex
func (rt *RouteAlgorithm) unpackBackwardXCHV(edge datastructure.Edge, path *[]datastructure.CHNode, ePath *[]datastructure.Edge,
	eta, dist *float64, cameFromf map[int32]cameFromPairXCHV, fCoordPath *[]datastructure.Coordinate) {

	isShortcut := rt.ch.IsShortcut(edge.EdgeID)

	if !isShortcut {
		if rt.ch.IsTrafficLight(edge.FromNodeID) {
			*eta += trafficLightAdditionalWeight
		}
		*eta += edge.Weight
		*dist += edge.Dist

		fromNode := rt.ch.GetNode(edge.FromNodeID)

		pointsInBetween := rt.ch.GetEdgePointsInBetween(edge.EdgeID)
		pointsInBetween = util.ReverseG(pointsInBetween)

		*fCoordPath = append(*fCoordPath, pointsInBetween...)

		*path = append(*path, fromNode)

		*ePath = append(*ePath, edge)

		// for calculating plateau we must update the camefrom to, because all edge in camefrom is a shorcut
		cameFromf[edge.ToNodeID] = cameFromPairXCHV{edge, edge.FromNodeID, *dist, *eta}
	} else {
		var (
			edgeTwo datastructure.Edge
			edgeOne datastructure.Edge
		)

		fromNodeID := edge.FromNodeID
		toNodeID := edge.ToNodeID

		for _, arcID := range rt.ch.GetNodeFirstOutEdges(fromNodeID) {
			arc := rt.ch.GetOutEdge(arcID)
			if arc.ToNodeID == edge.ViaNodeID {
				edgeOne = arc
			}
		}

		for _, arcID := range rt.ch.GetNodeFirstOutEdges(edge.ViaNodeID) {
			arc := rt.ch.GetOutEdge(arcID)
			if arc.ToNodeID == toNodeID {
				edgeTwo = arc
			}
		}

		rt.unpackBackwardXCHV(edgeTwo, path, ePath, eta, dist, cameFromf, fCoordPath)
		rt.unpackBackwardXCHV(edgeOne, path, ePath, eta, dist, cameFromf, fCoordPath)
	}
}

// dari common vertex ke target vertex
func (rt *RouteAlgorithm) unpackForwardXCHV(edge datastructure.Edge, path *[]datastructure.CHNode, ePath *[]datastructure.Edge,
	eta, dist *float64, cameFromb map[int32]cameFromPairXCHV, bCoordPath *[]datastructure.Coordinate) {

	isShortcut := rt.ch.IsShortcut(edge.EdgeID)

	if !isShortcut {
		if rt.ch.IsTrafficLight(edge.FromNodeID) {
			*eta += trafficLightAdditionalWeight
		}
		*eta += edge.Weight
		*dist += edge.Dist

		fromNode := rt.ch.GetNode(edge.FromNodeID)

		pointsInBetween := rt.ch.GetEdgePointsInBetween(edge.EdgeID)

		*bCoordPath = append(*bCoordPath, pointsInBetween...)

		*path = append(*path, fromNode)

		*ePath = append(*ePath, edge)

		cameFromb[edge.ToNodeID] = cameFromPairXCHV{edge, edge.FromNodeID, *dist, *eta}
	} else {

		var (
			edgeTwo datastructure.Edge
			edgeOne datastructure.Edge
		)

		fromNodeID := edge.FromNodeID
		toNodeID := edge.ToNodeID

		for _, arcID := range rt.ch.GetNodeFirstInEdges(edge.ViaNodeID) {
			arc := rt.ch.GetInEdge(arcID)
			if arc.ToNodeID == toNodeID {
				edgeOne = arc
			}
		}

		for _, arcID := range rt.ch.GetNodeFirstInEdges(fromNodeID) {
			arc := rt.ch.GetInEdge(arcID)
			if arc.ToNodeID == edge.ViaNodeID {
				edgeTwo = arc
			}
		}

		rt.unpackForwardXCHV(edgeOne, path, ePath, eta, dist, cameFromb, bCoordPath)
		rt.unpackForwardXCHV(edgeTwo, path, ePath, eta, dist, cameFromb, bCoordPath)

	}
}
