package routingalgorithm

import (
	"context"
	"fmt"
	"sort"

	"github.com/lintang-b-s/navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"
)

// https://www.microsoft.com/en-us/research/publication/alternative-routes-in-road-networks/
// X-CHV

type RouteAlgorithmI interface {
	ShortestPathBiDijkstraXCHV(from, to int32) ([]datastructure.CHNode,
		[]datastructure.Coordinate, []datastructure.Edge, float64, float64,
		map[int32]cameFromPairXCHV, map[int32]cameFromPairXCHV, int32)
}

type AlternativeRouteXCHV struct {
	rt      RouteAlgorithmI
	gamma   float64
	epsilon float64
	alpha   float64
	maxK    int
}

// always use default param. maxK = maximum number of alternative routes
func NewAlternativeRouteXCHV(k int, rt RouteAlgorithmI) *AlternativeRouteXCHV {
	if k == 0 {
		k = 2
	}
	return &AlternativeRouteXCHV{
		gamma:   0.8,
		epsilon: 0.25,
		alpha:   0.25,
		maxK:    k,
		rt:      rt,
	}
}

func (ar *AlternativeRouteXCHV) RunAlternativeRouteXCHV(from, to int32) ([]datastructure.AlternativeRouteInfo, error) {
	// first run the shortest path (s,t)
	nodes, path, edges, bestEta, bestDist, camefromf, camefromb, _ := ar.rt.ShortestPathBiDijkstraXCHV(from, to)

	if bestEta == -1 {
		return []datastructure.AlternativeRouteInfo{}, fmt.Errorf("no path found from %d to %d", from, to)
	}

	potentialRoutes := make([]datastructure.AlternativeRouteInfo, 0)
	for vnode := range camefromf {
		// get all via node (node that visited on forward and backward search)
		_, ok := camefromb[vnode]
		if !ok {
			continue
		}

		// use stopping criteria from x-bdv
		// prune any vertex u with dist(s, u) + dist(u, t) > (1 + epsilon)l(Opt).
		if camefromf[vnode].Dist+camefromb[vnode].Dist > (1+ar.epsilon)*bestDist {
			continue
		}

		edgeF := ar.buildViaNodeEdges(camefromf, vnode)
		edgeB := ar.buildViaNodeEdges(camefromb, vnode)
		vEdges := append(edgeF, edgeB...)
		// calculate approximate value of σ(v)
		sharedDistanceWithOpt := ar.calculateDistanceShare(nodes, vEdges)

		// for calculating plateau, we use camefrom from s-t shortest-path tree
		plateau := ar.calculatePlateauContainingV(camefromf, camefromb, vnode)

		objectiveValue := ar.calculateObjectiveFunctionValue(camefromf[vnode].Dist+camefromb[vnode].Dist,
			sharedDistanceWithOpt, plateau)
		alternativeRoute := datastructure.NewAlternativeRouteInfo(objectiveValue, vnode)

		potentialRoutes = append(potentialRoutes, alternativeRoute)
	}

	sort.Slice(potentialRoutes, func(i, j int) bool {
		// sort increasing by objective function value
		return potentialRoutes[i].ObjectiveValue < potentialRoutes[j].ObjectiveValue
	})

	alternativeRoutes := make([]datastructure.AlternativeRouteInfo, 0, len(potentialRoutes))
	// append best shortest path to alternative routes

	bestRoute := datastructure.NewAlternativeRouteInfo(bestDist, -1)
	bestRoute.Nodes = nodes
	bestRoute.Edges = edges
	bestRoute.Path = path
	alternativeRoutes = append(alternativeRoutes, bestRoute)
	alternativeRoutes[0].Dist = bestDist
	alternativeRoutes[0].Eta = bestEta

	workers := concurrent.NewWorkerPool[concurrent.AlternativeRouteParam, admisibleTestResult](10,
		len(potentialRoutes))

	duplicateRoutes := make(map[float64]struct{})

	k := 1
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	for _, alternativeRoute := range potentialRoutes {
		vnode := alternativeRoute.ViaNode

		workers.AddJob(concurrent.NewAlternativeRouteParam(
			from, to, vnode, nodes, alternativeRoute, bestDist, ctx))
	}

	workers.Close()
	workers.Start(ar.RunAdmisibleTestXCHV)

	workers.Wait()

	for admisibleTestResultItem := range workers.CollectResults() {
		passAdmisibleTest := admisibleTestResultItem.pass
		if passAdmisibleTest {

			if _, ok := duplicateRoutes[admisibleTestResultItem.alternativeRoute.Eta]; ok {
				continue
			}
			duplicateRoutes[admisibleTestResultItem.alternativeRoute.Eta] = struct{}{}

			alternativeRoutes = append(alternativeRoutes, admisibleTestResultItem.alternativeRoute)
			k++
		}
		if k >= ar.maxK {
			return alternativeRoutes, nil
		}
	}

	return alternativeRoutes, nil
}

func (ar *AlternativeRouteXCHV) RunAdmisibleTestXCHV(param concurrent.AlternativeRouteParam) admisibleTestResult {
	from := param.FromNodeID
	to := param.ToNodeID
	vnode := param.VNodeID
	optNodes := param.OptNodes
	optDist := param.OptDist
	alternativeRoute := param.AlternativeRoute
	ctx := param.Ctx

	if util.StopConcurrectOperation(ctx) {
		return newAdmisibleTestResult(false, datastructure.AlternativeRouteInfo{})
	}
	// calculate Pv (sv + vt shortest path) to obtain the actual values of l(v) and σ(v)
	svNodes, svPath, svEdges, svEta, svDist, _, _, _ := ar.rt.ShortestPathBiDijkstraXCHV(from, vnode)

	if util.StopConcurrectOperation(ctx) {
		return newAdmisibleTestResult(false, datastructure.AlternativeRouteInfo{})
	}
	vtNodes, vtPath, vtEdges, vtEta, vtDist, _, _, _ := ar.rt.ShortestPathBiDijkstraXCHV(vnode, to)

	if util.StopConcurrectOperation(ctx) {
		return newAdmisibleTestResult(false, datastructure.AlternativeRouteInfo{})
	}
	if vtDist == -1 || svDist == -1 {

		return newAdmisibleTestResult(false, datastructure.AlternativeRouteInfo{})
	}

	pvEdges := append(svEdges, vtEdges...)
	pvDist := svDist + vtDist
	pvEta := svEta + vtEta

	pvNodes := append(svNodes, vtNodes...)

	// calculate actual value of σ(v)
	sharedDistanceWithOpt := ar.calculateDistanceShare(optNodes, pvEdges)

	// check wether l(Pv \ Opt) < (1 + epsilon)l(Opt \ Pv)

	lengthPvExcludeOpt := pvDist - sharedDistanceWithOpt
	lengthOptExcludePv := optDist - sharedDistanceWithOpt
	if lengthPvExcludeOpt >= (1+ar.epsilon)*lengthOptExcludePv {
		return newAdmisibleTestResult(false, datastructure.AlternativeRouteInfo{})
	}

	// check wether σ(v) < γ · l(Opt)
	if sharedDistanceWithOpt/optDist >= ar.gamma {
		return newAdmisibleTestResult(false, datastructure.AlternativeRouteInfo{})
	}

	edgeIDContainV := searchVNodeInedges(pvEdges, vnode)

	// run a T-test with T = α · l(Pv \ Opt)
	if !ar.tTest(lengthPvExcludeOpt, edgeIDContainV, pvEdges,
		vnode) || edgeIDContainV == -1 {
		return newAdmisibleTestResult(false, datastructure.AlternativeRouteInfo{})
	}

	alternativeRoute.Eta = pvEta
	alternativeRoute.Dist = pvDist
	alternativeRoute.Nodes = pvNodes
	alternativeRoute.Edges = pvEdges
	alternativeRoute.Path = append(svPath, vtPath...)

	return newAdmisibleTestResult(true, alternativeRoute)
}

type admisibleTestResult struct {
	pass             bool
	alternativeRoute datastructure.AlternativeRouteInfo
}

func newAdmisibleTestResult(pass bool, alternativeRoute datastructure.AlternativeRouteInfo) admisibleTestResult {
	return admisibleTestResult{
		pass:             pass,
		alternativeRoute: alternativeRoute,
	}
}

func (ar *AlternativeRouteXCHV) buildViaNodeEdges(cameFrom map[int32]cameFromPairXCHV, viaNode int32) []datastructure.Edge {
	edges := make([]datastructure.Edge, 0)

	v := viaNode
	val := cameFrom[v]
	for val.NodeID != -1 {
		edges = append(edges, datastructure.Edge{
			FromNodeID: val.NodeID,
			ToNodeID:   v,
			Dist:       val.Dist,
		})
		v = val.NodeID
		val = cameFrom[v]
	}
	return edges
}

func (ar *AlternativeRouteXCHV) calculateDistanceShare(nodesOne []datastructure.CHNode,
	edgesTwo []datastructure.Edge) float64 {

	sharedDistance := 0.0

	// build nodesOne set
	nodesOneSet := make(map[int32]struct{})
	for _, node := range nodesOne {
		nodesOneSet[node.ID] = struct{}{}
	}

	for _, edge := range edgesTwo {
		_, fromNodeInPathOne := nodesOneSet[edge.FromNodeID]
		_, toNodeInPathOne := nodesOneSet[edge.ToNodeID]
		if fromNodeInPathOne && toNodeInPathOne {
			sharedDistance += edge.Dist
		}
	}

	// calculate the weight share
	return sharedDistance
}

/*
	It starts by building shortest path trees from s and to t. It then looks at plateaus, i.e., maximal paths that appear in both trees simultaneously.

In general, a plateau u–w gives a natural s–t path: follow the out-tree from s to u, then
the plateau, then the in-tree from w to t. The algorithm selects paths corresponding to
long plateaus, orders them according to some “goodness” criteria
*/
func (ar *AlternativeRouteXCHV) calculatePlateauContainingV(vCameFromf, vCameFromb map[int32]cameFromPairXCHV,
	vNode int32) float64 {
	plateau := 0.0

	// https://drops.dagstuhl.de/storage/01oasics/oasics-vol033-atmos2013/OASIcs.ATMOS.2013.108/OASIcs.ATMOS.2013.108.pdf (figure 1 & 2)

	// we perform backtrack from v to s, then for every node in the backtrack path
	// check if this node is also visited in backward search if yes, add the edge dist to plateau
	// and continue backtrack to the parent node, else break
	parentFrom := vCameFromf[vNode].NodeID
	parentFromEdgeDist := vCameFromf[vNode].Dist
	for parentFrom != -1 {
		// for a node the be included in plateau, the node must be traversed in both forward and backward search

		// check if parentFrom is visited in backward search
		if _, ok := vCameFromb[parentFrom]; !ok {
			break
		}

		plateau += parentFromEdgeDist

		parentFrom = vCameFromf[parentFrom].NodeID
		parentFromEdgeDist = vCameFromf[parentFrom].Dist
	}

	// then we perform backtrack from v to t, then for every node in the backtrack path
	// check if this node is also visited in forward search if yes, add the edge dist to plateau
	// and continue backtrack to the parent node, else break
	parentTo := vCameFromb[vNode].NodeID
	parentToEdgeDist := vCameFromb[vNode].Dist
	for parentTo != -1 {
		if _, ok := vCameFromf[parentTo]; !ok {
			break
		}

		plateau += parentToEdgeDist

		parentTo = vCameFromb[parentTo].NodeID
		parentToEdgeDist = vCameFromb[parentTo].Dist
	}

	return plateau
}

func (ar *AlternativeRouteXCHV) calculateObjectiveFunctionValue(pvLength, sharedEdgesDist, plateau float64) float64 {
	return 2*(pvLength) + sharedEdgesDist - plateau
}
func (ar *AlternativeRouteXCHV) tTest(lengthPvExcludeOpt float64, edgeIDContainV int,
	pvEdges []datastructure.Edge, vNode int32) bool {
	var (
		xNode int32
		yNode int32
	)
	T := ar.alpha * lengthPvExcludeOpt

	distancef := 0.0
	// Among all vertices in P1 that are at least T away from v, let x be the closest to v
	for edgeIDContainV-1 >= 0 && distancef < T {
		distancef += pvEdges[edgeIDContainV-1].Dist
		xNode = pvEdges[edgeIDContainV-1].FromNodeID

		edgeIDContainV--
	}

	distanceb := 0.0
	// Among all vertices in P2 that are at least T away from v, let y be the closest to v
	for edgeIDContainV < len(pvEdges) && distanceb < T {
		distanceb += pvEdges[edgeIDContainV].Dist
		yNode = pvEdges[edgeIDContainV].ToNodeID

		edgeIDContainV++
	}

	// v pass the T-test if shortest path from x to y contains v

	nodes, _, _, _, _, _, _, _ := ar.rt.ShortestPathBiDijkstraXCHV(xNode, yNode)

	for _, node := range nodes {
		if node.ID == vNode {
			return true
		}
	}
	return false
}

func searchVNodeInedges(edges []datastructure.Edge, nodeID int32) int {

	for i := 0; i < len(edges); i++ {
		if edges[i].FromNodeID == nodeID {
			return i
		}
	}

	return -1
}
