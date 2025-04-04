package contractor

import (
	"log"
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/util"
)

func (ch *ContractedGraph) kosarajuSCC() [][]int32 {
	n := int32(len(ch.ContractedNodes))
	components := make([][]int32, 0)

	order := make([]int32, 0, n)
	visited := make([]bool, n)

	for i := int32(0); i < n; i++ {
		if !visited[i] {
			ch.dfs(i, &order, visited, false)
		}
	}

	order = util.ReverseG[int32](order)

	// reset visited
	visited = make([]bool, n)

	roots := make([]int32, n)

	for _, v := range order {
		if !visited[v] {
			component := make([]int32, 0)
			ch.dfs(v, &component, visited, true)
			components = append(components, component)
			root := int32(math.MaxInt32)
			for _, node := range component {
				if node < root {
					root = node
				}
			}

			for _, node := range component {
				roots[node] = root
			}

		}
	}

	// add edges to condensation graph
	condAdj := make([][]int32, n)
	for v := int32(0); v < n; v++ {
		for _, outEdgeID := range ch.GetNodeFirstOutEdges(v) {
			outEdge := ch.GetOutEdge(outEdgeID)
			toNodeID := outEdge.ToNodeID

			if roots[v] != roots[toNodeID] {
				condAdj[roots[v]] = append(condAdj[roots[v]], roots[toNodeID])
			}
		}
	}

	log.Printf("Strongly Connected Components Count: %d\n", len(components))

	ch.SCC = make([]int32, n)
	ch.SCCNodesCount = make([]int32, len(components))
	for i, component := range components {
		for _, v := range component {
			ch.SCC[v] = int32(i)
		}
		ch.SCCNodesCount[i] = int32(len(component))
	}

	ch.SCCCondensationAdj = make([][]int32, len(components))
	for fromRootID, adjRootIDs := range condAdj {
		sccOfV := ch.SCC[fromRootID]
		for _, adjRootID := range adjRootIDs {
			sccOfAdjRootID := ch.SCC[adjRootID]
			ch.SCCCondensationAdj[sccOfV] = append(ch.SCCCondensationAdj[sccOfV], sccOfAdjRootID)
		}
	}

	return components
}

func (ch *ContractedGraph) dfs(v int32, output *[]int32,
	visited []bool, reversed bool) {
	visited[v] = true

	if !reversed {
		for _, outEdgeID := range ch.GetNodeFirstOutEdges(v) {
			outEdge := ch.GetOutEdge(outEdgeID)
			if !visited[outEdge.ToNodeID] {
				ch.dfs(outEdge.ToNodeID, output, visited, reversed)
			}
		}

	} else {
		for _, inEdgeID := range ch.GetNodeFirstInEdges(v) {
			inEdge := ch.GetInEdge(inEdgeID)
			if !visited[inEdge.ToNodeID] {
				ch.dfs(inEdge.ToNodeID, output, visited, reversed)
			}
		}
	}

	*output = append(*output, v)
}
