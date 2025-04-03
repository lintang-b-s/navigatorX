package contractor

import (
	"log"

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

	for _, v := range order {
		if !visited[v] {
			component := make([]int32, 0)
			ch.dfs(v, &component, visited, true)
			components = append(components, component)
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
