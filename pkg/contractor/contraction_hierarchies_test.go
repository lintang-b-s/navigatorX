package contractor

import (
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/util"
	"testing"

	"github.com/stretchr/testify/assert"
)

/*
dari https://jlazarsfeld.github.io/ch.150.project/sections/8-contraction/
p=0, v=1, q=2, w=3, r=4

	 p
	  \
	   \
	    10
	     \
		  v -----3----- r
		 /            /
		6            5
	   /    		/
	  q ---5----- w

semua edge bidirectional

setelah di kontraksi:

	 p ___
	| \   \___
	|  \      \ 13
	|  10          \__
	16   \            \
	|	  v -----3----- r
	|	 /          /  /
	|	6    _  9    5
	|  / _ /   		/
	 q------5----- w
*/
func NewGraph() *ContractedGraph {
	chGraph := NewContractedGraph()
	nodeP := datastructure.NewCHNode(1, 1, 0, 0, true)
	nodeV := datastructure.NewCHNode(1, 1, 0, 1, true)
	nodeQ := datastructure.NewCHNode(1, 1, 0, 2, true)
	nodeW := datastructure.NewCHNode(1, 1, 0, 3, true)
	nodeR := datastructure.NewCHNode(1, 1, 0, 4, true)

	edgePv := datastructure.NewEdgeCH(0, 10, 0, nodeP.ID, nodeV.ID, 0, 0)

	edgeVp := datastructure.NewEdgeCH(1, 10, 0, nodeV.ID, nodeP.ID, 0, 0)

	edgeVr := datastructure.NewEdgeCH(2, 3, 0, nodeV.ID, nodeR.ID, 0, 0)

	edgeRv := datastructure.NewEdgeCH(3, 3, 0, nodeR.ID, nodeV.ID, 0, 0)

	edgeVq := datastructure.NewEdgeCH(4, 6, 0, nodeV.ID, nodeQ.ID, 0, 0)

	edgeQv := datastructure.NewEdgeCH(5, 6, 0, nodeQ.ID, nodeV.ID, 0, 0)

	edgeQw := datastructure.NewEdgeCH(6, 5, 0, nodeQ.ID, nodeW.ID, 0, 0)

	edgeWq := datastructure.NewEdgeCH(7, 5, 0, nodeW.ID, nodeQ.ID, 0, 0)

	edgeWr := datastructure.NewEdgeCH(8, 5, 0, nodeW.ID, nodeR.ID, 0, 0)

	edgeRw := datastructure.NewEdgeCH(9, 5, 0, nodeR.ID, nodeW.ID, 0, 0)

	edges := []datastructure.EdgeCH{edgePv, edgeVp, edgeVr, edgeRv, edgeVq, edgeQv, edgeQw, edgeWq, edgeWr, edgeRw}
	streetDirections := make(map[string][2]bool)
	nodes := []datastructure.CHNode{nodeP, nodeV, nodeQ, nodeW, nodeR}
	chGraph.InitCHGraph(nodes, edges, streetDirections, util.NewIdMap(),
		make([]datastructure.EdgeExtraInfo, 0))

	return chGraph
}

func TestContractOneNode(t *testing.T) {
	chGraph := NewGraph()

	t.Run("contract node  v", func(t *testing.T) {
		contracted := make([]bool, chGraph.GetNumNodes())
		chGraph.contractNode(1, false, contracted) //  id 1 = node V

		outEdgesTotal := len(chGraph.GetOutEdges())
		assert.Equal(t, 16, outEdgesTotal) // 8 * 2 (bidirectional)

		inEdgesTotal := len(chGraph.GetInEdges())
		assert.Equal(t, 16, inEdgesTotal)

		// check node P
		nodePEdges := chGraph.GetNodeFirstOutEdges(0)
		pShortcutCount := 0
		shortcutPQ := datastructure.NewEdgeCH(0, 16, 0, 2, 0, 0, 0)
		shortcutPR := datastructure.NewEdgeCH(0, 13, 0, 4, 0, 0, 0)
		for _, edgeID := range nodePEdges {
			edge := chGraph.GetOutEdge(edgeID)
			isShortcut := chGraph.IsShortcut(edgeID)
			if isShortcut {
				pShortcutCount++
			}

			if isShortcut && edge.ToNodeID == 2 {
				assert.Equal(t, shortcutPQ.Weight, edge.Weight)
			} else if isShortcut && edge.ToNodeID == 4 {
				assert.Equal(t, shortcutPR.Weight, edge.Weight)
			}
		}
		assert.Equal(t, 2, pShortcutCount)

		// check node Q
		nodeQEdges := chGraph.GetNodeFirstOutEdges(2)
		qShortcutCount := 0
		shortcutQP := datastructure.NewEdgeCH(0, 16, 0, 0, 2, 0, 0)
		shortcutQR := datastructure.NewEdgeCH(0, 9, 0, 4, 2, 0, 0)
		for _, edgeID := range nodeQEdges {
			edge := chGraph.GetOutEdge(edgeID)
			isShortcut := chGraph.IsShortcut(edgeID)

			if isShortcut {
				qShortcutCount++
			}

			if isShortcut && edge.ToNodeID == 0 {
				assert.Equal(t, shortcutQP.Weight, edge.Weight)
			} else if isShortcut && edge.ToNodeID == 4 {
				assert.Equal(t, shortcutQR.Weight, edge.Weight)
			}
		}
		assert.Equal(t, 2, qShortcutCount)

		// check node R
		nodeREdges := chGraph.GetNodeFirstOutEdges(4)
		rShortcutCount := 0
		shortcutRP := datastructure.NewEdgeCH(0, 13, 0, 0, 4, 0, 0)
		shortcutRQ := datastructure.NewEdgeCH(0, 9, 0, 2, 4, 0, 0)
		for _, edgeID := range nodeREdges {
			edge := chGraph.GetOutEdge(edgeID)
			isShortcut := chGraph.IsShortcut(edgeID)

			if isShortcut {
				rShortcutCount++
			}

			if isShortcut && edge.ToNodeID == 0 {
				assert.Equal(t, shortcutRP.Weight, edge.Weight)
			} else if isShortcut && edge.ToNodeID == 2 {
				assert.Equal(t, shortcutRQ.Weight, edge.Weight)
			}
		}
		assert.Equal(t, 2, rShortcutCount)
	})
}
