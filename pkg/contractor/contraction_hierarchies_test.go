package contractor

import (
	"lintang/navigatorx/pkg/datastructure"
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
	maxSpeed := 0.06
	nodeP := datastructure.NewNode(1, 1, 0, "P", false, 2)
	nodeV := datastructure.NewNode(1, 1, 1, "V", false, 2)
	nodeQ := datastructure.NewNode(1, 1, 2, "Q", false, 2)
	nodeW := datastructure.NewNode(1, 1, 3, "W", false, 2)
	nodeR := datastructure.NewNode(1, 1, 4, "R", false, 2)

	edgePv := datastructure.NewEdge(nodeP, nodeV, 10, "p->v", maxSpeed, false, "primary", "primary", 2, nil)
	nodeP.AddEdge(edgePv)

	edgeVp := datastructure.NewEdge(nodeV, nodeP, 10, "v->p", maxSpeed, false, "primary", "primary", 2, nil)
	nodeV.AddEdge(edgeVp)

	edgeVr := datastructure.NewEdge(nodeV, nodeR, 3, "v->r", maxSpeed, false, "primary", "primary", 2, nil)
	nodeV.AddEdge(edgeVr)

	edgeRv := datastructure.NewEdge(nodeR, nodeV, 3, "r->v", maxSpeed, false, "primary", "primary", 2, nil)
	nodeR.AddEdge(edgeRv)

	edgeVq := datastructure.NewEdge(nodeV, nodeQ, 6, "v->q", maxSpeed, false, "primary", "primary", 2, nil)
	nodeV.AddEdge(edgeVq)

	edgeQv := datastructure.NewEdge(nodeQ, nodeV, 6, "q->v", maxSpeed, false, "primary", "primary", 2, nil)
	nodeQ.AddEdge(edgeQv)

	edgeQw := datastructure.NewEdge(nodeQ, nodeW, 5, "q->w", maxSpeed, false, "primary", "primary", 2, nil)
	nodeQ.AddEdge(edgeQw)

	edgeWq := datastructure.NewEdge(nodeW, nodeQ, 5, "w->q", maxSpeed, false, "primary", "primary", 2, nil)
	nodeW.AddEdge(edgeWq)

	edgeWr := datastructure.NewEdge(nodeW, nodeR, 5, "w->r", maxSpeed, false, "primary", "primary", 2, nil)
	nodeW.AddEdge(edgeWr)

	edgeRw := datastructure.NewEdge(nodeR, nodeW, 5, "r->w", maxSpeed, false, "primary", "primary", 2, nil)
	nodeR.AddEdge(edgeRw)

	streetDirections := make(map[string][2]bool)
	streetExtraInfo := make(map[string]datastructure.StreetExtraInfo)
	nodes := []datastructure.Node{*nodeP, *nodeV, *nodeQ, *nodeW, *nodeR}
	chGraph.InitCHGraph(nodes, 5, streetDirections, streetExtraInfo)

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
		nodePEdges := chGraph.GetFirstOutEdge(0)
		pShortcutCount := 0
		shortcutPQ := datastructure.NewEdgeCH(0, 16, 0, 2, 0, true, 0, 0, 0, false, 0, 0, 0, nil)
		shortcutPR := datastructure.NewEdgeCH(0, 13, 0, 4, 0, true, 0, 0, 0, false, 0, 0, 0, nil)
		for _, edgeID := range nodePEdges {
			edge := chGraph.GetOutEdge(edgeID)
			if edge.IsShortcut {
				pShortcutCount++
			}

			if edge.IsShortcut && edge.ToNodeIDX == 2 {
				assert.Equal(t, shortcutPQ.Weight, edge.Weight)
			} else if edge.IsShortcut && edge.ToNodeIDX == 4 {
				assert.Equal(t, shortcutPR.Weight, edge.Weight)
			}
		}
		assert.Equal(t, 2, pShortcutCount)

		// check node Q
		nodeQEdges := chGraph.GetFirstOutEdge(2)
		qShortcutCount := 0
		shortcutQP := datastructure.NewEdgeCH(0, 16, 0, 0, 2, true, 0, 0, 0, false, 0, 0, 0, nil)
		shortcutQR := datastructure.NewEdgeCH(0, 9, 0, 4, 2, true, 0, 0, 0, false, 0, 0, 0, nil)
		for _, edgeID := range nodeQEdges {
			edge := chGraph.GetOutEdge(edgeID)
			if edge.IsShortcut {
				qShortcutCount++
			}

			if edge.IsShortcut && edge.ToNodeIDX == 0 {
				assert.Equal(t, shortcutQP.Weight, edge.Weight)
			} else if edge.IsShortcut && edge.ToNodeIDX == 4 {
				assert.Equal(t, shortcutQR.Weight, edge.Weight)
			}
		}
		assert.Equal(t, 2, qShortcutCount)

		// check node R
		nodeREdges := chGraph.GetFirstOutEdge(4)
		rShortcutCount := 0
		shortcutRP := datastructure.NewEdgeCH(0, 13, 0, 0, 4, true, 0, 0, 0, false, 0, 0, 0, nil)
		shortcutRQ := datastructure.NewEdgeCH(0, 9, 0, 2, 4, true, 0, 0, 0, false, 0, 0, 0, nil)
		for _, edgeID := range nodeREdges {
			edge := chGraph.GetOutEdge(edgeID)
			if edge.IsShortcut {
				rShortcutCount++
			}

			if edge.IsShortcut && edge.ToNodeIDX == 0 {
				assert.Equal(t, shortcutRP.Weight, edge.Weight)
			} else if edge.IsShortcut && edge.ToNodeIDX == 2 {
				assert.Equal(t, shortcutRQ.Weight, edge.Weight)
			}
		}
		assert.Equal(t, 2, rShortcutCount)
	})
}
