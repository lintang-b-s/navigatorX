package contractor

import (
	"testing"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/navigatorx/pkg/util"

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
	nodeP := datastructure.NewCHNode(1, 1, 0, 0)
	nodeV := datastructure.NewCHNode(1, 1, 0, 1)
	nodeQ := datastructure.NewCHNode(1, 1, 0, 2)
	nodeW := datastructure.NewCHNode(1, 1, 0, 3)
	nodeR := datastructure.NewCHNode(1, 1, 0, 4)

	edgePv := datastructure.NewEdgeCHPlain(0, 10, 0, nodeP.ID, nodeV.ID)

	edgeVp := datastructure.NewEdgeCHPlain(1, 10, 0, nodeV.ID, nodeP.ID)

	edgeVr := datastructure.NewEdgeCHPlain(2, 3, 0, nodeV.ID, nodeR.ID)

	edgeRv := datastructure.NewEdgeCHPlain(3, 3, 0, nodeR.ID, nodeV.ID)

	edgeVq := datastructure.NewEdgeCHPlain(4, 6, 0, nodeV.ID, nodeQ.ID)

	edgeQv := datastructure.NewEdgeCHPlain(5, 6, 0, nodeQ.ID, nodeV.ID)

	edgeQw := datastructure.NewEdgeCHPlain(6, 5, 0, nodeQ.ID, nodeW.ID)

	edgeWq := datastructure.NewEdgeCHPlain(7, 5, 0, nodeW.ID, nodeQ.ID)

	edgeWr := datastructure.NewEdgeCHPlain(8, 5, 0, nodeW.ID, nodeR.ID)

	edgeRw := datastructure.NewEdgeCHPlain(9, 5, 0, nodeR.ID, nodeW.ID)

	edges := []datastructure.EdgeCH{edgePv, edgeVp, edgeVr, edgeRv, edgeVq, edgeQv, edgeQw, edgeWq, edgeWr, edgeRw}

	edgesExtraInfo := make([]datastructure.EdgeExtraInfo, 0)

	for _ = range edges {
		edgesExtraInfo = append(edgesExtraInfo, datastructure.EdgeExtraInfo{})
	}

	streetDirections := make(map[string][2]bool)
	nodes := []datastructure.CHNode{nodeP, nodeV, nodeQ, nodeW, nodeR}
	chGraph.InitCHGraph(nodes, edges, streetDirections, util.NewIdMap(),
		edgesExtraInfo, datastructure.NewNodeInfo())

	return chGraph
}

func TestContractNode(t *testing.T) {
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
		shortcutPQ := datastructure.NewEdgeCHPlain(0, 16, 0, 2, 0)
		shortcutPR := datastructure.NewEdgeCHPlain(0, 13, 0, 4, 0)
		for _, edgeID := range nodePEdges {
			edge := chGraph.GetOutEdge(edgeID)
			isShortcut := chGraph.IsShortcut(edge.FromNodeID, edge.ToNodeID, false)
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
		shortcutQP := datastructure.NewEdgeCHPlain(0, 16, 0, 0, 2)
		shortcutQR := datastructure.NewEdgeCHPlain(0, 9, 0, 4, 2)
		for _, edgeID := range nodeQEdges {
			edge := chGraph.GetOutEdge(edgeID)

			isShortcut := chGraph.IsShortcut(edge.FromNodeID, edge.ToNodeID, false)

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
		shortcutRP := datastructure.NewEdgeCHPlain(0, 13, 0, 0, 4)
		shortcutRQ := datastructure.NewEdgeCHPlain(0, 9, 0, 2, 4)
		for _, edgeID := range nodeREdges {
			edge := chGraph.GetOutEdge(edgeID)
			isShortcut := chGraph.IsShortcut(edge.FromNodeID, edge.ToNodeID, false)

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

func TestContractNodeFromFile(t *testing.T) {
	osmParser := osmparser.NewOSMParserV2()
	processedNodes, processedEdges, streetDirection,
		edgesExtraInfo, nodeInfo := osmParser.Parse("wa-microsoft.osm.pbf")

	chGraph := NewContractedGraph()

	chGraph.InitCHGraph(processedNodes, processedEdges, streetDirection, osmParser.GetTagStringIdMap(),
		edgesExtraInfo, nodeInfo)

}

func NewGraph2() *ContractedGraph {
	chGraph := NewContractedGraph()
	nodeP := datastructure.NewCHNode(1, 1, 0, 0)
	nodeV := datastructure.NewCHNode(1, 1, 0, 1)
	nodeQ := datastructure.NewCHNode(1, 1, 0, 2)
	nodeW := datastructure.NewCHNode(1, 1, 0, 3)
	nodeR := datastructure.NewCHNode(1, 1, 0, 4)
	nodeT := datastructure.NewCHNode(1, 1, 0, 5)

	edgePv := datastructure.NewEdgeCHPlain(0, 10, 0, nodeP.ID, nodeV.ID)

	edgeVp := datastructure.NewEdgeCHPlain(1, 10, 0, nodeV.ID, nodeP.ID)

	edgeVr := datastructure.NewEdgeCHPlain(2, 3, 0, nodeV.ID, nodeR.ID)

	edgeRv := datastructure.NewEdgeCHPlain(3, 3, 0, nodeR.ID, nodeV.ID)

	edgeVq := datastructure.NewEdgeCHPlain(4, 6, 0, nodeV.ID, nodeQ.ID)

	edgeQv := datastructure.NewEdgeCHPlain(5, 6, 0, nodeQ.ID, nodeV.ID)

	edgeQw := datastructure.NewEdgeCHPlain(6, 5, 0, nodeQ.ID, nodeW.ID)

	edgeWq := datastructure.NewEdgeCHPlain(7, 5, 0, nodeW.ID, nodeQ.ID)

	edgeWr := datastructure.NewEdgeCHPlain(8, 5, 0, nodeW.ID, nodeR.ID)

	edgeRw := datastructure.NewEdgeCHPlain(9, 5, 0, nodeR.ID, nodeW.ID)

	edgeTw := datastructure.NewEdgeCHPlain(10, 5, 0, nodeW.ID, nodeT.ID)

	edges := []datastructure.EdgeCH{edgePv, edgeVp, edgeVr, edgeRv, edgeVq, edgeQv, edgeQw, edgeWq, edgeWr, edgeRw, edgeTw}

	edgesExtraInfo := make([]datastructure.EdgeExtraInfo, 0)

	for _ = range edges {
		edgesExtraInfo = append(edgesExtraInfo, datastructure.EdgeExtraInfo{})
	}

	streetDirections := make(map[string][2]bool)
	nodes := []datastructure.CHNode{nodeP, nodeV, nodeQ, nodeW, nodeR, nodeT}
	chGraph.InitCHGraph(nodes, edges, streetDirections, util.NewIdMap(),
		edgesExtraInfo, datastructure.NewNodeInfo())

	return chGraph
}
