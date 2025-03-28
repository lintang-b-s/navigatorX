package contractor

import (
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/osmparser"
	"lintang/navigatorx/pkg/util"
	"sort"
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
	nodeP := datastructure.NewCHNode(1, 1, 0, 0)
	nodeV := datastructure.NewCHNode(1, 1, 0, 1)
	nodeQ := datastructure.NewCHNode(1, 1, 0, 2)
	nodeW := datastructure.NewCHNode(1, 1, 0, 3)
	nodeR := datastructure.NewCHNode(1, 1, 0, 4)

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

	t.Run("contraction", func(t *testing.T) {
		chGraph.Contraction()

		chGraph.BuildCompressedSparseRow()

		v := int32(len(chGraph.GetNodes()))
		for a := int32(1); a <= v; a++ {
			begin := chGraph.CsrN[a]
			end := chGraph.CsrN[a+1]
			outEdgesCsr := chGraph.CsrF[begin:end]

			outEdgesCHIDs := chGraph.GetNodeFirstOutEdges(a - 1)
			outEdgesCH := make([]datastructure.EdgeCH, len(outEdgesCHIDs))
			for i, edgeID := range outEdgesCHIDs {
				outEdgesCH[i] = chGraph.GetOutEdge(edgeID)
			}

			sort.Slice(outEdgesCH, func(i, j int) bool {
				return outEdgesCH[i].ToNodeID < outEdgesCH[j].ToNodeID
			})

			for i := 0; i < len(outEdgesCH); i++ {
				assert.Equal(t, outEdgesCH[i].ToNodeID+1, int32(outEdgesCsr[i]))
			}

			begin = chGraph.CsrNRev[a]
			end = chGraph.CsrNRev[a+1]

			inEdgesCsr := chGraph.CsrFRev[begin:end]
			inEdgesCHIDs := chGraph.GetNodeFirstInEdges(a - 1)
			inEdgesCH := make([]datastructure.EdgeCH, len(inEdgesCHIDs))

			for i, edgeID := range inEdgesCHIDs {
				inEdgesCH[i] = chGraph.GetInEdge(edgeID)
			}

			sort.Slice(inEdgesCH, func(i, j int) bool {
				return inEdgesCH[i].ToNodeID < inEdgesCH[j].ToNodeID
			})

			for i := 0; i < len(inEdgesCH); i++ {
				assert.Equal(t, inEdgesCH[i].ToNodeID+1, int32(inEdgesCsr[i]))
			}

		}
	})
}

func TestContractNodeFromFile(t *testing.T) {
	osmParser := osmparser.NewOSMParserV2()
	processedNodes, processedEdges, streetDirection,
		edgesExtraInfo, nodeInfo := osmParser.Parse("wa-microsoft.osm.pbf")

	chGraph := NewContractedGraph()

	chGraph.InitCHGraph(processedNodes, processedEdges, streetDirection, osmParser.GetTagStringIdMap(),
		edgesExtraInfo, nodeInfo)

	t.Run("contraction", func(t *testing.T) {
		chGraph.Contraction()

		chGraph.BuildCompressedSparseRow()

		v := int32(len(chGraph.GetNodes()))
		for a := int32(1); a <= v; a++ {
			begin := chGraph.CsrN[a]
			end := chGraph.CsrN[a+1]
			outEdgesCsr := chGraph.CsrF[begin:end]

			outEdgesCHIDs := chGraph.GetNodeFirstOutEdges(a - 1)
			outEdgesCH := make([]datastructure.EdgeCH, len(outEdgesCHIDs))
			for i, edgeID := range outEdgesCHIDs {
				outEdgesCH[i] = chGraph.GetOutEdge(edgeID)
			}

			sort.Slice(outEdgesCH, func(i, j int) bool {
				return outEdgesCH[i].ToNodeID < outEdgesCH[j].ToNodeID
			})

			for i := 0; i < len(outEdgesCH); i++ {
				assert.Equal(t, outEdgesCH[i].ToNodeID+1, int32(outEdgesCsr[i]))
			}

			begin = chGraph.CsrNRev[a]
			end = chGraph.CsrNRev[a+1]

			inEdgesCsr := chGraph.CsrFRev[begin:end]
			inEdgesCHIDs := chGraph.GetNodeFirstInEdges(a - 1)
			inEdgesCH := make([]datastructure.EdgeCH, len(inEdgesCHIDs))

			for i, edgeID := range inEdgesCHIDs {
				inEdgesCH[i] = chGraph.GetInEdge(edgeID)
			}

			sort.Slice(inEdgesCH, func(i, j int) bool {
				return inEdgesCH[i].ToNodeID < inEdgesCH[j].ToNodeID
			})

			for i := 0; i < len(inEdgesCH); i++ {
				assert.Equal(t, inEdgesCH[i].ToNodeID+1, int32(inEdgesCsr[i]))
			}

		}
	})
}

func NewGraph2() *ContractedGraph {
	chGraph := NewContractedGraph()
	nodeP := datastructure.NewCHNode(1, 1, 0, 0)
	nodeV := datastructure.NewCHNode(1, 1, 0, 1)
	nodeQ := datastructure.NewCHNode(1, 1, 0, 2)
	nodeW := datastructure.NewCHNode(1, 1, 0, 3)
	nodeR := datastructure.NewCHNode(1, 1, 0, 4)
	nodeT := datastructure.NewCHNode(1, 1, 0, 5)

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

	edgeTw := datastructure.NewEdgeCH(10, 5, 0, nodeW.ID, nodeT.ID, 0, 0)

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

func Test_BuildCompressedSparseRow(t *testing.T) {
	chGraph := NewGraph2()
	t.Run("contraction", func(t *testing.T) {
		chGraph.Contraction()

		chGraph.BuildCompressedSparseRow()

		v := int32(len(chGraph.GetNodes()))
		for a := int32(1); a <= v; a++ {
			begin := chGraph.CsrN[a]
			end := chGraph.CsrN[a+1]
			outEdgesCsr := chGraph.CsrF[begin:end]

			outEdgesCHIDs := chGraph.GetNodeFirstOutEdges(a - 1)
			outEdgesCH := make([]datastructure.EdgeCH, len(outEdgesCHIDs))
			for i, edgeID := range outEdgesCHIDs {
				outEdgesCH[i] = chGraph.GetOutEdge(edgeID)
			}

			sort.Slice(outEdgesCH, func(i, j int) bool {
				return outEdgesCH[i].ToNodeID < outEdgesCH[j].ToNodeID
			})

			for i := 0; i < len(outEdgesCH); i++ {
				assert.Equal(t, outEdgesCH[i].ToNodeID+1, int32(outEdgesCsr[i]))
			}

			begin = chGraph.CsrNRev[a]
			end = chGraph.CsrNRev[a+1]

			inEdgesCsr := chGraph.CsrFRev[begin:end]
			inEdgesCHIDs := chGraph.GetNodeFirstInEdges(a - 1)
			inEdgesCH := make([]datastructure.EdgeCH, len(inEdgesCHIDs))

			for i, edgeID := range inEdgesCHIDs {
				inEdgesCH[i] = chGraph.GetInEdge(edgeID)
			}

			sort.Slice(inEdgesCH, func(i, j int) bool {
				return inEdgesCH[i].ToNodeID < inEdgesCH[j].ToNodeID
			})

			for i := 0; i < len(inEdgesCH); i++ {
				assert.Equal(t, inEdgesCH[i].ToNodeID+1, int32(inEdgesCsr[i]))
			}

			// check real csr edges is the weight,dist,streetname,etc same with the original edge
			csrOutToNodeIDs, csrOutEdges := chGraph.GetNodeOutEdgesCsr(a)
			for i := 0; i < len(outEdgesCH); i++ {
				outEdgeCH := outEdgesCH[i]
				csrEdge := csrOutEdges[i]
				assert.Equal(t, outEdgeCH.Weight, csrEdge.Weight)
				assert.Equal(t, outEdgeCH.Dist, csrEdge.Dist)

				// chEdgeExtraInfo := chGraph.GetEdgeExtraInfo(outEdgeCH.EdgeID)
				// assert.Equal(t, chEdgeExtraInfo.StreetName, csrEdge.StreetName)
				// assert.Equal(t, chEdgeExtraInfo.RoadClass, csrEdge.RoadClass)
				// assert.Equal(t, chEdgeExtraInfo.RoadClassLink, csrEdge.RoadClassLink)
				// assert.Equal(t, chEdgeExtraInfo.PointsInBetween, csrEdge.PointsInBetween)

				toNodeID := csrOutToNodeIDs[i]
				assert.Equal(t, outEdgeCH.ToNodeID+1, toNodeID)
			}

			csrInToNodeIDs, csrInEdges := chGraph.GetNodeInEdgesCsr(a)
			for i := 0; i < len(inEdgesCH); i++ {
				inEdgeCH := inEdgesCH[i]
				csrEdge := csrInEdges[i]
				assert.Equal(t, inEdgeCH.Weight, csrEdge.Weight)
				assert.Equal(t, inEdgeCH.Dist, csrEdge.Dist)

				// chEdgeExtraInfo := chGraph.GetEdgeExtraInfo(inEdgeCH.EdgeID)
				// assert.Equal(t, chEdgeExtraInfo.StreetName, csrEdge.StreetName)
				// assert.Equal(t, chEdgeExtraInfo.RoadClass, csrEdge.RoadClass)
				// assert.Equal(t, chEdgeExtraInfo.RoadClassLink, csrEdge.RoadClassLink)
				// assert.Equal(t, chEdgeExtraInfo.PointsInBetween, csrEdge.PointsInBetween)

				toNodeID := csrInToNodeIDs[i]
				assert.Equal(t, inEdgeCH.ToNodeID+1, toNodeID)
			}
		}
	})
}
