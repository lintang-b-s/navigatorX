package contractor

import (
	"bytes"
	"encoding/gob"
	"errors"
	"io"
	"log"
	"os"
	"runtime"
	"sort"
	"time"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/geo"
	"github.com/lintang-b-s/navigatorx/pkg/server"
	"github.com/lintang-b-s/navigatorx/pkg/storage"
	"github.com/lintang-b-s/navigatorx/pkg/storage/buffer"
	"github.com/lintang-b-s/navigatorx/pkg/storage/disk"
	"github.com/lintang-b-s/navigatorx/pkg/util"
	"github.com/mmcloughlin/geohash"

	logStorage "github.com/lintang-b-s/navigatorx/pkg/storage/log"
)

type Metadata struct {
	MeanDegree       float64
	ShortcutsCount   int64
	degrees          []int
	InEdgeOrigCount  []int
	OutEdgeOrigCount []int
	EdgeCount        int
	NodeCount        int
}

type ContractedGraph struct {
	Metadata               Metadata
	Ready                  bool
	ContractedFirstOutEdge [][]int32
	ContractedFirstInEdge  [][]int32
	ContractedOutEdges     []datastructure.EdgeCH
	ContractedInEdges      []datastructure.EdgeCH
	ContractedNodes        []datastructure.CHNode

	nextEdgeID int32

	// graph storage

	NodeIDBlockIDMap  map[int32]int
	bufferPoolManager BufferPoolManager
	diskManager       *disk.DiskManager
	logManager        *logStorage.LogManager
	CompressedBlock   map[int]bool

	// edge extra info
	MapEdgeInfo datastructure.MapEdgeInfo
	// node extra info
	NodeInfo *datastructure.NodeInfo

	StreetDirection map[int][2]bool // 0 = forward, 1 = backward
	TagStringIDMap  util.IDMap
}

var maxPollFactorHeuristic = 5
var maxPollFactorContraction = 200

func NewContractedGraph() *ContractedGraph {
	dm := disk.NewDiskManager(storage.DB_DIR, storage.MAX_PAGE_SIZE)
	lm, err := logStorage.NewLogManager(dm, storage.LOG_FILE_NAME)
	if err != nil {
		panic(err)
	}

	bufferPoolManager := buffer.NewBufferPoolManager(storage.MAX_BUFFER_POOL_SIZE, dm, lm)

	return &ContractedGraph{
		ContractedOutEdges: make([]datastructure.EdgeCH, 0),
		ContractedInEdges:  make([]datastructure.EdgeCH, 0),
		ContractedNodes:    make([]datastructure.CHNode, 0),
		Ready:              false,
		StreetDirection:    make(map[int][2]bool),
		TagStringIDMap:     util.NewIdMap(),
		bufferPoolManager:  bufferPoolManager,
		diskManager:        dm,
		logManager:         lm,
		CompressedBlock:    make(map[int]bool),
	}
}

func NewContractedGraphFromOtherGraph(otherGraph *ContractedGraph) *ContractedGraph {
	outEdges := otherGraph.GetOutEdges()
	qOutEdges := make([]datastructure.EdgeCH, len(outEdges))
	copy(qOutEdges, outEdges)

	inEdges := otherGraph.GetInEdges()
	qInEdges := make([]datastructure.EdgeCH, len(inEdges))
	copy(qInEdges, inEdges)

	graphNodes := otherGraph.GetNodes()
	qNodes := make([]datastructure.CHNode, len(graphNodes))
	copy(qNodes, graphNodes)

	nodeOutEdges := otherGraph.GetFirstOutEdges()
	qFirstOutEdges := make([][]int32, len(nodeOutEdges))
	copy(qFirstOutEdges, nodeOutEdges)

	for i := range nodeOutEdges {
		qFirstOutEdges[i] = make([]int32, len(nodeOutEdges[i]))
		copy(qFirstOutEdges[i], nodeOutEdges[i])
	}

	nodeInEdges := otherGraph.GetFirstInEdges()
	qFirstInEdges := make([][]int32, len(nodeInEdges))
	copy(qFirstInEdges, nodeInEdges)

	for i := range nodeInEdges {
		qFirstInEdges[i] = make([]int32, len(nodeInEdges[i]))
		copy(qFirstInEdges[i], nodeInEdges[i]) // Deep copy
	}

	edgeInfo := otherGraph.MapEdgeInfo
	nodeInfo := *otherGraph.NodeInfo

	return &ContractedGraph{
		ContractedOutEdges:     qOutEdges,
		ContractedInEdges:      qInEdges,
		ContractedNodes:        qNodes,
		ContractedFirstOutEdge: qFirstOutEdges,
		ContractedFirstInEdge:  qFirstInEdges,
		MapEdgeInfo:            edgeInfo,
		NodeInfo:               &nodeInfo,
	}
}

func (ch *ContractedGraph) InitCHGraph(processedNodes []datastructure.CHNode,
	processedEdges []datastructure.EdgeCH, streetDirections map[string][2]bool,
	tagStringIdMap util.IDMap, edgesExtraInfo []datastructure.EdgeExtraInfo,
	nodeInfo *datastructure.NodeInfo) {

	ch.TagStringIDMap = tagStringIdMap

	gLen := len(processedNodes)

	for streetName, direction := range streetDirections {
		ch.StreetDirection[ch.TagStringIDMap.GetID(streetName)] = direction
	}

	ch.ContractedNodes = make([]datastructure.CHNode, gLen)

	copy(ch.ContractedNodes, processedNodes)

	ch.Metadata.degrees = make([]int, gLen)
	ch.Metadata.InEdgeOrigCount = make([]int, gLen)
	ch.Metadata.OutEdgeOrigCount = make([]int, gLen)
	ch.Metadata.ShortcutsCount = 0

	outEdgeID := int32(0)
	inEdgeID := int32(0)
	ch.ContractedFirstOutEdge = make([][]int32, len(ch.ContractedNodes))
	ch.ContractedFirstInEdge = make([][]int32, len(ch.ContractedNodes))

	log.Printf("intializing original osm graph...")

	// init graph original
	for _, edge := range processedEdges {

		ch.ContractedFirstOutEdge[edge.FromNodeID] = append(ch.ContractedFirstOutEdge[edge.FromNodeID], int32(outEdgeID))
		ch.ContractedOutEdges = append(ch.ContractedOutEdges, datastructure.NewEdgeCHPlain(
			outEdgeID, edge.Weight, edge.Dist, edge.ToNodeID, edge.FromNodeID,
		))

		outEdgeID++
		ch.Metadata.OutEdgeOrigCount[edge.FromNodeID]++

		// in Edges
		ch.ContractedFirstInEdge[edge.ToNodeID] = append(ch.ContractedFirstInEdge[edge.ToNodeID], int32(inEdgeID))

		ch.ContractedInEdges = append(ch.ContractedInEdges, datastructure.NewEdgeCHPlain(
			inEdgeID, edge.Weight, edge.Dist, edge.FromNodeID, edge.ToNodeID,
		))

		inEdgeID++
		ch.Metadata.InEdgeOrigCount[edge.FromNodeID]++

		// tambah degree nodenya
		ch.Metadata.degrees[edge.FromNodeID]++
		ch.Metadata.degrees[edge.ToNodeID]++

	}

	ch.saveExtraInfo(edgesExtraInfo, nodeInfo)

	log.Printf("initializing osm graph done...")

	ch.Metadata.EdgeCount = len(ch.ContractedOutEdges)
	ch.Metadata.NodeCount = gLen
	ch.Metadata.MeanDegree = float64(len(ch.ContractedOutEdges) * 1.0 / gLen)

}

func (ch *ContractedGraph) saveExtraInfo(edgesExtraInfo []datastructure.EdgeExtraInfo, nodeInfo *datastructure.NodeInfo) {
	ch.MapEdgeInfo = datastructure.NewMapEdgeInfo()
	for i, info := range edgesExtraInfo {
		edge := ch.GetOutEdge(int32(i))
		from := edge.FromNodeID
		to := edge.ToNodeID

		if _, ok := ch.MapEdgeInfo[from][to]; ok {
			continue
		}

		// for outgoing edge
		if ch.MapEdgeInfo[from] == nil {
			ch.MapEdgeInfo[from] = make(map[int32]datastructure.EdgeExtraInfo)
		}
		ch.MapEdgeInfo[from][to] = info

		// for incoming edge. edgeInfo harus dari from->to aja
		if ch.MapEdgeInfo[to] == nil {
			ch.MapEdgeInfo[to] = make(map[int32]datastructure.EdgeExtraInfo)
		}
		ch.MapEdgeInfo[to][from] = info
	}
	ch.NodeInfo = nodeInfo
}

func (ch *ContractedGraph) Contraction() (err error) {
	st := time.Now()
	nq := NewMinHeap[int32]()

	ch.nextEdgeID = int32(len(ch.ContractedOutEdges))

	ch.UpdatePrioritiesOfRemainingNodes(nq) // bikin node ordering

	log.Printf("total nodes: %d", len(ch.ContractedNodes))
	log.Printf("total edges: %d", len(ch.ContractedOutEdges))

	level := 0
	contracted := make([]bool, ch.Metadata.NodeCount)
	orderNum := int32(0)

	var polledItem, smallestItem PriorityQueueNode[int32]
	for nq.Size() != 0 {
		smallestItem, err = nq.GetMin()
		if err != nil {
			err = server.WrapErrorf(err, server.ErrInternalServerError, "internal server error")
			return
		}

		polledItem, err = nq.ExtractMin()
		if err != nil {
			err = server.WrapErrorf(err, server.ErrInternalServerError, "internal server error")
			return
		}

		// lazy update
		priority := ch.calculatePriority(polledItem.Item, contracted)

		if nq.Size() > 0 && priority > smallestItem.Rank {
			// current node importantnya lebih tinggi dari next pq item
			nq.Insert(PriorityQueueNode[int32]{Item: polledItem.Item, Rank: priority})
			continue
		}

		ch.ContractedNodes[polledItem.Item].OrderPos = orderNum

		ch.contractNode(polledItem.Item, contracted[polledItem.Item], contracted)
		contracted[polledItem.Item] = true
		level++
		orderNum++

		if (orderNum+1)%10000 == 0 {
			log.Printf("contracting node: %d...", orderNum+1)
		}
	}
	log.Printf("\ntotal shortcuts: %d \n", ch.Metadata.ShortcutsCount)

	ch.Metadata = Metadata{}
	runtime.GC()
	runtime.GC()
	end := time.Since(st)
	log.Printf("time for preprocessing contraction hierarchies: %v minutes\n", end.Minutes())
	return
}

func (ch *ContractedGraph) contractNode(nodeID int32, isContracted bool, contracted []bool) {

	if isContracted {
		return
	}
	ch.contractNodeNow(nodeID, contracted)

}

func (ch *ContractedGraph) contractNodeNow(nodeID int32, contracted []bool) {
	degree, _, _, _ := ch.findAndHandleShortcuts(nodeID, ch.addOrUpdateShortcut, int(ch.Metadata.MeanDegree*float64(maxPollFactorContraction)),
		contracted)
	ch.Metadata.MeanDegree = (ch.Metadata.MeanDegree*2 + float64(degree)) / 3

}

/*
findAndHandleShortcuts , ketika mengontraksi node v, kita  harus cari shortest path dari node u ke w yang meng ignore node v, dimana u adalah node yang terhubung ke v dan edge (u,v) \in E, dan w adalah node yang terhubung dari v dan edge (v,w) \in E.
kalau cost dari shortest path u->w  <= c(u,v) + c(v,w) , tambahkan shortcut edge (u,w).
*/
func (ch *ContractedGraph) findAndHandleShortcuts(nodeID int32, shortcutHandler func(fromNodeID, toNodeID int32, nodeIdx int32, weight float64,
	removedEdgeOne, removedEdgeTwo *datastructure.EdgeCH,
	outOrigEdgeCount, inOrigEdgeCount int),
	maxVisitedNodes int, contracted []bool) (int, int, int, error) {
	degree := 0
	shortcutCount := 0      // jumlah shortcut yang ditambahkan
	originalEdgesCount := 0 // = InEdgeCount(v) + OutEdgeCount(v)  setiap kali shortcut ditambahkan
	pMax := 0.0             // maximum cost path dari node u ke w, dimana u adalah semua node yang terhubung ke v & (u,v) \in E dan w adalah semua node yang terhubung ke v & (v, w) \in E
	pInMax := 0.0
	pOutMax := 0.0

	for _, idx := range ch.ContractedFirstInEdge[nodeID] {
		inEdge := ch.ContractedInEdges[idx]
		toNID := inEdge.ToNodeID
		if contracted[toNID] {
			continue
		}
		if inEdge.Weight > pInMax {
			pInMax = inEdge.Weight
		}
	}
	for _, idx := range ch.ContractedFirstOutEdge[nodeID] {
		outEdge := ch.ContractedOutEdges[idx]
		toNID := outEdge.ToNodeID
		if contracted[toNID] {
			continue
		}
		if outEdge.Weight > pOutMax {
			pOutMax = outEdge.Weight
		}
	}
	pMax = pInMax + pOutMax

	for _, inIdx := range ch.ContractedFirstInEdge[nodeID] {

		inEdge := ch.ContractedInEdges[inIdx]
		fromNodeID := inEdge.ToNodeID
		if fromNodeID == int32(nodeID) {
			continue
		}
		if contracted[fromNodeID] {
			continue
		}

		degree++

		for _, outID := range ch.ContractedFirstOutEdge[nodeID] {
			outEdge := ch.ContractedOutEdges[outID]
			toNode := outEdge.ToNodeID
			if contracted[toNode] {
				continue
			}

			if toNode == fromNodeID {
				// loop
				continue
			}

			existingDirectWeight := inEdge.Weight + outEdge.Weight

			maxWeight := ch.dijkstraWitnessSearch(fromNodeID, toNode, nodeID, existingDirectWeight, maxVisitedNodes, pMax,
				contracted)

			if maxWeight <= existingDirectWeight {
				// FOUND witness shortest path, there is a path from fromNodeID to toNodeID that is shorter than existingDirectWeight
				continue
			}

			// kalo d(u,w) > Pw , tambah shortcut. d(u,w) = shortest path dari u ke w tanpa melewati v
			// Pw = existingDirectWeight = d(u,v) + d(v,w)
			// d(u,v) = shortest path dari u ke w tanpa melewati v. Atau path dari u ke w tanpa melewati v yang cost nya <= Pw.

			shortcutCount++
			shortcutHandler(fromNodeID, toNode, nodeID, existingDirectWeight, &inEdge, &outEdge,
				ch.Metadata.OutEdgeOrigCount[nodeID], ch.Metadata.InEdgeOrigCount[nodeID])

		}
	}

	originalEdgesCount = ch.Metadata.InEdgeOrigCount[nodeID] + ch.Metadata.OutEdgeOrigCount[nodeID]

	return degree, shortcutCount, originalEdgesCount, nil
}

func countShortcut(fromNodeID, toNodeID int32, nodeID int32, weight float64, removedEdgeOne, removedEdgeTwo *datastructure.EdgeCH,
	outOrigEdgeCount, inOrigEdgeCount int) {
}

/*
addOrUpdateShortcut, menambahkan shortcut (u,w) jika path dari u->w tanpa lewati v cost nya lebih kecil dari c(u,v) + c(v,w).
*/
func (ch *ContractedGraph) addOrUpdateShortcut(fromNodeID, toNodeID int32, nodeID int32, weight float64, removedEdgeOne, removedEdgeTwo *datastructure.EdgeCH,
	outOrigEdgeCount, inOrigEdgeCount int) {

	exists := false
	for _, outID := range ch.ContractedFirstOutEdge[fromNodeID] {
		edge := ch.ContractedOutEdges[outID]
		if edge.ToNodeID != toNodeID {
			continue
		}
		exists = true
		isShortcut := ch.IsShortcut(edge.FromNodeID, edge.ToNodeID, false)

		if isShortcut && weight < edge.Weight { // only update edge weight when the edge is a shortcut
			edge.Weight = weight
		}
	}

	for _, inID := range ch.ContractedFirstInEdge[toNodeID] {
		edge := ch.ContractedInEdges[inID]
		if edge.ToNodeID != fromNodeID {
			continue
		}
		exists = true

		// reverse flag must be false. because we already set (to, from) incoming edge info in previous code.
		isShortcut := ch.IsShortcut(edge.FromNodeID, edge.ToNodeID, false)

		if isShortcut && weight < edge.Weight {
			edge.Weight = weight
		}
	}

	if !exists {
		ch.addShortcut(fromNodeID, toNodeID, nodeID, weight, removedEdgeOne, removedEdgeTwo)
		ch.Metadata.ShortcutsCount++
	}
}

func (ch *ContractedGraph) addShortcut(fromNodeID, toNodeID, contractedNodeID int32, weight float64, removedEdgeOne, removedEdgeTwo *datastructure.EdgeCH,
) {

	fromN := ch.ContractedNodes[fromNodeID]
	toN := ch.ContractedNodes[toNodeID]

	dist := geo.CalculateHaversineDistance(fromN.Lat, fromN.Lon, toN.Lat, toN.Lon) * 1000
	// add shortcut outcoming edge

	dup := false
	for _, outID := range ch.ContractedFirstOutEdge[fromNodeID] {
		edge := ch.ContractedOutEdges[outID]
		isShortcut := ch.IsShortcut(fromNodeID, toNodeID, false)
		// mark it as duplicate if the edge with fromNodeID and toNodeID exits (either a shortcut or not)
		if isShortcut && edge.ToNodeID == toNodeID && weight < edge.Weight {
			ch.ContractedOutEdges[outID].Weight = weight
			ch.ContractedOutEdges[outID].Dist = dist
			ch.ContractedOutEdges[outID].ViaNodeID = contractedNodeID
			dup = true
			break
		} else if !isShortcut && edge.ToNodeID == toNodeID {
			// if the edge is not a shortcut, but the edge with fromNodeID and toNodeID exits, then mark its as duplicate and dont update the edge
			dup = true
			break
		}
	}

	if !dup {

		currEdgeID := int32(len(ch.ContractedOutEdges))

		ch.ContractedOutEdges = append(ch.ContractedOutEdges, datastructure.NewEdgeCH(
			currEdgeID, weight, dist, toNodeID, fromNodeID, contractedNodeID,
		))

		ch.ContractedFirstOutEdge[fromNodeID] = append(ch.ContractedFirstOutEdge[fromNodeID], currEdgeID)
		ch.Metadata.degrees[fromNodeID]++

		ch.MapEdgeInfo.AppendEdgeInfo(fromNodeID, toNodeID, true)
	}

	dup = false
	// add shortcut
	for _, inID := range ch.ContractedFirstInEdge[toNodeID] {
		edge := ch.ContractedInEdges[inID]
		isShortcut := ch.IsShortcut(toNodeID, fromNodeID, false)
		if isShortcut && edge.ToNodeID == fromNodeID && weight < edge.Weight {
			ch.ContractedInEdges[inID].Weight = weight
			ch.ContractedInEdges[inID].Dist = dist
			ch.ContractedInEdges[inID].ViaNodeID = contractedNodeID
			dup = true
			break
		} else if !isShortcut && edge.ToNodeID == fromNodeID {
			// if the edge is not a shortcut, but the edge with fromNodeID and toNodeID exits, then mark its as duplicate and dont update the edge
			dup = true
			break
		}
	}

	// incoming edge
	if !dup {
		currEdgeID := int32(len(ch.ContractedInEdges))

		ch.ContractedInEdges = append(ch.ContractedInEdges, datastructure.NewEdgeCH(
			currEdgeID, weight, dist, fromNodeID, toNodeID, contractedNodeID,
		))

		ch.ContractedFirstInEdge[toNodeID] = append(ch.ContractedFirstInEdge[toNodeID], currEdgeID)

		ch.Metadata.degrees[toNodeID]++

		ch.MapEdgeInfo.AppendEdgeInfo(toNodeID, fromNodeID, true)
	}

}

func (ch *ContractedGraph) calculatePriority(nodeID int32, contracted []bool) float64 {

	_, shortcutsCount, originalEdgesCount, _ := ch.findAndHandleShortcuts(nodeID, countShortcut, int(ch.Metadata.MeanDegree*float64(maxPollFactorHeuristic)),
		contracted)

	// |shortcuts(v)| − |{(u, v) | v uncontracted}| − |{(v, w) | v uncontracted}|
	// outDegree+inDegree
	edgeDifference := shortcutsCount - ch.Metadata.degrees[nodeID]

	return float64(10*edgeDifference + 1*originalEdgesCount)
}

func (ch *ContractedGraph) UpdatePrioritiesOfRemainingNodes(nq *MinHeap[int32]) {

	contracted := make([]bool, ch.Metadata.NodeCount)

	for nodeID, _ := range ch.ContractedNodes {

		priority := ch.calculatePriority(int32(nodeID), contracted)
		nq.Insert(PriorityQueueNode[int32]{Item: int32(nodeID), Rank: priority})

		if (nodeID+1)%10000 == 0 {
			log.Printf("update node priority : %d...", nodeID+1)
		}
	}
}

func (ch *ContractedGraph) AddEdge(newEdge datastructure.EdgeCH) {

	fromNodeID := newEdge.FromNodeID
	toNodeID := newEdge.ToNodeID

	currEdgeID := int32(len(ch.ContractedOutEdges)) // new edge ID

	// add outEdges && inEdges for fromNodeID
	if len(ch.ContractedFirstOutEdge) <= int(fromNodeID) {
		ch.ContractedFirstOutEdge = append(ch.ContractedFirstOutEdge, []int32{})
	}

	if len(ch.ContractedFirstInEdge) <= int(toNodeID) {
		ch.ContractedFirstInEdge = append(ch.ContractedFirstInEdge, []int32{})
	}

	newEdge.EdgeID = currEdgeID

	// check for duplicate edge
	for _, outID := range ch.ContractedFirstOutEdge[fromNodeID] {
		edge := ch.ContractedOutEdges[outID]
		if edge.ToNodeID == toNodeID {
			return
		}
	}

	// add outEdge
	ch.ContractedOutEdges = append(ch.ContractedOutEdges, newEdge)

	ch.ContractedFirstOutEdge[fromNodeID] = append(ch.ContractedFirstOutEdge[fromNodeID], currEdgeID)

	// add inEdge
	currEdgeID = int32(len(ch.ContractedInEdges))

	temp := newEdge.FromNodeID
	newEdge.FromNodeID = newEdge.ToNodeID
	newEdge.ToNodeID = temp

	newEdge.EdgeID = currEdgeID

	ch.ContractedInEdges = append(ch.ContractedInEdges, newEdge)

	ch.ContractedFirstInEdge[toNodeID] = append(ch.ContractedFirstInEdge[toNodeID], currEdgeID)
}

func (ch *ContractedGraph) AddNode(node datastructure.CHNode) {
	ch.ContractedNodes = append(ch.ContractedNodes, node)
}

func (ch *ContractedGraph) RemoveEdge(edgeID int32, fromNodeID int32, toNodeID int32) {

	// remove outgoing edge
	for i, idx := range ch.ContractedFirstOutEdge[fromNodeID] {
		if idx == edgeID {
			ch.ContractedFirstOutEdge[fromNodeID] = append(ch.ContractedFirstOutEdge[fromNodeID][:i], ch.ContractedFirstOutEdge[fromNodeID][i+1:]...)
			break
		}
	}

	// remove incoming edge
	for i, idx := range ch.ContractedFirstInEdge[toNodeID] {
		if idx == edgeID {
			ch.ContractedFirstInEdge[toNodeID] = append(ch.ContractedFirstInEdge[toNodeID][:i], ch.ContractedFirstInEdge[toNodeID][i+1:]...)
			break
		}
	}
}

func (ch *ContractedGraph) IsChReady() bool {
	return ch.Ready
}

func (ch *ContractedGraph) GetNodeFirstOutEdges(nodeID int32) []int32 {
	return ch.ContractedFirstOutEdge[nodeID]
}

func (ch *ContractedGraph) GetNodeFirstInEdges(nodeID int32) []int32 {
	return ch.ContractedFirstInEdge[nodeID]
}

func (ch *ContractedGraph) GetOutEdge(edgeID int32) datastructure.EdgeCH {
	return ch.ContractedOutEdges[edgeID]
}

func (ch *ContractedGraph) GetInEdge(edgeID int32) datastructure.EdgeCH {
	return ch.ContractedInEdges[edgeID]
}

func (ch *ContractedGraph) GetOutEdges() []datastructure.EdgeCH {
	return ch.ContractedOutEdges
}

func (ch *ContractedGraph) GetInEdges() []datastructure.EdgeCH {
	return ch.ContractedInEdges
}

func (ch *ContractedGraph) GetNodes() []datastructure.CHNode {
	return ch.ContractedNodes
}

func (ch *ContractedGraph) GetFirstInEdges() [][]int32 {
	return ch.ContractedFirstInEdge
}

func (ch *ContractedGraph) GetFirstOutEdges() [][]int32 {
	return ch.ContractedFirstOutEdge
}

func (ch *ContractedGraph) GetNode(nodeID int32) datastructure.CHNode {
	return ch.ContractedNodes[nodeID]
}

func (ch *ContractedGraph) GetNumNodes() int {
	return len(ch.ContractedNodes)
}
func (ch *ContractedGraph) SetCHReady() {
	ch.Ready = true
}

func (ch *ContractedGraph) GetStreetDirection(streetName int) [2]bool {
	return ch.StreetDirection[streetName]
}

func (ch *ContractedGraph) GetStreetNameFromID(streetName int) string {
	return ch.TagStringIDMap.GetStr(streetName)
}

func (ch *ContractedGraph) GetRoadClassFromID(roadClass int) string {
	return ch.TagStringIDMap.GetStr(roadClass)
}

func (ch *ContractedGraph) GetRoadClassLinkFromID(roadClassLink int) string {
	return ch.TagStringIDMap.GetStr(roadClassLink)
}

func (ch *ContractedGraph) IsShortcut(fromNodeID, toNodeID int32, reverse bool) bool {

	return ch.MapEdgeInfo.GetEdgeInfo(fromNodeID, toNodeID, reverse).IsShortcut
}

func (ch *ContractedGraph) IsRoundabout(fromNodeID, toNodeID int32, reverse bool) bool {
	return ch.MapEdgeInfo.GetEdgeInfo(fromNodeID, toNodeID, reverse).Roundabout
}

func (ch *ContractedGraph) GetEdgePointsInBetween(fromNodeID, toNodeID int32, reverse bool) []datastructure.Coordinate {
	return ch.MapEdgeInfo.GetEdgeInfo(fromNodeID, toNodeID, reverse).PointsInBetween
}

func (ch *ContractedGraph) GetStreetName(fromNodeID, toNodeID int32, reverse bool) int {
	return ch.MapEdgeInfo.GetEdgeInfo(fromNodeID, toNodeID, reverse).StreetName
}

func (ch *ContractedGraph) GetRoadClass(fromNodeID, toNodeID int32, reverse bool) int {
	return ch.MapEdgeInfo.GetEdgeInfo(fromNodeID, toNodeID, reverse).RoadClass
}

func (ch *ContractedGraph) GetRoadClassLink(fromNodeID, toNodeID int32, reverse bool) int {
	return ch.MapEdgeInfo.GetEdgeInfo(fromNodeID, toNodeID, reverse).RoadClassLink
}

func (ch *ContractedGraph) GetLanes(fromNodeID, toNodeID int32, reverse bool) int {
	return ch.MapEdgeInfo.GetEdgeInfo(fromNodeID, toNodeID, reverse).Lanes
}

func (ch *ContractedGraph) SetPointsInBetween(fromNodeID, toNodeID int32, reverse bool, points []datastructure.Coordinate) {
	if !reverse {
		edgeInfo := ch.MapEdgeInfo[fromNodeID][toNodeID]
		edgeInfo.PointsInBetween = points
		ch.MapEdgeInfo[fromNodeID][toNodeID] = edgeInfo
	} else {
		edgeInfo := ch.MapEdgeInfo[toNodeID][fromNodeID]
		edgeInfo.PointsInBetween = points
		ch.MapEdgeInfo[toNodeID][fromNodeID] = edgeInfo
	}
}

func (ch *ContractedGraph) SetEdgeInfo(fromNodeID, toNodeID int32, reverse bool, edgeInfo datastructure.EdgeExtraInfo) {
	if !reverse {
		ch.MapEdgeInfo[fromNodeID][toNodeID] = edgeInfo
	} else {
		ch.MapEdgeInfo[toNodeID][fromNodeID] = edgeInfo
	}
}

func (ch *ContractedGraph) IsTrafficLight(nodeID int32) bool {
	trafficLight, ok := ch.NodeInfo.TrafficLight[nodeID]
	if !ok {
		return false
	}
	return trafficLight
}

func (ch *ContractedGraph) SaveToFile() error {
	buf := new(bytes.Buffer)
	enc := gob.NewEncoder(buf)
	err := enc.Encode(ch)

	if err != nil {
		return err
	}

	f, err := os.Create("./ch_graph.graph")

	if err != nil {
		return err
	}
	defer f.Close()
	_, err = f.Write(buf.Bytes())
	return err
}

func (ch *ContractedGraph) LoadGraph() error {
	f, err := os.Open("./ch_graph.graph")
	if err != nil {
		return err
	}
	defer f.Close()

	fileInfo, err := f.Stat()
	if err != nil {
		log.Println("Error getting file info:", err)
		return err
	}
	fileSize := fileInfo.Size()

	data := make([]byte, fileSize)
	_, err = io.ReadFull(f, data)
	if err != nil {
		log.Println("Error reading file:", err)
		return err
	}
	dec := gob.NewDecoder(bytes.NewReader(data))
	err = dec.Decode(&ch)
	return err
}

func (ch *ContractedGraph) removeOldGraph() {
	// empty old contracted graph

	ch.ContractedOutEdges = make([]datastructure.EdgeCH, 0)
	ch.ContractedInEdges = make([]datastructure.EdgeCH, 0)
	ch.ContractedFirstOutEdge = make([][]int32, 0)
	ch.ContractedFirstInEdge = make([][]int32, 0)
	ch.NodeInfo = nil
	ch.MapEdgeInfo = nil
}

/*
the idea is similar to the idea explained in: https://turing.iem.thm.de/routeplanning/hwy/mobileSubmit.pdf .
we group graph nodes based on proximity into 1 block. Insert adjacency array/compressed sparse row and edge info owned by nodes in one block into page (https://15445.courses.cs.cmu.edu/spring2023/slides/03-storage1.pdf).
one block/page is 16kb in size. insert each block/page insert into file. Create buffer pool manager/cache with replacer algorithm using lru to cache hundreds/thousands of blocks into memory.
*/
func (ch *ContractedGraph) GroupNodeByProximityAndSaveToDisk() error {

	// group node by its geohash
	geohashNodeIDMap := make(map[string][]int32)
	for _, node := range ch.ContractedNodes {
		geohash := geohash.EncodeWithPrecision(node.Lat, node.Lon, 5)

		geohashNodeIDMap[geohash] = append(geohashNodeIDMap[geohash], node.ID)
	}

	ch.NodeIDBlockIDMap = make(map[int32]int, len(ch.ContractedNodes))
	// flatten the nodeIDs
	nodesByProximity := make([]int32, 0)

	geohashKeys := make([]string, 0)
	for k := range geohashNodeIDMap {
		geohashKeys = append(geohashKeys, k)
	}
	sort.Strings(geohashKeys) // sort by geohash keys, so that every adjacent node in nodesByProximity is close to each other

	for _, key := range geohashKeys {
		nodesByProximity = append(nodesByProximity, geohashNodeIDMap[key]...)
	}

	blockID := 0
	nodeBlocks := make(map[int][]int32)

	initialHeaderSize := 8

	blockSizeMap := make(map[int]int)

	for i, nodeID := range nodesByProximity {
		if (i+1)%10000 == 0 {
			log.Printf("grouping node by proximity: %d...", i+1)
		}

		copyNodeBlocks := make([]int32, len(nodeBlocks[blockID]))
		copy(copyNodeBlocks, nodeBlocks[blockID])
		copyNodeBlocks = append(copyNodeBlocks, nodeID)

		currBlockSize, _, _, edgeInfoSize := ch.GetBlockSize(copyNodeBlocks)

		currBlockSize += initialHeaderSize

		if currBlockSize <= storage.MAX_PAGE_SIZE && edgeInfoSize <= storage.MAX_PAGE_SIZE {
			nodeBlocks[blockID] = append(nodeBlocks[blockID], nodeID)
		} else {

			nodeBlockSize, _, _, nodeEdgeInfoSize := ch.GetNodeBufferSize(nodeID)
			blockSizeMap[blockID] = currBlockSize - nodeBlockSize

			// add nodeID to new block
			blockID += 2 // block+1 for node edges extra info

			nodeBlocks[blockID] = append(nodeBlocks[blockID], nodeID)

			blockSizeMap[blockID] = nodeBlockSize
			if nodeBlockSize > storage.MAX_PAGE_SIZE || nodeEdgeInfoSize > storage.MAX_PAGE_SIZE {
				// handle when a block with only one node have size > 16kb
				ch.CompressedBlock[blockID] = true
			}
		}

		ch.NodeIDBlockIDMap[nodeID] = blockID
	}

	blockIDs := make([]int, 0, len(nodeBlocks))
	for k := range nodeBlocks {
		blockIDs = append(blockIDs, k)
	}
	sort.Ints(blockIDs)

	// build compressed sparse row representation for each group/block and write it to disk
	for i, blockID := range blockIDs {
		nodeIDs := nodeBlocks[blockID]
		if (i+1)%int(0.05*float64(len(blockIDs))) == 0 {
			log.Printf("building csr for block: %d...", i+1)
		}
		csrN, csrF, csrExtraInfo, isTrafficLight := ch.BuildCompressedSparseRowFromNodes(nodeIDs, false)
		csrNRev, csrFRev, csrRevExtraInfo, _ := ch.BuildCompressedSparseRowFromNodes(nodeIDs, true)

		for _, edgeinfo := range csrExtraInfo {
			if edgeinfo.IsShortcut && len(edgeinfo.PointsInBetween) > 0 {
				panic(errors.New("shortcut edge should not have points in between"))
			}
		}

		for _, edgeinfo := range csrRevExtraInfo {
			if edgeinfo.IsShortcut && len(edgeinfo.PointsInBetween) > 0 {
				panic(errors.New("shortcut edge should not have points in between"))
			}
		}

		blockPage := disk.NewPage(storage.MAX_PAGE_SIZE)
		newBlockID := disk.NewBlockID(storage.GRAPH_FILE_NAME, blockID)

		_, err := blockPage.WriteBlock(csrN, csrNRev, csrF, csrFRev,
			nodeIDs, isTrafficLight, ch.CompressedBlock[blockID])
		if err != nil {
			return err
		}

		nodeEdgesblockPage := disk.NewPage(storage.MAX_PAGE_SIZE)

		nodeEdgesBlockID := disk.NewBlockID(storage.GRAPH_FILE_NAME, blockID+1)

		_, err = nodeEdgesblockPage.WriteEdgeInfoBlock(csrExtraInfo, csrRevExtraInfo,
			csrF, csrFRev, ch.CompressedBlock[blockID])
		if err != nil {
			return err
		}

		err = ch.diskManager.Write(newBlockID, blockPage)

		if err != nil {
			return err
		}

		err = ch.diskManager.Write(nodeEdgesBlockID, nodeEdgesblockPage)
		if err != nil {
			return err
		}
	}

	ch.removeOldGraph()
	return nil
}

func (ch *ContractedGraph) GetBlockSize(nodeIDs []int32) (int, int, int, int) {
	size := 0
	sort.Slice(nodeIDs, func(i, j int) bool {
		return nodeIDs[i] < nodeIDs[j]
	})

	outEdgesLen := 0
	inEdgesLen := 0

	edgeInfoSize := 4 // edgesCount header

	for _, nodeID := range nodeIDs {
		currSize, nodeOutEdges, nodeInEdges, currEdgeSize := ch.GetNodeBufferSize(nodeID)

		size += currSize
		outEdgesLen += nodeOutEdges
		inEdgesLen += nodeInEdges
		edgeInfoSize += currEdgeSize
	}
	size += 8

	return size, outEdgesLen, inEdgesLen, edgeInfoSize
}

func (ch *ContractedGraph) GetNodeBufferSize(nodeID int32) (int, int, int, int) {
	bufSize := 4 * 2 // nodeIDs , (csrN,CsrNRev)

	bufSize += 40 * len(ch.GetNodeFirstOutEdges(nodeID))
	bufSize += 40 * len(ch.GetNodeFirstInEdges(nodeID))

	edgeInfoSize := 0

	type edgeWithDirection struct {
		edge    datastructure.EdgeCH
		reverse bool
	}

	uniqueEdge := make(map[int32]map[int32]edgeWithDirection)

	for _, edgeID := range ch.GetNodeFirstOutEdges(nodeID) {
		fromNodeID, toNodeID := ch.GetOutEdge(edgeID).FromNodeID, ch.GetOutEdge(edgeID).ToNodeID
		if _, ok := uniqueEdge[fromNodeID]; !ok {
			uniqueEdge[fromNodeID] = make(map[int32]edgeWithDirection)
		}
		uniqueEdge[fromNodeID][toNodeID] = edgeWithDirection{ch.GetOutEdge(edgeID), false}
	}

	for _, edgeID := range ch.GetNodeFirstInEdges(nodeID) {
		fromNodeID, toNodeID := ch.GetInEdge(edgeID).FromNodeID, ch.GetInEdge(edgeID).ToNodeID
		if _, ok := uniqueEdge[fromNodeID]; !ok {
			uniqueEdge[fromNodeID] = make(map[int32]edgeWithDirection)
		}
		uniqueEdge[fromNodeID][toNodeID] = edgeWithDirection{ch.GetInEdge(edgeID), true}
	}

	for _, toMap := range uniqueEdge {
		for _, edgeW := range toMap {
			edgeInfo := ch.MapEdgeInfo.GetEdgeInfo(edgeW.edge.FromNodeID, edgeW.edge.ToNodeID, edgeW.reverse)
			currSize := edgeInfo.GetEdgeInfoSize()

			edgeInfoSize += currSize
			edgeInfoSize += 16 // edgeID, fromNodeID, prefixSumSize, edgeSize
		}
	}

	return bufSize, len(ch.GetNodeFirstOutEdges(nodeID)), len(ch.GetNodeFirstInEdges(nodeID)), edgeInfoSize
}

func (ch *ContractedGraph) BuildCompressedSparseRowFromNodes(nodes []int32, reverse bool) ([]int,
	[]datastructure.EdgeCH, []datastructure.EdgeExtraInfo, []bool) {

	isTrafficLight := make([]bool, len(nodes))

	sort.Slice(nodes, func(i, j int) bool {
		return nodes[i] < nodes[j]
	})

	csrN := make([]int, len(nodes)+1)
	csrF := make([]datastructure.EdgeCH, 0)
	edgesExtraInfo := make([]datastructure.EdgeExtraInfo, 0)

	if !reverse {
		for i, node := range nodes {
			edgeIDs := ch.GetNodeFirstOutEdges(node)

			outEdges := make([]datastructure.EdgeCH, len(edgeIDs))
			for j, edgeID := range edgeIDs {
				outEdges[j] = ch.GetOutEdge(edgeID)
			}

			sort.Slice(outEdges, func(i, j int) bool {
				return outEdges[i].ToNodeID < outEdges[j].ToNodeID
			})

			for _, edge := range outEdges {
				edgeInfo := ch.MapEdgeInfo.GetEdgeInfo(edge.FromNodeID, edge.ToNodeID, false)

				edgesExtraInfo = append(edgesExtraInfo, edgeInfo)
			}

			csrF = append(csrF, outEdges...)
			csrN[i+1] = len(csrF)

			isTrafficLight[i] = ch.IsTrafficLight(node)
		}

	} else {

		for i, node := range nodes {
			edgeIDs := ch.GetNodeFirstInEdges(node)

			inEdges := make([]datastructure.EdgeCH, len(edgeIDs))
			for j, edgeID := range edgeIDs {
				inEdges[j] = ch.GetInEdge(edgeID)
			}

			sort.Slice(inEdges, func(i, j int) bool {
				return inEdges[i].ToNodeID < inEdges[j].ToNodeID
			})

			for _, edge := range inEdges {
				// reverse flag must be false, because there may be cases where (from->to) is not a shortcut, but (to->from) is a shortcut
				edgeInfo := ch.MapEdgeInfo.GetEdgeInfo(edge.FromNodeID, edge.ToNodeID, false)

				edgesExtraInfo = append(edgesExtraInfo, edgeInfo)
			}

			csrF = append(csrF, inEdges...)
			csrN[i+1] = len(csrF)
		}
	}

	return csrN, csrF, edgesExtraInfo, isTrafficLight
}
func (ch *ContractedGraph) AccessPageNodeEdge(nodeID int32) (*disk.Page, error) {
	blockID := disk.NewBlockID(storage.GRAPH_FILE_NAME, ch.NodeIDBlockIDMap[nodeID])
	page, err := ch.bufferPoolManager.FetchPage(blockID)
	if err != nil {
		return nil, err
	}
	return page, nil
}

func (ch *ContractedGraph) AccessPageNodeEdgeInfo(nodeID int32) (*disk.Page, error) {
	blockID := disk.NewBlockID(storage.GRAPH_FILE_NAME, ch.NodeIDBlockIDMap[nodeID]+1)
	page, err := ch.bufferPoolManager.FetchPage(blockID)

	if err != nil {
		return nil, err
	}

	return page, nil
}

func (ch *ContractedGraph) UnpinPage(nodeID int32) {
	blockID := disk.NewBlockID(storage.GRAPH_FILE_NAME, ch.NodeIDBlockIDMap[nodeID])
	ch.bufferPoolManager.UnpinPage(blockID, false)
}

func (ch *ContractedGraph) GetNodeOutEdgesCsr2(nodeID int32) ([]datastructure.EdgeCH, error) {
	page, err := ch.AccessPageNodeEdge(nodeID)
	if err != nil {
		return []datastructure.EdgeCH{}, err
	}
	defer ch.UnpinPage(nodeID)

	blockID := ch.NodeIDBlockIDMap[nodeID]
	return page.GetNodeOutEdgesCsr(nodeID, ch.CompressedBlock[blockID])
}

func (ch *ContractedGraph) GetNodeInEdgesCsr2(nodeID int32) ([]datastructure.EdgeCH, error) {
	page, err := ch.AccessPageNodeEdge(nodeID)
	if err != nil {
		return []datastructure.EdgeCH{}, err
	}
	defer ch.UnpinPage(nodeID)

	blockID := ch.NodeIDBlockIDMap[nodeID]
	return page.GetNodeInEdgesCsr(nodeID, ch.CompressedBlock[blockID])
}

func (ch *ContractedGraph) IsShortcutCsr(fromNodeID, toNodeID int32, reverse bool) (bool, error) {

	page, err := ch.AccessPageNodeEdgeInfo(fromNodeID)
	if err != nil {
		return false, err
	}
	defer ch.UnpinPage(fromNodeID)
	blockID := ch.NodeIDBlockIDMap[fromNodeID]
	edgeInfo, err := page.GetEdgeInfo(fromNodeID, toNodeID, reverse, ch.CompressedBlock[blockID])

	return edgeInfo.IsShortcut, err
}

func (ch *ContractedGraph) IsRoundaboutCsr(fromNodeID, toNodeID int32) (bool, error) {
	page, err := ch.AccessPageNodeEdgeInfo(fromNodeID)
	if err != nil {
		return false, err
	}
	defer ch.UnpinPage(fromNodeID)
	blockID := ch.NodeIDBlockIDMap[fromNodeID]
	edgeInfo, err := page.GetEdgeInfo(fromNodeID, toNodeID, false, ch.CompressedBlock[blockID])

	return edgeInfo.Roundabout, err
}

func (ch *ContractedGraph) GetEdgePointsInBetweenCsr(fromNodeID, toNodeID int32, reverse bool) ([]datastructure.Coordinate, error) {
	page, err := ch.AccessPageNodeEdgeInfo(fromNodeID)
	if err != nil {
		return []datastructure.Coordinate{}, err
	}
	defer ch.UnpinPage(fromNodeID)
	blockID := ch.NodeIDBlockIDMap[fromNodeID]
	edgeInfo, err := page.GetEdgeInfo(fromNodeID, toNodeID, reverse, ch.CompressedBlock[blockID])

	return edgeInfo.PointsInBetween, err
}

func (ch *ContractedGraph) GetStreetNameCsr(fromNodeID, toNodeID int32) (int, error) {
	page, err := ch.AccessPageNodeEdgeInfo(fromNodeID)
	if err != nil {
		return 0, err
	}
	defer ch.UnpinPage(fromNodeID)
	blockID := ch.NodeIDBlockIDMap[fromNodeID]
	edgeInfo, err := page.GetEdgeInfo(fromNodeID, toNodeID, false, ch.CompressedBlock[blockID])

	return edgeInfo.StreetName, err
}

func (ch *ContractedGraph) GetEdgeInfo(fromNodeID, toNodeID int32) (datastructure.EdgeExtraInfo, error) {
	page, err := ch.AccessPageNodeEdgeInfo(fromNodeID)
	if err != nil {
		return datastructure.EdgeExtraInfo{}, err
	}
	defer ch.UnpinPage(fromNodeID)
	blockID := ch.NodeIDBlockIDMap[fromNodeID]
	edgeInfo, err := page.GetEdgeInfo(fromNodeID, toNodeID, false, ch.CompressedBlock[blockID])

	return edgeInfo, err
}

func (ch *ContractedGraph) GetRoadClassCsr(fromNodeID, toNodeID int32) (int, error) {
	page, err := ch.AccessPageNodeEdgeInfo(fromNodeID)
	if err != nil {
		return 0, err
	}
	defer ch.UnpinPage(fromNodeID)
	blockID := ch.NodeIDBlockIDMap[fromNodeID]
	edgeInfo, err := page.GetEdgeInfo(fromNodeID, toNodeID, false, ch.CompressedBlock[blockID])

	return edgeInfo.RoadClass, err
}

func (ch *ContractedGraph) GetRoadClassLinkCsr(fromNodeID, toNodeID int32) (int, error) {
	page, err := ch.AccessPageNodeEdgeInfo(fromNodeID)
	if err != nil {
		return 0, err
	}
	defer ch.UnpinPage(fromNodeID)
	blockID := ch.NodeIDBlockIDMap[fromNodeID]
	edgeInfo, err := page.GetEdgeInfo(fromNodeID, toNodeID, false, ch.CompressedBlock[blockID])

	return edgeInfo.RoadClassLink, err
}

func (ch *ContractedGraph) GetLanesCsr(fromNodeID, toNodeID int32) (int, error) {
	page, err := ch.AccessPageNodeEdgeInfo(fromNodeID)
	if err != nil {
		return 0, err
	}
	defer ch.UnpinPage(fromNodeID)
	blockID := ch.NodeIDBlockIDMap[fromNodeID]
	edgeInfo, err := page.GetEdgeInfo(fromNodeID, toNodeID, false, ch.CompressedBlock[blockID])

	return edgeInfo.RoadClassLink, err
}

func (ch *ContractedGraph) IsTrafficLightCsr(nodeID int32) (bool, error) {
	page, err := ch.AccessPageNodeEdge(nodeID)
	if err != nil {
		return false, err
	}
	defer ch.UnpinPage(nodeID)
	return page.IsNodeTrafficLight(nodeID), err
}
