package contractor

import (
	"bytes"
	"encoding/gob"
	"io"
	"log"
	"os"
	"runtime"
	"time"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/geo"
	"github.com/lintang-b-s/navigatorx/pkg/server"
	"github.com/lintang-b-s/navigatorx/pkg/util"
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
	GraphStorage           *datastructure.GraphStorage
	ContractedNodes        []datastructure.CHNode
	Metadata               Metadata
	ContractedFirstOutEdge [][]int32
	ContractedFirstInEdge  [][]int32

	nextEdgeID int32

	StreetDirection map[int][2]bool // 0 = forward, 1 = backward
	TagStringIDMap  util.IDMap
}

var maxPollFactorHeuristic = 5
var maxPollFactorContraction = 200

func NewContractedGraph() *ContractedGraph {

	return &ContractedGraph{
		ContractedNodes: make([]datastructure.CHNode, 0),

		StreetDirection: make(map[int][2]bool),
		TagStringIDMap:  util.NewIdMap(),
	}

}

func NewContractedGraphFromOtherGraph(otherGraph *ContractedGraph) *ContractedGraph {

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

	graphStorage := *otherGraph.GraphStorage
	return &ContractedGraph{

		ContractedNodes:        qNodes,
		ContractedFirstOutEdge: qFirstOutEdges,
		ContractedFirstInEdge:  qFirstInEdges,
		GraphStorage:           &graphStorage,
	}
}

func (ch *ContractedGraph) InitCHGraph(processedNodes []datastructure.CHNode,
	graphStorage *datastructure.GraphStorage, streetDirections map[string][2]bool,
	tagStringIdMap util.IDMap) {

	ch.TagStringIDMap = tagStringIdMap

	gLen := len(processedNodes)

	for streetName, direction := range streetDirections {
		ch.StreetDirection[ch.TagStringIDMap.GetID(streetName)] = direction
	}
	ch.GraphStorage = graphStorage

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
	for _, edge := range ch.GraphStorage.EdgeStorage {

		ch.ContractedFirstOutEdge[edge.FromNodeID] = append(ch.ContractedFirstOutEdge[edge.FromNodeID], int32(outEdgeID))

		outEdgeID++
		ch.Metadata.OutEdgeOrigCount[edge.FromNodeID]++

		// in Edges
		ch.ContractedFirstInEdge[edge.ToNodeID] = append(ch.ContractedFirstInEdge[edge.ToNodeID], int32(inEdgeID))

		inEdgeID++
		ch.Metadata.InEdgeOrigCount[edge.FromNodeID]++

		// tambah degree nodenya
		ch.Metadata.degrees[edge.FromNodeID]++
		ch.Metadata.degrees[edge.ToNodeID]++

	}

	log.Printf("initializing osm graph done...")

	ch.Metadata.EdgeCount = len(ch.GraphStorage.EdgeStorage)
	ch.Metadata.NodeCount = gLen
	ch.Metadata.MeanDegree = float64(len(ch.GraphStorage.EdgeStorage) * 1.0 / gLen)

}

func (ch *ContractedGraph) Contraction() (err error) {
	st := time.Now()
	nq := NewMinHeap[int32]()

	ch.nextEdgeID = int32(len(ch.GraphStorage.EdgeStorage))

	ch.UpdatePrioritiesOfRemainingNodes(nq) // bikin node ordering

	log.Printf("total nodes: %d", len(ch.ContractedNodes))
	log.Printf("total edges: %d", len(ch.GraphStorage.EdgeStorage))
	ch.GraphStorage.SetStartShortcutID(ch.nextEdgeID)

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
	outOrigEdgeCount, inOrigEdgeCount int),
	maxVisitedNodes int, contracted []bool) (int, int, int, error) {
	degree := 0
	shortcutCount := 0      // jumlah shortcut yang ditambahkan
	originalEdgesCount := 0 // = InEdgeCount(v) + OutEdgeCount(v)  setiap kali shortcut ditambahkan
	pMax := 0.0             // maximum cost path dari node u ke w, dimana u adalah semua node yang terhubung ke v & (u,v) \in E dan w adalah semua node yang terhubung ke v & (v, w) \in E
	pInMax := 0.0
	pOutMax := 0.0

	for _, idx := range ch.ContractedFirstInEdge[nodeID] {
		inEdge := ch.GraphStorage.GetInEdge(idx)
		toNID := inEdge.ToNodeID
		if contracted[toNID] {
			continue
		}
		if inEdge.Weight > pInMax {
			pInMax = inEdge.Weight
		}
	}
	for _, idx := range ch.ContractedFirstOutEdge[nodeID] {
		outEdge := ch.GraphStorage.GetOutEdge(idx)
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

		inEdge := ch.GraphStorage.GetInEdge(inIdx)
		fromNodeID := inEdge.ToNodeID
		if fromNodeID == int32(nodeID) {
			continue
		}
		if contracted[fromNodeID] {
			continue
		}

		degree++

		for _, outID := range ch.ContractedFirstOutEdge[nodeID] {
			outEdge := ch.GraphStorage.GetOutEdge(outID)
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
			shortcutHandler(fromNodeID, toNode, nodeID, existingDirectWeight,
				ch.Metadata.OutEdgeOrigCount[nodeID], ch.Metadata.InEdgeOrigCount[nodeID])

		}
	}

	originalEdgesCount = ch.Metadata.InEdgeOrigCount[nodeID] + ch.Metadata.OutEdgeOrigCount[nodeID]

	return degree, shortcutCount, originalEdgesCount, nil
}

func countShortcut(fromNodeID, toNodeID int32, nodeID int32, weight float64,
	outOrigEdgeCount, inOrigEdgeCount int) {
}

/*
addOrUpdateShortcut, menambahkan shortcut (u,w) jika path dari u->w tanpa lewati v cost nya lebih kecil dari c(u,v) + c(v,w).
*/
func (ch *ContractedGraph) addOrUpdateShortcut(fromNodeID, toNodeID int32, nodeID int32, weight float64,
	outOrigEdgeCount, inOrigEdgeCount int) {

	exists := false
	for _, outID := range ch.ContractedFirstOutEdge[fromNodeID] {
		edge := ch.GraphStorage.GetOutEdge(outID)
		if edge.ToNodeID != toNodeID {
			continue
		}
		exists = true
		isShortcut := ch.IsShortcut(edge.EdgeID)

		if isShortcut && weight < edge.Weight { // only update edge weight when the edge is a shortcut
			edge.Weight = weight
		}
	}

	for _, inID := range ch.ContractedFirstInEdge[toNodeID] {
		edge := ch.GraphStorage.GetInEdge(inID)
		if edge.ToNodeID != fromNodeID {
			continue
		}
		exists = true

		isShortcut := ch.IsShortcut(edge.EdgeID)

		if isShortcut && weight < edge.Weight {
			edge.Weight = weight
		}
	}

	if !exists {
		ch.addShortcut(fromNodeID, toNodeID, nodeID, weight)
		ch.Metadata.ShortcutsCount++
	}
}

func (ch *ContractedGraph) addShortcut(fromNodeID, toNodeID, contractedNodeID int32, weight float64,
) {

	fromN := ch.ContractedNodes[fromNodeID]
	toN := ch.ContractedNodes[toNodeID]

	dist := geo.CalculateHaversineDistance(fromN.Lat, fromN.Lon, toN.Lat, toN.Lon) * 1000
	// add shortcut outcoming edge

	dupf := false
	for _, outID := range ch.ContractedFirstOutEdge[fromNodeID] {
		edge := ch.GraphStorage.GetOutEdge(outID)
		isShortcut := ch.IsShortcut(outID)
		// mark it as duplicate if the edge with fromNodeID and toNodeID exits (either a shortcut or not)
		if isShortcut && edge.ToNodeID == toNodeID && weight < edge.Weight {

			ch.GraphStorage.UpdateEdge(outID, weight, dist, contractedNodeID)
			dupf = true
			break
		} else if !isShortcut && edge.ToNodeID == toNodeID {
			// if the edge is not a shortcut, but the edge with fromNodeID and toNodeID exits, then mark its as duplicate and dont update the edge
			dupf = true
			break
		}
	}

	currEdgeID := int32(len(ch.GraphStorage.EdgeStorage))
	if !dupf {

		ch.ContractedFirstOutEdge[fromNodeID] = append(ch.ContractedFirstOutEdge[fromNodeID], currEdgeID)
		ch.Metadata.degrees[fromNodeID]++
	}

	dupb := false
	// add shortcut
	for _, inID := range ch.ContractedFirstInEdge[toNodeID] {
		edge := ch.GraphStorage.GetInEdge(inID)
		isShortcut := ch.IsShortcut(inID)
		if isShortcut && edge.ToNodeID == fromNodeID && weight < edge.Weight {
			ch.GraphStorage.UpdateEdge(inID, weight, dist, contractedNodeID)
			dupb = true
			break
		} else if !isShortcut && edge.ToNodeID == fromNodeID {
			// if the edge is not a shortcut, but the edge with fromNodeID and toNodeID exits, then mark its as duplicate and dont update the edge
			dupb = true
			break
		}
	}

	// incoming edge
	if !dupb {

		ch.ContractedFirstInEdge[toNodeID] = append(ch.ContractedFirstInEdge[toNodeID], currEdgeID)

		ch.Metadata.degrees[toNodeID]++

	}

	if !dupf || !dupb {
		ch.GraphStorage.AppendEdgeStorage(datastructure.NewEdge(
			currEdgeID, toNodeID, fromNodeID, contractedNodeID, weight, dist,
		))
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

func (ch *ContractedGraph) AddEdge(newEdge datastructure.Edge) {

	fromNodeID := newEdge.FromNodeID
	toNodeID := newEdge.ToNodeID

	currEdgeID := int32(len(ch.GraphStorage.EdgeStorage)) // new edge ID

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
		edge := ch.GetOutEdge(outID)
		if edge.ToNodeID == toNodeID {
			return
		}
	}

	// add outEdge
	ch.GraphStorage.AppendEdgeStorage(newEdge) // only add edgestorage once

	ch.ContractedFirstOutEdge[fromNodeID] = append(ch.ContractedFirstOutEdge[fromNodeID], currEdgeID)

	// add inEdge

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

func (ch *ContractedGraph) GetNodeFirstOutEdges(nodeID int32) []int32 {
	return ch.ContractedFirstOutEdge[nodeID]
}

func (ch *ContractedGraph) GetNodeFirstInEdges(nodeID int32) []int32 {
	return ch.ContractedFirstInEdge[nodeID]
}

func (ch *ContractedGraph) GetOutEdge(edgeID int32) datastructure.Edge {
	return ch.GraphStorage.GetOutEdge(edgeID)
}

func (ch *ContractedGraph) GetInEdge(edgeID int32) datastructure.Edge {
	return ch.GraphStorage.GetInEdge(edgeID)
}

func (ch *ContractedGraph) GetOutEdges() []datastructure.Edge {
	return ch.GraphStorage.GetOutEdges()
}

func (ch *ContractedGraph) GetOutEdgesLen() int {
	return ch.GraphStorage.GetOutEdgesLen()
}

func (ch *ContractedGraph) GetInEdges() []datastructure.Edge {
	return ch.GraphStorage.GetInEdges()
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

func (ch *ContractedGraph) GetStreetDirection(streetName int) [2]bool {
	return ch.StreetDirection[streetName]
}

func (ch *ContractedGraph) GetStreetNameFromID(streetName int) string {
	return ch.TagStringIDMap.GetStr(streetName)
}

func (ch *ContractedGraph) GetRoadClassFromID(roadClass uint8) string {
	return ch.TagStringIDMap.GetStr(int(roadClass))
}

func (ch *ContractedGraph) GetRoadClassLinkFromID(roadClassLink uint8) string {
	return ch.TagStringIDMap.GetStr(int(roadClassLink))
}

func (ch *ContractedGraph) IsShortcut(edgeID int32) bool {
	shortcut := ch.GraphStorage.IsShortcut(edgeID)
	return shortcut
}

func (ch *ContractedGraph) IsRoundabout(edgeID int32) bool {
	_, roundabout := ch.GraphStorage.GetEdgeExtraInfo(edgeID, false)
	return roundabout
}
func (ch *ContractedGraph) GetEdgeInfo(edgeID int32) datastructure.EdgeExtraInfo {
	edgeInfo, _ := ch.GraphStorage.GetEdgeExtraInfo(edgeID, false)
	return edgeInfo
}

func (ch *ContractedGraph) GetEdgePointsInBetween(edgeID int32) []datastructure.Coordinate {
	return ch.GraphStorage.GetPointsInbetween(edgeID)
}

func (ch *ContractedGraph) SetPointsInBetween(edgeID int32, points []datastructure.Coordinate) (int, int) {
	startIndex := len(ch.GraphStorage.GlobalPoints)
	ch.GraphStorage.AppendGlobalPoints(points)
	endIndex := len(ch.GraphStorage.GlobalPoints)
	return startIndex, endIndex
}

func (ch *ContractedGraph) IsTrafficLight(nodeID int32) bool {
	trafficLight := ch.GraphStorage.GetTrafficLight(nodeID)

	return trafficLight
}

func (ch *ContractedGraph) SetEdgeInfo(edgeInfo datastructure.EdgeExtraInfo) {
	ch.GraphStorage.AppendMapEdgeInfo(edgeInfo)
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
	defer func() {
		runtime.GC() // reduce heap size after loading graph
		runtime.GC() // reduce heap size after loading graph
	}()
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
