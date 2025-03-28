package contractor

import (
	"bytes"
	"encoding/gob"
	"io"
	"log"
	"os"
	"runtime"
	"sort"
	"time"

	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"lintang/navigatorx/pkg/server"
	"lintang/navigatorx/pkg/util"
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

	// compressed sparse row representation
	CsrN        []int32 // indexed by "from" ID; outdeg(v) == N[1+v]-N[v]. (notes: nodeID are 1-indexed)
	CsrF        []int32 // "to" vertexIDs accessed via N[]
	CsrNRev     []int32 // indexed by "to" ID; indeg(v) == N[1+v]-N[v]. (notes: nodeID are 1-indexed)
	CsrFRev     []int32 // "from" vertexIDs accessed via N[]
	CsrOutEdges []datastructure.EdgeCH
	CsrInEdges  []datastructure.EdgeCH

	// edge extra info
	EdgeInfo *datastructure.EdgeInfo
	// node extra info
	NodeInfo *datastructure.NodeInfo

	StreetDirection map[int][2]bool // 0 = forward, 1 = backward
	TagStringIDMap  util.IDMap
}

var maxPollFactorHeuristic = 5
var maxPollFactorContraction = 200

func NewContractedGraph() *ContractedGraph {
	return &ContractedGraph{
		ContractedOutEdges: make([]datastructure.EdgeCH, 0),
		ContractedInEdges:  make([]datastructure.EdgeCH, 0),
		ContractedNodes:    make([]datastructure.CHNode, 0),
		Ready:              false,
		StreetDirection:    make(map[int][2]bool),
		TagStringIDMap:     util.NewIdMap(),
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

	edgeInfo := *otherGraph.EdgeInfo
	nodeInfo := *otherGraph.NodeInfo

	return &ContractedGraph{
		ContractedOutEdges:     qOutEdges,
		ContractedInEdges:      qInEdges,
		ContractedNodes:        qNodes,
		ContractedFirstOutEdge: qFirstOutEdges,
		ContractedFirstInEdge:  qFirstInEdges,
		EdgeInfo:               &edgeInfo,
		NodeInfo:               &nodeInfo,
	}
}

func (ch *ContractedGraph) InitCHGraph(processedNodes []datastructure.CHNode,
	processedEdges []datastructure.EdgeCH, streetDirections map[string][2]bool,
	tagStringIdMap util.IDMap, edgesExtraInfo []datastructure.EdgeExtraInfo,
	nodeInfo *datastructure.NodeInfo) {

	ch.TagStringIDMap = tagStringIdMap

	ch.saveExtraInfo(edgesExtraInfo, nodeInfo)

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
		ch.ContractedOutEdges = append(ch.ContractedOutEdges, datastructure.NewEdgeCH(
			outEdgeID, edge.Weight, edge.Dist, edge.ToNodeID, edge.FromNodeID, -1, -1,
		))

		outEdgeID++
		ch.Metadata.OutEdgeOrigCount[edge.FromNodeID]++

		// in Edges
		ch.ContractedFirstInEdge[edge.ToNodeID] = append(ch.ContractedFirstInEdge[edge.ToNodeID], int32(inEdgeID))

		ch.ContractedInEdges = append(ch.ContractedInEdges, datastructure.NewEdgeCH(
			inEdgeID, edge.Weight, edge.Dist, edge.FromNodeID, edge.ToNodeID, -1, -1,
		))

		inEdgeID++
		ch.Metadata.InEdgeOrigCount[edge.FromNodeID]++

		// tambah degree nodenya
		ch.Metadata.degrees[edge.FromNodeID]++
		ch.Metadata.degrees[edge.ToNodeID]++

	}

	log.Printf("initializing osm graph done...")

	ch.Metadata.EdgeCount = len(ch.ContractedOutEdges)
	ch.Metadata.NodeCount = gLen
	ch.Metadata.MeanDegree = float64(len(ch.ContractedOutEdges) * 1.0 / gLen)

}

func (ch *ContractedGraph) saveExtraInfo(edgesExtraInfo []datastructure.EdgeExtraInfo, nodeInfo *datastructure.NodeInfo) {
	ch.EdgeInfo = datastructure.NewEdgeEdgeInfo()
	for i, info := range edgesExtraInfo {
		ch.EdgeInfo.SetEdgeInfo(int32(i), info.StreetName,
			info.RoadClass, info.Lanes, info.RoadClassLink,
			info.PointsInBetween, info.Roundabout, info.IsShortcut)
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
	orderNum := int64(0)

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
		isShortcut := ch.IsShortcut(edge.EdgeID)
		if edge.ToNodeID != toNodeID || !isShortcut {
			continue
		}
		exists = true
		if weight < edge.Weight {
			edge.Weight = weight
		}
	}

	for _, inID := range ch.ContractedFirstInEdge[toNodeID] {
		edge := ch.ContractedInEdges[inID]
		isShortcut := ch.IsShortcut(edge.EdgeID)
		if edge.ToNodeID != fromNodeID || !isShortcut {
			continue
		}
		exists = true
		if weight < edge.Weight {
			edge.Weight = weight
		}
	}

	if !exists {
		ch.addShortcut(fromNodeID, toNodeID, weight, removedEdgeOne, removedEdgeTwo)
		ch.Metadata.ShortcutsCount++
	}
}

func (ch *ContractedGraph) addShortcut(fromNodeID, toNodeID int32, weight float64, removedEdgeOne, removedEdgeTwo *datastructure.EdgeCH,
) {

	fromN := ch.ContractedNodes[fromNodeID]
	toN := ch.ContractedNodes[toNodeID]

	dist := geo.CalculateHaversineDistance(fromN.Lat, fromN.Lon, toN.Lat, toN.Lon)
	// add shortcut outcoming edge

	currEdgeID := int32(len(ch.ContractedOutEdges))

	ch.ContractedOutEdges = append(ch.ContractedOutEdges, datastructure.NewEdgeCH(
		currEdgeID, weight, dist, toNodeID, fromNodeID, removedEdgeOne.EdgeID, removedEdgeTwo.EdgeID,
	))

	ch.ContractedFirstOutEdge[fromNodeID] = append(ch.ContractedFirstOutEdge[fromNodeID], currEdgeID)
	ch.Metadata.degrees[fromNodeID]++

	ch.EdgeInfo.SetShortcut(currEdgeID)

	currEdgeID = int32(len(ch.ContractedInEdges))

	ch.ContractedInEdges = append(ch.ContractedInEdges, datastructure.NewEdgeCH(
		currEdgeID, weight, dist, fromNodeID, toNodeID, removedEdgeOne.EdgeID, removedEdgeTwo.EdgeID,
	))

	ch.ContractedFirstInEdge[toNodeID] = append(ch.ContractedFirstInEdge[toNodeID], currEdgeID)

	ch.Metadata.degrees[toNodeID]++

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

func (ch *ContractedGraph) IsShortcut(edgeID int32) bool {

	shortcut, ok := ch.EdgeInfo.IsShortcut[edgeID]
	if !ok {
		return false
	}
	return shortcut
}

func (ch *ContractedGraph) IsRoundabout(edgeID int32) bool {
	roundabout, ok := ch.EdgeInfo.Roundabout[edgeID]
	if !ok {
		return false
	}
	return roundabout
}

func (ch *ContractedGraph) GetEdgePointsInBetween(edgeID int32) []datastructure.Coordinate {
	return ch.EdgeInfo.PointsInBetween[edgeID]
}

func (ch *ContractedGraph) GetStreetName(edgeID int32) int {
	return ch.EdgeInfo.StreetName[edgeID]
}

func (ch *ContractedGraph) GetRoadClass(edgeID int32) int {
	return ch.EdgeInfo.RoadClass[edgeID]
}

func (ch *ContractedGraph) GetRoadClassLink(edgeID int32) int {
	return ch.EdgeInfo.RoadClassLink[edgeID]
}

func (ch *ContractedGraph) GetLanes(edgeID int32) int {
	return ch.EdgeInfo.Lanes[edgeID]
}

func (ch *ContractedGraph) IsTrafficLight(nodeID int32) bool {
	trafficLight, ok := ch.NodeInfo.TrafficLight[nodeID]
	if !ok {
		return false
	}
	return trafficLight
}

func (ch *ContractedGraph) SetPointsInBetween(edgeID int32, pointsInBetween []datastructure.Coordinate) {
	ch.EdgeInfo.PointsInBetween[edgeID] = pointsInBetween
}

func (ch *ContractedGraph) AppendEdgeInfo(edgeInfo datastructure.EdgeExtraInfo) {
	ch.EdgeInfo.SetEdgeInfo(int32(len(ch.EdgeInfo.IsShortcut)), edgeInfo.StreetName, edgeInfo.RoadClass, edgeInfo.Lanes, edgeInfo.RoadClassLink,
		edgeInfo.PointsInBetween, edgeInfo.Roundabout, edgeInfo.IsShortcut)
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

// https://www.usenix.org/system/files/login/articles/login_winter20_16_kelly.pdf
func (ch *ContractedGraph) BuildCompressedSparseRow() {
	ch.CsrN = make([]int32, len(ch.ContractedNodes)+2)
	ch.CsrF = make([]int32, len(ch.ContractedOutEdges))
	ch.CsrNRev = make([]int32, len(ch.ContractedNodes)+2)
	ch.CsrFRev = make([]int32, len(ch.ContractedInEdges))

	ch.CsrOutEdges = make([]datastructure.EdgeCH, 0, len(ch.ContractedOutEdges))

	for i := range ch.ContractedNodes {

		// add csrOutEdges
		nodeEdgeIDs := ch.GetNodeFirstOutEdges(int32(i))
		nodeEdges := make([]datastructure.EdgeCH, len(nodeEdgeIDs))
		for j, edgeID := range nodeEdgeIDs {
			nodeEdges[j] = ch.GetOutEdge(edgeID)
		}

		sort.Slice(nodeEdges, func(i, j int) bool {
			return nodeEdges[i].ToNodeID < nodeEdges[j].ToNodeID
		})

		for _, edge := range nodeEdges {
			ch.CsrOutEdges = append(ch.CsrOutEdges, datastructure.EdgeCH{
				Weight:         edge.Weight,
				Dist:           edge.Dist,
				RemovedEdgeOne: edge.RemovedEdgeOne,
				RemovedEdgeTwo: edge.RemovedEdgeTwo,
				FromNodeID:     edge.FromNodeID + 1,
				ToNodeID:       edge.ToNodeID + 1,
			})
		}

		// add csrInEdges
		nodeEdgeIDs = ch.GetNodeFirstInEdges(int32(i))
		nodeEdges = make([]datastructure.EdgeCH, len(nodeEdgeIDs))
		for j, edgeID := range nodeEdgeIDs {
			nodeEdges[j] = ch.GetInEdge(edgeID)
		}

		sort.Slice(nodeEdges, func(i, j int) bool {
			return nodeEdges[i].ToNodeID < nodeEdges[j].ToNodeID
		})

		for _, edge := range nodeEdges {
			// edgeExtraInfo := ch.GetEdgeExtraInfo(edge.EdgeID)
			ch.CsrInEdges = append(ch.CsrInEdges, datastructure.EdgeCH{
				Weight:         edge.Weight,
				Dist:           edge.Dist,
				RemovedEdgeOne: edge.RemovedEdgeOne,
				RemovedEdgeTwo: edge.RemovedEdgeTwo,
				FromNodeID:     edge.FromNodeID + 1,
				ToNodeID:       edge.ToNodeID + 1,
			})
		}
	}

	// notes: nodeID in contractedNodes and ContractedOutEdges are 0-indexed, but
	// nodeID in compressed sparse row are 1-indexed

	v := int32(len(ch.ContractedNodes))

	// populate number of edges for each node
	// in this step.for each node a | a \in [1,v]. N[a+1] = number of out edges for node a
	for a := int32(1); a <= v; a++ {
		origNodeID := a - 1

		// a = nodeID for N (1-indexed). from 1 to V
		for _, edgeID := range ch.GetNodeFirstOutEdges(int32(origNodeID)) {
			edge := ch.GetOutEdge(edgeID)
			b := edge.ToNodeID + 1

			ch.CsrN[a]++
			ch.CsrNRev[b]++
		}
	}

	t := int32(0)

	// calculate prefix sum.for each node a | a \in  [1,v+1]. N[a] is prefix sum of N[a-1] + N[a]. N[V+1] = E. E = number of edges
	// in this step, the out-degree of any vertex a is N[a+1] minus N[a].
	for a := int32(1); a <= v; a++ {
		t += ch.CsrN[a]
		ch.CsrN[a] = t
	}

	ch.CsrN[v+1] = t

	t = int32(0)

	// calculate prefix sum.for each node b | b \in  [1,v+1]. N[b] is prefix sum of N[b-1] + N[b]. N[V+1] = E. E = number of edges
	// in this step, the out-degree of any vertex b is N[b+1] minus N[b].
	for b := int32(1); b <= v; b++ {
		t += ch.CsrNRev[b]
		ch.CsrNRev[b] = t
	}

	ch.CsrNRev[v+1] = t

	// add edges to csrF
	// for each node a | a \in [1,v]. add edges sequentially from index N[a+1]-1 to N[a] inclusive
	// in this step. from F[N[a]] to F[N[a+1]-1] inclusive contains the out edges for node a

	for a := int32(1); a <= v; a++ {
		// a = nodeID for N (1-indexed). from 1 to V
		for _, edgeID := range ch.GetNodeFirstOutEdges(int32(a - 1)) {
			edge := ch.GetOutEdge(edgeID)
			b := edge.ToNodeID + 1

			ch.CsrN[a]--
			ch.CsrF[ch.CsrN[a]] = b // add directed edge a->b

			ch.CsrNRev[b]--
			ch.CsrFRev[ch.CsrNRev[b]] = a // add directed edge b->a
		}
	}

	// sort outgoing edge IDs for each vertex

	for a := int32(1); a <= v; a++ {
		begin := ch.CsrN[a]
		end := ch.CsrN[a+1]
		sort.Slice(ch.CsrF[begin:end], func(i, j int) bool {
			return ch.CsrF[begin+int32(i)] < ch.CsrF[begin+int32(j)]
		})

		begin = ch.CsrNRev[a]
		end = ch.CsrNRev[a+1]
		sort.Slice(ch.CsrFRev[begin:end], func(i, j int) bool {
			return ch.CsrFRev[begin+int32(i)] < ch.CsrFRev[begin+int32(j)]
		})
	}

	for i := range ch.ContractedNodes {
		ch.ContractedNodes[i].ID++
	}

	ch.ContractedNodes = append(make([]datastructure.CHNode, 1), ch.ContractedNodes...)
	ch.removeOldGraph()
}

func (ch *ContractedGraph) removeOldGraph() {
	// empty old contracted graph
	v := int32(len(ch.ContractedNodes))
	for a := int32(1); a <= v; a++ {
		origNodeID := a - 1
		ch.ContractedNodes[origNodeID].ID = int32(a)

	}

	ch.ContractedOutEdges = make([]datastructure.EdgeCH, 0)
	ch.ContractedInEdges = make([]datastructure.EdgeCH, 0)
	ch.ContractedFirstOutEdge = make([][]int32, 0)
	ch.ContractedFirstInEdge = make([][]int32, 0)
}

// GetNodeOutEdgesCsr. return out edges for nodeID in format ([]toNodeIDs, []EdgeCH)
func (ch *ContractedGraph) GetNodeOutEdgesCsr(nodeID int32) ([]int32, []datastructure.EdgeCH) {
	// in this step. from F[N[a]] to F[N[a+1]-1] inclusive (or  F[ [ N[a], N[a+1] ) ] ) contains the out edges for node a
	return ch.CsrF[ch.CsrN[nodeID]:ch.CsrN[nodeID+1]], ch.CsrOutEdges[ch.CsrN[nodeID]:ch.CsrN[nodeID+1]]
}

func (ch *ContractedGraph) GetNodeInEdgesCsr(nodeID int32) ([]int32, []datastructure.EdgeCH) {
	// in this step. from FRev[NRev[a]] to FRev[NRev[a+1]-1] inclusive (or  FRev[ [ NRev[a], NRev[a+1] ) ] ) contains the in edges for node a

	return ch.CsrFRev[ch.CsrNRev[nodeID]:ch.CsrNRev[nodeID+1]], ch.CsrInEdges[ch.CsrNRev[nodeID]:ch.CsrNRev[nodeID+1]]
}
