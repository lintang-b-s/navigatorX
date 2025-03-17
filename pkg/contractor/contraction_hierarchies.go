package contractor

import (
	"bytes"
	"encoding/gob"
	"fmt"
	"io"
	"log"
	"os"
	"runtime"
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

	StreetDirection map[int][2]bool // 0 = forward, 1 = backward
	StreetInfo      map[int]datastructure.StreetExtraInfo
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
		StreetInfo:         make(map[int]datastructure.StreetExtraInfo),
		TagStringIDMap:     util.NewIdMap(),
	}
}

func (ch *ContractedGraph) InitCHGraph(processedNodes []datastructure.CHNode,
	processedEdges []datastructure.EdgeCH, streetDirections map[string][2]bool,
	tagStringIdMap util.IDMap) {

	ch.TagStringIDMap = tagStringIdMap

	gLen := len(processedNodes)

	for streetName, direction := range streetDirections {
		ch.StreetDirection[ch.TagStringIDMap.GetID(streetName)] = direction
	}

	for _, node := range processedNodes {

		ch.ContractedNodes = append(ch.ContractedNodes, datastructure.CHNode{
			ID:           node.ID,
			Lat:          node.Lat,
			Lon:          node.Lon,
			TrafficLight: node.TrafficLight,
		})
	}

	ch.Metadata.degrees = make([]int, gLen)
	ch.Metadata.InEdgeOrigCount = make([]int, gLen)
	ch.Metadata.OutEdgeOrigCount = make([]int, gLen)
	ch.Metadata.ShortcutsCount = 0

	outEdgeID := int32(0)
	inEdgeID := int32(0)
	ch.ContractedFirstOutEdge = make([][]int32, len(ch.ContractedNodes))
	ch.ContractedFirstInEdge = make([][]int32, len(ch.ContractedNodes))

	duplicateEdges := make(map[int]map[int]struct{})

	log.Printf("intializing original osm graph...")

	// init graph original
	for _, edge := range processedEdges {

		if _, ok := duplicateEdges[int(edge.FromNodeID)]; !ok {
			duplicateEdges[int(edge.FromNodeID)] = make(map[int]struct{})
		}

		if _, ok := duplicateEdges[int(edge.FromNodeID)][int(edge.ToNodeID)]; ok {
			continue
		}

		ch.ContractedFirstOutEdge[edge.FromNodeID] = append(ch.ContractedFirstOutEdge[edge.FromNodeID], int32(outEdgeID))
		ch.ContractedOutEdges = append(ch.ContractedOutEdges, datastructure.NewEdgeCH(
			outEdgeID, edge.Weight, edge.Dist, edge.ToNodeID, edge.FromNodeID, false, -1, -1,
			edge.StreetName, edge.Roundabout, edge.RoadClass, edge.RoadClassLink, edge.Lanes, edge.PointsInBetween,
		))

		outEdgeID++
		ch.Metadata.OutEdgeOrigCount[edge.FromNodeID] = int(outEdgeID)

		ch.StreetInfo[edge.StreetName] = datastructure.StreetExtraInfo{
			// skip
		}

		// in Edges
		ch.ContractedFirstInEdge[edge.ToNodeID] = append(ch.ContractedFirstInEdge[edge.ToNodeID], int32(inEdgeID))

		ch.ContractedInEdges = append(ch.ContractedInEdges, datastructure.NewEdgeCH(
			inEdgeID, edge.Weight, edge.Dist, edge.FromNodeID, edge.ToNodeID, false, -1, -1,
			edge.StreetName, edge.Roundabout, edge.RoadClass, edge.RoadClassLink, edge.Lanes, edge.PointsInBetween,
		))

		inEdgeID++
		ch.Metadata.InEdgeOrigCount[edge.FromNodeID] = int(inEdgeID)

		// tambah degree nodenya
		ch.Metadata.degrees[edge.FromNodeID]++

		duplicateEdges[int(edge.FromNodeID)][int(edge.ToNodeID)] = struct{}{}
	}

	log.Printf("initializing osm graph done...")

	ch.Metadata.EdgeCount = len(ch.ContractedOutEdges)
	ch.Metadata.NodeCount = gLen
	ch.Metadata.MeanDegree = float64(len(ch.ContractedOutEdges) * 1.0 / gLen)

}

func (ch *ContractedGraph) Contraction() (err error) {
	st := time.Now()
	nq := NewMinHeap[int32]()

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
	log.Printf("time for preprocessing contraction hierarchies: %v menit\n", end.Minutes())
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
	originalEdgesCount := 0 // += InEdgeCount(v) + OutEdgeCount(v)  setiap kali shortcut ditambahkan
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
			return 0, 0, 0, fmt.Errorf(`unexpected loop-edge at node: %v `, nodeID)
		}
		if contracted[fromNodeID] {
			continue
		}

		incomingEdgeWeight := inEdge.Weight

		// outging edge dari nodeID
		degree++

		for _, outID := range ch.ContractedFirstOutEdge[nodeID] {
			outEdge := ch.ContractedOutEdges[outID]
			toNode := outEdge.ToNodeID
			if contracted[toNode] {
				continue
			}

			if toNode == fromNodeID {
				// gak perlu search untuk witness dari node balik ke node itu lagi
				continue
			}

			existingDirectWeight := incomingEdgeWeight + outEdge.Weight

			maxWeight := ch.dijkstraWitnessSearch(fromNodeID, toNode, nodeID, existingDirectWeight, maxVisitedNodes, pMax,
				contracted)

			if maxWeight <= existingDirectWeight {
				// FOUND witness shortest path, tidak perlu add shortcut
				continue
			}
			// kalo d(u,w) > Pw , tambah shortcut
			// Pw = existingDirectWeight = d(u,v) + d(v,w)
			// d(u,v) = shortest path dari u ke w tanpa lewatin v. Atau path dari u ke w tanpa lewatin v yang cost nya <= Pw.
			shortcutCount++
			originalEdgesCount += ch.Metadata.InEdgeOrigCount[nodeID] + ch.Metadata.OutEdgeOrigCount[nodeID]
			shortcutHandler(fromNodeID, toNode, nodeID, existingDirectWeight, &inEdge, &outEdge,
				ch.Metadata.OutEdgeOrigCount[nodeID], ch.Metadata.InEdgeOrigCount[nodeID])

		}
	}
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
		if edge.ToNodeID != toNodeID || !edge.IsShortcut {
			continue
		}
		exists = true
		if weight < edge.Weight {
			edge.Weight = weight
		}
	}

	for _, inID := range ch.ContractedFirstInEdge[toNodeID] {
		edge := ch.ContractedInEdges[inID]
		if edge.ToNodeID != fromNodeID || !edge.IsShortcut {
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

func (ch *ContractedGraph) addShortcut(fromNodeID, toNodeID int32, weight float64, removedEdgeOne, removedEdgeTwo *datastructure.EdgeCH) {

	fromN := ch.ContractedNodes[fromNodeID]
	toN := ch.ContractedNodes[toNodeID]
	fromLoc := geo.NewLocation(fromN.Lat, fromN.Lon)
	toLoc := geo.NewLocation(toN.Lat, toN.Lon)
	dist := geo.HaversineDistance(fromLoc, toLoc)
	// add shortcut outcoming edge
	dup := false
	for _, outID := range ch.ContractedFirstOutEdge[fromNodeID] {
		edge := ch.ContractedOutEdges[outID]
		if edge.ToNodeID == toNodeID && edge.Weight > weight {
			ch.ContractedOutEdges[outID].Weight = weight
			ch.ContractedOutEdges[outID].Dist = dist
			ch.ContractedOutEdges[outID].RemovedEdgeOne = removedEdgeOne.EdgeID
			ch.ContractedOutEdges[outID].RemovedEdgeTwo = removedEdgeTwo.EdgeID
			dup = true
			break
		}
	}
	if !dup {

		currEdgeID := int32(len(ch.ContractedOutEdges))

		ch.ContractedOutEdges = append(ch.ContractedOutEdges, datastructure.NewEdgeCH(
			currEdgeID, weight, dist, toNodeID, fromNodeID, true, removedEdgeOne.EdgeID, removedEdgeTwo.EdgeID, 0, false, 0, 0, 0, nil,
		))

		ch.ContractedFirstOutEdge[fromNodeID] = append(ch.ContractedFirstOutEdge[fromNodeID], currEdgeID)
		ch.Metadata.degrees[fromNodeID]++
	}

	dup = false
	// add shortcut incoming edge
	for _, inID := range ch.ContractedFirstInEdge[toNodeID] {
		edge := ch.ContractedInEdges[inID]
		if edge.ToNodeID == fromNodeID && edge.Weight > weight {
			ch.ContractedInEdges[inID].Weight = weight
			ch.ContractedInEdges[inID].Dist = dist
			ch.ContractedInEdges[inID].RemovedEdgeOne = removedEdgeOne.EdgeID
			ch.ContractedInEdges[inID].RemovedEdgeTwo = removedEdgeTwo.EdgeID
			dup = true
			break
		}

	}
	if !dup {

		currEdgeID := int32(len(ch.ContractedInEdges))

		ch.ContractedInEdges = append(ch.ContractedInEdges, datastructure.NewEdgeCH(
			currEdgeID, weight, dist, fromNodeID, toNodeID, true, removedEdgeOne.EdgeID, removedEdgeTwo.EdgeID, 0, false, 0, 0, 0, nil,
		))

		ch.ContractedFirstInEdge[toNodeID] = append(ch.ContractedFirstInEdge[toNodeID], currEdgeID)

		ch.Metadata.degrees[toNodeID]++

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
			log.Printf("updating priority of node: %d...", nodeID+1)
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

	reversedPoints := make([]datastructure.Coordinate, len(newEdge.PointsInBetween))
	copy(reversedPoints, newEdge.PointsInBetween)
	util.ReverseG(reversedPoints)
	newEdge.PointsInBetween = reversedPoints

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

func (ch *ContractedGraph) GetFirstOutNonShortcutEdge(nodeID int32) []int32 {
	nonShortcutEdges := make([]int32, 0, len(ch.ContractedFirstOutEdge[nodeID]))
	for _, edgeID := range ch.ContractedFirstOutEdge[nodeID] {
		if !ch.ContractedOutEdges[edgeID].IsShortcut {
			nonShortcutEdges = append(nonShortcutEdges, edgeID)
		}
	}
	return nonShortcutEdges
}

func (ch *ContractedGraph) GetFirstInNonShortcutEdge(nodeID int32) []int32 {
	nonShortcutEdges := make([]int32, 0, len(ch.ContractedFirstInEdge[nodeID]))
	for _, edgeID := range ch.ContractedFirstInEdge[nodeID] {
		if !ch.ContractedInEdges[edgeID].IsShortcut {
			nonShortcutEdges = append(nonShortcutEdges, edgeID)
		}
	}
	return nonShortcutEdges
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

func (ch *ContractedGraph) GetstreetInfo(streetName int) datastructure.StreetExtraInfo {
	return ch.StreetInfo[streetName]
}

func (ch *ContractedGraph) GetStreetInfo(streetName int) datastructure.StreetExtraInfo {
	return ch.StreetInfo[streetName]
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
