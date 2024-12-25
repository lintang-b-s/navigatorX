package contractor

import (
	"bytes"
	"encoding/gob"
	"errors"
	"fmt"
	"io"
	"os"
	"runtime"
	"time"

	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"lintang/navigatorx/pkg/server"
	"lintang/navigatorx/pkg/util"

	"github.com/k0kubun/go-ansi"

	"github.com/schollz/progressbar/v3"
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
	ContractedNodes        []datastructure.CHNode2

	IsLoaded           bool
	StreetDirection    map[int][2]bool // 0 = forward, 1 = backward
	StreetInfo         map[int]datastructure.StreetExtraInfo
	StreetNameIDMap    util.IDMap
	RoadClassIDMap     util.IDMap
	RoadClassLinkIDMap util.IDMap
}

var maxPollFactorHeuristic = 5
var maxPollFactorContraction = 200

func NewContractedGraph() *ContractedGraph {
	return &ContractedGraph{
		ContractedOutEdges: make([]datastructure.EdgeCH, 0),
		ContractedInEdges:  make([]datastructure.EdgeCH, 0),
		ContractedNodes:    make([]datastructure.CHNode2, 0),
		Ready:              false,
		StreetDirection:    make(map[int][2]bool),
		StreetInfo:         make(map[int]datastructure.StreetExtraInfo),
		StreetNameIDMap:    util.NewIdMap(),
		RoadClassIDMap:     util.NewIdMap(),
		RoadClassLinkIDMap: util.NewIdMap(),
	}
}

func (ch *ContractedGraph) InitCHGraph(nodes []datastructure.Node, edgeCount int, streetDirections map[string][2]bool,
	streetExtraInfo map[string]datastructure.StreetExtraInfo) map[int64]int32 {
	gLen := len(nodes)

	var nodeIdxMap = make(map[int64]int32)
	for streetName, direction := range streetDirections {
		ch.StreetDirection[ch.StreetNameIDMap.GetID(streetName)] = direction
	}
	for streetName, extraInfo := range streetExtraInfo {
		ch.StreetInfo[ch.StreetNameIDMap.GetID(streetName)] = extraInfo
	}

	bar := progressbar.NewOptions(3,
		progressbar.OptionSetWriter(ansi.NewAnsiStdout()),
		progressbar.OptionEnableColorCodes(true),
		progressbar.OptionShowBytes(true),
		progressbar.OptionSetWidth(15),
		progressbar.OptionSetDescription("[cyan][3/7][reset] Membuat graph..."),
		progressbar.OptionSetTheme(progressbar.Theme{
			Saucer:        "[green]=[reset]",
			SaucerHead:    "[green]>[reset]",
			SaucerPadding: " ",
			BarStart:      "[",
			BarEnd:        "]",
		}))

	for i, node := range nodes {

		ch.ContractedNodes = append(ch.ContractedNodes, datastructure.CHNode2{
			IDx:          int32(i),
			Lat:          node.Lat,
			Lon:          node.Lon,
			TrafficLight: node.TrafficLight,
		})

		nodeIdxMap[node.ID] = int32(i)

	}

	bar.Add(1)
	ch.Metadata.degrees = make([]int, gLen)
	ch.Metadata.InEdgeOrigCount = make([]int, gLen)
	ch.Metadata.OutEdgeOrigCount = make([]int, gLen)
	ch.Metadata.ShortcutsCount = 0

	outEdgeIDx := int32(0)
	inEdgeIDx := int32(0)
	ch.ContractedFirstOutEdge = make([][]int32, len(ch.ContractedNodes))
	ch.ContractedFirstInEdge = make([][]int32, len(ch.ContractedNodes))

	// init graph original
	for idx, node := range nodes {
		outEdgeCounter := 0
		for _, edge := range node.Out_to {
			maxSpeed := edge.MaxSpeed * 1000 / 60 // m /min
			cost := edge.Cost / maxSpeed
			toIdx := nodeIdxMap[edge.To.ID]
			streetName := ch.StreetNameIDMap.GetID(edge.StreetName)
			roadClass := ch.RoadClassIDMap.GetID(edge.RoadClass)
			roadClassLink := ch.RoadClassLinkIDMap.GetID(edge.RoadClassLink)

			ch.ContractedFirstOutEdge[idx] = append(ch.ContractedFirstOutEdge[idx], int32(outEdgeIDx))
			ch.ContractedOutEdges = append(ch.ContractedOutEdges, datastructure.EdgeCH{outEdgeIDx, cost, edge.Cost, toIdx,
				int32(idx), false, -1, -1, streetName, edge.Roundabout,
				roadClass, roadClassLink, edge.Lanes, edge.NodesInBetween})

			// tambah degree nodenya
			ch.Metadata.degrees[idx]++
			outEdgeCounter++

			outEdgeIDx++
		}
		ch.Metadata.OutEdgeOrigCount[idx] = outEdgeCounter
	}

	bar.Add(1)
	// init graph edge dibalik
	for i := 0; i < gLen; i++ {
		inEdgeCounter := 0
		for _, outIDx := range ch.ContractedFirstOutEdge[i] {
			edge := ch.ContractedOutEdges[outIDx]
			to := edge.ToNodeIDX
			weight := edge.Weight
			nodesInBetween := edge.NodesInBetween // tidak perlu direverse
			ch.ContractedFirstInEdge[to] = append(ch.ContractedFirstInEdge[to], int32(inEdgeIDx))

			ch.ContractedInEdges = append(ch.ContractedInEdges, datastructure.EdgeCH{inEdgeIDx, weight,
				edge.Dist, int32(i), to, false, -1, -1, edge.StreetName,
				edge.Roundabout, edge.RoadClass, edge.RoadClassLink, edge.Lanes, nodesInBetween})

			// tambah degree nodenya
			ch.Metadata.degrees[i]++
			inEdgeCounter++

			inEdgeIDx++
		}
		ch.Metadata.InEdgeOrigCount[i] = inEdgeCounter
	}

	bar.Add(1)

	ch.Metadata.EdgeCount = edgeCount
	ch.Metadata.NodeCount = gLen
	ch.Metadata.MeanDegree = float64(edgeCount * 1.0 / gLen)

	return nodeIdxMap
}

func (ch *ContractedGraph) Contraction() (err error) {
	st := time.Now()
	nq := NewMinHeap[int32]()

	ch.UpdatePrioritiesOfRemainingNodes(nq) // bikin node ordering

	level := 0
	contracted := make([]bool, ch.Metadata.NodeCount)
	orderNum := int64(0)

	bar := progressbar.NewOptions(nq.Size(),
		progressbar.OptionSetWriter(ansi.NewAnsiStdout()),
		progressbar.OptionEnableColorCodes(true),
		progressbar.OptionShowBytes(true),
		progressbar.OptionSetWidth(15),
		progressbar.OptionSetDescription("[cyan][7/7][reset] Membuat contracted graph (contraction hiearchies)..."),
		progressbar.OptionSetTheme(progressbar.Theme{
			Saucer:        "[green]=[reset]",
			SaucerHead:    "[green]>[reset]",
			SaucerPadding: " ",
			BarStart:      "[",
			BarEnd:        "]",
		}))
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
		bar.Add(1)
	}
	fmt.Printf("\ntotal shortcuts dibuat: %d \n", ch.Metadata.ShortcutsCount)

	ch.Metadata = Metadata{}
	runtime.GC()
	runtime.GC()
	end := time.Now().Sub(st)
	fmt.Printf("lama preprocessing contraction hierarchies: %v menit\n", end.Minutes())
	return
}

func (ch *ContractedGraph) contractNode(nodeIDx int32, isContracted bool, contracted []bool) {

	if isContracted {
		return
	}
	ch.contractNodeNow(nodeIDx, contracted)

}

func (ch *ContractedGraph) contractNodeNow(nodeIDx int32, contracted []bool) {
	degree, _, _, _ := ch.findAndHandleShortcuts(nodeIDx, ch.addOrUpdateShortcut, int(ch.Metadata.MeanDegree*float64(maxPollFactorContraction)),
		contracted)
	ch.Metadata.MeanDegree = (ch.Metadata.MeanDegree*2 + float64(degree)) / 3

}

/*
findAndHandleShortcuts , ketika mengontraksi node v, kita  harus cari shortest path dari node u ke w yang meng ignore node v, dimana u adalah node yang terhubung ke v dan edge (u,v) \in E, dan w adalah node yang terhubung dari v dan edge (v,w) \in E.
kalau cost dari shortest path u->w  <= c(u,v) + c(v,w) , tambahkan shortcut edge (u,w).
*/
func (ch *ContractedGraph) findAndHandleShortcuts(nodeIDx int32, shortcutHandler func(fromNodeIDx, toNodeIDx int32, nodeIdx int32, weight float64,
	removedEdgeOne, removedEdgeTwo *datastructure.EdgeCH,
	outOrigEdgeCount, inOrigEdgeCount int),
	maxVisitedNodes int, contracted []bool) (int, int, int, error) {
	degree := 0
	shortcutCount := 0      // jumlah shortcut yang ditambahkan
	originalEdgesCount := 0 // += InEdgeCount(v) + OutEdgeCount(v)  setiap kali shortcut ditambahkan
	pMax := 0.0             // maximum cost path dari node u ke w, dimana u adalah semua node yang terhubung ke v & (u,v) \in E dan w adalah semua node yang terhubung ke v & (v, w) \in E
	pInMax := 0.0
	pOutMax := 0.0

	for _, idx := range ch.ContractedFirstInEdge[nodeIDx] {
		inEdge := ch.ContractedInEdges[idx]
		toNIDx := inEdge.ToNodeIDX
		if contracted[toNIDx] {
			continue
		}
		if inEdge.Weight > pInMax {
			pInMax = inEdge.Weight
		}
	}
	for _, idx := range ch.ContractedFirstOutEdge[nodeIDx] {
		outEdge := ch.ContractedOutEdges[idx]
		toNIDx := outEdge.ToNodeIDX
		if contracted[toNIDx] {
			continue
		}
		if outEdge.Weight > pOutMax {
			pOutMax = outEdge.Weight
		}
	}
	pMax = pInMax + pOutMax

	for _, inIdx := range ch.ContractedFirstInEdge[nodeIDx] {
		inEdge := ch.ContractedInEdges[inIdx]
		fromNodeIDx := inEdge.ToNodeIDX
		if fromNodeIDx == int32(nodeIDx) {
			return 0, 0, 0, errors.New(fmt.Sprintf(`unexpected loop-edge at node: %v `, nodeIDx))
		}
		if contracted[fromNodeIDx] {
			continue
		}

		incomingEdgeWeight := inEdge.Weight

		// outging edge dari nodeIDx
		degree++

		for _, outIDx := range ch.ContractedFirstOutEdge[nodeIDx] {
			outEdge := ch.ContractedOutEdges[outIDx]
			toNode := outEdge.ToNodeIDX
			if contracted[toNode] {
				continue
			}

			if toNode == fromNodeIDx {
				// gak perlu search untuk witness dari node balik ke node itu lagi
				continue
			}

			existingDirectWeight := incomingEdgeWeight + outEdge.Weight

			maxWeight := ch.dijkstraWitnessSearch(fromNodeIDx, toNode, nodeIDx, existingDirectWeight, maxVisitedNodes, pMax,
				contracted)

			if maxWeight <= existingDirectWeight {
				// FOUND witness shortest path, tidak perlu add shortcut
				continue
			}
			// kalo d(u,w) > Pw , tambah shortcut
			// Pw = existingDirectWeight = d(u,v) + d(v,w)
			// d(u,v) = shortest path dari u ke w tanpa lewatin v. Atau path dari u ke w tanpa lewatin v yang cost nya <= Pw.
			shortcutCount++
			originalEdgesCount += ch.Metadata.InEdgeOrigCount[nodeIDx] + ch.Metadata.OutEdgeOrigCount[nodeIDx]
			shortcutHandler(fromNodeIDx, toNode, nodeIDx, existingDirectWeight, &inEdge, &outEdge,
				ch.Metadata.OutEdgeOrigCount[nodeIDx], ch.Metadata.InEdgeOrigCount[nodeIDx])

		}
	}
	return degree, shortcutCount, originalEdgesCount, nil
}

func countShortcut(fromNodeIDx, toNodeIDx int32, nodeIDx int32, weight float64, removedEdgeOne, removedEdgeTwo *datastructure.EdgeCH,
	outOrigEdgeCount, inOrigEdgeCount int) {
	// shortcutCount++
}

/*
addOrUpdateShortcut, menambahkan shortcut (u,w) jika path dari u->w tanpa lewati v cost nya lebih kecil dari c(u,v) + c(v,w).
*/
func (ch *ContractedGraph) addOrUpdateShortcut(fromNodeIDx, toNodeIDx int32, nodeIDx int32, weight float64, removedEdgeOne, removedEdgeTwo *datastructure.EdgeCH,
	outOrigEdgeCount, inOrigEdgeCount int) {

	exists := false
	for _, outIDx := range ch.ContractedFirstOutEdge[fromNodeIDx] {
		edge := ch.ContractedOutEdges[outIDx]
		if edge.ToNodeIDX != toNodeIDx || !edge.IsShortcut {
			continue
		}
		exists = true
		if weight < edge.Weight {
			edge.Weight = weight
		}
	}

	for _, inIDx := range ch.ContractedFirstInEdge[toNodeIDx] {
		edge := ch.ContractedInEdges[inIDx]
		if edge.ToNodeIDX != fromNodeIDx || !edge.IsShortcut {
			continue
		}
		exists = true
		if weight < edge.Weight {
			edge.Weight = weight
		}
	}

	if !exists {
		ch.addShortcut(fromNodeIDx, toNodeIDx, weight, removedEdgeOne, removedEdgeTwo)
		ch.Metadata.ShortcutsCount++
	}
}

func (ch *ContractedGraph) addShortcut(fromNodeIDx, toNodeIDx int32, weight float64, removedEdgeOne, removedEdgeTwo *datastructure.EdgeCH) {

	fromN := ch.ContractedNodes[fromNodeIDx]
	toN := ch.ContractedNodes[toNodeIDx]
	fromLoc := geo.NewLocation(fromN.Lat, fromN.Lon)
	toLoc := geo.NewLocation(toN.Lat, toN.Lon)
	dist := geo.HaversineDistance(fromLoc, toLoc)
	// add shortcut outcoming edge
	dup := false
	for _, outIDx := range ch.ContractedFirstOutEdge[fromNodeIDx] {
		edge := ch.ContractedOutEdges[outIDx]
		if edge.ToNodeIDX == toNodeIDx && edge.Weight > weight {
			edge.Weight = weight
			edge.Dist = dist
			edge.RemovedEdgeOne = removedEdgeOne.EdgeIDx
			edge.RemovedEdgeTwo = removedEdgeTwo.EdgeIDx
			dup = true
			break
		}
	}
	if !dup {

		currEdgeIDx := int32(len(ch.ContractedOutEdges))
		ch.ContractedOutEdges = append(ch.ContractedOutEdges, datastructure.EdgeCH{currEdgeIDx, weight, dist, toNodeIDx, fromNodeIDx, true,
			removedEdgeOne.EdgeIDx, removedEdgeTwo.EdgeIDx, 0, false, 0, 0, 0, nil})
		ch.ContractedFirstOutEdge[fromNodeIDx] = append(ch.ContractedFirstOutEdge[fromNodeIDx], currEdgeIDx)
		ch.Metadata.degrees[fromNodeIDx]++
	}

	dup = false
	// add shortcut incoming edge
	for _, inIDx := range ch.ContractedFirstInEdge[toNodeIDx] {
		edge := ch.ContractedInEdges[inIDx]
		if edge.ToNodeIDX == fromNodeIDx && edge.Weight > weight {
			edge.Weight = weight
			edge.Dist = dist
			edge.RemovedEdgeOne = removedEdgeOne.EdgeIDx
			edge.RemovedEdgeTwo = removedEdgeTwo.EdgeIDx
			dup = true
			break
		}

	}
	if !dup {

		currEdgeIDx := int32(len(ch.ContractedInEdges))
		ch.ContractedInEdges = append(ch.ContractedInEdges, datastructure.EdgeCH{currEdgeIDx, weight, dist, fromNodeIDx, toNodeIDx, true,
			removedEdgeOne.EdgeIDx, removedEdgeTwo.EdgeIDx, 0, false, 0, 0, 0, nil})
		ch.ContractedFirstInEdge[toNodeIDx] = append(ch.ContractedFirstInEdge[toNodeIDx], currEdgeIDx)

		ch.Metadata.degrees[toNodeIDx]++

	}
}

func (ch *ContractedGraph) calculatePriority(nodeIDx int32, contracted []bool) float64 {

	_, shortcutsCount, originalEdgesCount, _ := ch.findAndHandleShortcuts(nodeIDx, countShortcut, int(ch.Metadata.MeanDegree*float64(maxPollFactorHeuristic)),
		contracted)

	// |shortcuts(v)| − |{(u, v) | v uncontracted}| − |{(v, w) | v uncontracted}|
	// outDegree+inDegree
	edgeDifference := shortcutsCount - ch.Metadata.degrees[nodeIDx]

	return float64(10*edgeDifference + 1*originalEdgesCount)
}

func (ch *ContractedGraph) UpdatePrioritiesOfRemainingNodes(nq *MinHeap[int32]) {

	bar := progressbar.NewOptions(len(ch.ContractedNodes),
		progressbar.OptionSetWriter(ansi.NewAnsiStdout()),
		progressbar.OptionEnableColorCodes(true),
		progressbar.OptionShowBytes(true),
		progressbar.OptionSetWidth(15),
		progressbar.OptionSetDescription("[cyan][6/7][reset] Membuat node ordering (contraction hiearchies)..."),
		progressbar.OptionSetTheme(progressbar.Theme{
			Saucer:        "[green]=[reset]",
			SaucerHead:    "[green]>[reset]",
			SaucerPadding: " ",
			BarStart:      "[",
			BarEnd:        "]",
		}))

	contracted := make([]bool, ch.Metadata.NodeCount)

	for nodeIDx, _ := range ch.ContractedNodes {

		priority := ch.calculatePriority(int32(nodeIDx), contracted)
		nq.Insert(PriorityQueueNode[int32]{Item: int32(nodeIDx), Rank: priority})
		bar.Add(1)
	}
	fmt.Println("")
}

func (ch *ContractedGraph) IsChReady() bool {
	return ch.Ready
}

func (ch *ContractedGraph) GetFirstOutEdge(nodeIDx int32) []int32 {
	return ch.ContractedFirstOutEdge[nodeIDx]
}

func (ch *ContractedGraph) GetFirstInEdge(nodeIDx int32) []int32 {
	return ch.ContractedFirstInEdge[nodeIDx]
}

func (ch *ContractedGraph) GetOutEdge(edgeIDx int32) datastructure.EdgeCH {
	return ch.ContractedOutEdges[edgeIDx]
}

func (ch *ContractedGraph) GetInEdge(edgeIDx int32) datastructure.EdgeCH {
	return ch.ContractedInEdges[edgeIDx]
}

func (ch *ContractedGraph) GetOutEdges() []datastructure.EdgeCH {
	return ch.ContractedOutEdges
}

func (ch *ContractedGraph) GetInEdges() []datastructure.EdgeCH {
	return ch.ContractedInEdges
}

func (ch *ContractedGraph) GetNode(nodeIDx int32) datastructure.CHNode2 {
	return ch.ContractedNodes[nodeIDx]
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

func (ch *ContractedGraph) GetStreetInfo(streetName int) datastructure.StreetExtraInfo {
	return ch.StreetInfo[streetName]
}

func (ch *ContractedGraph) GetStreetNameFromID(streetName int) string {
	return ch.StreetNameIDMap.GetStr(streetName)
}

func (ch *ContractedGraph) GetRoadClassFromID(roadClass int) string {
	return ch.RoadClassIDMap.GetStr(roadClass)
}

func (ch *ContractedGraph) GetRoadClassLinkFromID(roadClassLink int) string {
	return ch.RoadClassLinkIDMap.GetStr(roadClassLink)
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
		fmt.Println("Error getting file info:", err)
		return err
	}
	fileSize := fileInfo.Size()

	data := make([]byte, fileSize)
	_, err = io.ReadFull(f, data)
	if err != nil {
		fmt.Println("Error reading file:", err)
		return err
	}
	dec := gob.NewDecoder(bytes.NewReader(data))
	err = dec.Decode(&ch)
	return err
}
