package disk

import (
	"bytes"
	"encoding/binary"
	"errors"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"
)

// Page . menyimpan data satu block page di dalam memori buffer (also disimpan di disk). (berukuran blockSize)
type Page struct {
	bb *bytes.Buffer
}

func NewPage(blockSize int) *Page {

	bb := bytes.NewBuffer(make([]byte, blockSize))
	return &Page{bb}
}

func NewPageFromByteSlice(b []byte) *Page {
	return &Page{bytes.NewBuffer(b)}
}

func (p *Page) GetInt(offset int32) int32 {
	return int32(binary.LittleEndian.Uint32(p.bb.Bytes()[offset:]))
}

// PutInt. set int ke byte array page di posisi = offset.
func (p *Page) PutInt(offset int32, val int32) {
	binary.LittleEndian.PutUint32(p.bb.Bytes()[offset:], uint32(val))
}

// GetBytes. return byte array dari byte array page di posisi = offset. di awal ada panjang bytes nya sehingga buat read bytes tinggal baca buffer page[offset+4:offset+4+length]
func (p *Page) GetBytes(offset int32) []byte {
	length := p.GetInt(offset)
	b := make([]byte, length)
	copy(b, p.bb.Bytes()[offset+4:offset+4+length])
	return b
}

// PutBytes. set byte array ke byte array page di posisi = offset.
func (p *Page) PutBytes(offset int32, b []byte) (int, error) {
	if offset+int32(len(b)) > int32(len(p.bb.Bytes())) {
		return 0, errors.New("put bytes out of bound")
	}
	p.PutInt(offset, int32(len(b)))
	copy(p.bb.Bytes()[offset+4:], b)
	return len(b) + 4, nil
}

// GetString. return string dari byte array page di posisi= offset.
func (p *Page) GetString(offset int32) string {
	return string(p.GetBytes(offset))
}

// putString. set string ke byte array page di posisi = offset.
func (p *Page) PutString(offset int32, s string) {
	p.PutBytes(offset, []byte(s))
}

func (p *Page) Contents() []byte {
	return p.bb.Bytes()
}

// graph related operation
/*

		page structure:
		n = nodesCount+1
		n = total csrN/csrNRev
		m = total csrF/csrFRev

			|nodesCount | fEdgesCount | nodeID | nodeID0 | .... | nodeIDN-1  | csrN[0] | .... | csrN[n] | csrF[0] | csrF[1] | ... | csrF[m] | csrFRev[0] | ... | csrFRev[m] |
				4 byte		4 byte		4 byte									4 byte					 	40 byte				  												40 byte

			|  						header    							 												|	data 	 																														  |


	UPDATE:
	use bitpacking
			(nodeID, isNodeTrafficLight)
	bit:      0-30 			31



	csrN		(csrN, 		csrNRev)
				0-15		16-31
*/

func (pg *Page) WriteBlock(csrN, csrNRev []int, csrF, csrFRev []datastructure.EdgeCH,
	nodeIDs []int32, isTrafficLight []bool) (int, error) {

	// write to page

	// write header first

	nodesCount := int32(len(csrN) - 1)
	pg.PutInt(0, nodesCount)

	// fEdgesCount
	pg.PutInt(4, int32(len(csrF)))

	// nodeNOffset
	offset := int32(8)

	var (
		i int32 = 0
	)
	for ; i < nodesCount; i++ {

		pg.PutInt(offset, util.BitPackIntBool(nodeIDs[i], isTrafficLight[i], 31)) // nodeID & isTrafficLight flag
		offset += 4

	}

	// 8 + 4 * nodesCount

	// csrN & csrNRev
	for i := int32(0); i < int32(len(csrN)); i++ {
		pg.PutInt(offset+i*4, util.BitPackInt(int32(csrN[i]), int32(csrNRev[i]), 16)) // csrN & csrNRev
	}

	// 8 +  4 * nodesCount  + 4 * (nodesCount+1)

	offset += int32(len(csrN) * 4)
	for i := int32(0); i < int32(len(csrF)); i++ {
		edge := csrF[i]

		serializedEdge := edge.Serialize()
		_, err := pg.PutBytes(offset+i*40, serializedEdge) // 36 + 4
		if err != nil {
			return 0, err
		}
	}

	// 8 + 4 * nodesCount  + 4 * (nodesCount+1) + 40 * len(nodeOutEdges)

	offset += int32(len(csrF) * 40)

	// 8 + 4 * nodesCount  + 4 * (nodesCount+1) + 40 * len(nodeOutEdges)
	for i := int32(0); i < int32(len(csrFRev)); i++ {
		edge := csrFRev[i]

		serializedEdge := edge.Serialize()
		_, err := pg.PutBytes(offset+i*40, serializedEdge) // 36 + 4
		if err != nil {
			return 0, err
		}
	}

	return int(offset) + 40*len(csrFRev), nil
}

func (pg *Page) IsNodeTrafficLight(nodeID int32) bool {
	startNOffset := int32(8)
	nodesCount := pg.GetInt(0)

	var (
		isTrafficLight bool
	)

	for i := int32(0); i < nodesCount; i++ {
		currNodeID, traffic := util.BitUnpackIntBool(pg.GetInt(startNOffset+4*i), 31)
		if currNodeID == nodeID {

			isTrafficLight = traffic
			break
		}

	}

	return isTrafficLight
}

// GetNodeOutEdgesCsr. return out edges for nodeID in format ([]toNodeIDs, []EdgeCH)
func (pg *Page) GetNodeOutEdgesCsr(nodeID int32) []datastructure.EdgeCH {
	// in this step. from F[N[a]] to F[N[a+1]-1] inclusive (or  F[ [ N[a], N[a+1] ) ] ) contains the out edges for node a

	startNOffset := int32(8)
	nodesCount := pg.GetInt(0)

	var (
		nodeOffsetInNodeIDs int32 = -1
	)

	for i := int32(0); i < nodesCount; i++ {
		currNodeID, _ := util.BitUnpackIntBool(pg.GetInt(startNOffset+4*i), 31)
		if currNodeID == nodeID {
			nodeOffsetInNodeIDs = i
			break
		}
	}

	headerLength := startNOffset + 4*nodesCount

	nodeIDOffset := headerLength + 4*nodeOffsetInNodeIDs
	nextNodeIDOffset := headerLength + 4*nodeOffsetInNodeIDs + 4

	nodeN, _ := util.BitUnpackInt(pg.GetInt(nodeIDOffset), 16)         // csrN[nodeIDOffset]
	nextNodeN, _ := util.BitUnpackInt(pg.GetInt(nextNodeIDOffset), 16) // csrN[nodeIDOffset+1]

	csrNLength := 4 * (nodesCount + 1)

	edges := make([]datastructure.EdgeCH, 0)

	for i := nodeN; i < nextNodeN; i++ {
		edgesBuf := pg.GetBytes(headerLength + csrNLength + i*40)
		edge := datastructure.DeserializeEdgeCH(edgesBuf)
		edges = append(edges, edge)
	}

	return edges
}

func (pg *Page) GetNodeInEdgesCsr(nodeID int32) []datastructure.EdgeCH {
	// in this step. from FRev[NRev[a]] to FRev[NRev[a+1]-1] inclusive (or  FRev[ [ NRev[a], NRev[a+1] ) ] ) contains the in edges for node a

	startNOffset := int32(8)
	nodesCount := pg.GetInt(0)

	var (
		nodeOffsetInNodeIDs int32 = -1
	)

	for i := int32(0); i < nodesCount; i++ {
		currNodeID, _ := util.BitUnpackIntBool(pg.GetInt(startNOffset+4*i), 31)
		if currNodeID == nodeID {
			nodeOffsetInNodeIDs = i
			break
		}
	}

	headerLength := startNOffset + 4*nodesCount

	nodeIDOffset := headerLength + 4*nodeOffsetInNodeIDs
	nextNodeIDOffset := headerLength + 4*nodeOffsetInNodeIDs + 4

	_, nodeN := util.BitUnpackInt(pg.GetInt(nodeIDOffset), 16)         // csrNRev[nodeIDOffset]
	_, nextNodeN := util.BitUnpackInt(pg.GetInt(nextNodeIDOffset), 16) // // csrNRev[nodeIDOffset+1]

	fEdgesCount := pg.GetInt(4)

	forwardLength := 4*(nodesCount+1) + 40*fEdgesCount // csrN + csrF length

	edges := make([]datastructure.EdgeCH, 0)

	for i := nodeN; i < nextNodeN; i++ {
		edgesBuf := pg.GetBytes(headerLength + forwardLength + i*40)
		edge := datastructure.DeserializeEdgeCH(edgesBuf)
		edges = append(edges, edge)
	}

	return edges
}

/*
blockID for edgeinfoPage is blockID for csrpage+1
all edges in this page is the same as the edges in csr page
edgeinfo page structure:

	 	| edgesCount | edgeID0 | .... | edgeIDN	| edge0InfoSize | edge1InfoSize | ... | edgeNInfoSize  | edge0Info | edge1Info | ..... | edgeNInfo |
			4 byte		8 byte						4 byte									4byte				dynamic size..

		edgeIInfo:
		| StreetName | RoadClass | RoadClassLink | Lanes | Roundabout | IsShourtcut | PointsInBetween |
			4 byte		4 byte			4 byte		4 byte	1 byte		 1 byte			dynamic

		use bitpacking for edgeInfo
		https://wiki.openstreetmap.org/wiki/Map_features

		maxRoadclass is 120 so its 7 bits -> 8 bits
		maxRoadClassLink is 5 so its 3 bits -> 4 bits
		maxLanes is just 8 so its 3 bits -> 4 bits
		roundabout is 1 bit
		shortcut is 1 bit

		bitpackedEdgeInfoField: (RoadClass, RoadClassLink, Lanes, Roundabout, IsShortcut)
									0-7 		8-11 		12-15 	16 		17

		bitpackedEdgeInfo:
		| Streetname | bitpackedEdgeInfoField | PointsInBetween |
			4 byte			4 byte				dynamic


	edgeIDs[i]: (fromNodeID, toNodeID)

	for get edgeInfo for (qFromNodeID, qToNodeID) we need to find the edgeIDOffset first
	edgeIDOffset = linearSearch(edgeIDs, (qFromNodeID, qToNodeID)). first pas is compare fromNodeID == qFromNodeID && toNodeID == qToNodeID
	second pass is compare fromNodeID == qToNodeID && toNodeID == qFromNodeID

	edgeInfoOffset =  edgeInfoSize[edgeIDOffset-1] .if edgeIDOffset == 0 then return 0
	edgeSize =  edgeInfoSize[edgeIDOffset]

	edgeInfoOffset += 12 byte*edgesCount // initial header
	then get the edgeInfo from edgeInfoBuf[edgeInfoOffset:edgeInfoOffset+edgeSize]

	edgeInfoSize store prefix sum size of edgeInfo in edgeInfoBuf
	edgeInfoSize[0] = edgeSize[0]
	edgeInfoSize[i] = edgeSize[i] + edgeInfoSize[i-1]


	edgeInfoBuf store all edgeInfo in this page
*/
func (pg *Page) WriteEdgeInfoBlock(edgeInfo []datastructure.EdgeExtraInfo, edgeRevInfo []datastructure.EdgeExtraInfo,
	edges []datastructure.EdgeCH, revEdges []datastructure.EdgeCH) (int, error) {

	offset := int32(4)

	type edgeWithInfo struct {
		fromNodeID int32
		toNodeID   int32
		reverse    bool
		edgeInfo   datastructure.EdgeExtraInfo
	}

	// first maintain unique map (fromNodeID, toNodeID)
	edgeIDsMap := make(map[int32]map[int32]edgeWithInfo)
	for i := int32(0); i < int32(len(edgeInfo)); i++ {
		edge := edges[i]
		edgeInfo := edgeInfo[i]

		if _, ok := edgeIDsMap[edge.FromNodeID]; !ok {
			edgeIDsMap[edge.FromNodeID] = make(map[int32]edgeWithInfo)
		}
		edgeIDsMap[edge.FromNodeID][edge.ToNodeID] = edgeWithInfo{
			fromNodeID: edge.FromNodeID,
			toNodeID:   edge.ToNodeID,
			reverse:    false,
			edgeInfo:   edgeInfo,
		}
	}

	// reversed edges
	for i := int32(0); i < int32(len(revEdges)); i++ {
		edge := revEdges[i]
		revEdgeInfo := edgeRevInfo[i]

		if _, ok := edgeIDsMap[edge.FromNodeID]; !ok {
			edgeIDsMap[edge.FromNodeID] = make(map[int32]edgeWithInfo)
		}
		edgeIDsMap[edge.FromNodeID][edge.ToNodeID] = edgeWithInfo{
			fromNodeID: edge.FromNodeID,
			toNodeID:   edge.ToNodeID,
			reverse:    true,
			edgeInfo:   revEdgeInfo,
		}
	}

	edgesInfo := make([]edgeWithInfo, 0)
	for _, toMap := range edgeIDsMap {
		for _, edgeInfo := range toMap {
			edgesInfo = append(edgesInfo, edgeInfo)
		}
	}

	pg.PutInt(0, int32(len(edgesInfo)))

	// write edgeIDs (fromNodeID, toNodeID)
	for _, edgeW := range edgesInfo {
		fromNodeID := edgeW.fromNodeID
		toNodeID := edgeW.toNodeID
		pg.PutInt(offset, util.BitPackIntBool(fromNodeID, edgeW.reverse, 31))
		offset += 4
		pg.PutInt(offset, toNodeID)
		offset += 4

	}

	prefixSumBufSize := int32(0)

	for _, edgeW := range edgesInfo {

		serializedEdge := edgeW.edgeInfo.Serialize()

		prefixSumBufSize += int32(len(serializedEdge) + 4) // 4 for storing byte slice size in PutBytes()
		pg.PutInt(offset, prefixSumBufSize)
		offset += 4
	}

	for _, edgeW := range edgesInfo {
		serializedEdge := edgeW.edgeInfo.Serialize()

		byteSliceSize, err := pg.PutBytes(offset, serializedEdge)
		if err != nil {
			return 0, err
		}
		offset += int32(byteSliceSize)
	}

	return int(offset), nil
}

func (pg *Page) GetEdgeInfo(fromNodeID, toNodeID int32, reverse bool) (datastructure.EdgeExtraInfo, error) {
	if fromNodeID == toNodeID {
		return datastructure.EdgeExtraInfo{}, nil
	}
	edgesCount := pg.GetInt(0)

	var (
		offset       int32 = 4
		edgeIDOffset int32 = 0
		edgeInfoBuf  []byte
	)

	for i := int32(0); i < edgesCount; i++ {
		eFromNodeID, _ := util.BitUnpackIntBool(pg.GetInt(offset), 31)
		offset += 4
		eToNodeID := pg.GetInt(offset)
		offset += 4

		if eFromNodeID == fromNodeID && eToNodeID == toNodeID  {
			edgeIDOffset = i
			break
		}
	}

	offset = 4 + 12*edgesCount

	startEdgeInfoSizeOffset := 4 + 8*edgesCount

	if edgeIDOffset == 0 {
		edgeInfoBuf = pg.GetBytes(offset)
	} else {
		// get the offset of edgeInfos[edgeID]
		edgeInfoOffset := pg.GetInt(startEdgeInfoSizeOffset + (edgeIDOffset-1)*4)
		edgeInfoBuf = pg.GetBytes(offset + edgeInfoOffset)
	}

	edgeInfo, err := datastructure.DeserializeEdgeExtraInfo(edgeInfoBuf)

	return edgeInfo, err
}
