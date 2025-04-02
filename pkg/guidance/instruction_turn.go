package guidance

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
)

/*
GetAlternativeTurns. get jumlah belokan alternatif yang bisa dilakukan dari baseNode sekarang & bukan currentEdge/prevEdge. Misalkan:

		 |
		 |
	 alternative
		 |
--prev-- B --currentEdge---
		 |
		 |
	alternative
		 |

ada 4 belokan yang bisa dilakukan dari baseNode B. belokan didapat dari  contractedGraph.GetFirstOutEdge(baseNode)
--- / | = jalan 2 arah
*/ // nolint: gofmt
func (ife *InstructionsFromEdges) GetAlternativeTurns(baseNode, adjNode, prevNode int32) (int, []datastructure.EdgeCH, error) {
	alternativeTurns := []datastructure.EdgeCH{}
	edges, err := ife.contractedGraph.GetNodeOutEdgesCsr2(baseNode)
	if err != nil {
		return 0, alternativeTurns, err
	}
	for _, edge := range edges {
		if edge.ToNodeID != prevNode && edge.ToNodeID != adjNode {
			alternativeTurns = append(alternativeTurns, edge)
		}
	}

	return 1 + len(alternativeTurns), alternativeTurns, nil
}

func (ife *InstructionsFromEdges) isLeavingCurrentStreet(prevStreetName, currentStreetName string, prevEdge, currentEdge datastructure.EdgeCH,
	prevEdgeInfo, currentEdgeInfo datastructure.EdgeExtraInfo) (bool, error) {
	if isSameName(currentStreetName, prevStreetName) {
		// isSameName == false - bisa ketika nama street kosong di osm.
		return false, nil
	}

	if ok := ife.isSameRoadClassAndLink(prevEdge, currentEdge, prevEdgeInfo, currentEdgeInfo); !ok {
		// leave current street jika prevStreetName  != currentStreetName && roadclass/roadclassLink prevEdge,currentEdge beda.
		return true, nil
	}
	return false, nil
}

/*
getOtherEdgeContinueDirection. get alternativeEdges lain dari baseNode yang arahnya continue. Misalkan

				---- currentEdge-----

--prevEdge-- baseNode

				----alternativeEdge-----

delta bearing antara currentEdge dan alternativeEdge mendekati 0°
*/ // nolint: gofmt
func (ife *InstructionsFromEdges) getOtherEdgeContinueDirection(prevLat, prevLon, prevOrientation float64, alternativeTurns []datastructure.EdgeCH) datastructure.EdgeCH {
	var tmpSign int
	for _, edge := range alternativeTurns {

		node := ife.contractedGraph.GetNode(edge.ToNodeID)
		lat, lon := node.Lat, node.Lon

		tmpSign = getTurnDirection(prevLat, prevLon, lat, lon, prevOrientation)
		if math.Abs(float64(tmpSign)) <= 1 {
			return edge
		}
	}
	return datastructure.EdgeCH{}
}

/*
	isStreetMerged. cek apakah 2 street (prevEdge, otherEdge) merged ke 1 street. Misal kalau jalan di indo:
	--prevEdge-->
					baseNode <--currentEdge-->
	<--otherEdge--

examplenya di jalan solo-semarang, A.Yani : -7.5533505900708455, 110.82338424980728
*/ // nolint: gofmt
func (ife *InstructionsFromEdges) isStreetMerged(currentEdge, prevEdge datastructure.EdgeCH,
	currentEdgeInfo, prevEdgeInfo datastructure.EdgeExtraInfo) (bool, error) {
	baseNode := currentEdge.FromNodeID

	name := ife.contractedGraph.GetStreetNameFromID(currentEdgeInfo.StreetName)
	roadClass := currentEdgeInfo.RoadClass
	if roadClass != prevEdgeInfo.RoadClass {
		return false, nil
	}

	var otherEdge *datastructure.EdgeCH = nil // inEdge dari BaseNode selain PrevEdge yang mengarah ke BaseNode
	outEdges, err := ife.contractedGraph.GetNodeOutEdgesCsr2(baseNode)
	if err != nil {
		return false, err
	}

	for _, edge := range outEdges {

		edgeStreetName := currentEdgeInfo.StreetName

		edgeRoadClass := currentEdgeInfo.RoadClass

		if edge.EdgeID != currentEdge.EdgeID && edge.EdgeID != prevEdge.EdgeID &&
			edge.ToNodeID != currentEdge.ToNodeID && edge.ToNodeID != prevEdge.FromNodeID &&
			roadClass == edgeRoadClass &&
			isSameName(name, ife.contractedGraph.GetStreetNameFromID(edgeStreetName)) {
			if otherEdge != nil {
				return false, nil
			}
			otherEdge = &edge
		}
	}

	if otherEdge == nil {
		return false, nil
	}
	otherEdgeInfo, err := ife.GetEdgeInfo(otherEdge.FromNodeID, otherEdge.ToNodeID)
	if err != nil {
		return false, err
	}
	currentEdgeDirection := ife.contractedGraph.GetStreetDirection(currentEdgeInfo.StreetName) // [0] forward, [1] reversed
	if currentEdgeDirection[1] {

		prevEdgeDirection := ife.contractedGraph.GetStreetDirection(prevEdgeInfo.StreetName)
		if prevEdgeDirection[1] {
			// prevEdge harus tidak punya reversed direction
			return false, nil
		}

		otherEdgeStreetName := otherEdgeInfo.StreetName

		otherEdgeLanes := otherEdgeInfo.Lanes

		otherEdgeDirection := ife.contractedGraph.GetStreetDirection(otherEdgeStreetName)
		if !otherEdgeDirection[0] || otherEdgeDirection[1] {
			// otherEdge harus forward edge & bukan reversed edge
			return false, nil
		}

		laneDiff := prevEdgeInfo.Lanes + otherEdgeLanes - currentEdgeInfo.Lanes // setidaknya lane dari currentEdge 2. lane dari other & prev 1
		return laneDiff <= 1, nil
	}
	return false, nil
}

/*
	isStreetSplit.	cek apakah edge sekarang hasil dari split edge sebelumnya.

	 							--currentEdge-->
	 	<--prevEdge-->baseNode
							   	<--otherEdge--
		examplenya di -7.559777239220366, 110.83649946865347
*/ // nolint: gofmt
func (ife *InstructionsFromEdges) isStreetSplit(currentEdge, prevEdge datastructure.EdgeCH,
	currentEdgeInfo, prevEdgeInfo datastructure.EdgeExtraInfo) (bool, error) {
	baseNode := currentEdge.FromNodeID

	name := ife.contractedGraph.GetStreetNameFromID(currentEdgeInfo.StreetName)
	roadClass := currentEdgeInfo.RoadClass
	if !isSameName(name, ife.contractedGraph.GetStreetNameFromID(prevEdgeInfo.StreetName)) || roadClass != prevEdgeInfo.RoadClass {
		return false, nil
	}

	var otherEdge *datastructure.EdgeCH = nil // inEdge dari BaseNode selain PrevEdge yang mengarah ke BaseNode
	outEdges, err := ife.contractedGraph.GetNodeInEdgesCsr2(baseNode)
	if err != nil {
		return false, err
	}

	for _, edge := range outEdges {

		edgeStreetName := currentEdgeInfo.StreetName
		edgeRoadClass := currentEdgeInfo.RoadClass

		if edge.EdgeID != currentEdge.EdgeID && edge.EdgeID != prevEdge.EdgeID &&
			edge.ToNodeID != currentEdge.ToNodeID && edge.ToNodeID != prevEdge.FromNodeID &&
			roadClass == edgeRoadClass &&
			isSameName(name, ife.contractedGraph.GetStreetNameFromID(edgeStreetName)) {
			if otherEdge != nil {
				return false, nil
			}
			otherEdge = &edge
		}
	}
	if otherEdge == nil {
		return false, nil
	}

	otherEdgeInfo, err := ife.GetEdgeInfo(otherEdge.FromNodeID, otherEdge.ToNodeID)
	if err != nil {
		return false, err
	}
	otherEdgeStretName := otherEdgeInfo.StreetName
	otherEdgeLanes := otherEdgeInfo.Lanes

	prevEdgeDirection := ife.contractedGraph.GetStreetDirection(prevEdgeInfo.StreetName) // [0] forward, [1] reversed
	if !prevEdgeDirection[1] {
		// jika prevEdge bukan reversed
		return false, nil
	}
	otherEdgeDirection := ife.contractedGraph.GetStreetDirection(otherEdgeStretName)
	if !otherEdgeDirection[1] || otherEdgeDirection[0] {
		// jika otherEdge tidak reversed atau arahnya forward
		return false, nil
	}

	laneDiff := prevEdgeInfo.Lanes - (otherEdgeLanes + currentEdgeInfo.Lanes) // setidak nya lane dari PrevEdge 2. lane dari  otherEdge  & currentEdge cuma 1
	return laneDiff <= 1, nil
}

func (ife *InstructionsFromEdges) isSameRoadClassAndLink(edge1, edge2 datastructure.EdgeCH,
	edge1Info, edge2Info datastructure.EdgeExtraInfo) bool {

	return edge1Info.RoadClass == edge2Info.RoadClass && edge1Info.RoadClassLink == edge2Info.RoadClassLink
}

func abs(a int) int {
	if a < 0 {
		return -a
	}
	return a
}

const (
	DEGREE_TO_RADIANS = 0.017453292519943295
)

func toRadians(degrees float64) float64 {
	return degrees * DEGREE_TO_RADIANS
}

func alignOrientation(baseOrientation, orientation float64) float64 {
	var resultOrientation float64
	if baseOrientation >= 0 {
		if orientation < -math.Pi+baseOrientation {
			resultOrientation = orientation + 2*math.Pi
		} else {
			resultOrientation = orientation
		}
	} else if orientation > math.Pi+baseOrientation {
		resultOrientation = orientation - 2*math.Pi
	} else {
		resultOrientation = orientation
	}
	return resultOrientation
}

func isSameName(name1, name2 string) bool {
	if name1 == "" || name2 == "" {
		// seringkali di osm, nama street kosong "", better dianggap false
		return false
	}
	return name1 == name2
}

func calcOrientation(lat1, lon1, lat2, lon2 float64) float64 {
	prevBearing := BearingTo(lat1, lon1, lat2, lon2)
	prevBearing = toRadians(prevBearing)
	return prevBearing
}

func calculateOrientationDelta(prevLatitude, prevLongitude, latitude, longitude, prevOrientation float64) float64 {
	orientation := calcOrientation(prevLatitude, prevLongitude, latitude, longitude)
	orientation = alignOrientation(prevOrientation, orientation)
	return orientation - prevOrientation
}

func getTurnDirection(prevLatitude, prevLongitude, latitude, longitude, prevOrientation float64) int {
	delta := calculateOrientationDelta(prevLatitude, prevLongitude, latitude, longitude, prevOrientation)
	absDelta := math.Abs(delta)
	deltaDegree := absDelta * (180 / math.Pi)
	if deltaDegree < 12 {
		// 12°
		return CONTINUE_ON_STREET
	} else if deltaDegree < 40 {

		if delta < 0 {
			return (TURN_SLIGHT_LEFT)
		} else {
			return (TURN_SLIGHT_RIGHT)
		}
	} else if deltaDegree < 105 {

		if delta < 0 {
			return (TURN_LEFT)
		} else {
			return (TURN_RIGHT)
		}

	} else if delta < 0 {
		return (TURN_SHARP_LEFT)

	} else {
		return (TURN_SHARP_RIGHT)

	}
}
