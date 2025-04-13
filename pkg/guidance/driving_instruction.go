package guidance

import (
	"errors"
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
)

type ContractedGraph interface {
	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32

	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.Edge
	GetInEdge(edgeID int32) datastructure.Edge
	GetStreetDirection(streetName int) [2]bool
	GetStreetNameFromID(streetName int) string
	GetRoadClassFromID(roadClass uint8) string
	GetRoadClassLinkFromID(roadClassLink uint8) string

	IsRoundabout(edgeID int32) bool
	IsShortcut(edgeID int32) bool

	GetEdgeInfo(edgeID int32) datastructure.EdgeExtraInfo
}

type InstructionsFromEdges struct {
	contractedGraph       ContractedGraph
	ways                  []*datastructure.Instruction
	prevEdge              datastructure.Edge
	prevEdgeInfo          datastructure.EdgeExtraInfo
	edgeInfo              datastructure.EdgeExtraInfo
	prevNode              int32   // previous Node (base node prevEdge)
	prevOrientation       float64 // orientasi/selisih bearing antara prevEdge dengan currentEdge
	doublePrevOrientation float64 // orientasi/selisih bearing antara prevEdge dengan prevPrevEdge
	prevInstruction       *datastructure.Instruction
	prevInRoundabout      bool   // apakah sebelumnya di roundabout (bundaran)
	doublePrevStreetName  string // streetname prevPrevEdge
	doublePrevNode        int32
}

func NewInstructionsFromEdges(contractedGraph ContractedGraph) *InstructionsFromEdges {
	return &InstructionsFromEdges{
		contractedGraph:       contractedGraph,
		ways:                  make([]*datastructure.Instruction, 0),
		prevNode:              -1,
		prevInRoundabout:      false,
		doublePrevOrientation: 0,
	}
}

func GetTurnDescriptions(instructions []*datastructure.Instruction) ([]string, error) {
	var turnDescriptions []string
	for _, instr := range instructions {
		desc := instr.GetTurnDescription()
		turnDescriptions = append(turnDescriptions, desc)
	}
	return turnDescriptions, nil
}

func (ife *InstructionsFromEdges) GetDrivingDirections(path []datastructure.Edge) ([]datastructure.DrivingDirection, error) {
	drivingInstructions := make([]datastructure.DrivingDirection, 0)
	if len(path) == 0 {

		return drivingInstructions, errors.New("path is empty")
	}

	for _, edge := range path {
		prevEdgeInfo := ife.GetEdgeInfo(ife.prevEdge.EdgeID)

		ife.prevEdgeInfo = prevEdgeInfo
		edgeInfo := ife.GetEdgeInfo(edge.EdgeID)

		ife.edgeInfo = edgeInfo
		ife.AddInstructionFromEdge(edge)
	}
	ife.Finish()
	drivingInstructionsDesc, err := GetTurnDescriptions(ife.ways)
	if err != nil {
		return nil, err
	}

	prevETA := 0.0
	prevDist := 0.0
	for i, _ := range ife.ways {
		drivingInstructions = append(drivingInstructions, datastructure.NewDrivingDirection(*ife.ways[i], drivingInstructionsDesc[i], prevETA, prevDist))
		prevETA = ife.ways[i].Time
		prevDist = ife.ways[i].Distance
	}
	return drivingInstructions, nil
}

func (ife *InstructionsFromEdges) AddInstructionFromEdge(edge datastructure.Edge) error {
	adjNode := edge.ToNodeID
	baseNode := edge.FromNodeID

	baseNodeData := ife.contractedGraph.GetNode(baseNode)

	adjNodeData := ife.contractedGraph.GetNode(adjNode)
	adjLat := adjNodeData.Lat
	adjLon := adjNodeData.Lon
	var latitude, longitude float64

	isRoundabout := ife.contractedGraph.IsRoundabout(edge.EdgeID)

	latitude = adjLat
	longitude = adjLon
	var prevNodeData datastructure.CHNode
	if ife.prevNode != -1 {
		prevNodeData = ife.contractedGraph.GetNode(ife.prevNode)
	}

	prevEdgeExtraInfo := ife.prevEdgeInfo

	edgeExtraInfo := ife.edgeInfo

	name := ife.contractedGraph.GetStreetNameFromID(edgeExtraInfo.StreetName)

	if ife.prevInstruction == nil && !isRoundabout {
		// start point dari shortetest path & bukan bundaran (roundabout)
		sign := datastructure.START
		point := datastructure.NewCoordinate(baseNodeData.Lat, baseNodeData.Lon)
		newIns := datastructure.NewInstruction(sign, name, point, false)
		ife.prevInstruction = &newIns

		baseEdgeNode := ife.contractedGraph.GetNode(baseNode)
		startLat := baseEdgeNode.Lat
		startLon := baseEdgeNode.Lon
		heading := BearingTo(startLat, startLon, latitude, longitude)
		ife.prevInstruction.ExtraInfo["heading"] = heading // bearing dari titik awal ke edge.ToNodeID (arah edge)
		ife.ways = append(ife.ways, ife.prevInstruction)

	} else if isRoundabout {
		// current edge bundaran
		if !ife.prevInRoundabout {
			sign := datastructure.USE_ROUNDABOUT
			point := datastructure.NewCoordinate(baseNodeData.Lat, baseNodeData.Lon)
			roundaboutInstruction := datastructure.NewRoundaboutInstruction()
			ife.doublePrevOrientation = ife.prevOrientation
			if ife.prevInstruction != nil {

				ife.prevOrientation = calcOrientation(prevNodeData.Lat, prevNodeData.Lon, baseNodeData.Lat, baseNodeData.Lon)

			} else {
				//start point dari shortetest path & dan bundaran (roundabout)
				ife.prevOrientation = calcOrientation(baseNodeData.Lat, baseNodeData.Lon, latitude, longitude)
			}

			prevIns := datastructure.NewInstructionWithRoundabout(sign, name, point, true, roundaboutInstruction)
			ife.prevInstruction = &prevIns

			ife.ways = append(ife.ways, ife.prevInstruction)
		}

		outgoingEdges := ife.contractedGraph.GetNodeFirstOutEdges(adjNode)

		for _, e := range outgoingEdges {
			if ife.contractedGraph.IsShortcut(e) {
				continue
			}
			eIsRoundabout := ife.contractedGraph.IsRoundabout(e)

			if !eIsRoundabout {
				// add jumlah exit Point dari bundaran
				// exit point didapat dari outgoing edges (yang bukan roundabout) dari node yang terhubung di osm way roundabout
				// example: https://www.google.com/maps/dir/'-7.75596,110.37666'/-7.7674052,110.3747602/@-7.7645774,110.3751945,18.42z/am=t/data=!4m7!4m6!1m3!2m2!1d110.37666!2d-7.75596!1m0!3e0?hl=id&entry=ttu&g_ep=EgoyMDI1MDQwMi4xIKXMDSoASAFQAw%3D%3D
				roundaboutInstruction := ife.prevInstruction
				roundaboutInstruction.Roundabout.ExitNumber++
				break
			}
		}

	} else if ife.prevInRoundabout {

		ife.prevInstruction.Name = name
		roundaboutInstruction := ife.prevInstruction
		roundaboutInstruction.Roundabout.Exited = true

		ife.doublePrevStreetName = ife.contractedGraph.GetStreetNameFromID(prevEdgeExtraInfo.StreetName)

	} else {
		sign, err := ife.GetTurnSign(edge, baseNode, ife.prevNode, adjNode, name)
		if err != nil {
			return err
		}
		if sign != datastructure.IGNORE {
			isUTurn, uTurnType := ife.CheckUTurn(sign, name, edge) // check apakah U-TURN, kalau iya, prevInstruction sebelumnya (sign == RIGHTTURN) ganti ke U-Turn
			if isUTurn {
				ife.prevInstruction.Sign = uTurnType
				ife.prevInstruction.Name = name
			} else {
				// bukan U-turn -> continue/right/left
				point := datastructure.NewCoordinate(baseNodeData.Lat, baseNodeData.Lon)
				prevIns := datastructure.NewInstruction(sign, name, point, false)
				ife.prevInstruction = &prevIns
				ife.doublePrevOrientation = ife.prevOrientation
				ife.doublePrevStreetName = ife.contractedGraph.GetStreetNameFromID(prevEdgeExtraInfo.StreetName)
				ife.ways = append(ife.ways, ife.prevInstruction)
			}
		} else {

			ife.prevInstruction.Distance += edge.Dist
			if edge.Weight == 0 {
				ife.prevInstruction.Time += 0
			} else {
				currEdgeSpeed := (edge.Dist / edge.Weight)
				ife.prevInstruction.Time += edge.Dist / currEdgeSpeed
			}
		}
	}

	ife.doublePrevNode = ife.prevNode
	ife.prevInRoundabout = isRoundabout
	ife.prevNode = baseNode
	ife.prevEdge = edge

	return nil
}

func (ife *InstructionsFromEdges) GetEdgeInfo(edgeID int32) datastructure.EdgeExtraInfo {

	return ife.contractedGraph.GetEdgeInfo(edgeID)
}

/*
CheckUTurn. check jika current edge adalah U-turn. Misalkan:

A --doublePrevEdge-->B
				    |
					|
				PrevEdge
					|
					|
					|
D <--currentEdge---C

jika dari A->B belok kanan, dan dari B->C belok kanan, dan delta bearing antara A->B dan C->D mendekati 180 derajat, maka bisa dianggap U-turn
*/ // nolint: gofmt
func (ife *InstructionsFromEdges) CheckUTurn(sign int, name string, edge datastructure.Edge) (bool, int) {
	isUTurn := false
	uTurnType := datastructure.U_TURN_UNKNOWN

	if ife.doublePrevOrientation != 0 && (sign > 0) == (ife.prevInstruction.Sign > 0) &&
		(abs(sign) == datastructure.TURN_SLIGHT_RIGHT || abs(sign) == datastructure.TURN_RIGHT || abs(sign) == datastructure.TURN_SHARP_RIGHT) &&
		(abs(ife.prevInstruction.Sign) == datastructure.TURN_SLIGHT_RIGHT || abs(ife.prevInstruction.Sign) == datastructure.TURN_RIGHT || abs(ife.prevInstruction.Sign) == datastructure.TURN_SHARP_RIGHT) &&
		isSameName(ife.doublePrevStreetName, name) {
		node := ife.contractedGraph.GetNode(edge.ToNodeID)
		pointLat, pointLon := node.Lat, node.Lon
		baseNodeData := ife.contractedGraph.GetNode(edge.FromNodeID)
		currentOrientation := calcOrientation(baseNodeData.Lat, baseNodeData.Lon, pointLat, pointLon)
		diff := math.Abs(ife.doublePrevOrientation - currentOrientation)
		diffAngle := diff * (180 / math.Pi)
		if diffAngle > 155 && diffAngle < 205 {
			isUTurn = true
			if sign < 0 {
				uTurnType = datastructure.U_TURN_LEFT
			} else {
				uTurnType = datastructure.U_TURN_RIGHT
			}
		}
	}
	return isUTurn, uTurnType
}

/*
Finish. tambah final instruction.
*/
func (ife *InstructionsFromEdges) Finish() error {

	doublePrevNode := ife.contractedGraph.GetNode(ife.doublePrevNode)

	baseNodeData := ife.contractedGraph.GetNode(ife.prevEdge.FromNodeID)

	prevEdgeExtraInfo := ife.GetEdgeInfo(ife.prevEdge.EdgeID)

	node := ife.contractedGraph.GetNode(ife.prevEdge.ToNodeID)
	point := datastructure.NewCoordinate(node.Lat, node.Lon)
	finishInstruction := datastructure.NewInstruction(datastructure.FINISH, ife.contractedGraph.GetStreetNameFromID(prevEdgeExtraInfo.StreetName), point, false)
	finishInstruction.ExtraInfo["heading"] = BearingTo(doublePrevNode.Lat, doublePrevNode.Lon, baseNodeData.Lat, baseNodeData.Lon)
	newIns := datastructure.NewInstruction(finishInstruction.Sign, finishInstruction.Name, finishInstruction.Point, false)
	ife.ways = append(ife.ways, &newIns)

	return nil
}

/*
GetTurnSign. Medapatkan turn sign dari setiap 2 edge bersebelahan pada shortest path berdasarkan selisih bearing. Misalkan:

prevNode----prevEdge----BaseNode
							|
							|
						currentEdge
							|
							|
						AdjNode

*/ // nolint: gofmt
func (ife *InstructionsFromEdges) GetTurnSign(edge datastructure.Edge, baseNode, prevNode, adjNode int32, name string) (int, error) {
	baseNodeData := ife.contractedGraph.GetNode(edge.FromNodeID)
	point := ife.contractedGraph.GetNode(edge.ToNodeID)
	lat := point.Lat
	lon := point.Lon
	var prevNodeData datastructure.CHNode
	if ife.prevNode != -1 {
		prevNodeData = ife.contractedGraph.GetNode(ife.prevNode)
	}

	ife.prevOrientation = calcOrientation(prevNodeData.Lat, prevNodeData.Lon, baseNodeData.Lat, baseNodeData.Lon)
	sign := getTurnDirection(baseNodeData.Lat, baseNodeData.Lon, lat, lon, ife.prevOrientation)

	alternativeTurnsCount, alternativeTurns, err := ife.GetAlternativeTurns(baseNode, adjNode, prevNode)
	if err != nil {
		return 0, err
	}
	isStreetSplit, err := ife.isStreetSplit(edge, ife.prevEdge, ife.edgeInfo, ife.prevEdgeInfo)
	if err != nil {
		return 0, err
	}
	isStreetMerged, err := ife.isStreetMerged(edge, ife.prevEdge, ife.edgeInfo, ife.prevEdgeInfo)
	if err != nil {
		return 0, err
	}

	if alternativeTurnsCount == 1 {
		if math.Abs(float64(sign)) > 1 && !(isStreetMerged || isStreetSplit) {

			return sign, nil
		}
		return datastructure.IGNORE, nil
	}

	prevEdgeExtraInfo := ife.prevEdgeInfo

	edgeExtraInfo := ife.edgeInfo

	prevEdgeStreetName := ife.contractedGraph.GetStreetNameFromID(prevEdgeExtraInfo.StreetName)
	if math.Abs(float64(sign)) > 1 {
		if (isSameName(name, prevEdgeStreetName)) ||
			isStreetMerged || isStreetSplit {
			return datastructure.IGNORE, nil
		}
		return sign, nil
	}

	if ife.prevEdge.Weight == 0 {
		return sign, nil
	}

	// get edge lain dari baseNode yang arahnya continue
	otherContinueEdge := ife.getOtherEdgeContinueDirection(baseNodeData.Lat, baseNodeData.Lon, ife.prevOrientation, alternativeTurns) //

	otherContinueEdgeExtraInfo := ife.GetEdgeInfo(otherContinueEdge.EdgeID)

	prevCurrEdgeOrientationDiff := calculateOrientationDelta(baseNodeData.Lat, baseNodeData.Lon, lat, lon, ife.prevOrientation) // bearing difference antara prevNode->baseNode->edge.ToNodeID/adjNode
	if otherContinueEdge.Weight != 0 {
		// terdapat edge lain (yang terhubung dengan baseNode) yang arahnya sama continue.
		if !isSameName(name, prevEdgeStreetName) {
			// current Street Name != prevEdge Street Name
			roadClass := ife.contractedGraph.GetRoadClassFromID(edgeExtraInfo.RoadClass)
			prevRoadClass := ife.contractedGraph.GetRoadClassFromID(prevEdgeExtraInfo.RoadClass)
			otherRoadClass := ife.contractedGraph.GetRoadClassFromID(otherContinueEdgeExtraInfo.RoadClass)

			link := ife.contractedGraph.GetRoadClassLinkFromID(edgeExtraInfo.RoadClassLink)
			prevLink := ife.contractedGraph.GetRoadClassLinkFromID(prevEdgeExtraInfo.RoadClassLink)
			otherLink := ife.contractedGraph.GetRoadClassLinkFromID(otherContinueEdgeExtraInfo.RoadClassLink)

			node := ife.contractedGraph.GetNode(otherContinueEdge.ToNodeID)
			tmpLat, tmpLon := node.Lat, node.Lon

			if isMajorRoad(roadClass) {
				if (roadClass == prevRoadClass && link == prevLink) && (otherRoadClass != prevRoadClass || otherLink != prevLink) {
					// current road class == major road class && prevRoadClass sama dg current edge roadClass
					return datastructure.IGNORE, nil
				}
			}

			prevOtherEdgeOrientation := calculateOrientationDelta(baseNodeData.Lat, baseNodeData.Lon, tmpLat, tmpLon, ife.prevOrientation) // bearing difference antara prevNode->baseNode->otherContinueEdge.ToNodeID

			if math.Abs(prevCurrEdgeOrientationDiff)*(180/math.Pi) < 6 && math.Abs(prevOtherEdgeOrientation)*(180/math.Pi) > 8.6 && isSameName(name, prevEdgeStreetName) {
				// bearing difference antara prevEdge dan currentEDge < 6° (CONTINUE Direction), Edge otherContinueEdge > 8.6 (TURN SLIGHT or more direction). Nama prevEdge street == current Street
				return datastructure.CONTINUE_ON_STREET, nil
			}

			if roadClass == "residential" || prevRoadClass == "residential" || (roadClass == "unclassified" && prevRoadClass == "unclassified") {
				// skip roadclass residential untuk mengurangi instructions.
				return datastructure.IGNORE, nil
			}

			/*
				jika dari baseNode ada 2 jalan yang arahnya sama sama lurus/sedikit belok, tambah turn instruction ke currEdge. Misalkan:

						-----currentEdge---------
				baseNode
						-----otherContinueEdge---
			*/ // nolint: gofmt
			if prevCurrEdgeOrientationDiff > prevOtherEdgeOrientation {
				return datastructure.KEEP_RIGHT, nil
			} else {
				return datastructure.KEEP_LEFT, nil
			}
		}
	}

	if ok, err := ife.isLeavingCurrentStreet(prevEdgeStreetName, name, ife.prevEdge, edge, prevEdgeExtraInfo, edgeExtraInfo); !(isStreetMerged || isStreetSplit) &&
		(math.Abs(prevCurrEdgeOrientationDiff)*(180/math.Pi) > 34 || ok) {
		return sign, err
	} else if err != nil {
		return datastructure.IGNORE, err
	}

	return datastructure.IGNORE, nil
}

func isMajorRoad(roadClass string) bool {
	return roadClass == "motorway" || roadClass == "trunk" || roadClass == "primary" || roadClass == "secondary" || roadClass == "tertiary"
}
