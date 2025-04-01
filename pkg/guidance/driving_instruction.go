package guidance

import (
	"errors"
	"log"
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
)

type ContractedGraph interface {
	GetNodeOutEdgesCsr2(nodeID int32) ([]datastructure.EdgeCH, error)
	GetNodeInEdgesCsr2(nodeID int32) ([]datastructure.EdgeCH, error)
	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.EdgeCH
	GetInEdge(edgeID int32) datastructure.EdgeCH
	GetStreetDirection(streetName int) [2]bool
	GetStreetNameFromID(streetName int) string
	GetRoadClassFromID(roadClass int) string
	GetRoadClassLinkFromID(roadClassLink int) string

	IsShortcutCsr(nodeID int32, edgeID int32, reverse bool) (bool, error)
	IsRoundaboutCsr(nodeID int32, edgeID int32) (bool, error)
	GetEdgePointsInBetweenCsr(nodeID int32, edgeID int32, reverse bool) ([]datastructure.Coordinate, error)
	GetStreetNameCsr(nodeID int32, edgeID int32) (int, error)
	GetRoadClassCsr(nodeID int32, edgeID int32) (int, error)
	GetRoadClassLinkCsr(nodeID int32, edgeID int32) (int, error)
	GetLanesCsr(nodeID int32, edgeID int32) (int, error)
}

type InstructionsFromEdges struct {
	ContractedGraph       ContractedGraph
	Ways                  []*Instruction
	PrevEdge              datastructure.EdgeCH
	PrevNode              int32   // previous Node (base node prevEdge)
	PrevOrientation       float64 // orientasi/selisih bearing antara prevEdge dengan currentEdge
	doublePrevOrientation float64 // orientasi/selisih bearing antara prevEdge dengan prevPrevEdge
	PrevInstruction       *Instruction
	PrevInRoundabout      bool   // apakah sebelumnya di roundabout (bundaran)
	DoublePrevStreetName  string // streetname prevPrevEdge
	DoublePrevNode        int32
}

func NewInstructionsFromEdges(contractedGraph ContractedGraph) *InstructionsFromEdges {
	return &InstructionsFromEdges{
		ContractedGraph:       contractedGraph,
		Ways:                  make([]*Instruction, 0),
		PrevNode:              -1,
		PrevInRoundabout:      false,
		doublePrevOrientation: 0,
	}
}

func GetTurnDescriptions(instructions []*Instruction) ([]string, error) {
	var turnDescriptions []string
	for _, instr := range instructions {
		desc := instr.GetTurnDescription()
		turnDescriptions = append(turnDescriptions, desc)
	}
	return turnDescriptions, nil
}

type DrivingInstruction struct {
	Instruction string
	Point       datastructure.Coordinate
	StreetName  string
	ETA         float64
	Distance    float64
}

func NewDrivingInstruction(ins Instruction, description string, prevETA, prevDist float64) DrivingInstruction {
	return DrivingInstruction{
		Instruction: description,
		Point:       ins.Point,
		StreetName:  ins.Name,
		ETA:         Round(prevETA, 2),
		Distance:    Round(prevDist, 2),
	}
}

func (ife *InstructionsFromEdges) GetDrivingInstructions(path []datastructure.EdgeCH) ([]DrivingInstruction, error) {
	drivingInstructions := make([]DrivingInstruction, 0)
	if len(path) == 0 {

		return drivingInstructions, errors.New("path is empty")
	}

	for _, edge := range path {
		ife.AddInstructionFromEdge(edge)
	}
	ife.Finish()
	drivingInstructionsDesc, err := GetTurnDescriptions(ife.Ways)
	if err != nil {
		return nil, err
	}

	prevETA := 0.0
	prevDist := 0.0
	for i, _ := range ife.Ways {
		drivingInstructions = append(drivingInstructions, NewDrivingInstruction(*ife.Ways[i], drivingInstructionsDesc[i], prevETA, prevDist))
		prevETA = ife.Ways[i].Time
		prevDist = ife.Ways[i].Distance
	}
	return drivingInstructions, nil
}

func (ife *InstructionsFromEdges) AddInstructionFromEdge(edge datastructure.EdgeCH) error {
	adjNode := edge.ToNodeID
	baseNode := edge.FromNodeID

	baseNodeData := ife.ContractedGraph.GetNode(baseNode)

	adjNodeData := ife.ContractedGraph.GetNode(adjNode)
	adjLat := adjNodeData.Lat
	adjLon := adjNodeData.Lon
	var latitude, longitude float64

	isRoundabout, err := ife.ContractedGraph.IsRoundaboutCsr(edge.FromNodeID, edge.ToNodeID)
	if err != nil {
		return err
	}
	latitude = adjLat
	longitude = adjLon
	var prevNodeData datastructure.CHNode
	if ife.PrevNode != -1 {
		prevNodeData = ife.ContractedGraph.GetNode(ife.PrevNode)
	}

	prevEdgeExtraInfo, err := ife.GetEdgeInfo(ife.PrevEdge.FromNodeID, ife.PrevEdge.ToNodeID)
	if err != nil {
		return err
	}

	edgeExtraInfo, err := ife.GetEdgeInfo(edge.FromNodeID, edge.ToNodeID)
	if err != nil {
		return err
	}

	name := ife.ContractedGraph.GetStreetNameFromID(edgeExtraInfo.StreetName)

	if ife.PrevInstruction == nil && !isRoundabout {
		// start point dari shortetest path & bukan bundaran (roundabout)
		sign := START
		point := datastructure.NewCoordinate(baseNodeData.Lat, baseNodeData.Lon)
		newIns := NewInstruction(sign, name, point, false)
		ife.PrevInstruction = &newIns

		baseEdgeNode := ife.ContractedGraph.GetNode(baseNode)
		startLat := baseEdgeNode.Lat
		startLon := baseEdgeNode.Lon
		heading := BearingTo(startLat, startLon, latitude, longitude)
		ife.PrevInstruction.ExtraInfo["heading"] = heading // bearing dari titik awal ke edge.ToNodeID (arah edge)
		ife.Ways = append(ife.Ways, ife.PrevInstruction)

	} else if isRoundabout {
		// current edge bundaran
		if !ife.PrevInRoundabout {
			sign := USE_ROUNDABOUT
			point := datastructure.NewCoordinate(baseNodeData.Lat, baseNodeData.Lon)
			roundaboutInstruction := NewRoundaboutInstruction()
			ife.doublePrevOrientation = ife.PrevOrientation
			if ife.PrevInstruction != nil {

				outEdges, err := ife.ContractedGraph.GetNodeOutEdgesCsr2(baseNode)
				if err != nil {
					return err
				}
				for _, e := range outEdges {
					// add jumlah exit Point dari bundaran
					eIsRoundabout, err := ife.ContractedGraph.IsRoundaboutCsr(e.FromNodeID, e.ToNodeID)

					if err != nil {
						return err
					}

					if (e.ToNodeID != ife.PrevNode) && !eIsRoundabout {
						roundaboutInstruction.ExitNumber++
						break
					}
				}

				ife.PrevOrientation = calcOrientation(prevNodeData.Lat, prevNodeData.Lon, baseNodeData.Lat, baseNodeData.Lon)

			} else {
				//start point dari shortetest path & dan bundaran (roundabout)
				ife.PrevOrientation = calcOrientation(baseNodeData.Lat, baseNodeData.Lon, latitude, longitude)
			}

			prevIns := NewInstructionWithRoundabout(sign, name, point, true, roundaboutInstruction)
			ife.PrevInstruction = &prevIns

			ife.Ways = append(ife.Ways, ife.PrevInstruction)
		}

		outgoingEdges, err := ife.ContractedGraph.GetNodeInEdgesCsr2(adjNode)
		if err != nil {
			return err
		}
		for _, e := range outgoingEdges {
			eIsRoundabout, err := ife.ContractedGraph.IsRoundaboutCsr(e.FromNodeID, e.ToNodeID)
			if err != nil {
				return err
			}
			if !eIsRoundabout {
				// add jumlah exit Point dari bundaran
				roundaboutInstruction := ife.PrevInstruction
				roundaboutInstruction.Roundabout.ExitNumber++
				break
			}
		}

	} else if ife.PrevInRoundabout {

		ife.PrevInstruction.Name = name
		roundaboutInstruction := ife.PrevInstruction
		roundaboutInstruction.Roundabout.Exited = true

		ife.DoublePrevStreetName = ife.ContractedGraph.GetStreetNameFromID(prevEdgeExtraInfo.StreetName)

	} else {
		sign, err := ife.GetTurnSign(edge, baseNode, ife.PrevNode, adjNode, name)
		if err != nil {
			return err
		}
		if sign != IGNORE {
			isUTurn, uTurnType := ife.CheckUTurn(sign, name, edge) // check apakah U-TURN, kalau iya, prevInstruction sebelumnya (sign == RIGHTTURN) ganti ke U-Turn
			if isUTurn {
				ife.PrevInstruction.Sign = uTurnType
				ife.PrevInstruction.Name = name
			} else {
				// bukan U-turn -> continue/right/left
				point := datastructure.NewCoordinate(baseNodeData.Lat, baseNodeData.Lon)
				prevIns := NewInstruction(sign, name, point, false)
				ife.PrevInstruction = &prevIns
				ife.doublePrevOrientation = ife.PrevOrientation
				ife.DoublePrevStreetName = ife.ContractedGraph.GetStreetNameFromID(prevEdgeExtraInfo.StreetName)
				ife.Ways = append(ife.Ways, ife.PrevInstruction)
			}
		} else {

			ife.PrevInstruction.Distance += edge.Dist
			if edge.Weight == 0 {
				ife.PrevInstruction.Time += 0
			} else {
				currEdgeSpeed := (edge.Dist / edge.Weight)
				ife.PrevInstruction.Time += edge.Dist / currEdgeSpeed
			}
		}
	}

	ife.DoublePrevNode = ife.PrevNode
	ife.PrevInRoundabout = isRoundabout
	ife.PrevNode = baseNode
	ife.PrevEdge = edge

	return nil
}

func (ife *InstructionsFromEdges) GetEdgeInfo(fromNodeID, toNodeID int32) (datastructure.EdgeExtraInfo, error) {
	streetNamePrevEdge, err := ife.ContractedGraph.GetStreetNameCsr(fromNodeID, toNodeID)
	if err != nil {
		return datastructure.EdgeExtraInfo{}, err
	}
	roadClassPrevEdge, err := ife.ContractedGraph.GetRoadClassCsr(fromNodeID, toNodeID)
	if err != nil {
		return datastructure.EdgeExtraInfo{}, err
	}
	lanesPrevEdge, err := ife.ContractedGraph.GetLanesCsr(fromNodeID, toNodeID)
	if err != nil {
		return datastructure.EdgeExtraInfo{}, err
	}
	roadClassLinkPrevEdge, err := ife.ContractedGraph.GetRoadClassLinkCsr(fromNodeID, toNodeID)
	if err != nil {
		return datastructure.EdgeExtraInfo{}, err
	}
	pointsInBetweenPrevEdge, err := ife.ContractedGraph.GetEdgePointsInBetweenCsr(fromNodeID, toNodeID, false)
	if err != nil {
		return datastructure.EdgeExtraInfo{}, err
	}
	isRoundaboutPrevEdge, err := ife.ContractedGraph.IsRoundaboutCsr(fromNodeID, toNodeID)
	if err != nil {
		return datastructure.EdgeExtraInfo{}, err
	}
	return datastructure.NewEdgeExtraInfo(
		streetNamePrevEdge,
		roadClassPrevEdge,
		lanesPrevEdge,
		roadClassLinkPrevEdge,
		pointsInBetweenPrevEdge,
		isRoundaboutPrevEdge,
		false,
	), nil
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
func (ife *InstructionsFromEdges) CheckUTurn(sign int, name string, edge datastructure.EdgeCH) (bool, int) {
	isUTurn := false
	uTurnType := U_TURN_UNKNOWN

	if ife.doublePrevOrientation != 0 && (sign > 0) == (ife.PrevInstruction.Sign > 0) &&
		(abs(sign) == TURN_SLIGHT_RIGHT || abs(sign) == TURN_RIGHT || abs(sign) == TURN_SHARP_RIGHT) &&
		(abs(ife.PrevInstruction.Sign) == TURN_SLIGHT_RIGHT || abs(ife.PrevInstruction.Sign) == TURN_RIGHT || abs(ife.PrevInstruction.Sign) == TURN_SHARP_RIGHT) &&
		isSameName(ife.DoublePrevStreetName, name) {
		node := ife.ContractedGraph.GetNode(edge.ToNodeID)
		pointLat, pointLon := node.Lat, node.Lon
		baseNodeData := ife.ContractedGraph.GetNode(edge.FromNodeID)
		currentOrientation := calcOrientation(baseNodeData.Lat, baseNodeData.Lon, pointLat, pointLon)
		diff := math.Abs(ife.doublePrevOrientation - currentOrientation)
		diffAngle := diff * (180 / math.Pi)
		if diffAngle > 155 && diffAngle < 205 {
			isUTurn = true
			if sign < 0 {
				uTurnType = U_TURN_LEFT
			} else {
				uTurnType = U_TURN_RIGHT
			}
		}
	}
	return isUTurn, uTurnType
}

/*
Finish. tambah final instruction.
*/
func (ife *InstructionsFromEdges) Finish() error {

	doublePrevNode := ife.ContractedGraph.GetNode(ife.DoublePrevNode)

	baseNodeData := ife.ContractedGraph.GetNode(ife.PrevEdge.FromNodeID)

	prevEdgeExtraInfo, err := ife.GetEdgeInfo(ife.PrevEdge.FromNodeID, ife.PrevEdge.ToNodeID)
	if err != nil {
		return err
	}

	node := ife.ContractedGraph.GetNode(ife.PrevEdge.ToNodeID)
	point := datastructure.NewCoordinate(node.Lat, node.Lon)
	finishInstruction := NewInstruction(FINISH, ife.ContractedGraph.GetStreetNameFromID(prevEdgeExtraInfo.StreetName), point, false)
	finishInstruction.ExtraInfo["heading"] = BearingTo(doublePrevNode.Lat, doublePrevNode.Lon, baseNodeData.Lat, baseNodeData.Lon)
	newIns := NewInstruction(finishInstruction.Sign, finishInstruction.Name, finishInstruction.Point, false)
	ife.Ways = append(ife.Ways, &newIns)

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
func (ife *InstructionsFromEdges) GetTurnSign(edge datastructure.EdgeCH, baseNode, prevNode, adjNode int32, name string) (int, error) {
	baseNodeData := ife.ContractedGraph.GetNode(edge.FromNodeID)
	point := ife.ContractedGraph.GetNode(edge.ToNodeID)
	lat := point.Lat
	lon := point.Lon
	var prevNodeData datastructure.CHNode
	if ife.PrevNode != -1 {
		prevNodeData = ife.ContractedGraph.GetNode(ife.PrevNode)
	}

	ife.PrevOrientation = calcOrientation(prevNodeData.Lat, prevNodeData.Lon, baseNodeData.Lat, baseNodeData.Lon)
	sign := getTurnDirection(baseNodeData.Lat, baseNodeData.Lon, lat, lon, ife.PrevOrientation)

	alternativeTurnsCount, alternativeTurns, err := ife.GetAlternativeTurns(baseNode, adjNode, prevNode)
	if err != nil {
		return 0, err
	}
	isStreetSplit, err := ife.isStreetSplit(edge, ife.PrevEdge)
	if err != nil {
		return 0, err
	}
	isStreetMerged, err := ife.isStreetMerged(edge, ife.PrevEdge)
	if err != nil {
		return 0, err
	}

	if alternativeTurnsCount == 1 {
		if math.Abs(float64(sign)) > 1 && !(isStreetMerged || isStreetSplit) {

			return sign, nil
		}
		return IGNORE, nil
	}

	prevEdgeExtraInfo, err := ife.GetEdgeInfo(ife.PrevEdge.FromNodeID, ife.PrevEdge.ToNodeID)
	if err != nil {
		log.Printf("Error getting edge info: %v", err)
		return IGNORE, nil
	}

	edgeExtraInfo, err := ife.GetEdgeInfo(edge.FromNodeID, edge.ToNodeID)

	prevEdgeStreetName := ife.ContractedGraph.GetStreetNameFromID(prevEdgeExtraInfo.StreetName)
	if math.Abs(float64(sign)) > 1 {
		if (isSameName(name, prevEdgeStreetName)) ||
			isStreetMerged || isStreetSplit {
			return IGNORE, nil
		}
		return sign, nil
	}

	if ife.PrevEdge.Weight == 0 {
		return sign, nil
	}

	// get edge lain dari baseNode yang arahnya continue
	otherContinueEdge := ife.getOtherEdgeContinueDirection(baseNodeData.Lat, baseNodeData.Lon, ife.PrevOrientation, alternativeTurns) //

	otherContinueEdgeExtraInfo, err := ife.GetEdgeInfo(otherContinueEdge.FromNodeID, otherContinueEdge.ToNodeID)

	if err != nil {
		log.Printf("Error getting edge info: %v", err)
		return IGNORE, nil
	}

	prevCurrEdgeOrientationDiff := calculateOrientationDelta(baseNodeData.Lat, baseNodeData.Lon, lat, lon, ife.PrevOrientation) // bearing difference antara prevNode->baseNode->edge.ToNodeID/adjNode
	if otherContinueEdge.Weight != 0 {
		// terdapat edge lain (yang terhubung dengan baseNode) yang arahnya sama continue.
		if !isSameName(name, prevEdgeStreetName) {
			// current Street Name != prevEdge Street Name
			roadClass := ife.ContractedGraph.GetRoadClassFromID(edgeExtraInfo.RoadClass)
			prevRoadClass := ife.ContractedGraph.GetRoadClassFromID(prevEdgeExtraInfo.RoadClass)
			otherRoadClass := ife.ContractedGraph.GetRoadClassFromID(otherContinueEdgeExtraInfo.RoadClass)

			link := ife.ContractedGraph.GetRoadClassLinkFromID(edgeExtraInfo.RoadClassLink)
			prevLink := ife.ContractedGraph.GetRoadClassLinkFromID(prevEdgeExtraInfo.RoadClassLink)
			otherLink := ife.ContractedGraph.GetRoadClassLinkFromID(otherContinueEdgeExtraInfo.RoadClassLink)

			node := ife.ContractedGraph.GetNode(otherContinueEdge.ToNodeID)
			tmpLat, tmpLon := node.Lat, node.Lon

			if isMajorRoad(roadClass) {
				if (roadClass == prevRoadClass && link == prevLink) && (otherRoadClass != prevRoadClass || otherLink != prevLink) {
					// current road class == major road class && prevRoadClass sama dg current edge roadClass
					return IGNORE, nil
				}
			}

			prevOtherEdgeOrientation := calculateOrientationDelta(baseNodeData.Lat, baseNodeData.Lon, tmpLat, tmpLon, ife.PrevOrientation) // bearing difference antara prevNode->baseNode->otherContinueEdge.ToNodeID

			if math.Abs(prevCurrEdgeOrientationDiff)*(180/math.Pi) < 6 && math.Abs(prevOtherEdgeOrientation)*(180/math.Pi) > 8.6 && isSameName(name, prevEdgeStreetName) {
				// bearing difference antara prevEdge dan currentEDge < 6° (CONTINUE Direction), Edge otherContinueEdge > 8.6 (TURN SLIGHT or more direction). Nama prevEdge street == current Street
				return CONTINUE_ON_STREET, nil
			}

			if roadClass == "residential" || prevRoadClass == "residential" || (roadClass == "unclassified" && prevRoadClass == "unclassified") {
				// skip roadclass residential untuk mengurangi instructions.
				return IGNORE, nil
			}

			/*
				jika dari baseNode ada 2 jalan yang arahnya sama sama lurus/sedikit belok, tambah turn instruction ke currEdge. Misalkan:

						-----currentEdge---------
				baseNode
						-----otherContinueEdge---
			*/ // nolint: gofmt
			if prevCurrEdgeOrientationDiff > prevOtherEdgeOrientation {
				return KEEP_RIGHT, nil
			} else {
				return KEEP_LEFT, nil
			}
		}
	}

	if ok, err := ife.isLeavingCurrentStreet(prevEdgeStreetName, name, ife.PrevEdge, edge, alternativeTurns); !(isStreetMerged || isStreetSplit) &&
		(math.Abs(prevCurrEdgeOrientationDiff)*(180/math.Pi) > 34 || ok) {
		return sign, err
	} else if err != nil {
		return IGNORE, err
	}

	return IGNORE, nil
}

func isMajorRoad(roadClass string) bool {
	return roadClass == "motorway" || roadClass == "trunk" || roadClass == "primary" || roadClass == "secondary" || roadClass == "tertiary"
}
