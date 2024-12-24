package guidance

import (
	"errors"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"math"
)

type ContractedGraph interface {
	GetFirstOutEdge(nodeIDx int32) []int32
	GetFirstInEdge(nodeIDx int32) []int32
	GetNode(nodeIDx int32) datastructure.CHNode2
	GetOutEdge(edgeIDx int32) datastructure.EdgeCH
	GetInEdge(edgeIDx int32) datastructure.EdgeCH
	GetStreetDirection(streetName string) [2]bool
	GetStreetInfo(streetName string) datastructure.StreetExtraInfo
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

func NewDrivingInstruction(ins Instruction, description string) DrivingInstruction {
	return DrivingInstruction{
		Instruction: description,
		Point:       ins.Point,
		StreetName:  ins.Name,
		ETA:         Round(ins.Time, 2),
		Distance:    Round(ins.Distance, 2),
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

	for i, _ := range ife.Ways {
		drivingInstructions = append(drivingInstructions, NewDrivingInstruction(*ife.Ways[i], drivingInstructionsDesc[i]))
	}
	return drivingInstructions, nil
}

func (ife *InstructionsFromEdges) AddInstructionFromEdge(edge datastructure.EdgeCH) {
	adjNode := edge.ToNodeIDX
	baseNode := edge.BaseNodeIDx

	baseNodeData := ife.ContractedGraph.GetNode(baseNode)

	adjNodeData := ife.ContractedGraph.GetNode(adjNode)
	adjLat := adjNodeData.Lat
	adjLon := adjNodeData.Lon
	var latitude, longitude float64

	isRoundabout := edge.Roundabout

	latitude = adjLat
	longitude = adjLon
	var prevNodeData datastructure.CHNode2
	if ife.PrevNode != -1 {
		prevNodeData = ife.ContractedGraph.GetNode(ife.PrevNode)
	}

	name := edge.StreetName

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
		ife.PrevInstruction.ExtraInfo["heading"] = heading // bearing dari titik awal ke edge.ToNodeIDX (arah edge)
		ife.Ways = append(ife.Ways, ife.PrevInstruction)

	} else if isRoundabout {
		// current edge bundaran
		if !ife.PrevInRoundabout {
			sign := USE_ROUNDABOUT
			point := datastructure.NewCoordinate(baseNodeData.Lat, baseNodeData.Lon)
			roundaboutInstruction := NewRoundaboutInstruction()
			ife.doublePrevOrientation = ife.PrevOrientation
			if ife.PrevInstruction != nil {

				outEdges := ife.ContractedGraph.GetFirstOutEdge(baseNode)
				for _, eIDx := range outEdges {
					// add jumlah exit Point dari bundaran
					e := ife.ContractedGraph.GetOutEdge(eIDx)
					if (e.ToNodeIDX != ife.PrevNode) && !e.Roundabout {
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

		outgoingEdges := ife.ContractedGraph.GetFirstOutEdge(adjNode)
		for _, eIDx := range outgoingEdges {
			e := ife.ContractedGraph.GetOutEdge(eIDx)
			if !e.Roundabout {
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

		ife.DoublePrevStreetName = ife.PrevEdge.StreetName

	} else {
		sign := ife.GetTurnSign(edge, baseNode, ife.PrevNode, adjNode, name)
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
				ife.DoublePrevStreetName = ife.PrevEdge.StreetName
				ife.Ways = append(ife.Ways, ife.PrevInstruction)
			}
		}
	}
	prevLoc := geo.NewLocation(ife.PrevInstruction.Point.Lat, ife.PrevInstruction.Point.Lon)
	adjLoc := geo.NewLocation(adjLat, adjLon)
	dist := geo.HaversineDistance(prevLoc, adjLoc) * 1000
	ife.PrevInstruction.Distance += dist
	if edge.Weight == 0 {
		ife.PrevInstruction.Time += 0
	} else {
		currEdgeSpeed := (edge.Dist / edge.Weight)
		ife.PrevInstruction.Time += dist / currEdgeSpeed
	}

	ife.DoublePrevNode = ife.PrevNode
	ife.PrevInRoundabout = isRoundabout
	ife.PrevNode = baseNode
	ife.PrevEdge = edge
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
		node := ife.ContractedGraph.GetNode(edge.ToNodeIDX)
		pointLat, pointLon := node.Lat, node.Lon
		baseNodeData := ife.ContractedGraph.GetNode(edge.BaseNodeIDx)
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
func (ife *InstructionsFromEdges) Finish() {

	doublePrevNode := ife.ContractedGraph.GetNode(ife.DoublePrevNode)

	baseNodeData := ife.ContractedGraph.GetNode(ife.PrevEdge.BaseNodeIDx)

	node := ife.ContractedGraph.GetNode(ife.PrevEdge.ToNodeIDX)
	point := datastructure.NewCoordinate(node.Lat, node.Lon)
	finishInstruction := NewInstruction(FINISH, ife.PrevEdge.StreetName, point, false)
	finishInstruction.ExtraInfo["heading"] = BearingTo(doublePrevNode.Lat, doublePrevNode.Lon, baseNodeData.Lat, baseNodeData.Lon)
	newIns := NewInstruction(finishInstruction.Sign, finishInstruction.Name, finishInstruction.Point, false)
	ife.Ways = append(ife.Ways, &newIns)
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
func (ife *InstructionsFromEdges) GetTurnSign(edge datastructure.EdgeCH, baseNode, prevNode, adjNode int32, name string) int {
	baseNodeData := ife.ContractedGraph.GetNode(edge.BaseNodeIDx)
	point := ife.ContractedGraph.GetNode(edge.ToNodeIDX)
	lat := point.Lat
	lon := point.Lon
	var prevNodeData datastructure.CHNode2
	if ife.PrevNode != -1 {
		prevNodeData = ife.ContractedGraph.GetNode(ife.PrevNode)
	}

	ife.PrevOrientation = calcOrientation(prevNodeData.Lat, prevNodeData.Lon, baseNodeData.Lat, baseNodeData.Lon)
	sign := getTurnDirection(baseNodeData.Lat, baseNodeData.Lon, lat, lon, ife.PrevOrientation)

	alternativeTurnsCount, alternativeTurns := ife.GetAlternativeTurns(baseNode, adjNode, prevNode)

	if alternativeTurnsCount == 1 {
		if math.Abs(float64(sign)) > 1 && !(ife.isStreetMerged(edge, ife.PrevEdge) || ife.isStreetSplit(edge, ife.PrevEdge)) {

			return sign
		}
		return IGNORE
	}

	if math.Abs(float64(sign)) > 1 {
		if (isSameName(name, ife.PrevEdge.StreetName)) ||
			ife.isStreetMerged(edge, ife.PrevEdge) || ife.isStreetSplit(edge, ife.PrevEdge) {
			return IGNORE
		}
		return sign
	}

	if ife.PrevEdge.Weight == 0 {
		return sign
	}

	// get edge lain dari baseNode yang arahnya continue
	otherContinueEdge := ife.getOtherEdgeContinueDirection(baseNodeData.Lat, baseNodeData.Lon, ife.PrevOrientation, alternativeTurns) //

	prevCurrEdgeOrientationDiff := calculateOrientationDelta(baseNodeData.Lat, baseNodeData.Lon, lat, lon, ife.PrevOrientation) // bearing difference antara prevNode->baseNode->edge.ToNodeIDx/adjNode
	if otherContinueEdge.Weight != 0 {
		// terdapat edge lain (yang terhubung dengan baseNode) yang arahnya sama continue.
		if !isSameName(name, ife.PrevEdge.StreetName) {
			// current Street Name != prevEdge Street Name
			roadClass := edge.RoadClass
			prevRoadClass := ife.PrevEdge.RoadClass
			otherRoadClass := otherContinueEdge.RoadClass

			link := edge.RoadClassLink
			prevLink := ife.PrevEdge.RoadClassLink
			otherLink := otherContinueEdge.RoadClassLink

			node := ife.ContractedGraph.GetNode(otherContinueEdge.ToNodeIDX)
			tmpLat, tmpLon := node.Lat, node.Lon

			if isMajorRoad(roadClass) {
				if (roadClass == prevRoadClass && link == prevLink) && (otherRoadClass != prevRoadClass || otherLink != prevLink) {
					// current road class == major road class && prevRoadClass sama dg current edge roadClass
					return IGNORE
				}
			}

			prevOtherEdgeOrientation := calculateOrientationDelta(baseNodeData.Lat, baseNodeData.Lon, tmpLat, tmpLon, ife.PrevOrientation) // bearing difference antara prevNode->baseNode->otherContinueEdge.ToNodeIDx

			if math.Abs(prevCurrEdgeOrientationDiff)*(180/math.Pi) < 6 && math.Abs(prevOtherEdgeOrientation)*(180/math.Pi) > 8.6 && isSameName(name, ife.PrevEdge.StreetName) {
				// bearing difference antara prevEdge dan currentEDge < 6° (CONTINUE Direction), Edge otherContinueEdge > 8.6 (TURN SLIGHT or more direction). Nama prevEdge street == current Street
				return CONTINUE_ON_STREET
			}

			if roadClass == "residential" || prevRoadClass == "residential" || (roadClass == "unclassified" && prevRoadClass == "unclassified") {
				// skip roadclass residential untuk mengurangi instructions.
				return IGNORE
			}

			/*
				jika dari baseNode ada 2 jalan yang arahnya sama sama lurus/sedikit belok, tambah turn instruction ke currEdge. Misalkan:

						-----currentEdge---------
				baseNode
						-----otherContinueEdge---
			*/ // nolint: gofmt
			if  prevCurrEdgeOrientationDiff >  prevOtherEdgeOrientation {
				return KEEP_RIGHT
			} else {
				return KEEP_LEFT
			}
		}
	}

	if !(ife.isStreetMerged(edge, ife.PrevEdge) || ife.isStreetSplit(edge, ife.PrevEdge)) &&
		(math.Abs(prevCurrEdgeOrientationDiff)*(180/math.Pi) > 34 || ife.isLeavingCurrentStreet(ife.PrevEdge.StreetName, name, ife.PrevEdge, edge, alternativeTurns)) {
		return sign
	}

	return IGNORE
}

func isMajorRoad(roadClass string) bool {
	return roadClass == "motorway" || roadClass == "trunk" || roadClass == "primary" || roadClass == "secondary" || roadClass == "tertiary"
}
