package matching

import (
	"fmt"
	"log"
	"math"
	"os"
	"sort"
	"strings"

	"github.com/lintang-b-s/navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/navigatorx/pkg/contractor"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/engine/routingalgorithm"
	"github.com/lintang-b-s/navigatorx/pkg/geo"
)

type ContractedGraph interface {
	GetOutEdges() []datastructure.Edge
	GetInEdges() []datastructure.Edge

	GetNodes() []datastructure.CHNode
	GetFirstInEdges() [][]int32
	GetFirstOutEdges() [][]int32
	AddEdge(newEdge datastructure.Edge)
	SetEdgeInfo(edgeInfo datastructure.EdgeExtraInfo)
	RemoveEdge(edgeID int32, fromNodeID int32, toNodeID int32)
	AddNode(node datastructure.CHNode)

	SnapLocationToRoadNetworkNodeH3(ways []datastructure.KVEdge, wantToSnap []float64) int32

	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32

	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.Edge
	GetInEdge(edgeID int32) datastructure.Edge
	IsRoundabout(edgeID int32) bool

	GetOutEdgesLen() int

	GetEdgeInfo(edgeID int32) datastructure.EdgeExtraInfo

	IsShortcut(edgeID int32) bool
	GetEdgePointsInBetween(edgeID int32) []datastructure.Coordinate
	IsTrafficLight(nodeID int32) bool

	SaveToFile() error
	LoadGraph() error
	GetStreetDirection(streetName int) [2]bool

	GetStreetNameFromID(streetName int) string
	GetRoadClassFromID(roadClass uint8) string
	GetRoadClassLinkFromID(roadClassLink uint8) string
	SetPointsInBetween(edgeID int32, points []datastructure.Coordinate) (int, int)
}

type RouteAlgorithm interface {
	ShortestPathAStar(int32, int32) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64)
	ShortestPathBiDijkstraCH(from, to int32) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64)
	ShortestPathBiDijkstra(from, to int32, fromEdgeFilter, toEdgeFilter func(edge datastructure.Edge) bool) ([]datastructure.Coordinate, []datastructure.Edge,
		float64, float64)
}
type kvdb interface {
	GetNearestStreetsFromPointCoord(lat, lon float64) ([]datastructure.KVEdge, error)
}

type HMMMapMatching struct {
	ch ContractedGraph
	db kvdb
}

func NewHMMMapMatching(ch ContractedGraph, db kvdb) *HMMMapMatching {
	return &HMMMapMatching{
		ch: ch,
		db: db,
	}
}

const (
	sigmaZ = 4.07
	beta   = 0.0009
)

func computeTransitionLogProb(routeLength, greateCircleDistance float64) float64 {
	obsStateDiff := math.Abs(greateCircleDistance - routeLength)
	return math.Log(1.0/beta) - (obsStateDiff / beta)
}

func computeEmissionLogProb(obsStateDist float64) float64 {
	return math.Log(1.0/(math.Sqrt(2*math.Pi)*sigmaZ)) + (-0.5 * math.Pow(obsStateDist/sigmaZ, 2))
}

type transitionWithProb struct {
	transition     Transition
	transitionProb float64
	path           []datastructure.Coordinate
}

func newTransitionWithProb(prevState, currState int, transitionProb float64,
	path []datastructure.Coordinate) transitionWithProb {
	return transitionWithProb{
		transition:     NewTransition(prevState, currState),
		transitionProb: transitionProb,
		path:           path,
	}
}

func (hmm *HMMMapMatching) calculateTransitionProb(param concurrent.CalculateTransitionProbParam) transitionWithProb {

	i, j, k, linearDistance := param.I, param.J, param.K, param.LinearDistance
	routeAlgo, maxTransitionDist := param.RouteAlgo, param.MaxTransitionDist

	// for every state between 2 adjacent gps observation, calculate the route length

	prevState := param.PrevObservation.State[j]
	currentState := param.Gps[i].State[k]

	if prevState.EdgeToNodeID == currentState.EdgeFromNodeID {
		projectionDist := geo.CalculateHaversineDistance(prevState.ProjectionLoc[0], prevState.ProjectionLoc[1],
			currentState.ProjectionLoc[0], currentState.ProjectionLoc[1]) * 1000

		transitionProb := computeTransitionLogProb(projectionDist,
			linearDistance)

		return newTransitionWithProb(prevState.StateID, currentState.StateID, transitionProb,
			[]datastructure.Coordinate{datastructure.NewCoordinate(prevState.ProjectionLoc[0], prevState.ProjectionLoc[1])})

	}

	// if state is virtual node, then only consider the virtual edge
	// if not, consider all edges of the state graph node
	sourceEdgeFilter := func(edge datastructure.Edge) bool {
		return true
	}

	targetEdgeFilter := func(edge datastructure.Edge) bool {
		return true
	}

	if prevState.GetType() == datastructure.VirtualNode {
		prevStatef := prevState
		sourceEdgeFilter = func(edge datastructure.Edge) bool {
			return edge.EdgeID == prevStatef.OutgoingVirtualEdgeID
		}
	}

	if currentState.GetType() == datastructure.VirtualNode {
		currentStatef := currentState
		targetEdgeFilter = func(edge datastructure.Edge) bool {
			return edge.EdgeID == currentStatef.IncomingVirtualEdgeID
		}
	}

	// calculate route shortest path distance between prev state and current state
	path, _, _, routeDist := routeAlgo.ShortestPathBiDijkstra(prevState.ProjectionID, currentState.ProjectionID, sourceEdgeFilter,
		targetEdgeFilter)
	// shortest path harus dari antara 2 projectionPoint.

	routeDist *= 1000

	if routeDist != -1000 && math.Abs(routeDist-linearDistance) < maxTransitionDist {

		transitionProb := computeTransitionLogProb(routeDist,
			linearDistance)

		return newTransitionWithProb(prevState.StateID, currentState.StateID, transitionProb, path)

	} else if prevState.EdgeID == currentState.EdgeID {

		projectionDist := geo.CalculateHaversineDistance(prevState.ProjectionLoc[0], prevState.ProjectionLoc[1],
			currentState.ProjectionLoc[0], currentState.ProjectionLoc[1]) * 1000

		transitionProb := computeTransitionLogProb(projectionDist,
			linearDistance)

		return newTransitionWithProb(prevState.StateID, currentState.StateID, transitionProb,
			[]datastructure.Coordinate{datastructure.NewCoordinate(prevState.ProjectionLoc[0], prevState.ProjectionLoc[1])})

	}

	return newTransitionWithProb(prevState.StateID, currentState.StateID, math.Inf(-1), nil)
}

func (hmm *HMMMapMatching) MapMatch(gps []datastructure.StateObservationPair, nextStateID int) ([]datastructure.Coordinate, []datastructure.Edge, []datastructure.Coordinate) {

	stateDataMap := make(map[int]*datastructure.State)
	viterbi := NewViterbiAlgorithm(true)

	viterbiResetCount := 0

	statesPath := make([]int, 0)

	prevObservation := gps[0]

	for i := 0; i < len(gps); i++ {
		for j := 0; j < len(gps[i].State); j++ {
			fromNode := hmm.ch.GetNode(gps[i].State[j].EdgeFromNodeID)
			toNode := hmm.ch.GetNode(gps[i].State[j].EdgeToNodeID)

			pointsInBetween := hmm.ch.GetEdgePointsInBetween(gps[i].State[j].EdgeID)
			pos := geo.PointPositionBetweenLinePoints(gps[i].Observation.Lat,
				gps[i].Observation.Lon, pointsInBetween) - 1

			from := pointsInBetween[pos]
			to := pointsInBetween[pos+1]

			projection := geo.ProjectPointToLineCoord(datastructure.NewCoordinate(
				from.Lat,
				from.Lon,
			), datastructure.NewCoordinate(
				to.Lat,
				to.Lon,
			), datastructure.NewCoordinate(
				gps[i].Observation.Lat,
				gps[i].Observation.Lon,
			))

			gps[i].State[j].ProjectionLoc = [2]float64{projection.Lat, projection.Lon}

			distToSource := geo.CalculateHaversineDistance(gps[i].Observation.Lat, gps[i].Observation.Lon, fromNode.Lat, fromNode.Lon) * 1000
			distToTarget := geo.CalculateHaversineDistance(gps[i].Observation.Lat, gps[i].Observation.Lon, toNode.Lat, toNode.Lon) * 1000

			projectionID := int32(-1) // first set projectionID  == -1
			// set projectionID of state to graph node ID if the distance between gps observation and source node/target node is so close
			if distToSource < distToTarget && distToSource < minDistToGraphNode {
				projectionID = fromNode.ID
				gps[i].State[j].Type = datastructure.GraphNode
			} else if distToTarget < distToSource && distToTarget < minDistToGraphNode {
				projectionID = toNode.ID
				gps[i].State[j].Type = datastructure.GraphNode
			}
			// if not set to graph node, then its a virtual node with initial projectionID = -1
			gps[i].State[j].ProjectionID = projectionID

		}
	}

	processedObsCount := 0

	queryGraph := hmm.buildQueryGraph(gps)

	hmm.updateAllHMMStates(gps, queryGraph, nextStateID)

	routeAlgo := routingalgorithm.NewRouteAlgorithm(queryGraph)
	observationPath := make([]datastructure.Coordinate, 0)

	for i := 0; i < len(gps); i++ {

		emissionProbMatrix := make(map[int]float64)
		states := make([]int, 0)

		transitionProbMatrix := make(map[Transition]float64)

		for j := 0; j < len(gps[i].State); j++ {

			projectionLoc := gps[i].State[j].ProjectionLoc

			distance := geo.CalculateHaversineDistance(projectionLoc[0], projectionLoc[1],
				gps[i].Observation.Lat, gps[i].Observation.Lon) * 1000 // distance between gps observation point and projection point (gps point to road segment/state)
			emissionProb := computeEmissionLogProb(distance)

			emissionProbMatrix[gps[i].State[j].StateID] = emissionProb

			states = append(states, gps[i].State[j].StateID)

			stateDataMap[gps[i].State[j].StateID] = gps[i].State[j]

		}

		if i == 0 {
			viterbi.StartWithInitialStateProbabilities(int(gps[i].Observation.ID), states, emissionProbMatrix)

			processedObsCount++
		} else {
			// calculate linear distance between prev observation and current observation
			linearDistance := geo.CalculateHaversineDistance(prevObservation.Observation.Lat, prevObservation.Observation.Lon,
				gps[i].Observation.Lat, gps[i].Observation.Lon) * 1000

			workers := concurrent.NewWorkerPool[concurrent.CalculateTransitionProbParam, transitionWithProb](30,
				len(gps[i].State)*len(prevObservation.State))

			for j := 0; j < len(prevObservation.State); j++ {
				for k := 0; k < len(gps[i].State); k++ {
					if prevObservation.State[j].EdgeFromNodeID == gps[i].State[k].EdgeToNodeID &&
						prevObservation.State[j].EdgeToNodeID == gps[i].State[k].EdgeFromNodeID {
						// if u-turn is detected, then skip this state pair
						continue
					}

					workers.AddJob(concurrent.NewCalculateTransitionProbParam(prevObservation, gps, i, j, k, len(gps[i].State)*len(prevObservation.State),
						linearDistance, maxTransitionDist, routeAlgo))
				}
			}

			workers.Close()
			workers.Start(hmm.calculateTransitionProb)

			workers.Wait()

			for transitionItem := range workers.CollectResults() {
				if transitionItem.transitionProb == math.Inf(-1) {
					continue
				}
				transitionProbMatrix[transitionItem.transition] = transitionItem.transitionProb
			}

			err := viterbi.NextStep(int(gps[i].Observation.ID), states, emissionProbMatrix, transitionProbMatrix, nil)
			if err != nil {
				hmm.handleHMMBreak(gps, i, viterbi, stateDataMap, transitionProbMatrix, prevObservation, routeAlgo,
					&statesPath, &observationPath, &viterbiResetCount)
				break
			} else {
				processedObsCount++
			}

		}

		prevObservation = gps[i]

		if (processedObsCount+1)%(int(0.05*float64(len(gps)))) == 0 {
			log.Printf("Processed %v out of %v observations", processedObsCount+1, len(gps))
		}

	}

	path := viterbi.ComputeMostLikelySequence()
	for _, p := range path {
		statesPath = append(statesPath, p.State)
		obs := gps[p.Observation]
		observationPath = append(observationPath, datastructure.NewCoordinate(obs.Observation.Lat, obs.Observation.Lon))
	}
	solutions := make([]datastructure.Coordinate, 0)
	solutionEdges := make([]datastructure.Edge, 0)

	for i := 1; i < len(statesPath); i++ {

		projectionLoc := stateDataMap[statesPath[i]].ProjectionLoc
		solutions = append(solutions, datastructure.Coordinate{
			Lat: projectionLoc[0],
			Lon: projectionLoc[1]})

		solutionEdges = append(solutionEdges, queryGraph.GetOutEdge(stateDataMap[statesPath[i]].EdgeID))
	}

	log.Printf("Viterbi reset count %v", viterbiResetCount)
	log.Printf("Processed %v out of %v observations", processedObsCount, len(gps))

	return solutions, solutionEdges, observationPath
}

func (hmm *HMMMapMatching) buildQueryGraph(gps []datastructure.StateObservationPair) *contractor.ContractedGraph {
	// create query graph from ch graph

	queryGraph := contractor.NewContractedGraphFromOtherGraph(hmm.ch.(*contractor.ContractedGraph))

	hmm.splitEdges(gps, queryGraph)

	return queryGraph
}

func (hmm *HMMMapMatching) splitEdges(gps []datastructure.StateObservationPair, queryGraph ContractedGraph) {

	snaps := make([]*datastructure.State, 0) // store all hidden state candidates

	// flatten all gps states into one slice.
	for i := 0; i < len(gps); i++ {
		for j := 0; j < len(gps[i].State); j++ {
			snaps = append(snaps, gps[i].State[j])
		}
	}

	// split all matched edges at each snap point (only apply to states that have type = virtual node). split into <source, projected1, projected2, ...., target>
	// first create map<edgeId, []snap>
	edgeSnapMap := make(map[int32][]*datastructure.State)
	for i := 0; i < len(snaps); i++ {
		snap := snaps[i]
		if snap.Type == datastructure.GraphNode {
			continue
		}

		if _, ok := edgeSnapMap[snap.EdgeID]; !ok {
			edgeSnapMap[snap.EdgeID] = make([]*datastructure.State, 0)
		}
		edgeSnapMap[snap.EdgeID] = append(edgeSnapMap[snap.EdgeID], snap)

	}

	// split all edges in edgesSnapMap
	count := 0
	for edgeID, edgeSnaps := range edgeSnapMap {
		if (count+1)%100 == 0 {
			log.Printf("splitting edge %d of %d edges in querygraph", count+1, len(edgeSnapMap))
		}

		// for all <edgeID, []snap> create virtual edge & virtual node between edge.
		count++
		edge := queryGraph.GetOutEdge(edgeID)

		edgePointsInBetween := queryGraph.GetEdgePointsInBetween(edge.EdgeID)

		fromNode := queryGraph.GetNode(edge.FromNodeID)
		toNode := queryGraph.GetNode(edge.ToNodeID)

		snapIndexInPoints := make(map[int]int)

		getProjectionPlaceInLinePoints(edgePointsInBetween, edgeSnaps, snapIndexInPoints)

		removeUnsuedEdgesInOsmWay(queryGraph, edge)

		// sort snaps based on their projection place in line points.

		snapDists := make([]float64, len(edgeSnaps))
		for i := 0; i < len(edgeSnaps); i++ {
			snapDists[i] = geo.CalculateHaversineDistance(fromNode.Lat, fromNode.Lon,
				edgeSnaps[i].ProjectionLoc[0], edgeSnaps[i].ProjectionLoc[1])
		}

		sort.Slice(edgeSnaps, func(i, j int) bool {
			a := edgeSnaps[i]
			b := edgeSnaps[j]
			distA := geo.CalculateHaversineDistance(fromNode.Lat, fromNode.Lon,
				a.ProjectionLoc[0], a.ProjectionLoc[1])
			distB := geo.CalculateHaversineDistance(fromNode.Lat, fromNode.Lon,
				b.ProjectionLoc[0], b.ProjectionLoc[1])

			if distA < distB {
				return true
			} else if distA > distB {
				return false
			} else {
				return a.ObservationID < b.ObservationID
			}
		})

		// add virtual edges to the graph

		prevPoint := edgePointsInBetween[0]
		prevNodeID := fromNode.ID
		prevPositionInLinePoints := 0

		// for every snap projection point, add virtual edge <prevNodeID, newProjectionNodeID>
		for i := 0; i < len(edgeSnaps); i++ {

			snap := edgeSnaps[i]
			currSnapProjection := datastructure.NewCoordinate(snap.ProjectionLoc[0], snap.ProjectionLoc[1])

			if prevPoint.Lat == currSnapProjection.Lat &&
				prevPoint.Lon == currSnapProjection.Lon {

				edgeSnaps[i].ProjectionID = prevNodeID
				continue
			}

			// add virtual edge & projected node into query graph
			newNodeID := edgeSnaps[i].ProjectionID
			if edgeSnaps[i].ProjectionID == -1 {
				newNodeID = int32(len(queryGraph.GetNodes()))

				newProjectionNode := datastructure.NewCHNode(
					currSnapProjection.Lat,
					currSnapProjection.Lon,
					-1,
					newNodeID,
				)
				edgeSnaps[i].ProjectionID = newNodeID

				queryGraph.AddNode(newProjectionNode)
			} else {
				log.Printf("node already exist")
			}

			// add virtual edge
			hmm.addVirtualEdge(queryGraph, edge, prevNodeID, newNodeID,
				prevPoint, currSnapProjection, prevPositionInLinePoints,
				snapIndexInPoints[edgeSnaps[i].StateID])

			prevNodeID = newNodeID
			prevPoint = currSnapProjection

		}

		// add <finalProjection, toNode> virtual edge
		toNodeCoordinate := datastructure.NewCoordinate(toNode.Lat, toNode.Lon)
		hmm.addVirtualEdge(queryGraph, edge, prevNodeID, toNode.ID,
			prevPoint, toNodeCoordinate, snapIndexInPoints[edgeSnaps[len(edgeSnaps)-1].StateID],
			len(edgePointsInBetween))

	}

}

func removeUnsuedEdgesInOsmWay(queryGraph ContractedGraph, matchedEdge datastructure.Edge) {

	queryGraph.RemoveEdge(matchedEdge.EdgeID, matchedEdge.FromNodeID, matchedEdge.ToNodeID)

}

func (hmm *HMMMapMatching) addVirtualEdge(queryGraph ContractedGraph,
	matchedEdge datastructure.Edge, prevNodeID, nodeID int32, prevProjection, currProjection datastructure.Coordinate,
	prevPositionInLinePoints, currPositionInLinePoints int) int32 {

	// calculate edge distance
	dist := 0.0

	dist = geo.CalculateHaversineDistance(prevProjection.Lat, prevProjection.Lon,
		currProjection.Lat, currProjection.Lon)

	dist *= 1000 // meter

	speed := matchedEdge.Dist / matchedEdge.Weight // meter/minutes
	eta := dist / speed

	// add new edge to graph
	newEdgeID := int32(queryGraph.GetOutEdgesLen())
	newEdge := datastructure.NewEdge(newEdgeID, nodeID, prevNodeID, -1, eta, dist)

	queryGraph.AddEdge(newEdge)

	matchedEdgeInfo := queryGraph.GetEdgeInfo(matchedEdge.EdgeID)

	queryGraph.SetEdgeInfo(
		datastructure.NewEdgeExtraInfo(
			matchedEdgeInfo.StreetName,
			matchedEdgeInfo.RoadClass,
			matchedEdgeInfo.Lanes,
			matchedEdgeInfo.RoadClassLink,
			uint32(0), uint32(0),
		))

	return newEdgeID
}

// updateAllHMMStates.
func (hmm *HMMMapMatching) updateAllHMMStates(gps []datastructure.StateObservationPair, queryGraph ContractedGraph, nextStateID int) {

	for i := 0; i < len(gps); i++ {
		initialGpsStatesLen := len(gps[i].State)
		for j := 0; j < initialGpsStatesLen; j++ {
			state := gps[i].State[j]
			if state.GetType() == datastructure.VirtualNode {
				virtualOutgouingEdge := queryGraph.GetNodeFirstOutEdges(state.ProjectionID)[0]
				virtualIncomingEdge := queryGraph.GetNodeFirstInEdges(state.ProjectionID)[0]

				state.SetIncomingVirtualEdge(virtualIncomingEdge)
				state.SetOutgoingVirtualEdge(virtualOutgouingEdge)

				// add new hidden state for this gps observation same as this state but with reversed incoming & outgoing virtual edge
				newState := *state
				newState.SetIncomingVirtualEdge(virtualOutgouingEdge)
				newState.SetOutgoingVirtualEdge(virtualIncomingEdge)
				newState.StateID = nextStateID

				temp := newState.EdgeFromNodeID
				newState.EdgeFromNodeID = newState.EdgeToNodeID
				newState.EdgeToNodeID = temp
				nextStateID++

				gps[i].State = append(gps[i].State, &newState)
			}
		}
	}
}

// getProjectionPlaceInLinePoints get the projection place of each snap point in line points
func getProjectionPlaceInLinePoints(linePoints []datastructure.Coordinate, edgeSnaps []*datastructure.State, snappedIndexInPoints map[int]int) {
	for i := 0; i < len(edgeSnaps); i++ {
		position := geo.PointPositionBetweenLinePoints(edgeSnaps[i].ProjectionLoc[0], edgeSnaps[i].ProjectionLoc[1], linePoints)
		snappedIndexInPoints[edgeSnaps[i].StateID] = position
	}
}

// for debugging purpose, In this case It's too difficult to debug using only vscode/IDE debugger considering the number of transitions.
func (hmm *HMMMapMatching) handleHMMBreak(gps []datastructure.StateObservationPair, i int, viterbi *ViterbiAlgorithm,
	stateDataMap map[int]*datastructure.State, transitionProbMatrix map[Transition]float64,
	prevObservation datastructure.StateObservationPair, routeAlgo RouteAlgorithm,
	statesPath *[]int, observationPath *[]datastructure.Coordinate, viterbiResetCount *int) {
	log.Printf("broken viterbi at observation: %v", i)

	hist := viterbi.messageHistory
	var sb strings.Builder
	sb.WriteString("Message history with log probabilities\n\n")

	for j, message := range hist {

		fmt.Fprintf(&sb, "Time step %d\n", j)

		for state, value := range message {
			fmt.Fprintf(&sb, "stateID %v: %f  lat, lon: %v, %v\n", state, value,
				stateDataMap[state].ProjectionLoc[0], stateDataMap[state].ProjectionLoc[1])
		}

		if j == len(hist)-1 {

			fmt.Fprintf(&sb, "\n PrevObservation: \n")
			fmt.Fprintf(&sb, "prevObs lat, lon: %v, %v\n", prevObservation.Observation.Lat, prevObservation.Observation.Lon)
			for _, prevState := range prevObservation.State {
				fmt.Fprintf(&sb, "Observed stateID %v: lat, lon: %v, %v, stateEdgeID: %v, \n", prevState.StateID, prevState.ProjectionLoc[0], prevState.ProjectionLoc[1],
					prevState.EdgeID)
				prevFromNode := hmm.ch.GetNode(prevState.EdgeFromNodeID)
				prevToNode := hmm.ch.GetNode(prevState.EdgeToNodeID)
				fmt.Fprintf(&sb, "edgeFromLat, edgeFromLat: %v, %v, edgeToLat, edgeToLat: %v, %v\n", prevFromNode.Lat, prevFromNode.Lon,
					prevToNode.Lat, prevToNode.Lon)

				fmt.Fprintf(&sb, "fromNodeID, toNodeID: %v, %v\n", prevState.EdgeFromNodeID, prevState.EdgeToNodeID)
			}

			fmt.Fprintf(&sb, "\n currObs: %v\n", gps[i].Observation.ID)
			fmt.Fprintf(&sb, "\n currObs lat, lon: %v, %v\n", gps[i].Observation.Lat, gps[i].Observation.Lon)
			for _, currState := range gps[i].State {
				currFromNode := hmm.ch.GetNode(currState.EdgeFromNodeID)
				currToNode := hmm.ch.GetNode(currState.EdgeToNodeID)
				fmt.Fprintf(&sb, "Observed stateID %v: lat, lon: %v, %v, stateEdgeID: %v\n", currState.StateID, currState.ProjectionLoc[0], currState.ProjectionLoc[1],
					currState.EdgeID)

				fmt.Fprintf(&sb, "edgeFromLat, edgeFromLat: %v, %v, edgeToLat, edgeToLat: %v, %v\n", currFromNode.Lat, currFromNode.Lon,
					currToNode.Lat, currToNode.Lon)

				fmt.Fprintf(&sb, "fromNodeID, toNodeID: %v, %v\n", currState.EdgeFromNodeID, currState.EdgeToNodeID)
			}
		}

		fmt.Fprintf(&sb, "\ntransitionProbMatrix:\n")
		for transition, prob := range transitionProbMatrix {
			fmt.Fprintf(&sb, "From %v to %v: %v\n", transition.From, transition.To, prob)
		}

		fmt.Fprintf(&sb, "\n")
	}

	//write sb string to file
	f, err := os.Create(fmt.Sprintf("output/message_history_%v.txt", i))

	if err != nil {
		log.Println(err)
	}

	_, err = f.WriteString(sb.String())
	if err != nil {
		log.Println(err)
	}

	*viterbiResetCount++
	path := viterbi.ComputeMostLikelySequence()
	if len(path) > 1 {
		for j := 0; j < len(path)-1; j++ {
			p := path[j]
			*statesPath = append(*statesPath, p.State)
			obs := gps[p.Observation]
			*observationPath = append(*observationPath, datastructure.NewCoordinate(obs.Observation.Lat, obs.Observation.Lon))
		}
	}

}
