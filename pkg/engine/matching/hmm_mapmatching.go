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
	"github.com/lintang-b-s/navigatorx/pkg/guidance"
)

const (
	routeNotFoundEta = -1000
)

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

	linearDistance := param.LinearDistance // in meter
	routeAlgo, maxTransitionDist := param.RouteAlgo, param.MaxTransitionDist

	// for every state between 2 adjacent gps observation, calculate the route length

	prevState := param.PrevState
	currentState := param.CurrentState

	// if state is virtual node, then only consider the virtual edge
	// if not, consider all edges of the state graph node
	sourceEdgeFilter := func(edge datastructure.Edge) bool {
		return true
	}

	targetEdgeFilter := func(edge datastructure.Edge) bool {
		return true
	}

	// calculate route shortest path distance between prev state and current state. routeDist in km
	path, _, _, routeDist := routeAlgo.ShortestPathBiDijkstra(prevState.ProjectionID, currentState.ProjectionID, sourceEdgeFilter,
		targetEdgeFilter)
	// shortest path harus dari antara 2 projectionPoint.

	routeDist *= 1000 // now routeDist in meter

	if routeDist != routeNotFoundEta && math.Abs(routeDist-linearDistance) < maxTransitionDist {

		transitionProb := computeTransitionLogProb(routeDist,
			linearDistance)

		return newTransitionWithProb(prevState.StateID, currentState.StateID, transitionProb, path)

	}
	return newTransitionWithProb(prevState.StateID, currentState.StateID, math.Inf(-1), nil)
}

func (hmm *HMMMapMatching) MapMatch(gps []datastructure.StateObservationPair, nextStateID *int) ([]datastructure.Coordinate, []datastructure.Edge, []datastructure.Coordinate, error) {

	stateDataMap := make(map[int]*datastructure.State)
	viterbi := NewViterbiAlgorithm(true)

	viterbiResetCount := 0

	statesPath := make([]int, 0)

	prevObservation := gps[0]

	for i := 0; i < len(gps); i++ {
		for j := 0; j < len(gps[i].State); j++ {
			fromNode := hmm.ch.GetNode(gps[i].State[j].EdgeFromNodeID)
			toNode := hmm.ch.GetNode(gps[i].State[j].EdgeToNodeID)

			edge := hmm.ch.GetOutEdge(gps[i].State[j].EdgeID)
			projection, _ := hmm.projectPointToEdgeGeometry(edge, datastructure.NewCoordinate(
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

	// hmm.updateAllHMMStates(gps, queryGraph, nextStateID)

	routeAlgo := routingalgorithm.NewRouteAlgorithm(queryGraph)
	observationPath := make([]datastructure.Coordinate, 0)

	isBreak := false

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
				gps[i].Observation.Lat, gps[i].Observation.Lon) * 1000 // in meter

			workers := concurrent.NewWorkerPool[concurrent.CalculateTransitionProbParam, transitionWithProb](30,
				len(gps[i].State)*len(prevObservation.State))

			for j := 0; j < len(prevObservation.State); j++ {
				for k := 0; k < len(gps[i].State); k++ {

					prevState := prevObservation.State[j]
					currentState := gps[i].State[k]
					workers.AddJob(concurrent.NewCalculateTransitionProbParam(prevObservation, prevState, currentState, len(gps[i].State)*len(prevObservation.State),
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
				viterbiResetCount++
				hmm.handleHMMBreak(gps, i, viterbi, stateDataMap, transitionProbMatrix, prevObservation)
				isBreak = true
				break
			} else {
				processedObsCount++
			}

		}

		prevObservation = gps[i]

	}

	path := viterbi.ComputeMostLikelySequence()
	for _, p := range path {
		statesPath = append(statesPath, p.State)
		obs := gps[p.Observation]
		observationPath = append(observationPath, datastructure.NewCoordinate(obs.Observation.Lat, obs.Observation.Lon))
	}

	solutions := make([]datastructure.Coordinate, 0)
	solutionEdges := make([]datastructure.Edge, 0)
	obsCoords := make([]datastructure.Coordinate, 0, len(gps))

	for i := 0; i < len(statesPath); i++ {

		projectionLoc := stateDataMap[statesPath[i]].ProjectionLoc
		solutions = append(solutions, datastructure.Coordinate{
			Lat: projectionLoc[0],
			Lon: projectionLoc[1]})

		obsCoords = append(obsCoords, datastructure.NewCoordinate(
			gps[stateDataMap[statesPath[i]].ObservationID].Observation.Lat,
			gps[stateDataMap[statesPath[i]].ObservationID].Observation.Lon))
		solutionEdges = append(solutionEdges, queryGraph.GetOutEdge(stateDataMap[statesPath[i]].EdgeID))
	}

	log.Printf("Viterbi reset count %v", viterbiResetCount)
	log.Printf("Processed %v out of %v observations", processedObsCount, len(gps))

	if isBreak {
		return solutions, solutionEdges, observationPath, hmmbreakErr
	}
	return solutions, solutionEdges, observationPath, nil

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

		// for all <edgeID, []snap> create virtual edge & virtual node between edge.
		count++
		edge := queryGraph.GetOutEdge(edgeID)

		edgePointsInBetween := queryGraph.GetEdgePointsInBetween(edge.EdgeID)

		fromNode := queryGraph.GetNode(edge.FromNodeID)
		toNode := queryGraph.GetNode(edge.ToNodeID)

		removeUnsuedEdgesInOsmWay(queryGraph, edge)

		// sort snaps based on their projection place in line points.

		if isEdgeRoundabout(edgePointsInBetween) {
			// is edge is a roundabout , then sort based on observationID
			sort.Slice(edgeSnaps, func(i, j int) bool {
				a := edgeSnaps[i]
				b := edgeSnaps[j]

				return a.ObservationID < b.ObservationID
			})
		} else {
			// is edge is not roundabout , then sort based on distance to fromNode
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
		}

		// add virtual edges to the graph

		prevPoint := edgePointsInBetween[0]
		prevNodeID := fromNode.ID

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
			}

			// add virtual edge
			hmm.addVirtualEdge(queryGraph, edge, prevNodeID, newNodeID,
				prevPoint, currSnapProjection)

			prevNodeID = newNodeID
			prevPoint = currSnapProjection

		}

		// add <finalProjection, toNode> virtual edge
		toNodeCoordinate := datastructure.NewCoordinate(toNode.Lat, toNode.Lon)
		hmm.addVirtualEdge(queryGraph, edge, prevNodeID, toNode.ID,
			prevPoint, toNodeCoordinate,
		)

	}
}

func (hmm *HMMMapMatching) projectPointToEdgeGeometry(
	edge datastructure.Edge, point datastructure.Coordinate) (datastructure.Coordinate, int) {
	edgePoints := hmm.ch.GetEdgePointsInBetween(edge.EdgeID)

	bestProjection := edgePoints[0]

	bestIndex := 0
	minDist := math.MaxFloat64
	for i := 0; i < len(edgePoints)-1; i++ {
		p1 := edgePoints[i]
		p2 := edgePoints[i+1]

		projection := geo.ProjectPointToLineCoord(p1, p2, point)
		distance := geo.CalculateHaversineDistance(point.Lat, point.Lon,
			projection.Lat, projection.Lon) * 1000

		if distance < minDist {
			minDist = distance
			bestProjection = projection
			bestIndex = i
		}
	}

	return bestProjection, bestIndex
}

func removeUnsuedEdgesInOsmWay(queryGraph ContractedGraph, matchedEdge datastructure.Edge) {

	queryGraph.RemoveEdge(matchedEdge.EdgeID, matchedEdge.FromNodeID, matchedEdge.ToNodeID)

}

func (hmm *HMMMapMatching) addVirtualEdge(queryGraph ContractedGraph,
	matchedEdge datastructure.Edge, prevNodeID, nodeID int32, prevProjection, currProjection datastructure.Coordinate,
) int32 {

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

func isEdgeRoundabout(linePoints []datastructure.Coordinate) bool {
	sumDeltaBearing := 0.0
	for i := 0; i < len(linePoints)-2; i++ {
		p1 := linePoints[i]
		p2 := linePoints[i+1]
		p3 := linePoints[i+2]

		bearing1 := guidance.BearingTo(p1.Lat, p1.Lon, p2.Lat, p2.Lon)
		bearing2 := guidance.BearingTo(p2.Lat, p2.Lon, p3.Lat, p3.Lon)

		sumDeltaBearing += bearing2 - bearing1
	}
	if sumDeltaBearing >= 270 {
		return true
	}
	return false
}

// for debugging purpose, In this case It's too difficult to debug using only vscode/IDE debugger considering the number of transitions.
func (hmm *HMMMapMatching) handleHMMBreak(gps []datastructure.StateObservationPair, i int, viterbi *ViterbiAlgorithm,
	stateDataMap map[int]*datastructure.State, transitionProbMatrix map[Transition]float64,
	prevObservation datastructure.StateObservationPair,
) {
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

}
