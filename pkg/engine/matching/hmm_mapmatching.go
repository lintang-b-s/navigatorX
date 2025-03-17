package matching

import (
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/engine/routingalgorithm"
	"lintang/navigatorx/pkg/geo"
	"lintang/navigatorx/pkg/util"
	"log"
	"math"
)

type ContractedGraph interface {
	IsChReady() bool
	SnapLocationToRoadNetworkNodeH3(ways []datastructure.SmallWay, wantToSnap []float64) int32
	GetOutEdge(edgeID int32) datastructure.EdgeCH
	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdges() []datastructure.EdgeCH
	GetInEdges() []datastructure.EdgeCH
	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32
	GetNodes() []datastructure.CHNode
	GetFirstInEdges() [][]int32
	GetFirstOutEdges() [][]int32
	AddEdge(newEdge datastructure.EdgeCH)
	RemoveEdge(edgeID int32, fromNodeID int32, toNodeID int32)
	AddNode(node datastructure.CHNode)
}

type RouteAlgorithm interface {
	ShortestPathAStar(int32, int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64)
	ShortestPathBiDijkstraCH(from, to int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64)
	ShortestPathBiDijkstra(from, to int32, fromEdgeFilter, toEdgeFilter func(edge datastructure.EdgeCH) bool) ([]datastructure.Coordinate, []datastructure.EdgeCH,
		float64, float64)
}
type kvdb interface {
	GetNearestStreetsFromPointCoord(lat, lon float64) ([]datastructure.SmallWay, error)
	GetMapMatchWay(key int32) (datastructure.MapMatchOsmWay, error)
}

type HMMMapMatching struct {
	ch    ContractedGraph
	db    kvdb
	route RouteAlgorithm
}

func NewHMMMapMatching(ch ContractedGraph, db kvdb, route RouteAlgorithm) *HMMMapMatching {
	return &HMMMapMatching{
		ch:    ch,
		db:    db,
		route: route,
	}
}

const (
	sigmaZ = 4.07
	beta   = 0.009
)

func computeTransitionLogProb(routeLength, greateCircleDistance float64) float64 {
	obsStateDiff := math.Abs(greateCircleDistance - routeLength)
	return math.Log(1.0/beta) - (obsStateDiff / beta)
}

func computeEmissionLogProb(obsStateDist float64) float64 {
	return math.Log(1.0/(math.Sqrt(2*math.Pi)*sigmaZ)) + (-0.5 * math.Pow(obsStateDist/sigmaZ, 2))
}

func (hmm *HMMMapMatching) HiddenMarkovModelDecodingMapMatching(gps []datastructure.StateObservationPair, nextStateID int) ([]datastructure.Coordinate, []datastructure.EdgeCH, []datastructure.Coordinate) {

	stateDataMap := make(map[int]*datastructure.State)
	viterbi := NewViterbiAlgorithm(true)

	viterbiResetCount := 0

	statesPath := make([]int, 0)
	routePath := make(map[int]map[int][]datastructure.Coordinate)

	statePairCount := 0

	prevObservation := gps[0]

	for i := 0; i < len(gps); i++ {
		for j := 0; j < len(gps[i].State); j++ {
			fromNode := hmm.ch.GetNode(gps[i].State[j].EdgeFromNodeID)
			toNode := hmm.ch.GetNode(gps[i].State[j].EdgeToNodeID)

			projection := geo.ProjectPointToLineCoord(geo.NewCoordinate(
				fromNode.Lat,
				fromNode.Lon,
			), geo.NewCoordinate(
				toNode.Lat,
				toNode.Lon,
			), geo.NewCoordinate(
				gps[i].Observation.Lat,
				gps[i].Observation.Lon,
			))

			gps[i].State[j].ProjectionLoc = [2]float64{projection.Lat, projection.Lon}

			distToSource := geo.HaversineDistance(geo.NewLocation(gps[i].Observation.Lat, gps[i].Observation.Lon), geo.NewLocation(fromNode.Lat, fromNode.Lon)) * 1000
			distToTarget := geo.HaversineDistance(geo.NewLocation(gps[i].Observation.Lat, gps[i].Observation.Lon), geo.NewLocation(toNode.Lat, toNode.Lon)) * 1000

			projectionID := int32(-1) // first set projectionID  == -1
			// set projectionID of state to graph node ID if the distance between gps observation and source node/target node is so close
			if distToSource < distToTarget && distToSource < minDistToGraphNode {
				projectionID = fromNode.ID
				gps[i].State[j].Type = datastructure.GraphNode
			} else if distToTarget < distToSource && distToSource < minDistToGraphNode {
				projectionID = toNode.ID
				gps[i].State[j].Type = datastructure.GraphNode
			}
			// if not set to graph node, then its a virtual node with initial projectionID = -1
			gps[i].State[j].ProjectionID = projectionID

		}
	}

	processedObsCount := 0
	startNewViterbi := true
	prevPrevObservation := gps[0]

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
		} else if startNewViterbi {
			emissionProbMatrix = make(map[int]float64)
			states = make([]int, 0)

			for j := 0; j < len(prevObservation.State); j++ {

				projectionLoc := prevObservation.State[j].ProjectionLoc

				distance := geo.CalculateHaversineDistance(projectionLoc[0], projectionLoc[1],
					prevObservation.Observation.Lat, prevObservation.Observation.Lon) * 1000 // distance between gps observation point and projection point (gps point to road segment/state)
				emissionProb := computeEmissionLogProb(distance)

				emissionProbMatrix[prevObservation.State[j].StateID] = emissionProb

				states = append(states, prevObservation.State[j].StateID)
			}

			viterbi.StartWithInitialStateProbabilities(int(prevObservation.Observation.ID), states, emissionProbMatrix)

			processedObsCount++
			startNewViterbi = false
			// harus masih pakai prevObservation sebelumnya
			continue
		} else if !startNewViterbi {
			// calculate linear distance between prev observation and current observation
			linearDistance := geo.CalculateHaversineDistance(prevObservation.Observation.Lat, prevObservation.Observation.Lon,
				gps[i].Observation.Lat, gps[i].Observation.Lon) * 1000

			for j := 0; j < len(prevObservation.State); j++ {

				for k := 0; k < len(gps[i].State); k++ {

					// for every state between 2 adjacent gps observation, calculate the route length

					prevState := prevObservation.State[j]
					currentState := gps[i].State[k]

					// if state is virtual node, then only consider the virtual edge
					// if not, consider all edges of the state graph node
					sourceEdgeFilter := func(edge datastructure.EdgeCH) bool {
						return true
					}

					if prevState.GetType() == datastructure.VirtualNode {
						prevStatef := prevState
						sourceEdgeFilter = func(edge datastructure.EdgeCH) bool {
							return edge.EdgeID == prevStatef.OutgoingVirtualEdgeID
						}
					}

					targetEdgeFilter := func(edge datastructure.EdgeCH) bool {
						return true
					}

					if currentState.GetType() == datastructure.VirtualNode {
						currentStatef := currentState
						targetEdgeFilter = func(edge datastructure.EdgeCH) bool {
							return edge.EdgeID == currentStatef.IncomingVirtualEdgeID
						}
					}

					// calculate route shortest path distance between prev state and current state
					path, _, _, routeDist := routeAlgo.ShortestPathBiDijkstra(prevState.ProjectionID, currentState.ProjectionID, sourceEdgeFilter,
						targetEdgeFilter)
					// BUG: antara hasil query nearby road segment jelek / beberapa virtual node di graph yang seharusnya connected tapi tidak?.
					// jadi previous valid state yang probnya != -inf , gak ada lanjutan path nya ke current states..
					//  tapi pas di debug, result viterbi state yang prob != -inf salah kalau dibandingin sama ground truthnya
					// call queryGraph.GetOutEdge(queryGraph.GetNodeFirstOutEdges(state.EdgeFromNodeID)[0])
					// di edge yang benar di groound truth dapat -inf prob (route not found from prevState)
					//&& math.Abs(routeDist-linearDistance) < maximumTransitionDistance
					routeDist *= 1000
					if routeDist != -1000 {
						statePairCount++
						// not found a path from previous state edge to current state edge
						// better give very small transition probability

						transitionProb := computeTransitionLogProb(routeDist,
							linearDistance)

						transition := Transition{From: prevState.StateID, To: currentState.StateID}

						transitionProbMatrix[transition] = transitionProb

						if _, ok := routePath[prevState.StateID]; !ok {
							routePath[prevState.StateID] = make(map[int][]datastructure.Coordinate)
						}
						routePath[prevState.StateID][currentState.StateID] = path

					}
				}
			}

			err := viterbi.NextStep(int(gps[i].Observation.ID), states, emissionProbMatrix, transitionProbMatrix, nil)
			if err != nil {
				log.Println(viterbi.MessageHistoryString())
				viterbiResetCount++
				path := viterbi.ComputeMostLikelySequence()
				if len(path) > 1 {
					for i := 0; i < len(path)-1; i++ {
						p := path[i]
						statesPath = append(statesPath, p.State)
						obs := gps[p.Observation]
						observationPath = append(observationPath, datastructure.NewCoordinate(obs.Observation.Lat, obs.Observation.Lon))
					}
				}

				viterbi = NewViterbiAlgorithm(true)
				startNewViterbi = true

				log.Printf("broken viterbi at observation: %v", i)

				if (processedObsCount+1)%(int(0.1*float64(len(gps)))) == 0 {
					log.Printf("Processed %v out of %v observations", processedObsCount+1, len(gps))
				}
				prevObservation = prevPrevObservation

				prevPrevObsID := util.BinarySearch(gps, prevObservation, func(a, b datastructure.StateObservationPair) int {
					if a.Observation.ID < b.Observation.ID {
						return -1
					} else if a.Observation.ID > b.Observation.ID {
						return 1
					}
					return 0
				})

				prevPrevObservation = gps[prevPrevObsID-1]

				continue
			} else {
				processedObsCount++
			}

		}

		prevPrevObservation = prevObservation
		prevObservation = gps[i]

		startNewViterbi = false

		if (processedObsCount+1)%(int(0.1*float64(len(gps)))) == 0 {
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
	solutionEdges := make([]datastructure.EdgeCH, 0)

	for i := 1; i < len(statesPath); i++ {

		projectionLoc := stateDataMap[statesPath[i]].ProjectionLoc
		solutions = append(solutions, datastructure.Coordinate{
			Lat: projectionLoc[0],
			Lon: projectionLoc[1]})

		solutionEdges = append(solutionEdges, queryGraph.GetOutEdge(stateDataMap[statesPath[i]].EdgeID))
	}

	log.Printf("Viterbi reset count %v", viterbiResetCount)
	log.Printf("Processed %v out of %v observations", processedObsCount, len(gps))
	log.Printf("Found %v state pairs", statePairCount)

	return solutions, solutionEdges, observationPath
}

func (hmm *HMMMapMatching) buildQueryGraph(gps []datastructure.StateObservationPair) *contractor.ContractedGraph {
	// copy query graph. slice is passed by reference
	qOutEdges := make([]datastructure.EdgeCH, len(hmm.ch.GetOutEdges()))
	copy(qOutEdges, hmm.ch.GetOutEdges())
	qInEdges := make([]datastructure.EdgeCH, len(hmm.ch.GetInEdges()))
	copy(qInEdges, hmm.ch.GetInEdges())
	qNodes := make([]datastructure.CHNode, len(hmm.ch.GetNodes()))
	copy(qNodes, hmm.ch.GetNodes())
	qFirstOutEdges := make([][]int32, len(hmm.ch.GetFirstOutEdges()))
	copy(qFirstOutEdges, hmm.ch.GetFirstOutEdges())
	qFirstInEdges := make([][]int32, len(hmm.ch.GetFirstInEdges()))
	copy(qFirstInEdges, hmm.ch.GetFirstInEdges())

	queryGraph := &contractor.ContractedGraph{
		ContractedOutEdges:     qOutEdges,
		ContractedInEdges:      qInEdges,
		ContractedNodes:        qNodes,
		ContractedFirstOutEdge: qFirstOutEdges,
		ContractedFirstInEdge:  qFirstInEdges,
	}

	hmm.splitEdges(gps, queryGraph)

	return queryGraph
}

func (hmm *HMMMapMatching) splitEdges(gps []datastructure.StateObservationPair, queryGraph ContractedGraph) error {

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

	for edgeID, edgeSnaps := range edgeSnapMap {
		// for all <edgeID, []snap> create virtual edge & virtual node between edge.
		// edge := queryGraph.GetOutEdge(edgeID)
		edge, err := hmm.db.GetMapMatchWay(edgeID)
		if err != nil {
			return err
		}
		pointsInBetween := edge.PointsInBetween

		fromNode := queryGraph.GetNode(edge.FromNodeID)
		toNode := queryGraph.GetNode(edge.ToNodeID)

		snapIndexInPoints := make(map[int]int)

		// sort snaps based on their projection place in line points.

		// sort based only distance from sourceNode give bad result (lot of viterbi break)

		util.QuickSort(edgeSnaps, 0, len(edgeSnaps)-1, func(a, b *datastructure.State) int {

			if a.ObservationID < b.ObservationID {
				return -1
			} else if a.ObservationID > b.ObservationID {
				return 1
			} else {

				return 0
			}

		})

		// add virtual edges to the graph

		prevPoint := pointsInBetween[0]
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
					false,
				)
				edgeSnaps[i].ProjectionID = newNodeID

				queryGraph.AddNode(newProjectionNode)
			} else {
				log.Printf("node already exist")
			}

			// add virtual edge
			addVirtualEdge(queryGraph, edge, prevNodeID, newNodeID,
				prevPoint, currSnapProjection, prevPositionInLinePoints,
				snapIndexInPoints[edgeSnaps[i].StateID])

			prevNodeID = newNodeID
			prevPoint = currSnapProjection
			prevPositionInLinePoints = snapIndexInPoints[edgeSnaps[i].StateID]

		}

		// add <finalProjection, toNode> virtual edge
		toNodeCoordinate := datastructure.NewCoordinate(toNode.Lat, toNode.Lon)
		addVirtualEdge(queryGraph, edge, prevNodeID, toNode.ID,
			prevPoint, toNodeCoordinate, snapIndexInPoints[edgeSnaps[len(edgeSnaps)-1].StateID],
			len(pointsInBetween))

		edgeSnaps[len(edgeSnaps)-1].ProjectionID = prevNodeID

		// remove old edge (from->to) from query graph
		// queryGraph.RemoveEdge(edgeID, fromNode.ID, toNode.ID)
	}

	return nil
}

func addVirtualEdge(queryGraph ContractedGraph,
	matchedEdge datastructure.MapMatchOsmWay, prevNodeID, nodeID int32, prevProjection, currProjection datastructure.Coordinate,
	prevPositionInLinePoints, currPositionInLinePoints int) int32 {
	pointsInBetweenNewEdge := make([]datastructure.Coordinate, 0)

	// add points in between source and target of new edge
	pointsInBetweenNewEdge = append(pointsInBetweenNewEdge, datastructure.NewCoordinate(prevProjection.Lat, prevProjection.Lon))

	for i := prevPositionInLinePoints + 1; i < currPositionInLinePoints; i++ {
		pointsInBetweenNewEdge = append(pointsInBetweenNewEdge, matchedEdge.PointsInBetween[i])
	}
	pointsInBetweenNewEdge = append(pointsInBetweenNewEdge, datastructure.NewCoordinate(currProjection.Lat, currProjection.Lon))

	// calculate edge distance
	dist := 0.0
	for i := 0; i < len(pointsInBetweenNewEdge)-1; i++ {
		dist += geo.CalculateHaversineDistance(pointsInBetweenNewEdge[i].Lat, pointsInBetweenNewEdge[i].Lon,
			pointsInBetweenNewEdge[i+1].Lat, pointsInBetweenNewEdge[i+1].Lon)
	}
	dist *= 1000 // meter

	speed := matchedEdge.Speed // meter/minutes
	eta := dist / speed

	// add new edge to graph
	newEdgeID := int32(len(queryGraph.GetOutEdges()))
	newEdge := datastructure.NewEdgeCH(newEdgeID, eta, dist, nodeID, prevNodeID, false, -1, -1, 0,
		false, 0, 0, 0, pointsInBetweenNewEdge)

	queryGraph.AddEdge(newEdge)

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
				// newState := *state
				// newState.SetIncomingVirtualEdge(virtualOutgouingEdge)
				// newState.SetOutgoingVirtualEdge(virtualIncomingEdge)
				// newState.StateID = nextStateID
				// nextStateID++

				// gps[i].State = append(gps[i].State, &newState)
			}
		}
	}
}

// getProjectionPlaceInLinePoints get the projection place of each snap point in line points
// this is not used. the result of this function is so bad.
func getProjectionPlaceInLinePoints(linePoints []datastructure.Coordinate, edgeSnaps []*datastructure.State, snappedIndexInPoints map[int]int) {
	for i := 0; i < len(edgeSnaps); i++ {
		position := geo.PointPositionBetweenLinePoints(edgeSnaps[i].ProjectionLoc[0], edgeSnaps[i].ProjectionLoc[1], linePoints)
		snappedIndexInPoints[edgeSnaps[i].StateID] = position
	}
}
