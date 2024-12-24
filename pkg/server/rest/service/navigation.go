package service

import (
	"context"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"lintang/navigatorx/pkg/guidance"
	"lintang/navigatorx/pkg/server"

	"sync"
)

type ContractedGraph interface {
	SnapLocationToRoadNetworkNodeH3(ways []datastructure.SmallWay, wantToSnap []float64) int32
	SnapLocationToRoadNetworkNodeH3ForMapMatching(ways []datastructure.SmallWay, wantToSnap []float64) []datastructure.State

	IsChReady() bool
	InitCHGraph(nodes []datastructure.Node, edgeCount int, streetDirections map[string][2]bool, sWays, hmmEdges []datastructure.SurakartaWay,
		streetExtraInfo map[string]datastructure.StreetExtraInfo) map[int64]int32
	SetNodeMapIdx(nodeMap map[int64]int32)
	GetFirstOutEdge(nodeIDx int32) []int32
	GetFirstInEdge(nodeIDx int32) []int32
	GetNode(nodeIDx int32) datastructure.CHNode2
	GetOutEdge(edgeIDx int32) datastructure.EdgeCH
	GetInEdge(edgeIDx int32) datastructure.EdgeCH
	GetNumNodes() int
	Contraction() (err error)
	SetCHReady()
	GetStreetDirection(streetName string) [2]bool
	SaveToFile() error
	LoadGraph() error
	GetStreetInfo(streetName string) datastructure.StreetExtraInfo
}

type RoutingAlgorithm interface {
	ShortestPathBiDijkstra(from, to int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64)
	// AStar(from, to int32) (pathN []datastructure.CHNode, path string, eta float64, found bool, dist float64)
	ShortestPathManyToManyBiDijkstraWorkers(from []int32, to []int32) map[int32]map[int32]datastructure.SPSingleResultResult
	CreateDistMatrix(spPair [][]int32) map[int32]map[int32]datastructure.SPSingleResultResult
}

type KVDB interface {
	GetNearestStreetsFromPointCoord(lat, lon float64, hmm bool) ([]datastructure.SmallWay, error)
}

type Matching interface {
	HiddenMarkovModelDecodingMapMatching(gps []datastructure.StateObservationPair) []datastructure.Coordinate
}

type Heuristics interface {
	TravelingSalesmanProblemSimulatedAnnealing(cities []int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64, [][]float64)
	TravelingSalesmanProblemAntColonyOptimization(cities []int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64, [][]float64)
}

type InstructionsFromEdges interface {
	GetDrivingInstructions(path []datastructure.EdgeCH) ([]string, error)
}

type Hungarian interface {
	Solve(original [][]float64) (float64, map[int]int, error)
}
type NavigationService struct {
	CH          ContractedGraph
	KV          KVDB
	hungarian   Hungarian
	routing     RoutingAlgorithm
	mapMatching Matching
	heuristic   Heuristics
}

func NewNavigationService(contractedGraph ContractedGraph, kv KVDB, hung Hungarian, routing RoutingAlgorithm, mapMatching Matching, heu Heuristics) *NavigationService {
	return &NavigationService{CH: contractedGraph, KV: kv, hungarian: hung, routing: routing, mapMatching: mapMatching, heuristic: heu}
}

func (uc *NavigationService) ShortestPathETA(ctx context.Context, srcLat, srcLon float64,
	dstLat float64, dstLon float64) (string, float64, []guidance.DrivingInstruction, bool, []datastructure.Coordinate, float64, bool, error) {

	from := &datastructure.Node{
		Lat: srcLat,
		Lon: srcLon,
	}
	to := &datastructure.Node{
		Lat: dstLat,
		Lon: dstLon,
	}

	var err error
	fromSurakartaNode, err := uc.SnapLocToStreetNode(from.Lat, from.Lon)
	if err != nil {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, false, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
	}
	toSurakartaNode, err := uc.SnapLocToStreetNode(to.Lat, to.Lon)
	if err != nil {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, false, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
	}

	var pN = []datastructure.Coordinate{}
	var p string
	var eta float64
	var found bool
	var dist float64
	var ePath []datastructure.EdgeCH
	if uc.CH.IsChReady() {

		pN, ePath, eta, dist = uc.routing.ShortestPathBiDijkstra(fromSurakartaNode, toSurakartaNode)
		p = datastructure.RenderPath2(pN)
		if eta != -1 {
			found = true
		}
	} else {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, false, server.WrapErrorf(err, server.ErrBadParamInput, "wait until contraction hierarchies preprocessing finish!!")
	}

	if !found {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, false, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
	}
	var route []datastructure.Coordinate = make([]datastructure.Coordinate, 0)

	drivingInstruction := guidance.NewInstructionsFromEdges(uc.CH)
	instructions, err := drivingInstruction.GetDrivingInstructions(ePath)

	if err != nil {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, false, server.WrapErrorf(err, server.ErrInternalServerError, "internal server error")
	}

	return p, dist, instructions, found, route, eta, true, nil
}

func (uc *NavigationService) SnapLocToStreetNode(lat, lon float64) (int32, error) {
	ways, err := uc.KV.GetNearestStreetsFromPointCoord(lat, lon, false)
	if err != nil {
		return 0, err
	}
	streetNodeIDx := uc.CH.SnapLocationToRoadNetworkNodeH3(ways, []float64{lat, lon})

	return streetNodeIDx, nil
}

type ShortestPathResult struct {
	PathsCH []datastructure.Coordinate
	ePath   []datastructure.EdgeCH
	ETA     float64
	Found   bool
	Dist    float64
	Index   int
	IsCH    bool
}

// harusnya pakai algoritma di: https://renatowerneck.wordpress.com/wp-content/uploads/2016/06/adgw13-alternatives.pdf
func (uc *NavigationService) ShortestPathAlternativeStreetETA(ctx context.Context, srcLat, srcLon float64,
	alternativeStreetLat float64, alternativeStreetLon float64,
	dstLat float64, dstLon float64) (string, float64, []guidance.DrivingInstruction, bool, []datastructure.Coordinate, float64, bool, error) {

	from := &datastructure.Node{
		Lat: srcLat,
		Lon: srcLon,
	}

	alternativeStreet := &datastructure.Node{
		Lat: alternativeStreetLat,
		Lon: alternativeStreetLon,
	}

	to := &datastructure.Node{
		Lat: dstLat,
		Lon: dstLon,
	}

	var err error
	fromSurakartaNode, err := uc.SnapLocToStreetNode(from.Lat, from.Lon)
	if err != nil {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, false, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
	}

	alternativeStreetSurakartaNode, err := uc.SnapLocToStreetNode(alternativeStreet.Lat, alternativeStreet.Lon)
	if err != nil {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, false, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
	}

	toSurakartaNode, err := uc.SnapLocToStreetNode(to.Lat, to.Lon)
	if err != nil {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, false, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
	}

	// concurrently find the shortest path dari fromSurakartaNode ke alternativeStreetSurakartaNode
	// dan dari alternativeStreetSurakartaNode ke toSurakartaNode
	var wg sync.WaitGroup

	paths := make([]ShortestPathResult, 2)

	pathChan := make(chan ShortestPathResult, 2)
	wg.Add(1)
	wg.Add(1)

	if !uc.CH.IsChReady() {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, false, server.WrapErrorf(err, server.ErrBadParamInput, "wait until contraction hierarchies preprocessing finish!!")

	}
	go func(wgg *sync.WaitGroup) {

		defer wgg.Done()
		var pN = []datastructure.Coordinate{}
		var eta float64
		var found bool
		var dist float64
		var isCH bool
		var ePath []datastructure.EdgeCH
		if uc.CH.IsChReady() {
			pN, ePath, eta, dist = uc.routing.ShortestPathBiDijkstra(fromSurakartaNode, alternativeStreetSurakartaNode)
			if eta != -1 {
				found = true
			}
			isCH = true
		}
		pathChan <- ShortestPathResult{
			PathsCH: pN,
			ePath:   ePath,
			ETA:     eta,
			Found:   found,
			Dist:    dist,
			Index:   0,
			IsCH:    isCH,
		}
	}(&wg)

	go func(wgg *sync.WaitGroup) {

		defer wgg.Done()
		var pN = []datastructure.Coordinate{}
		var eta float64
		var found bool
		var dist float64
		var isCH bool
		var ePath []datastructure.EdgeCH

		if uc.CH.IsChReady() {
			pN, ePath, eta, dist = uc.routing.ShortestPathBiDijkstra(alternativeStreetSurakartaNode, toSurakartaNode)
			if eta != -1 {
				found = true
			}
			isCH = true
		}
		//  else { di versi sebelumnya masih support A*, habis ganti algoritma driving instruction baru sementara tidak support A*
		// 	ppp, _, eta, found, dist = uc.routing.AStarCH(alternativeStreetSurakartaNode, toSurakartaNode)

		// }
		pathChan <- ShortestPathResult{
			PathsCH: pN,
			ePath:   ePath,
			ETA:     eta,
			Found:   found,
			Dist:    dist,
			Index:   1,
			IsCH:    isCH,
		}

	}(&wg)

	go func() {
		wg.Wait()
		close(pathChan)
	}()

	for p := range pathChan {
		if p.Index == 0 {
			paths[0] = p
		} else {
			paths[1] = p
		}
	}

	// concatedPaths := []datastructure.CHNode{}
	// if !paths[0].IsCH {
	// 	paths[0].Paths = paths[0].Paths[:len(paths[0].Paths)-1] // exclude start node dari paths[1]
	// 	concatedPaths = append(concatedPaths, paths[0].Paths...)
	// 	concatedPaths = append(concatedPaths, paths[1].Paths...)
	// }
	concatedPathsCH := []datastructure.Coordinate{}
	concatedEdgesCH := []datastructure.EdgeCH{}
	paths[0].PathsCH = paths[0].PathsCH[:len(paths[0].PathsCH)-1] // exclude start node dari paths[1]
	concatedPathsCH = append(concatedPathsCH, paths[0].PathsCH...)
	concatedPathsCH = append(concatedPathsCH, paths[1].PathsCH...)

	concatedEdgesCH = append(concatedEdgesCH, paths[0].ePath...)
	concatedEdgesCH = append(concatedEdgesCH, paths[1].ePath...)

	eta := paths[0].ETA + paths[1].ETA
	dist := paths[0].Dist + paths[1].Dist
	found := paths[0].Found && paths[1].Found
	isCH := paths[0].IsCH
	// eta satuannya minute
	if !found {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, false, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
	}
	var route []datastructure.Coordinate = make([]datastructure.Coordinate, 0)

	navPaths := ""
	navPaths = datastructure.RenderPath2(concatedPathsCH)

	drivingInstruction := guidance.NewInstructionsFromEdges(uc.CH)
	instructions, err := drivingInstruction.GetDrivingInstructions(concatedEdgesCH)

	return navPaths, dist, instructions, found, route, eta, isCH, nil
}

func (uc *NavigationService) HiddenMarkovModelDecodingMapMatching(ctx context.Context, gps []datastructure.Coordinate) (string, []datastructure.Coordinate, error) {
	hmmPair := []datastructure.StateObservationPair{}

	stateID := 0
	for i, gpsPoint := range gps {
		if i < len(gps)-1 && len(gps) > 300 {
			// // preprocessing , buang gps points yang jaraknya lebih dari 2*5 meter dari previous gps point
			//  biar gak terlalu lama viterbinya O(T*|S|^2)
			currGpsLoc := geo.NewLocation(gpsPoint.Lat, gpsPoint.Lon)
			nextGpsLoc := geo.NewLocation(gps[i+1].Lat, gps[i+1].Lon)
			if geo.HaversineDistance(currGpsLoc, nextGpsLoc)*1000 >= 2*5 {
				continue
			}
		}
		nearestRoadNodes, err := uc.NearestStreetNodesForMapMatching(gpsPoint.Lat, gpsPoint.Lon)
		if len(nearestRoadNodes) == 0 {
			continue
		}
		if err != nil {
			return "", []datastructure.Coordinate{}, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
		}
		for i := range nearestRoadNodes {
			nearestRoadNodes[i].ID = stateID

			stateID++
		}

		chNodeGPS := datastructure.CHNode2{
			Lat: gpsPoint.Lat,
			Lon: gpsPoint.Lon,
		}

		hmmPair = append(hmmPair, datastructure.StateObservationPair{
			Observation: chNodeGPS,
			State:       nearestRoadNodes,
		})
	}

	path := uc.mapMatching.HiddenMarkovModelDecodingMapMatching(hmmPair)

	return datastructure.RenderPath2(path), path, nil
}

type TargetResult struct {
	TargetCoord         datastructure.Coordinate
	Path                string
	Dist                float64
	ETA                 float64
	DrivingInstructions []guidance.DrivingInstruction
}

func (uc *NavigationService) ManyToManyQuery(ctx context.Context, sourcesLat, sourcesLon, destsLat, destsLon []float64) (map[datastructure.Coordinate][]TargetResult, error) {
	sources := []int32{}
	dests := []int32{}

	for i := 0; i < len(sourcesLat); i++ {
		srcNode, _ := uc.SnapLocToStreetNode(sourcesLat[i], sourcesLon[i])
		sources = append(sources, srcNode)
	}

	for i := 0; i < len(destsLat); i++ {
		dstNode, _ := uc.SnapLocToStreetNode(destsLat[i], destsLon[i])
		dests = append(dests, dstNode)
	}

	manyToManyRes := make(map[datastructure.Coordinate][]TargetResult)

	scMap := uc.routing.ShortestPathManyToManyBiDijkstraWorkers(sources, dests)

	for i, src := range sources {
		srcCoord := datastructure.Coordinate{
			Lat: sourcesLat[i],
			Lon: sourcesLon[i],
		}

		for j, dest := range dests {
			currPath := datastructure.RenderPath2(scMap[src][dest].Paths)
			currDist := scMap[src][dest].Dist
			currETA := scMap[src][dest].Eta
			curEdgePath := scMap[src][dest].EdgePath
			drivingInstruction := guidance.NewInstructionsFromEdges(uc.CH)
			instructions, err := drivingInstruction.GetDrivingInstructions(curEdgePath)
			if err != nil {
				return manyToManyRes, server.WrapErrorf(err, server.ErrInternalServerError, "internal server error")
			}

			manyToManyRes[srcCoord] = append(manyToManyRes[srcCoord], TargetResult{
				TargetCoord: datastructure.Coordinate{
					Lat: destsLat[j],
					Lon: destsLon[j],
				},
				Path:                currPath,
				Dist:                currDist,
				ETA:                 currETA,
				DrivingInstructions: instructions,
			})
		}

	}
	return manyToManyRes, nil
}

func (uc *NavigationService) TravelingSalesmanProblemSimulatedAnneal(ctx context.Context, citiesLat []float64, citiesLon []float64) ([]datastructure.Coordinate, []guidance.DrivingInstruction, string, float64, float64, error) {

	citiesID := []int32{}
	for i := 0; i < len(citiesLat); i++ {
		cityNode, _ := uc.SnapLocToStreetNode(citiesLat[i], citiesLon[i])
		citiesID = append(citiesID, cityNode)
	}

	tspTourNodes, tspEdgePath, bestETA, bestDistance, bestTourCitiesOrder := uc.heuristic.TravelingSalesmanProblemSimulatedAnnealing(citiesID)
	cititesTour := []datastructure.Coordinate{}
	for i := 0; i < len(bestTourCitiesOrder); i++ {
		cititesTour = append(cititesTour, datastructure.Coordinate{
			Lat: bestTourCitiesOrder[i][0],
			Lon: bestTourCitiesOrder[i][1],
		})
	}
	drivingInstruction := guidance.NewInstructionsFromEdges(uc.CH)
	instructions, err := drivingInstruction.GetDrivingInstructions(tspEdgePath)
	if err != nil {
		return []datastructure.Coordinate{}, []guidance.DrivingInstruction{}, "", 0, 0, server.WrapErrorf(err, server.ErrInternalServerError, "internal server error")
	}
	return cititesTour, instructions, datastructure.RenderPath2(tspTourNodes), bestETA, bestDistance, nil
}

func (uc *NavigationService) TravelingSalesmanProblemAntColonyOptimization(ctx context.Context, citiesLat []float64, citiesLon []float64) ([]datastructure.Coordinate, []guidance.DrivingInstruction, string, float64, float64, error) {

	citiesID := []int32{}
	for i := 0; i < len(citiesLat); i++ {
		cityNode, _ := uc.SnapLocToStreetNode(citiesLat[i], citiesLon[i])
		citiesID = append(citiesID, cityNode)
	}

	tspTourNodes, tspEdgePath, bestETA, bestDistance, bestTourCitiesOrder := uc.heuristic.TravelingSalesmanProblemAntColonyOptimization(citiesID)
	cititesTour := []datastructure.Coordinate{}
	for i := 0; i < len(bestTourCitiesOrder); i++ {
		cititesTour = append(cititesTour, datastructure.Coordinate{
			Lat: bestTourCitiesOrder[i][0],
			Lon: bestTourCitiesOrder[i][1],
		})
	}
	drivingInstruction := guidance.NewInstructionsFromEdges(uc.CH)
	instructions, err := drivingInstruction.GetDrivingInstructions(tspEdgePath)
	if err != nil {
		return []datastructure.Coordinate{}, []guidance.DrivingInstruction{}, "", 0, 0, server.WrapErrorf(err, server.ErrInternalServerError, "internal server error")
	}
	return cititesTour, instructions, datastructure.RenderPath2(tspTourNodes), bestETA, bestDistance, nil
}

func (uc *NavigationService) NearestStreetNodesForMapMatching(lat, lon float64) ([]datastructure.State, error) {
	ways, err := uc.KV.GetNearestStreetsFromPointCoord(lat, lon, true)
	if err != nil {
		return []datastructure.State{}, err
	}
	streetNodes := uc.CH.SnapLocationToRoadNetworkNodeH3ForMapMatching(ways, []float64{lat, lon})

	if err != nil {
		return []datastructure.State{}, err
	}
	return streetNodes, nil
}

type MatchedRiderDriver struct {
	Driver              string
	Rider               string
	ETA                 float64
	DrivingInstructions []guidance.DrivingInstruction
}

type UserStreetNode struct {
	Username     string
	StreetNodeID int32
}

// WeightedBipartiteMatching solve rider driver matching secara optimal
// @riderLatLon: latitude, longitude setiap rider
func (uc *NavigationService) WeightedBipartiteMatching(ctx context.Context, riderLatLon map[string][]float64, driverLatLon map[string][]float64) (matched []MatchedRiderDriver, totEta float64, err error) {
	riderLen := len(riderLatLon)
	distMatrix := make([][]float64, riderLen)
	for i := range distMatrix {
		distMatrix[i] = make([]float64, len(driverLatLon))
	}

	riderNodes := []UserStreetNode{}
	driverNodes := []UserStreetNode{}
	spPair := [][]int32{}
	for riderName, riderLatLon := range riderLatLon {
		var currRider int32
		currRider, err = uc.NodeFinder(riderLatLon[0], riderLatLon[1])
		if err != nil {
			return
		}
		riderNodes = append(riderNodes, UserStreetNode{
			Username:     riderName,
			StreetNodeID: currRider,
		})
	}

	for driverName, driverLatLon := range driverLatLon {
		var currdriver int32
		currdriver, err = uc.NodeFinder(driverLatLon[0], driverLatLon[1])
		if err != nil {
			return
		}
		driverNodes = append(driverNodes, UserStreetNode{
			Username:     driverName,
			StreetNodeID: currdriver,
		})
	}

	for _, rider := range riderNodes {
		for _, driver := range driverNodes {

			spPair = append(spPair, []int32{rider.StreetNodeID, driver.StreetNodeID})
		}
	}

	distMatPair := uc.routing.CreateDistMatrix(spPair)

	for i := 0; i < len(riderNodes); i++ {
		for j := 0; j < len(driverNodes); j++ {
			distMatrix[i][j] = distMatPair[riderNodes[i].StreetNodeID][driverNodes[j].StreetNodeID].Eta
		}
	}

	totEta, matchInt, err := uc.hungarian.Solve(distMatrix)
	if err != nil {
		err = server.WrapErrorf(err, server.ErrBadParamInput, "rider and driver location input cannot be empty!")
		return
	}

	for rider, driver := range matchInt {

		riderName, driverName := riderNodes[rider].Username, driverNodes[driver].Username
		var instructions = []guidance.DrivingInstruction{}
		drivingInstruction := guidance.NewInstructionsFromEdges(uc.CH)
		instructions, _ = drivingInstruction.GetDrivingInstructions(distMatPair[riderNodes[rider].StreetNodeID][driverNodes[driver].StreetNodeID].EdgePath)

		matched = append(matched, MatchedRiderDriver{driverName, riderName, distMatrix[rider][driver], instructions})
	}
	return
}

func (uc *NavigationService) NodeFinder(srcLat, srcLon float64) (int32, error) {
	from := &datastructure.Node{
		Lat: srcLat,
		Lon: srcLon,
	}

	var err error
	fromSurakartaNode, err := uc.SnapLocToStreetNode(from.Lat, from.Lon)
	if err != nil {
		return 0, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
	}

	return fromSurakartaNode, nil
}
