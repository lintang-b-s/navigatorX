package service

import (
	"context"
	"fmt"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/guidance"
	"github.com/lintang-b-s/navigatorx/pkg/server"

	"sync"
)

type ContractedGraph interface {
	SnapLocationToRoadNetworkNodeH3(edges []datastructure.KVEdge, wantToSnap []float64) int32
	SnapLocationToRoadNetworkNodeH3WithSccAnalysis(edgesFrom, edgesTo []datastructure.KVEdge,
		wantToSnapFrom, wantToSnapTo []float64) (int32, int32, datastructure.Coordinate, datastructure.Coordinate, error)

	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32

	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.Edge
	GetInEdge(edgeID int32) datastructure.Edge
	IsRoundabout(edgeID int32) bool

	IsShortcut(edgeID int32) bool
	GetEdgePointsInBetween(edgeID int32) []datastructure.Coordinate
	IsTrafficLight(nodeID int32) bool

	SaveToFile() error
	LoadGraph() error
	GetStreetDirection(streetName int) [2]bool

	GetEdgeInfo(edgeID int32) datastructure.EdgeExtraInfo

	GetStreetNameFromID(streetName int) string
	GetRoadClassFromID(roadClass uint8) string
	GetRoadClassLinkFromID(roadClassLink uint8) string
}

type RoutingAlgorithm interface {
	ShortestPathBiDijkstraCH(from, to int32) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64)
	ShortestPathManyToManyBiDijkstraWorkers(from []int32, to []int32) map[int32]map[int32]datastructure.SPSingleResultResult
	CreateDistMatrix(spPair [][]int32) map[int32]map[int32]datastructure.SPSingleResultResult
	ShortestPathAStar(from, to int32) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64)
}

type AlternativeRouteXCHV interface {
	RunAlternativeRouteXCHV(from, to int32) ([]datastructure.AlternativeRouteInfo, error)
}

type KVDB interface {
	GetNearestStreetsFromPointCoord(lat, lon float64) ([]datastructure.KVEdge, error)
}

type Heuristics interface {
	TravelingSalesmanProblemSimulatedAnnealing(cities []int32) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64, [][]float64)
	TravelingSalesmanProblemAntColonyOptimization(cities []int32) ([]datastructure.Coordinate, []datastructure.Edge, float64, float64, [][]float64)
}

type InstructionsFromEdges interface {
	GetDrivingInstructions(path []datastructure.Edge) ([]string, error)
}

type Hungarian interface {
	Solve(original [][]float64) (float64, map[int]int, error)
}
type NavigationService struct {
	CH                 ContractedGraph
	KV                 KVDB
	hungarian          Hungarian
	routing            RoutingAlgorithm
	alternativeRouting AlternativeRouteXCHV
	heuristic          Heuristics
}

func NewNavigationService(contractedGraph ContractedGraph, kv KVDB, hung Hungarian, routing RoutingAlgorithm, heu Heuristics,
	arRouting AlternativeRouteXCHV,
) *NavigationService {
	return &NavigationService{CH: contractedGraph, KV: kv, hungarian: hung, routing: routing, heuristic: heu,
		alternativeRouting: arRouting}
}

func (uc *NavigationService) ShortestPathETA(ctx context.Context, srcLat, srcLon float64,
	dstLat float64, dstLon float64) (string, float64, []guidance.DrivingInstruction, bool, []datastructure.Coordinate, float64,
	[]datastructure.Edge, bool, error) {

	from := &datastructure.CHNode{
		Lat: srcLat,
		Lon: srcLon,
	}
	to := &datastructure.CHNode{
		Lat: dstLat,
		Lon: dstLon,
	}

	fromSurakartaNode, toSurakartaNode, projectionFrom, projectionTo, err := uc.SnapLocToStreetNodeWithSccAnalysis(from.Lat, from.Lon, to.Lat, to.Lon)
	if err != nil {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, []datastructure.Edge{}, false,
			server.WrapErrorf(err, server.ErrNotFound, fmt.Sprintf("no path found from %v,%v to %v,%v", from.Lat, from.Lon, to.Lat, to.Lon))
	}

	found := false

	var (
		pN    []datastructure.Coordinate
		ePath []datastructure.Edge
		eta   float64
		dist  float64
	)

	pN, ePath, eta, dist = uc.routing.ShortestPathBiDijkstraCH(fromSurakartaNode, toSurakartaNode)

	
	pN = append([]datastructure.Coordinate{projectionFrom}, pN...)
	pN = append(pN, projectionTo)
	p := datastructure.CreatePolyline(pN)
	if eta != -1 {
		found = true
	}

	if !found {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, []datastructure.Edge{}, false, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
	}
	var route []datastructure.Coordinate = make([]datastructure.Coordinate, 0)

	drivingInstruction := guidance.NewInstructionsFromEdges(uc.CH)
	instructions, err := drivingInstruction.GetDrivingInstructions(ePath)

	if err != nil {
		return "", 0, []guidance.DrivingInstruction{}, false, []datastructure.Coordinate{}, 0.0, []datastructure.Edge{}, false, server.WrapErrorf(err, server.ErrInternalServerError, "internal server error")
	}

	return p, dist, instructions, found, route, eta, ePath, true, nil
}

func (uc *NavigationService) ShortestPathWithAlternativeRoutes(ctx context.Context, srcLat, srcLon float64,
	dstLat float64, dstLon float64) ([]datastructure.AlternativeRouteInfo, []string, [][]guidance.DrivingInstruction, error) {
	from := &datastructure.CHNode{
		Lat: srcLat,
		Lon: srcLon,
	}
	to := &datastructure.CHNode{
		Lat: dstLat,
		Lon: dstLon,
	}

	fromSurakartaNode, toSurakartaNode, projectionFrom, projectionTo, err := uc.SnapLocToStreetNodeWithSccAnalysis(from.Lat, from.Lon, to.Lat, to.Lon)
	if err != nil {
		return make([]datastructure.AlternativeRouteInfo, 0), []string{}, [][]guidance.DrivingInstruction{},
			server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")

	}

	alternativeRoutes, err := uc.alternativeRouting.RunAlternativeRouteXCHV(fromSurakartaNode, toSurakartaNode)
	if err != nil {
		return make([]datastructure.AlternativeRouteInfo, 0), []string{}, [][]guidance.DrivingInstruction{},
			server.WrapErrorf(err, server.ErrNotFound, fmt.Sprintf("no path found from %v,%v to %v,%v", from.Lat, from.Lon, to.Lat, to.Lon))
	}

	routePolylines := make([]string, 0, len(alternativeRoutes))
	for _, route := range alternativeRoutes {

		route.Path = append([]datastructure.Coordinate{projectionFrom}, route.Path...)
		route.Path = append(route.Path, projectionTo)

		routePolylines = append(routePolylines, datastructure.CreatePolyline(route.Path))
	}

	routeDrivingInstructions := make([][]guidance.DrivingInstruction, 0, len(alternativeRoutes))
	for _, route := range alternativeRoutes {
		drivingInstruction := guidance.NewInstructionsFromEdges(uc.CH)
		instructions, err := drivingInstruction.GetDrivingInstructions(route.Edges)
		if err != nil {
			return make([]datastructure.AlternativeRouteInfo, 0), []string{}, [][]guidance.DrivingInstruction{}, server.WrapErrorf(err, server.ErrInternalServerError, "internal server error")
		}
		routeDrivingInstructions = append(routeDrivingInstructions, instructions)
	}

	return alternativeRoutes, routePolylines, routeDrivingInstructions, nil
}

func (uc *NavigationService) SnapLocToStreetNodeWithSccAnalysis(fromLat, fromLon, toLat, toLon float64) (int32, int32, datastructure.Coordinate, datastructure.Coordinate, error) {
	edgesFrom, err := uc.KV.GetNearestStreetsFromPointCoord(fromLat, fromLon)
	if err != nil {
		return -1, -1, datastructure.Coordinate{}, datastructure.Coordinate{}, err
	}

	edgesTo, err := uc.KV.GetNearestStreetsFromPointCoord(toLat, toLon)
	if err != nil {
		return -1, -1, datastructure.Coordinate{}, datastructure.Coordinate{}, err
	}

	edgeFromID, edgeToID, projectionFrom, projectionTo, err := uc.CH.SnapLocationToRoadNetworkNodeH3WithSccAnalysis(edgesFrom, edgesTo,
		[]float64{fromLat, fromLon}, []float64{toLat, toLon})
	if err != nil {
		return -1, -1, datastructure.Coordinate{}, datastructure.Coordinate{}, err
	}

	return edgeFromID, edgeToID, projectionFrom, projectionTo, nil
}

func (uc *NavigationService) SnapLocToStreetNode(lat, lon float64) (int32, error) {
	edges, err := uc.KV.GetNearestStreetsFromPointCoord(lat, lon)
	if err != nil {
		return 0, err
	}
	streetNodeID := uc.CH.SnapLocationToRoadNetworkNodeH3(edges, []float64{lat, lon})

	return streetNodeID, nil
}

type ShortestPathResult struct {
	PathsCH []datastructure.Coordinate
	ePath   []datastructure.Edge
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

	from := &datastructure.CHNode{
		Lat: srcLat,
		Lon: srcLon,
	}

	alternativeStreet := &datastructure.CHNode{
		Lat: alternativeStreetLat,
		Lon: alternativeStreetLon,
	}

	to := &datastructure.CHNode{
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

	go func(wgg *sync.WaitGroup) {

		defer wgg.Done()
		var pN = []datastructure.Coordinate{}
		var eta float64
		var found bool
		var dist float64
		var isCH bool
		var ePath []datastructure.Edge

		pN, ePath, eta, dist = uc.routing.ShortestPathBiDijkstraCH(fromSurakartaNode, alternativeStreetSurakartaNode)

		if err != nil {
			return
		}
		if eta != -1 {
			found = true
		}
		isCH = true

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
		var ePath []datastructure.Edge

		pN, ePath, eta, dist = uc.routing.ShortestPathBiDijkstraCH(alternativeStreetSurakartaNode, toSurakartaNode)

		if err != nil {
			return
		}
		if eta != -1 {
			found = true
		}
		isCH = true

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

	concatedPathsCH := []datastructure.Coordinate{}
	concatedEdgesCH := []datastructure.Edge{}
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
	navPaths = datastructure.CreatePolyline(concatedPathsCH)

	drivingInstruction := guidance.NewInstructionsFromEdges(uc.CH)
	instructions, err := drivingInstruction.GetDrivingInstructions(concatedEdgesCH)

	return navPaths, dist, instructions, found, route, eta, isCH, nil
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
			currPath := datastructure.CreatePolyline(scMap[src][dest].Paths)
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
	return cititesTour, instructions, datastructure.CreatePolyline(tspTourNodes), bestETA, bestDistance, nil
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
	return cititesTour, instructions, datastructure.CreatePolyline(tspTourNodes), bestETA, bestDistance, nil
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
	from := &datastructure.CHNode{
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
