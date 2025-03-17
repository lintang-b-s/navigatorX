package service

import (
	"context"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"lintang/navigatorx/pkg/server"
	"log"
)

type RouteAlgorithm interface {
	ShortestPathAStar(int32, int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64)
	ShortestPathBiDijkstraCH(from, to int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64)
}

type KVDB interface {
	GetNearestStreetsFromPointCoord(lat, lon float64) ([]datastructure.SmallWay, error)
	GetMapMatchWay(key int32) (datastructure.MapMatchOsmWay, error)
}

type Matching interface {
	HiddenMarkovModelDecodingMapMatching(gps []datastructure.StateObservationPair, nextStateID int) ([]datastructure.Coordinate, []datastructure.EdgeCH, []datastructure.Coordinate)
}

type RoadSnapper interface {
	SnapToRoads(p datastructure.Point) []datastructure.OSMObject
	SnapToRoadsWithinRadius(p datastructure.Point, radius float64, k int) []datastructure.OSMObject
}
type ContractedGraph interface {
	SnapLocationToRoadNetworkNodeH3(ways []datastructure.SmallWay, wantToSnap []float64) int32
	SnapLocationToRoadNetworkNodeH3ForMapMatching(ways []datastructure.SmallWay, wantToSnap []float64) []datastructure.State

	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32
	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.EdgeCH
	GetInEdge(edgeID int32) datastructure.EdgeCH
	GetNumNodes() int
	Contraction() (err error)
	SetCHReady()
	SaveToFile() error
	LoadGraph() error
	GetStreetDirection(streetName int) [2]bool
	GetStreetInfo(streetName int) datastructure.StreetExtraInfo
	GetStreetNameFromID(streetName int) string
	GetRoadClassFromID(roadClass int) string
	GetRoadClassLinkFromID(roadClassLink int) string
}

type MapMatchingService struct {
	mapMatching Matching
	roadSnapper RoadSnapper
	kv          KVDB
	ch          ContractedGraph
	routeAlgo   RouteAlgorithm
}

func NewMapMatchingService(mapMatching Matching, rs RoadSnapper, kv KVDB, ch ContractedGraph, rt RouteAlgorithm) *MapMatchingService {
	return &MapMatchingService{mapMatching: mapMatching, roadSnapper: rs, kv: kv, ch: ch, routeAlgo: rt}
}

const (
	sigmaZ = 8.07
)

func (uc *MapMatchingService) HiddenMarkovModelDecodingMapMatching(ctx context.Context, gps []datastructure.Coordinate) (string,
	[]datastructure.Coordinate, []datastructure.EdgeCH, []datastructure.Coordinate, error) {
	hmmPair := []datastructure.StateObservationPair{}

	stateID := 0
	obsID := 0

	prevGpsPoint := gps[0]
	for i := 0; i < len(gps); i++ {
		gpsPoint := gps[i]
		//  preprocessing , buang gps points yang jaraknya kurang dari 2*sigmaz meter dari previous gps point

		if i == 0 || i == len(gps)-1 || geo.CalculateHaversineDistance(prevGpsPoint.Lat, prevGpsPoint.Lon,
			gpsPoint.Lat, gpsPoint.Lon)*1000 > 2*sigmaZ {

			nearestEdges, err := uc.NearestRoadSegmentsForMapMatching(gpsPoint.Lat, gpsPoint.Lon, obsID)
			if err != nil {
				return "", []datastructure.Coordinate{}, []datastructure.EdgeCH{}, []datastructure.Coordinate{}, server.WrapErrorf(err, server.ErrNotFound, "sorry!! the location you entered is not covered on my map :(, please use diferrent opensteetmap pbf file")
			}
			if len(nearestEdges) == 0 {
				continue
			}

			for j := range nearestEdges {
				nearestEdges[j].StateID = stateID

				stateID++
			}

			checkDuplicateEdge(nearestEdges)

			chNodeGPS := datastructure.NewCHNode(gpsPoint.Lat, gpsPoint.Lon, 0, int32(obsID), false)
			obsID++
			hmmPair = append(hmmPair, datastructure.StateObservationPair{
				Observation: chNodeGPS,
				State:       nearestEdges,
			})
			prevGpsPoint = gpsPoint
		}

	}

	path, edges, obsPath := uc.mapMatching.HiddenMarkovModelDecodingMapMatching(hmmPair, stateID)

	return datastructure.RenderPath2(path), path, edges, obsPath, nil
}

func (uc *MapMatchingService) NearestRoadSegmentsForMapMatching(lat, lon float64, obsID int) ([]*datastructure.State, error) {
	nearestEdges := uc.roadSnapper.SnapToRoads(datastructure.Point{Lat: lat, Lon: lon})

	stsData := make([]*datastructure.State, 0, len(nearestEdges)*2)
	for _, roadSegment := range nearestEdges {

		dist := geo.HaversineDistance(geo.NewLocation(lat, lon), geo.NewLocation(roadSegment.Lat, roadSegment.Lon)) * 1000
		edgeID := int32(roadSegment.ID)

		edge, err := uc.kv.GetMapMatchWay(edgeID)
		if err != nil {
			return nil, err
		}

		stsData = append(stsData, &datastructure.State{
			Lat:             roadSegment.Lat,
			Lon:             roadSegment.Lon,
			Dist:            dist,
			EdgeID:          edgeID, // edgeID sama edgeID di outEdge chGraph beda... how ????
			PointsInBetween: edge.PointsInBetween,
			EdgeFromNodeID:  edge.FromNodeID,
			EdgeToNodeID:    edge.ToNodeID,
			StreetName:      roadSegment.Tag[1],
			RoadClass:       roadSegment.Tag[2],
			ProjectionID:    -1,
			Type:            datastructure.VirtualNode, // by default all virtual node
			ObservationID:   obsID,
		})

	}

	return stsData, nil
}

func (uc *MapMatchingService) NearestRoadSegments(ctx context.Context, lat, lon float64, radius float64, k int) ([]datastructure.EdgeCH, []float64, error) {
	nearestEdges := uc.roadSnapper.SnapToRoadsWithinRadius(datastructure.Point{Lat: lat, Lon: lon}, radius, k)

	selected := make(map[int32]struct{})

	dists := make([]float64, 0, len(nearestEdges))

	edges := make([]datastructure.EdgeCH, 0)
	for _, roadSegment := range nearestEdges {
		if _, ok := selected[int32(roadSegment.ID)]; ok {
			// can be duplicate . because i insert two same edges into r-tree but with different bounding boxes (depending on the baseNode/targetNode of the edge).
			continue
		}

		selected[int32(roadSegment.ID)] = struct{}{}

		edgeID := int32(roadSegment.ID)

		edge := uc.ch.GetOutEdge(edgeID)

		edges = append(edges, edge)

		gpsLoc := geo.NewCoordinate(lat, lon)
		fromNodeLoc := geo.NewCoordinate(edge.PointsInBetween[0].Lat, edge.PointsInBetween[0].Lon)
		toNodeLoc := geo.NewCoordinate(edge.PointsInBetween[len(edge.PointsInBetween)-1].Lat, edge.PointsInBetween[len(edge.PointsInBetween)-1].Lon)
		projection := geo.ProjectPointToLineCoord(fromNodeLoc, toNodeLoc, gpsLoc)
		projectionLoc := geo.NewLocation(projection.Lat, projection.Lon)

		dists = append(dists, geo.HaversineDistance(projectionLoc, geo.NewLocation(roadSegment.Lat, roadSegment.Lon)))
	}

	return edges, dists, nil
}

type transition struct {
	from int32
	to   int32
}

func checkDuplicateEdge(edges []*datastructure.State) {
	set := make(map[int32]struct{})
	setEdge := make(map[transition]struct{})
	for _, edge := range edges {
		if _, ok := set[edge.EdgeID]; ok {
			log.Printf("duplicate edge %d", edge.EdgeID)
		}
		set[edge.EdgeID] = struct{}{}

		if _, ok := setEdge[transition{from: edge.EdgeID, to: edge.EdgeID}]; ok {

			log.Printf("duplicate edge %d", edge.EdgeID)
		}
		setEdge[transition{from: edge.EdgeID, to: edge.EdgeID}] = struct{}{}

	}
}

type State struct {
	ID              int
	Lat             float64
	Lon             float64
	Dist            float64
	EdgeID          int32
	PointsInBetween []datastructure.Coordinate
}
