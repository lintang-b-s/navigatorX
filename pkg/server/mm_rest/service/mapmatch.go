package service

import (
	"context"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"log"
)

type RouteAlgorithm interface {
	ShortestPathAStar(int32, int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64)
	ShortestPathBiDijkstraCH(from, to int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64)
}

type KVDB interface {
	GetNearestStreetsFromPointCoord(lat, lon float64) ([]datastructure.KVEdge, error)
}

type Matching interface {
	MapMatch(gps []datastructure.StateObservationPair, nextStateID int) ([]datastructure.Coordinate, []datastructure.EdgeCH, []datastructure.Coordinate)
}

type RoadSnapper interface {
	SnapToRoads(p datastructure.Point) []datastructure.OSMObject
	SnapToRoadsWithinRadius(p datastructure.Point, radius float64, k int) []datastructure.OSMObject
}
type ContractedGraph interface {
	SnapLocationToRoadNetworkNodeH3(ways []datastructure.KVEdge, wantToSnap []float64) int32

	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32
	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.EdgeCH
	GetInEdge(edgeID int32) datastructure.EdgeCH
	GetOutEdges() []datastructure.EdgeCH
	GetInEdges() []datastructure.EdgeCH
	GetNumNodes() int
	Contraction() (err error)
	SetCHReady()
	SaveToFile() error
	LoadGraph() error
	GetStreetDirection(streetName int) [2]bool
	GetStreetNameFromID(streetName int) string
	GetRoadClassFromID(roadClass int) string
	GetRoadClassLinkFromID(roadClassLink int) string

	GetEdgeExtraInfo(edgeID int) datastructure.EdgeExtraInfo
}

type MapMatchingService struct {
	mapMatching Matching
	roadSnapper RoadSnapper
	kv          KVDB
	ch          ContractedGraph
}

func NewMapMatchingService(mapMatching Matching, rs RoadSnapper, kv KVDB, ch ContractedGraph) *MapMatchingService {
	return &MapMatchingService{mapMatching: mapMatching, roadSnapper: rs, kv: kv, ch: ch}
}

const (
	distThresold = 4.07
)

func (uc *MapMatchingService) MapMatch(ctx context.Context, gps []datastructure.Coordinate) (string,
	[]datastructure.Coordinate, []datastructure.EdgeCH, []datastructure.Coordinate, error) {
	hmmPair := []datastructure.StateObservationPair{}

	stateID := 0
	obsID := 0

	obsWithLotNearbyEdges := 0
	obsWithEmptyNearbyEdges := 0

	prevGpsPoint := gps[0]
	for i := 0; i < len(gps); i++ {
		gpsPoint := gps[i]

		//  preprocessing , buang gps points yang jaraknya kurang dari 2*distThresold meter dari previous gps point

		if i == 0 || i == len(gps)-1 || geo.CalculateHaversineDistance(prevGpsPoint.Lat, prevGpsPoint.Lon,
			gpsPoint.Lat, gpsPoint.Lon)*1000 > 2*distThresold {

			nearestEdges := uc.NearestRoadSegmentsForMapMatching(gpsPoint.Lat, gpsPoint.Lon, obsID)

			if len(nearestEdges) == 0 {
				obsWithEmptyNearbyEdges++
				continue
			}

			if len(nearestEdges) > 20 {
				obsWithLotNearbyEdges++
			}

			for j := range nearestEdges {
				nearestEdges[j].StateID = stateID

				stateID++
			}

			chNodeGPS := datastructure.NewCHNode(gpsPoint.Lat, gpsPoint.Lon, 0, int32(obsID), false)
			obsID++
			hmmPair = append(hmmPair, datastructure.StateObservationPair{
				Observation: chNodeGPS,
				State:       nearestEdges,
			})
			prevGpsPoint = gpsPoint
		}

	}

	log.Printf("obs with lot nearby edges: %d\n", obsWithLotNearbyEdges)
	log.Printf("obs with empty nearby edges: %d\n", obsWithEmptyNearbyEdges)

	path, edges, obsPath := uc.mapMatching.MapMatch(hmmPair, stateID)

	return datastructure.CreatePolyline(path), path, edges, obsPath, nil
}

func (uc *MapMatchingService) NearestRoadSegmentsForMapMatching(lat, lon float64, obsID int) []*datastructure.State {
	nearestEdges := uc.roadSnapper.SnapToRoads(datastructure.Point{Lat: lat, Lon: lon})

	stsData := make([]*datastructure.State, 0, len(nearestEdges))
	for _, roadSegment := range nearestEdges {

		dist := geo.CalculateHaversineDistance(lat, lon, roadSegment.Lat, roadSegment.Lon) * 1000
		edgeID := int32(roadSegment.ID)

		edge := uc.ch.GetOutEdge(edgeID)
		pointsInBetween := uc.ch.GetEdgeExtraInfo(int(edgeID)).PointsInBetween

		stsData = append(stsData, &datastructure.State{

			Dist:            dist,
			EdgeID:          edgeID, // edgeID sama edgeID di outEdge chGraph beda... how ????
			PointsInBetween: pointsInBetween,
			EdgeFromNodeID:  edge.FromNodeID,
			EdgeToNodeID:    edge.ToNodeID,
			StreetName:      roadSegment.Tag[1],
			RoadClass:       roadSegment.Tag[2],
			ProjectionID:    -1,
			Type:            datastructure.VirtualNode, // by default all virtual node
			ObservationID:   obsID,
		})

	}

	return stsData
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

		pointsInBetween := uc.ch.GetEdgeExtraInfo(int(edgeID)).PointsInBetween
		fromNodeLoc := geo.NewCoordinate(pointsInBetween[0].Lat, pointsInBetween[0].Lon)
		toNodeLoc := geo.NewCoordinate(pointsInBetween[len(pointsInBetween)-1].Lat, pointsInBetween[len(pointsInBetween)-1].Lon)
		projection := geo.ProjectPointToLineCoord(fromNodeLoc, toNodeLoc, gpsLoc)

		dists = append(dists, geo.CalculateHaversineDistance(projection.Lat, projection.Lon, roadSegment.Lat, roadSegment.Lon))
	}

	return edges, dists, nil
}

type transition struct {
	from int32
	to   int32
}

type State struct {
	ID              int
	Lat             float64
	Lon             float64
	Dist            float64
	EdgeID          int32
	PointsInBetween []datastructure.Coordinate
}
