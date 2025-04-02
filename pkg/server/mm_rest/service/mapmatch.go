package service

import (
	"context"
	"log"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/geo"
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
	GetEdgePointsInBetween(fromNodeID, toNodeID int32, reverse bool) []datastructure.Coordinate
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
	distThresold = 4.0
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

			chNodeGPS := datastructure.NewCHNode(gpsPoint.Lat, gpsPoint.Lon, 0, int32(obsID))
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
	filteredEdges := uc.FilterEdges(nearestEdges, lat, lon, obsID)

	return filteredEdges
}

const (
	radius = 80.0 // dist(projected, obs) <  80 meter
)

func (uc *MapMatchingService) FilterEdges(edges []datastructure.OSMObject, pLat, pLon float64,
	obsID int) []*datastructure.State {
	// create projection and check if dist(projection, queryPoint) < radius
	filteredEdges := make([]*datastructure.State, 0, len(edges))

	edgeSet := make(map[int32]struct{}, len(edges))

	for _, edge := range edges {

		if _, ok := edgeSet[int32(edge.ID)]; ok {
			continue
		}
		edgeSet[int32(edge.ID)] = struct{}{}

		edgeID := int32(edge.ID)

		edge := uc.ch.GetOutEdge(edgeID)

		pointsInBetween := uc.ch.GetEdgePointsInBetween(edge.FromNodeID, edge.ToNodeID, false)

		pos := geo.PointPositionBetweenLinePoints(pLat, pLon, pointsInBetween) - 1

		fromPoint := datastructure.NewCoordinate(pointsInBetween[pos].Lat, pointsInBetween[pos].Lon)
		toPoint := datastructure.NewCoordinate(pointsInBetween[pos+1].Lat, pointsInBetween[pos+1].Lon)

		projection := geo.ProjectPointToLineCoord(fromPoint, toPoint, datastructure.NewCoordinate(pLat, pLon))

		dist := geo.CalculateHaversineDistance(projection.Lat, projection.Lon, pLat, pLon) * 1000

		if dist < radius {
			filteredEdges = append(filteredEdges, &datastructure.State{
				Dist:            dist,
				EdgeID:          edgeID,
				PointsInBetween: pointsInBetween,
				EdgeFromNodeID:  edge.FromNodeID,
				EdgeToNodeID:    edge.ToNodeID,
				ProjectionID:    -1,
				Type:            datastructure.VirtualNode, // by default all virtual node
				ObservationID:   obsID,
			})
		}
	}

	return filteredEdges
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

		gpsLoc := datastructure.NewCoordinate(lat, lon)

		pointsInBetween := uc.ch.GetEdgePointsInBetween(edge.FromNodeID, edge.ToNodeID, false)
		fromNodeLoc := datastructure.NewCoordinate(pointsInBetween[0].Lat, pointsInBetween[0].Lon)
		toNodeLoc := datastructure.NewCoordinate(pointsInBetween[len(pointsInBetween)-1].Lat, pointsInBetween[len(pointsInBetween)-1].Lon)
		projection := geo.ProjectPointToLineCoord(fromNodeLoc, toNodeLoc, gpsLoc)

		dists = append(dists, geo.CalculateHaversineDistance(projection.Lat, projection.Lon, roadSegment.Lat, roadSegment.Lon))
	}

	return edges, dists, nil
}

type State struct {
	ID              int
	Lat             float64
	Lon             float64
	Dist            float64
	EdgeID          int32
	PointsInBetween []datastructure.Coordinate
}
