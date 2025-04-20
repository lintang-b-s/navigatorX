package service

import (
	"context"
	"sort"
	"sync"

	"github.com/lintang-b-s/navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/geo"
)

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

func (uc *MapMatchingService) MapMatch(ctx context.Context, gps []datastructure.Coordinate) (string, []datastructure.Coordinate, []datastructure.Edge, []datastructure.Coordinate, error) {
	hmmPair := []datastructure.StateObservationPair{}

	stateID := 0
	obsID := 0

	filteredGps := uc.filterGPS(gps)

	workers := concurrent.NewWorkerPool[concurrent.SnapGpsToRoadSegmentsParam, snapResult](10,
		len(filteredGps))

	var mu sync.Mutex

	for i := 0; i < len(filteredGps); i++ {
		gpsPoint := filteredGps[i]
		workers.AddJob(concurrent.NewSnapGpsToRoadSegmentsParam(gpsPoint, obsID, &stateID, &mu))
		obsID++
	}

	workers.Close()
	workers.Start(uc.snapGpsToRoadSegment)

	workers.Wait()
	for snapItem := range workers.CollectResults() {
		hmmPair = append(hmmPair, datastructure.StateObservationPair{
			Observation: snapItem.gpsNode,
			State:       snapItem.nearbyEdges,
		})
	}

	sort.Slice(hmmPair, func(i, j int) bool {
		return hmmPair[i].Observation.ID < hmmPair[j].Observation.ID
	})

	path, edges, obsPath := uc.mapMatching.MapMatch(hmmPair, &stateID)

	return datastructure.CreatePolyline(path), path, edges, obsPath, nil
}

type snapResult struct {
	nearbyEdges []*datastructure.State
	gpsNode     datastructure.CHNode
}

func NewSnapResult(nearbyEdges []*datastructure.State, gpsNode datastructure.CHNode) snapResult {
	return snapResult{
		nearbyEdges: nearbyEdges,
		gpsNode:     gpsNode,
	}
}

func (uc *MapMatchingService) snapGpsToRoadSegment(param concurrent.SnapGpsToRoadSegmentsParam) snapResult {
	gpsPoint, obsID, stateID, mu := param.GpsPoint, param.ObsID, param.StateID, param.Mu
	nearestEdges := uc.NearestRoadSegmentsForMapMatching(gpsPoint.Lat, gpsPoint.Lon, obsID)

	mu.Lock()
	defer mu.Unlock()
	for j := range nearestEdges {
		nearestEdges[j].StateID = *stateID

		*stateID++
	}

	chNodeGPS := datastructure.NewCHNode(gpsPoint.Lat, gpsPoint.Lon, 0, int32(obsID))
	return NewSnapResult(nearestEdges, chNodeGPS)
}

func (uc *MapMatchingService) filterGPS(gps []datastructure.Coordinate) []datastructure.Coordinate {
	filteredGps := make([]datastructure.Coordinate, 0, len(gps))
	prevGpsPoint := gps[0]
	for i := 0; i < len(gps); i++ {
		gpsPoint := gps[i]

		//  preprocessing , buang gps points yang jaraknya kurang dari 2*distThresold meter dari previous gps point

		if i == 0 || i == len(gps)-1 || geo.CalculateHaversineDistance(prevGpsPoint.Lat, prevGpsPoint.Lon,
			gpsPoint.Lat, gpsPoint.Lon)*1000 > 2*distThresold {
			filteredGps = append(filteredGps, gpsPoint)
			prevGpsPoint = gpsPoint
		}
	}

	return filteredGps
}

func (uc *MapMatchingService) NearestRoadSegmentsForMapMatching(lat, lon float64, obsID int) []*datastructure.State {
	nearestEdges := uc.roadSnapper.SnapToRoads(datastructure.Point{Lat: lat, Lon: lon})
	filteredEdges := uc.FilterEdges(nearestEdges, lat, lon, obsID)

	return filteredEdges
}

const (
	radius = 150.0
	k      = 30
)

func (uc *MapMatchingService) FilterEdges(edges []datastructure.OSMObject, pLat, pLon float64,
	obsID int) []*datastructure.State {
	// create projection and check if dist(projection, queryPoint) < radius
	filteredEdges := make([]*datastructure.State, 0, len(edges))

	edgeSet := make(map[int32]struct{}, len(edges))

	for _, edgeObj := range edges {

		if _, ok := edgeSet[int32(edgeObj.ID)]; ok {
			continue
		}
		edgeSet[int32(edgeObj.ID)] = struct{}{}

		edgeID := int32(edgeObj.ID)

		edge := uc.ch.GetOutEdge(edgeID)

		pointsInBetween := uc.ch.GetEdgePointsInBetween(edge.EdgeID)

		pos := geo.PointPositionBetweenLinePoints(pLat, pLon, pointsInBetween) - 1

		fromPoint := datastructure.NewCoordinate(pointsInBetween[pos].Lat, pointsInBetween[pos].Lon)
		toPoint := datastructure.NewCoordinate(pointsInBetween[pos+1].Lat, pointsInBetween[pos+1].Lon)

		projection := geo.ProjectPointToLineCoord(fromPoint, toPoint, datastructure.NewCoordinate(pLat, pLon))

		dist := geo.CalculateHaversineDistance(projection.Lat, projection.Lon, pLat, pLon) * 1000

		if dist < radius {
			filteredEdges = append(filteredEdges, &datastructure.State{
				Dist:              dist,
				EdgeID:            edgeID,
				PointsInBetween:   pointsInBetween,
				EdgeFromNodeID:    edge.FromNodeID,
				EdgeToNodeID:      edge.ToNodeID,
				ProjectionID:      -1,
				Type:              datastructure.VirtualNode, // by default all virtual node
				ObservationID:     obsID,
				PerpendicularDist: dist,
			})
		}

	}

	sort.Slice(filteredEdges, func(i, j int) bool {
		return filteredEdges[i].Dist < filteredEdges[j].Dist
	})

	filteredEdges = filteredEdges[:min(len(filteredEdges), k)]

	return filteredEdges
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func (uc *MapMatchingService) NearestRoadSegments(ctx context.Context, lat, lon float64, radius float64, k int) ([]datastructure.Edge, []float64, error) {
	nearestEdges := uc.roadSnapper.SnapToRoadsWithinRadius(datastructure.Point{Lat: lat, Lon: lon}, radius, k)

	dists := make([]float64, 0, len(nearestEdges))

	edges := make([]datastructure.Edge, 0)
	for _, roadSegment := range nearestEdges {

		edgeID := int32(roadSegment.ID)

		edge := uc.ch.GetOutEdge(edgeID)

		edges = append(edges, edge)

		gpsLoc := datastructure.NewCoordinate(lat, lon)

		pointsInBetween := uc.ch.GetEdgePointsInBetween(edge.EdgeID)
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
