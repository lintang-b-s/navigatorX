package concurrent

import (
	"context"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
)

type KVEdge struct {
	CenterLoc           []float64 // [lat, lon]
	IntersectionNodesID []int32
	PointsInBetween     []Coordinate
	WayID               int32
	FromNodeID          int32
	ToNodeID            int32
}
type SaveWayJobItem struct {
	KeyStr string
	ValArr []KVEdge
}

type RouteAlgorithm interface {
	ShortestPathBiDijkstra(from, to int32, fromEdgeFilter, toEdgeFilter func(edge datastructure.Edge) bool) ([]datastructure.Coordinate, []datastructure.Edge,
		float64, float64)
}

type CalculateTransitionProbParam struct {
	PrevObservation   datastructure.StateObservationPair
	Gps               []datastructure.StateObservationPair
	I                 int
	J                 int
	K                 int
	LinearDistance    float64
	MaxTransitionDist float64
	RouteAlgo         RouteAlgorithm
	StatePairCount    int
}

func NewCalculateTransitionProbParam(prevObservation datastructure.StateObservationPair, gps []datastructure.StateObservationPair, i, j, k, statePairCount int, linearDistance, maxTransitionDist float64, routeAlgo RouteAlgorithm) CalculateTransitionProbParam {
	return CalculateTransitionProbParam{
		PrevObservation: prevObservation,
		Gps:             gps,

		I:                 i,
		J:                 j,
		K:                 k,
		LinearDistance:    linearDistance,
		MaxTransitionDist: maxTransitionDist,
		RouteAlgo:         routeAlgo,
		StatePairCount:    statePairCount,
	}
}

type AlternativeRouteParam struct {
	FromNodeID        int32
	ToNodeID          int32
	VNodeID           int32
	OptNodes          []datastructure.CHNode
	AlternativeRoute  datastructure.AlternativeRouteInfo
	OptDist           float64
	Ctx               context.Context
	AlternativeRoutes *[]datastructure.AlternativeRouteInfo
}

func NewAlternativeRouteParam(fromNodeID, toNodeID, vNodeID int32, optNodes []datastructure.CHNode,
	altRoute datastructure.AlternativeRouteInfo, optDist float64, ctx context.Context,
	alts *[]datastructure.AlternativeRouteInfo) AlternativeRouteParam {
	return AlternativeRouteParam{
		FromNodeID:        fromNodeID,
		ToNodeID:          toNodeID,
		VNodeID:           vNodeID,
		OptNodes:          optNodes,
		AlternativeRoute:  altRoute,
		OptDist:           optDist,
		Ctx:               ctx,
		AlternativeRoutes: alts,
	}
}

type JobI interface {
	[]int32 | SaveWayJobItem | []KVEdge | CalculateTransitionProbParam | AlternativeRouteParam
}

type Job[T JobI] struct {
	ID      int
	JobItem T
}
type JobFunc[T JobI, G any] func(job T) G

type Coordinate struct {
	Lat float64
	Lon float64
}

func NewCoordinate(lat, lon float64) Coordinate {
	return Coordinate{
		Lat: lat,
		Lon: lon,
	}
}

func NewCoordinates(lat, lon []float64) []Coordinate {
	coords := make([]Coordinate, len(lat))
	for i := range lat {
		coords[i] = NewCoordinate(lat[i], lon[i])
	}
	return coords
}
