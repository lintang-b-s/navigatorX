package geo

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/util"

	"github.com/golang/geo/s2"
)

type Coordinate struct {
	Lat float64
	Lon float64
}

func NewCoordinate(lat, lon float64) Coordinate {
	return Coordinate{Lat: lat, Lon: lon}
}

func ProjectPointToLineCoord(nearestStPoint Coordinate, secondNearestStPoint Coordinate,
	snap Coordinate) Coordinate {
	nearestStPoint = MakeSixDigitsAfterComa2(nearestStPoint, 6)
	secondNearestStPoint = MakeSixDigitsAfterComa2(secondNearestStPoint, 6)
	snapLat := snap.Lat
	snapLon := snap.Lon
	MakeSixDigitsAfterComaLatLon(&snapLat, &snapLon, 6)

	nearestStS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(nearestStPoint.Lat, nearestStPoint.Lon))
	secondNearestStS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(secondNearestStPoint.Lat, secondNearestStPoint.Lon))
	snapS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(snapLat, snapLon))
	projection := s2.Project(snapS2, nearestStS2, secondNearestStS2)
	projectLatLng := s2.LatLngFromPoint(projection)
	return Coordinate{projectLatLng.Lat.Degrees(), projectLatLng.Lng.Degrees()}
}

const (
	tolerancePointInLine = 1e-3
)

// IsPointBetweenLine checks if a point is between two points
// lat,lon is the projection of query point to the line
// lats, lons are the line points
func PointPositionBetweenLinePoints(lat, lon float64, linePoints []datastructure.Coordinate) int {
	minDiff := math.MaxFloat64
	var pos int
	for i := 0; i < len(linePoints)-1; i++ {

		currQueryDist := s2.LatLngFromDegrees(lat, lon).Distance(s2.LatLngFromDegrees(linePoints[i].Lat, linePoints[i].Lon)).Radians()
		nextQueryDist := s2.LatLngFromDegrees(lat, lon).Distance(s2.LatLngFromDegrees(linePoints[i+1].Lat, linePoints[i+1].Lon)).Radians()

		currNextDist := s2.LatLngFromDegrees(linePoints[i].Lat, linePoints[i].Lon).Distance(s2.LatLngFromDegrees(linePoints[i+1].Lat, linePoints[i+1].Lon)).Radians()

		diff := math.Abs(currQueryDist + nextQueryDist - currNextDist)
		if diff < tolerancePointInLine && diff < minDiff {
			minDiff = diff
			pos = i + 1
		}
	}
	return pos
}

func MakeSixDigitsAfterComa2(n Coordinate, precision int) Coordinate {

	if util.CountDecimalPlacesF64(n.Lat) != precision {
		n.Lat = util.RoundFloat(n.Lat+0.000001, 6)
	}
	if util.CountDecimalPlacesF64(n.Lon) != precision {
		n.Lon = util.RoundFloat(n.Lon+0.000001, 6)
	}
	return n
}
func MakeSixDigitsAfterComaLatLon(lat, lon *float64, precision int) {

	if util.CountDecimalPlacesF64(*lat) != precision {
		*lat = util.RoundFloat(*lat+0.000001, 6)
	}
	if util.CountDecimalPlacesF64(*lon) != precision {
		*lon = util.RoundFloat(*lon+0.000001, 6)
	}
}
