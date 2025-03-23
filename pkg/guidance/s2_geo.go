package guidance

import (
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/util"

	"github.com/golang/geo/s2"
)

type Coordinate struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

func MakeSixDigitsAfterComa2(n datastructure.CHNode, precision int) datastructure.CHNode {

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

func ProjectPointToLineCoord(nearestStPoint datastructure.CHNode, secondNearestStPoint datastructure.CHNode,
	snap []float64) Coordinate {
	nearestStPoint = MakeSixDigitsAfterComa2(nearestStPoint, 6)
	secondNearestStPoint = MakeSixDigitsAfterComa2(secondNearestStPoint, 6)
	snapLat := snap[0]
	snapLon := snap[1]
	MakeSixDigitsAfterComaLatLon(&snapLat, &snapLon, 6)

	nearestStS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(nearestStPoint.Lat, nearestStPoint.Lon))
	secondNearestStS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(secondNearestStPoint.Lat, secondNearestStPoint.Lon))
	snapS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(snapLat, snapLon))
	projection := s2.Project(snapS2, nearestStS2, secondNearestStS2)
	projectLatLng := s2.LatLngFromPoint(projection)
	return Coordinate{projectLatLng.Lat.Degrees(), projectLatLng.Lng.Degrees()}
}
