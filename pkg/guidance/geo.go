package guidance

import (
	"math"
)

//	φ is latitude, λ is longitude
//
// https://www.movable-type.co.uk/scripts/latlong.html
func MidPoint(lat1, lon1 float64, lat2, lon2 float64) (float64, float64) {
	p1LatRad := degToRad(lat1)
	p2LatRad := degToRad(lat2)

	diffLon := degToRad(lon2 - lon1)

	bx := math.Cos(p2LatRad) * math.Cos(diffLon)
	by := math.Cos(p2LatRad) * math.Sin(diffLon)

	newLon := degToRad(lon1) + math.Atan2(by, math.Cos(p1LatRad)+bx)
	newLat := math.Atan2(math.Sin(p1LatRad)+math.Sin(p2LatRad), math.Sqrt((math.Cos(p1LatRad)+bx)*(math.Cos(p1LatRad)+bx)+by*by))

	return radToDeg(newLat), radToDeg(newLon)
}

func degToRad(d float64) float64 {
	return d * math.Pi / 180.0
}

func radToDeg(r float64) float64 {
	return 180.0 * r / math.Pi
}

/*
BearingTo. menghitung sudut initial bearing untuk edge (p1,p2).
https://www.movable-type.co.uk/scripts/latlong.html
*/
func BearingTo(p1Lat, p1Lon, p2Lat, p2Lon float64) float64 {

	dLon := (p2Lon - p1Lon) * math.Pi / 180.0

	lat1 := p1Lat * math.Pi / 180.0
	lat2 := p2Lat * math.Pi / 180.0

	y := math.Sin(dLon) * math.Cos(lat2)
	x := math.Cos(lat1)*math.Sin(lat2) -
		math.Sin(lat1)*math.Cos(lat2)*math.Cos(dLon)
	brng := math.Mod(math.Atan2(y, x)*180.0/math.Pi+360, 360.0)

	return brng
}
