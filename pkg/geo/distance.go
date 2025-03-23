package geo

import "math"

const (
	earthRadiusKM = 6371.0
	earthRadiusM  = 6371007
)

func havFunction(angleRad float64) float64 {
	return (1 - math.Cos(angleRad)) / 2.0
}

func degreeToRadians(angle float64) float64 {
	return angle * (math.Pi / 180.0)
}

// very slow
func CalculateHaversineDistance(latOne, longOne, latTwo, longTwo float64) float64 {
	latOne = degreeToRadians(latOne)
	longOne = degreeToRadians(longOne)
	latTwo = degreeToRadians(latTwo)
	longTwo = degreeToRadians(longTwo)

	a := havFunction(latOne-latTwo) + math.Cos(latOne)*math.Cos(latTwo)*havFunction(longOne-longTwo)
	c := 2.0 * math.Asin(math.Sqrt(a))
	return earthRadiusKM * c
}

func degToRad(d float64) float64 {
	return d * math.Pi / 180.0
}

func radToDeg(r float64) float64 {
	return 180.0 * r / math.Pi
}

// GetDestinationPoint returns the destination point given the starting point, bearing and distance
// dist in km
func GetDestinationPoint(lat1, lon1 float64, bearing float64, dist float64) (float64, float64) {

	dr := dist / earthRadiusKM

	bearing = (bearing * (math.Pi / 180.0))

	lat1 = (lat1 * (math.Pi / 180.0))
	lon1 = (lon1 * (math.Pi / 180.0))

	lat2Part1 := math.Sin(lat1) * math.Cos(dr)
	lat2Part2 := math.Cos(lat1) * math.Sin(dr) * math.Cos(bearing)

	lat2 := math.Asin(lat2Part1 + lat2Part2)

	lon2Part1 := math.Sin(bearing) * math.Sin(dr) * math.Cos(lat1)
	lon2Part2 := math.Cos(dr) - (math.Sin(lat1) * math.Sin(lat2))

	lon2 := lon1 + math.Atan2(lon2Part1, lon2Part2)
	lon2 = math.Mod((lon2+3*math.Pi), (2*math.Pi)) - math.Pi

	lat2 = lat2 * (180.0 / math.Pi)
	lon2 = lon2 * (180.0 / math.Pi)

	return lat2, lon2
}
