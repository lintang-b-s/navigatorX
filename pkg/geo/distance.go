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
