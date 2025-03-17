package geo

import "math"

// haversine distance
const earthRadiusKM = 6371.0

type Location struct {
	Latitude  float64
	Longitude float64
}

func degreeToRadians(angle float64) float64 {
	return angle * (math.Pi / 180.0)
}

func NewLocation(lat_degree float64, long_degree float64) Location {
	return Location{
		Latitude:  degreeToRadians(lat_degree),
		Longitude: degreeToRadians(long_degree),
	}
}

func havFunction(angle_rad float64) float64 {
	return (1 - math.Cos(angle_rad)) / 2.0
}

func havFormula(locationOne Location, locationTwo Location) float64 {
	var latitude_diff float64 = locationOne.Latitude - locationTwo.Latitude
	var longitude_diff float64 = locationOne.Longitude - locationTwo.Longitude

	var hav_latitude float64 = havFunction(latitude_diff)
	var hav_longitude float64 = havFunction(longitude_diff)

	return hav_latitude + math.Cos(locationOne.Latitude)*math.Cos(locationTwo.Latitude)*hav_longitude
}

func archaversine(hav_angle float64) float64 {
	var sqrt_hav_angle float64 = math.Sqrt(hav_angle)
	return 2.0 * math.Asin(sqrt_hav_angle)
}

func HaversineDistance(locationOne Location, locationTwo Location) float64 {
	var hav_central_angle float64 = havFormula(locationOne, locationTwo)
	var central_angle_rad float64 = archaversine(hav_central_angle)
	return earthRadiusKM * central_angle_rad
}

func CalculateHaversineDistance(lat, long, lat2, long2 float64) float64 {
	locationOne := NewLocation(lat, long)
	locationTwo := NewLocation(lat2, long2)
	return HaversineDistance(locationOne, locationTwo)
}
