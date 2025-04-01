package datastructure

type Coordinate struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

// 16 byte (128bit)

func NewCoordinate(lat, lon float64) Coordinate {
	return Coordinate{
		Lat: lat,
		Lon: lon,
	}
}

func serializeCoordinates(coords []Coordinate) []byte {

	return createPolylineByteSlice(coords)
}

func deserializeCoordinates(data []byte) ([]Coordinate, error) {
	coords, err := decodePolylineByteSlice(data)
	return coords, err
}

func NewCoordinates(lat, lon []float64) []Coordinate {
	coords := make([]Coordinate, len(lat))
	for i := range lat {
		coords[i] = NewCoordinate(lat[i], lon[i])
	}
	return coords
}
