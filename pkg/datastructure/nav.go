package datastructure

import (
	"bytes"
	"encoding/binary"
	"math"
)

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

func serializeCoordinates(coords []Coordinate) ([]byte, error) {
	buf := bytes.NewBuffer(make([]byte, 0, 16*len(coords)))
	for _, c := range coords {
		cBuf := make([]byte, 16)
		// lat
		binary.LittleEndian.PutUint64(cBuf[0:8], math.Float64bits(c.Lat))
		// lon
		binary.LittleEndian.PutUint64(cBuf[8:16], math.Float64bits(c.Lon))
		buf.Write(cBuf)
	}

	return buf.Bytes(), nil
}

func deserializeCoordinates(data []byte) ([]Coordinate, error) {

	buf := bytes.NewBuffer(data)
	coords := make([]Coordinate, 0, buf.Len()/16)
	for buf.Len() > 0 {
		cBuf := buf.Next(16)
		lat := math.Float64frombits(binary.LittleEndian.Uint64(cBuf[0:8]))
		lon := math.Float64frombits(binary.LittleEndian.Uint64(cBuf[8:16]))
		coords = append(coords, NewCoordinate(lat, lon))
	}
	return coords, nil
}

func NewCoordinates(lat, lon []float64) []Coordinate {
	coords := make([]Coordinate, len(lat))
	for i := range lat {
		coords[i] = NewCoordinate(lat[i], lon[i])
	}
	return coords
}

type AlternativeRouteInfo struct {
	Nodes          []CHNode
	Path           []Coordinate
	Edges          []Edge
	ObjectiveValue float64

	Eta     float64
	Dist    float64
	ViaNode int32
}

func NewAlternativeRouteInfo(objectiveValue float64,
	viaNode int32) AlternativeRouteInfo {
	return AlternativeRouteInfo{
		ObjectiveValue: objectiveValue,
		ViaNode:        viaNode,
	}
}
