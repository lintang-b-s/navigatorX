package kv

import (
	"lintang/navigatorx/pkg/concurrent"

	"github.com/DataDog/zstd"
	"github.com/kelindar/binary"
)

// yang dipakai di road snap cuma intersection node sama centerLoc
type SmallWay struct {
	CenterLoc           []float64 // [lat, lon]
	IntersectionNodesID []int64
	NodesInBetween []Coordinate
	WayID int32
}
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


func (s *SmallWay) toConcurrentWay() concurrent.SmallWay {
	return concurrent.SmallWay{
		CenterLoc:           s.CenterLoc,
		IntersectionNodesID: s.IntersectionNodesID,
	}	
}

func Encode(sw []SmallWay) []byte {
	encoded, _ := binary.Marshal(sw)
	return encoded
}

func Decode(bb []byte) ([]SmallWay, error) {
	var ch []SmallWay
	binary.Unmarshal(bb, &ch)
	return ch, nil
}

func Compress(bb []byte) ([]byte, error) {
	var bbCompressed []byte
	bbCompressed, err := zstd.Compress(bbCompressed, bb)
	if err != nil {
		return []byte{}, err
	}
	return bbCompressed, nil
}

func Decompress(bbCompressed []byte) ([]byte, error) {
	var bb []byte
	bb, err := zstd.Decompress(bb, bbCompressed)
	if err != nil {
		return []byte{}, err
	}

	return bb, nil
}
