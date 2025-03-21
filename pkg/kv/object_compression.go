package kv

import (
	"lintang/navigatorx/pkg/datastructure"

	"github.com/kelindar/binary"
)

func encodeWays(sw []SmallWay) ([]byte, error) {
	bb := encode(sw)

	bbCompressed := bb

	return bbCompressed, nil
}

func loadWays(bbCompressed []byte) ([]SmallWay, error) {
	var sw []SmallWay

	sw, err := decode(bbCompressed)

	return sw, err
}

func encodeWay(sw SmallWay) ([]byte, error) {
	bb := encodeOneWay(sw)

	bbCompressed := bb

	return bbCompressed, nil
}
func loadWay(bbCompressed []byte) (SmallWay, error) {
	var sw SmallWay

	sw, err := decodeOneWay(bbCompressed)

	return sw, err
}

func encodeMapMatchWay(way datastructure.MapMatchOsmWay) ([]byte, error) {
	encoded, err := binary.Marshal(way)
	return encoded, err
}

func decodeMapMatchWay(bb []byte) (datastructure.MapMatchOsmWay, error) {
	var way datastructure.MapMatchOsmWay
	err := binary.Unmarshal(bb, &way)
	return way, err
}
