package kv

import (
	"lintang/navigatorx/pkg/datastructure"

	"github.com/kelindar/binary"
)

func encodeEdges(sw []datastructure.KVEdge) ([]byte, error) {
	bb := encode(sw)

	bbCompressed := bb

	return bbCompressed, nil
}

func loadEdges(bbCompressed []byte) ([]datastructure.KVEdge, error) {
	var sw []datastructure.KVEdge

	sw, err := decode(bbCompressed)

	return sw, err
}

func encode(sw []datastructure.KVEdge) []byte {
	encoded, _ := binary.Marshal(sw)
	return encoded
}

func decode(bb []byte) ([]datastructure.KVEdge, error) {
	var ch []datastructure.KVEdge
	binary.Unmarshal(bb, &ch)
	return ch, nil
}
