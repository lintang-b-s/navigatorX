package util

import (
	"encoding/binary"
	"testing"
)

func TestBitPacking(t *testing.T) {

	var buf [8]byte
	bitpackedEdgeInfoField := int32(125)
	bitpackedEdgeInfoField = BitPackInt(bitpackedEdgeInfoField, int32(4), 8)
	bitpackedEdgeInfoField = BitPackInt(bitpackedEdgeInfoField, int32(8), 14)
	bitpackedEdgeInfoField = BitPackIntBool(bitpackedEdgeInfoField, false, 20)
	bitpackedEdgeInfoField = BitPackIntBool(bitpackedEdgeInfoField, false, 21)
	bitpackedEdgeInfoField = BitPackIntBool(bitpackedEdgeInfoField, true, 31)

	binary.LittleEndian.PutUint32(buf[4:8], uint32(bitpackedEdgeInfoField))

	bitpack, isShortcut := BitUnpackIntBool(int32(binary.LittleEndian.Uint32(buf[4:8])), 21)
	_ = bitpack
	if isShortcut {
		t.Errorf("Bitpack isShortcut: %t\n", isShortcut)
	}

	bitpack, isRoundabout := BitUnpackIntBool(int32(binary.LittleEndian.Uint32(buf[4:8])), 20)
	_ = bitpack
	if isRoundabout {
		t.Errorf("Bitpack isRoundabout: %t\n", isRoundabout)
	}

	_, trafficLight := BitUnpackIntBool(int32(binary.LittleEndian.Uint32(buf[4:8])), 31)
	if !trafficLight {
		t.Errorf("Bitpack trafficLight: %t\n", trafficLight)
	}
}
