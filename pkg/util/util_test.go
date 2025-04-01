package util

import (
	"encoding/binary"
	"fmt"
	"testing"
)

func TestQuickSort(t *testing.T) {

	arr := []int{4, 3, 2, 1, 10, 5555, -1, 20, 100, -100}
	arr = QuickSortG(arr, func(a, b int) int {
		if a < b {
			return -1
		} else if a > b {
			return 1
		} else {
			return 0
		}
	})

	for i := 0; i < len(arr); i++ {
		if i == 0 {
			continue
		}
		if arr[i] < arr[i-1] {
			t.Errorf("Error in sorting")
		}
	}
}

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
	fmt.Printf("Bitpack isShortcut: %t\n", isShortcut)

	bitpack, isRoundabout := BitUnpackIntBool(int32(binary.LittleEndian.Uint32(buf[4:8])), 20)
	_ = bitpack
	fmt.Printf("Bitpack isRoundabout: %t\n", isRoundabout)

	_, trafficLight := BitUnpackIntBool(int32(binary.LittleEndian.Uint32(buf[4:8])), 31)
	fmt.Printf("Bitpack trafficLight: %t\n", trafficLight)
}
