package util

import "testing"

func TestQuickSort(t *testing.T) {

	arr := []int{4, 3, 2, 1, 10, 5555, -1, 20, 100, -100}
	QuickSort(arr, 0, len(arr)-1, func(a, b int) int {
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
