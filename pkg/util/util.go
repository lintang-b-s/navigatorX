package util

import (
	"math"
	"strconv"
	"strings"

	"golang.org/x/exp/rand"
)

func RoundFloat(val float64, precision uint) float64 {
	ratio := math.Pow(10, float64(precision))
	return math.Round(val*ratio) / ratio
}

func CountDecimalPlacesF64(value float64) int {
	strValue := strconv.FormatFloat(value, 'f', -1, 64)

	parts := strings.Split(strValue, ".")

	if len(parts) < 2 {
		return 0
	}

	return len(parts[1])
}

func ReverseG[T any](arr []T) []T {
	copyArr := make([]T, len(arr)) // should do on the copy )
	copy(copyArr, arr)
	for i, j := 0, len(copyArr)-1; i < j; i, j = i+1, j-1 {
		copyArr[i], copyArr[j] = copyArr[j], copyArr[i]
	}
	return copyArr
}

func BinarySearch[T any](arr []T, target T, compare func(a, b T) int) int {
	left := 0
	right := len(arr)
	for left <= right {
		mid := left + (right-left)/2
		if compare(arr[mid], target) > 0 {
			right = mid - 1
		} else if compare(arr[mid], target) < 0 {
			left = mid + 1
		} else {
			return mid
		}

	}
	return left
}

func generateRandomInt(min, max int) int {
	return min + rand.Intn(max-min)
}

func QuickSortG[T any](arr []T, compare func(a, b T) int) []T {
	copyArr := make([]T, len(arr)) // should do on the copy )
	copy(copyArr, arr)
	return QuickSort(copyArr, 0, len(arr)-1, compare)
}

func QuickSort[T any](arr []T, low, high int, compare func(a, b T) int) []T {
	if low < high {
		pivotIndex := generateRandomInt(low, high)
		pivotValue := arr[pivotIndex]

		arr[pivotIndex], arr[high] = arr[high], arr[pivotIndex]

		i := low - 1

		for j := low; j < high; j++ {
			if compare(arr[j], pivotValue) < 0 {
				i++
				arr[i], arr[j] = arr[j], arr[i]
			}
		}

		arr[i+1], arr[high] = arr[high], arr[i+1]

		QuickSort(arr, low, i, compare)
		QuickSort(arr, i+2, high, compare)
	}
	return arr
}

func BitPackInt(a int32, b int32, offset int32) int32 {
	return b<<offset | a
}

func BitUnpackInt(packed int32, offset int32) (int32, int32) {
	return packed & bitmask[offset-1], packed >> offset
}

func BitPackInt64(a int64, b int64, offset int) int64 {
	return b<<offset | a
}

func BitUnpackInt64(packed int64, offset int64) (int32, int32) {
	return int32(packed & int64(bitmask[offset-1])), int32(packed >> offset)
}

func BitPackIntBool(a int32, b bool, offset int32) int32 {
	if b {
		return a | 1<<offset
	}
	return a
}

func BitUnpackIntBool(packed int32, offset int32) (int32, bool) {
	return packed & bitmask[offset-1], packed&(1<<offset) != 0
}

var bitmask = []int32{
	0b00000000000000000000000000000000, // 0 bits
	0b00000000000000000000000000000001, // 1 bit
	0b00000000000000000000000000000011, // 2 bits
	0b00000000000000000000000000000111, // 3 bits
	0b00000000000000000000000000001111, // 4 bits
	0b00000000000000000000000000011111, // 5 bits
	0b00000000000000000000000000111111, // 6 bits
	0b00000000000000000000000001111111, // 7 bits
	0b00000000000000000000000011111111, // 8 bits
	0b00000000000000000000000111111111, // 9 bits
	0b00000000000000000000001111111111, // 10 bits
	0b00000000000000000000011111111111, // 11 bits
	0b00000000000000000000111111111111, // 12 bits
	0b00000000000000000001111111111111, // 13 bits
	0b00000000000000000011111111111111, // 14 bits
	0b00000000000000000111111111111111, // 15 bits
	0b00000000000000001111111111111111, // 16 bits
	0b00000000000000011111111111111111, // 17 bits
	0b00000000000000111111111111111111, // 18 bits
	0b00000000000001111111111111111111, // 19 bits
	0b00000000000011111111111111111111, // 20 bits
	0b00000000000111111111111111111111, // 21 bits
	0b00000000001111111111111111111111, // 22 bits
	0b00000000011111111111111111111111, // 23 bits
	0b00000000111111111111111111111111, // 24 bits
	0b00000001111111111111111111111111, // 25 bits
	0b00000011111111111111111111111111, // 26 bits
	0b00000111111111111111111111111111, // 27 bits
	0b00001111111111111111111111111111, // 28 bits
	0b00011111111111111111111111111111, // 29 bits
	0b00111111111111111111111111111111, // 30 bits
	0b01111111111111111111111111111111, // 31 bits
}
