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

func ReverseG[T any](arr []T) {
	for i, j := 0, len(arr)-1; i < j; i, j = i+1, j-1 {
		arr[i], arr[j] = arr[j], arr[i]
	}
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


func QuickSort[T any](arr []T, low, high int, compare func(a, b T) int) {
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
}
