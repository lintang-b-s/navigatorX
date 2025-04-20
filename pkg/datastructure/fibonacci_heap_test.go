package datastructure_test

import (
	"math"
	"testing"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/stretchr/testify/assert"
	"golang.org/x/exp/rand"
)

func generateRandomInteger2(min int, max int) int {

	return min + rand.Intn(max-min)
}

type cobanode struct {
}

func TestFibonaccyHeapInsertExtractMin(t *testing.T) {
	pq := datastructure.NewFibonacciHeap[*cobanode]()
	if pq == nil {
		t.Errorf("PriorityQueue is nil")
	}

	min := math.MaxFloat64
	for i := 0; i < 10000; i++ {
		itemDummy := &cobanode{}
		priority := float64(generateRandomInteger2(0, 10000))
		if priority < min {
			min = priority
		}
		pq.Insert(itemDummy, priority)

		assert.Equal(t, min, pq.GetMin().GetPriority())
	}

	prevItem := pq.ExtractMin()

	for i := 1; i < 10000; i++ {

		item := pq.ExtractMin()

		if prevItem.GetPriority() > item.GetPriority() {
			t.Errorf("PriorityQueue is not sorted")
		}
		prevItem = item
	}

}

func TestFibonaccyHeapInsertDecreaseKey(t *testing.T) {
	pq := datastructure.NewFibonacciHeap[*cobanode]()
	if pq == nil {
		t.Errorf("PriorityQueue is nil")
	}

	itemSlice := make([]*datastructure.Entry[*cobanode], 10000)
	min := math.MaxFloat64
	for i := 0; i < 10000; i++ {
		itemDummy := &cobanode{}
		priority := float64(generateRandomInteger2(1000, 10000000))
		if priority < min {
			min = priority
		}
		curr := pq.Insert(itemDummy, priority)

		assert.Equal(t, min, pq.GetMin().GetPriority())
		itemSlice[i] = curr
	}

	for i := 0; i < 10000; i++ {

		pq.DecreaseKey(itemSlice[i], float64(generateRandomInteger2(0, int(itemSlice[i].GetPriority()))))

	}

	prevItem := pq.ExtractMin()

	for i := 1; i < 10000; i++ {

		item := pq.ExtractMin()

		if prevItem.GetPriority() > item.GetPriority() {
			t.Errorf("PriorityQueue is not sorted")
		}
		prevItem = item
	}

}

// turns out fibonacci heap is much faster than binary heap (68.26 ns/op vs 411 ns/op)
// but when i do load test query sp using k6, p95 binary heap is faster? maybe because allocation/op is smaller? (each fibonacci heap item have 4 pointer)
func BenchmarkFibonacciHeapDecreaseKey(b *testing.B) {
	pq := datastructure.NewFibonacciHeap[*cobanode]()
	if pq == nil {
		b.Errorf("PriorityQueue is nil")
	}

	min := math.MaxFloat64
	for i := 0; i < b.N; i++ {
		itemDummy := &cobanode{}
		priority := float64(generateRandomInteger2(1000, 10000000))
		if priority < min {
			min = priority
		}
		curr := pq.Insert(itemDummy, priority)

		pq.DecreaseKey(curr, float64(generateRandomInteger2(0, int(curr.GetPriority()))))

	}
}
