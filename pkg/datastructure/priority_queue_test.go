package datastructure

import (
	"testing"

	"golang.org/x/exp/rand"
)

func generateRandomInteger(min int, max int) int {

	return min + rand.Intn(max-min)
}

func TestPriorityQueue(t *testing.T) {
	pq := NewMinHeap()
	if pq == nil {
		t.Errorf("PriorityQueue is nil")
	}

	for i := 0; i < 10000; i++ {
		itemDummy := &RtreeNode{}
		item := PriorityQueueNodeRtree2{Rank: float64(generateRandomInteger(0, 10000)), Item: itemDummy, isObjectBoundingRectangle: false}
		pq.Insert(item)
	}

	prevItem, err := pq.ExtractMin()
	if err != true {
		t.Errorf("Error extract min")
	}

	for i := 1; i < 10000; i++ {
		item, err := pq.ExtractMin()
		if err != true {
			t.Errorf("Error extract min")
		}

		if prevItem.Rank > item.Rank {
			t.Errorf("PriorityQueue is not sorted")
		}
		prevItem = item
	}

}
