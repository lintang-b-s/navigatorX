package contractor

import (
	"testing"

	"golang.org/x/exp/rand"
)

func generateRandomInteger(min int, max int) int {

	return min + rand.Intn(max-min)
}

func TestPriorityQueue(t *testing.T) {
	pq := NewMinHeap[int32]()
	if pq == nil {
		t.Errorf("PriorityQueue is nil")
	}

	for i := 0; i < 10000; i++ {
		item := PriorityQueueNode[int32]{Rank: float64(generateRandomInteger(0, 10000)), Item: int32(i)}
		pq.Insert(item)

		if (i+1)%100 == 0 {
			prevRank := item.Rank
			item.Rank = float64(generateRandomInteger(0, int(item.Rank)-1))
			_ = prevRank
			err := pq.DecreaseKey(item)
			if err != nil {
				t.Errorf("Error decrease key")
			}
		}
	}

	prevItem, err := pq.ExtractMin()
	if err != nil {
		t.Errorf("Error extract min")
	}
	for i := 1; i < 10000; i++ {
		item, err := pq.ExtractMin()
		if err != nil {
			t.Errorf("Error extract min")
		}

		if prevItem.Rank > item.Rank {
			t.Errorf("PriorityQueue is not sorted")
		}
	}
}
