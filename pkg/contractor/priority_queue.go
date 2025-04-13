package contractor

import (
	"errors"
	"math"
)

type Item interface {
	int32
}

type PriorityQueueNode[T Item] struct {
	Rank float64
	Item T
}

// MinHeap binary heap priorityqueue
type MinHeap[T Item] struct {
	heap []PriorityQueueNode[T]
	pos  map[T]int
}

func NewMinHeap[T Item]() *MinHeap[T] {
	return &MinHeap[T]{
		heap: make([]PriorityQueueNode[T], 0),
		pos:  make(map[T]int),
	}
}

// parent get index dari parent
func (h *MinHeap[T]) parent(index int) int {
	return (index - 1) / 2
}

// leftChild get index dari left child
func (h *MinHeap[T]) leftChild(index int) int {
	return 2*index + 1
}

// rightChild get index dari right child
func (h *MinHeap[T]) rightChild(index int) int {
	return 2*index + 2
}

// heapifyUp mempertahankan heap property. check apakah parent dari index lebih besar kalau iya swap, then recursive ke parent.  O(logN) tree height.
func (h *MinHeap[T]) heapifyUp(index int) {
	for index != 0 && h.heap[index].Rank < h.heap[h.parent(index)].Rank {
		h.heap[index], h.heap[h.parent(index)] = h.heap[h.parent(index)], h.heap[index]

		h.pos[h.heap[index].Item] = index
		h.pos[h.heap[h.parent(index)].Item] = h.parent(index)
		index = h.parent(index)
	}
}

// heapifyDown mempertahankan heap property. check apakah nilai salah satu children dari index lebih kecil kalau iya swap, then recursive ke children yang kecil tadi.  O(logN) tree height.
func (h *MinHeap[T]) heapifyDown(index int) {
	smallest := index
	left := h.leftChild(index)
	right := h.rightChild(index)

	if left < len(h.heap) && h.heap[left].Rank < h.heap[smallest].Rank {
		smallest = left
	}
	if right < len(h.heap) && h.heap[right].Rank < h.heap[smallest].Rank {
		smallest = right
	}
	if smallest != index {
		h.heap[index], h.heap[smallest] = h.heap[smallest], h.heap[index]
		h.pos[h.heap[index].Item] = index
		h.pos[h.heap[smallest].Item] = smallest

		h.heapifyDown(smallest)
	}
}

// isEmpty check apakah heap kosong
func (h *MinHeap[T]) isEmpty() bool {
	return len(h.heap) == 0
}

// size ukuran heap
func (h *MinHeap[T]) Size() int {
	return len(h.heap)
}

// getMin mendapatkan nilai minimum dari min-heap (index 0)
func (h *MinHeap[T]) GetMin() (PriorityQueueNode[T], error) {
	if h.isEmpty() {
		return PriorityQueueNode[T]{}, errors.New("heap is empty")
	}
	return h.heap[0], nil
}

func (h *MinHeap[T]) GetMinRank() float64 {
	if h.isEmpty() {
		return math.MaxFloat64
	}
	return h.heap[0].Rank
}

// insert item baru
func (h *MinHeap[T]) Insert(key PriorityQueueNode[T]) {
	h.heap = append(h.heap, key)
	index := h.Size() - 1
	h.pos[key.Item] = index
	h.heapifyUp(index)
}

// extractMin ambil nilai minimum dari min-heap (index 0) & pop dari heap. O(logN), heapifyDown(0) O(logN)
func (h *MinHeap[T]) ExtractMin() (PriorityQueueNode[T], error) {
	if h.isEmpty() {
		return PriorityQueueNode[T]{}, errors.New("heap is empty")
	}
	root := h.heap[0]
	h.heap[0] = h.heap[h.Size()-1]
	h.heap = h.heap[:h.Size()-1]
	h.pos[root.Item] = -1
	h.heapifyDown(0)
	return root, nil
}

// deleteNode delete node specific. O(N) linear search.
func (h *MinHeap[T]) DeleteNode(item PriorityQueueNode[T]) error {
	index := -1
	// Find the index of the node to delete
	for i := 0; i < h.Size(); i++ {
		if i == h.pos[item.Item] {
			index = i
			break
		}
	}
	if index == -1 {
		return errors.New("key not found in the heap")
	}
	// Replace the node with the last element
	h.heap[index] = h.heap[h.Size()-1]
	h.heap = h.heap[:h.Size()-1]
	h.pos[item.Item] = -1
	// Restore heap property
	h.heapifyUp(index)
	h.heapifyDown(index)
	return nil
}

// decreaseKey update Rank dari item min-heap.   O(logN) heapify.
func (h *MinHeap[T]) DecreaseKey(item PriorityQueueNode[T]) error {
	if h.pos[item.Item] < 0 || h.pos[item.Item] >= h.Size() || item.Rank > h.heap[h.pos[item.Item]].Rank {
		return errors.New("invalid index or new value")
	}
	h.heap[h.pos[item.Item]] = item
	h.heapifyUp(h.pos[item.Item])
	return nil
}

func (h *MinHeap[T]) GetItem(item T) PriorityQueueNode[T] {
	return h.heap[h.pos[item]]
}
