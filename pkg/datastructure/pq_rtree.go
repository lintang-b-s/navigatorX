package datastructure

type PriorityQueueNodeRtree2 struct {
	Rank                      float64
	Item                      BoundedItem
	isObjectBoundingRectangle bool
}

func NewPriorityQueueNodeRtree2(rank float64, item BoundedItem, isMBR bool) PriorityQueueNodeRtree2 {
	return PriorityQueueNodeRtree2{rank, item, isMBR}
}

// MinHeap binary heap priorityqueue
type MinHeap struct {
	heap []PriorityQueueNodeRtree2
}

func NewMinHeap() *MinHeap {
	return &MinHeap{
		heap: make([]PriorityQueueNodeRtree2, 0),
	}
}

// parent get index dari parent
func (h *MinHeap) parent(index int) int {
	return (index - 1) / 2
}

// leftChild get index dari left child
func (h *MinHeap) leftChild(index int) int {
	return 2*index + 1
}

// rightChild get index dari right child
func (h *MinHeap) rightChild(index int) int {
	return 2*index + 2
}

// heapifyUp mempertahankan heap property. check apakah parent dari index lebih besar kalau iya swap, then recursive ke parent.  O(logN) tree height.
func (h *MinHeap) heapifyUp(index int) {
	for index != 0 && h.heap[index].Rank < h.heap[h.parent(index)].Rank {
		h.heap[index], h.heap[h.parent(index)] = h.heap[h.parent(index)], h.heap[index]

		index = h.parent(index)
	}
}

// heapifyDown mempertahankan heap property. check apakah nilai salah satu children dari index lebih kecil kalau iya swap, then recursive ke children yang kecil tadi.  O(logN) tree height.
func (h *MinHeap) heapifyDown(index int) {
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

		h.heapifyDown(smallest)
	}
}

// isEmpty check apakah heap kosong
func (h *MinHeap) isEmpty() bool {
	return len(h.heap) == 0
}

// size ukuran heap
func (h *MinHeap) Size() int {
	return len(h.heap)
}

// getMin mendapatkan nilai minimum dari min-heap (index 0)
func (h *MinHeap) GetMin() (PriorityQueueNodeRtree2, bool) {
	if h.isEmpty() {
		return PriorityQueueNodeRtree2{}, false
	}
	return h.heap[0], true
}

// insert item baru
func (h *MinHeap) Insert(key PriorityQueueNodeRtree2) {
	h.heap = append(h.heap, key)
	index := h.Size() - 1

	parent := (index - 1) / 2
	for ; index != 0 && h.heap[parent].Rank > h.heap[index].Rank; parent = (index - 1) / 2 {
		h.heap[parent], h.heap[index] = h.heap[index], h.heap[parent]
		index = parent
	}
}

// extractMin ambil nilai minimum dari min-heap (index 0) & pop dari heap. O(logN), heapifyDown(0) O(logN)
func (h *MinHeap) ExtractMin() (PriorityQueueNodeRtree2, bool) {
	if h.isEmpty() {
		return PriorityQueueNodeRtree2{}, false
	}
	root := h.heap[0]
	h.heap[0] = h.heap[h.Size()-1]
	h.heap = h.heap[:h.Size()-1]
	index := 0

	for {
		smallest := index
		left := index*2 + 1
		right := index*2 + 2
		if left < len(h.heap) && h.heap[left].Rank <= h.heap[smallest].Rank {
			smallest = left
		}
		if right < len(h.heap) && h.heap[right].Rank <= h.heap[smallest].Rank {
			smallest = right
		}
		if smallest == index {
			break
		}
		h.heap[smallest], h.heap[index] = h.heap[index], h.heap[smallest]
		index = smallest
	}

	return root, true
}
