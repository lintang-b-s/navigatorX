package buffer

import (
	"sync"
)

type ListNode struct {
	Key  int
	next *ListNode
	prev *ListNode
}

func NewListNode(key int, next, prev *ListNode) *ListNode {
	return &ListNode{Key: key, next: next, prev: prev}
}

type DoubleLinkedList struct {
	head *ListNode // most recently used
	tail *ListNode // least recently used
}

// null <--> head <-> tail <-> null
//
//	-> next
//	<- prev
func NewDoubleLinkedList() *DoubleLinkedList {
	head := NewListNode(-1, nil, nil)
	tail := NewListNode(-1, nil, nil)
	head.next = tail
	tail.prev = head

	return &DoubleLinkedList{head: head, tail: tail}
}

func (d *DoubleLinkedList) Remove(node *ListNode) {
	node.prev.next = node.next
	node.next.prev = node.prev
}

// PushFront. push ke nextnya head. node paling front adalah node most recently used
func (d *DoubleLinkedList) PushFront(val int) *ListNode {
	newNode := NewListNode(val, nil, nil)

	nextFrontNode := d.head

	d.head.next.prev = newNode
	newNode.next = d.head.next

	newNode.prev = nextFrontNode
	nextFrontNode.next = newNode

	return newNode
}

// GetBack. return node prevnya tail. node ini adalah node least recently used
func (d *DoubleLinkedList) GetBack() *ListNode {
	return d.tail.prev
}

// Size. return jumlah node dalam list
func (d *DoubleLinkedList) Size() int {
	size := 0
	curr := d.head
	for curr != nil {
		size++
		curr = curr.next
	}
	return size
}

// GetKey. return key dari node.
func (d *DoubleLinkedList) GetKey(node *ListNode) int {
	return node.Key
}

type LRUReplacer struct {
	mu       sync.RWMutex
	capacity int
	lst      *DoubleLinkedList
	index    map[int]*ListNode
}

func NewLRUReplacer(capacity int) *LRUReplacer {
	return &LRUReplacer{
		mu:       sync.RWMutex{},
		capacity: capacity,
		lst:      NewDoubleLinkedList(),
		index:    make(map[int]*ListNode),
	}
}

// Unpin. marks a frame as unpinned, making it eligible for eviction dari LRU
func (lru *LRUReplacer) Unpin(frameID int) {
	lru.mu.RLock()

	if len(lru.index) >= lru.capacity {
		lru.mu.RUnlock()
		// lru full -> dont do anything.. least recently used harus di write ke disk dulu..
		return
	}

	if _, ok := lru.index[frameID]; ok {
		lru.mu.RUnlock()
		// already in the list
		return
	}
	lru.mu.RUnlock()

	lru.mu.Lock()
	defer lru.mu.Unlock()

	elem := lru.lst.PushFront(frameID) // most recently used
	lru.index[frameID] = elem
}

// Size. return jumlah frame dalam LRU
func (lru *LRUReplacer) Size() int {
	lru.mu.Lock()
	defer lru.mu.Unlock()

	return len(lru.index)
}

// Pin marks a frame as pinned. buat frame jadi ineligible for eviction dari LRU
func (lru *LRUReplacer) Pin(frameID int) {
	lru.mu.RLock()

	if elem, ok := lru.index[frameID]; ok {
		lru.mu.RUnlock()

		lru.mu.Lock()
		defer lru.mu.Unlock()

		lru.lst.Remove(elem)       // remove from list
		delete(lru.index, frameID) // remove from index
	} else {
		lru.mu.RUnlock()
	}

}

// Victim. return frameID yang akan di evict dari LRU (yang least recently used di prevnya tail..)
func (lru *LRUReplacer) Victim(frameID *int) bool {
	lru.mu.Lock()
	defer lru.mu.Unlock()

	if lru.lst.Size() == 0 {
		return false
	}

	frontElem := lru.lst.GetBack() // least recently used

	val := frontElem.Key

	lru.lst.Remove(frontElem) // remove dari list
	if _, val := lru.index[val]; !val {
		return false
	}

	*frameID = val         // set frameID ke least recently used
	delete(lru.index, val) // remove dari index
	return true
}

// Remove. remove frame dari LRU
func (lru *LRUReplacer) Remove(frameID int) {
	lru.mu.Lock()
	defer lru.mu.Unlock()

	if elem, ok := lru.index[frameID]; ok {
		lru.lst.Remove(elem)
		delete(lru.index, frameID)
	}
}
