package datastructure

import (
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/util"
)

type Entry[T any] struct {
	degree   int
	isMarked bool

	next   *Entry[T]
	prev   *Entry[T]
	child  *Entry[T]
	parent *Entry[T]

	elem     T
	priority float64
}

func NewEntry[T any](elem T, priority float64) *Entry[T] {
	e := &Entry[T]{
		elem:     elem,
		priority: priority,
	}
	e.next = e
	e.prev = e

	return e
}

func (e *Entry[T]) GetPriority() float64 {
	return e.priority
}

func (e *Entry[T]) GetElem() T {
	return e.elem
}

/*
amortized analysis ref: https://www.utsc.utoronto.ca/~atafliovich/cscb63/content/week10/clrs_fibonacci_chapter.pdf

potential function:
pot(Hi) = t(Hi) + 2m(Hi)

where Hi is the heap after operation-i
t(Hi) is the number of trees in root list after operation-i
m(Hi) is the number of marked nodes after operation-i
*/
type FibonaccyHeap[T any] struct {
	mMin  *Entry[T]
	mSize int
}

func NewFibonacciHeap[T any]() *FibonaccyHeap[T] {
	return &FibonaccyHeap[T]{
		mMin:  nil,
		mSize: 0,
	}
}

func (f *FibonaccyHeap[T]) GetMin() *Entry[T] {

	return f.mMin
}

func (f *FibonaccyHeap[T]) GetMinRank() float64 {
	if f.mMin == nil {
		return math.MaxFloat64
	}
	return f.mMin.priority
}

func (f *FibonaccyHeap[T]) Size() int {
	return f.mSize
}

/*
Insert. insert new entry to the heap

after operation insert-i
t(H_i) = t(H_{i-1}) + 1
m(H_i) = m(H_{i-1})
ci = 1 (actual cost)

ci' = 1 + ( t(H_{i-1}) + 1 + 2m(H_{i-1})  ) - ( t(H_{i-1}) + 2m(H_{i-1}) ) = 2
amortized cost = O(1)
*/
func (f *FibonaccyHeap[T]) Insert(value T, priority float64) *Entry[T] {

	result := NewEntry(value, priority)

	f.mMin = f.mergeLists(f.mMin, result)
	f.mSize++

	return result
}

func (f *FibonaccyHeap[T]) mergeLists(one *Entry[T], two *Entry[T]) *Entry[T] {

	if one == nil && two == nil {
		return nil
	} else if one != nil && two == nil {
		return one
	} else if one == nil && two != nil {
		return two
	}

	// both non-null
	/* merge two list


	+----+     +----+     +----+
	|    |--N->|one |--N->|    |
	|    |<-P--|    |<-P--|    |
	+----+     +----+     +----+


	+----+     +----+     +----+
	|    |--N->|two |--N->|    |
	|    |<-P--|    |<-P--|    |
	+----+     +----+     +----+

	to:

	+----+     +----+     +----+---+
	|    |--N->|one |     |    |   |
	|    |<-P--|    |     |    |<+ |
	+----+     +----+<-\  +----+ | |
	                 \  P        | |
	                  N  \       N |
	+----+     +----+  \->+----+ | |
	|    |--N->|two |     |    | | |
	|    |<-P--|    |     |    | | P
	+----+     +----+     +----+ | |
	             ^ |             | |
	             | +-------------+ |
	             +-----------------+

	*/
	oneNext := one.next
	one.next = two.next
	one.next.prev = one
	two.next = oneNext
	two.next.prev = two

	if one.priority < two.priority {
		return one
	}
	return two
}

/*
DecreaseKey. update priority of entry to newPriority

assume decreaseKey call perform c calls of recursive cascade-cut up to the root node
ci = c (actual cost)
after operation decreasekey-i node x
t(H_i) = t(H_{i-1}) + c (the original t(H_{i-1}), c-1 produced by cascading cut, and tree rooted at x)
m(H_i) <= m(H_{i-1})-c+2 (c-1 unmarked by cascading cut, and 1 already marked at last cascading-cut)

ci' (at most) =    c + ( t(H_{i-1}) + c  + 2( m(H_{i-1})-c+2) ) - (t(H_{i-1}) + 2m(H_{i-1})) =  c + 4 - c =  4
amortized cost = O(1)
*/
func (f *FibonaccyHeap[T]) DecreaseKey(entry *Entry[T], newPriority float64) {
	util.AssertPanic(newPriority <= entry.priority, "new priority must be less or equal than old priority")
	f.decreaseUnchecked(entry, newPriority)
}

func (f *FibonaccyHeap[T]) decreaseUnchecked(entry *Entry[T], priority float64) {
	// decrease key
	entry.priority = priority

	if entry.parent != nil && entry.priority <= entry.parent.priority {
		// node priority now higher than its parent priority
		// cut the node from its parent
		f.cutNode(entry)
	}

	if entry.priority < f.mMin.priority {
		// update min
		f.mMin = entry
	}

}

func (f *FibonaccyHeap[T]) cutNode(entry *Entry[T]) {
	// clear node mark
	entry.isMarked = false

	if entry.parent == nil {
		// base case: entry is root (no cascasding-cut further up)
		return
	}

	if entry.next != nil {
		// adjust the next and prev pointers of the sibling entry
		entry.next.prev = entry.prev
		entry.prev.next = entry.next
	}

	if entry.parent.child == entry {
		// remove entry from the child list of parent
		if entry.next != entry {
			entry.parent.child = entry.next
		} else {
			entry.parent.child = nil
		}
	}

	entry.parent.degree--

	entry.prev = entry
	entry.next = entry

	// add entry to the root list of H
	f.mMin = f.mergeLists(f.mMin, entry)

	// do cascade-cut
	if entry.parent.isMarked {
		// parent is marked, cut the parent
		f.cutNode(entry.parent)
	} else {
		// parent not marked. mark the parent
		entry.parent.isMarked = true
	}

	// clear parent pointer
	entry.parent = nil
}

/*
ExtractMin. remove the min node from the heap and return it

upon calling consolidate, the size of the root list is at most D(n)+t(H)-1 (original T(H), minus extracted min node, plus children of extracted min node)
so the amount of work of consolidate first loop D(n)+t(H)
ci = D(n)+t(H) (actual cost)

t(H_i) = t(H_{i-1}) + 1 = D(n)+1 (at most D(n) roots remain after consolidate operation)
m(H_i) = m(H_{i-1}) (no new marked nodes after extractMin Operation)

ci' = D(n)+t(H) + (D(n)+1  + 2 m(H)) - (t(H) + 2m(H)) = D(n)
maximum degree D(n) of any node in a fibonaci heap is O(log n)
amortized cost = O(log n)
*/
func (f *FibonaccyHeap[T]) ExtractMin() *Entry[T] {
	util.AssertPanic(f.mMin != nil, "heap is empty")

	f.mSize--

	minElem := f.mMin

	if f.mMin.next == f.mMin {
		// if heap only consist of one node
		f.mMin = nil
	} else {
		// adjust the pointer of the next and prev pointers of the sibling entry
		f.mMin.prev.next = f.mMin.next
		f.mMin.next.prev = f.mMin.prev
		f.mMin = f.mMin.next
	}

	if minElem.child != nil {
		// iterate minElemen childrens circular list and clear parent pointer of all of the min element childrens
		start := minElem.child

		curr := minElem.child
		for {
			curr.parent = nil
			curr = curr.next
			if curr == start {
				break
			}
		}
	}

	f.mMin = f.mergeLists(f.mMin, minElem.child)

	if f.mMin == nil {
		// is no entries left, return the min node
		return minElem
	}

	// else, call consolidate(H)

	// create treeTable slice, where treetable[i] contain either nil or tree with degree i
	treeTable := make([]*Entry[T], 0)

	// first add all root node to toVisit slice
	toVisit := make([]*Entry[T], 0)
	for curr := f.mMin; len(toVisit) == 0 || toVisit[0] != curr; curr = curr.next {
		toVisit = append(toVisit, curr)
	}

	for _, curr := range toVisit {
		for {
			for curr.degree >= len(treeTable) {
				// ensure list have size == curr.degree+1, so it can contain tree rooted at curr
				treeTable = append(treeTable, nil)
			}

			if treeTable[curr.degree] == nil {
				// set treeTable[d] = curr
				// and continue to next root
				treeTable[curr.degree] = curr
				break
			}

			// otherwise, merge curr with tree inside treeTable[d]
			other := treeTable[curr.degree]
			treeTable[curr.degree] = nil

			// ensure higher priority node is the children of lower priority node
			var (
				min, max *Entry[T]
			)

			if other.priority < curr.priority {
				min, max = other, curr
			} else {
				min, max = curr, other
			}

			// adjust the next and prev pointer of sibling entry
			max.next.prev = max.prev
			max.prev.next = max.next

			// merge curr tree and other tree (call FIB-HEAP-LINK(H,y,x))
			max.next = max
			max.prev = max
			min.child = f.mergeLists(min.child, max)

			max.parent = min

			// clear max mark
			max.isMarked = false

			// increase the min degree (has new child)
			min.degree++

			// continue merging this curr tree
			// (with other tree inside treeTable that have the same new degree)
			curr = min
		}

		// update heap min node
		if curr.priority <= f.mMin.priority {
			f.mMin = curr
		}
	}

	return minElem
}
