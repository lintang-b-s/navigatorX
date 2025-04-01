package datastructure

import (
	"bytes"
	"encoding/gob"
	"fmt"
	"math"
	"os"
)

type RtreeBoundingBox struct {
	// number of dimensions
	Dim int
	// Edges[i][0] = low value, Edges[i][1] = high value
	// i = 0,...,Dim
	Edges [][2]float64
}

func NewRtreeBoundingBox(dim int, minVal []float64, maxVal []float64) RtreeBoundingBox {
	b := RtreeBoundingBox{Dim: dim, Edges: make([][2]float64, dim)}
	for axis := 0; axis < dim; axis++ {
		b.Edges[axis] = [2]float64{minVal[axis], maxVal[axis]}
	}

	return b
}

func BoundingBox(b RtreeBoundingBox, bb RtreeBoundingBox) RtreeBoundingBox {
	newBound := NewRtreeBoundingBox(b.Dim, make([]float64, b.Dim), make([]float64, b.Dim))

	for axis := 0; axis < b.Dim; axis++ {
		if b.Edges[axis][0] <= bb.Edges[axis][0] {
			newBound.Edges[axis][0] = b.Edges[axis][0]
		} else {
			newBound.Edges[axis][0] = bb.Edges[axis][0]
		}

		if b.Edges[axis][1] >= bb.Edges[axis][1] {
			newBound.Edges[axis][1] = b.Edges[axis][1]
		} else {
			newBound.Edges[axis][1] = bb.Edges[axis][1]
		}
	}

	return newBound
}

// area calculates the area (in N dimensions) of a bounding box.
func area(b RtreeBoundingBox) float64 {
	area := 1.0
	for axis := 0; axis < b.Dim; axis++ {
		area *= b.Edges[axis][1] - b.Edges[axis][0]
	}
	return area
}

// Overlaps checks if two bounding boxes overlap.
func Overlaps(b RtreeBoundingBox, bb RtreeBoundingBox) bool {
	for axis := 0; axis < b.Dim; axis++ {
		if !(b.Edges[axis][0] < bb.Edges[axis][1]) || !(bb.Edges[axis][0] < b.Edges[axis][1]) {
			/*


				____________________	______________________
				|	b				|   |					   |
				|					|   |			bb		   |
				|	   				|   |					   |
				____________________    |  ____________________

				or

				____________________	______________________
				|	bb				|   |					   |
				|					|   |			b		   |
				|	   				|   |					   |
				____________________    |  ____________________


			*/
			return false
		}
	}

	return true
}

// isBBSame determines if two bounding boxes are identical
func (b *RtreeBoundingBox) IsBBSame(bb RtreeBoundingBox) bool {
	for axis := 0; axis < b.Dim; axis++ {
		if b.Edges[axis][0] != bb.Edges[axis][0] || b.Edges[axis][1] != bb.Edges[axis][1] {
			return false
		}
	}

	return true
}

type BoundedItem interface {
	GetBound() RtreeBoundingBox
	isLeafNode() bool
	IsData() bool
}

// rtree node. can be either a leaf node or a internal node or leafData.
type RtreeNode struct {
	// entries. can be either a leaf node or a  internal node.
	// leafNode has items in the form of a list of RtreeLeaf (*RtreeNode with 0 Items & a Leaf)
	Items  []*RtreeNode
	Parent *RtreeNode

	Bound RtreeBoundingBox
	// isLeaf. true if  this node is a leafNode.
	IsLeaf bool

	Leaf OSMObject // if this node is a leafData
}

// isLeaf. true if this node is a leafNode.
func (node *RtreeNode) isLeafNode() bool {
	return node.IsLeaf
}

func (node *RtreeNode) GetBound() RtreeBoundingBox {
	return node.Bound
}

func (node *RtreeNode) ComputeBB() RtreeBoundingBox {
	if len(node.Items) == 1 {
		return node.Items[0].GetBound()
	}
	bb := BoundingBox(node.Items[0].GetBound(), node.Items[1].GetBound())
	for i := 2; i < len(node.Items); i++ {
		bb = BoundingBox(bb, node.Items[i].GetBound())
	}
	return bb
}

func (node *RtreeNode) IsData() bool {
	return false
}

type Rtree struct {
	Root          *RtreeNode
	Size          int
	MinChildItems int
	MaxChildItems int
	Dimensions    int
	Height        int
}

func NewRtree(minChildItems, maxChildItems, dimensions int) *Rtree {

	return &Rtree{
		Root: &RtreeNode{
			IsLeaf: true,
			Items:  make([]*RtreeNode, 0, maxChildItems),
		},
		Size:          0,
		Height:        1,
		MinChildItems: minChildItems,
		MaxChildItems: maxChildItems,
		Dimensions:    dimensions,
	}

}

func (rt *Rtree) InsertLeaf(bound RtreeBoundingBox, leaf OSMObject, reinsert bool) {

	leaf.SetBound(bound)
	newLeaf := &RtreeNode{}
	newLeaf.Bound = bound
	newLeaf.Leaf = leaf

	leafNode := rt.chooseLeaf(rt.Root, leaf.GetBound())
	leafNode.Items = append(leafNode.Items, newLeaf)

	newLeaf.Parent = leafNode

	if !reinsert {
		rt.Size++
	}

	var l, ll *RtreeNode
	l = leafNode
	if len(leafNode.Items) > rt.MaxChildItems {
		l, ll = rt.SplitNode(leafNode)
	}

	p, pp := rt.adjustTree(l, ll)
	if pp != nil {
		// 14. [Grow tree taller.] If node split propagation caused the root to split,
		// create a new root whose children are
		// the two resulting nodes.
		rt.Root = &RtreeNode{}
		pp.Bound = pp.ComputeBB()

		rt.Root.Items = []*RtreeNode{p, pp}
		p.Parent = rt.Root
		pp.Parent = rt.Root
		rt.Height++

		rt.Root.Bound = rt.Root.ComputeBB()
	}
}

func (rt *Rtree) adjustTree(l, ll *RtreeNode) (*RtreeNode, *RtreeNode) {
	//ATI. [Initialize.] Set N=L. If L was split
	// previously, set NN to be the resulting
	// second node.
	n := l
	var nn *RtreeNode
	if ll != nil {
		nn = ll
	}

	if n == rt.Root {
		n.Bound = n.ComputeBB()
		//AT2. [Check if done.] If N is the root, stop.
		return n, nn
	}
	//AT3. [Adjust covering rectangle in parent
	// entry.] Let P be the parent node of
	// N, and let EN be N's entry in P.
	// Adjust En.I s o that it tightly encloses
	// all entry rectangles in N
	p := n.Parent
	en := p.Items[0]

	for i := 0; i < len(p.Items); i++ {
		if p.Items[i] == n {
			en = n
		}
	}

	en.Bound = n.ComputeBB()

	//AT4. [Propagate node split upward.] If N
	// has a partner NN resulting from an
	// earlier split, create a new entry ENN
	// with ENN.p pointing to NN and Enn.I
	// enclosing all rectangles in NN. Add
	// Enn to P if there is room Otherwise,
	// invoke SplitNode to produce P and
	// PP containing Em and all P ’s old
	// entries.
	//AT5. [Move up to next level.] Set N=P and
	//set NN-PP if a split occurred.
	//Repeat from AT2.

	if nn != nil {
		enn := nn
		enn.Bound = nn.ComputeBB()
		enn.Parent = p

		p.Items = append(p.Items, enn)
		if len(p.Items) > rt.MaxChildItems {
			return rt.adjustTree(rt.SplitNode(p))
		}
	}

	return rt.adjustTree(p, nil)
}

func (rt *Rtree) SplitNode(l *RtreeNode) (*RtreeNode, *RtreeNode) {
	//QSl. [Pick first entry for each group.]
	// Apply Algorithm PickSeeds to choose
	// two entries to be the first elements
	// of the groups. Assign each to a group
	firstEntryGroupOne, firstEntryGroupTwo := rt.linearPickSeeds(l)

	remaining := make([]*RtreeNode, 0, len(l.Items)-2)
	for i := 0; i < len(l.Items); i++ {
		if l.Items[i] != firstEntryGroupOne && l.Items[i] != firstEntryGroupTwo {
			remaining = append(remaining, l.Items[i])
		}
	}

	groupOne := l
	groupOne.Items = []*RtreeNode{firstEntryGroupOne}
	groupOne.Parent = l.Parent
	firstEntryGroupOne.Parent = groupOne

	groupTwo := &RtreeNode{
		Parent: l.Parent,
		Items:  []*RtreeNode{firstEntryGroupTwo},
		IsLeaf: l.IsLeaf,
	}
	firstEntryGroupTwo.Parent = groupTwo

	//QS2. [Check if done.] If all entries have
	// been assigned, stop. If one group has
	// so few entries that all the rest must
	// be assigned to it in order for it to
	// have the minimum number m , assign
	// them and stop.
	for len(remaining) > 0 {
		// QS3. [Select entry to assign.] Invoke Algorithm PickNext to choose the next
		// entry to assign. Add it to the group
		// whose covering rectangle will have to
		// be enlarged least to accommodate it.
		// Resolve ties by adding the entry to
		// the group with smaller area, then to
		// the one with fewer entries, then to
		// either. Repeat from QS2.

		nextEntryIdx := rt.pickNext(groupOne, groupTwo, remaining)
		groupOneBB := groupOne.ComputeBB()
		groupTwoBB := groupTwo.ComputeBB()

		bbGroupOne := BoundingBox(groupOneBB, remaining[nextEntryIdx].GetBound())
		enlargementOne := area(bbGroupOne) - area(groupOneBB)

		bbGroupTwo := BoundingBox(groupTwoBB, remaining[nextEntryIdx].GetBound())
		enlargementTwo := area(bbGroupTwo) - area(groupTwoBB)

		if len(groupOne.Items)+len(remaining) <= rt.MinChildItems {
			groupOne.Items = append(groupOne.Items, remaining[nextEntryIdx])
			remaining[nextEntryIdx].Parent = groupOne
		} else if len(groupTwo.Items)+len(remaining) <= rt.MinChildItems {
			groupTwo.Items = append(groupTwo.Items, remaining[nextEntryIdx])
			remaining[nextEntryIdx].Parent = groupTwo
		} else {
			if enlargementOne < enlargementTwo ||
				(enlargementOne == enlargementTwo && area(bbGroupOne) < area(bbGroupTwo)) ||
				(enlargementOne == enlargementTwo && area(bbGroupOne) == area(bbGroupTwo) &&
					len(groupOne.Items) < len(groupTwo.Items)) {
				groupOne.Items = append(groupOne.Items, remaining[nextEntryIdx])
				remaining[nextEntryIdx].Parent = groupOne
			} else if enlargementOne > enlargementTwo ||
				(enlargementOne == enlargementTwo && area(bbGroupOne) > area(bbGroupTwo)) ||
				(enlargementOne == enlargementTwo && area(bbGroupOne) == area(bbGroupTwo) &&
					len(groupOne.Items) > len(groupTwo.Items)) {
				groupTwo.Items = append(groupTwo.Items, remaining[nextEntryIdx])
				remaining[nextEntryIdx].Parent = groupTwo
			}
		}

		remaining = append(remaining[:nextEntryIdx], remaining[nextEntryIdx+1:]...)
	}

	return groupOne, groupTwo
}

func (rt *Rtree) pickNext(groupOne, groupTwo *RtreeNode, remaining []*RtreeNode) int {
	/*
		PN1. [Determine cost of putting each
		entry in each group.] For each entry
		E not yet in a group, calculate d1=
		the area increase required in the
		covering rectangle of Group 1 to
		include E.I. Calculate d2 similarly
		for Group 2

		PN2. [Find entry with greatest preference
		for one group.] Choose any entry
		with the maximum difference
		between d 1 and d 2
	*/
	chosen := 0
	maxDiff := math.Inf(-1)
	groupOneBB := groupOne.GetBound()
	groupTwoBB := groupTwo.GetBound()
	for i := 0; i < len(remaining); i++ {
		enBBGroupOne := BoundingBox(groupOneBB, remaining[i].GetBound())
		d1 := area(enBBGroupOne) - area(groupOneBB)

		enBBGroupTwo := BoundingBox(groupTwoBB, remaining[i].GetBound())
		d2 := area(enBBGroupTwo) - area(groupTwoBB)

		d := math.Abs(d1 - d2)

		if d > maxDiff {
			chosen = i
			maxDiff = d
		}
	}
	return chosen
}

/*
LPSl.[Find extreme rectangles along all
dimensions.] Along each dimension,
find the entry whose rectangle has
the highest low side, and the one
with the lowest high side. Record the
separation.

LPS2. [Adjust for shape of the rectangle
cluster.] Normalize the separations
by dividing by the width of the entire
set along the corresponding dimension.

LPS3. [Select the most extreme pair.]
Choose the pair with the greatest
normalized separation along any
dimension.
*/
func (rt *Rtree) linearPickSeeds(l *RtreeNode) (*RtreeNode, *RtreeNode) {

	entryOne := l.Items[0]
	entryTwo := l.Items[1]

	greatestNormalizedSeparation := math.Inf(-1)
	for axis := 0; axis < rt.Dimensions; axis++ {
		lowestHighSide := math.Inf(1)
		highestLowSide := math.Inf(-1)

		highestHighSide := math.Inf(-1)
		lowestLowSide := math.Inf(1)

		lowestHighSideIdx := 0
		highestLowSideIdx := 0

		for i := 0; i < len(l.Items); i++ {
			lowSideEdge := l.Items[i].Bound.Edges[axis][0]
			if lowSideEdge > highestLowSide {
				highestLowSide = lowSideEdge
				highestLowSideIdx = i
			}

			if lowSideEdge < lowestLowSide {
				lowestLowSide = lowSideEdge
			}

			highSideEdge := l.Items[i].Bound.Edges[axis][1]

			if highSideEdge < lowestHighSide {
				lowestHighSide = highSideEdge
				lowestHighSideIdx = i
			}

			if highSideEdge > highestHighSide {
				highestHighSide = highSideEdge
			}

		}

		lWidth := highestLowSide - lowestHighSide

		widthAlongDimension := highestHighSide - lowestLowSide

		if lWidth/widthAlongDimension > greatestNormalizedSeparation {
			greatestNormalizedSeparation = lWidth / widthAlongDimension
			entryOne = l.Items[highestLowSideIdx]
			entryTwo = l.Items[lowestHighSideIdx]
		}
	}

	return entryOne, entryTwo
}

/*
CLl. [Initialize.] Set N to be the root
node.
CL2. [Leaf check.] If N is a leaf, return N.
CL3. [Choose subtree.] If Af is not a leaf,
let F be the entry in N whose rectangle F.I needs least enlargement to
include E.I. Resolve ties by choosing
the entry with the rectangle of smallest area.
CL4. [Descend until a leaf is reached.] Set
N to be the child node pointed to by
F.p and repeat from CL2.
*/
func (rt *Rtree) chooseLeaf(node *RtreeNode, bound RtreeBoundingBox) *RtreeNode {

	if node.isLeafNode() {
		return node
	}
	var chosen *RtreeNode

	minAreaEnlargement := math.MaxFloat64
	idxEntryWithMinAreaEnlargement := 0
	for i, item := range node.Items {
		itembb := item.GetBound()

		bb := BoundingBox(itembb, bound)

		enlargement := area(bb) - area(itembb)
		if enlargement < minAreaEnlargement ||
			(enlargement == minAreaEnlargement &&
				area(bb) < area(node.Items[idxEntryWithMinAreaEnlargement].GetBound())) {
			minAreaEnlargement = enlargement
			idxEntryWithMinAreaEnlargement = i
		}
	}

	chosen = node.Items[idxEntryWithMinAreaEnlargement]

	return rt.chooseLeaf(chosen, bound)
}

func (rt *Rtree) Search(bound RtreeBoundingBox) []RtreeNode {
	results := []RtreeNode{}
	return rt.search(rt.Root, bound, results)
}

func (rt *Rtree) search(node *RtreeNode, bound RtreeBoundingBox,
	results []RtreeNode) []RtreeNode {

	// S1. [Search subtrees.] If T is not a leaf,
	// check each entry E to determine
	// whether E.I Overlaps S. For all overlapping entries, invoke Search on the tree
	// whose root node is pointed to by E.p
	if !node.isLeafNode() {
		for _, e := range node.Items {
			if Overlaps(e.GetBound(), bound) {
				results = rt.search(e, bound, results)
			}
		}
	} else {
		for _, e := range node.Items {
			if Overlaps(e.GetBound(), bound) {
				// S2. [Search leaf node.] If T is a leaf, check
				// all entries E to determine whether E.I
				// Overlaps S. If so, E is a qualifying
				// record
				results = append(results, *e)
			}
		}
	}

	return results
}

type Point struct {
	Lat float64
	Lon float64
}

func NewPoint(lat, lon float64) Point {
	return Point{Lat: lat, Lon: lon}
}

// MinDist computes the square of the distance from a point to a rectangle. If the point is contained in the rectangle then the distance is zero.
func (p Point) MinDist(r RtreeBoundingBox) float64 {

	// Edges[0] = {minLat, maxLat}
	// Edges[1] = {minLon, maxLon}
	rLat, rLon := 0.0, 0.0
	if p.Lat < r.Edges[0][0] {
		rLat = r.Edges[0][0]
	} else if p.Lat > r.Edges[0][1] {
		rLat = r.Edges[0][1]
	} else {
		rLat = p.Lat
	}

	if p.Lon < r.Edges[1][0] {
		rLon = r.Edges[1][0]
	} else if p.Lon > r.Edges[1][1] {
		rLon = r.Edges[1][1]
	} else {
		rLon = p.Lon
	}

	sum := euclidianDistanceEquiRectangularAprox(p.Lat, p.Lon, rLat, rLon)

	return sum
}

func (p Point) MinDistHaversine(r RtreeBoundingBox) float64 {

	// Edges[0] = {minLat, maxLat}
	// Edges[1] = {minLon, maxLon}
	sum := 0.0
	rLat, rLon := 0.0, 0.0
	if p.Lat < r.Edges[0][0] {
		rLat = r.Edges[0][0]
	} else if p.Lat > r.Edges[0][1] {
		rLat = r.Edges[0][1]
	} else {
		rLat = p.Lat
	}

	if p.Lon < r.Edges[1][0] {
		rLon = r.Edges[1][0]
	} else if p.Lon > r.Edges[1][1] {
		rLon = r.Edges[1][1]
	} else {
		rLon = p.Lon
	}

	sum += HaversineDistance(p.Lat, p.Lon, rLat, rLon)

	return sum
}

// MinDist computes the square of the distance from a point to a rectangle (farthest point in rectangle). If the point is contained in the rectangle then the distance is zero.
func (p Point) maxDist(r RtreeBoundingBox) float64 {

	// Edges[0] = {minLat, maxLat}
	// Edges[1] = {minLon, maxLon}
	sum := 0.0
	rLat, rLon := 0.0, 0.0
	if p.Lat < r.Edges[0][0] {
		rLat = r.Edges[0][1]
	} else if p.Lat > r.Edges[0][1] {
		rLat = r.Edges[0][0]
	} else {
		rLat = p.Lat
	}

	if p.Lon < r.Edges[1][0] {
		rLon = r.Edges[1][1]
	} else if p.Lon > r.Edges[1][1] {
		rLon = r.Edges[1][0]
	} else {
		rLon = p.Lon
	}

	sum += euclidianDistanceEquiRectangularAprox(p.Lat, p.Lon, rLat, rLon)

	return sum
}

type OSMObject struct {
	ID    int
	Lat   float64
	Lon   float64
	Tag   map[int]int
	Bound RtreeBoundingBox
}

func NewOSMObject(id int, lat, lon float64, tag map[int]int,
	bound RtreeBoundingBox) OSMObject {
	return OSMObject{
		ID:    id,
		Lat:   lat,
		Lon:   lon,
		Tag:   tag,
		Bound: bound,
	}
}

func (o *OSMObject) GetBound() RtreeBoundingBox {
	return o.Bound
}

func (o *OSMObject) SetBound(bb RtreeBoundingBox) {
	o.Bound = bb
}

func (o *OSMObject) isLeafNode() bool {
	return false
}

func (o *OSMObject) IsData() bool {
	return true
}

func (rt *Rtree) NearestNeighboursPQ(k int, p Point) []OSMObject {
	nearestLists := make([]OSMObject, 0, k)

	callback := func(n OSMObject) bool {
		nearestLists = append(nearestLists, n)
		if rt.Size > k {
			return len(nearestLists) < k
		}
		return len(nearestLists) < rt.Size
	}

	rt.incrementalNearestNeighbor(p, callback)

	return nearestLists
}

// NearestNeighboursRadiusFilterOSM. returns the k nearest neighbours (with filtered osm feature) within a given radius in km.
func (rt *Rtree) NearestNeighboursRadiusFilterOSM(k, offfset int, p Point, maxRadius float64,
	osmFeature int) []OSMObject {
	nearestLists := make([]OSMObject, 0, k*100)

	callback := func(n OSMObject) bool {
		dist := HaversineDistance(p.Lat, p.Lon, n.Lat, n.Lon)
		if _, ok := n.Tag[osmFeature]; ok && dist <= maxRadius {
			nearestLists = append(nearestLists, n)
		}

		return dist <= maxRadius
	}

	rt.incrementalNearestNeighbor(p, callback)

	if len(nearestLists) > offfset {
		nearestLists = nearestLists[offfset:]
	}

	if len(nearestLists) > k {
		nearestLists = nearestLists[:k]
	}

	return nearestLists
}

const (
	maxK = 30
)

func (rt *Rtree) NearestNeighboursRadiusDifferentStreetID(p Point, maxRadius float64,
) []OSMObject {
	nearestLists := make([]OSMObject, 0, maxK)

	edgeSet := make(map[int]struct{})
	edgeCoordSet := make(map[[2]float64]struct{}) // tiap nearby edges harus unique {lat,lon}nya (for map matching).
	callback := func(n OSMObject) bool {

		dist := HaversineDistance(p.Lat, p.Lon, n.Lat, n.Lon)

		_, ok1 := edgeCoordSet[[2]float64{n.Lat, n.Lon}]
		_, ok2 := edgeSet[n.ID]
		if !ok1 && !ok2 {
			nearestLists = append(nearestLists, n)
		}

		edgeSet[n.ID] = struct{}{}
		edgeCoordSet[[2]float64{n.Lat, n.Lon}] = struct{}{}

		return dist <= maxRadius
		// if rt.Size > maxK {
		// 	return len(nearestLists) < maxK && dist <= maxRadius
		// }
		// return len(nearestLists) < rt.Size
	}

	rt.incrementalNearestNeighbor(p, callback)

	return nearestLists
}

func (rt *Rtree) NearestNeighboursRadiusDifferentStreetIDWithinRadius(k, offfset int, p Point, maxRadius float64,
) []OSMObject {
	nearestLists := make([]OSMObject, 0, k*100)
	stretIDSet := make(map[int]OSMObject)

	callback := func(n OSMObject) bool {
		dist := HaversineDistance(p.Lat, p.Lon, n.Lat, n.Lon)
		val := n.ID

		sameObj, sameStreet := stretIDSet[val]
		if sameStreet && sameObj.Tag[0] != n.Tag[0] {
			nearestLists = append(nearestLists, n)
			stretIDSet[val] = n
		} else if !sameStreet {
			nearestLists = append(nearestLists, n)
			stretIDSet[val] = n
		}

		return dist <= maxRadius && len(nearestLists) < k
	}

	rt.incrementalNearestNeighbor(p, callback)

	return nearestLists
}

// https://dl.acm.org/doi/pdf/10.1145/320248.320255 (Fig. 4.  incremental nearest neighbor algorithm)
func (rt *Rtree) incrementalNearestNeighbor(p Point, callback func(OSMObject) bool) {
	pq := NewMinHeap()
	pq.Insert(NewPriorityQueueNodeRtree2(0, rt.Root, false))

	for pq.Size() > 0 {

		element, ok := pq.ExtractMin()
		if !ok {
			return
		}
		if element.Item.IsData() {

			qObjectDist := euclidianDistanceEquiRectangularAprox(p.Lat, p.Lon,
				element.Item.(*OSMObject).Lat, element.Item.(*OSMObject).Lon)

			if first, pqSizeMoreThanZero := pq.GetMin(); element.isObjectBoundingRectangle &&
				pqSizeMoreThanZero && qObjectDist > first.Rank {

				pq.Insert(NewPriorityQueueNodeRtree2(qObjectDist, element.Item, false))

			} else {
				if !callback(*element.Item.(*OSMObject)) {
					return
				}
			}
		} else if element.Item.isLeafNode() {

			for _, item := range element.Item.(*RtreeNode).Items {
				pq.Insert(NewPriorityQueueNodeRtree2(p.MinDist(item.Leaf.GetBound()),
					&item.Leaf, true))
			}
		} else {
			for _, item := range element.Item.(*RtreeNode).Items {
				pq.Insert(NewPriorityQueueNodeRtree2(p.MinDist(item.GetBound()), item, false))
			}
		}
	}
}

func (rt *Rtree) ImprovedNearestNeighbor(p Point) OSMObject {

	nearest := OSMObject{}

	callback := func(n OSMObject) bool {
		nearest = n

		return false
	}

	rt.incrementalNearestNeighbor(p, callback)

	return nearest
}

func (rt *Rtree) Delete(leaf OSMObject) bool {
	// Dl. [Find node containing record.]
	// Invoke FindLeaf to locate the leaf
	// node L containing E. Stop if the
	// record was not found.
	l, leafLevel := rt.FindLeaf(leaf, rt.Root, 1)
	if l == nil {
		return false
	}

	// D2. [Delete record.] Remove E from L.
	for i, item := range l.Items {
		if item.Leaf.ID == leaf.ID {
			l.Items[i] = l.Items[len(l.Items)-1]
			l.Items = l.Items[:len(l.Items)-1]
			break
		}
	}

	q := make([]NodeWithLevel, 0, rt.MaxChildItems)

	//D3. [Propagate changes.] Invoke CondenseTree, passing L.
	// CT1. [Initialize.] Set N=L. Set Q, the-set
	// of eliminated nodes, to be empty.
	rt.condenseTree(l, q, leafLevel)
	rt.Size--

	// D4. [Shorten tree.] If the root node has
	// only one child after the tree has
	// been adjusted, make thé child the
	// new root.
	if len(rt.Root.Items) == 1 {
		rt.Root = rt.Root.Items[0]
		rt.Root.Parent = nil
		rt.Height--
	}

	return true
}

func (rt *Rtree) condenseTree(n *RtreeNode, q []NodeWithLevel, nLevel int) {

	if n == rt.Root {
		if len(n.Items) != 0 {
			n.Bound = n.ComputeBB() // harus recompute bb dari root kalau misal children dari root yang masuk ke q, len(items) == 0 -> mbr root gak diupdate & salah
		}
		// CT2. [Find parent entry.] If N is the root,
		// go to CT6.
		// CT6. [Re-insert orphaned entries.] Reinsert all entries of nodes in set Q.
		// Entries from eliminated leaf nodes
		// are re-inserted in tree leaves as
		// described in Algorithm Insert, but
		// entries from higher-level nodes must
		// be placed higher in the tree, so that
		// leaves of their dependent subtrees
		// will be on the same level as leaves of
		// the main tree.

		for _, qEntry := range q {

			if len(qEntry.node.Items) > 0 {
				qEntry.node.Bound = qEntry.node.ComputeBB()

				if qEntry.node.isLeafNode() {
					for i := 0; i < len(qEntry.node.Items); i++ {
						entry := qEntry.node.Items[i]

						rt.InsertLeaf(entry.GetBound(), entry.Leaf, true)
					}
				} else {
					for i := 0; i < len(qEntry.node.Items); i++ {
						entry := qEntry.node.Items[i]
						entry.Bound = entry.ComputeBB()
						rt.InsertLevel(entry, qEntry.level)
					}
				}
			}
		}

	} else {
		// CT2. [Find parent entry.] If N is the root,
		// go to CT6. Otherwise let P be the
		// parent of N, and let EN be N's entry
		// in P.
		p := n.Parent

		enID := -1
		for i := 0; i < len(p.Items); i++ {
			if p.Items[i] == n {
				enID = i
			}
		}

		// CT3. [Eliminate under-full node.] If N has
		// fewer than m entries, delete EN from
		// P and add N to set Q.
		if len(n.Items) < rt.MinChildItems {

			q = append(q, NodeWithLevel{
				node:  n,
				level: nLevel,
			})

			p.Items[enID] = p.Items[len(p.Items)-1]
			p.Items = p.Items[:len(p.Items)-1]
		} else {
			// CT4. [Adjust covering rectangle.] -If N has
			// not been eliminated, adjust EN.I to
			// tightly contain all entries in N.

			p.Items[enID].Bound = p.Items[enID].ComputeBB()
		}

		// CT5. [Move up one level in tree.] Set N=P
		// and repeat from CT2
		rt.condenseTree(p, q, nLevel-1)
	}
}

func (rt *Rtree) FindLeaf(leaf OSMObject, node *RtreeNode, level int) (*RtreeNode, int) {
	// FL1. [Search subtrees.] If T is not a leaf,
	// check each entry F in T to determine if F.I Overlaps E.I. For each
	// such entry inyoke FindLeaf on the
	// tree whose root is pointed to by F.p
	// until E is found
	if !node.isLeafNode() {
		for _, item := range node.Items {
			if Overlaps(item.GetBound(), leaf.GetBound()) {
				foundNode, leafLevel := rt.FindLeaf(leaf, item, level+1)
				if foundNode != nil {
					return foundNode, leafLevel
				}
			}
		}
	}

	// FL2. [Search leaf node for record.] If T is
	// a leaf, check each entry to see if it
	// matches E. If E is found return T.
	for _, item := range node.Items {
		if item.Leaf.ID == leaf.ID {
			return node, level
		}
	}
	return nil, 0
}

type NodeWithLevel struct {
	node   *RtreeNode
	level  int
	isLeaf bool
}

// InsertLevel. insert node ke items dari node lain di level=level.
// node bisa berupa leafData (*RtreeNode with 0 items & a leaf) atau internal Node (*RtreeNode with >= m childrens).
// leafData diinsert ke level = rt.Height
// internal Node diinsert ke node lain dengan  level sebelumnya - 1
func (rt *Rtree) InsertLevel(node *RtreeNode, level int) {

	levelNode := rt.ChooseLevel(rt.Root, node.GetBound(), level, 1)

	levelNode.Items = append(levelNode.Items, node)

	node.Parent = levelNode

	var l, ll *RtreeNode
	l = levelNode
	if len(levelNode.Items) > rt.MaxChildItems {
		l, ll = rt.SplitNode(levelNode)
	}

	p, pp := rt.adjustTree(l, ll)
	if pp != nil {
		// 14. [Grow tree taller.] If node split propagation caused the root to split,
		// create a new root whose children are
		// the two resulting nodes.
		rt.Root = &RtreeNode{}
		pp.Bound = pp.ComputeBB()

		rt.Root.Items = []*RtreeNode{p, pp}
		p.Parent = rt.Root
		pp.Parent = rt.Root
		rt.Height++

		rt.Root.Bound = rt.Root.ComputeBB()
	}
}

func (rt *Rtree) ChooseLevel(node *RtreeNode, bound RtreeBoundingBox,
	desiredLevel, level int) *RtreeNode {

	if desiredLevel == level {

		return node
	}
	var chosen *RtreeNode

	minAreaEnlargement := math.MaxFloat64
	idxEntryWithMinAreaEnlargement := 0
	for i, item := range node.Items {
		itembb := item.GetBound()

		bb := BoundingBox(itembb, bound)

		enlargement := area(bb) - area(itembb)
		if enlargement < minAreaEnlargement ||
			(enlargement == minAreaEnlargement &&
				area(bb) < area(node.Items[idxEntryWithMinAreaEnlargement].GetBound())) {
			minAreaEnlargement = enlargement
			idxEntryWithMinAreaEnlargement = i
		}
	}

	chosen = node.Items[idxEntryWithMinAreaEnlargement]

	return rt.ChooseLevel(chosen, bound, desiredLevel, level+1)
}

// gabisa di encode pakai json /gob encoder karena ada cyclic reference/pointer ke childnode
func SerializeRtreeData(workingDir string, outputDir string, items []OSMObject) error {

	buf := new(bytes.Buffer)
	enc := gob.NewEncoder(buf)

	err := enc.Encode(items)
	if err != nil {
		return err
	}

	var rtreeFile *os.File
	if workingDir != "/" {
		rtreeFile, err = os.OpenFile(workingDir+"/"+outputDir+"/"+"rtree.dat", os.O_RDWR|os.O_CREATE, 0700)
		if err != nil {
			return err
		}
	} else {
		rtreeFile, err = os.OpenFile(outputDir+"/"+"rtree.dat", os.O_RDWR|os.O_CREATE, 0700)
		if err != nil {
			return err
		}
	}
	_, err = rtreeFile.Write(buf.Bytes())

	return err
}

func (rt *Rtree) Deserialize(workingDir string, outputDir string) error {

	var rtreeFile *os.File
	var err error
	if workingDir != "/" {
		rtreeFile, err = os.Open(workingDir + "/" + outputDir + "/" + "rtree.dat")
		if err != nil {
			return fmt.Errorf("error opening file: %v", err)
		}
	} else {
		rtreeFile, err = os.Open(outputDir + "/" + "rtree.dat")
		if err != nil {
			return fmt.Errorf("error opening file: %v", err)
		}
	}

	stat, err := os.Stat(rtreeFile.Name())
	if err != nil {
		return fmt.Errorf("error when getting metadata file stat: %w", err)
	}

	buf := make([]byte, stat.Size()*2)

	_, err = rtreeFile.Read(buf)
	if err != nil {
		return err
	}

	gobDec := gob.NewDecoder(bytes.NewBuffer(buf))

	items := []OSMObject{}
	err = gobDec.Decode(&items)
	if err != nil {
		return err
	}

	for _, item := range items {

		rt.InsertLeaf(item.GetBound(), item, false)
	}

	return nil
}
