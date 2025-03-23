package snap

import (
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/datastructure"
	"log"
)

type Rtree interface {
	InsertLeaf(bound datastructure.RtreeBoundingBox, leaf datastructure.OSMObject, reinsert bool)
	ImprovedNearestNeighbor(p datastructure.Point) datastructure.OSMObject
	NearestNeighboursRadiusDifferentStreetID(p datastructure.Point, maxRadius float64) []datastructure.OSMObject
	NearestNeighboursRadiusDifferentStreetIDWithinRadius(k, offfset int, p datastructure.Point, maxRadius float64,
	) []datastructure.OSMObject
}

type RoadSnapper struct {
	rtree Rtree
}

func NewRoadSnapper(rtree Rtree) *RoadSnapper {
	return &RoadSnapper{rtree: rtree}
}

func (rs *RoadSnapper) SnapToRoad(p datastructure.Point) datastructure.OSMObject {
	return rs.rtree.ImprovedNearestNeighbor(p)
}

const (
	minRadius = 0.01 // 100 meter ( >= 50 meter)
)

func (rs *RoadSnapper) SnapToRoads(p datastructure.Point) []datastructure.OSMObject {
	var nearestEdges []datastructure.OSMObject

	radius := minRadius
	nearestEdges = rs.rtree.NearestNeighboursRadiusDifferentStreetID(p, radius)

	counter := 0

	for counter < 2 && len(nearestEdges) == 0 {
		radius += 0.005
		counter++
		nearestEdges = rs.rtree.NearestNeighboursRadiusDifferentStreetID(p, radius)

	}

	return nearestEdges
}

func (rs *RoadSnapper) SnapToRoadsWithinRadius(p datastructure.Point, radius float64, k int) []datastructure.OSMObject {
	var nearestEdges []datastructure.OSMObject

	nearestEdges = rs.rtree.NearestNeighboursRadiusDifferentStreetIDWithinRadius(k, 0, p, radius)

	counter := 0

	for counter < 2 && len(nearestEdges) < k {
		radius += 0.05
		counter++
		nearestEdges = rs.rtree.NearestNeighboursRadiusDifferentStreetIDWithinRadius(k, 0, p, radius)

	}
	if len(nearestEdges) > k {
		nearestEdges = nearestEdges[:k]
	}
	return nearestEdges
}

func (rs *RoadSnapper) InsertNode(bound datastructure.RtreeBoundingBox, leaf datastructure.OSMObject, reinsert bool) {
	rs.rtree.InsertLeaf(bound, leaf, reinsert)
}

// should not use osm ways directly, but use graph edges (for better map matching) [already tested]
func (rs *RoadSnapper) BuildRoadSnapper(ch *contractor.ContractedGraph) {

	duplicateEdges := make(map[int]map[int]struct{})

	for idx, edge := range ch.GetOutEdges() {

		if _, ok := duplicateEdges[int(edge.FromNodeID)]; !ok {
			duplicateEdges[int(edge.FromNodeID)] = make(map[int]struct{})
		}

		if _, ok := duplicateEdges[int(edge.FromNodeID)][int(edge.ToNodeID)]; ok {
			continue
		}

		if (idx+1)%10000 == 0 {
			log.Printf("insert osm way id %d to r-tree...", idx+1)
		}

		rs.insertEdgeToRtree(edge, ch)

		duplicateEdges[int(edge.FromNodeID)][int(edge.ToNodeID)] = struct{}{}
	}
}

const (
	tolerance = 0.0003
)

// insertEdgeToRtree insert filtered osm ways (road segments) to rtree
func (rs *RoadSnapper) insertEdgeToRtree(edge datastructure.EdgeCH, ch *contractor.ContractedGraph) {
	tag := make(map[int]int)
	tag[0] = int(edge.EdgeID)

	fromNode := ch.GetNode(edge.FromNodeID)
	toNode := ch.GetNode(edge.ToNodeID)

	latMin := min(fromNode.Lat-tolerance, toNode.Lat-tolerance)
	latMax := max(fromNode.Lat+tolerance, toNode.Lat+tolerance)
	lonMin := min(fromNode.Lon-tolerance, toNode.Lon-tolerance)
	lonMax := max(fromNode.Lon+tolerance, toNode.Lon+tolerance)

	// from node
	nodeBB := datastructure.NewRtreeBoundingBox(
		2, []float64{latMin, lonMin}, []float64{latMax, lonMax},
	)

	osmObj := datastructure.NewOSMObject(
		int(edge.EdgeID), fromNode.Lat, fromNode.Lon,
		tag, nodeBB,
	)


	rs.rtree.InsertLeaf(nodeBB, osmObj, false)

	pointsInbetween := ch.GetEdgeExtraInfo(int(edge.EdgeID)).PointsInBetween
	if len(pointsInbetween) > 2 {

		quarterPoint := pointsInbetween[len(pointsInbetween)/4]

		nodeBB = datastructure.NewRtreeBoundingBox(
			2, []float64{latMin, lonMin}, []float64{latMax, lonMax},
		)

		osmObj = datastructure.NewOSMObject(
			int(edge.EdgeID), quarterPoint.Lat, quarterPoint.Lon,
			tag, nodeBB,
		)
		rs.rtree.InsertLeaf(nodeBB, osmObj, false)

		midIdx := len(pointsInbetween) / 2
		midPoint := pointsInbetween[midIdx]

		nodeBB = datastructure.NewRtreeBoundingBox(
			2, []float64{latMin, lonMin}, []float64{latMax, lonMax},
		)

		osmObj = datastructure.NewOSMObject(
			int(edge.EdgeID), midPoint.Lat, midPoint.Lon,
			tag, nodeBB,
		)

		rs.rtree.InsertLeaf(nodeBB, osmObj, false)

		threeQuarterPoint := pointsInbetween[len(pointsInbetween)*3/4]

		nodeBB = datastructure.NewRtreeBoundingBox(
			2, []float64{latMin, lonMin}, []float64{latMax, lonMax},
		)

		osmObj = datastructure.NewOSMObject(
			int(edge.EdgeID), threeQuarterPoint.Lat, threeQuarterPoint.Lon,
			tag, nodeBB,
		)

		rs.rtree.InsertLeaf(nodeBB, osmObj, false)

	}

	osmObj = datastructure.NewOSMObject(
		int(edge.EdgeID), toNode.Lat, toNode.Lon,
		tag, nodeBB,
	)

	rs.rtree.InsertLeaf(nodeBB, osmObj, false)

}

func max(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}

func min(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}
