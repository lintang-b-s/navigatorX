package snap

import (
	"lintang/navigatorx/pkg/contractor"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"log"
)

type Rtree interface {
	InsertLeaf(bound datastructure.RtreeBoundingBox, leaf datastructure.OSMObject, reinsert bool)
	ImprovedNearestNeighbor(p datastructure.Point) datastructure.OSMObject
	NearestNeighboursRadiusDifferentStreetID(p datastructure.Point, maxRadius float64) []datastructure.OSMObject
	NearestNeighboursRadiusDifferentStreetIDWithinRadius(k, offfset int, p datastructure.Point, maxRadius float64,
	) []datastructure.OSMObject
	Search(bound datastructure.RtreeBoundingBox) []datastructure.RtreeNode
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
	minRadius = 0.3 // 300 meter  (dari  paper fmm)
)

func (rs *RoadSnapper) SnapToRoads(p datastructure.Point) []datastructure.OSMObject {

	upperRightLat, upperRightLon := geo.GetDestinationPoint(p.Lat, p.Lon, 45, 2*minRadius)
	lowerLeftLat, lowerLeftLon := geo.GetDestinationPoint(p.Lat, p.Lon, 225, 2*minRadius)

	bound := datastructure.NewRtreeBoundingBox(
		2, []float64{lowerLeftLat, lowerLeftLon},
		[]float64{upperRightLat, upperRightLon},
	)

	radius := minRadius
	nearestRtreeNodes := rs.rtree.Search(bound)

	nearestEdges := make([]datastructure.OSMObject, 0, len(nearestRtreeNodes))

	for _, node := range nearestRtreeNodes {
		nearestEdges = append(nearestEdges, node.Leaf)
	}

	counter := 0

	for counter < 2 && len(nearestEdges) == 0 {
		radius += 0.05
		counter++
		nearestRtreeNodes = rs.rtree.Search(bound)
		nearestEdges = make([]datastructure.OSMObject, 0, len(nearestRtreeNodes))
		for _, node := range nearestRtreeNodes {
			nearestEdges = append(nearestEdges, node.Leaf)
		}
	}

	return nearestEdges
}

func (rs *RoadSnapper) SnapToRoadsWithinRadius(p datastructure.Point, radius float64, k int) []datastructure.OSMObject {
	bound := datastructure.NewRtreeBoundingBox(
		2, []float64{p.Lat - radius, p.Lon - radius},
		[]float64{p.Lat + radius, p.Lon + radius},
	)

	nearestRtreeNodes := rs.rtree.Search(bound)

	nearestEdges := make([]datastructure.OSMObject, 0, len(nearestRtreeNodes))

	for _, node := range nearestRtreeNodes {
		nearestEdges = append(nearestEdges, node.Leaf)
	}

	counter := 0

	for counter < 2 && len(nearestEdges) == 0 {
		radius += 0.05
		counter++
		nearestRtreeNodes = rs.rtree.Search(bound)
		nearestEdges = make([]datastructure.OSMObject, 0, len(nearestRtreeNodes))
		for _, node := range nearestRtreeNodes {
			nearestEdges = append(nearestEdges, node.Leaf)
		}
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
	edgeBBRadius = 0.025 // 25 meter from fromNode & toNode
)

// insertEdgeToRtree insert filtered osm ways (road segments) to rtree
func (rs *RoadSnapper) insertEdgeToRtree(edge datastructure.EdgeCH, ch *contractor.ContractedGraph) {
	tag := make(map[int]int)
	tag[0] = int(edge.EdgeID)

	fromNode := ch.GetNode(edge.FromNodeID)
	toNode := ch.GetNode(edge.ToNodeID)

	upperFromLat, upperFromLon := geo.GetDestinationPoint(fromNode.Lat, fromNode.Lon, 45, edgeBBRadius)
	lowerFromLat, lowerFromLon := geo.GetDestinationPoint(fromNode.Lat, fromNode.Lon, 225, edgeBBRadius)

	upperToLat, upperToLon := geo.GetDestinationPoint(toNode.Lat, toNode.Lon, 45, edgeBBRadius)
	lowerToLat, lowerToLon := geo.GetDestinationPoint(toNode.Lat, toNode.Lon, 225, edgeBBRadius)

	latMin := min(lowerFromLat, lowerToLat)
	latMax := max(upperFromLat, upperToLat)

	lonMin := min(lowerFromLon, lowerToLon)
	lonMax := max(upperFromLon, upperToLon)

	// from node
	nodeBB := datastructure.NewRtreeBoundingBox(
		2, []float64{latMin, lonMin}, []float64{latMax, lonMax},
	)

	osmObj := datastructure.NewOSMObject(
		int(edge.EdgeID), fromNode.Lat, fromNode.Lon,
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
