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

func (rs *RoadSnapper) SnapToRoads(p datastructure.Point) []datastructure.OSMObject {
	var nearestEdges []datastructure.OSMObject

	radius := 0.1
	nearestEdges = rs.rtree.NearestNeighboursRadiusDifferentStreetID(p, radius)

	counter := 0

	for counter < 5 && len(nearestEdges) == 0 {
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

	for counter < 5 && len(nearestEdges) < k {
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

// yang seharusnya diinsert di r-tree itu osm way bukan graph edge. biar candidate states setiap observation lebih dikit
// & match candidatesnya lebih akurat.
func (rs *RoadSnapper) BuildRoadSnapper(ways []datastructure.MapMatchOsmWay, ch *contractor.ContractedGraph) {

	duplicateEdges := make(map[int]map[int]struct{})

	for idx, way := range ways {

		if _, ok := duplicateEdges[int(way.FromNodeID)]; !ok {
			duplicateEdges[int(way.FromNodeID)] = make(map[int]struct{})
		}

		if _, ok := duplicateEdges[int(way.FromNodeID)][int(way.ToNodeID)]; ok {
			continue
		}

		if (idx+1)%10000 == 0 {
			log.Printf("insert osm way id %d to r-tree...", idx+1)
		}

		rs.insertEdgeToRtree(way, ch)

		duplicateEdges[int(way.FromNodeID)][int(way.ToNodeID)] = struct{}{}
	}
}

// insertEdgeToRtree insert filtered osm ways (road segments) to rtree
func (rs *RoadSnapper) insertEdgeToRtree(way datastructure.MapMatchOsmWay, ch *contractor.ContractedGraph) {
	tag := make(map[int]int)
	tag[0] = int(way.ID)

	fromNode := ch.GetNode(way.FromNodeID)

	// mid node
	nodeBB := datastructure.NewRtreeBoundingBox(
		2, []float64{fromNode.Lat - 0.001, fromNode.Lon - 0.001}, []float64{fromNode.Lat + 0.001, fromNode.Lon + 0.001},
	)

	osmObj := datastructure.NewOSMObject(
		int(way.ID), fromNode.Lat, fromNode.Lon,
		tag, nodeBB,
	)

	rs.rtree.InsertLeaf(nodeBB, osmObj, false)

	// to node
	toNode := ch.GetNode(way.ToNodeID)

	nodeBB = datastructure.NewRtreeBoundingBox(
		2, []float64{toNode.Lat - 0.001, toNode.Lon - 0.001}, []float64{toNode.Lat + 0.001, toNode.Lon + 0.001},
	)

	osmObj = datastructure.NewOSMObject(
		int(way.ID), toNode.Lat, toNode.Lon,
		tag, nodeBB,
	)

	rs.rtree.InsertLeaf(nodeBB, osmObj, false)
}
