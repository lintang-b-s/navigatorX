package contractor

import (
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"lintang/navigatorx/pkg/guidance"
	"sort"
)

type NodePoint struct {
	Node datastructure.CHNode
	Dist float64
	Idx  int32
}

type SmallWay struct {
	CenterLoc           []float64 // [lat, lon]
	IntersectionNodesID []int32
	PointsInBetween     []datastructure.Coordinate
	WayID               int32
}

type NearestStreet struct {
	Dist   float64
	Street *SmallWay
}

func (ch *ContractedGraph) SnapLocationToRoadNetworkNodeH3(ways []datastructure.KVEdge, wantToSnap []float64) int32 {
	nearestStreets := []NearestStreet{}
	for i, w := range ways {
		street := ways[i]
		if len(street.IntersectionNodesID) < 1 {
			continue
		}

		nearestStreets = append(nearestStreets, NearestStreet{
			Dist: geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], w.CenterLoc[0], w.CenterLoc[1]),
			Street: &SmallWay{
				CenterLoc:           w.CenterLoc,
				IntersectionNodesID: w.IntersectionNodesID,
			},
		})
	}

	sort.Slice(nearestStreets, func(i, j int) bool {
		return nearestStreets[i].Dist < nearestStreets[j].Dist
	})

	if len(nearestStreets) >= 7 {
		nearestStreets = nearestStreets[:7]
	}

	best := 100000000.0
	snappedStNode := int32(0)

	for _, street := range nearestStreets {

		// mencari 2 point dijalan yg paling dekat dg gps
		streetNodes := []NodePoint{}
		for _, nodeID := range street.Street.IntersectionNodesID {

			node := ch.GetNode(nodeID)

			cNode := datastructure.CHNode{
				Lat:          node.Lat,
				Lon:          node.Lon,
				OrderPos:     node.OrderPos,
				ID:           node.ID,
				TrafficLight: node.TrafficLight,
			}
			streetNodes = append(streetNodes, NodePoint{cNode, geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], node.Lat, node.Lon), int32(nodeID)})
		}

		sort.Slice(streetNodes, func(i, j int) bool {
			return streetNodes[i].Dist < streetNodes[j].Dist
		})

		if len(street.Street.IntersectionNodesID) >= 2 {
			nearestStPoint := streetNodes[0].Node
			nearestStNodeIdx := streetNodes[0].Idx
			secondNearestStPoint := streetNodes[1].Node

			// project point ke line segment jalan antara 2 point tadi
			projection := guidance.ProjectPointToLineCoord(nearestStPoint, secondNearestStPoint, wantToSnap)

			// ambil streetNode yang jarak antara hasil projection dg lokasi gps  paling kecil
			if geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], projection.Lat, projection.Lon) < best {
				best = geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], projection.Lat, projection.Lon)
				snappedStNode = nearestStNodeIdx
			}
		} else {
			nearestStPoint := streetNodes[0].Node

			if geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], nearestStPoint.Lat, nearestStPoint.Lon) < best {
				best = geo.CalculateHaversineDistance(wantToSnap[0], wantToSnap[1], nearestStPoint.Lat, nearestStPoint.Lon)
				snappedStNode = nearestStPoint.ID
			}
		}

	}

	return snappedStNode
}
