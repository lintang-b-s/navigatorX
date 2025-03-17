package contractor

import (
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"lintang/navigatorx/pkg/guidance"
	"sort"
)

type NodePoint struct {
	Node     datastructure.CHNode
	Dist     float64
	Idx      int32
	Location geo.Location
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

func (ch *ContractedGraph) SnapLocationToRoadNetworkNodeH3(ways []datastructure.SmallWay, wantToSnap []float64) int32 {
	nearestStreets := []NearestStreet{}
	for i, w := range ways {
		street := ways[i]
		if len(street.IntersectionNodesID) < 1 {
			continue
		}

		homeLoc := geo.NewLocation(wantToSnap[0], wantToSnap[1])
		st := geo.NewLocation(w.CenterLoc[0], w.CenterLoc[1])
		nearestStreets = append(nearestStreets, NearestStreet{
			Dist: geo.HaversineDistance(homeLoc, st),
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

	wantToSnapLoc := geo.NewLocation(wantToSnap[0], wantToSnap[1])
	best := 100000000.0
	snappedStNode := int32(0)

	for _, street := range nearestStreets {

		// mencari 2 point dijalan yg paling dekat dg gps
		streetNodes := []NodePoint{}
		for _, nodeID := range street.Street.IntersectionNodesID {

			node := ch.GetNode(nodeID)
			nodeLoc := geo.NewLocation(node.Lat, node.Lon)
			cNode := datastructure.CHNode{
				Lat:          node.Lat,
				Lon:          node.Lon,
				OrderPos:     node.OrderPos,
				ID:           node.ID,
				TrafficLight: node.TrafficLight,
			}
			streetNodes = append(streetNodes, NodePoint{cNode, geo.HaversineDistance(wantToSnapLoc, nodeLoc), int32(nodeID), nodeLoc})
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
			projectionLoc := geo.NewLocation(projection.Lat, projection.Lon)
			// ambil streetNode yang jarak antara hasil projection dg lokasi gps  paling kecil
			if geo.HaversineDistance(wantToSnapLoc, projectionLoc) < best {
				best = geo.HaversineDistance(wantToSnapLoc, projectionLoc)
				snappedStNode = nearestStNodeIdx
			}
		} else {
			nearestStPoint := streetNodes[0].Node
			nearestStPointLoc := geo.NewLocation(nearestStPoint.Lat, nearestStPoint.Lon)
			if geo.HaversineDistance(wantToSnapLoc, nearestStPointLoc) < best {
				best = geo.HaversineDistance(wantToSnapLoc, nearestStPointLoc)
				snappedStNode = nearestStPoint.ID
			}
		}

	}

	return snappedStNode
}

type State struct {
	ID             int
	Lat            float64
	Lon            float64
	Dist           float64
	EdgeID         int32
	NodesInBetween []datastructure.Coordinate
}

func (ch *ContractedGraph) SnapLocationToRoadNetworkNodeRtreeForMapMatching(nearestNodes []datastructure.OSMObject, wantToSnap []float64,
	way map[int32]datastructure.SmallWay) []datastructure.State {
	stsData := make([]datastructure.State, len(nearestNodes))

	for i, node := range nearestNodes {
		dist := geo.HaversineDistance(geo.NewLocation(wantToSnap[0], wantToSnap[1]), geo.NewLocation(node.Lat, node.Lon))
		nodeWayID := int32(node.ID)
		stsData[i] = datastructure.State{
			Lat:             node.Lat,
			Lon:             node.Lon,
			Dist:            dist,
			EdgeID:          nodeWayID,
			PointsInBetween: way[nodeWayID].PointsInBetween,
		}
	}

	return stsData
}

func (ch *ContractedGraph) SnapLocationToRoadNetworkNodeH3ForMapMatching(ways []datastructure.SmallWay, wantToSnap []float64) []datastructure.State {

	sts := []State{}
	nearestStreets := []NearestStreet{}
	for _, w := range ways {

		homeLoc := geo.NewLocation(wantToSnap[0], wantToSnap[1])
		st := geo.NewLocation(w.CenterLoc[0], w.CenterLoc[1])
		nearestStreets = append(nearestStreets, NearestStreet{
			Dist: geo.HaversineDistance(homeLoc, st),
			Street: &SmallWay{
				CenterLoc:       w.CenterLoc,
				WayID:           w.WayID,
				PointsInBetween: w.PointsInBetween,
			},
		})
	}

	sort.Slice(nearestStreets, func(i, j int) bool {
		return nearestStreets[i].Dist < nearestStreets[j].Dist
	})

	if len(nearestStreets) >= 4 {
		nearestStreets = nearestStreets[:4]
	}

	wantToSnapLoc := geo.NewLocation(wantToSnap[0], wantToSnap[1])

	for _, st := range nearestStreets {

		street := st.Street

		// mencari 2 point dijalan yg paling dekat dg gps
		streetNodes := []NodePoint{}
		if len(street.PointsInBetween) < 2 {
			continue
		}
		for _, node := range street.PointsInBetween {

			nodeLoc := geo.NewLocation(node.Lat, node.Lon)
			chNode := datastructure.NewCHNode(node.Lat, node.Lon, 0, 0, false)
			streetNodes = append(streetNodes, NodePoint{chNode, geo.HaversineDistance(wantToSnapLoc, nodeLoc), -1, nodeLoc})
		}

		sort.Slice(streetNodes, func(i, j int) bool {
			return streetNodes[i].Dist < streetNodes[j].Dist
		})

		sts = append(sts, State{
			Lat:            streetNodes[0].Node.Lat,
			Lon:            streetNodes[0].Node.Lon,
			Dist:           geo.HaversineDistance(wantToSnapLoc, streetNodes[0].Location),
			EdgeID:         street.WayID,
			NodesInBetween: street.PointsInBetween,
		})
	}

	for i := len(sts) - 1; i >= 0; i-- {
		if sts[i].Dist*1000 >= 20 {
			sts[i] = sts[len(sts)-1]
			sts = sts[:len(sts)-1]
		}
	}

	stsData := make([]datastructure.State, len(sts))
	for i, st := range sts {
		stsData[i] = datastructure.State{
			StateID:         st.ID,
			Lat:             st.Lat,
			Lon:             st.Lon,
			Dist:            st.Dist,
			EdgeID:          st.EdgeID,
			PointsInBetween: st.NodesInBetween,
		}
	}

	return stsData
}
