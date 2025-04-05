package snap

// func TestSnap(t *testing.T) {

// 	edge := datastructure.Edge{
// 		EdgeID:     1,
// 		FromNodeID: 0,
// 		ToNodeID:   1,
// 	}

// 	node1 := datastructure.CHNode{
// 		ID:  0,
// 		Lat: 47.642563,
// 		Lon: -122.322375,
// 	}
// 	node2 := datastructure.CHNode{
// 		ID:  1,
// 		Lat: 47.642478,
// 		Lon: -122.322182,
// 	}

// 	graphStorage := datastructure.NewGraphStorage()
// 	graphStorage.AppendEdgeStorage(edge)

// 	ch := contractor.NewContractedGraph()
// 	ch.InitCHGraph([]datastructure.CHNode{node1, node2}, graphStorage, nil, util.NewIdMap())

// 	rt := datastructure.NewRtree(25, 50, 2)
// 	snap := NewRoadSnapper(rt)

// 	snap.BuildRoadSnapper(ch)

// 	qlat, qlon := 47.64145, -122.3218167

// 	edges := snap.SnapToRoads(datastructure.NewPoint(qlat, qlon))
// 	fmt.Printf("edges: %v\n", edges)
// }

// func TestSnapFromFileS(t *testing.T) {

// 	ch := contractor.NewContractedGraph()
// 	err := ch.LoadGraph()

// 	if err != nil {
// 		log.Fatal(err)
// 	}

// 	db, err := badger.Open(badger.DefaultOptions("./navigatorx_db"))
// 	if err != nil {
// 		log.Fatal(err)
// 	}

// 	kvDB := kv.NewKVDB(db)
// 	defer kvDB.Close()

// 	rtree := datastructure.NewRtree(25, 50, 2)
// 	roadSnapper := NewRoadSnapper(rtree)

// 	roadSnapper.BuildRoadSnapper(ch)

// 	qlat, qlon := 47.64145, -122.3218167

// 	edges := roadSnapper.SnapToRoads(datastructure.NewPoint(qlat, qlon))

// 	sort.Slice(edges, func(i, j int) bool {
// 		distToQuery := geo.CalculateHaversineDistance(qlat, qlon, edges[i].Lat, edges[i].Lon)
// 		distToQuery2 := geo.CalculateHaversineDistance(qlat, qlon, edges[j].Lat, edges[j].Lon)
// 		return distToQuery < distToQuery2
// 	})
// 	if len(edges) == 0 {
// 		t.Errorf("expected at least one edge, got %d", len(edges))
// 	}

// 	filteredEdges := filterEdges(edges, qlat, qlon, 0, ch)
// 	if len(filteredEdges) == 0 {
// 		t.Errorf("expected at least one edge, got %d", len(filteredEdges))
// 	}
// }

// const (
// 	radius = 300.0
// 	k      = 28
// )

// func filterEdges(edges []datastructure.OSMObject, pLat, pLon float64,
// 	obsID int, ch *contractor.ContractedGraph) []*datastructure.State {
// 	// create projection and check if dist(projection, queryPoint) < radius
// 	filteredEdges := make([]*datastructure.State, 0, len(edges))

// 	edgeSet := make(map[int32]struct{}, len(edges))

// 	for _, edgeObj := range edges {

// 		if _, ok := edgeSet[int32(edgeObj.ID)]; ok {
// 			continue
// 		}
// 		edgeSet[int32(edgeObj.ID)] = struct{}{}

// 		edgeID := int32(edgeObj.ID)

// 		edge := ch.GetOutEdge(edgeID)

// 		if math.Abs(edgeObj.Lat-47.6424759) < 0.00001 || math.Abs(edgeObj.Lon-(-122.3221824)) < 0.00001 {
// 			log.Printf("debug")
// 		}

// 		pointsInBetween := ch.GetEdgePointsInBetween(edge.EdgeID)

// 		pos := geo.PointPositionBetweenLinePoints(pLat, pLon, pointsInBetween) - 1

// 		fromPoint := datastructure.NewCoordinate(pointsInBetween[pos].Lat, pointsInBetween[pos].Lon)
// 		toPoint := datastructure.NewCoordinate(pointsInBetween[pos+1].Lat, pointsInBetween[pos+1].Lon)

// 		projection := geo.ProjectPointToLineCoord(fromPoint, toPoint, datastructure.NewCoordinate(pLat, pLon))

// 		dist := geo.CalculateHaversineDistance(projection.Lat, projection.Lon, pLat, pLon) * 1000

// 		if dist < radius {
// 			filteredEdges = append(filteredEdges, &datastructure.State{
// 				Dist:              dist,
// 				EdgeID:            edgeID,
// 				PointsInBetween:   pointsInBetween,
// 				EdgeFromNodeID:    edge.FromNodeID,
// 				EdgeToNodeID:      edge.ToNodeID,
// 				ProjectionID:      -1,
// 				Type:              datastructure.VirtualNode, // by default all virtual node
// 				ObservationID:     obsID,
// 				PerpendicularDist: dist,
// 			})
// 		}

// 	}

// 	sort.Slice(filteredEdges, func(i, j int) bool {
// 		return filteredEdges[i].Dist < filteredEdges[j].Dist
// 	})

// 	filteredEdges = filteredEdges[:minInt(len(filteredEdges), k)]

// 	for _, edge := range filteredEdges {

// 		fromNode := ch.GetNode(edge.EdgeFromNodeID)
// 		if math.Abs(fromNode.Lat-47.6424759) < 0.00001 || math.Abs(fromNode.Lon-(-122.3221824)) < 0.00001 {
// 			log.Printf("debug")
// 		}

// 	}

// 	return filteredEdges
// }

// func minInt(a, b int) int {
// 	if a < b {
// 		return a
// 	}
// 	return b
// }
