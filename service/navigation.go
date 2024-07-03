package service

import (
	"context"
	"lintang/coba_osm/alg"
	"lintang/coba_osm/util"
	"sort"

	"github.com/dhconnelly/rtreego"
)

type NavigationService struct {
}

func NewNavigationService() *NavigationService {
	return &NavigationService{}
}

func (uc *NavigationService) ShortestPath(ctx context.Context, srcLat, srcLon float64,
	dstLat float64, dstLon float64) (string, float64, bool, []alg.Coordinate, error) {

	from := &alg.Node{
		Lat: util.RoundFloat(srcLat, 6),
		Lon: util.RoundFloat(srcLon, 6),
	}
	to := &alg.Node{
		Lat: util.RoundFloat(dstLat, 6),
		Lon: util.RoundFloat(dstLon, 6),
	}

	var err error
	fromSurakartaNode, err := SnapLocationToRoadNetworkNodeRtree(from.Lat, from.Lon)
	if err != nil {
		// render.Render(w, r, ErrInvalidRequest(errors.New("internal server error")))
		return "", 0, false, []alg.Coordinate{}, nil
	}
	toSurakartaNode, err := SnapLocationToRoadNetworkNodeRtree(to.Lat, to.Lon)
	if err != nil {
		// render.Render(w, r, ErrInvalidRequest(errors.New("internal server error")))
		return "", 0, false, []alg.Coordinate{}, nil
	}

	if fromSurakartaNode == nil || toSurakartaNode == nil {
		// render.Render(w, r, ErrInvalidRequest(errors.New("node not found")))
		return "", 0, false, []alg.Coordinate{}, nil
	}

	p, dist, found := alg.SorthestPath(fromSurakartaNode, toSurakartaNode)
	var route []alg.Coordinate = make([]alg.Coordinate, 0)
	for i := range p {
		pathN := p[len(p)-1-i].(*alg.Node)
		route = append(route, alg.Coordinate{
			Lat: pathN.Lat,
			Lon: pathN.Lon,
		})
	}

	return alg.RenderPath(p), dist * 100, found, route, nil
}

type NodePoint struct {
	Node *alg.Node
	Dist float64
}

func SnapLocationToRoadNetworkNodeRtree(lat, lon float64) (snappedRoadNode *alg.Node, err error) {
	wantToSnap := rtreego.Point{lat, lon}
	stNeighbors := alg.StRTree.NearestNeighbors(3, wantToSnap)

	wantToSnapLoc := alg.NewLocation(wantToSnap[0], wantToSnap[1])

	snappedStNode := &alg.Node{}
	best := 100000000.0

	// snap point ke  node jalan terdekat/posisi location seharusnya
	for _, st := range stNeighbors {
		street := st.(*alg.StreetRect).Street
		nearestStPoint := street.Nodes[0]       // node di jalan yg paling dekat dg gps
		secondNearestStPoint := street.Nodes[0] // node di jalan yang paling dekat kedua dg gps

		// mencari 2 point dijalan yg paling dekat dg gps
		streetNodes := []NodePoint{}
		for _, node := range street.Nodes {
			nodeLoc := alg.NewLocation(node.Lat, node.Lon)
			streetNodes = append(streetNodes, NodePoint{node, alg.HaversineDistance(wantToSnapLoc, nodeLoc)})
		}

		sort.Slice(streetNodes, func(i, j int) bool {
			return streetNodes[i].Dist < streetNodes[j].Dist
		})

		nearestStPoint = streetNodes[0].Node
		secondNearestStPoint = streetNodes[1].Node

		// project point ke line segment jalan antara 2 point tadi
		projection := alg.ProjectPointToLine(*nearestStPoint, *secondNearestStPoint, wantToSnap)

		projectionLoc := alg.NewLocation(projection.Lat, projection.Lon)

		// ambil streetNode yang jarak antara hasil projection dg lokasi gps  paling kecil
		if alg.HaversineDistance(wantToSnapLoc, projectionLoc) < best {
			best = alg.HaversineDistance(wantToSnapLoc, projectionLoc)
			snappedStNode = nearestStPoint
		}
	}

	return snappedStNode, nil
}

// func SnapLocationToRoadNetworkNode(lat, lon float64) (snappedLat, snappedLon float64, err error) {
// 	timeout := 2000 * time.Millisecond
// 	client := httpclient.NewClient(httpclient.WithHTTPTimeout(timeout))

// 	mapMatchBody := MapMatchingRequest{
// 		Shape: []ShapeReq{
// 			{
// 				Lat:  lat,
// 				Lon:  lon,
// 				Type: "break",
// 			},
// 			{
// 				Lat:  lat + 0.0000000001,
// 				Lon:  lon + 0.0000000001,
// 				Type: "via",
// 			},
// 		},
// 		Costing:    "auto",
// 		ShapeMatch: "map_snap",
// 	}
// 	bodyBytes, _ := json.Marshal(&mapMatchBody)
// 	reader := bytes.NewReader(bodyBytes)

// 	res, err := client.Post("http://localhost:8002/trace_attributes?json", reader, http.Header{})

// 	// res, err := http.Post("http://localhost:8002/trace_attributes?json", "application/json", reader)
// 	if err != nil {
// 		err = errors.New("internal server error")
// 		return
// 	}
// 	var errorValhalla = &ValhallaErrorResp{}
// 	if res.StatusCode == 400 {
// 		err = json.NewDecoder(res.Body).Decode(errorValhalla)
// 		if err != nil {
// 			err = errors.New("internal server error")
// 			return
// 		}
// 	}
// 	fmt.Println(errorValhalla)
// 	defer res.Body.Close()

// 	matchedPoints := &MapMatchingResponse{}
// 	err = json.NewDecoder(res.Body).Decode(matchedPoints)
// 	if err != nil {
// 		err = errors.New("internal server error")
// 		return
// 	}

// 	snappedLat = matchedPoints.MatchedPoints[0].Lat
// 	snappedLon = matchedPoints.MatchedPoints[0].Lon
// 	return
// }

// func (uc *NavigationService) ShortestPath(ctx context.Context, srcLat, srcLon float64,
// 	dstLat float64, dstLon float64) (string, float64, bool, []alg.Coordinate, error) {

// 	from := &alg.Node{
// 		Lat: util.RoundFloat(srcLat, 6),
// 		Lon: util.RoundFloat(srcLon, 6),
// 	}
// 	to := &alg.Node{
// 		Lat: util.RoundFloat(dstLat, 6),
// 		Lon: util.RoundFloat(dstLon, 6),
// 	}

// 	var err error
// 	from.Lat, from.Lon, err = SnapLocationToRoadNetworkNode(from.Lat, from.Lon)
// 	if err != nil {
// 		// render.Render(w, r, ErrInvalidRequest(errors.New("internal server error")))
// 		return "", 0, false, []alg.Coordinate{}, nil
// 	}
// 	to.Lat, to.Lon, err = SnapLocationToRoadNetworkNode(to.Lat, to.Lon)
// 	if err != nil {
// 		// render.Render(w, r, ErrInvalidRequest(errors.New("internal server error")))
// 		return "", 0, false, []alg.Coordinate{}, nil
// 	}

// 	var fromSurakartaNode *alg.Node = alg.SurakartaGraphData.Nodes[0]
// 	var toSurakartaNode *alg.Node = alg.SurakartaGraphData.Nodes[0]
// 	for _, n := range alg.SurakartaGraphData.Nodes {

// 		for i := 6; i >= 3; i-- {
// 			if util.RoundFloat(n.Lat, uint(i)) == util.RoundFloat(from.Lat, uint(i)) && util.RoundFloat(n.Lon, uint(i)) == util.RoundFloat(from.Lon, uint(i)) {
// 				if fromSurakartaNode != nil &&
// 					math.Abs(n.Lat-from.Lat) > math.Abs(fromSurakartaNode.Lat-from.Lat) &&
// 					math.Abs(n.Lon-from.Lon) > math.Abs(fromSurakartaNode.Lon-from.Lon) {
// 					// node graph (n) saat ini gak lebih dekat dg lokasi from
// 					continue
// 				}
// 				fromSurakartaNode = n
// 				break
// 			}
// 		}

// 		for i := 6; i >= 3; i-- {
// 			if util.RoundFloat(n.Lat, uint(i)) == util.RoundFloat(to.Lat, uint(i)) && util.RoundFloat(n.Lon, uint(i)) == util.RoundFloat(to.Lon, uint(i)) {
// 				if toSurakartaNode != nil &&
// 					math.Abs(n.Lat-to.Lat) > math.Abs(toSurakartaNode.Lat-to.Lat) &&
// 					math.Abs(n.Lon-to.Lon) > math.Abs(toSurakartaNode.Lon-to.Lon) {
// 					// node graph (n) saat ini gak lebih dekat dg lokasi to
// 					continue
// 				}
// 				toSurakartaNode = n
// 				break
// 			}
// 		}

// 	}

// 	if fromSurakartaNode == nil || toSurakartaNode == nil {
// 		// render.Render(w, r, ErrInvalidRequest(errors.New("node not found")))
// 		return "", 0, false, []alg.Coordinate{}, nil
// 	}

// 	p, dist, found := alg.SorthestPath(fromSurakartaNode, toSurakartaNode)
// 	var route []alg.Coordinate = make([]alg.Coordinate, 0)
// 	for i := range p {
// 		pathN := p[len(p)-1-i].(*alg.Node)
// 		route = append(route, alg.Coordinate{
// 			Lat: pathN.Lat,
// 			Lon: pathN.Lon,
// 		})
// 	}

// 	return alg.RenderPath(p), dist, found, route, nil
// }

// type ShapeReq struct {
// 	Lat  float64 `json:"lat"`
// 	Lon  float64 `json:"lon"`
// 	Type string  `json:"type"`
// }

// // buat map matching valhalla
// type MapMatchingRequest struct {
// 	Shape      []ShapeReq `json:"shape"`
// 	Costing    string     `json:"costing"`
// 	ShapeMatch string     `json:"shape_match"`
// }

// type MapMatchingResponse struct {
// 	MatchedPoints []ShapeReq `json:"matched_points"`
// }

// type ValhallaErrorResp struct {
// 	ErrorCode int    `json:"error_code"`
// 	Error     string `json:"error"`
// }
