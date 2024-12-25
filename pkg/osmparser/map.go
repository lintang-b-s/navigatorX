package osmparser

import (
	"sort"
	"strconv"
	"strings"

	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"lintang/navigatorx/pkg/guidance"

	"github.com/k0kubun/go-ansi"
	"github.com/paulmach/osm"
	"github.com/schollz/progressbar/v3"
)

type Coordinate struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

var ValidRoadType = map[string]bool{
	"motorway":       true,
	"trunk":          true,
	"primary":        true,
	"secondary":      true,
	"tertiary":       true,
	"unclassified":   true,
	"residential":    true,
	"motorway_link":  true,
	"trunk_link":     true,
	"primary_link":   true,
	"secondary_link": true,
	"tertiary_link":  true,
	"living_street":  true,
	"path":           true,
	"road":           true,
	"service":        true,
	"track":          true,
}

func InitGraph(ways []*osm.Way, trafficLightNodeIdMap map[osm.NodeID]bool) ([]datastructure.SurakartaWay, []datastructure.Node,
	[]datastructure.SurakartaWay, map[string][2]bool, map[string]datastructure.StreetExtraInfo) {
	var SurakartaNodeMap = make(map[int64]*datastructure.Node)

	oneWayTypesMap := make(map[string]int64)
	twoWayTypesMap := make(map[string]int64)

	bar := progressbar.NewOptions(len(ways),
		progressbar.OptionSetWriter(ansi.NewAnsiStdout()),
		progressbar.OptionEnableColorCodes(true),
		progressbar.OptionShowBytes(true),
		progressbar.OptionSetWidth(15),
		progressbar.OptionSetDescription("[cyan][2/7][reset] saving Openstreetmap way & node  ..."),
		progressbar.OptionSetTheme(progressbar.Theme{
			Saucer:        "[green]=[reset]",
			SaucerHead:    "[green]>[reset]",
			SaucerPadding: " ",
			BarStart:      "[",
			BarEnd:        "]",
		}))

	surakartaWays := []datastructure.SurakartaWay{}
	for wayIDx, way := range ways {
		namaJalan := ""
		roadTypes := make(map[string]int)

		for _, tag := range way.Tags {
			if tag.Key == "highway" {
				twoWayTypesMap[tag.Key+"="+tag.Value] += 1
				roadTypes[tag.Value] += 1
			}

			if strings.Contains(tag.Key, "oneway") && !strings.Contains(tag.Value, "no") {
				oneWayTypesMap[tag.Key+"="+tag.Value] += 1
			}
			if tag.Key == "name" {
				namaJalan = tag.Value
			}
		}

		if !isOsmWayUsedByCars(way.TagMap()) {
			continue
		}

		sWay := datastructure.SurakartaWay{
			Nodes: make([]datastructure.Coordinate, 0),
			WayID: int32(wayIDx),
		}

		streetNodeLats := []float64{}
		streetNodeLon := []float64{}

		// creategraph node
		for i := 0; i < len(way.Nodes); i++ {
			currNode := way.Nodes[i]

			node := &datastructure.Node{
				Lat:          currNode.Lat,
				Lon:          currNode.Lon,
				ID:           int64(currNode.ID),
				StreetName:   namaJalan,
				TrafficLight: trafficLightNodeIdMap[currNode.ID],
			}

			if fromRealNode, ok := SurakartaNodeMap[node.ID]; ok {
				node = fromRealNode
				node.UsedInRoad += 1
			} else {
				node.UsedInRoad = 1
				SurakartaNodeMap[node.ID] = node
			}

			// add node ke surakartaway
			sWay.Nodes = append(sWay.Nodes, datastructure.Coordinate{
				Lat: node.Lat,
				Lon: node.Lon,
			})
			// append node lat & node lon
			streetNodeLats = append(streetNodeLats, node.Lat)
			streetNodeLon = append(streetNodeLon, node.Lon)

		}
		sort.Float64s(streetNodeLats)
		sort.Float64s(streetNodeLon)

		// https://www.movable-type.co.uk/scripts/latlong.html
		centerLat, centerLon := guidance.MidPoint(streetNodeLats[0], streetNodeLon[0], streetNodeLats[len(streetNodeLats)-1], streetNodeLon[len(streetNodeLon)-1])
		sWay.CenterLoc = []float64{centerLat, centerLon}

		sWay.ID = int32(wayIDx)
		surakartaWays = append(surakartaWays, sWay)
		bar.Add(1)
	}

	surakartaNodes, hmmEdges, graphEdges, streetDirections, streetExtraInfo := processOnlyIntersectionRoadNodes(SurakartaNodeMap, ways, surakartaWays)

	return hmmEdges, surakartaNodes, graphEdges, streetDirections, streetExtraInfo
}

func processOnlyIntersectionRoadNodes(nodeMap map[int64]*datastructure.Node, ways []*osm.Way, surakartaWays []datastructure.SurakartaWay) ([]datastructure.Node,
	[]datastructure.SurakartaWay, []datastructure.SurakartaWay, map[string][2]bool, map[string]datastructure.StreetExtraInfo) {
	surakartaNodes := []datastructure.Node{}
	alreadyAdded := make(map[int64]struct{})
	intersectionNodes := []int64{}
	streetDirection := make(map[string][2]bool)
	hmmEdges := []datastructure.SurakartaWay{}

	streetExtraInfo := make(map[string]datastructure.StreetExtraInfo)
	for wayIDx, way := range ways {
		maxSpeed, isOneWay, reversedOneWay, roadType, namaJalan, jumlahLanes, roadclassLink, roundabout, streetInfo := getMaxspeedOneWayRoadType(*way)
		if !isOsmWayUsedByCars(way.TagMap()) {
			continue
		}
		maxSpeed *= 0.8
		streetExtraInfo[namaJalan] = datastructure.StreetExtraInfo{
			Destination:      streetInfo.Destination,
			DestinationRef:   streetInfo.Destination,
			MotorwayJunction: streetInfo.MotorwayJunction}

		if !ValidRoadType[roadType] {
			continue
		}
		currSurakartaWay := &surakartaWays[wayIDx]

		startIdx := 0
		var from *datastructure.Node = nil
		for ; startIdx < len(way.Nodes); startIdx++ {
			curr := nodeMap[int64(way.Nodes[startIdx].ID)]
			if curr.UsedInRoad >= 2 {
				from = curr
				break
			}
		}

		if from == nil {
			continue
		}

		if _, ok := alreadyAdded[from.ID]; !ok {
			intersectionNodes = append(intersectionNodes, from.ID)
			alreadyAdded[from.ID] = struct{}{}
		}

		prevIdx := startIdx + 1
		for i := startIdx + 1; i < len(way.Nodes); i++ {
			currNode := way.Nodes[i]
			// idnya masih pake id osm
			to := nodeMap[int64(currNode.ID)]

			if to.UsedInRoad >= 2 {
				// nodenya ada di intersection of 2  or more roads
				// add edge antara dua node intersection
				fromLoc := geo.NewLocation(from.Lat, from.Lon)
				toLoc := geo.NewLocation(to.Lat, to.Lon)
				fromToDistance := geo.HaversineDistance(fromLoc, toLoc) * 1000 // meter
				centerLat, centerLon := guidance.MidPoint(from.Lat, from.Lon, to.Lat, to.Lon)

				nodesInBetween := []datastructure.Coordinate{}
				if isOneWay && !reversedOneWay {
					for nodeIt := prevIdx + 1; nodeIt < i; nodeIt++ {
						nodesInBetween = append(nodesInBetween, datastructure.NewCoordinate(way.Nodes[nodeIt].Lat, way.Nodes[nodeIt].Lon))
					}
					edge := datastructure.Edge{
						From:           from,
						To:             to,
						Cost:           fromToDistance,
						MaxSpeed:       maxSpeed,
						StreetName:     namaJalan,
						RoadClass:      roadType,
						RoadClassLink:  roadclassLink,
						Lanes:          jumlahLanes,
						Roundabout:     roundabout,
						NodesInBetween: nodesInBetween,
					}
					from.Out_to = append(from.Out_to, edge)
					currSurakartaWay.IntersectionNodesID = append(currSurakartaWay.IntersectionNodesID, from.ID)
					streetDirection[namaJalan] = [2]bool{true, false}

					hmmEdgeNodes := []datastructure.Coordinate{datastructure.NewCoordinate(from.Lat, from.Lon)}
					hmmEdgeNodes = append(hmmEdgeNodes, nodesInBetween...)
					hmmEdgeNodes = append(hmmEdgeNodes, datastructure.NewCoordinate(to.Lat, to.Lon))

					hmmEdges = append(hmmEdges, datastructure.SurakartaWay{
						ID:        int32(len(hmmEdges)),
						Nodes:     hmmEdgeNodes,
						WayID:     int32(wayIDx),
						CenterLoc: []float64{centerLat, centerLon},
					})
				} else if isOneWay && reversedOneWay {
					for nodeIt := i - 1; nodeIt > prevIdx; nodeIt-- {
						nodesInBetween = append(nodesInBetween, datastructure.NewCoordinate(way.Nodes[nodeIt].Lat, way.Nodes[nodeIt].Lon))
					}
					reverseEdge := datastructure.Edge{
						From:           to,
						To:             from,
						Cost:           fromToDistance,
						MaxSpeed:       maxSpeed,
						StreetName:     namaJalan,
						RoadClass:      roadType,
						RoadClassLink:  roadclassLink,
						Lanes:          jumlahLanes,
						Roundabout:     roundabout,
						NodesInBetween: nodesInBetween,
					}
					to.Out_to = append(to.Out_to, reverseEdge)
					currSurakartaWay.IntersectionNodesID = append(currSurakartaWay.IntersectionNodesID, to.ID)
					streetDirection[namaJalan] = [2]bool{false, true}

					hmmEdgeNodes := []datastructure.Coordinate{datastructure.NewCoordinate(to.Lat, to.Lon)}
					hmmEdgeNodes = append(hmmEdgeNodes, nodesInBetween...)
					hmmEdgeNodes = append(hmmEdgeNodes, datastructure.NewCoordinate(from.Lat, from.Lon))
					hmmEdges = append(hmmEdges, datastructure.SurakartaWay{
						ID:        int32(len(hmmEdges)),
						Nodes:     hmmEdgeNodes,
						WayID:     int32(wayIDx),
						CenterLoc: []float64{centerLat, centerLon},
					})

				} else {
					for nodeIt := prevIdx + 1; nodeIt < i; nodeIt++ {
						nodesInBetween = append(nodesInBetween, datastructure.NewCoordinate(way.Nodes[nodeIt].Lat, way.Nodes[nodeIt].Lon))
					}
					edge := datastructure.Edge{
						From:           from,
						To:             to,
						Cost:           fromToDistance,
						MaxSpeed:       maxSpeed,
						StreetName:     namaJalan,
						RoadClass:      roadType,
						RoadClassLink:  roadclassLink,
						Lanes:          jumlahLanes,
						Roundabout:     roundabout,
						NodesInBetween: nodesInBetween,
					}
					from.Out_to = append(from.Out_to, edge)
					hmmEdgeNodes := []datastructure.Coordinate{datastructure.NewCoordinate(from.Lat, from.Lon)}
					hmmEdgeNodes = append(hmmEdgeNodes, nodesInBetween...)
					hmmEdgeNodes = append(hmmEdgeNodes, datastructure.NewCoordinate(to.Lat, to.Lon))

					hmmEdges = append(hmmEdges, datastructure.SurakartaWay{
						ID:        int32(len(hmmEdges)),
						Nodes:     hmmEdgeNodes,
						WayID:     int32(wayIDx),
						CenterLoc: []float64{centerLat, centerLon},
					})

					nodesInBetween = []datastructure.Coordinate{}
					for nodeIt := i - 1; nodeIt > prevIdx; nodeIt-- {
						nodesInBetween = append(nodesInBetween, datastructure.NewCoordinate(way.Nodes[nodeIt].Lat, way.Nodes[nodeIt].Lon))
					}
					reverseEdge := datastructure.Edge{
						From:           to,
						To:             from,
						Cost:           fromToDistance,
						MaxSpeed:       maxSpeed,
						StreetName:     roadType,
						RoadClass:      roadType,
						RoadClassLink:  roadclassLink,
						Lanes:          jumlahLanes,
						Roundabout:     roundabout,
						NodesInBetween: nodesInBetween,
					}
					to.Out_to = append(to.Out_to, reverseEdge)
					currSurakartaWay.IntersectionNodesID = append(currSurakartaWay.IntersectionNodesID, from.ID)
					currSurakartaWay.IntersectionNodesID = append(currSurakartaWay.IntersectionNodesID, to.ID)
					streetDirection[namaJalan] = [2]bool{true, true}

					hmmEdgeNodes = []datastructure.Coordinate{datastructure.NewCoordinate(to.Lat, to.Lon)}
					hmmEdgeNodes = append(hmmEdgeNodes, nodesInBetween...)
					hmmEdgeNodes = append(hmmEdgeNodes, datastructure.NewCoordinate(from.Lat, from.Lon))

					hmmEdges = append(hmmEdges, datastructure.SurakartaWay{
						ID:        int32(len(hmmEdges)),
						Nodes:     hmmEdgeNodes,
						WayID:     int32(wayIDx),
						CenterLoc: []float64{centerLat, centerLon},
					})
				}
				if _, ok := alreadyAdded[to.ID]; !ok {
					intersectionNodes = append(intersectionNodes, to.ID)
					alreadyAdded[to.ID] = struct{}{}
				}
				from = to
				prevIdx = i
			}

		}
	}

	for _, node := range intersectionNodes {

		// hanya append node intersection
		surakartaNodes = append(surakartaNodes, *nodeMap[node])
	}
	graphEdges := []datastructure.SurakartaWay{}
	for _, way := range surakartaWays {

		if len(way.IntersectionNodesID) > 0 {
			graphEdges = append(graphEdges, way)
		}
	}

	return surakartaNodes, hmmEdges, graphEdges, streetDirection, streetExtraInfo
}

func getMaxspeedOneWayRoadType(way osm.Way) (float64, bool, bool, string, string, int, string, bool, datastructure.StreetExtraInfo) {
	jumlahLanes := 0

	maxSpeed := 40.0

	isOneWay := false // 0, 1
	reversedOneWay := false

	roadTypes := make(map[string]int)
	namaJalan := ""

	roadType := ""
	roadClassLink := ""
	roundAbout := false
	streetInfo := datastructure.StreetExtraInfo{}

	for _, tag := range way.Tags {
		if tag.Key == "highway" && !strings.Contains(tag.Value, "link") {
			roadTypes[tag.Value] += 1
			roadType = tag.Value
		}
		if strings.Contains(tag.Key, "oneway") && !strings.Contains(tag.Value, "no") {
			isOneWay = true
			if strings.Contains(tag.Value, "-1") {
				reversedOneWay = true
			}
		}
		if strings.Contains(tag.Key, "maxspeed") {
			_, err := strconv.ParseFloat(tag.Value, 64)
			if err != nil {
				maxSpeed, _ = strconv.ParseFloat(tag.Value, 64)
			}
		}

		if tag.Key == "highway" && strings.Contains(tag.Value, "link") {
			roadClassLink = tag.Value
		}

		if strings.Contains(tag.Value, "roundabout") {
			roundAbout = true
		}

		if tag.Key == "name" {
			namaJalan = tag.Value
		}

		if tag.Key == "lanes" {
			jumlahLanes, _ = strconv.Atoi(tag.Value)
		}
		if tag.Key == "destination" {
			streetInfo.Destination = tag.Value
		}
		if tag.Key == "destination:ref" {
			streetInfo.DestinationRef = tag.Value
		}
		if (tag.Key == "highway" && tag.Value == "motorway") || (tag.Key == "highway" && tag.Value == "motorway_link") {
			for _, tag2 := range way.Tags {
				if tag2.Key == "highway" && tag2.Value == "motorway_junction" {
					streetInfo.MotorwayJunction = tag.Value
				}
			}
		}

	}
	if maxSpeed == 40.0 || maxSpeed == 0 {
		maxSpeed = datastructure.RoadTypeMaxSpeed(roadType)
	}
	return maxSpeed, isOneWay, reversedOneWay, roadType, namaJalan, jumlahLanes, roadClassLink, roundAbout, streetInfo
}
