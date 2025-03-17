package osmparser

import (
	"context"
	"io"
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/geo"
	"lintang/navigatorx/pkg/util"
	"log"
	"os"
	"strconv"
	"strings"

	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
)

type node struct {
	id    int64
	coord nodeCoord
}

type nodeCoord struct {
	lat float64
	lon float64
}

type OsmParser struct {
	wayNodeMap        map[int64]NodeType
	relationMemberMap map[int64]struct{}
	acceptedNodeMap   map[int64]nodeCoord
	barrierNodes      map[int64]bool
	nodeTag           map[int64]map[int]int
	tagStringIdMap    util.IDMap
	nodeIDMap         map[int64]int32
}

func NewOSMParserV2() *OsmParser {
	return &OsmParser{
		wayNodeMap:        make(map[int64]NodeType),
		relationMemberMap: make(map[int64]struct{}),
		acceptedNodeMap:   make(map[int64]nodeCoord),
		barrierNodes:      make(map[int64]bool),
		nodeTag:           make(map[int64]map[int]int),
		tagStringIdMap:    util.NewIdMap(),
		nodeIDMap:         make(map[int64]int32),
	}
}
func (o *OsmParser) GetTagStringIdMap() util.IDMap {
	return o.tagStringIdMap
}

var (
	skipHighway = map[string]struct{}{
		"footway":                struct{}{},
		"construction":           struct{}{},
		"cycleway":               struct{}{},
		"path":                   struct{}{},
		"pedestrian":             struct{}{},
		"busway":                 struct{}{},
		"steps":                  struct{}{},
		"bridleway":              struct{}{},
		"corridor":               struct{}{},
		"street_lamp":            struct{}{},
		"bus_stop":               struct{}{},
		"crossing":               struct{}{},
		"cyclist_waiting_aid":    struct{}{},
		"elevator":               struct{}{},
		"emergency_bay":          struct{}{},
		"emergency_access_point": struct{}{},
		"give_way":               struct{}{},
		"phone":                  struct{}{},
		"ladder":                 struct{}{},
		"milestone":              struct{}{},
		"passing_place":          struct{}{},
		"platform":               struct{}{},
		"speed_camera":           struct{}{},
		"track":                  struct{}{},
		"bus_guideway":           struct{}{},
		"speed_display":          struct{}{},
		"stop":                   struct{}{},
		"toll_gantry":            struct{}{},
		"traffic_mirror":         struct{}{},
		"traffic_signals":        struct{}{},
		"trailhead":              struct{}{},
	}

	reversed = map[string]struct{}{
		"no":         {},
		"restricted": {},
	}
)

func (p *OsmParser) Parse(mapFile string) ([]datastructure.CHNode, []datastructure.EdgeCH, map[string][2]bool,
	[]datastructure.MapMatchOsmWay) {
	mapMatchWays := make([]datastructure.MapMatchOsmWay, 0)

	f, err := os.Open(mapFile)

	if err != nil {
		log.Fatal(err)
	}
	defer f.Close()

	scanner := osmpbf.New(context.Background(), f, 0)
	// must not be parallel
	countWays := 0
	for scanner.Scan() {
		o := scanner.Object()

		tipe := o.ObjectID().Type()

		switch tipe {
		case osm.TypeWay:
			{
				way := o.(*osm.Way)
				if len(way.Nodes) < 2 {
					continue
				}

				if !acceptOsmWay(way) {
					continue
				}
				if (countWays+1)%50000 == 0 {
					log.Printf("reading openstreetmap ways: %d...", countWays+1)
				}
				countWays++

				for i, node := range way.Nodes {
					if _, ok := p.wayNodeMap[int64(node.ID)]; !ok {
						if i == 0 || i == len(way.Nodes)-1 {
							p.wayNodeMap[int64(node.ID)] = END_NODE
						} else {
							p.wayNodeMap[int64(node.ID)] = BETWEEN_NODE
						}
					} else {
						p.wayNodeMap[int64(node.ID)] = JUNCTION_NODE
					}
				}
			}
		case osm.TypeNode:
			{
			}
		case osm.TypeRelation:
			{
				relation := o.(*osm.Relation)
				for _, member := range relation.Members {
					if member.Type == osm.TypeRelation {
						continue
					}
				}

				if relation.Tags.Find("type") == "route" {
					for _, member := range relation.Members {
						p.relationMemberMap[member.Ref] = struct{}{}
					}
				}

			}
		}
	}
	scanner.Close()
	edges := []datastructure.EdgeCH{}
	f.Seek(0, io.SeekStart)
	if err != nil {
		log.Fatal(err)
	}
	scanner = osmpbf.New(context.Background(), f, 0)
	//must not be parallel
	defer scanner.Close()
	//
	streetDirection := make(map[string][2]bool)
	countWays = 0
	countNodes := 0
	for scanner.Scan() {
		o := scanner.Object()

		tipe := o.ObjectID().Type()

		switch tipe {
		case osm.TypeWay:
			{
				way := o.(*osm.Way)
				if len(way.Nodes) < 2 {
					continue
				}

				if !acceptOsmWay(way) {
					continue
				}
				if (countWays+1)%50000 == 0 {
					log.Printf("processing openstreetmap ways: %d...", countWays+1)
				}
				countWays++

				p.processWay(way, &edges, streetDirection, &mapMatchWays)
			}
		case osm.TypeNode:
			{

				if (countNodes+1)%50000 == 0 {
					log.Printf("processing openstreetmap nodes: %d...", countNodes+1)
				}
				countNodes++
				node := o.(*osm.Node)

				if _, ok := p.wayNodeMap[int64(node.ID)]; ok {
					p.acceptedNodeMap[int64(node.ID)] = nodeCoord{
						lat: node.Lat,
						lon: node.Lon,
					}
				}

				if node.Tags.Find("barrier") != "" ||
					node.Tags.Find("ford") != "" {
					p.barrierNodes[int64(node.ID)] = true
				}

				for _, tag := range node.Tags {
					if strings.Contains(tag.Key, "created_by") ||
						strings.Contains(tag.Key, "source") ||
						strings.Contains(tag.Key, "note") ||
						strings.Contains(tag.Key, "fixme") {
						continue
					}
					tagID := p.tagStringIdMap.GetID(tag.Key)
					if _, ok := p.nodeTag[int64(node.ID)]; !ok {
						p.nodeTag[int64(node.ID)] = make(map[int]int)
					}
					p.nodeTag[int64(node.ID)][tagID] = p.tagStringIdMap.GetID(tag.Value)
					if strings.Contains(tag.Value, "traffic_signals") {
						p.nodeTag[int64(node.ID)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] = 1
					}
				}

			}
		case osm.TypeRelation:
			{

			}
		}
	}

	processedNodes := make([]datastructure.CHNode, len(p.nodeIDMap)+1)

	for nodeID, nodeIDX := range p.nodeIDMap {
		coord := p.acceptedNodeMap[nodeID]
		if p.nodeTag[int64(nodeID)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] == 1 {
			processedNodes[nodeIDX] = datastructure.NewCHNode(coord.lat, coord.lon, 0, nodeIDX, true)
		} else {
			processedNodes[nodeIDX] = datastructure.NewCHNode(coord.lat, coord.lon, 0, nodeIDX, false)
		}
	}

	log.Printf("total edges: %d", len(edges))
	log.Printf("total osm way filtered: %d", len(mapMatchWays))

	return processedNodes, edges, streetDirection, mapMatchWays
}

type wayExtraInfo struct {
	oneWay  bool
	forward bool
}

func (p *OsmParser) processWay(way *osm.Way, edges *[]datastructure.EdgeCH,
	streetDirection map[string][2]bool, mapMatchWays *[]datastructure.MapMatchOsmWay) error {
	tempMap := make(map[string]string)
	name := way.Tags.Find("name")

	tempMap[STREET_NAME] = name

	refName := way.Tags.Find("ref")
	tempMap[STREET_REF] = refName

	speed := 0.0
	maxSpeed := 0.0
	highwayTypeSpeed := 0.0

	wayExtraInfoData := wayExtraInfo{}
	okvf, okmvf, okvb, okmvb := getReversedOneWay(way)
	if val := way.Tags.Find("oneway"); val != "" || okvf || okmvf || okvb || okmvb {
		wayExtraInfoData.oneWay = true
	}

	if way.Tags.Find("oneway") == "-1" || okvf || okmvf {
		// okvf / omvf = restricted/not allowed forward.
		wayExtraInfoData.forward = false

	} else if way.Tags.Find("oneway") != "-1" && !okvf && !okmvf {
		wayExtraInfoData.forward = true
	}

	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {
			streetDirection[name] = [2]bool{true, false} // {forward, backward}
		} else {
			streetDirection[name] = [2]bool{false, true}
		}
	} else {
		streetDirection[name] = [2]bool{true, true}
	}

	for _, tag := range way.Tags {
		switch tag.Key {
		case "junction":
			{
				tempMap[JUNCTION] = tag.Value
			}
		case "highway":
			{
				highwayTypeSpeed = RoadTypeMaxSpeed2(tag.Value)

				if strings.Contains(tag.Value, "link") {
					tempMap[ROAD_CLASS_LINK] = tag.Value
				} else {
					tempMap[ROAD_CLASS] = tag.Value
				}
			}
		case "lanes":
			{
				tempMap[LANES] = tag.Value
			}
		case "maxspeed":
			{
				if strings.Contains(tag.Value, "mph") {

					currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " mph", "", -1), 64)
					if err != nil {
						return err
					}
					maxSpeed = currSpeed * 1.60934
				} else if strings.Contains(tag.Value, "km/h") {
					currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " km/h", "", -1), 64)
					if err != nil {
						return err
					}
					maxSpeed = currSpeed
				} else if strings.Contains(tag.Value, "knots") {
					currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " knots", "", -1), 64)
					if err != nil {
						return err
					}
					maxSpeed = currSpeed * 1.852
				} else {
					currSpeed, err := strconv.ParseFloat(tag.Value, 64)
					if err != nil {
						return err
					}
					maxSpeed = currSpeed
				}
			}

		}

	}

	if maxSpeed == 0 {
		maxSpeed = highwayTypeSpeed
	}

	distance := 0.0 // in km
	prev := p.acceptedNodeMap[int64(way.Nodes[0].ID)]
	for i := 1; i < len(way.Nodes); i++ {
		curr := p.acceptedNodeMap[int64(way.Nodes[i].ID)]
		distance += geo.CalculateHaversineDistance(prev.lat, prev.lon, curr.lat, curr.lon)
		prev = curr
	}

	if speed == 0 && maxSpeed != 0.0 {
		speed = maxSpeed // km/h
	} else {
		speed = 35.0
	}

	firstNode := int64(-1)
	lastNode := int64(-1)
	lastNodeCoord := datastructure.NewCoordinate(0, 0)
	pointsInBetween := make([]datastructure.Coordinate, 0, len(way.Nodes))

	waySegment := []node{}
	for _, wayNode := range way.Nodes {
		nodeCoord := p.acceptedNodeMap[int64(wayNode.ID)]
		nodeData := node{
			id:    int64(wayNode.ID),
			coord: nodeCoord,
		}
		if p.isJunctionNode(int64(nodeData.id)) {
			if len(waySegment) > 1 {
				waySegment = append(waySegment, nodeData)
				p.processSegment(waySegment, tempMap, speed, edges, wayExtraInfoData)
				waySegment = []node{}
			}
			waySegment = append(waySegment, nodeData)

			if firstNode == -1 {
				firstNode = nodeData.id
			}

			lastNode = nodeData.id
			lastNodeCoord = datastructure.NewCoordinate(nodeCoord.lat, nodeCoord.lon)
		} else {
			waySegment = append(waySegment, nodeData)
		}

		if firstNode != -1 {
			pointsInBetween = append(pointsInBetween, datastructure.NewCoordinate(nodeCoord.lat, nodeCoord.lon))
		}
	}
	if len(waySegment) > 1 {
		p.processSegment(waySegment, tempMap, speed, edges, wayExtraInfoData)
	}

	if firstNode == lastNode && len(pointsInBetween) == 0 {
		// this osm way is not a roundabout & only have one node
		return nil
	}

	for i := len(pointsInBetween) - 1; i >= 0; i-- {
		currPoint := pointsInBetween[i]
		if currPoint.Lat == lastNodeCoord.Lat && currPoint.Lon == lastNodeCoord.Lon {
			break
		}
		pointsInBetween = append(pointsInBetween[:i], pointsInBetween[:i+1]...)
	}

	mapMatchWayDist := 0.0
	for i := 1; i < len(pointsInBetween); i++ {
		mapMatchWayDist += geo.CalculateHaversineDistance(pointsInBetween[i-1].Lat, pointsInBetween[i-1].Lon, pointsInBetween[i].Lat, pointsInBetween[i].Lon)
	}

	speed = speed * 1000 / 60 // km/h to m/min
	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {
			*mapMatchWays = append(*mapMatchWays,
				datastructure.NewMathMatchOsmWay(len(*mapMatchWays), pointsInBetween, p.nodeIDMap[firstNode], p.nodeIDMap[lastNode], speed, mapMatchWayDist))

		} else {
			reversedPointsInBetween := make([]datastructure.Coordinate, len(pointsInBetween))
			copy(reversedPointsInBetween, pointsInBetween)
			util.ReverseG(reversedPointsInBetween)

			*mapMatchWays = append(*mapMatchWays,
				datastructure.NewMathMatchOsmWay(len(*mapMatchWays), reversedPointsInBetween, p.nodeIDMap[lastNode], p.nodeIDMap[firstNode], speed, mapMatchWayDist))

		}
	} else {
		*mapMatchWays = append(*mapMatchWays,
			datastructure.NewMathMatchOsmWay(len(*mapMatchWays), pointsInBetween, p.nodeIDMap[firstNode], p.nodeIDMap[lastNode], speed, mapMatchWayDist))

		reversedPointsInBetween := make([]datastructure.Coordinate, len(pointsInBetween))
		copy(reversedPointsInBetween, pointsInBetween)
		util.ReverseG(reversedPointsInBetween)

		*mapMatchWays = append(*mapMatchWays,
			datastructure.NewMathMatchOsmWay(len(*mapMatchWays), reversedPointsInBetween, p.nodeIDMap[lastNode], p.nodeIDMap[firstNode], speed, mapMatchWayDist))

	}

	return nil
}

func isRestricted(value string) bool {
	if value == "no" || value == "restricted" || value == "military" || value == "emergency" || value == "private" || value == "permit" {
		return true
	}
	return false
}

func getReversedOneWay(way *osm.Way) (bool, bool, bool, bool) {
	vehicleForward := way.Tags.Find("vehicle:forward")
	motorVehicleForward := way.Tags.Find("motor_vehicle:forward")
	vehicleBackward := way.Tags.Find("vehicle:backward")
	motorVehicleBackward := way.Tags.Find("motor_vehicle:backward")
	return isRestricted(vehicleForward), isRestricted(motorVehicleForward), isRestricted(vehicleBackward), isRestricted(motorVehicleBackward)
}

func (p *OsmParser) processSegment(segment []node, tempMap map[string]string, speed float64, edges *[]datastructure.EdgeCH,
	wayExtraInfoData wayExtraInfo) {
	if len(segment) == 2 && segment[0].id == segment[1].id {
		// skip

	} else if segment[0].id == segment[len(segment)-1].id {
		// loop
		p.processSegment2(segment[0:len(segment)-1], tempMap, speed, edges, wayExtraInfoData)
		p.processSegment2(segment[len(segment)-2:], tempMap, speed, edges, wayExtraInfoData)
	} else {
		p.processSegment2(segment, tempMap, speed, edges, wayExtraInfoData)
	}
}

func (p *OsmParser) processSegment2(segment []node, tempMap map[string]string, speed float64, edges *[]datastructure.EdgeCH,
	wayExtraInfoData wayExtraInfo) {
	waySegment := []node{}
	for i := 0; i < len(segment); i++ {
		nodeData := segment[i]
		if _, ok := p.barrierNodes[int64(nodeData.id)]; ok {
			p.barrierNodes[int64(nodeData.id)] = false

			if len(waySegment) != 0 {
				waySegment = append(waySegment, nodeData)
				p.addEdge(waySegment, tempMap, speed, edges, wayExtraInfoData)
				waySegment = []node{}
			}
			waySegment = append(waySegment, nodeData)
		} else {
			waySegment = append(waySegment, nodeData)
		}
	}
	if len(waySegment) > 1 {
		p.addEdge(waySegment, tempMap, speed, edges, wayExtraInfoData)
	}
}

func (p *OsmParser) addEdge(segment []node, tempMap map[string]string, speed float64, edges *[]datastructure.EdgeCH,
	wayExtraInfoData wayExtraInfo) {
	from := segment[0]
	if _, ok := p.nodeIDMap[from.id]; !ok {
		p.nodeIDMap[from.id] = int32(len(p.nodeIDMap))
	}
	to := segment[len(segment)-1]
	if _, ok := p.nodeIDMap[to.id]; !ok {
		p.nodeIDMap[to.id] = int32(len(p.nodeIDMap))
	}

	edgePoints := []datastructure.Coordinate{}
	distance := 0.0
	for i := 0; i < len(segment); i++ {
		edgePoints = append(edgePoints, datastructure.Coordinate{
			Lat: segment[i].coord.lat,
			Lon: segment[i].coord.lon,
		})
		if i > 0 {
			distance += geo.CalculateHaversineDistance(segment[i-1].coord.lat, segment[i-1].coord.lon, segment[i].coord.lat, segment[i].coord.lon)
		}
	}

	isRoundabout := false
	if val, ok := tempMap["junction"]; ok {
		if val == "roundabout" {
			isRoundabout = true
		}
		if val == "circular" {
			isRoundabout = true
		}
	}

	distanceInMeter := distance * 1000
	etaWeight := distanceInMeter / (speed * 1000 / 60) // in minutes

	lanes, err := strconv.Atoi(tempMap[LANES])
	if err != nil {
		lanes = 2 // assume
	}
	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {
			*edges = append(*edges, datastructure.EdgeCH{
				EdgeID:          int32(len(*edges)),
				Weight:          etaWeight,
				Dist:            distanceInMeter, // meter
				ToNodeID:        p.nodeIDMap[to.id],
				FromNodeID:      p.nodeIDMap[from.id],
				StreetName:      p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				Roundabout:      isRoundabout,
				RoadClass:       p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
				RoadClassLink:   p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
				Lanes:           lanes,
				PointsInBetween: edgePoints,
			})
		} else {

			reversedEdgePoints := make([]datastructure.Coordinate, len(edgePoints))
			copy(reversedEdgePoints, edgePoints)
			util.ReverseG(reversedEdgePoints)
			*edges = append(*edges, datastructure.EdgeCH{
				EdgeID:          int32(len(*edges)),
				Weight:          etaWeight,
				Dist:            distanceInMeter, // meter
				ToNodeID:        p.nodeIDMap[from.id],
				FromNodeID:      p.nodeIDMap[to.id],
				StreetName:      p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				Roundabout:      isRoundabout,
				RoadClass:       p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
				RoadClassLink:   p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
				Lanes:           lanes,
				PointsInBetween: reversedEdgePoints,
			})
		}
	} else {
		*edges = append(*edges, datastructure.EdgeCH{
			EdgeID:          int32(len(*edges)),
			Weight:          etaWeight,
			Dist:            distanceInMeter, // meter
			ToNodeID:        p.nodeIDMap[to.id],
			FromNodeID:      p.nodeIDMap[from.id],
			StreetName:      p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			Roundabout:      isRoundabout,
			RoadClass:       p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
			RoadClassLink:   p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
			Lanes:           lanes,
			PointsInBetween: edgePoints,
		})

		reversedEdgePoints := make([]datastructure.Coordinate, len(edgePoints))
		copy(reversedEdgePoints, edgePoints)
		util.ReverseG(reversedEdgePoints)
		*edges = append(*edges, datastructure.EdgeCH{
			EdgeID:          int32(len(*edges)),
			Weight:          etaWeight,
			Dist:            distanceInMeter, // meter
			ToNodeID:        p.nodeIDMap[from.id],
			FromNodeID:      p.nodeIDMap[to.id],
			StreetName:      p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			Roundabout:      isRoundabout,
			RoadClass:       p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
			RoadClassLink:   p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
			Lanes:           lanes,
			PointsInBetween: reversedEdgePoints,
		})
	}
}

func RoadTypeMaxSpeed2(roadType string) float64 {
	switch roadType {
	case "motorway":
		return 100
	case "trunk":
		return 70
	case "primary":
		return 65
	case "secondary":
		return 60
	case "tertiary":
		return 50
	case "unclassified":
		return 30
	case "residential":
		return 30
	case "service":
		return 20
	case "motorway_link":
		return 70
	case "trunk_link":
		return 65
	case "primary_link":
		return 60
	case "secondary_link":
		return 50
	case "tertiary_link":
		return 40
	case "living_street":
		return 10
	case "road":
		return 20
	case "track":
		return 15
	default:
		return 40
	}
}

func (p *OsmParser) isJunctionNode(nodeID int64) bool {
	return p.wayNodeMap[int64(nodeID)] == JUNCTION_NODE
}

func acceptOsmWay(way *osm.Way) bool {
	highway := way.Tags.Find("highway")
	junction := way.Tags.Find("junction")
	if highway != "" {
		if _, ok := skipHighway[highway]; !ok {
			return true
		}
	} else if way.Tags.Find("route") == "road" {
		return true
	} else if junction != "" {
		return true
	}
	return false
}
