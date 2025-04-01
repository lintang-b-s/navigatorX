package osmparser

import (
	"context"
	"io"
	"log"
	"os"
	"strconv"
	"strings"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx/pkg/geo"
	"github.com/lintang-b-s/navigatorx/pkg/util"

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

	// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Telenav
	acceptedHighway = map[string]struct{}{
		"motorway":         struct{}{},
		"motorway_link":    struct{}{},
		"trunk":            struct{}{},
		"trunk_link":       struct{}{},
		"primary":          struct{}{},
		"primary_link":     struct{}{},
		"secondary":        struct{}{},
		"secondary_link":   struct{}{},
		"residential":      struct{}{},
		"residential_link": struct{}{},
		"service":          struct{}{},
		"tertiary":         struct{}{},
		"tertiary_link":    struct{}{},
		"road":             struct{}{},
		"track":            struct{}{},
		"unclassified":     struct{}{},
		"undefined":        struct{}{},
		"unknown":          struct{}{},
		"living_street":    struct{}{},
		"private":          struct{}{},
	}
)

func (p *OsmParser) Parse(mapFile string) ([]datastructure.CHNode, []datastructure.EdgeCH, map[string][2]bool, 
	[]datastructure.EdgeExtraInfo,*datastructure.NodeInfo,
) {

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
	edges := make([]datastructure.EdgeCH, 0)

	edgesExtraInfo := make([]datastructure.EdgeExtraInfo, 0)
	edgeSet := make(map[int32]map[int32]struct{})

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

				p.processWay(way, &edges, streetDirection, &edgesExtraInfo, edgeSet)
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

	nodeInfo := datastructure.NewNodeInfo()

	for nodeID, nodeIDX := range p.nodeIDMap {
		coord := p.acceptedNodeMap[nodeID]
		if p.nodeTag[int64(nodeID)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] == 1 {
			processedNodes[nodeIDX] = datastructure.NewCHNode(coord.lat, coord.lon, 0, nodeIDX)
			nodeInfo.SetTrafficLight(nodeIDX)
		} else {
			processedNodes[nodeIDX] = datastructure.NewCHNode(coord.lat, coord.lon, 0, nodeIDX)
		}
	}

	log.Printf("total edges: %d", len(edges))

	return processedNodes, edges, streetDirection, edgesExtraInfo, nodeInfo
}

type wayExtraInfo struct {
	oneWay  bool
	forward bool
}

func (p *OsmParser) processWay(way *osm.Way, edges *[]datastructure.EdgeCH,
	streetDirection map[string][2]bool, edgesExtraInfo *[]datastructure.EdgeExtraInfo,
	edgeSet map[int32]map[int32]struct{}) error {
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

	junctionNodes := make([]int32, 0, len(way.Nodes))

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
				p.processSegment(waySegment, tempMap, speed, edges, edgesExtraInfo, wayExtraInfoData,
					edgeSet)
				waySegment = []node{}
			}
			waySegment = append(waySegment, nodeData)

			junctionNodes = append(junctionNodes, p.nodeIDMap[nodeData.id])
		} else {
			waySegment = append(waySegment, nodeData)
		}

	}
	if len(waySegment) > 1 {
		p.processSegment(waySegment, tempMap, speed, edges, edgesExtraInfo, wayExtraInfoData, edgeSet)
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
	edgesExtraInfo *[]datastructure.EdgeExtraInfo, wayExtraInfoData wayExtraInfo, edgeSet map[int32]map[int32]struct{}) {

	if len(segment) == 2 && segment[0].id == segment[1].id {
		// skip

	} else if segment[0].id == segment[len(segment)-1].id {
		// loop
		p.processSegment2(segment[0:len(segment)-1], tempMap, speed, edges, edgesExtraInfo, wayExtraInfoData, edgeSet)
		p.processSegment2(segment[len(segment)-2:], tempMap, speed, edges, edgesExtraInfo, wayExtraInfoData, edgeSet)
	} else {
		p.processSegment2(segment, tempMap, speed, edges, edgesExtraInfo, wayExtraInfoData, edgeSet)
	}
}

func (p *OsmParser) processSegment2(segment []node, tempMap map[string]string, speed float64, edges *[]datastructure.EdgeCH,
	edgesExtraInfo *[]datastructure.EdgeExtraInfo, wayExtraInfoData wayExtraInfo, edgeSet map[int32]map[int32]struct{}) {
	waySegment := []node{}
	for i := 0; i < len(segment); i++ {
		nodeData := segment[i]
		if _, ok := p.barrierNodes[int64(nodeData.id)]; ok {
			p.barrierNodes[int64(nodeData.id)] = false

			if len(waySegment) != 0 {
				waySegment = append(waySegment, nodeData)
				p.addEdge(waySegment, tempMap, speed, edges, edgesExtraInfo, wayExtraInfoData, edgeSet)
				waySegment = []node{}
			}
			waySegment = append(waySegment, nodeData)
		} else {
			waySegment = append(waySegment, nodeData)
		}
	}
	if len(waySegment) > 1 {
		p.addEdge(waySegment, tempMap, speed, edges, edgesExtraInfo, wayExtraInfoData, edgeSet)
	}
}

func (p *OsmParser) addEdge(segment []node, tempMap map[string]string, speed float64, edges *[]datastructure.EdgeCH,
	edgesExtraInfo *[]datastructure.EdgeExtraInfo, wayExtraInfoData wayExtraInfo, edgeSet map[int32]map[int32]struct{}) {
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

	if from == to {
		return
	}

	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {
			if _, ok := edgeSet[p.nodeIDMap[from.id]]; !ok {
				edgeSet[p.nodeIDMap[from.id]] = make(map[int32]struct{})
			}

			if _, ok := edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]]; ok {
				return
			}

			*edgesExtraInfo = append(*edgesExtraInfo, datastructure.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
				lanes,
				p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
				edgePoints, isRoundabout, false,
			),
			)

			*edges = append(*edges, datastructure.EdgeCH{
				EdgeID:     int32(len(*edges)),
				Weight:     etaWeight,
				Dist:       distanceInMeter, // meter
				ToNodeID:   p.nodeIDMap[to.id],
				FromNodeID: p.nodeIDMap[from.id],
			})

			edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]] = struct{}{}

		} else {
			if _, ok := edgeSet[p.nodeIDMap[to.id]]; !ok {
				edgeSet[p.nodeIDMap[to.id]] = make(map[int32]struct{})
			}

			if _, ok := edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]]; ok {
				return
			}

			edgePoints = util.ReverseG(edgePoints)

			*edgesExtraInfo = append(*edgesExtraInfo, datastructure.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
				lanes,
				p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
				edgePoints, isRoundabout, false),
			)

			*edges = append(*edges, datastructure.EdgeCH{
				EdgeID:     int32(len(*edges)),
				Weight:     etaWeight,
				Dist:       distanceInMeter, // meter
				ToNodeID:   p.nodeIDMap[from.id],
				FromNodeID: p.nodeIDMap[to.id],
			})

			edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]] = struct{}{}
		}
	} else {
		if _, ok := edgeSet[p.nodeIDMap[from.id]]; !ok {
			edgeSet[p.nodeIDMap[from.id]] = make(map[int32]struct{})
		}

		if _, ok := edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]]; ok {
			return
		}

		*edgesExtraInfo = append(*edgesExtraInfo, datastructure.NewEdgeExtraInfo(
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
			lanes,
			p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
			edgePoints, isRoundabout, false),
		)

		*edges = append(*edges, datastructure.EdgeCH{
			EdgeID:     int32(len(*edges)),
			Weight:     etaWeight,
			Dist:       distanceInMeter, // meter
			ToNodeID:   p.nodeIDMap[to.id],
			FromNodeID: p.nodeIDMap[from.id],
		})

		// reversed edge
		if _, ok := edgeSet[p.nodeIDMap[to.id]]; !ok {
			edgeSet[p.nodeIDMap[to.id]] = make(map[int32]struct{})
		}

		edgePoints = util.ReverseG(edgePoints)
		*edgesExtraInfo = append(*edgesExtraInfo, datastructure.NewEdgeExtraInfo(
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
			lanes,
			p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
			edgePoints, isRoundabout, false),
		)

		*edges = append(*edges, datastructure.EdgeCH{
			EdgeID:     int32(len(*edges)),
			Weight:     etaWeight,
			Dist:       distanceInMeter, // meter
			ToNodeID:   p.nodeIDMap[from.id],
			FromNodeID: p.nodeIDMap[to.id],
		})

		edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]] = struct{}{}
		edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]] = struct{}{}
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
	} else if junction != "" {
		return true
	}
	return false
}
