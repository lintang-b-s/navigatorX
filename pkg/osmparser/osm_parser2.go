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
	maxNodeID         int64
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
		"motorroad":        struct{}{},
	}

	//https://wiki.openstreetmap.org/wiki/Key:barrier
	// for splitting street segment to 2 disconnected graph edge
	// if the access tag of the barrier node is != "no" , we dont split the segment
	// for example, at the barrier at the entrance to FMIPA UGM, where entry is only allowed after 16.00 WIB or before 8.00 wib. (https://www.openstreetmap.org/node/8837559088#map=19/-7.767125/110.375436&layers=N)

	acceptedBarrierType = map[string]struct{}{
		"bollard":    struct{}{},
		"swing_gate": struct{}{},

		"jersey_barrier": struct{}{},
		"lift_gate":      struct{}{},
		"block":          struct{}{},
		"gate":           struct{}{},
	}
)

func (p *OsmParser) Parse(mapFile string) ([]datastructure.CHNode, *datastructure.GraphStorage, map[string][2]bool,

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

	graphStorage := datastructure.NewGraphStorage()

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

				p.processWay(way, graphStorage, streetDirection, edgeSet)
			}
		case osm.TypeNode:
			{

				if (countNodes+1)%50000 == 0 {
					log.Printf("processing openstreetmap nodes: %d...", countNodes+1)
				}
				countNodes++
				node := o.(*osm.Node)

				p.maxNodeID = max(p.maxNodeID, int64(node.ID))

				if _, ok := p.wayNodeMap[int64(node.ID)]; ok {
					p.acceptedNodeMap[int64(node.ID)] = nodeCoord{
						lat: node.Lat,
						lon: node.Lon,
					}
				}
				accessType := node.Tags.Find("access")
				barrierType := node.Tags.Find("barrier")

				if _, ok := acceptedBarrierType[barrierType]; ok && accessType == "no" && barrierType != "" {
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

		if val, ok := p.nodeTag[int64(nodeID)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)]; ok && val == 1 {

			graphStorage.SetTrafficLight(nodeIDX)
		}
		processedNodes[nodeIDX] = datastructure.NewCHNode(coord.lat, coord.lon, 0, nodeIDX)

	}

	log.Printf("total edges: %d", len(graphStorage.EdgeStorage))

	return processedNodes, graphStorage, streetDirection
}

type wayExtraInfo struct {
	oneWay  bool
	forward bool
}

func (p *OsmParser) processWay(way *osm.Way, graphStorage *datastructure.GraphStorage,
	streetDirection map[string][2]bool,
	edgeSet map[int32]map[int32]struct{}) error {
	tempMap := make(map[string]string)
	name := way.Tags.Find("name")

	tempMap[STREET_NAME] = name

	refName := way.Tags.Find("ref")
	tempMap[STREET_REF] = refName

	maxSpeed := 0.0
	highwayTypeSpeed := 0.0

	wayExtraInfoData := wayExtraInfo{}
	okvf, okmvf, okvb, okmvb := getReversedOneWay(way)
	if val := way.Tags.Find("oneway"); val == "yes" || val == "-1" || okvf || okmvf || okvb || okmvb {
		wayExtraInfoData.oneWay = true
	}

	if way.Tags.Find("oneway") == "-1" || okvf || okmvf {
		// okvf / omvf = restricted/not allowed forward.
		wayExtraInfoData.forward = false

	} else {
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
				highwayTypeSpeed = roadTypeMaxSpeed2(tag.Value)

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
					// without unit
					// dont use this
				}
			}

		}

	}

	if maxSpeed == 0 {
		maxSpeed = highwayTypeSpeed
	}
	if maxSpeed == 0 {
		maxSpeed = 30
	}

	waySegment := []node{}
	for _, wayNode := range way.Nodes {
		nodeCoord := p.acceptedNodeMap[int64(wayNode.ID)]
		nodeData := node{
			id:    int64(wayNode.ID),
			coord: nodeCoord,
		}
		if p.isJunctionNode(int64(nodeData.id)) {

			waySegment = append(waySegment, nodeData)
			p.processSegment(waySegment, tempMap, maxSpeed, graphStorage, wayExtraInfoData,
				edgeSet)
			waySegment = []node{}

			waySegment = append(waySegment, nodeData)

		} else {
			waySegment = append(waySegment, nodeData)
		}

	}
	if len(waySegment) > 1 {
		p.processSegment(waySegment, tempMap, maxSpeed, graphStorage, wayExtraInfoData, edgeSet)
	}

	return nil
}

func isRestricted(value string) bool {
	if value == "no" || value == "restricted" {
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

func (p *OsmParser) processSegment(segment []node, tempMap map[string]string, speed float64, graphStorage *datastructure.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[int32]map[int32]struct{}) {

	if len(segment) == 2 && segment[0].id == segment[1].id {
		// skip
		return
	} else if len(segment) > 2 && segment[0].id == segment[len(segment)-1].id {
		// loop
		p.processSegment2(segment[0:len(segment)-1], tempMap, speed, graphStorage, wayExtraInfoData, edgeSet)
		p.processSegment2(segment[len(segment)-2:], tempMap, speed, graphStorage, wayExtraInfoData, edgeSet)
	} else {
		p.processSegment2(segment, tempMap, speed, graphStorage, wayExtraInfoData, edgeSet)
	}
}

func (p *OsmParser) processSegment2(segment []node, tempMap map[string]string, speed float64, graphStorage *datastructure.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[int32]map[int32]struct{}) {
	waySegment := []node{}
	for i := 0; i < len(segment); i++ {
		nodeData := segment[i]
		if _, ok := p.barrierNodes[int64(nodeData.id)]; ok {

			if len(waySegment) != 0 {
				// if current node is a barrier
				// add the barrier node and process the segment (add edge)
				waySegment = append(waySegment, nodeData)
				p.addEdge(waySegment, tempMap, speed, graphStorage, wayExtraInfoData, edgeSet)
				waySegment = []node{}
			}
			// copy the barrier node but with different id so that previous edge (with barrier) not connected with the new edge

			nodeData = p.copyNode(nodeData)
			waySegment = append(waySegment, nodeData)

		} else {
			waySegment = append(waySegment, nodeData)
		}
	}
	if len(waySegment) > 1 {
		p.addEdge(waySegment, tempMap, speed, graphStorage, wayExtraInfoData, edgeSet)
	}
}

func (p *OsmParser) copyNode(nodeData node) node {
	// use the same coordinate but different id & and the newID is not used
	newMaxID := p.maxNodeID + 1
	p.acceptedNodeMap[newMaxID] = nodeCoord{
		lat: nodeData.coord.lat,
		lon: nodeData.coord.lon,
	}
	p.maxNodeID++
	return node{
		id: newMaxID,
		coord: nodeCoord{
			lat: nodeData.coord.lat,
			lon: nodeData.coord.lon,
		},
	}
}

func (p *OsmParser) addEdge(segment []node, tempMap map[string]string, speed float64, graphStorage *datastructure.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[int32]map[int32]struct{}) {
	from := segment[0]

	to := segment[len(segment)-1]

	if from == to {
		return
	}

	if _, ok := p.nodeIDMap[from.id]; !ok {
		p.nodeIDMap[from.id] = int32(len(p.nodeIDMap))
	}
	if _, ok := p.nodeIDMap[to.id]; !ok {
		p.nodeIDMap[to.id] = int32(len(p.nodeIDMap))
	}

	edgePoints := []datastructure.Coordinate{}
	distance := 0.0
	for i := 0; i < len(segment); i++ {
		if i != 0 && i != len(segment)-1 && p.nodeTag[int64(segment[i].id)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] == 1 {

			distToFromNode := geo.CalculateHaversineDistance(from.coord.lat, from.coord.lon, segment[i].coord.lat, segment[i].coord.lon)
			distToToNode := geo.CalculateHaversineDistance(to.coord.lat, to.coord.lon, segment[i].coord.lat, segment[i].coord.lon)
			if distToFromNode < distToToNode {
				if _, ok := p.nodeTag[int64(segment[0].id)]; !ok {
					p.nodeTag[int64(segment[0].id)] = make(map[int]int)
				}
				p.nodeTag[int64(segment[0].id)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] = 1
			} else {
				if _, ok := p.nodeTag[int64(segment[len(segment)-1].id)]; !ok {
					p.nodeTag[int64(segment[len(segment)-1].id)] = make(map[int]int)
				}
				p.nodeTag[int64(segment[len(segment)-1].id)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] = 1
			}
		}
		edgePoints = append(edgePoints, datastructure.Coordinate{
			Lat: segment[i].coord.lat,
			Lon: segment[i].coord.lon,
		})
		if i > 0 {
			distance += geo.CalculateHaversineDistance(segment[i-1].coord.lat, segment[i-1].coord.lon, segment[i].coord.lat, segment[i].coord.lon)
		}
	}

	edgePoints = geo.RamerDouglasPeucker(edgePoints) // simplify edge geometry

	isRoundabout := false
	if val, ok := tempMap[JUNCTION]; ok {
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
		lanes = 1 // assume
	}

	if _, ok := edgeSet[p.nodeIDMap[from.id]]; !ok {
		edgeSet[p.nodeIDMap[from.id]] = make(map[int32]struct{})
	}
	if _, ok := edgeSet[p.nodeIDMap[to.id]]; !ok {
		edgeSet[p.nodeIDMap[to.id]] = make(map[int32]struct{})
	}

	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {

			if _, ok := edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]]; ok {
				return
			}

			edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]] = struct{}{}

			startPointsIndex := len(graphStorage.GlobalPoints)

			graphStorage.AppendGlobalPoints(edgePoints)
			endPointsIndex := len(graphStorage.GlobalPoints)

			graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
				uint8(lanes),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
				uint32(startPointsIndex), uint32(endPointsIndex),
			),
			)

			graphStorage.SetRoundabout(int32(len(graphStorage.EdgeStorage)), isRoundabout)

			graphStorage.AppendEdgeStorage(
				datastructure.NewEdge(int32(len(graphStorage.EdgeStorage)), p.nodeIDMap[to.id], p.nodeIDMap[from.id],
					-1, etaWeight, distanceInMeter))

		} else {

			if _, ok := edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]]; ok {
				return
			}
			edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]] = struct{}{}

			graphStorage.SetRoundabout(int32(len(graphStorage.EdgeStorage)), isRoundabout)

			edgePoints = util.ReverseG(edgePoints)

			startPointsIndex := len(graphStorage.GlobalPoints)

			graphStorage.AppendGlobalPoints(edgePoints)
			endPointsIndex := len(graphStorage.GlobalPoints)

			graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
				uint8(lanes),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
				uint32(startPointsIndex), uint32(endPointsIndex),
			),
			)

			graphStorage.AppendEdgeStorage(
				datastructure.NewEdge(int32(len(graphStorage.EdgeStorage)), p.nodeIDMap[from.id], p.nodeIDMap[to.id], -1,
					etaWeight, distanceInMeter))

		}
	} else {
		if _, ok := edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]]; ok {
			return
		}
		edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]] = struct{}{}
		edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]] = struct{}{}

		// !reversed edge
		graphStorage.SetRoundabout(int32(len(graphStorage.EdgeStorage)), isRoundabout)

		startPointsIndex := len(graphStorage.GlobalPoints)

		graphStorage.AppendGlobalPoints(edgePoints)
		endPointsIndex := len(graphStorage.GlobalPoints)

		graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
			uint8(lanes),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
			uint32(startPointsIndex), uint32(endPointsIndex),
		),
		)

		graphStorage.AppendEdgeStorage(
			datastructure.NewEdge(int32(len(graphStorage.EdgeStorage)), p.nodeIDMap[to.id], p.nodeIDMap[from.id],
				-1, etaWeight, distanceInMeter))

		// reversed edge

		graphStorage.SetRoundabout(int32(len(graphStorage.EdgeStorage)), isRoundabout)

		graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
			uint8(lanes),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
			uint32(endPointsIndex), uint32(startPointsIndex),
		),
		)

		graphStorage.AppendEdgeStorage(
			datastructure.NewEdge(int32(len(graphStorage.EdgeStorage)), p.nodeIDMap[from.id], p.nodeIDMap[to.id],
				-1, etaWeight, distanceInMeter))

	}
}

func roadTypeMaxSpeed2(roadType string) float64 {
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
		return 40
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
		return 5
	case "road":
		return 20
	case "track":
		return 15
	case "motorroad":
		return 90
	default:
		return 30
	}
}

func (p *OsmParser) isJunctionNode(nodeID int64) bool {
	return p.wayNodeMap[int64(nodeID)] == JUNCTION_NODE
}

func acceptOsmWay(way *osm.Way) bool {
	highway := way.Tags.Find("highway")
	junction := way.Tags.Find("junction")
	if highway != "" {
		if _, ok := acceptedHighway[highway]; ok {
			return true
		}
	} else if junction != "" {
		return true
	}
	return false
}

func max(a, b int64) int64 {
	if a > b {
		return a
	}
	return b
}
