package datastructure

import (
	"lintang/navigatorx/pkg/concurrent"

	"github.com/twpayne/go-polyline"
)

type EdgePair struct {
	Weight        float64
	Dist          float64
	ToNodeIDX     int32
	IsShortcut    bool
	EdgeIDx       int32
	Roundabout    bool
	RoadClass     string
	RoadClassLink string
	Lanes         int
}

type Edge struct {
	From          *Node
	To            *Node
	Cost          float64
	StreetName    string
	MaxSpeed      float64
	Roundabout    bool
	RoadClass     string
	RoadClassLink string
	Lanes         int
	NodesInBetween []Coordinate
}

type Node struct {
	Tags         []string
	Out_to       []Edge
	Lat, Lon     float64
	ID           int64
	StreetName   string
	TrafficLight bool
	UsedInRoad   int
}


type CHNode2 struct {
	Lat          float64
	Lon          float64
	OrderPos     int64
	IDx          int32
	TrafficLight bool
}

func NewCHNode(lat, lon float64, orderPos int64, idx int32, trafficLight bool) CHNode2 {
	return CHNode2{
		Lat:          lat,
		Lon:          lon,
		OrderPos:     orderPos,
		IDx:          idx,
		TrafficLight: trafficLight,
	}
}

type SurakartaWay struct {
	ID                  int32
	CenterLoc           []float64 // [lat, lon]
	Nodes               []Coordinate // yang bukan intersectionNodes
	IntersectionNodesID []int64
	WayID		  int32
}

type Metadata struct {
	MeanDegree       float64
	ShortcutsCount   int64
	degrees          []int
	InEdgeOrigCount  []int
	OutEdgeOrigCount []int
	EdgeCount        int
	NodeCount        int
}

type EdgeCH struct {
	EdgeIDx        int32
	Weight         float64
	Dist           float64
	ToNodeIDX      int32
	BaseNodeIDx    int32
	IsShortcut     bool
	RemovedEdgeOne int32
	RemovedEdgeTwo int32
	StreetName     string
	Roundabout     bool
	RoadClass      string
	RoadClassLink  string
	Lanes          int
	NodesInBetween []Coordinate

}

type StreetExtraInfo struct {
	Destination      string
	DestinationRef   string
	MotorwayJunction string
}

func RoadTypeMaxSpeed(roadType string) float64 {
	switch roadType {
	case "motorway":
		return 95
	case "trunk":
		return 85
	case "primary":
		return 75
	case "secondary":
		return 65
	case "tertiary":
		return 50
	case "unclassified":
		return 50
	case "residential":
		return 30
	case "service":
		return 20
	case "motorway_link":
		return 90
	case "trunk_link":
		return 80
	case "primary_link":
		return 70
	case "secondary_link":
		return 60
	case "tertiary_link":
		return 50
	case "living_street":
		return 20
	default:
		return 40
	}
}

type SPSingleResultResult struct {
	Source    int32
	Dest      int32
	Paths     []Coordinate
	EdgePath []EdgeCH
	Dist      float64
	Eta       float64
}

type StateObservationPair struct {
	Observation CHNode2
	State       []State
}

type State struct {
	ID     int
	NodeID int32
	Lat    float64
	Lon    float64
	Dist   float64
	EdgeID int32
	NodesInBetween []Coordinate
}
type SmallWay struct {
	CenterLoc           []float64 // [lat, lon]
	IntersectionNodesID []int64
	NodesInBetween []Coordinate
	WayID int32
}

func (s *SmallWay) ToConcurrentWay() concurrent.SmallWay {
	nodesInBetweenLat := []float64{}
	nodesInBetweenLon := []float64{}
	for _, n := range s.NodesInBetween {
		nodesInBetweenLat = append(nodesInBetweenLat, n.Lat)
		nodesInBetweenLon = append(nodesInBetweenLon, n.Lon)
	}
	return concurrent.SmallWay{
		CenterLoc:           s.CenterLoc,
		IntersectionNodesID: s.IntersectionNodesID,
		NodesInBetween: concurrent.NewCoordinates(nodesInBetweenLat, nodesInBetweenLon),
		WayID: s.WayID,
	}
}


func RenderPath2(path []Coordinate) string {
	s := ""
	coords := make([][]float64, 0)
	for _, p := range path {
		pT := p
		coords = append(coords, []float64{pT.Lat, pT.Lon})
	}
	s = string(polyline.EncodeCoords(coords))
	return s
}
