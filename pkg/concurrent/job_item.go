package concurrent

type KVEdge struct {
	CenterLoc           []float64 // [lat, lon]
	IntersectionNodesID []int32
	PointsInBetween     []Coordinate
	WayID               int32
	FromNodeID          int32
	ToNodeID            int32
}
type SaveWayJobItem struct {
	KeyStr string
	ValArr []KVEdge
}

type JobI interface {
	[]int32 | SaveWayJobItem | []KVEdge
}

type Job[T JobI] struct {
	ID      int
	JobItem T
}
type JobFunc[T JobI, G any] func(job T) G

type Coordinate struct {
	Lat float64
	Lon float64
}

func NewCoordinate(lat, lon float64) Coordinate {
	return Coordinate{
		Lat: lat,
		Lon: lon,
	}
}

func NewCoordinates(lat, lon []float64) []Coordinate {
	coords := make([]Coordinate, len(lat))
	for i := range lat {
		coords[i] = NewCoordinate(lat[i], lon[i])
	}
	return coords
}
