package matching

import "errors"

const (
	minDistToGraphNode = 2      // 2 meter
	maxTransitionDist  = 1000.0 // 1km
)

var (
	hmmbreakErr = errors.New("hmmbreak error")
)
