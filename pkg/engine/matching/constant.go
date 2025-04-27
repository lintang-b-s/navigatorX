package matching

import "errors"

const (
	minDistToGraphNode = 2      // 2 meter
	maxTransitionDist  = 2000.0 // 2km
)

var (
	hmmbreakErr = errors.New("hmmbreak error")
)
