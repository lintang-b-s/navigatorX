package geo

import (
	"testing"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
)

func TestIsPointInLine(t *testing.T) {

	lat, lon := 47.667347, -122.120561

	linePoints := []datastructure.Coordinate{
		datastructure.NewCoordinate(47.667324, -122.118989),
		datastructure.NewCoordinate(47.667338, -122.121784),
	}

	result := PointPositionBetweenLinePoints(lat, lon, linePoints)
	if result != 1 {
		t.Errorf("Expected 1, got %d", result)
	}
}
