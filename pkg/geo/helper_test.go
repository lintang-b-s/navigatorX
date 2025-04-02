package geo

import (
	"testing"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
)

func TestDouglasPecker(t *testing.T) {
	lineCoords := []datastructure.Coordinate{
		{-7.565837, 110.831586},
		{-7.566063, 110.832379},
		{-7.566406, 110.833232},
	}

	simplified := RamesDouglasPeucker(lineCoords)
	if len(simplified) > 2 {
		t.Errorf("expected 2, got %d", len(simplified))
	}
}
