package datastructure_test

import (
	"testing"

	"github.com/lintang-b-s/navigatorx/pkg/geo"

	"github.com/stretchr/testify/assert"
)

func TestHaversine(t *testing.T) {
	cases := []struct {
		latOne, longOne, latTwo, longTwo float64
		expectedDist                     float64
	}{
		{
			latOne:       -7.557155997491524,
			longOne:      110.77170252731288,
			latTwo:       -7.550209300671982,
			longTwo:      110.78942094938256,
			expectedDist: 2.1,
		},
		{
			latOne:  -7.546196863318374,
			longOne: 110.7775170972345,

			latTwo:       -7.550209300671982,
			longTwo:      110.78942094938256,
			expectedDist: 1.38,
		},
		{
			latOne:       -7.759889166547908,
			longOne:      110.36689459108496,
			latTwo:       -7.760335932763678,
			longTwo:      110.37671195413539,
			expectedDist: 1.08,
		},
		{
			latOne:       -7.700002453207869,
			longOne:      110.37712514761436,
			latTwo:       -7.760335932763678,
			longTwo:      110.37671195413539,
			expectedDist: 6.7,
		},
	}

	t.Run("success haversine distance", func(t *testing.T) {
		for _, c := range cases {
			dist := geo.CalculateHaversineDistance(c.latOne, c.longOne, c.latTwo, c.longTwo)
			assert.InDelta(t, c.expectedDist, dist, 0.1)
		}
	})
}
