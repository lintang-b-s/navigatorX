package matching_test

import (
	"lintang/navigatorx/pkg/engine/matching"
	"lintang/navigatorx/pkg/util"
	"testing"

	"github.com/stretchr/testify/assert"
)

// https://www.cis.upenn.edu/~cis2620/notes/Example-Viterbi-DNA.pdf
func TestViterbiAlgo(t *testing.T) {

	t.Run("success run viterbi example dna ", func(t *testing.T) {
		// 1 = H, 2 = L
		states := []matching.ViterbiNode{
			{ID: 1, NodeID: -1, Lat: -1, Lon: -1},
			{ID: 2, NodeID: -1, Lat: -1, Lon: -1},
		}

		// 1 = A, 2 = C, 3 = G, 4 = T
		observations := []matching.ViterbiNode{
			{ID: 3, NodeID: -1}, {ID: 3, NodeID: -1}, {ID: 2, NodeID: -1},
			{ID: 1, NodeID: -1}, {ID: 2, NodeID: -1}, {ID: 4, NodeID: -1}, {ID: 3, NodeID: -1},
			{ID: 1, NodeID: -1}, {ID: 1, NodeID: -1},
		}

		// Initial Prob H: -1, L: -1
		startProb := map[int]float64{
			1: -1,
			2: -1,
		}

		// Transition Prob H->H = -2.3222
		// Transition Prob
		transitionProb := map[int]map[int]float64{
			1: {1: -1, 2: -1},
			2: {1: -1.322, 2: -0.737},
		}

		// emission Prob
		// emission Prob
		emissionProb := map[int]map[int]float64{

			1: {1: -2.322, 2: -1.737, 3: -1.737, 4: -2.322},
			2: {1: -1.737, 2: -2.322, 3: -2.322, 4: -1.737},
		}

		viterbi := matching.NewViterbi(observations, states, transitionProb, emissionProb, startProb, nil)
		maxProb, path, _ := viterbi.RunViterbi()
		assert.Equal(t, util.RoundFloat(maxProb, 2), -24.49)
		assert.Equal(t, path, []matching.ViterbiNode{{1, -1, -1, -1}, {1, -1, -1, -1}, {1, -1, -1, -1},
			{2, -1, -1, -1}, {2, -1, -1, -1}, {2, -1, -1, -1}, {2, -1, -1, -1},
			{2, -1, -1, -1}, {2, -1, -1, -1}})
	})

}
