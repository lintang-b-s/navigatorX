package matching_test

import (
	"errors"
	"lintang/navigatorx/pkg/engine/matching"
	"testing"

	"github.com/stretchr/testify/assert"
)

// https://www.cis.upenn.edu/~cis2620/notes/Example-Viterbi-DNA.pdf
func TestViterbiAlgo2(t *testing.T) {

	t.Run("success run viterbi example dna ", func(t *testing.T) {
		// 1 = H, 2 = L
		states := []int{
			1,
			2,
		}

		// 1 = A, 2 = C, 3 = G, 4 = T
		observations := []int{
			3, 3, 2,
			1, 2, 4, 3,
			1, 1,
		}

		// (convert to negative log probability)
		// Initial Prob H: -1, L: -1 (already in log-space)
		startProb := map[int]float64{
			1: -1,
			2: -1,
		}

		// Transition Prob H->H = -2.3222 (already in log-space)
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
		viterbi := matching.NewViterbiAlgorithm(true)

		emissionLogProb := make(map[int]float64)
		transitionLogProb := make(map[matching.Transition]float64)

		var prevTimeStep int = -1
		for _, obs := range observations {
			for _, state := range states {
				emissionLogProb[state] = emissionProb[state][obs]
			}

			if prevTimeStep == -1 {

				viterbi.StartWithInitialStateProbabilities(0, states, startProb)
			} else {

				for _, from := range states {
					for _, to := range states {
						transition := matching.Transition{From: from, To: to}
						transitionLogProb[transition] = transitionProb[from][to]
					}
				}

				viterbi.NextStep(obs, states, emissionLogProb, transitionLogProb, nil)
			}

			if viterbi.IsBroken() {
				panic(errors.New("viterbi is broken"))
			}

			prevTimeStep = obs
		}

		path := viterbi.ComputeMostLikelySequence()
		statesPath := make([]int, 0)
		for _, p := range path {
			statesPath = append(statesPath, p.State)
		}

		// assert.Equal(t, -24.49, util.RoundFloat(maxProb, 2))
		assert.Equal(t, []int{states[0], states[0], states[0],
			states[1], states[1], states[1], states[1],
			states[1], states[1]}, statesPath)
	})

}
