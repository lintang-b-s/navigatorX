package matching

import (
	"lintang/navigatorx/pkg/datastructure"
	"lintang/navigatorx/pkg/util"
	"math"
)

type Viterbi struct {
	Obs            []ViterbiNode
	States         []ViterbiNode
	TransitionProb map[int]map[int]float64 //   Transition  probabilities  give  the  probability  of  a  vehicle  moving  between  the  candidate  road  matches  at  these  two  times.
	EmmissionProb  map[int]map[int]float64 // This  gives  the  likelihood  that  the  measurement 𝑧𝑡 would be observed if the vehicle were actually on  road segment 𝑟𝑖.
	InitialProb    map[int]float64
	StateRoadNodes map[int][]datastructure.Coordinate
}

// ViterbiNode: observations(gps points) atau hidden states(road segments)
type ViterbiNode struct {
	ID     int
	NodeID int32
	Lat    float64
	Lon    float64
}

func NewViterbi(ob []ViterbiNode, sts []ViterbiNode,
	trProb, emissionProb map[int]map[int]float64, initialProb map[int]float64, stateRoadNodes map[int][]datastructure.Coordinate) *Viterbi {
	return &Viterbi{
		Obs:            ob,
		States:         sts,
		TransitionProb: trProb,
		EmmissionProb:  emissionProb,
		InitialProb:    initialProb,
		StateRoadNodes: stateRoadNodes,
	}
}

// RunViterbi: run viterbi algorithm, get most probable sequence of hidden states (road segments) given sequence of observations (gps points) O(|T|*|S|^2)
func (v *Viterbi) RunViterbi() (float64, []ViterbiNode, []datastructure.Coordinate) {
	viterbi := []map[int]float64{}
	viterbi = append(viterbi, map[int]float64{})
	path := []ViterbiNode{}
	parent := []map[ViterbiNode]ViterbiNode{}
	parent = append(parent, make(map[ViterbiNode]ViterbiNode))

	for _, s := range v.States {
		var initProb float64 = 0
		if val, ok := v.InitialProb[s.ID]; ok {
			initProb = val
		}
		viterbi[0][s.ID] = initProb + v.EmmissionProb[s.ID][v.Obs[0].ID]
	}

	for t := 1; t < len(v.Obs); t++ {
		viterbi = append(viterbi, make(map[int]float64))
		parent = append(parent, make(map[ViterbiNode]ViterbiNode))

		for _, s := range v.States {
			if _, ok := v.EmmissionProb[s.ID][v.Obs[t].ID]; !ok {
				continue
			}
			state := ViterbiNode{}
			maxTransitionProb := math.Inf(-1)
			for _, prevS := range v.States {
				_, okv := viterbi[t-1][prevS.ID]
				_, okt := v.TransitionProb[prevS.ID][s.ID]
				if !okt || !okv {
					continue
				}

				transitionProb := viterbi[t-1][prevS.ID] + v.TransitionProb[prevS.ID][s.ID]
				if transitionProb > maxTransitionProb {
					maxTransitionProb = transitionProb
					state = prevS
				}
			}

			viterbi[t][s.ID] = maxTransitionProb + v.EmmissionProb[s.ID][v.Obs[t].ID]
			parent[t][s] = state

		}
	}

	prob := math.Inf(-1)
	state := ViterbiNode{}
	roadPath := []datastructure.Coordinate{}
	for _, s := range v.States {
		_, ok := viterbi[len(v.Obs)-1][s.ID]
		if !ok {
			continue
		}
		if viterbi[len(v.Obs)-1][s.ID] > prob {
			prob = viterbi[len(v.Obs)-1][s.ID]
			state = s
		}
	}

	for t := len(v.Obs) - 1; t > 0; t-- {
		path = append(path, state)
		if _, ok := v.StateRoadNodes[state.ID]; ok {
			roadPath = append(roadPath, v.StateRoadNodes[state.ID]...)
		}
		state = parent[t][state]
	}
	path = append(path, state)
	roadPath = append(roadPath, v.StateRoadNodes[state.ID]...)
	util.ReverseG(path)
	util.ReverseG(roadPath)
	return prob, path, roadPath
}
