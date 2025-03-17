package matching

// func TestEmissionProb(t *testing.T) {
// 	distance := 3.347848983572151
// 	emissionProb := computeEmissionLogProb(distance)
// 	assert.InDelta(t, -4.83320315719618, emissionProb, 0.001)
// }

// func TestTransitionProb(t *testing.T) {
// 	straightDIstance := 129.41622903017912
// 	routeLength := 196.856488
// 	emissionProb := computeTransitionLogProb(routeLength, straightDIstance)
// 	assert.InDelta(t, -5.649206013989518, emissionProb, 0.001)
// }

// func TestComputeTransitionLogProb(t *testing.T) {
// 	prevLayer := &CandidateLayer{
// 		Observation: &datastructure.CHNode{
// 			ID:          1,
// 			Lat:          -7.782878144018028,
// 			Lon:          110.38798611289823,
// 			OrderPos:     1,
// 			TrafficLight: true,
// 		},
// 		States: []*datastructure.State{
// 			{
// 				StateID: 1,
// 				NodeID:  2,
// 				Lat:     -7.783035537807805,
// 				Lon:     110.38775961564208,
// 				Dist:    100,
// 				EdgeID:  1,
// 			},
// 			{
// 				StateID: 2,
// 				NodeID:  3,
// 				Lat:     -7.783003498249734,
// 				Lon:     110.38836724217842,
// 				Dist:    100,
// 				EdgeID:  2,
// 			},
// 		},
// 	}

// 	currentLayer := &CandidateLayer{
// 		Observation: &datastructure.CHNode{
// 			ID:          2,
// 			Lat:          -7.782216636838503,
// 			Lon:          110.38807173981624,
// 			OrderPos:     1,
// 			TrafficLight: true,
// 		},
// 		States: []*datastructure.State{
// 			{
// 				StateID: 3,
// 				NodeID:  4,
// 				Lat:     -7.782050567038742,
// 				Lon:     110.38794310620472,
// 				Dist:    100,
// 				EdgeID:  3,
// 			},
// 			{
// 				StateID: 4,
// 				NodeID:  5,
// 				Lat:     -7.7820467049496,
// 				Lon:     110.38828612916878,
// 				Dist:    100,
// 				EdgeID:  4,
// 			},
// 		},
// 	}

// 	routeLengths := make(map[int]map[int]float64)
// 	routeLengths[1] = make(map[int]float64)
// 	routeLengths[1][3] = 110
// 	routeLengths[2] = make(map[int]float64)
// 	routeLengths[1][4] = 150

// 	routeLengths[2][3] = 170
// 	routeLengths[2][4] = 160

// 	hmm := NewHMMMapMatching(nil, nil, nil)
// 	hmm.computeTransitionLogProb(prevLayer, currentLayer, routeLengths)
// 	for _, state := range currentLayer.TransitionLogProbabilities {
// 		assert.NotEqual(t, 0.0, state.prob)
// 	}

// 	assert.Equal(t, 4, len(currentLayer.TransitionLogProbabilities))
// }

// func TestComputeEmissionLogProb(t *testing.T) {

// 	currentLayer := &CandidateLayer{
// 		Observation: &datastructure.CHNode{
// 			ID:          2,
// 			Lat:          -7.782216636838503,
// 			Lon:          110.38807173981624,
// 			OrderPos:     1,
// 			TrafficLight: true,
// 		},
// 		States: []*datastructure.State{
// 			{
// 				StateID:       3,
// 				NodeID:        4,
// 				Lat:           -7.782050567038742,
// 				Lon:           110.38794310620472,
// 				Dist:          100,
// 				EdgeID:        3,
// 				ProjectionLoc: [2]float64{-7.782050567038742, 110.38794310620472},
// 			},
// 			{
// 				StateID:       4,
// 				NodeID:        5,
// 				Lat:           -7.7820467049496,
// 				Lon:           110.38828612916878,
// 				Dist:          100,
// 				EdgeID:        4,
// 				ProjectionLoc: [2]float64{-7.7820467049496, 110.38828612916878},
// 			},
// 		},
// 	}

// 	routeLengths := make(map[int]map[int]float64)
// 	routeLengths[1] = make(map[int]float64)
// 	routeLengths[1][3] = 110
// 	routeLengths[2] = make(map[int]float64)
// 	routeLengths[1][4] = 150

// 	routeLengths[2][3] = 170
// 	routeLengths[2][4] = 160

// 	hmm := NewHMMMapMatching(nil, nil, nil)
// 	hmm.computeEmissionLogProb(currentLayer)
// 	for _, state := range currentLayer.EmissionLogProbabilities {
// 		assert.NotEqual(t, 0.0, state.prob)
// 	}

// 	assert.Equal(t, 2, len(currentLayer.EmissionLogProbabilities))
// }
