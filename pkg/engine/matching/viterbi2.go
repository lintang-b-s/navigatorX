package matching

import (
	"errors"
	"fmt"
	"lintang/navigatorx/pkg/util"
	"math"
	"strings"
)

type Transition struct {
	From int
	To   int
}

func NewTransition(from, to int) Transition {
	return Transition{From: from, To: to}
}

type SequenceState struct {
	State                int
	Observation          int
	TransitionDescriptor int
}

// extendedState.  backpointer for each state
type extendedState struct {
	state                int
	backPointer          *extendedState
	observation          int
	transitionDescriptor int
}

// forwardStepResult.  result for single forward step
type forwardStepResult struct {
	newMessage        map[int]float64
	newExtendedStates map[int]*extendedState
}

type ViterbiAlgorithm struct {
	// for retrieving the most likely sequence using back pointers
	lastExtendedStates map[int]*extendedState

	prevCandidates []int

	// For each state s_t of the current time step t, message[s_t] contains the log
	// probability of the most likely sequence ending in state s_t
	message map[int]float64

	isBroken bool

	messageHistory     []map[int]float64
	keepMessageHistory bool
}

type ViterbiNode2 struct {
	ID int
}

func NewViterbiAlgorithm(keepMessageHistory bool) *ViterbiAlgorithm {
	v := &ViterbiAlgorithm{
		keepMessageHistory: keepMessageHistory,
	}

	if keepMessageHistory {
		v.messageHistory = make([]map[int]float64, 0)
	}

	return v
}

// set initial probabilities for the first time step.
func (v *ViterbiAlgorithm) StartWithInitialStateProbabilities(
	observation int,
	initialStates []int,
	initialLogProbabilities map[int]float64) error {

	return v.initializeStateProbabilities(observation, initialStates, initialLogProbabilities)
}


func (v *ViterbiAlgorithm) StartWithInitialObservation(
	observation int,
	candidates []int,
	emissionLogProbabilities map[int]float64) error {

	return v.initializeStateProbabilities(observation, candidates, emissionLogProbabilities)
}


func (v *ViterbiAlgorithm) NextStep(
	observation int,
	candidates []int,
	emissionLogProbabilities map[int]float64,
	transitionLogProbabilities map[Transition]float64,
	transitionDescriptors map[Transition]int) error {

	if v.message == nil {
		return fmt.Errorf("StartWithInitialStateProbabilities() or StartWithInitialObservation() must be called first")
	}

	if v.isBroken {
		return fmt.Errorf("method must not be called after an HMM break")
	}

	// Forward step
	forwardStepResult, err := v.forwardStep(
		observation,
		v.prevCandidates,
		candidates,
		v.message,
		emissionLogProbabilities,
		transitionLogProbabilities,
		transitionDescriptors)

	if err != nil {
		return err
	}

	v.isBroken = v.hmmBreak(forwardStepResult.newMessage)
	if v.isBroken {
		return errors.New("HMM break occurred")
	}

	if v.keepMessageHistory {
		v.messageHistory = append(v.messageHistory, forwardStepResult.newMessage)
	}

	v.message = forwardStepResult.newMessage
	v.lastExtendedStates = forwardStepResult.newExtendedStates

	// Defensive copy
	v.prevCandidates = make([]int, len(candidates))
	copy(v.prevCandidates, candidates)

	return nil
}

// NextStepWithoutDescriptors is a convenience wrapper for NextStep without transition descriptors
func (v *ViterbiAlgorithm) NextStepWithoutDescriptors(
	observation int,
	candidates []int,
	emissionLogProbabilities map[int]float64,
	transitionLogProbabilities map[Transition]float64) error {

	return v.NextStep(
		observation,
		candidates,
		emissionLogProbabilities,
		transitionLogProbabilities,
		make(map[Transition]int))
}

// ComputeMostLikelySequence returns the most likely sequence of states for all time steps
func (v *ViterbiAlgorithm) ComputeMostLikelySequence() []SequenceState {
	if v.message == nil {
		
		return []SequenceState{}
	} else {
		return v.retrieveMostLikelySequence()
	}
}

// IsBroken returns whether an HMM break occurred in the last time step
func (v *ViterbiAlgorithm) IsBroken() bool {
	return v.isBroken
}

// MessageHistory returns the sequence of intermediate forward messages
func (v *ViterbiAlgorithm) MessageHistory() []map[int]float64 {
	return v.messageHistory
}

func (v *ViterbiAlgorithm) MessageHistoryString() (string, error) {
	if !v.keepMessageHistory {
		return "", fmt.Errorf("message history was not recorded")
	}

	var sb strings.Builder
	sb.WriteString("Message history with log probabilities\n\n")

	for i, message := range v.messageHistory {
		fmt.Fprintf(&sb, "Time step %d\n", i)

		for state, value := range message {
			fmt.Fprintf(&sb, "%v: %f\n", state, value)
		}

		sb.WriteString("\n")
	}

	return sb.String(), nil
}

// hmmBreak returns whether the specified message is either empty or only contains
// state candidates with zero probability (-inf in log-based) and thus causes the HMM to break
func (v *ViterbiAlgorithm) hmmBreak(message map[int]float64) bool {
	for _, logProbability := range message {
		if logProbability != math.Inf(-1) {
			return false
		}
	}
	return true
}

func (v *ViterbiAlgorithm) initializeStateProbabilities(
	observation int,
	candidates []int,
	initialLogProbabilities map[int]float64) error {

	if v.message != nil {
		return fmt.Errorf("initial probabilities have already been set")
	}

	// Set initial log probability for each start state candidate
	initialMessage := make(map[int]float64)
	for _, candidate := range candidates {
		logProbability, ok := initialLogProbabilities[candidate]
		if !ok {
			return fmt.Errorf("no initial probability for %v", candidate)
		}
		initialMessage[candidate] = logProbability
	}

	v.isBroken = v.hmmBreak(initialMessage)
	if v.isBroken {
		return nil
	}

	v.message = initialMessage
	if v.keepMessageHistory {
		v.messageHistory = append(v.messageHistory, v.message)
	}

	v.lastExtendedStates = make(map[int]*extendedState)
	for _, candidate := range candidates {
		v.lastExtendedStates[candidate] = &extendedState{
			state:                candidate,
			backPointer:          nil,
			observation:          observation,
			transitionDescriptor: -1,
		}
	}

	v.prevCandidates = make([]int, len(candidates))
	copy(v.prevCandidates, candidates)

	return nil
}

func (v *ViterbiAlgorithm) forwardStep(
	observation int,
	prevCandidates []int,
	curCandidates []int,
	message map[int]float64,
	emissionLogProbabilities map[int]float64,
	transitionLogProbabilities map[Transition]float64,
	transitionDescriptors map[Transition]int) (*forwardStepResult, error) {

	result := &forwardStepResult{
		newMessage:        make(map[int]float64, len(curCandidates)),
		newExtendedStates: make(map[int]*extendedState, len(curCandidates)),
	}

	if len(prevCandidates) == 0 {
		return nil, fmt.Errorf("prevCandidates must not be empty")
	}

	for _, curState := range curCandidates {
		maxLogProbability := math.Inf(-1)
		var maxPrevState int = -1

		for _, prevState := range prevCandidates {
			logProbability := message[prevState] + v.transitionLogProbability(
				prevState, curState, transitionLogProbabilities)

			if logProbability > maxLogProbability {
				maxLogProbability = logProbability
				maxPrevState = prevState
			}
		}

		emissionProb, ok := emissionLogProbabilities[curState]
		if !ok {
			return nil, fmt.Errorf("no emission probability for %v", curState)
		}

		result.newMessage[curState] = maxLogProbability + emissionProb

		// maxPrevState will be nil if there is no transition with non -inf probability
		if maxPrevState != -1 {
			transition := NewTransition(maxPrevState, curState)
			result.newExtendedStates[curState] = &extendedState{
				state:                curState,
				backPointer:          v.lastExtendedStates[maxPrevState],
				observation:          observation,
				transitionDescriptor: transitionDescriptors[transition],
			}
		}

	}

	return result, nil
}

func (v *ViterbiAlgorithm) transitionLogProbability(
	prevState int,
	curState int,
	transitionLogProbabilities map[Transition]float64) float64 {

	transition := NewTransition(prevState, curState)
	logProb, ok := transitionLogProbabilities[transition]

	if !ok {
		return math.Inf(-1) // Transition has zero probability (log-based)
	}

	return logProb
}

func (v *ViterbiAlgorithm) mostLikelyState() int {
	var result int
	maxLogProbability := math.Inf(-1)

	for state, logProb := range v.message {
		if logProb > maxLogProbability {
			result = state
			maxLogProbability = logProb
		}
	}

	return result
}

func (v *ViterbiAlgorithm) retrieveMostLikelySequence() []SequenceState {
	lastState := v.mostLikelyState()

	result := make([]SequenceState, 0)
	es := v.lastExtendedStates[lastState]

	for es != nil {
		ss := SequenceState{
			State:                es.state,
			Observation:          es.observation,
			TransitionDescriptor: es.transitionDescriptor,
		}
		result = append(result, ss)
		es = es.backPointer
	}

	util.ReverseG(result)

	return result
}
