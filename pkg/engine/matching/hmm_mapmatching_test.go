package matching

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestEmissionProb(t *testing.T) {
	distance := 3.347848983572151
	emissionProb := computeEmissionLogProb(distance)
	assert.InDelta(t, -2.66, emissionProb, 0.001)
}

func TestTransitionProb(t *testing.T) {
	straightDIstance := 129.41622903017912
	routeLength := 196.856488
	transitionProb := computeTransitionLogProb(routeLength, straightDIstance)
	assert.InDelta(t, -74926.607, transitionProb, 0.001)
}
