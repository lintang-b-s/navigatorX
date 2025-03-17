package heuristics

import (
	"errors"
	"lintang/navigatorx/pkg/concurrent"
	"lintang/navigatorx/pkg/datastructure"
	"math"
	"sync"
	"time"

	"golang.org/x/exp/rand"
)

type ACO struct {
	distMatrix           [][]float64
	pheromoneMatrix      [][]float64
	heuristicMatrix      [][]float64
	transitionProbMatrix [][]float64
	nAnts                int
	alpha                float64
	beta                 float64
	evaporationRate      float64
	intensification      float64
	betaEvaporationRate  float64
	exploitationRate     float64
	nJobs                int

	bestPath         []int
	bestScore        float64
	bestPerIteration []float64
	bestPathHistory  [][]int

	fit     bool
	fitTime float64
	mapSize int

	lock sync.Mutex
}

func NewACO(distMatrix [][]float64, nAnts int, alpha, beta, evaporationRate, intensification, betaEvaporationRate, exploitationRate float64, nJobs int) *ACO {
	nNodes := len(distMatrix)
	pheromoneMatrix := make([][]float64, nNodes)
	heuristicMatrix := make([][]float64, nNodes)
	transitionProbMatrix := make([][]float64, nNodes)

	for i := 0; i < nNodes; i++ {
		pheromoneMatrix[i] = make([]float64, nNodes)
		heuristicMatrix[i] = make([]float64, nNodes)
		transitionProbMatrix[i] = make([]float64, nNodes)
		for j := 0; j < nNodes; j++ {
			if i != j {
				pheromoneMatrix[i][j] = 1.0
				if distMatrix[i][j] != 0 {
					heuristicMatrix[i][j] = 1.0 / distMatrix[i][j]
				} else {
					heuristicMatrix[i][j] = 0.0
				}
			} else {
				pheromoneMatrix[i][j] = 0.0
				heuristicMatrix[i][j] = 0.0
			}
			transitionProbMatrix[i][j] = math.Pow(pheromoneMatrix[i][j], alpha) * math.Pow(heuristicMatrix[i][j], beta)
		}
	}

	aco := &ACO{
		distMatrix:           distMatrix,
		pheromoneMatrix:      pheromoneMatrix,
		heuristicMatrix:      heuristicMatrix,
		transitionProbMatrix: transitionProbMatrix,
		nAnts:                nAnts,
		alpha:                alpha,
		beta:                 beta,
		evaporationRate:      evaporationRate,
		intensification:      intensification,
		betaEvaporationRate:  betaEvaporationRate,
		exploitationRate:     exploitationRate,
		nJobs:                nJobs,
		bestScore:            math.Inf(1),
		mapSize:              nNodes,
		bestPerIteration:     make([]float64, 0),
		bestPathHistory:      make([][]int, 0),
		fit:                  false,
	}

	return aco
}

/*
	 GetNextNode mengembalikan node selanjutnya yang akan dikunjungi oleh ant.
		generate bilangan random. jika lebih kecil dari exploitationRate, maka pilih node dengan nilai transitionProb[curr] terbesar.
		otherwise, sample secara random satu node dari nodesLeft dengan probability transitionProb[curr]
*/
func (aco *ACO) GetNextNode(curr int, nodesLeft []int) int {
	num := make([]float64, len(nodesLeft)) // num[i] probability node i dipilih setelah kunjungi currNode, ( dibagi sum of val dari aco.transitionProbMatrix[curr])
	sum := 0.0
	for i, node := range nodesLeft {

		trans_prob_val := aco.transitionProbMatrix[curr][node]
		num[i] = trans_prob_val
		sum += trans_prob_val
	}

	for i := 0; i < len(num); i++ {
		num[i] /= sum
	}

	if rand.Float64() <= aco.exploitationRate {
		maxIdx := 0
		maxVal := math.Inf(-1)
		for i := 0; i < len(num); i++ {
			if num[i] > maxVal {
				maxVal = num[i]
				maxIdx = i
			}
		}
		return nodesLeft[maxIdx]
	} else {

		rngSource := rand.NewSource(uint64(time.Now().UnixNano()))
		rng := rand.New(rngSource)
		nextNode, _ := Choice(nodesLeft, 1, false, num, rng)
		return nextNode[0]
	}
}

// PathTotalEta menghitung total eta dari solusi tsp (path).
func (aco *ACO) PathTotalEta(path []int) float64 {
	score := 0.0
	n := len(path)
	for i := 0; i < n; i++ {
		from := path[i]
		to := path[(i+1)%n]
		score += aco.distMatrix[from][to]
	}
	return score
}

// StartAndRunAnt menjalankan pencarian solusi tsp untuk satu ant.
func (aco *ACO) StartAndRunAnt(antID int, wg *sync.WaitGroup, paths [][]int, scores []float64) {
	defer wg.Done()
	nodesLeft := make([]int, aco.mapSize)
	for i := 0; i < aco.mapSize; i++ {
		nodesLeft[i] = i
	}
	startIdx := rand.Intn(len(nodesLeft))
	currNode := nodesLeft[startIdx] // pilih node pertama path tasp secara random.
	path := []int{currNode}
	nodesLeft = append(nodesLeft[:startIdx], nodesLeft[startIdx+1:]...)

	for len(nodesLeft) > 0 {
		// kunjungi next node setelah currNode
		nextNode := aco.GetNextNode(currNode, nodesLeft)
		path = append(path, nextNode)

		for i, node := range nodesLeft {
			if node == nextNode {
				nodesLeft = append(nodesLeft[:i], nodesLeft[i+1:]...)
				break
			}
		}
		currNode = nextNode
	}
	score := aco.PathTotalEta(path)
	aco.lock.Lock()
	paths[antID] = path
	scores[antID] = score
	aco.lock.Unlock()
}

// Evaporate evaporate/mengurangi nilai pheromone pada edges yang tidak dilewati ant terbaik
func (aco *ACO) Evaporate(bestGlobalScore float64, bestEdges []int) {
	deltaR := 1 / bestGlobalScore
	for i := 0; i < aco.mapSize; i++ {
		for j := 0; j < aco.mapSize; j++ {
			aco.pheromoneMatrix[i][j] *= (1 - aco.evaporationRate)

		}
	}

	for i := 0; i < len(bestEdges); i++ {
		from := bestEdges[i]
		to := bestEdges[(i+1)%len(bestEdges)]
		aco.pheromoneMatrix[from][to] += aco.evaporationRate * deltaR
	}
	aco.beta *= (1 - aco.betaEvaporationRate)
}

// menambahkan ekstra pheromone ke edges yang dilewati path terbaik.
func (aco *ACO) Intensify(path []int) {
	n := len(path)
	for i := 0; i < n; i++ {
		from := path[i]
		to := path[(i+1)%n]
		aco.pheromoneMatrix[from][to] += aco.intensification
	}
}

// UpdateTransitionProbs memperbarui transitionProb setelah memperbarui pheromone
func (aco *ACO) UpdateTransitionProbs() {
	for i := 0; i < aco.mapSize; i++ {
		for j := 0; j < aco.mapSize; j++ {
			aco.transitionProbMatrix[i][j] = math.Pow(aco.pheromoneMatrix[i][j], aco.alpha) * math.Pow(aco.heuristicMatrix[i][j], aco.beta)
		}
	}
}

func (aco *ACO) Solve(maxIter, earlyStop int) ([]int, float64) {
	sameResult := 0
	startTime := time.Now()

	for iter := 0; iter < maxIter; iter++ {
		paths := make([][]int, aco.nAnts)
		scores := make([]float64, aco.nAnts)
		var wg sync.WaitGroup
		wg.Add(aco.nAnts)
		for antID := 0; antID < aco.nAnts; antID++ {
			go aco.StartAndRunAnt(antID, &wg, paths, scores)
		}
		wg.Wait()

		bestAnt := 0
		bestScore := math.Inf(1)
		for i := 0; i < aco.nAnts; i++ {
			if scores[i] < bestScore {
				bestScore = scores[i]
				bestAnt = i
			}
		}
		iterBestPath := paths[bestAnt]
		iterBestScore := bestScore
		aco.bestPerIteration = append(aco.bestPerIteration, iterBestScore)

		if iterBestScore < aco.bestScore {
			aco.bestScore = iterBestScore
			aco.bestPath = make([]int, len(iterBestPath))
			copy(aco.bestPath, iterBestPath)
			sameResult = 0
		} else if iterBestScore == aco.bestScore {
			sameResult++
		} else {
			sameResult = 0
		}

		aco.Evaporate(iterBestScore, iterBestPath)
		aco.Intensify(iterBestPath)
		aco.UpdateTransitionProbs()
		aco.bestPathHistory = append(aco.bestPathHistory, aco.bestPath)

		if sameResult > earlyStop {
			// berhenti jika hasil sama selama earlyStop iterasi
			break
		}
	}

	aco.fit = true
	aco.fitTime = time.Since(startTime).Seconds()
	return aco.bestPath, aco.bestScore
}

func cdf(probs []float64) []float64 {
	cdf := make([]float64, len(probs))
	cum := 0.0

	for i := range cdf {
		cum += probs[i]
		cdf[i] = cum
	}

	return cdf
}

func findIndexFromRight(val float64, cdf []float64) int {
	for i, cumProb := range cdf {
		if cumProb >= val {
			return i
		}
	}

	return len(cdf) - 1
}

func Choice[T any](
	arr []T,
	size int,
	replace bool,
	probs []float64,
	rng *rand.Rand,
) ([]T, error) {
	if !replace && (size > len(arr)) {
		return nil, errors.New("tidak dapat mengambil sampel lebih dari ukuran slice (without replacement)")
	}

	samples := make([]T, size)
	probsCopy := make([]float64, len(probs))
	copy(probsCopy, probs)

	for i := 0; i < size; i++ {
		cdf := cdf(probsCopy)

		if !replace {
			total := cdf[len(cdf)-1]
			for cdfInd := range cdf {
				cdf[cdfInd] /= total
			}
		}

		randFloat := rng.Float64()
		sampledIndex := findIndexFromRight(randFloat, cdf)
		samples[i] = arr[sampledIndex]

		if !replace {
			probsCopy[sampledIndex] = 0.0
		}
	}

	return samples, nil
}

type RouteAlgorithm interface {
	CallBidirectionalDijkstra(spMap []int32) datastructure.SPSingleResultResult
}
type ContractedGraph interface {
	GetNodeFirstOutEdges(nodeID int32) []int32
	GetNodeFirstInEdges(nodeID int32) []int32
	GetNode(nodeID int32) datastructure.CHNode
	GetOutEdge(edgeID int32) datastructure.EdgeCH
	GetInEdge(edgeID int32) datastructure.EdgeCH
	GetNumNodes() int
}

type Heuristics struct {
	route RouteAlgorithm
	ch    ContractedGraph
}

func NewHeuristics(route RouteAlgorithm, ch ContractedGraph) *Heuristics {
	return &Heuristics{
		route: route,
		ch:    ch,
	}
}

func (aco *Heuristics) TravelingSalesmanProblemAntColonyOptimization(cities []int32) ([]datastructure.Coordinate, []datastructure.EdgeCH, float64, float64, [][]float64) {

	spPair := [][]int32{}
	for i := 0; i < len(cities); i++ {
		for j := 0; j < len(cities); j++ {
			if i == j {
				continue
			}
			spPair = append(spPair, []int32{cities[i], cities[j]})
		}
	}

	workers := concurrent.NewWorkerPool[[]int32, datastructure.SPSingleResultResult](10, len(spPair))

	for i := 0; i < len(spPair); i++ {
		workers.AddJob(spPair[i])
	}
	workers.Close()

	spMap := make(map[int32]map[int32]datastructure.SPSingleResultResult)

	workers.Start(aco.route.CallBidirectionalDijkstra)
	go func() {
		workers.Wait()
	}()

	for i := 0; i < len(spPair); i++ {
		spMap[spPair[i][0]] = make(map[int32]datastructure.SPSingleResultResult)
	}
	spMap[cities[len(cities)-1]] = make(map[int32]datastructure.SPSingleResultResult)

	for curr := range workers.CollectResults() {

		spMap[curr.Source][curr.Dest] = curr
	}

	distancesMat := make([][]float64, len(cities))
	for i := 0; i < len(cities); i++ {
		distancesMat[i] = make([]float64, len(cities))
		for j := 0; j < len(cities); j++ {
			if i == j {
				distancesMat[i][j] = 0
			}
			distancesMat[i][j] = spMap[cities[i]][cities[j]].Eta // pake eta karena road network ada hierarchy nya
		}
	}

	bestTourCitiesLatLon := [][]float64{}

	acoTSP := NewACO(distancesMat, 30, 1.0, 0.5, 0.1, 2.0, 0.0, 0.05, len(cities))
	bestTour, bestETA := acoTSP.Solve(500, 150) // solve tsp pake ant colony optimization
	tspTourNodes := []datastructure.Coordinate{}
	tspTourEdgePath := []datastructure.EdgeCH{}
	bestDistance := 0.0
	for i := 0; i < len(bestTour); i++ {

		currPathNodes := spMap[cities[bestTour[i]]][cities[bestTour[(i+1)%len(bestTour)]]].Paths
		bestDistance += spMap[cities[bestTour[i]]][cities[bestTour[(i+1)%len(bestTour)]]].Dist
		currEdgePath := spMap[cities[bestTour[i]]][cities[bestTour[(i+1)%len(bestTour)]]].EdgePath
		bestTourCitiesLatLon = append(bestTourCitiesLatLon, []float64{aco.ch.GetNode(cities[bestTour[i]]).Lat, aco.ch.GetNode(cities[bestTour[i]]).Lon})
		tspTourNodes = append(tspTourNodes, currPathNodes...)
		tspTourEdgePath = append(tspTourEdgePath, currEdgePath...)
	}

	return tspTourNodes, tspTourEdgePath, bestETA, bestDistance, bestTourCitiesLatLon
}
