package riderdrivermatching

import (
	"errors"
	"math"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
)


// padMatrix tambahkan dummy row/column jika matrix rectangle (sampai jadi square matrix)
func padMatrix(matrix [][]float64) {
	iSize := len(matrix)
	jSize := len(matrix[0])

	if iSize > jSize {
		for i := range matrix {
			for len(matrix[i]) < iSize {
				matrix[i] = append(matrix[i], math.MaxInt32)
			}
		}
	} else if iSize < jSize {
		for len(matrix) < jSize {
			row := make([]float64, jSize)
			for i := range row {
				row[i] = math.MaxFloat64
			}
			matrix = append(matrix, row)
		}
	}
}

// step1 For each row, its minimum element is subtracted from every element in that row.
//
//	the minimum element in each column is subtracted from all the elements in that column
func step1(matrix [][]float64, step *int) {

	for i := range matrix {
		min := math.MaxFloat64
		for _, val := range matrix[i] {
			if val < min {
				min = val
			}
		}
		if min > 0 {
			for j := range matrix[i] {
				matrix[i][j] -= min
			}
		}
	}

	sz := len(matrix)
	for j := 0; j < sz; j++ {
		min := math.MaxFloat64

		for i := 0; i < sz; i++ {
			if matrix[i][j] < min {
				min = matrix[i][j]
			}
		}

		if min > 0 {
			for i := 0; i < sz; i++ {
				matrix[i][j] -= min
			}
		}
	}

	*step = 2
}

// clearCovers reset elemens ke 0.
func clearCovers(cover []int) {
	for i := range cover {
		cover[i] = 0
	}
}

// step2 All zeros in the matrix must be covered by marking as few rows and/or columns as possible
func step2(matrix [][]float64, M [][]int, RowCover, ColCover []int, step *int) {
	sz := len(matrix)
	for r := 0; r < sz; r++ {
		for c := 0; c < sz; c++ {
			if matrix[r][c] == 0 && RowCover[r] == 0 && ColCover[c] == 0 {
				M[r][c] = 1
				RowCover[r] = 1
				ColCover[c] = 1
			}
		}
	}
	clearCovers(RowCover)
	clearCovers(ColCover)
	*step = 3
}

// step3 Cover all columns containing a (starred) zero.
func step3(M [][]int, ColCover []int, step *int) {
	sz := len(M)
	colCount := 0
	for r := 0; r < sz; r++ {
		for c := 0; c < sz; c++ {
			if M[r][c] == 1 {
				ColCover[c] = 1
			}
		}
	}

	for _, n := range ColCover {
		if n == 1 {
			colCount++
		}
	}

	if colCount >= sz {
		/*
			If the number of starred zeros is n (or min(n,m)), where n is the number of people and m is the number of jobs), the algorithm terminates
		*/
		*step = 7 // Solution found
	} else {
		*step = 4
	}
}

// findAZero finds an uncovered zero in the matrix.
func findAZero(matrix [][]float64, RowCover, ColCover []int) (int, int) {
	sz := len(matrix)
	for r := 0; r < sz; r++ {
		for c := 0; c < sz; c++ {
			if matrix[r][c] == 0 && RowCover[r] == 0 && ColCover[c] == 0 {
				return r, c
			}
		}
	}
	return -1, -1
}

// starInRow checks if there's a starred zero in the given row.
func starInRow(row int, M [][]int) bool {
	for _, val := range M[row] {
		if val == 1 {
			return true
		}
	}
	return false
}

// findStarInRow finds the column of the starred zero in the given row.
func findStarInRow(row int, M [][]int) int {
	for c, val := range M[row] {
		if val == 1 {
			return c
		}
	}
	return -1
}

// Step4 Find a non-covered zero and prime it (mark it with a prime symbol). If no such zero can be found, meaning all zeroes are covered, skip to step 6.
func Step4(matrix [][]float64, M [][]int, RowCover, ColCover []int, pathRow0, pathCol0 *int, step *int) {

	for {
		// Find a non-covered zero and prime it (mark it with a prime symbol). If no such zero can be found, meaning all zeroes are covered, skip to step 6.
		r, c := findAZero(matrix, RowCover, ColCover)
		if r == -1 {
			*step = 6
			return
		} else {
			M[r][c] = 2
			if starInRow(r, M) {
				// If the zero is on the same row as a starred zero, cover the corresponding row, and uncover the column of the starred zero.
				starCol := findStarInRow(r, M)
				RowCover[r] = 1
				ColCover[starCol] = 0
			} else {
				// Else the non-covered zero has no assigned zero on its row. We make a path starting from the zero

				*pathRow0 = r
				*pathCol0 = c
				*step = 5
				return
			}
		}
	}
}

// findStarInCol finds the row of the starred zero in the given column.
func findStarInCol(c int, M [][]int) int {
	for r := 0; r < len(M); r++ {
		if M[r][c] == 1 {
			return r
		}
	}
	return -1
}

// findPrimeInRow finds the column of the primed zero in the given row.
func findPrimeInRow(r int, M [][]int) int {
	for c, val := range M[r] {
		if val == 2 {
			return c
		}
	}
	return -1
}

// augmentPath For all zeros encountered during the path, star primed zeros and unstar starred zeros.
func augmentPath(path [][]int, pathCount int, M [][]int) {
	for p := 0; p < pathCount; p++ {
		r, c := path[p][0], path[p][1]
		if M[r][c] == 1 {
			// unstar starred zeros.
			M[r][c] = 0
		} else {
			// star primed zeros
			M[r][c] = 1
		}
	}
}

// erasePrimes removes all primed zeros from the mask matrix.
func erasePrimes(M [][]int) {
	for r := 0; r < len(M); r++ {
		for c := 0; c < len(M[r]); c++ {
			if M[r][c] == 2 {
				M[r][c] = 0
			}
		}
	}
}

/*
step5 Substep 1: Find a starred zero on the corresponding column. If there is one, go to Substep 2, else, stop.

			Substep 2: Find a primed zero on the corresponding row (there should always be one). Go to Substep 1.

	 For all zeros encountered during the path, star primed zeros and unstar starred zeros.
	 Unprime all primed zeroes and uncover all lines.
	 Repeat Step 3.
*/
func step5(path [][]int, pathRow0, pathCol0 int, M [][]int, RowCover, ColCover []int, step *int) {
	r := -1
	c := -1
	pathCount := 1

	path[pathCount-1][0] = pathRow0
	path[pathCount-1][1] = pathCol0

	/*
		Substep 1: Find a starred zero on the corresponding column. If there is one, go to Substep 2, else, stop.
		Substep 2: Find a primed zero on the corresponding row (there should always be one). Go to Substep 1.
	*/

	for {
		r = findStarInCol(path[pathCount-1][1], M) // Find a starred zero on the corresponding column
		if r > -1 {
			pathCount++
			path[pathCount-1][0] = r
			path[pathCount-1][1] = path[pathCount-2][1]
		} else {
			break
		}

		c = findPrimeInRow(path[pathCount-1][0], M)
		if c != -1 {
			pathCount++
			path[pathCount-1][0] = path[pathCount-2][0]
			path[pathCount-1][1] = c
		} else {
			break
		}
	}

	augmentPath(path, pathCount, M) // For all zeros encountered during the path, star primed zeros and unstar starred zeros.
	clearCovers(RowCover)           // uncover all lines.
	clearCovers(ColCover)           // uncover all lines.
	erasePrimes(M)                  // Unprime all primed zeroes
	*step = 3
}

// findSmallest finds the smallest uncovered value in the matrix.
func findSmallest(matrix [][]float64, RowCover, ColCover []int) float64 {
	minval := math.MaxFloat64
	for r := 0; r < len(matrix); r++ {
		for c := 0; c < len(matrix[r]); c++ {
			if RowCover[r] == 0 && ColCover[c] == 0 && matrix[r][c] < minval {
				minval = matrix[r][c]
			}
		}
	}
	return minval
}

/*
step6 Otherwise, find the lowest uncovered value. Subtract this from every unmarked element and add it to every element covered by two lines. Go back to step 4.
*/
func step6(matrix [][]float64, RowCover, ColCover []int, step *int) {
	minval := findSmallest(matrix, RowCover, ColCover)
	for r := 0; r < len(matrix); r++ {
		for c := 0; c < len(matrix[r]); c++ {
			if RowCover[r] == 1 {
				matrix[r][c] += minval
			}
			if ColCover[c] == 0 {
				matrix[r][c] -= minval
			}
		}
	}
	*step = 4
}

// outputSolution hitung optimal cost
func outputSolution(original [][]float64, M [][]int) float64 {
	res := 0.0
	for r := 0; r < len(original); r++ {
		for c := 0; c < len(original[r]); c++ {
			if M[r][c] == 1 {
				res += original[r][c]
			}
		}
	}
	return res
}

type WorkerPool interface {
	AddJob(job []int32)
	Start(jobFunc func(job []int32) datastructure.SPSingleResultResult)
	Wait()
	Close()
}
type RouteAlgorithm interface {
	CallBidirectionalDijkstra(spMap []int32) datastructure.SPSingleResultResult
	CreateDistMatrix(spPair [][]int32) map[int32]map[int32]datastructure.SPSingleResultResult
}

type Hungarian struct {
	route RouteAlgorithm
}

func NewHungarian(route RouteAlgorithm) *Hungarian {
	return &Hungarian{route: route}
}

// Hungarian solve rider-driver matchmaking pakai algoritma hungarian.
func (h *Hungarian) Solve(original [][]float64) (float64, map[int]int, error) {
	if len(original) == 0 || len(original[0]) == 0 {
		return 0, map[int]int{}, errors.New("empty matrix")
	}

	matrix := make([][]float64, len(original))
	for i := range original {
		matrix[i] = make([]float64, len(original[i]))
		copy(matrix[i], original[i])
	}

	// Buat Rectangle matrix jadi square matrix (jumlah rider and driver beda)
	padMatrix(matrix)
	sz := len(matrix)

	// M[i][j] == 1 jika matrix[i][j] starred, == 2 jika primed, == 0 normal
	M := make([][]int, sz)
	for i := range M {
		M[i] = make([]int, sz)
	}

	// Buat nandain row/column yang tercover
	RowCover := make([]int, sz)
	ColCover := make([]int, sz)

	// Path yang dibuat step 5 (atau step 4 di wikipedia)
	path := make([][]int, sz+1)
	for i := range path {
		path[i] = make([]int, 2)
	}

	step := 1
	done := false
	for !done {
		switch step {

		case 1:
			step1(matrix, &step)
		case 2:
			step2(matrix, M, RowCover, ColCover, &step)
		case 3:
			step3(M, ColCover, &step)
		case 4:
			Step4(matrix, M, RowCover, ColCover, &path[0][0], &path[0][1], &step)
		case 5:
			step5(path, path[0][0], path[0][1], M, RowCover, ColCover, &step)
		case 6:
			step6(matrix, RowCover, ColCover, &step)
		case 7:
			for i := 0; i < len(M); i++ {
				M[i] = M[i][:len(original[i])]
			}
			M = M[:len(original)]
			done = true
		default:
			done = true
		}
	}

	total := outputSolution(original, M)
	match := make(map[int]int)
	for i := 0; i < len(M); i++ {
		for j := 0; j < len(M[i]); j++ {
			if M[i][j] == 1 {
				match[i] = j
			}
		}
	}
	return total, match, nil
}
