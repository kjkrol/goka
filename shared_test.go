package astar_test

import (
	"math"

	"github.com/kjkrol/astar"
)

type Point struct {
	X, Y int
}

const InsurmountableObstacle = 100.0

// setupPathFinder creates a deterministic grid of a given size, applies a cost multiplier,
// and returns the initialized solver along with boundaries and transition rules.

func setupGrid(size int) [][]float64 {
	// Allocate grid dynamically based on the requested benchmark/test size
	grid := make([][]float64, size)
	for y := range size {
		grid[y] = make([]float64, size)
		for x := range size {
			val := float64(((x + y) % 5) + 1)

			if x%4 == 2 && y%4 != 2 {
				val = InsurmountableObstacle
			}

			if (x == 0 && y == 0) || (x == size-1 && y == size-1) {
				val = 1
			}

			grid[y][x] = val
		}
	}

	return grid
}

func setupPathFinder(
	opts ...astar.SolverOption[Point],
) *astar.Solver[Point] {

	heuristic := func(p, g Point) float64 {
		return math.Abs(float64(g.X-p.X)) + math.Abs(float64(g.Y-p.Y))
	}

	pathFinder := astar.New(heuristic, opts...)

	return pathFinder
}

func setupTransitions(costWeight float64, grid [][]float64) astar.Transitions[Point] {
	size := len(grid)
	dirs := []Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}
	var transitions astar.Transitions[Point] = func(
		p, pp Point,
		buf []astar.Transition[Point],
	) []astar.Transition[Point] {
		for _, d := range dirs {
			nx, ny := p.X+d.X, p.Y+d.Y
			if nx >= 0 && nx < size && ny >= 0 && ny < size {
				if nx == pp.X && ny == pp.Y {
					continue
				}

				// Apply the dynamic scenario weight to the terrain cost
				cost := grid[ny][nx] * costWeight
				if cost >= InsurmountableObstacle {
					continue
				}

				buf = append(buf, astar.Transition[Point]{
					To:   Point{X: nx, Y: ny},
					Cost: cost,
				})
			}
		}
		return buf
	}
	return transitions
}
