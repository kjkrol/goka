package astar_test

import (
	"fmt"
	"math"

	"github.com/kjkrol/astar"
)

type Point struct {
	X, Y int
}

func setupPathFinder(opts ...astar.SolverOption[Point]) (*astar.Solver[Point], Point, Point) {
	const size = 64
	const insurmountableObstacle = 100.0

	grid := make([][]float64, size)
	for y := range size {
		grid[y] = make([]float64, size)
		for x := range size {
			val := float64(((x + y) % 5) + 1)

			if x%4 == 2 && y%4 != 2 {
				val = insurmountableObstacle
			}

			if (x == 0 && y == 0) || (x == size-1 && y == size-1) {
				val = 1
			}

			grid[y][x] = val
		}
	}

	start := Point{X: 0, Y: 0}
	goal := Point{X: 63, Y: 63}

	heuristic := func(p, g Point) float64 {
		return math.Abs(float64(g.X-p.X)) + math.Abs(float64(g.Y-p.Y))
	}

	cost := func(p Point) float64 {
		return grid[p.Y][p.X]
	}

	dirs := []Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}
	var successors astar.SuccessorsFunc[Point] = func(p, pp Point, buffer []Point) []Point {
		for _, d := range dirs {
			nx, ny := p.X+d.X, p.Y+d.Y
			if nx >= 0 && nx < size && ny >= 0 && ny < size {
				if nx == pp.X && ny == pp.Y {
					continue
				}
				if grid[ny][nx] == insurmountableObstacle {
					continue
				}
				buffer = append(buffer, Point{X: nx, Y: ny})
			}
		}
		return buffer
	}

	pathFinder := astar.New(heuristic, cost, successors, opts...)

	return pathFinder, start, goal
}

// --- Iter Examples ---

func ExampleSolver_Iter_defaultDict() {
	pathFinder, start, goal := setupPathFinder()

	var path []Point
	for goalAchieved := range pathFinder.Iter(start, goal) {
		if goalAchieved {
			path = pathFinder.Result()
			break
		}
	}

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
		fmt.Println("Path length >= 127:", len(path) >= 127)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
	// Path length >= 127: true
}

func ExampleSolver_Iter_indexedMapDict() {
	indexer := func(p Point) int { return p.Y*64 + p.X }
	pathFinder, start, goal := setupPathFinder(
		astar.WithInitCapacity[Point](64*64),
		astar.WithIndexedMapDict(indexer),
	)

	var path []Point
	for goalAchieved := range pathFinder.Iter(start, goal) {
		if goalAchieved {
			path = pathFinder.Result()
			break
		}
	}

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
		fmt.Println("Path length >= 127:", len(path) >= 127)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
	// Path length >= 127: true
}

func ExampleSolver_Iter_indexedSliceDict() {
	indexer := func(p Point) int { return p.Y*64 + p.X }
	pathFinder, start, goal := setupPathFinder(
		astar.WithIndexedSliceDict(64*64, indexer),
	)

	var path []Point
	for goalAchieved := range pathFinder.Iter(start, goal) {
		if goalAchieved {
			path = pathFinder.Result()
			break
		}
	}

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
		fmt.Println("Path length >= 127:", len(path) >= 127)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
	// Path length >= 127: true
}

// --- Solve Examples ---

func ExampleSolver_Solve_defaultDict() {
	pathFinder, start, goal := setupPathFinder()

	path := pathFinder.Solve(start, goal)

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
		fmt.Println("Path length >= 127:", len(path) >= 127)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
	// Path length >= 127: true
}

func ExampleSolver_Solve_indexedMapDict() {
	indexer := func(p Point) int { return p.Y*64 + p.X }
	pathFinder, start, goal := setupPathFinder(
		astar.WithInitCapacity[Point](64*64),
		astar.WithIndexedMapDict(indexer),
	)

	path := pathFinder.Solve(start, goal)

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
		fmt.Println("Path length >= 127:", len(path) >= 127)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
	// Path length >= 127: true
}

func ExampleSolver_Solve_indexedSliceDict() {
	indexer := func(p Point) int { return p.Y*64 + p.X }
	pathFinder, start, goal := setupPathFinder(
		astar.WithIndexedSliceDict(64*64, indexer),
	)

	path := pathFinder.Solve(start, goal)

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
		fmt.Println("Path length >= 127:", len(path) >= 127)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
	// Path length >= 127: true
}
