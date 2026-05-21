package goka

import (
	"fmt"
	"iter"
	"math"
)

type Point struct {
	X, Y int
}

func ExampleAStar() {
	const size = 64

	// 2. Initialize 64x64 grid
	grid := make([][]float64, size)
	for y := range size {
		grid[y] = make([]float64, size)
		for x := range size {
			// Generate repeatable weights from 1 to 5 to make the test predictable
			grid[y][x] = float64(((x + y) % 5) + 1)
		}
	}

	start := Point{X: 0, Y: 0}
	goal := Point{X: 63, Y: 63}

	// 3. Heuristic function (Manhattan distance for 4-way movement)
	// Note that the function "sees" the goal variable declared above!
	heuristic := func(p, g Point) float64 {
		return math.Abs(float64(g.X-p.X)) + math.Abs(float64(g.Y-p.Y))
	}

	// 4. Cost function
	// Note that the function "sees" the grid variable declared above!
	cost := func(p Point) float64 {
		return grid[p.Y][p.X]
	}

	// 5. Function generating neighbors using iterators (Zero allocations!)
	next := func(p Point) iter.Seq[Point] {
		return func(yield func(Point) bool) {
			dirs := []Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}

			for _, d := range dirs {
				nx, ny := p.X+d.X, p.Y+d.Y
				if nx >= 0 && nx < size && ny >= 0 && ny < size {
					// Yield the neighbor to the for-range loop in AStar.Run()
					// If the loop terminates early (yield returns false), stop generating.
					if !yield(Point{X: nx, Y: ny}) {
						return
					}
				}
			}
		}
	}

	astar := NewAStar(heuristic, cost, next)
	astar.Init(start, goal)
	path := astar.Solve()

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
