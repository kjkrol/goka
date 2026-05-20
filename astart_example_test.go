package goka

import (
	"fmt"
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
	heuristic := func(p Point) float64 {
		return math.Abs(float64(goal.X-p.X)) + math.Abs(float64(goal.Y-p.Y))
	}

	// 4. Cost function
	// Note that the function "sees" the grid variable declared above!
	cost := func(p Point) float64 {
		return grid[p.Y][p.X]
	}

	// 5. Function generating neighbors (Moves: Left, Right, Up, Down)
	next := func(p Point) []Point {
		var neighbors []Point
		// Directions in order: Left, Right, Up, Down
		dirs := []Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}

		for _, d := range dirs {
			nx, ny := p.X+d.X, p.Y+d.Y
			// Add neighbor only if it doesn't go out of bounds of the 64x64 map
			if nx >= 0 && nx < size && ny >= 0 && ny < size {
				neighbors = append(neighbors, Point{X: nx, Y: ny})
			}
		}
		return neighbors
	}

	astar := NewAStar(start, goal, heuristic, cost, next)
	path := astar.Run()

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
