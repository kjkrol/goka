package astar_test

import (
	"fmt"

	"github.com/kjkrol/astar"
)

// Point represents a coordinate on a 2D grid.
type Point2d struct {
	X, Y int
}

func ExampleSolver_Solve() {
	// Define a simple 3x3 grid where 1 represents a wall/blocked cell.
	grid := [][]int{
		{0, 0, 0},
		{1, 1, 0},
		{0, 0, 0},
	}
	width, height := 3, 3

	// Indexer maps a Point to a unique dense integer for maximum performance.
	// This allows the slice-based dictionary to bypass heavy hashmap lookups.
	indexer := func(p Point2d) int {
		return p.Y*width + p.X
	}

	// Define a simple Manhattan distance heuristic
	heuristic := func(from, to Point2d) float64 {
		dx := from.X - to.X
		dy := from.Y - to.Y
		if dx < 0 {
			dx = -dx
		}
		if dy < 0 {
			dy = -dy
		}
		return float64(dx + dy)
	}

	// Setup the solver with maximum optimizations for a bounded space.
	solver := astar.New[Point2d](
		heuristic,
		astar.WithInitCapacity[Point2d](width*height),
		astar.WithIndexedSliceDict(width*height, indexer),
	)

	// Define transitions (allowed movements on our grid).
	transitions := func(from, prev Point2d, buffer []astar.Transition[Point2d]) []astar.Transition[Point2d] {
		// Reset buffer slice while preserving underlying allocation
		buffer = buffer[:0]

		// 4-directional movement vectors (Up, Down, Left, Right)
		dirs := []Point2d{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}

		for _, d := range dirs {
			next := Point2d{X: from.X + d.X, Y: from.Y + d.Y}

			// 1. Boundary check
			if next.X < 0 || next.X >= width || next.Y < 0 || next.Y >= height {
				continue
			}
			// 2. Obstacle check
			if grid[next.Y][next.X] == 1 {
				continue
			}
			// 3. Prevent immediate backtracking to the previous state
			if next == prev {
				continue
			}

			// Append valid step with a uniform move cost of 1.0
			buffer = append(buffer, astar.Transition[Point2d]{To: next, Cost: 1.0})
		}
		return buffer
	}

	// Define start and target states
	from := Point2d{X: 0, Y: 0}
	to := Point2d{X: 0, Y: 2}

	// Solve the problem
	path := solver.Solve(from, to, transitions)

	fmt.Printf("Path length: %d\n", len(path))
	fmt.Printf("Steps: %v\n", path)

	// Output:
	// Path length: 7
	// Steps: [{0 0} {1 0} {2 0} {2 1} {2 2} {1 2} {0 2}]
}
