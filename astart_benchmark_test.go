package goka

import (
	"math"
	"testing"
)

func BenchmarkAStar(b *testing.B) {
	const size = 64

	// 1. Setup - initialize the grid before starting the timer
	grid := make([][]float64, size)
	for y := range size {
		grid[y] = make([]float64, size)
		for x := range size {
			grid[y][x] = float64(((x + y) % 5) + 1)
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
	successors := NewBufferedSuccessors(4, func(p Point, buffer []Point) []Point {
		for _, d := range dirs {
			nx, ny := p.X+d.X, p.Y+d.Y
			if nx >= 0 && nx < size && ny >= 0 && ny < size {
				buffer = append(buffer, Point{X: nx, Y: ny})
			}
		}
		return buffer
	})

	indexer := func(p Point) int { return p.Y*64 + p.X }
	astar := NewAStar(heuristic, cost, successors, WithIndexer(4096, indexer))

	// 2. Reset the timer!
	// This ensures the time needed to generate the map is not included in the result.
	b.ResetTimer()

	// 3. Benchmark loop
	// Go will automatically adjust the value of b.N so the test runs long enough
	// (usually about 1 second) for reliable results.
	for i := 0; i < b.N; i++ {
		_ = astar.Solve(start, goal) // We don't care about the returned result in the benchmark
	}
}
