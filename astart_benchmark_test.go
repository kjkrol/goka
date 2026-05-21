package goka_test

import (
	"math"
	"testing"

	"github.com/kjkrol/goka"
)

func BenchmarkAStar_Solve(b *testing.B) {
	const size = 64

	grid := make([][]float64, size)
	for y := range size {
		grid[y] = make([]float64, size)
		for x := range size {
			grid[y][x] = float64(((x + y) % 5) + 1)
		}
	}

	start := Point{X: 0, Y: 0}
	goal := Point{X: size - 1, Y: size - 1}

	heuristic := func(p, g Point) float64 {
		return math.Abs(float64(g.X-p.X)) + math.Abs(float64(g.Y-p.Y))
	}

	cost := func(p Point) float64 {
		return grid[p.Y][p.X]
	}

	dirs := []Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}
	successors := goka.NewBufferedSuccessors(4, func(p Point, buffer []Point) []Point {
		for _, d := range dirs {
			nx, ny := p.X+d.X, p.Y+d.Y
			if nx >= 0 && nx < size && ny >= 0 && ny < size {
				buffer = append(buffer, Point{X: nx, Y: ny})
			}
		}
		return buffer
	})

	b.Run("WithIndexer", func(b *testing.B) {
		indexer := func(p Point) int { return p.Y*size + p.X }
		astar := goka.NewAStar(heuristic, cost, successors, goka.WithIndexer(size*size, indexer))

		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			_ = astar.Solve(start, goal)
		}
	})

	b.Run("WithoutIndexer", func(b *testing.B) {
		astar := goka.NewAStar(heuristic, cost, successors)

		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			_ = astar.Solve(start, goal)
		}
	})
}
