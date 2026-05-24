package astar_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/kjkrol/astar"
)

func BenchmarkSolver_Solve(b *testing.B) {
	sizes := []int{64, 256, 512, 1024, 2048}

	const insurmountableObstacle = 100.0
	for _, size := range sizes {
		b.Run(fmt.Sprintf("Size_%dx%d", size, size), func(b *testing.B) {
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
			goal := Point{X: size - 1, Y: size - 1}

			heuristic := func(p, g Point) float64 {
				return (math.Abs(float64(g.X-p.X)) + math.Abs(float64(g.Y-p.Y)))
			}

			dirs := []Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}
			// heuristic should have higher impact on result than cost for better benchmarking
			costWeight := 0.1
			var successors astar.Successors[Point] = func(
				p Point,
				buf []astar.Successor[Point],
			) []astar.Successor[Point] {
				for _, d := range dirs {
					nx, ny := p.X+d.X, p.Y+d.Y
					if nx >= 0 && nx < size && ny >= 0 && ny < size {
						cost := grid[ny][nx] * costWeight
						if cost >= insurmountableObstacle {
							continue
						}
						buf = append(buf, astar.Successor[Point]{
							ID:   Point{X: nx, Y: ny},
							Cost: cost,
						})
					}
				}
				return buf
			}

			indexer := func(p Point) int { return p.Y*size + p.X }
			maxSize := size * size

			b.Run("WithIndexedSliceDict", func(b *testing.B) {
				pathFinder := astar.New(heuristic,
					astar.WithIndexedSliceDict(maxSize, indexer),
				)

				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					_ = pathFinder.Solve(start, goal, successors)
				}
			})

			b.Run("WithIndexedMapDict", func(b *testing.B) {
				pathFinder := astar.New(heuristic,
					astar.WithInitCapacity[Point](maxSize),
					astar.WithIndexedMapDict(indexer),
				)

				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					_ = pathFinder.Solve(start, goal, successors)
				}
			})

			b.Run("DefaultMapDict", func(b *testing.B) {
				pathFinder := astar.New(heuristic,
					astar.WithInitCapacity[Point](maxSize),
				)

				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					_ = pathFinder.Solve(start, goal, successors)
				}
			})
		})
	}
}
