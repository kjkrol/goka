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

			indexer := func(p Point) int { return p.Y*size + p.X }
			maxSize := size * size

			b.Run("WithIndexedSliceDict", func(b *testing.B) {
				pathFinder := astar.New(heuristic, cost, successors,
					astar.WithIndexedSliceDict(maxSize, indexer),
				)

				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					_ = pathFinder.Solve(start, goal)
				}
			})

			b.Run("WithIndexedMapDict", func(b *testing.B) {
				pathFinder := astar.New(heuristic, cost, successors,
					astar.WithInitCapacity[Point](maxSize),
					astar.WithIndexedMapDict(indexer),
				)

				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					_ = pathFinder.Solve(start, goal)
				}
			})

			b.Run("DefaultMapDict", func(b *testing.B) {
				pathFinder := astar.New(heuristic, cost, successors,
					astar.WithInitCapacity[Point](maxSize),
				)

				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					_ = pathFinder.Solve(start, goal)
				}
			})
		})
	}
}
