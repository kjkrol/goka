package goka_test

import (
	"fmt"
	"math"

	"github.com/kjkrol/goka"
)

type Point struct {
	X, Y int
}

func setupAStar(opts ...goka.AStarOption[Point]) (*goka.AStar[Point], Point, Point) {
	const size = 64

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
	successors := goka.NewBufferedSuccessors(4, func(p Point, buffer []Point) []Point {
		for _, d := range dirs {
			nx, ny := p.X+d.X, p.Y+d.Y
			if nx >= 0 && nx < size && ny >= 0 && ny < size {
				buffer = append(buffer, Point{X: nx, Y: ny})
			}
		}
		return buffer
	})

	astar := goka.NewAStar(heuristic, cost, successors, opts...)

	return astar, start, goal
}

func ExampleAStar_Iter_withIndexer() {
	indexer := func(p Point) int { return p.Y*64 + p.X }
	astar, start, goal := setupAStar(goka.WithIndexer(64*64, indexer))

	var path []Point
	for goalAchieved, node := range astar.Iter(start, goal) {
		if goalAchieved {
			path = node.Path()
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

func ExampleAStar_Iter_withoutIndexer() {
	astar, start, goal := setupAStar()

	var path []Point
	for goalAchieved, node := range astar.Iter(start, goal) {
		if goalAchieved {
			path = node.Path()
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

func ExampleAStar_Solve_withIndexer() {
	indexer := func(p Point) int { return p.Y*64 + p.X }
	astar, start, goal := setupAStar(goka.WithIndexer(64*64, indexer))

	path := astar.Solve(start, goal)

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

func ExampleAStar_Solve_withoutIndexer() {
	astar, start, goal := setupAStar()

	path := astar.Solve(start, goal)

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
