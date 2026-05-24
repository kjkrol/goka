package astar_test

import (
	"fmt"

	"github.com/kjkrol/astar"
)

// --- Iter Examples ---

func ExampleSolver_Iter_defaultDict() {
	// Requesting a 64x64 grid with a cost weight of 1.0
	const size = 64
	grid := setupGrid(size)
	pathFinder := setupPathFinder()

	transitions := setupTransitions(1.0, grid)
	start := Point{X: 0, Y: 0}
	goal := Point{X: size - 1, Y: size - 1}

	// Consume the iterator until completion.
	// The sequence self-terminates as soon as the goal is found.
	for range pathFinder.Iter(start, goal, transitions) {
	}

	path := pathFinder.Result()

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
}

func ExampleSolver_Iter_indexedMapDict() {
	const size = 64
	grid := setupGrid(size)
	indexer := func(p Point) int { return p.Y*size + p.X }
	pathFinder := setupPathFinder(
		astar.WithInitCapacity[Point](size*size),
		astar.WithIndexedMapDict(indexer),
	)

	transitions := setupTransitions(1.0, grid)
	start := Point{X: 0, Y: 0}
	goal := Point{X: size - 1, Y: size - 1}

	for range pathFinder.Iter(start, goal, transitions) {
	}

	path := pathFinder.Result()

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
}

func ExampleSolver_Iter_indexedSliceDict() {
	const size = 64
	grid := setupGrid(size)

	indexer := func(p Point) int { return p.Y*size + p.X }
	pathFinder := setupPathFinder(
		astar.WithIndexedSliceDict(size*size, indexer),
	)

	transitions := setupTransitions(1.0, grid)
	start := Point{X: 0, Y: 0}
	goal := Point{X: size - 1, Y: size - 1}

	for range pathFinder.Iter(start, goal, transitions) {
	}

	path := pathFinder.Result()

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
}

// --- Solve Examples ---

func ExampleSolver_Solve_defaultDict() {
	const size = 64
	grid := setupGrid(size)

	pathFinder := setupPathFinder()

	transitions := setupTransitions(1.0, grid)
	start := Point{X: 0, Y: 0}
	goal := Point{X: size - 1, Y: size - 1}

	path := pathFinder.Solve(start, goal, transitions)

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
}

func ExampleSolver_Solve_indexedMapDict() {
	const size = 64
	grid := setupGrid(size)
	indexer := func(p Point) int { return p.Y*size + p.X }
	pathFinder := setupPathFinder(
		astar.WithInitCapacity[Point](size*size),
		astar.WithIndexedMapDict(indexer),
	)

	transitions := setupTransitions(1.0, grid)
	start := Point{X: 0, Y: 0}
	goal := Point{X: size - 1, Y: size - 1}

	path := pathFinder.Solve(start, goal, transitions)

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
}

func ExampleSolver_Solve_indexedSliceDict() {
	const size = 64
	grid := setupGrid(size)

	indexer := func(p Point) int { return p.Y*size + p.X }
	pathFinder := setupPathFinder(
		astar.WithIndexedSliceDict(size*size, indexer),
	)

	transitions := setupTransitions(1.0, grid)
	start := Point{X: 0, Y: 0}
	goal := Point{X: size - 1, Y: size - 1}

	path := pathFinder.Solve(start, goal, transitions)

	if len(path) > 0 {
		fmt.Println("Path found:", true)
		fmt.Println("Start is correct:", path[0] == start)
		fmt.Println("Goal is correct:", path[len(path)-1] == goal)
	} else {
		fmt.Println("Path not found")
	}

	// Output:
	// Path found: true
	// Start is correct: true
	// Goal is correct: true
}
