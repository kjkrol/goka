# kjkrol/astar

<p align="center">
  <img src=".github/docs/img/logo.png" alt="kjkrol/astar Logo" width="100">
  <br>
    <a href="https://go.dev">
    <img src="https://img.shields.io/badge/Go-1.23+-00ADD8?style=flat-square&logo=go" alt="Go Version">
  </a>
  <a href="https://pkg.go.dev/github.com/kjkrol/astar">
    <img src="https://img.shields.io/badge/GoDoc-Reference-007d9c?style=flat-square&logo=go" alt="GoDoc">
  </a>
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg?style=flat-square" alt="License">
  </a>
  <a href="https://github.com/kjkrol/astar/actions">
    <img src="https://github.com/kjkrol/astar/actions/workflows/go.yml/badge.svg" alt="Go Quality Check">
  </a>
</p>

**kjkrol/astart** is a **highly performant, fully generic A*** solver for Go. Perfect for optimal pathfinding and abstract state-space search.

## What is A*?

The [A* (A-star) algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) is a graph traversal and state-space search algorithm heavily used in computer science due to its completeness, optimality, and optimal efficiency. It is widely considered the industry standard for finding the shortest path or optimal sequence of transitions between nodes.

While most commonly used as a **Pathfinder**, this library provides a completely domain-agnostic `Solver`. You can use it to solve complex puzzles, optimize network routing, or navigate grids, simply by providing your own `Heuristic` and `Transitions` logic. The Solver remains strictly agnostic of the underlying graph structure or domain-specific cost metrics, making it a universal state-space resolution engine.

# 📦 Installation

GOKe requires **Go 1.23** or newer.

```bash
go get github.com/kjkrol/goke
```

---

# ⏱️ Performance

This solver is built for extreme performance. By utilizing Go 1.18+ Generics, custom memory arenas, and highly optimized data structures, we drastically reduce memory allocations and prevent heap escapes during the hot path of the search.

**Key Optimizations:**
* **`WithIndexedSliceDict`:** Replaces standard Go maps with a pre-allocated slice for tracking visited nodes. It completely eliminates hashing overhead and interface boxing.
* **Node Arena:** An internal chunk-based memory arena ensures that millions of nodes can be processed with near-zero allocations after the initial setup.

## Benchmarks (Apple M1 Max)

The benchmarks below represent execution metrics under standard A* operational stress (`CostWeight_1.0`), where the algorithm fully evaluates path costs and alternative routes. 

As the state space expands, the `IndexedSliceDict` provides massive scaling advantages over map-based lookups. For a large **2048x2048** environment, it reduces execution time from **1.41 seconds down to just 0.54 seconds**—outperforming the standard Go map implementation by **2.6x**.

| Grid Size | Dictionary Type | Time (ms/op) | Memory (kB/op) | Allocs (allocs/op) |
| :--- | :--- | :--- | :--- | :--- |
| **64x64** | `IndexedSliceDict` | 0.36 ms | 4.2 kB | 12 |
| | `IndexedMapDict` | 0.58 ms | 4.1 kB | 12 |
| | `DefaultMapDict` | 0.62 ms | 4.1 kB | 12 |
| **256x256** | `IndexedSliceDict` | 6.85 ms | 33.2 kB | 14 |
| | `IndexedMapDict` | 10.98 ms | 16.2 kB | 14 |
| | `DefaultMapDict` | 11.94 ms | 16.3 kB | 14 |
| **512x512** | `IndexedSliceDict` | 30.15 ms | 388.3 kB | 16 |
| | `IndexedMapDict` | 59.11 ms | 49.8 kB | 16 |
| | `DefaultMapDict` | 60.11 ms | 49.8 kB | 16 |
| **1024x1024** | `IndexedSliceDict` | 126.10 ms | 6,253.1 kB | 21 |
| | `IndexedMapDict` | 288.12 ms | 124.6 kB | 21 |
| | `DefaultMapDict` | 308.14 ms | 124.7 kB | 21 |
| **2048x2048** | `IndexedSliceDict` | **541.76 ms** | 98,545.3 kB | **34** |
| | `IndexedMapDict` | 1,411.57 ms | 324.5 kB | 34 |
| | `DefaultMapDict` | 1,416.12 ms | 324.5 kB | 34 |

---

# 🚀 Getting Started (Pathfinding Example)

Here is a step-by-step example of how to configure the solver to find the optimal path on a 2D terrain grid.

### Define your domain state and world grid
The solver is generic, so you define the state representation. For a grid map, a Point struct represents coordinates, and a 2D slice simulates the world terrain.

```go
type Point struct {
	X, Y int
}

const (
	GridSize = 64
	
	// Terrain types and their cost/weights
	TerrainWalkway = 1.0
	TerrainMud     = 3.5
	TerrainWall    = 999.0 // Insurmountable obstacle
)

// Example world grid layout (pre-allocated or loaded from game data)
var grid [GridSize][GridSize]float64
```

Additionally, you need to define a heuristic function (e.g., Manhattan distance) to estimate the remaining distance to the target:

```go
// Heuristic is part of your domain definition
heuristic := func(from, to Point) float64 {
	dx := math.Abs(float64(to.X - from.X))
	dy := math.Abs(float64(to.Y - from.Y))
	return dx + dy
}
```

### Initialize the Solver

Pass your static heuristic into astar.New. To unlock maximum performance on fixed state spaces, use WithIndexedSliceDict by providing an indexer function mapped to your grid dimensions.

**Note:** The solver allocates its internal memory structures once during initialization. This instance is designed to be reused sequentially across multiple distinct pathfinding queries to avoid GC pressure. It is not thread-safe; if you need concurrent pathfinding, use separate solver instances per goroutine or orchestrate them via a pool.
```go
// The Indexer maps a 2D coordinate to a unique 1D slice index
indexer := func(p Point) int { 
	return p.Y * GridSize + p.X 
}
maxNodes := GridSize * GridSize

// Initialize the Solver once with static configuration
solver := astar.New(
	heuristic,
	astar.WithIndexedSliceDict(maxNodes, indexer),
)
```

### Define Rules & Execute Search (Reusing Buffers)

Every time you call Solve(), you pass a transition rule. This allows you to dynamicly change movement logic on the fly without reallocating solver internal buffers, ensuring zero-allocation hot paths.

```go
// Transitions: Populates the pre-allocated buffer with valid moves and terrain costs in a single pass.
dirs := []Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}

transitions := func(from, prev Point, buffer []astar.Transition[Point]) []astar.Transition[Point] {
	for _, d := range dirs {
		nx, ny := from.X+d.X, from.Y+d.Y
		
		// 1. Boundary check
		if nx >= 0 && nx < GridSize && ny >= 0 && ny < GridSize {
			
			// 2. Prevent immediate backtracking to the parent node
			if nx == prev.X && ny == prev.Y {
				continue
			}
			
			// 3. Static obstacle pruning (e.g., skip walls entirely)
			terrainCost := grid[ny][nx]
			if terrainCost >= TerrainWall {
				continue
			}
			
			// 4. Register valid transition with its intrinsic edge weight
			buffer = append(buffer, astar.Transition[Point]{
				To:   Point{X: nx, Y: ny},
				Cost: terrainCost, 
			})
		}
	}
	return buffer
}

start := Point{X: 0, Y: 0}
target := Point{X: 63, Y: 63}

// Execute the search by injecting the grid transition rules into the reused solver
path := solver.Solve(start, target, transitions)

if path != nil {
	fmt.Println("Found optimal path with steps:", len(path))
}
```

# License

**kjkrol/astar** is licensed under the MIT License. See the LICENSE [file](./LICENSE) for more details.
