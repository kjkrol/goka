package astar

import (
	"container/heap"
	"iter"
	"slices"
)

// Heuristic defines the estimated cost to reach the goal state from the current state.
//
// Note on Performance vs. Optimality:
// To drastically narrow the search space and speed up the solver, the heuristic's
// contribution should dominate the actual step cost. You can achieve this by multiplying
// the heuristic result by a weight (e.g., > 1.0) or by reducing the step cost.
// This effectively turns the algorithm into Weighted A*, which visits far fewer nodes
// but sacrifices the guarantee of finding the absolute shortest path.
type Heuristic[T comparable] func(current, goal T) float64

// Transition represents a valid movement or mutation in the state space,
// capturing the destination state and the cost incurred to reach it.
type Transition[T comparable] struct {
	To   T       // The destination state of this transition.
	Cost float64 // The edge weight or cost associated with this state change.
}

// Transitions defines how states mutate and how transition costs are calculated.
// It populates the provided reusable buffer with all valid, directly reachable transitions
// from the current state in a single pass.
//
// This decoupled design keeps the Solver strictly generic and agnostic of the underlying
// graph structure, state representation, or domain-specific cost metrics. It is the
// ideal place to:
//  1. Evaluate valid state mutations and compute dynamic transition costs (edge weights).
//  2. Filter out invalid, blocked, or out-of-bounds states.
//  3. Prevent immediate backtracking or simple cycles by utilizing the 'parent' state.
//  4. Perform early pruning by excluding transitions whose cost exceeds custom thresholds.
//
// Returns the sliced buffer containing the valid transitions.
type Transitions[T comparable] func(current, parent T, buffer []Transition[T]) []Transition[T]

// Indexer maps a complex state of type T to a unique, contiguous integer identifier.
// It is required when using highly optimized internal structures like IndexedSliceDict.
type Indexer[T comparable] func(T) int

// Solver is a generic, high-performance pathfinding and state-space search engine
// based on the A* algorithm. It is entirely agnostic to the underlying domain problem.
type Solver[T comparable] struct {
	open          *open[T]
	closed        *closed[T]
	heuristic     Heuristic[T]
	transitionBuf []Transition[T]
	current       *node[T]
}

// New initializes and returns a new Solver configured with the provided
// heuristic and optional configuration parameters.
func New[T comparable](
	heuristic Heuristic[T],
	opts ...SolverOption[T],
) *Solver[T] {
	cfg := newConfig[T]()

	for _, opt := range opts {
		opt(&cfg)
	}

	openDict := cfg.dictFactory(cfg.capacity)
	closedDict := cfg.dictFactory(cfg.capacity)

	return &Solver[T]{
		open:          newOpen[T](cfg.capacity, openDict),
		closed:        newClosed[T](closedDict),
		heuristic:     heuristic,
		transitionBuf: make([]Transition[T], 0, cfg.successorCapacity),
	}
}

// Solve executes the search from the start state to the goal state.
// It runs the iterator to completion and returns the final path.
// If no path is found, it returns nil.
func (a *Solver[T]) Solve(start, goal T, successors Transitions[T]) []T {
	for goalAchieved := range a.Iter(start, goal, successors) {
		if goalAchieved {
			return a.Result()
		}
	}
	return nil
}

// Iter returns a Go 1.23 iterator sequence that allows stepping through the algorithm's execution.
// It yields 'false' while actively searching, and yields 'true' exactly once the moment
// the goal state is reached, immediately terminating the sequence afterwards.
//
// This is exceptionally useful for visualizing the state space traversal, step-by-step
// debugging, or aborting the search early based on custom external conditions.
func (a *Solver[T]) Iter(start, goal T, successors Transitions[T]) iter.Seq[bool] {
	a.reset()
	a.open.insert(start, nil, 0, a.heuristic(start, goal))
	return func(yield func(bool) bool) {
		for a.open.isNotEmpty() {
			a.current = a.open.removeBest()
			goalAchieved := (a.current.ID == goal)
			if goalAchieved {
				yield(true)
				return
			}
			a.process(goal, successors)
			if !yield(false) {
				return
			}
		}
	}
}

// Result reconstructs and returns the path from the starting state to the current state.
// It is typically called immediately after the Iter sequence yields 'true'.
func (a *Solver[T]) Result() []T {
	if a.current == nil {
		return nil
	}

	var path []T
	node := a.current

	for node != nil {
		path = append(path, node.ID)
		node = node.Parent
	}

	slices.Reverse(path)

	return path
}

func (a *Solver[T]) process(goal T, successors Transitions[T]) {
	parentID := a.current.ID
	if a.current.Parent != nil {
		parentID = a.current.Parent.ID
	}
	for _, successor := range successors(a.current.ID, parentID, a.transitionBuf[:0]) {
		successorID := successor.To
		G := a.current.G + successor.Cost
		F := G + a.heuristic(successorID, goal)

		inOpen, hasBetter := a.open.containsBetterOrEqual(successorID, G)
		if hasBetter {
			continue
		}

		inClosed, hasBetter := a.closed.containsBetterOrEqual(successorID, G)
		if hasBetter {
			continue
		}

		if inClosed {
			a.closed.remove(successorID)
		}

		if inOpen {
			a.open.update(successorID, a.current, G, F)
		} else {
			a.open.insert(successorID, a.current, G, F)
		}
	}
	a.closed.insert(a.current)
}

func (a *Solver[T]) reset() {
	a.closed.reset()
	a.open.reset()
	a.current = nil
}

// ---------------
// node
// ---------------
type node[T comparable] struct {
	ID     T
	G, F   float64
	Parent *node[T]
	Index  int
}

// ---------------
// Closed Nodes
// ---------------
type closed[T comparable] struct {
	dict nodeDict[T]
}

func newClosed[T comparable](dict nodeDict[T]) *closed[T] {
	return &closed[T]{dict: dict}
}

func (c *closed[T]) insert(node *node[T]) {
	c.dict.set(node.ID, node)
}

func (c *closed[T]) containsBetterOrEqual(successorID T, tentativeG float64) (exists, hasBetter bool) {
	if existingNode, ok := c.dict.get(successorID); ok {
		exists = true
		hasBetter = existingNode.G <= tentativeG
	}
	return
}

func (c *closed[T]) remove(successorID T) {
	c.dict.remove(successorID)
}

func (c *closed[T]) reset() {
	c.dict.clear()
}

// ---------------
// Open Nodes
// ---------------
type open[T comparable] struct {
	openPQ *openNodesPriorityQueue[T]
	dict   nodeDict[T]
	arena  *nodeArena[T]
}

func newOpen[T comparable](capacity int, dict nodeDict[T]) *open[T] {
	openPQ := &openNodesPriorityQueue[T]{}
	heap.Init(openPQ)

	return &open[T]{
		openPQ: openPQ,
		dict:   dict,
		arena:  newNodeArena[T](capacity),
	}
}

func (o *open[T]) isNotEmpty() bool {
	return o.openPQ.Len() > 0
}

func (o *open[T]) insert(id T, parent *node[T], g, f float64) {
	node := o.arena.Get()
	node.ID = id
	node.Parent = parent
	node.G = g
	node.F = f
	node.Index = -1
	heap.Push(o.openPQ, node)
	o.dict.set(node.ID, node)
}

func (o *open[T]) update(id T, parent *node[T], g, f float64) {
	if x, ok := o.dict.get(id); ok {
		x.Parent = parent
		x.G = g
		x.F = f
		heap.Fix(o.openPQ, x.Index)
	}
}

// best means the node with the lowest F value, which is at the top of the priority queue
func (o *open[T]) removeBest() *node[T] {
	node := heap.Pop(o.openPQ).(*node[T])
	o.dict.remove(node.ID)
	return node
}

func (o *open[T]) containsBetterOrEqual(successorID T, tentativeG float64) (exists, hasBetter bool) {
	if existingNode, ok := o.dict.get(successorID); ok {
		exists = true
		hasBetter = existingNode.G <= tentativeG
	}
	return
}

func (o *open[T]) reset() {
	o.dict.clear()
	if o.openPQ != nil {
		clear(*o.openPQ)
		*o.openPQ = (*o.openPQ)[:0]
	}
	o.arena.Reset()
}

// ---------------
// (internal) Open Nodes Priority (by Node.F) Queue
// ---------------
type openNodesPriorityQueue[T comparable] []*node[T]

var _ (heap.Interface) = (*openNodesPriorityQueue[any])(nil)

func (q *openNodesPriorityQueue[T]) Push(x any) {
	newNode := x.(*node[T])
	newNode.Index = len(*q)
	*q = append(*q, newNode)
}
func (q *openNodesPriorityQueue[T]) Pop() any {
	old := *q
	n := len(old)
	node := old[n-1]
	old[n-1] = nil
	node.Index = -1
	*q = old[0 : n-1]
	return node
}
func (q *openNodesPriorityQueue[T]) Len() int           { return len(*q) }
func (q *openNodesPriorityQueue[T]) Less(i, j int) bool { return (*q)[i].F < (*q)[j].F }
func (q *openNodesPriorityQueue[T]) Swap(i, j int) {
	(*q)[i].Index = j
	(*q)[j].Index = i
	(*q)[i], (*q)[j] = (*q)[j], (*q)[i]
}
