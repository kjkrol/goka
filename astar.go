package goka

import (
	"container/heap"
	"iter"
	"slices"
)

const defaultMapSize = 1024

type Heuristic[T comparable] func(current, goal T) float64
type Cost[T comparable] func(T) float64
type SuccessorSupplier[T comparable] func(T) iter.Seq[T]

type AStar[T comparable] struct {
	goalID     T
	open       *open[T]
	closed     *closed[T]
	heuristic  Heuristic[T]
	cost       Cost[T]
	successors SuccessorSupplier[T]
}

func NewAStar[T comparable](
	heuristic Heuristic[T],
	cost Cost[T],
	successors SuccessorSupplier[T],
) *AStar[T] {
	return &AStar[T]{
		heuristic:  heuristic,
		cost:       cost,
		open:       newOpen[T](),
		closed:     newClosed[T](),
		successors: successors,
	}
}

func (a *AStar[T]) Init(start, goal T) {
	a.open.insert(start, nil, 0, a.heuristic(start, goal))
	a.goalID = goal
}

func (a *AStar[T]) Solve() []T {
	for a.open.isNotEmpty() {
		current := a.open.removeBest()
		if current.ID == a.goalID {
			return current.Path()
		}
		a.step(current)
	}
	return nil
}

func (a *AStar[T]) Steps() iter.Seq2[bool, []T] {
	return func(yield func(bool, []T) bool) {
		for a.open.isNotEmpty() {
			current := a.open.removeBest()

			if current.ID == a.goalID {
				yield(true, current.Path())
				return
			}

			a.step(current)

			if !yield(false, current.Path()) {
				return
			}
		}

		yield(false, nil)
	}
}

func (a *AStar[T]) step(current *Node[T]) {
	for successorID := range a.successors(current.ID) {
		G := current.G + a.cost(successorID)
		F := G + a.heuristic(successorID, a.goalID)

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
			a.open.update(successorID, current, G, F)
		} else {
			a.open.insert(successorID, current, G, F)
		}
	}
	a.closed.insert(current)
}

func (a *AStar[T]) Reset() {
	a.closed.reset()
	a.open.reset()

	var zero T
	a.goalID = zero
}

// ---------------
// Closed Nodes
// ---------------
type closed[T comparable] struct {
	closedMap map[T](*Node[T])
}

func newClosed[T comparable]() *closed[T] {
	return &closed[T]{
		closedMap: make(map[T]*Node[T], defaultMapSize),
	}
}

func (c *closed[T]) insert(node *Node[T]) {
	c.closedMap[node.ID] = node
}

func (c *closed[T]) containsBetterOrEqual(successorID T, tentativeG float64) (exists, hasBetter bool) {
	if existingNode, ok := c.closedMap[successorID]; ok {
		exists = true
		hasBetter = existingNode.G <= tentativeG
	}
	return
}

func (c *closed[T]) remove(successorID T) {
	delete(c.closedMap, successorID)
}

func (c *closed[T]) reset() {
	clear(c.closedMap)
}

// ---------------
// Open Nodes
// ---------------
type open[T comparable] struct {
	openPQ  *openNodesPriorityQueue[T]
	openMap map[T](*Node[T])
	arena   *nodeArena[T]
}

func newOpen[T comparable]() *open[T] {
	openPQ := &openNodesPriorityQueue[T]{}
	heap.Init(openPQ)
	openMap := make(map[T]*Node[T], defaultMapSize)

	return &open[T]{
		openPQ:  openPQ,
		openMap: openMap,
		arena:   newNodeArena[T](),
	}
}

func (o *open[T]) isNotEmpty() bool {
	return o.openPQ.Len() > 0
}

func (o *open[T]) insert(id T, parent *Node[T], g, f float64) {
	node := o.arena.Get()
	node.ID = id
	node.Parent = parent
	node.G = g
	node.F = f
	node.Index = -1
	heap.Push(o.openPQ, node)
	o.openMap[node.ID] = node
}

func (o *open[T]) update(id T, parent *Node[T], g, f float64) {
	x := o.openMap[id]
	x.Parent = parent
	x.G = g
	x.F = f
	heap.Fix(o.openPQ, x.Index)
}

// best means the node with the lowest F value, which is at the top of the priority queue
func (o *open[T]) removeBest() *Node[T] {
	node := heap.Pop(o.openPQ).(*Node[T])
	delete(o.openMap, node.ID)
	return node
}

func (o *open[T]) containsBetterOrEqual(successorID T, tentativeG float64) (exists, hasBetter bool) {
	if existingNode, ok := o.openMap[successorID]; ok {
		exists = true
		hasBetter = existingNode.G <= tentativeG
	}
	return
}

func (o *open[T]) reset() {
	clear(o.openMap)
	if o.openPQ != nil {
		clear(*o.openPQ)
		*o.openPQ = (*o.openPQ)[:0]
	}
	o.arena.Reset()
}

// ---------------
// Node
// ---------------
type Node[T comparable] struct {
	ID     T
	G, F   float64
	Parent *Node[T]
	Index  int
}

func (n *Node[T]) Path() []T {
	if n == nil {
		return nil
	}

	var path []T
	current := n

	for current != nil {
		path = append(path, current.ID)
		current = current.Parent
	}

	slices.Reverse(path)

	return path
}

// ---------------
// (internal) Open Nodes Priority (by Node.F) Queue
// ---------------
type openNodesPriorityQueue[T comparable] []*Node[T]

var _ (heap.Interface) = (*openNodesPriorityQueue[any])(nil)

func (q *openNodesPriorityQueue[T]) Push(x any) {
	newNode := x.(*Node[T])
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

// ---------------
// Node Arena
// ---------------
const arenaChunkSize = 1024

type nodeArena[T comparable] struct {
	chunks   [][]Node[T]
	chunkIdx int
	nodeIdx  int
}

func newNodeArena[T comparable]() *nodeArena[T] {
	return &nodeArena[T]{
		chunks: [][]Node[T]{make([]Node[T], arenaChunkSize)},
	}
}

func (a *nodeArena[T]) Get() *Node[T] {
	if a.nodeIdx >= arenaChunkSize {
		a.chunkIdx++
		a.nodeIdx = 0
		if a.chunkIdx >= len(a.chunks) {
			a.chunks = append(a.chunks, make([]Node[T], arenaChunkSize))
		}
	}

	node := &a.chunks[a.chunkIdx][a.nodeIdx]
	a.nodeIdx++
	return node
}

func (a *nodeArena[T]) Reset() {
	a.chunkIdx = 0
	a.nodeIdx = 0
}
