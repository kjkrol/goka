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
	openPQ     *openNodesPriorityQueue[T]
	open       map[T](*Node[T])
	closed     map[T](*Node[T])
	heuristic  Heuristic[T]
	cost       Cost[T]
	successors SuccessorSupplier[T]
	arena      *nodeArena[T]
}

func NewAStar[T comparable](
	heuristic Heuristic[T],
	cost Cost[T],
	successors SuccessorSupplier[T],
) *AStar[T] {
	openPQ := &openNodesPriorityQueue[T]{}
	heap.Init(openPQ)
	open := make(map[T]*Node[T], defaultMapSize)
	closed := make(map[T]*Node[T], defaultMapSize)

	return &AStar[T]{
		heuristic:  heuristic,
		cost:       cost,
		openPQ:     openPQ,
		open:       open,
		closed:     closed,
		successors: successors,
		arena:      newNodeArena[T](),
	}
}

func (a *AStar[T]) Init(start, goal T) {
	a.openPush(start, nil, 0, a.heuristic(start, goal))
	a.goalID = goal
}

func (a *AStar[T]) Solve() []T {
	for a.openPQ.Len() > 0 {

		current := a.openPop()

		if current.ID == a.goalID {
			return current.Path()
		}

		for successorID := range a.successors(current.ID) {
			a.processSuccessor(successorID, current)
		}

		a.closed[current.ID] = current
	}
	return nil
}

func (a *AStar[T]) processSuccessor(successorID T, current *Node[T]) {
	tentativeG := current.G + a.cost(successorID)
	tentativeF := tentativeG + a.heuristic(successorID, a.goalID)

	inOpen, isBetter := a.isOpenHasBetter(successorID, tentativeG)
	if isBetter {
		return
	}

	if a.isClosedHasBetter(successorID, tentativeG) {
		return
	}

	if inOpen {
		a.openUpdate(successorID, current, tentativeG, tentativeF)
	} else {
		a.openPush(successorID, current, tentativeG, tentativeF)
	}
}

func (a *AStar[T]) openPush(id T, parent *Node[T], g, f float64) {
	node := a.arena.Get()
	node.ID = id
	node.Parent = parent
	node.G = g
	node.F = f
	node.Index = -1
	heap.Push(a.openPQ, node)
	a.open[node.ID] = node
}

func (a *AStar[T]) openUpdate(id T, parent *Node[T], g, f float64) {
	x := a.open[id]
	x.Parent = parent
	x.G = g
	x.F = f
	heap.Fix(a.openPQ, x.Index)
}

func (a *AStar[T]) openPop() *Node[T] {
	node := heap.Pop(a.openPQ).(*Node[T])
	delete(a.open, node.ID)
	return node
}

func (a *AStar[T]) isOpenHasBetter(successorID T, tentativeG float64) (exists, hasBetter bool) {
	if existingNode, ok := a.open[successorID]; ok {
		exists = true
		hasBetter = existingNode.G <= tentativeG
	}
	return
}

func (a *AStar[T]) isClosedHasBetter(successorID T, tentativeG float64) bool {
	if existingNode, ok := a.closed[successorID]; ok {
		hasBetter := existingNode.G <= tentativeG
		if !hasBetter {
			delete(a.closed, successorID)
		}
		return hasBetter
	}
	return false
}

func (a *AStar[T]) Reset() {
	clear(a.open)
	clear(a.closed)

	if a.openPQ != nil {
		clear(*a.openPQ)
		*a.openPQ = (*a.openPQ)[:0]
	}

	a.arena.Reset()

	var zero T
	a.goalID = zero
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

func (n *Node[T]) Reset() {
	var zero T
	n.ID = zero
	n.G = 0
	n.F = 0
	n.Parent = nil
	n.Index = -1
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
