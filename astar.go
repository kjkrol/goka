package goka

import (
	"container/heap"
	"iter"
	"slices"
)

const defaultMapSize = 1024

type Heuristic[T comparable] func(current, goal T) float64
type Cost[T comparable] func(T) float64
type SuccessorsFunc[T comparable] func(current T, buffer []T) []T

type AStar[T comparable] struct {
	open       *open[T]
	closed     *closed[T]
	heuristic  Heuristic[T]
	cost       Cost[T]
	successors Successors[T]
}

func NewAStar[T comparable](
	heuristic Heuristic[T],
	cost Cost[T],
	successors Successors[T],
	opts ...AStarOption[T],
) *AStar[T] {
	cfg := aStarConfig[T]{
		mapCapacity: defaultMapSize,
		indexer:     nil,
	}

	for _, opt := range opts {
		opt(&cfg)
	}

	var openDict, closedDict nodeDict[T]

	if cfg.indexer != nil {
		openDict = newSliceDict(cfg.mapCapacity, cfg.indexer)
		closedDict = newSliceDict(cfg.mapCapacity, cfg.indexer)
	} else {
		openDict = newMapDict[T](cfg.mapCapacity)
		closedDict = newMapDict[T](cfg.mapCapacity)
	}

	return &AStar[T]{
		heuristic:  heuristic,
		cost:       cost,
		open:       newOpen[T](openDict),
		closed:     newClosed[T](closedDict),
		successors: successors,
	}
}

func (a *AStar[T]) Solve(start, goal T) []T {
	for current := range a.Iter(start, goal) {
		if current.ID == goal {
			return current.Path()
		}
	}
	return nil
}

func (a *AStar[T]) Iter(start, goal T) iter.Seq[*Node[T]] {
	a.reset()
	a.open.insert(start, nil, 0, a.heuristic(start, goal))
	return func(yield func(*Node[T]) bool) {
		for a.open.isNotEmpty() {
			current := a.open.removeBest()
			if current.ID != goal {
				a.process(current, goal)
			}
			if !yield(current) {
				return
			}
		}
	}
}

func (a *AStar[T]) process(current *Node[T], goal T) {
	for _, successorID := range a.successors.Successors(current.ID) {
		G := current.G + a.cost(successorID)
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
			a.open.update(successorID, current, G, F)
		} else {
			a.open.insert(successorID, current, G, F)
		}
	}
	a.closed.insert(current)
}

func (a *AStar[T]) reset() {
	a.closed.reset()
	a.open.reset()
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

func (c *closed[T]) insert(node *Node[T]) {
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

func newOpen[T comparable](dict nodeDict[T]) *open[T] {
	openPQ := &openNodesPriorityQueue[T]{}
	heap.Init(openPQ)

	return &open[T]{
		openPQ: openPQ,
		dict:   dict,
		arena:  newNodeArena[T](),
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
	o.dict.set(node.ID, node)
}

func (o *open[T]) update(id T, parent *Node[T], g, f float64) {
	if x, ok := o.dict.get(id); ok {
		x.Parent = parent
		x.G = g
		x.F = f
		heap.Fix(o.openPQ, x.Index)
	}
}

// best means the node with the lowest F value, which is at the top of the priority queue
func (o *open[T]) removeBest() *Node[T] {
	node := heap.Pop(o.openPQ).(*Node[T])
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

// ---------------
//	Buffered Successors
// ---------------

type Successors[T comparable] interface {
	Successors(current T) []T
}

type BufferedSuccessors[T comparable] struct {
	generate SuccessorsFunc[T]
	buf      []T
}

func NewBufferedSuccessors[T comparable](capacity int, generate SuccessorsFunc[T]) *BufferedSuccessors[T] {
	return &BufferedSuccessors[T]{
		generate: generate,
		buf:      make([]T, 0, capacity),
	}
}

func (b *BufferedSuccessors[T]) Successors(current T) []T {
	b.buf = b.generate(current, b.buf[:0])
	return b.buf
}

// ---------------
// Internal Node Dictionary
// ---------------
type nodeDict[T comparable] interface {
	get(id T) (*Node[T], bool)
	set(id T, node *Node[T])
	remove(id T)
	clear()
}

// ---------------
// Map-based Node Dictionary
// ---------------
type mapDict[T comparable] struct {
	m map[T]*Node[T]
}

func newMapDict[T comparable](capacity int) *mapDict[T] {
	return &mapDict[T]{m: make(map[T]*Node[T], capacity)}
}
func (d *mapDict[T]) get(id T) (*Node[T], bool) { n, ok := d.m[id]; return n, ok }
func (d *mapDict[T]) set(id T, node *Node[T])   { d.m[id] = node }
func (d *mapDict[T]) remove(id T)               { delete(d.m, id) }
func (d *mapDict[T]) clear()                    { clear(d.m) }

// ---------------
// Slice-based Node Dictionary (for fixed-size, integer-indexable IDs)
// ---------------
type sliceDict[T comparable] struct {
	nodes   []*Node[T]
	indexOf func(T) int
}

func newSliceDict[T comparable](maxSize int, indexer func(T) int) *sliceDict[T] {
	return &sliceDict[T]{
		nodes:   make([]*Node[T], maxSize),
		indexOf: indexer,
	}
}
func (d *sliceDict[T]) get(id T) (*Node[T], bool) {
	n := d.nodes[d.indexOf(id)]
	return n, n != nil
}
func (d *sliceDict[T]) set(id T, node *Node[T]) { d.nodes[d.indexOf(id)] = node }
func (d *sliceDict[T]) remove(id T)             { d.nodes[d.indexOf(id)] = nil }
func (d *sliceDict[T]) clear()                  { clear(d.nodes) }

// ---------------
// AStar Options
// ---------------
type AStarOption[T comparable] func(*aStarConfig[T])

type aStarConfig[T comparable] struct {
	mapCapacity int
	indexer     func(T) int
}

func WithIndexer[T comparable](maxSize int, indexer func(T) int) AStarOption[T] {
	return func(c *aStarConfig[T]) {
		c.mapCapacity = maxSize
		c.indexer = indexer
	}
}

func WithMapCapacity[T comparable](capacity int) AStarOption[T] {
	return func(c *aStarConfig[T]) {
		c.mapCapacity = capacity
	}
}
