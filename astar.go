package goka

import (
	"container/heap"
	"iter"
	"slices"
)

type Heuristic[T comparable] func(T) float64
type Cost[T comparable] func(T) float64
type Successor[T comparable] func(T) iter.Seq[T]

type AStar[T comparable] struct {
	start     *Node[T]
	goal      *Node[T]
	openPQ    *PriorityQueue[T]
	open      map[T](*Node[T])
	closed    map[T](*Node[T])
	heuristic Heuristic[T]
	cost      Cost[T]
	next      Successor[T]
}

func NewAStar[T comparable](
	start, goal T,
	heuristic Heuristic[T],
	cost Cost[T],
	next Successor[T],
) *AStar[T] {
	openPQ := &PriorityQueue[T]{}
	heap.Init(openPQ)
	open := make(map[T]*Node[T])

	startNode := &Node[T]{
		ID: start,
		G:  0,
		F:  heuristic(start),
	}
	heap.Push(openPQ, startNode)
	open[start] = startNode

	return &AStar[T]{
		start:     startNode,
		goal:      &Node[T]{ID: goal},
		heuristic: heuristic,
		cost:      cost,
		openPQ:    openPQ,
		open:      open,
		closed:    make(map[T]*Node[T]),
		next:      next,
	}
}

func (a *AStar[T]) Run() []T {
	for a.openPQ.Len() > 0 {
		current := heap.Pop(a.openPQ).(*Node[T])
		delete(a.open, current.ID)

		if current.ID == a.goal.ID {
			return current.Path()
		}

	successors_loop:
		for nextID := range a.next(current.ID) {
			successor := &Node[T]{
				ID:     nextID,
				Parent: current,
			}
			successor.G = current.G + a.cost(nextID)
			successor.F = successor.G + a.heuristic(nextID)

			inOpen := false
			if existingNode, ok := a.open[successor.ID]; ok {
				if existingNode.F <= successor.F {
					continue successors_loop
				}
				inOpen = ok
			}

			if existingNode, ok := a.closed[successor.ID]; ok {
				if existingNode.F <= successor.F {
					continue successors_loop
				}
				delete(a.closed, nextID)
			}

			if inOpen {
				// refresh existing node
				x := a.open[nextID]
				x.Parent = current
				x.G = successor.G
				x.F = successor.F
				heap.Fix(a.openPQ, x.Index)
			} else {
				// add new node
				a.open[nextID] = successor
				heap.Push(a.openPQ, successor)
			}
		}

		a.closed[current.ID] = current
	}
	return nil
}

// ---------------
// Node
// ---------------
type Node[T comparable] struct {
	ID     T
	G      float64
	F      float64
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
// Priority Queue
// ---------------
type PriorityQueue[T comparable] []*Node[T]

var _ (heap.Interface) = (*PriorityQueue[any])(nil)

func (q *PriorityQueue[T]) Push(x any) {
	newNode := x.(*Node[T])
	newNode.Index = len(*q)
	*q = append(*q, newNode)
}
func (q *PriorityQueue[T]) Pop() any {
	old := *q
	n := len(old)
	node := old[n-1]
	old[n-1] = nil
	node.Index = -1
	*q = old[0 : n-1]
	return node
}
func (q *PriorityQueue[T]) Len() int           { return len(*q) }
func (q *PriorityQueue[T]) Less(i, j int) bool { return (*q)[i].F < (*q)[j].F }
func (q *PriorityQueue[T]) Swap(i, j int) {
	(*q)[i].Index = j
	(*q)[j].Index = i
	(*q)[i], (*q)[j] = (*q)[j], (*q)[i]
}
