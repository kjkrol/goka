package astar

// ---------------
// Node Arena
// ---------------
type nodeArena[T comparable] struct {
	chunks   [][]node[T]
	chunkIdx int
	nodeIdx  int
}

func newNodeArena[T comparable](initialCapacity int) *nodeArena[T] {
	if initialCapacity <= 0 {
		initialCapacity = 1024
	}
	return &nodeArena[T]{
		chunks: [][]node[T]{make([]node[T], initialCapacity)},
	}
}

func (a *nodeArena[T]) Get() *node[T] {
	currentChunk := a.chunks[a.chunkIdx]

	if a.nodeIdx >= len(currentChunk) {
		a.chunkIdx++
		a.nodeIdx = 0

		if a.chunkIdx >= len(a.chunks) {
			newSize := len(currentChunk) * 2
			a.chunks = append(a.chunks, make([]node[T], newSize))
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
