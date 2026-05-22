package astar

// ---------------
//	Buffered Successors
// ---------------

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
