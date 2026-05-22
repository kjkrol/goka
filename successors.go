package astar

// ---------------
//	Buffered Successors
// ---------------

type bufferedSuccessors[T comparable] struct {
	generate SuccessorsFunc[T]
	buf      []T
}

func NewBufferedSuccessors[T comparable](capacity int, generate SuccessorsFunc[T]) *bufferedSuccessors[T] {
	return &bufferedSuccessors[T]{
		generate: generate,
		buf:      make([]T, 0, capacity),
	}
}

func (b *bufferedSuccessors[T]) Successors(current T) []T {
	b.buf = b.generate(current, b.buf[:0])
	return b.buf
}
