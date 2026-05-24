package astar

const defaultCapacity = 1024
const defaultSuccessorCapacity = 4

type config[T comparable] struct {
	capacity       int
	transitionsCap int
	dictFactory    func(capacity int) nodeDict[T]
}

func newConfig[T comparable]() config[T] {
	return config[T]{
		capacity:       defaultCapacity,
		transitionsCap: defaultSuccessorCapacity,
		dictFactory: func(capacity int) nodeDict[T] {
			return newMapDict[T](capacity)
		},
	}
}
