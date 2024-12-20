package quadtree

import (
	"fmt"

	"github.com/kjkrol/gokg/pkg/geometry"
)

type ExampleItem struct {
	pos geometry.Vec[int]
}

func (ei *ExampleItem) Vector() geometry.Vec[int] {
	return ei.pos
}

func ExampleQuadTree() {
	// Create a bounded plane and a quadtree
	boundedPlane := geometry.NewBoundedPlane(64, 64)
	qtree := NewQuadTree(boundedPlane)
	defer qtree.Close()

	// Add items to the quadtree
	items := []*ExampleItem{
		{pos: geometry.Vec[int]{X: 32, Y: 32}},
		{pos: geometry.Vec[int]{X: 32, Y: 31}},
		{pos: geometry.Vec[int]{X: 32, Y: 33}},
		{pos: geometry.Vec[int]{X: 31, Y: 32}},
		{pos: geometry.Vec[int]{X: 33, Y: 32}},
	}
	for _, item := range items {
		qtree.Add(item)
	}

	// Find neighbors of a target item
	target := &ExampleItem{pos: geometry.Vec[int]{X: 32, Y: 32}}
	neighbors := qtree.FindNeighbors(target, 1)

	// Print the neighbors
	for _, neighbor := range neighbors {
		fmt.Println(neighbor.Vector())
	}

	// Output:
	// (32,31)
	// (31,32)
	// (33,32)
	// (32,33)
}
