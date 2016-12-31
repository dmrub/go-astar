package astar

import (
	"container/heap"
)

// astar is an A* pathfinding implementation.

// Pather is an interface which allows A* searching on arbitrary objects which
// can represent a weighted graph.
type Pather interface {
	// PathNeighbors returns the direct neighboring nodes of this node which
	// can be pathed to.
	PathNeighbors(world interface{}) []Pather
	// PathNeighbourCost calculates the exact movement Cost to neighbor nodes.
	PathNeighborCost(world interface{}, to Pather) float64
	// PathEstimatedCost is a heuristic method for estimating movement costs
	// between non-adjacent nodes.
	PathEstimatedCost(world interface{}, to Pather) float64
}

// node is a wrapper to store A* data for a Pather node.
type node struct {
	pather Pather
	Cost   float64
	Rank   float64
	parent *node
	open   bool
	closed bool
	index  int
}

// nodeMap is a collection of nodes keyed by Pather nodes for quick reference.
type nodeMap map[Pather]*node

type Pathfinder struct {
	nodes nodeMap
	World interface{}
}

// Public constructor for the Pathfinder struct, initializes the nodes
func NewPathfinder() (pf *Pathfinder) {
	return &Pathfinder{
		nodes: make(nodeMap),
		World: nil,
	}
}

// Get returns the Pather object wrapped in a node, instantiating if required.
func (pf *Pathfinder) Get(p Pather) *node {
	n, ok := pf.nodes[p]
	if !ok {
		n = &node{
			pather: p,
		}
		pf.nodes[p] = n
	}
	return n
}

// Path calculates a short path and the distance between the two Pather nodes.
//
// If no path is found, found will be false.
func (pf *Pathfinder) Search(from, to Pather) (path []Pather, distance float64, found bool) {
	nq := &priorityQueue{}
	heap.Init(nq)
	fromNode := pf.Get(from)
	fromNode.open = true
	heap.Push(nq, fromNode)
	for {
		if nq.Len() == 0 {
			// There's no path, return found false.
			return
		}
		current := heap.Pop(nq).(*node)
		current.open = false
		current.closed = true

		if current == pf.Get(to) {
			// Found a path to the goal.
			p := []Pather{}
			curr := current
			for curr != nil {
				p = append(p, curr.pather)
				curr = curr.parent
			}
			return p, current.Cost, true
		}


		for _, neighbor := range current.pather.PathNeighbors(pf.World) {
			cost := current.Cost + current.pather.PathNeighborCost(pf.World, neighbor)
			neighborNode := pf.Get(neighbor)
			if cost < neighborNode.Cost {
				if neighborNode.open {
					heap.Remove(nq, neighborNode.index)
				}
				neighborNode.open = false
				neighborNode.closed = false
			}
			if !neighborNode.open && !neighborNode.closed {
				neighborNode.Cost = cost
				neighborNode.open = true
				neighborNode.Rank = cost + neighbor.PathEstimatedCost(pf.World, to)
				neighborNode.parent = current
				heap.Push(nq, neighborNode)
			}
		}
	}
}
