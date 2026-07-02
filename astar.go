// Copyright 2013 Frederik Zipp. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Package astar implements the A* search algorithm for finding least-cost paths.
package astar

import (
	"container/heap"
	"iter"
)

// The Graph interface is the minimal interface a graph data structure
// must satisfy to be suitable for the A* algorithm.
type Graph[Node any] interface {
	// Neighbours returns the neighbour nodes of node n in the graph.
	Neighbours(n Node) iter.Seq[Node]
}

// A CostFunc is a function that returns a cost for the transition
// from node a to node b.
type CostFunc[Node any] func(a, b Node) float64

// A Path is a sequence of nodes in a graph.
type Path[Node any] []Node

// newPath creates a new path with one start node. More nodes can be
// added with append().
func newPath[Node any](start Node) Path[Node] {
	return []Node{start}
}

// last returns the last node of path p. It is not removed from the path.
func (p Path[Node]) last() Node {
	return p[len(p)-1]
}

// cont creates a new path, which is a continuation of path p with the
// additional node n.
func (p Path[Node]) cont(n Node) Path[Node] {
	cp := make([]Node, len(p), len(p)+1)
	copy(cp, p)
	cp = append(cp, n)
	return cp
}

// Cost calculates the total cost of path p by applying the cost function d
// to all path segments and returning the sum.
func (p Path[Node]) Cost(d CostFunc[Node]) (c float64) {
	for i := 1; i < len(p); i++ {
		c += d(p[i-1], p[i])
	}
	return c
}

// Finder uses the A-star algorithm to iteratively find the lowest-cost path.
//
// If you want to find the path in a single go, see the FindPath function.
type Finder[Node comparable] struct {
	g      Graph[Node]
	d, h   CostFunc[Node]
	closed set[Node]
	pq     priorityQueue[Path[Node]]
	start  Node
	dest   Node
}

// NewFinder creates a new Finder to find the best path from start to dest
// in g, using the cost function d and cost heuristic function h.
func NewFinder[Node comparable](g Graph[Node], start, dest Node, d, h CostFunc[Node]) *Finder[Node] {
	f := &Finder[Node]{
		g:      g,
		start:  start,
		dest:   dest,
		d:      d,
		h:      h,
		closed: set[Node]{},
		pq:     priorityQueue[Path[Node]]{},
	}
	heap.Init(&f.pq)
	heap.Push(&f.pq, &item[Path[Node]]{value: newPath(start)})
	return f
}

// Iterate advances the search for the path.
//
// If done==false, then more work is needed, so path should be ignored
// and Iterate should be called again.
// If done==true, then the process is complete, and Iterate should not
// be called again.
// Once that happens, path!=nil contains the desired lowest-cost path
// while path==nil means no such path exists.
func (f *Finder[Node]) Iterate() (path Path[Node], done bool) {
	if f.pq.Len() == 0 {
		// no path exists anymore
		return nil, true
	}
	p := heap.Pop(&f.pq).(*item[Path[Node]]).value
	n := p.last()
	if f.closed.Contains(n) {
		return nil, false
	}
	if n == f.dest {
		// Path found
		return p, true
	}
	f.closed.Add(n)

	for nb := range f.g.Neighbours(n) {
		cp := p.cont(nb)
		heap.Push(&f.pq, &item[Path[Node]]{
			value:    cp,
			priority: -(cp.Cost(f.d) + f.h(nb, f.dest)),
		})
	}
	return nil, false
}

func (f *Finder[Node]) findPath() Path[Node] {
	for {
		if p, done := f.Iterate(); done {
			return p
		}
	}
}

// FindPath finds the least-cost path between start and dest in graph g
// using the cost function d and the cost heuristic function h.
// Returns nil if no path was found.
//
// If you want to spread the work of finding a path over multiple iterations,
// see the Finder type.
func FindPath[Node comparable](g Graph[Node], start, dest Node, d, h CostFunc[Node]) Path[Node] {
	return NewFinder(g, start, dest, d, h).findPath()
}
