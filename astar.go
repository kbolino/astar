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

type Finder[Node comparable] struct {
	g      Graph[Node]
	start  Node
	dest   Node
	d, h   CostFunc[Node]
	closed set[Node]
	pq     priorityQueue[Path[Node]]
}

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

func (f *Finder[Node]) findPath() Path[Node] {
	for f.pq.Len() > 0 {
		p := heap.Pop(&f.pq).(*item[Path[Node]]).value
		n := p.last()
		if f.closed.Contains(n) {
			continue
		}
		if n == f.dest {
			// Path found
			return p
		}
		f.closed.Add(n)

		for nb := range f.g.Neighbours(n) {
			cp := p.cont(nb)
			heap.Push(&f.pq, &item[Path[Node]]{
				value:    cp,
				priority: -(cp.Cost(f.d) + f.h(nb, f.dest)),
			})
		}
	}
	return nil
}

// FindPath finds the least-cost path between start and dest in graph g
// using the cost function d and the cost heuristic function h.
// Returns nil if no path was found.
func FindPath[Node comparable](g Graph[Node], start, dest Node, d, h CostFunc[Node]) Path[Node] {
	return NewFinder(g, start, dest, d, h).findPath()
}
