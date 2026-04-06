"""A file for EECE 5550 HW 3 Problem 1 - Route Planning"""
from typing import Optional, Callable, Any, Self



# ========== Part a: A* ========== #

class Vertex():
    """A vertex"""
    def __init__(self, value: Optional[Any] = None) -> Self:
        self.value = value

class Edge():
    """An edge"""
    def __init__(self, start: Vertex, finish: Vertex, directed: Optional[bool] = False, weight: Optional[Any] = None) -> Self:
        self.start = start
        self.finish = finish
        self.vertices = (start, finish)
        self.directed = directed
        self.weight = weight

class Graph():
    """A Graph"""
    def __init__(self, vertices: Optional[list[Vertex]] = None, edges: Optional[list[Edge]] = None, 
                 weight_fn: Optional[Callable[[Edge], float]] = None, neighbor_fn: Optional[Callable[[Vertex], list[Vertex]]] = None) -> Self:
        """Init a graph"""
        self.V = set(vertices) if vertices is not None else set()
        self.E = set(edges) if edges is not None else set()
        self.weight_fn = weight_fn if weight_fn is not None else lambda x: 1

        if neighbor_fn is not None:
            self.neighbor_fn = neighbor_fn
        else:
            self.neighbor_fn = self.get_neighbors

        
    def get_neighbors(self, vertex: Vertex) -> list[Vertex]:
        """Return a list of all neighbors of the vertex"""
        assert vertex in self.V, f"Vertex {vertex} not in V"

        neighbors = []
        for edge in self.E:
            if vertex in edge.vertices:
                neighbors.append(*edge.vertices.pop(vertex))

        return neighbors


def run_a_star(graph: Graph, start: Vertex, end: Vertex, heuristic: Callable[[Vertex, Vertex], float]):
    """Run the A* Algorithm"""

    costTo = {}
    estTotalCost = {}






if __name__ == "__main__":
    a = Vertex(3)
    b = Vertex(4)

    e = Edge(a,b)

    g = Graph([a,b],[e])
    c = g.weight_fn(e)
    print(c)

