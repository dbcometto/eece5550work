"""A file for EECE 5550 HW 3 Problem 1 - Route Planning"""
from typing import Optional, Callable, Any, Self
import heapq



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

    def get_other(self, vertex: Vertex) -> Vertex:
        """Return the other end of the edge"""
        if vertex == self.start:
            return self.finish
        elif vertex == self.finish:
            return self.start
        

class Graph():
    """A Graph"""
    def __init__(self, vertices: Optional[list[Vertex]] = None, edges: Optional[list[Edge]] = None, 
                 weight_fn: Optional[Callable[[Vertex, Vertex], float]] = None, neighbor_fn: Optional[Callable[[Vertex], list[Vertex]]] = None) -> Self:
        """Init a graph"""
        self.V = vertices if vertices is not None else []
        self.E = edges # can be None
        self.weight_fn = weight_fn if weight_fn is not None else lambda x, y: 1

        if neighbor_fn is not None:
            self.neighbor_fn = neighbor_fn
        else:
            self.neighbor_fn = self.default_neighbor_fn


    def get_neighbors(self, vertex: Vertex) -> list[Vertex]:
        """Return a list of all neighbors of a vertex"""
        return self.neighbor_fn(vertex)
    

    def get_weight(self, start: Vertex, end: Vertex) -> float:
        """Return a weight given two vertices"""
        if start in self.V and end in self.get_neighbors(start):
            return self.weight_fn(start,end)


    def default_neighbor_fn(self, vertex: Vertex) -> list[Vertex]:
        """Return a list of all neighbors of the vertex"""
        assert vertex in self.V, f"Vertex {vertex} not in V"

        neighbors = []
        for edge in self.E:
            if vertex in edge.vertices:
                neighbors.append(*edge.vertices.pop(vertex))

        return neighbors






def setup_and_run_a_star(vertexSet: list[Vertex], start: Vertex, goal: Vertex, 
                         neighbor_fn: Callable[[Vertex], list[Vertex]], weight_fn: Callable[[Vertex, Vertex], float], 
                         heuristic_fn: Callable[[Vertex, Vertex], float]) -> list[Vertex]:
    """Set up and run the A* Algorithm"""

    graph = Graph(vertexSet, weight_fn=weight_fn, neighbor_fn=neighbor_fn)

    return run_a_star(graph,start, goal, heuristic_fn)



def run_a_star(graph: Graph, start: Vertex, goal: Vertex, heuristic: Callable[[Vertex, Vertex], float]) -> list[Vertex]:
    """Run the A* Algorithm"""
    # Init
    costTo = {v: 1_000_000 for v in graph.V}
    estTotalCost = {v: 1_000_000 for v in graph.V}
    pred = {v: None for v in graph.V}

    # Starting
    costTo[start] = 0
    estTotalCost[start] = heuristic(start, goal)
    Q = heapq.heapify([(heuristic(start, goal), start)])

    # Main Loop
    while len(Q) != 0:
        p,v = heapq.heappop(Q)

        if p <= estTotalCost[v]:
            if v == goal:
                return recoverPath(start, goal, pred)
            
            for n in graph.neighbor_fn(v):
                pvi = costTo[v] + graph.get_weight(v,n)
                if pvi < costTo[n]:
                    pred[n] = v
                    costTo[n] = pvi
                    estTotalCost[n] = pvi + heuristic(n,goal)

                    # heapq doesn't let me remove old entries so doing a "lazy deletion" method of skipping when it shows up

                    heapq.heappush(Q,(estTotalCost[n],n))

    return []



def recoverPath(start: Vertex, goal: Vertex, pred: dict, max_tries = 1000) -> list[Vertex]:
    """Recover the optimal path"""
    path = [goal]
    v = goal
    tries = 0
    while v != start:
        v = pred[v]
        
        if v is None:
            raise RuntimeError("Pred was not complete")
        
        path.append[v]
        tries += 1

        if tries >= max_tries:
            raise RuntimeError("Failed to find path")

    return path





if __name__ == "__main__":
    a = Vertex(3)
    b = Vertex(4)

    e = Edge(a,b)

    g = Graph([a,b],[e])
    c = g.weight_fn(e)
    print(c)

