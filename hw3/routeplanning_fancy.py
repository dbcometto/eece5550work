"""A file for EECE 5550 HW 3 Problem 1 - Route Planning"""
from typing import Optional, Callable, Any, Self
import heapq
import numpy as np
import math
from PIL import Image
from enum import IntEnum
from tqdm import tqdm



# ========== Part a: A* ========== #

class Vertex():
    """A vertex"""
    def __init__(self, id: Optional[Any] = None, value: Optional[Any] = None) -> Self:
        self.id = id
        self.value = value

    def __str__(self):
        """Make a string"""
        return str(self.id)
    
    def __eq__(self, other):
        """Determine if a vertex equals another"""
        if isinstance(other, Vertex):
            return self.id == other.id
        elif isinstance(other, type(self.id)):
            return self.id == other
        else:
            raise NotImplemented
        
    def __lt__(self, other):
        """Less than"""
        if isinstance(other, Vertex):
            return self.id < other.id
        elif isinstance(other, type(self.id)):
            return self.id < other
        else:
            raise NotImplemented

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
    costTo = {v.id: 1_000_000 for v in graph.V}
    estTotalCost = {v.id: 1_000_000 for v in graph.V}
    pred = {v.id: None for v in graph.V}

    # Starting
    costTo[start.id] = 0
    estTotalCost[start.id] = heuristic(start, goal)
    Q = [(heuristic(start, goal), start)]
    heapq.heapify(Q)

    # Main Loop
    with tqdm(desc="Running A*") as pbar:
        while len(Q) != 0:
            p,v = heapq.heappop(Q)

            # if p <= estTotalCost[v.id]:
            if v == goal:
                return recoverPath(start, goal, pred)
            
            for n in graph.neighbor_fn(v):
                pvi = costTo[v.id] + graph.get_weight(v,n)
                if pvi < costTo[n.id]:
                    pred[n.id] = v
                    costTo[n.id] = pvi
                    estTotalCost[n.id] = pvi + heuristic(n,goal)

                    for i in Q: # Find it in the queue, update priority, and re-sort (slower than "lazy deletion" by ignoring old values?)
                        if i[1] == i:
                            i[0] == pvi
                            heapq.heapify(Q)
                            break

                    heapq.heappush(Q,(estTotalCost[n.id],n))

            pbar.set_postfix({
                "Q_size": len(Q),
                "Recent": v.id,
                "HeurStart": heuristic(s,v),
                "HeurGoal": heuristic(v,g),
            })
            pbar.update(1)

    return []



def recoverPath(start: Vertex, goal: Vertex, pred: dict, max_tries = 1000) -> list[Vertex]:
    """Recover the optimal path"""
    path = [goal]
    v = goal
    tries = 0
    while v != start:
        v = pred[v.id]
        
        if v is None:
            raise RuntimeError("Pred was not complete")
        
        path.append(v)
        tries += 1

        if tries >= max_tries:
            raise RuntimeError("Failed to find path")

    return reversed(path)

class GRID(IntEnum):
    CLEAR = 1
    OCCUPIED = 0



if __name__ == "__main__":

    # ========== Easy Test ========== #

    # a = Vertex((0,0))
    # b = Vertex((1,0))
    # c = Vertex((0,5))
    # d = Vertex((2,2))

    # V = [a,b,c,d]

    # neighbors = {
    #     a: [b,c],
    #     b: [a,d],
    #     c: [a,d],
    #     d: [b,c]
    # }
    # def neighbor_fn(v):
    #     return neighbors[v]

    # def weight_fn(v, u):
    #     return np.linalg.norm(np.array(v.id)-np.array(u.id))
    
    # def heuristic_fn(v,u):
    #     return weight_fn(v,u)
    
    # # print([str(s) for s in neighbor_fn(a)])
    # # print(weight_fn(a,b))

    # path = setup_and_run_a_star(V,a,d,neighbor_fn,weight_fn,heuristic_fn)

    # print(f"Path: {[str(s) for s in path]}")




    # ========== Route Planning ========== #

    with Image.open('occupancy_map.png') as image:
        occupancy_map = (np.asarray(image) > 0).astype(int)

    numrows, numcols = occupancy_map.shape

    # Make vertex set and neighbor map
    V = []
    neighbors = {}
    for r in range(numrows):
        for c in range(numcols):
            if occupancy_map[r,c] == GRID.CLEAR:
                v = Vertex((r,c))
                V.append(v)

                n = []
                for rp in range(r-1,r+2):
                    for cp in range(c-1,c+2):
                        if 0 <= rp < numrows and 0 <= cp < numcols:
                            if occupancy_map[rp,cp] == GRID.CLEAR:
                                n.append(Vertex((rp,cp)))

                neighbors[v.id] = n


    def neighbor_fn(v):
        return neighbors[v.id]

    def weight_fn(v, u):
        vr, vc = v.id
        ur, uc = u.id
        return ((ur-vr)**2+(uc-vc)**2)**0.5
    
    def heuristic_fn(v,u):
        return weight_fn(v,u)
    
    # print([str(s) for s in neighbor_fn(a)])
    # print(weight_fn(a,b))
    s = Vertex((635, 140))
    g = Vertex((350, 400))
    path = setup_and_run_a_star(V,s,g,neighbor_fn,weight_fn,heuristic_fn)

    print(f"Path: {[str(s) for s in path]}")

