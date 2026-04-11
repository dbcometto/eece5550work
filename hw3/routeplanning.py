"""A file for EECE 5550 HW 3 Problem 1 - Route Planning"""
from typing import Optional, Callable, Any, Self
import heapq
import numpy as np
import math
from PIL import Image
from enum import IntEnum
from tqdm import tqdm
import matplotlib.pyplot as plt
import json

import networkx as nx



# ========== Part a-b: A* ========== #

def run_a_star(vertices: list[tuple], start: tuple, goal: tuple, get_neighbors: Callable[[tuple], list[tuple]], get_weight: Callable[[tuple, tuple], float], get_heuristic: Callable[[tuple, tuple], float]) -> list[tuple]:
    """Run the A* Algorithm"""
    # Init
    costTo = {v: 1_000_000 for v in vertices}
    estTotalCost = {v: 1_000_000 for v in vertices}
    pred = {v: None for v in vertices}

    # Starting
    costTo[start] = 0
    estTotalCost[start] = get_heuristic(start, goal)
    Q = [(get_heuristic(start, goal), start)]
    heapq.heapify(Q)

    # Main Loop
    with tqdm(desc="Running A*") as pbar:
        while len(Q) != 0:
            p,v = heapq.heappop(Q)

            if p > estTotalCost[v]: # skip stale entries (instead of resorting Q)
                continue

            if v == goal:
                return recoverPath(start, goal, pred)
            
            for n in get_neighbors(v):
                pvi = costTo[v] + get_weight(v,n)
                if pvi < costTo[n]:
                    pred[n] = v
                    costTo[n] = pvi
                    estTotalCost[n] = pvi + get_heuristic(n,goal)

                    # for i in Q: # Find it in the queue, update priority, and re-sort (slower than "lazy deletion" by ignoring old values?)
                    #     if i[1] == i:
                    #         i[0] == pvi
                    #         heapq.heapify(Q)
                    #         break

                    heapq.heappush(Q,(estTotalCost[n],n))

            pbar.set_postfix({
                "Q_size": len(Q),
                "Recent": v,
                # "HeurStart": get_heuristic(s,v),
                # "HeurGoal": get_heuristic(v,g),
            })
            pbar.update(1)

    return []



def recoverPath(start: tuple, goal: tuple, pred: dict, max_tries = 1000) -> list[tuple]:
    """Recover the found path from goal to start"""
    path = [goal]
    v = goal
    tries = 0
    while v != start:
        v = pred[v]
        
        if v is None:
            raise RuntimeError("Pred was not complete")
        
        path.append(v)
        tries += 1

        if tries >= max_tries:
            raise RuntimeError("Failed to find path")

    return path














# ========== Part c: PRM ========== #

def sample_voxel(map: np.ndarray, nprandom = None, maxiter = 100) -> tuple[int]:
    """Randomy sample a clear voxel from the map"""
    rows, cols = map.shape
    nprandom = nprandom if nprandom is not None else np.random.default_rng()

    chosen = False
    k = 0
    while not chosen and k < maxiter:
        v = nprandom.integers(low=[0,0],high=[rows,cols],size=(2,),dtype=int)
        v = tuple([int(x) for x in v])
        if map[v] == GRID.CLEAR:
            chosen = True
        k += 1

    return v

def generate_line_path(start: tuple[int], end: tuple[int], roffset = 0.5, coffset = 0.5) -> list[tuple[int]]:
    """Use Grid Traversal to return tiles connecting two locations in a straight line"""
    r0,c0 = start
    r1,c1 = end

    dr = r1-r0
    dc = c1-c0

    stepr = 1 if dr>0 else -1 if dr<0 else math.inf
    stepc = 1 if dc>0 else -1 if dc<0 else math.inf

    dtr = stepr/dr if dr != 0 else math.inf
    dtc = stepc/dc if dc != 0 else math.inf

    nextrgrid = stepr if dr>0 else 0
    nextcgrid = stepc if dc>0 else 0

    tOfNextR = abs(nextrgrid-roffset)*dtr
    tOfNextC = abs(nextcgrid-coffset)*dtc

    v = start
    hits =[v]
    while v != end:
        # print(v)
        if tOfNextR <= tOfNextC:
            v = (v[0] + stepr, v[1])
            tOfNextR += dtr
        else:
            v = (v[0], v[1] + stepc)
            tOfNextC += dtc
        hits.append(v)

        if min(tOfNextR,tOfNextC) > 1: # catch any errors
            break

    return hits


def check_line_path(map: np.ndarray, path: list[tuple[int]]) -> bool:
    """Determine if all cells connecting the straight line path are clear"""
    clear = True
    for cell in path:
        if map[cell] != GRID.CLEAR:
            clear = False
            break

    return clear

def calc_distance(u: tuple[int], v: tuple[int]) -> float:
    """Calculate the euclidian distance between two indices"""
    vr, vc = v
    ur, uc = u
    return ((ur-vr)**2+(uc-vc)**2)**0.5


def add_to_prm(graph: nx.Graph, map: np.ndarray, new_v: tuple[int], maxdist: Optional[float]=75) -> None:
    """Add a node to the PRM"""
    graph.add_node(new_v)

    for v in graph.nodes():
        if v != new_v:
            d = calc_distance(new_v,v)
            if d < maxdist:
                if check_line_path(map,generate_line_path(new_v,v)):
                    graph.add_edge(v, new_v, weight=d)

def make_prm(map: np.ndarray, size: int, seed: Optional[int]=None, maxdist: Optional[float]=75) -> nx.graph:
    """Create the PRM"""
    rng = np.random.default_rng(seed)
    graph = nx.Graph()

    for k in tqdm(range(size),desc="Making PRM"):
        new_v = sample_voxel(map, rng)
        add_to_prm(graph, map, new_v, maxdist=maxdist)

    return graph
        



def use_prm(graph: nx.Graph, map: np.ndarray, start: tuple[int], goal: tuple[int], heuristic_fn: Callable[[tuple[int],tuple[int]],float], maxdist: Optional[float]=75) -> list[tuple[int]]:
    """Use the PRM for routeplanning"""

    add_to_prm(graph, map, start, maxdist=maxdist)
    add_to_prm(graph, map, goal, maxdist=maxdist)

    path = nx.astar_path(graph,start,goal,heuristic=heuristic_fn,weight="weight")

    return path








class GRID(IntEnum):
    CLEAR = 1
    OCCUPIED = 0



























if __name__ == "__main__":

    # ========== Route Planning ========== #

    regenerate_path = False
    savepath = "C:\\workspace\\eece5550work\\hw3\\path.json"

    with Image.open('occupancy_map.png') as image:
        occupancy_map = (np.asarray(image) > 0).astype(int)

    numrows, numcols = occupancy_map.shape

    s = (635, 140)
    g = (350, 400)
    


    if regenerate_path:
        
        # Make vertex set and neighbor map
        V = []
        neighbors = {}
        for r in range(numrows):
            for c in range(numcols):
                if occupancy_map[r,c] == GRID.CLEAR:
                    v = (r,c)
                    V.append(v)

                    n = []
                    for rp in range(r-1,r+2):
                        for cp in range(c-1,c+2):
                            if 0 <= rp < numrows and 0 <= cp < numcols:
                                if occupancy_map[rp,cp] == GRID.CLEAR:
                                    n.append((rp,cp))

                    neighbors[v] = n


        def neighbor_fn(v):
            return neighbors[v]

        def weight_fn(v, u):
            vr, vc = v
            ur, uc = u
            return ((ur-vr)**2+(uc-vc)**2)**0.5
        
        def heuristic_fn(v,u):
            return weight_fn(v,u)
        
        path = run_a_star(V,s,g,neighbor_fn,weight_fn,heuristic_fn)
        
        with open(savepath,"w") as f:
            json.dump(path,f)

    else:
        with open(savepath,"r") as f:
            path = json.load(f)


    # ===== Plot the image ===== #

    pathr = [p[0] for p in path]
    pathc = [p[1] for p in path]
    # planned_map[s] = 2


    fig,ax = plt.subplots(1,1,figsize=(8,8))

    ax.imshow(occupancy_map,cmap="gray")
    ax.plot(pathc,pathr,color="red",label="Path")
    ax.scatter(s[1],s[0],color="green",label="Start",marker="+")
    ax.scatter(g[1],g[0],color="blue",label="Goal",marker="x")

    ax.set_title("Planned Path from A*")
    ax.set_xlabel("Column")
    ax.set_ylabel("Row")
    ax.legend(bbox_to_anchor=(1.18,1))

    # plt.show()







    # ========== PRM ========== #

    N = 2500
    s = (635, 140)
    g = (350, 400)

    regenerate_path = False
    savepath = "C:\\workspace\\eece5550work\\hw3\\prm_path_2500.json"

    with Image.open('occupancy_map.png') as image:
        occupancy_map = (np.asarray(image) > 0).astype(int)

    if regenerate_path:
        prm = make_prm(occupancy_map,N,2025)

        def heuristic_fn(v, u):
            vr, vc = v
            ur, uc = u
            return ((ur-vr)**2+(uc-vc)**2)**0.5

        path = use_prm(prm,occupancy_map,s,g,heuristic_fn)

        nodes = list(prm.nodes())
        edges = list(prm.edges())

        data = {
            "nodes": nodes,
            "edges": edges,
            "path": path
        }

        with open(savepath,"w") as f:
            json.dump(data,f)

    else:
        print("loading")
        with open(savepath,"r") as f:
            data = json.load(f)

            nodes = data["nodes"]
            edges = data["edges"]
            path = data["path"]


    # ===== Plot the image ===== #

    pathr = [p[0] for p in path]
    pathc = [p[1] for p in path]
    # planned_map[s] = 2

    print("saving path only")
    with open("C:\\workspace\\eece5550work\\hw3\\prm_only_path_2500.json","w") as f:
        json.dump(path,f)


    fig,ax = plt.subplots(1,1,figsize=(8,8))

    ax.imshow(occupancy_map,cmap="gray")
    ax.plot(pathc,pathr,color="red",marker='o',label="Path")
    ax.scatter(s[1],s[0],color="lime",label="Start",marker="+",s=100,zorder=10)
    ax.scatter(g[1],g[0],color="blue",label="Goal",marker="x",s=100,zorder=10)

    ax.set_title("Planned Path from A*/PRM")
    ax.set_xlabel("Column")
    ax.set_ylabel("Row")
    ax.legend(bbox_to_anchor=(1.18,1))


    fig2,ax2 = plt.subplots(1,1,figsize=(8,8))
    noder = [p[0] for p in nodes]
    nodec = [p[1] for p in nodes]

    ax2.imshow(occupancy_map,cmap="gray",zorder=-2)

    k = 0
    for e in edges:
        if k == 0:
            ax2.plot([e[0][1],e[1][1]],[e[0][0],e[1][0]],color="gray", label="edges", linewidth=0.1,zorder=-1)
            k = 1  
        else:
            ax2.plot([e[0][1],e[1][1]],[e[0][0],e[1][0]],color="gray",linewidth=0.1, zorder=-1)

    ax2.plot(pathc,pathr,color="red",label="Path")
    ax2.scatter(nodec,noder,color="orange",marker='o',facecolor='none',label="Nodes")
    ax2.scatter(s[1],s[0],color="lime",label="Start",marker="+",s=100,zorder=10)
    ax2.scatter(g[1],g[0],color="blue",label="Goal",marker="x",s=100,zorder=10)

    ax2.set_title("Planned Path from A*/PRM")
    ax2.set_xlabel("Column")
    ax2.set_ylabel("Row")
    ax2.legend(bbox_to_anchor=(1.18,1))

    plt.show()