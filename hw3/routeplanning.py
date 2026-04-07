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



# ========== Part a: A* ========== #



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
        
        # print([str(s) for s in neighbor_fn(a)])
        # print(weight_fn(a,b))
        
        path = run_a_star(V,s,g,neighbor_fn,weight_fn,heuristic_fn)
        
        with open(savepath,"w") as f:
            json.dump(path,f)

    else:
        with open(savepath,"r") as f:
            path = json.load(f)

    # print(f"Path: {path}")


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

    plt.show()

