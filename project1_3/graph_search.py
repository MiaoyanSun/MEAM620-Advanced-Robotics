#from heapq import heappush, heappop  # Recommended.
from math import sqrt

import numpy as np

#from flightsim.world import World
from proj1_3.code.occupancy_map import OccupancyMap # Recommended.
from proj1_3.code.BinaryHeap import BinaryHeap
from proj1_3.code.Node import Node

def trace_back_path(node):
    path = []
    path.append(np.asarray(goal_))
    while node.get_index_coordinates() != start_node.get_index_coordinates():
        path.append(np.asarray(occ_map.index_to_metric_center(node.get_index_coordinates())))
        node = node.parent

    path.append(np.asarray(start_))
    np.array(path).reshape((-1, 3))
    path.reverse()

    return path



def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        path,       xyz position coordinates along the path !!!in meters!!! with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
    """

    # While not required, we have provided an occupancy map you may use or modify.
    global occ_map
    occ_map = OccupancyMap(world, resolution, margin)

    global start_, goal_
    start_ = start
    goal_ = goal

    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))


    # my code here:::
    global start_node, goal_node

    goal_node = Node(goal_index)

    start_node = Node(start_index)
    start_node.g = 0
    if astar:
        start_node.h = sqrt((goal_[0] - start_[0])**2 + (goal_[1] - start_[1])**2 + (goal_[2] - start_[2])**2)
    else:
        start_node.h = 0
    start_node.set_k()
    OPEN = BinaryHeap()
    OPEN_dict = {}  # for searching in constant time

    # # start searching by pushing start to OPEN list
    OPEN.insert(start_node)
    OPEN_dict[start_node.get_index_coordinates()] = 1   # YES this works !
    CLOSED_dict = {}

    while (CLOSED_dict.get(goal_node.get_index_coordinates()) == None and OPEN.size() != 0):
        node = OPEN.poll()
        if node.get_index_coordinates() == goal_node.get_index_coordinates():
            goal_node = node
        OPEN_dict.pop(node.get_index_coordinates())    # delete the node from OPEN_dict accordingly
        CLOSED_dict[node.get_index_coordinates()] = 1



        nodeR, nodeC, nodeS = node.get_index_coordinates()[0], node.get_index_coordinates()[1], node.get_index_coordinates()[2]
        add = [1, 0, -1]
        for i in add:
            for j in add:
                for k in add:
                    if i == j == k == 0:
                        continue
                    else:
                        nei = (nodeR + i, nodeC + j, nodeS + k)
                        if occ_map.is_valid_index(nei) and not occ_map.is_occupied_index(nei): # this is a valid neighbor
                            #node.add_neighbors(nei) # may delete
                            cell = Node(nei)
                            # if cell is neither in OPEN nor in CLOSED
                            if OPEN_dict.get(cell.get_index_coordinates()) == None and CLOSED_dict.get(cell.get_index_coordinates()) == None:
                                cell.g = node.g + np.linalg.norm(np.asarray(node.get_index_coordinates()) - np.asarray(nei))
                                if astar:
                                    cell.h = sqrt((goal_[0] - occ_map.index_to_metric_negative_corner(nei)[0]) ** 2 + (goal_[1] - occ_map.index_to_metric_negative_corner(nei)[1]) ** 2 + (goal_[2] - occ_map.index_to_metric_negative_corner(nei)[2]) ** 2)
                                else:
                                    cell.h = 0
                                cell.set_k()
                                cell.parent = node
                                OPEN.insert(cell)
                                OPEN_dict[cell.get_index_coordinates()] = 1
                            elif OPEN_dict.get(cell.get_index_coordinates()) == 1:
                                currK = cell.k
                                newG = node.g + np.linalg.norm(np.asarray(node.get_index_coordinates()) - np.asarray(nei))
                                newK = newG + cell.h
                                if newK < currK:
                                    cell.g = newG
                                    cell.set_k(newK)
                                    cell.parent = node



    #print("nodes expanded:", len(CLOSED_dict))

    path = np.array
    if CLOSED_dict.get(goal_node.get_index_coordinates()) == 1: # goal is reached
        path = np.array(trace_back_path(goal_node))
        return path
    # else: # no path for this map
    #     print("no path found:C")

    # my code above:::

    return None
