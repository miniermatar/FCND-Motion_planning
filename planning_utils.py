from enum import Enum
from queue import PriorityQueue
import numpy as np
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point, LineString
from sampling import Sampler
import networkx as nx
from udacidrone.frame_utils import global_to_local
import re
safety_distance=5

def generate_path(start_pos,safety_distance,global_home):
    #Finding closest node to start and goal positions
    print("\tFinding closest node to start position ...")
    start_pos_graph=closest_neighbor_start(g,start_pos,polygons)
    if start_pos_graph==None:
        return [0,0,0,0] #not path found from start location
    print('Starting position: {0} \nGoal position {1}'.format(start_pos, goal_pos_local))

    #finding collision free path
    path, cost = a_star(g, heuristic, start_pos_graph, goal_pos_graph)
    path.append(goal_pos_local)

    #creating waypoints
    waypoints = [[int(p[0]),int(p[1]),int(p[2]), 0] for p in path]

    #calculating yaw angle to be sent with waypoint
    for i in range(0,len(waypoints)-1,1):
        waypoints[i+1][3]=np.arctan2((waypoints[i+1][1]-waypoints[i][1]), (waypoints[i+1][0]-waypoints[i][0]))

    print(len(waypoints), waypoints)
    return waypoints

def within_boundaries (data,coord):

    # minimum and maximum north coordinates11
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
    if north_min<=coord[0]<=north_max:
        if east_min<=coord[1]<=east_max:
            return True
        else:
            return False
    else:
        return False

def can_connect(n1, n2,polygons):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= min(n1[2], n2[2]):
            return False
    return True


def closest_neighbor(graph,point,polygons):
    #ientifying closest point in the graph from goal
    closest_neighbor=None
    distance=500
    for coordinate in graph.nodes:
        dist=LA.norm(np.array(coordinate) - np.array(point))
        if dist < distance:
            if can_connect(coordinate, point, polygons):
                closest_neighbor=coordinate
                distance=dist
    return closest_neighbor

def can_connect_start(n1, n2,polygons):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= n1[2]:
            return False
    return True

def closest_neighbor_start(graph,point,polygons):
    #ientifying closest point in the graph from start and goal
    closest_neighbor_start=None
    distance=300
    for coordinate in graph.nodes:
        dist=LA.norm(np.array(coordinate) - np.array(point))
        if dist < distance:
            if can_connect_start(coordinate, point, polygons):
                closest_neighbor_start=coordinate
                distance=dist
    return closest_neighbor_start

def a_star(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
    branch = {}
    found = False
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    
                    branch[next_node] = (new_cost, current_node)
             
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
            
    return path[::-1], path_cost

def heuristic(n1, n2):
    return LA.norm(np.array(n2) - np.array(n1))

# Read in obstacle map
print("Loading colliders data ...")
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
sampler = Sampler(data,safety_distance)
polygons = sampler._polygons

#reading graph
print("Reading graph ...")
g=nx.read_gpickle("graph.gpickle")

#loading data to determine global home
f_line= open("colliders.csv", "r")
start_loc = re.findall("[-\d]+\.\d+",f_line.readline())
lat=float(start_loc[0])
lon=float(start_loc[1])
f_line.close()
global_home=(lon,lat,0)

print("\tFinding goal ...")
while True:
    goal_lon=np.random.uniform(-122.40107126,-122.39054357)
    goal_lat=np.random.uniform(37.78848764,37.79673471)

    goal_pos=np.array([goal_lon,goal_lat,-5]) #altitude set 5 meters above the ground
    goal_pos_local=global_to_local(goal_pos,global_home)

    if within_boundaries (data,goal_pos_local)==True:
        goal_pos_graph=closest_neighbor(g,goal_pos_local,polygons)
        if closest_neighbor(g,goal_pos_local,polygons)!=None:
            break

print("\tGoal position global: {} \n\tGoal Position Local {}".format(goal_pos,goal_pos_local))


