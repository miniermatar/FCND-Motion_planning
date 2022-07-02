from enum import Enum
from queue import PriorityQueue
import numpy as np
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point, LineString
from sampling import Sampler

import sys
import subprocess #as recommended at: https://www.activestate.com/resources/quick-reads/how-to-install-python-packages-using-a-script/

# implement pip as a subprocess:
subprocess.check_call([sys.executable, '-m', 'pip', 'install', '-I', 'networkx==2.1'])

# process output with an API in the subprocess module:
reqs = subprocess.check_output([sys.executable, '-m', 'pip',
'freeze'])
installed_packages = [r.decode().split('==')[0] for r in reqs.split()]

import networkx as nx



def generate_path(start_pos,safety_distance):
    # Read in obstacle map
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
    #north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    #east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    sampler = Sampler(data,safety_distance)
    polygons = sampler._polygons
    goal_rnd=[]
    while len(goal_rnd)==0:
        goal_rnd = sampler.sample(1)
    goal_pos=goal_rnd[0]
    print('Starting position: {0}, Goal position {1}'.format(start_pos, goal_rnd))
    load_graph=True
    if load_graph==True:
        g=nx.read_gpickle("graph.gpickle")
    else:
        nodes = sampler.sample(500)
        print("Numbers of nodes: {}".format(len(nodes)))
        import time
        t0 = time.time()
        g = create_graph(nodes, 10, polygons)
        print('graph took {0} seconds to build'.format(time.time()-t0))
    
    print("Number of edges")
    start_pos_graph=closest_neighbor(g,start_pos,polygons)
   # if can_connect(start_pos_graph, start_pos, polygons):
    #    print ("Start connected")
    goal_pos_graph=closest_neighbor(g,goal_pos,polygons)
 #   if can_connect(goal_pos_graph, goal_pos, polygons):
  #      print ("Goal connected")
    path, cost = a_star(g, heuristic, start_pos_graph, goal_pos_graph)
    path.append(goal_pos)
    
   
    waypoints = [[int(p[0]),int(p[1]),int(p[2]), 0] for p in path]
    for i in range(0,len(waypoints)-1,1):
        waypoints[i+1][3]=np.arctan2((waypoints[i+1][1]-waypoints[i][1]), (waypoints[i+1][0]-waypoints[i][0]))
    #waypoints_simulator = [[int(p[0] - north_min), int(p[1] - east_min), p[2], 0] for p in path]
    print(len(waypoints), waypoints)
    return waypoints


"""python
# Define two waypoints with heading = 0 for both
wp1 = [n1, e1, a1, 0]
wp2 = [n2, e2, a2, 0]
# Set heading of wp2 based on relative position to wp1
wp2[3] = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))
"""


def can_connect(n1, n2,polygons):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= min(n1[2], n2[2]):
            return False
    return True

def create_graph(nodes, k, polygons):
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        
        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue
                
            if can_connect(n1, n2, polygons):
                g.add_edge(n1, n2, weight=1)
    return g

def closest_neighbor(graph,point,polygons):
    #ientifying closest point in the graph from start and goal
    closest_neighbor=None
    distance=100
    for coordinate in graph.nodes:
        dist=LA.norm(np.array(coordinate) - np.array(point))
        if dist < distance:
            if can_connect(coordinate, point, polygons):
                closest_neighbor=coordinate
                distance=dist
    return closest_neighbor
    
    

    
def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    return valid_actions


def a_star(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""
    
    # TODO: complete

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

