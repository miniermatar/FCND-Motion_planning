from enum import Enum
from queue import PriorityQueue
import numpy as np
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point, LineString
from sampling import Sampler
import networkx as nx



def generate_path(start_pos,safety_distance):
    # Read in obstacle map
    print("\tLoading colliders data ...")
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
    sampler = Sampler(data,safety_distance)
    polygons = sampler._polygons
    
    #reading graph
    print("\tReading graph ...")
    g=nx.read_gpickle("graph.gpickle")
    
    
    print("\tFinding goal ...")
    goal_pos_graph=None
    while goal_pos_graph==None:
        goal_rnd=[]
        while len(goal_rnd)==0:
            goal_rnd = sampler.sample(1)
        goal_pos=goal_rnd[0]
        goal_pos_graph=closest_neighbor(g,goal_pos,polygons)

    #Finding closest node to start and goal positions
    print("\tFinding closest node to start position ...")
    start_pos_graph=closest_neighbor_start(g,start_pos,polygons)
    if start_pos_graph==None:
        return [0,0,0,0] #not path found from start location
    
    print('Starting position: {0} \nGoal position {1}'.format(start_pos, goal_rnd))        
    
    #finding collision free path
  
    path, cost = a_star(g, heuristic, start_pos_graph, goal_pos_graph)
    path.append(goal_pos)
    
    #creating waypoints
    waypoints = [[int(p[0]),int(p[1]),int(p[2]), 0] for p in path]
    
    #calculating yaw angle to be sent with waypoint
    for i in range(0,len(waypoints)-1,1):
        waypoints[i+1][3]=np.arctan2((waypoints[i+1][1]-waypoints[i][1]), (waypoints[i+1][0]-waypoints[i][0]))
    
    print(len(waypoints), waypoints)
    
    return waypoints


def can_connect(n1, n2,polygons):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= min(n1[2], n2[2]):
            return False
    return True


def closest_neighbor(graph,point,polygons):
    #ientifying closest point in the graph from start and goal
    closest_neighbor=None
    distance=200
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
    distance=200
    for coordinate in graph.nodes:
        dist=LA.norm(np.array(coordinate) - np.array(point))
        if dist < distance:
            if can_connect_start(coordinate, point, polygons):
                closest_neighbor_start=coordinate
                distance=dist
    return closest_neighbor_start
    
    

    
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

