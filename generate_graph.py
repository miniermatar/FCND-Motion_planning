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



safety_distance=5

data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
sampler = Sampler(data,safety_distance)
polygons = sampler._polygons
nodes = sampler.sample(1000)
print("Numbers of nodes: {}".format(len(nodes)))
print("Generating graph ...")
import time
t0 = time.time()
g = create_graph(nodes, 10, polygons)
print('graph took {0} seconds to build'.format(time.time()-t0))
print("Number of edges", len(g.edges))  
nx.write_gpickle(g, "graph.gpickle")
    

