from collections import deque
from heapq import heappush, heappop
from itertools import count
import networkx as nx
from networkx.algorithms.shortest_paths.weighted import *
from networkx.utils import generate_unique_node

__all__ = ['dijkstra_path',
           'dijkstra_path_length',
           'bidirectional_dijkstra',
           'single_source_dijkstra',
           'single_source_dijkstra_path',
           'single_source_dijkstra_path_length',
           'all_pairs_dijkstra_path',
           'all_pairs_dijkstra_path_length',
           'dijkstra_predecessor_and_distance',
           'bellman_ford',
           'negative_edge_cycle',
           'goldberg_radzik',
           'johnson']

def dijkstra_path(G, source, target, weight='weight'):

    (length, path) = single_source_dijkstra(G, source, target=target,
                                            weight=weight)
    try:
        return 1, path[target]
    except KeyError:
        return 0, []
        # raise nx.NetworkXNoPath(
        #     "node %s not reachable from %s" % (source, target))