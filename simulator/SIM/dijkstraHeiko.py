from scipy.sparse import csr_matrix

# run Dijkstra's algorithm, starting at index 0
from scipy.sparse.csgraph import dijkstra

# Dijkstra's algorithm for shortest paths
# David Eppstein, UC Irvine, 4 April 2002

# http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/117228
from priodict import priorityDictionary

def shortestpath(G,start,end):
    # D,P = dijkstra(G,start,end)
    from scipy.sparse import csr_matrix
    Matrix_sparse = _dict_to_csr(G)
    # run Dijkstra's algorithm, starting at index 0
    D, P = dijkstra(Matrix_sparse, indices=start, return_predecessors=True, directed=True)
    Path = []
    while 1:
        Path.append(end)
        if end == start: break
        end = P[end]
    Path.reverse()
    return Path

def _dict_to_csr(term_dict):
    term_dict_v = list(term_dict.itervalues())
    term_dict_k = list(term_dict.iterkeys())
    shape = list(repeat(np.asarray(term_dict_k).max() + 1,2))
    csr = csr_matrix((term_dict_v, zip(*term_dict_k)), shape = shape)
    return csr

# # example, CLR p.528
# G = {'s': {'u':10, 'x':5},
# 	'u': {'v':1, 'x':2},
# 	'v': {'y':4},
# 	'x':{'u':3,'v':9,'y':2},
# 	'y':{'s':7,'v':6}}
#
# print Dijkstra(G,'s')
# print shortestPath(G,'s','v')