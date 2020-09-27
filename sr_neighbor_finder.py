from config import *
from CGALPY.Arr2 import *
from CGALPY.Ker import *
from CGALPY.Pol2 import *
from CGALPY.BSO2 import *
from CGALPY.SS import *


# noinspection PyArgumentList
class NeighborsFinder:
    def __init__(self, points):
        self.tree = Kd_tree()
        self.tree.insert(points)

    # def tree_k_nn(self, query, k):
    #     search_nearest = True
    #     sort_neighbors = True
    #     epsilon = FT(0)
    #
    #     search = K_neighbor_search(self.tree, query, k, epsilon, search_nearest, Euclidean_distance(), sort_neighbors)
    #     lst = []
    #     search.k_neighbors(lst)
    #     return [x for x, y in lst]

    def neighbors_in_radius(self, query, r):
        epsilon = FT(0)
        a = Fuzzy_sphere(query, r, epsilon)
        res = []
        self.tree.search(a, res)
        return res
