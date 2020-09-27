from config import *
import time
import random
from sr_neighbor_finder import NeighborsFinder
import queue
import heapq
import math
import numpy as np
import matplotlib.pyplot as plt


class PrmNode:
    def __init__(self, pt, is_sparse=False):
        self.point = pt
        self.connections = {}
        self.bfs_dist_from_t = None
        self.father_in_bfs_dist_from_t = None
        self.real_dist_from_t = None
        self.father_in_dist_from_t = None
        self.srm_counter = Config().srm_drrt_config['sr_add_srm_once_in']
        self.is_sparse = is_sparse
        self.sparse_connections = {}
        self.sparse_rep = None


class PrmGraph:
    def __init__(self):
        self.points_to_nodes = {}

    def add_node(self, p, is_sparse=False):
        if p not in self.points_to_nodes.keys():
            p1_node = PrmNode(p, is_sparse)
            self.points_to_nodes[p] = p1_node

    def insert_edge(self, p1, p2, is_sparse=False):
        if p1 not in self.points_to_nodes.keys():
            p1_node = PrmNode(p1, is_sparse=is_sparse)
            self.points_to_nodes[p1] = p1_node
        else:
            p1_node = self.points_to_nodes[p1]
        if p2 not in self.points_to_nodes.keys():
            p2_node = PrmNode(p2, is_sparse=is_sparse)
            self.points_to_nodes[p2] = p2_node
        else:
            p2_node = self.points_to_nodes[p2]
        if p1_node == p2_node:
            return
        # noinspection PyArgumentList
        dist = math.sqrt(Euclidean_distance().transformed_distance(p1_node.point, p2_node.point).to_double())
        if is_sparse:
            p1_node.sparse_connections[p2_node] = dist
            p2_node.sparse_connections[p1_node] = dist
        else:
            p1_node.connections[p2_node] = dist
            p2_node.connections[p1_node] = dist

    def has_path(self, p1, p2, is_sparse=False):
        if p1 not in self.points_to_nodes.keys() or p2 not in self.points_to_nodes.keys():
            return False
        q = queue.Queue()
        visited = {p1: True}
        q.put(p1)
        while not q.empty():
            curr = q.get()
            if curr == p2:
                return True
            else:
                if is_sparse:
                    keys = self.points_to_nodes[curr].sparse_connections.keys()
                else:
                    keys = self.points_to_nodes[curr].connections.keys()
                for next_n in keys:
                    next_p = next_n.point
                    if next_p not in visited:
                        visited[next_p] = True
                        q.put(next_p)
        return False

    def calc_bfs_dist_from_t(self, t):
        if t not in self.points_to_nodes.keys():
            return False
        self.points_to_nodes[t].bfs_dist_from_t = 0
        temp_i = 0
        q = [(0, temp_i, t)]
        heapq.heapify(q)
        visited = {t: True}
        while len(q) > 0:
            c_dist, _, curr = heapq.heappop(q)
            curr_n = self.points_to_nodes[curr]
            for next_n in curr_n.connections.keys():
                next_p = next_n.point
                if next_p not in visited:
                    next_n.bfs_dist_from_t = c_dist + 1
                    next_n.father_in_bfs_dist_from_t = curr_n
                    visited[next_p] = True
                    temp_i += 1
                    heapq.heappush(q, (next_n.bfs_dist_from_t, temp_i, next_p))
        return True

    def calc_real_dist_from_t(self, t):
        if t not in self.points_to_nodes.keys():
            return False
        self.points_to_nodes[t].real_dist_from_t = 0
        temp_i = 0
        q = [(0, temp_i, t)]
        heapq.heapify(q)
        done = {}
        while len(q) > 0:
            c_dist, _, curr = heapq.heappop(q)
            done[curr] = True
            curr_n = self.points_to_nodes[curr]
            for next_n in curr_n.connections.keys():
                next_p = next_n.point
                if next_p not in done:
                    temp_i += 1
                    alt = c_dist + next_n.connections[curr_n]
                    if next_n.real_dist_from_t is None or alt < next_n.real_dist_from_t:
                        next_n.real_dist_from_t = alt
                        next_n.father_in_dist_from_t = curr_n
                        heapq.heappush(q, (next_n.real_dist_from_t, temp_i, next_p))
        return True


def get_min_max(obstacles):
    max_x = max(max(v.x() for v in obs) for obs in obstacles)
    max_y = max(max(v.y() for v in obs) for obs in obstacles)
    min_x = min(min(v.x() for v in obs) for obs in obstacles)
    min_y = min(min(v.y() for v in obs) for obs in obstacles)
    return max_x.to_double(), max_y.to_double(), min_x.to_double(), min_y.to_double()


def generate_milestones(cd, n, max_x, max_y, min_x, min_y):
    v = []
    if Config().sr_prm_config['use_grid']:
        diff = FT(Config().sr_prm_config['grid_size'])
        Config().sr_prm_config['number_of_neighbors_to_connect'] = 8
        x = FT(min_x)
        while x < FT(max_x):
            y = FT(min_y)
            while y < FT(max_y):
                if cd.is_valid_conf(Point_2(x, y)):
                    v.append(xy_to_2n_d_point(x, y))
                y += diff
            x += diff
    else:
        while len(v) < n:
            x = FT(random.uniform(min_x, max_x))
            y = FT(random.uniform(min_y, max_y))
            if cd.is_valid_conf(Point_2(x, y)):
                v.append(xy_to_2n_d_point(x, y))
    return v


def make_graph(cd, milestones, origin, destination, create_sparse):
    milestones += [origin, destination]
    nn = NeighborsFinder(milestones)
    g = PrmGraph()

    for milestone in milestones:
        g.add_node(milestone)
        # the + 1 to number_of_neighbors is to count for count v as it's neighbor
        nearest = nn.k_nn(milestone, Config().sr_prm_config['number_of_neighbors_to_connect'])
        for neighbor in nearest[1:]:  # first point is self and no need for edge from v to itself
            if cd.path_collision_free(milestone, neighbor):
                g.insert_edge(milestone, neighbor)

    return g


# def print_sr_sparse_graph(g):
#     n_s_v = 0
#     for v in g.points_to_nodes.values():
#         if v.is_sparse:
#             n_s_v += 1
#             plt.plot([v.point[0].to_double()], [v.point[1].to_double()], 'o', color='black')
#             # print("p:", v.point, "con", len(v.sparse_connections))
#             for con in v.sparse_connections.keys():
#                 plt.plot([v.point[0].to_double(), con.point[0].to_double()],
#                          [v.point[1].to_double(), con.point[1].to_double()], color='black')
#         else:
#             if v.sparse_rep is not None:
#                 plt.plot([v.point[0].to_double()], [v.point[1].to_double()], 'o', color='blue')
#                 plt.plot([v.point[0].to_double(), v.sparse_rep[0][0].to_double()],
#                          [v.point[1].to_double(), v.sparse_rep[0][1].to_double()], color='blue')
#     print(n_s_v)
#     # axes = plt.gca()
#     # axes.set_xlim([-1.7, 1.7])
#     # axes.set_ylim([-1.7, 1.7])
#     # plt.plot([-1.3, -1.3, -0.1, -0.1],
#     #          [1, 0.1, 0.1, 1], color='red')
#     # plt.plot([0.1, 0.1, 1.3, 1.3],
#     #          [1, 0.1, 0.1, 1], color='red')
#     # plt.plot([-1.3, -1.3, -0.1, -0.1, -1.3],
#     #          [-0.1, -1.3, -1.3, -0.1, -0.1], color='red')
#     # plt.plot([0.1, 0.1, 1.3, 1.3, 0.1],
#     #          [-0.1, -1.3, -1.3, -0.1, -0.1], color='red')
#
#     plt.savefig("temp/sparse_graph, T" + str(time.time()) + ".png")
#     plt.close()
#     for v in g.points_to_nodes.values():
#         if v.is_sparse:
#             plt.plot([v.point[0].to_double()], [v.point[1].to_double()], 'o', color='black')
#             # print("p:", v.point, "con", len(v.sparse_connections))
#             for con in v.sparse_connections.keys():
#                 plt.plot([v.point[0].to_double(), con.point[0].to_double()],
#                          [v.point[1].to_double(), con.point[1].to_double()], color='black')
#     # axes = plt.gca()
#     # axes.set_xlim([-1.7, 1.7])
#     # axes.set_ylim([-1.7, 1.7])
#     # plt.plot([-1.3, -1.3, -0.1, -0.1],
#     #          [1, 0.1, 0.1, 1], color='red')
#     # plt.plot([0.1, 0.1, 1.3, 1.3],
#     #          [1, 0.1, 0.1, 1], color='red')
#     # plt.plot([-1.3, -1.3, -0.1, -0.1, -1.3],
#     #          [-0.1, -1.3, -1.3, -0.1, -0.1], color='red')
#     # plt.plot([0.1, 0.1, 1.3, 1.3, 0.1],
#     #          [-0.1, -1.3, -1.3, -0.1, -0.1], color='red')
#
#     plt.savefig("temp/sparse_graph, T" + str(time.time()) + ".png")
#     plt.close()

def generate_path(path, robots, obstacles, destinations):
    # start = time.time()
    origin = two_d_point_to_2n_d_point(origin)
    destination = two_d_point_to_2n_d_point(destination)
    max_x, max_y, min_x, min_y = get_min_max(obstacles)
    if not cd.is_valid_conf(origin) or not cd.is_valid_conf(destination):
        print("invalid input")
        return False
    number_of_points_to_find = Config().sr_prm_config['number_of_milestones_to_find']
    milestones = generate_milestones(cd, number_of_points_to_find, max_x, max_y, min_x, min_y)
    g = make_graph(cd, milestones, origin, destination, create_sparse)
    if g.has_path(origin, destination, create_sparse):
        g.calc_bfs_dist_from_t(destination)
        g.calc_real_dist_from_t(destination)
        # print_sr_sparse_graph(g)
        return True, g
    else:
        print("failed to find a valid path in prm")
        return False, PrmGraph()
