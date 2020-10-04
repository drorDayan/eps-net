from config import *
import time
import random
from sr_neighbor_finder import NeighborsFinder
import queue
import heapq
import math
import numpy as np
import matplotlib.pyplot as plt
import Collision_detection
import CGALPY.Ker as KER
import CGALPY.SS as SS

Config = Config()


class PrmNode:
    def __init__(self, pt, name):
        self.point = pt
        self.name = name
        self.out_connections = {}
        self.in_connections = {}
        # self.bfs_dist_from_t = None
        # self.father_in_bfs_dist_from_t = None
        # self.real_dist_from_t = None
        # self.father_in_dist_from_t = None


class PrmEdge:
    def __init__(self, src, dest):
        self.src = src
        self.dest = dest


class PrmGraph:
    def __init__(self, milestones):
        self.points_to_nodes = {}
        self.edges = []
        for milestone in milestones:
            if milestone in self.points_to_nodes:
                print("Error, re entry of:", milestone.point, "in", milestone)
            self.points_to_nodes[milestone.point] = milestone

    def insert_edge(self, milestone, neighbor):
        neighbor_node = self.points_to_nodes[neighbor]
        if neighbor_node in milestone.out_connections or neighbor_node in milestone.in_connections:
            return
        milestone.out_connections[neighbor_node] = True
        neighbor_node.in_connections[milestone] = True
        self.edges.append(PrmEdge(milestone, neighbor))


    # def add_node(self, p, is_sparse=False):
    #     if p not in self.points_to_nodes.keys():
    #         p1_node = PrmNode(p, is_sparse)
    #         self.points_to_nodes[p] = p1_node
    #
    # def insert_edge(self, p1, p2, is_sparse=False):
    #     if p1 not in self.points_to_nodes.keys():
    #         p1_node = PrmNode(p1, is_sparse=is_sparse)
    #         self.points_to_nodes[p1] = p1_node
    #     else:
    #         p1_node = self.points_to_nodes[p1]
    #     if p2 not in self.points_to_nodes.keys():
    #         p2_node = PrmNode(p2, is_sparse=is_sparse)
    #         self.points_to_nodes[p2] = p2_node
    #     else:
    #         p2_node = self.points_to_nodes[p2]
    #     if p1_node == p2_node:
    #         return
    #     # noinspection PyArgumentList
    #     dist = math.sqrt(Euclidean_distance().transformed_distance(p1_node.point, p2_node.point).to_double())
    #     if is_sparse:
    #         p1_node.sparse_connections[p2_node] = dist
    #         p2_node.sparse_connections[p1_node] = dist
    #     else:
    #         p1_node.connections[p2_node] = dist
    #         p2_node.connections[p1_node] = dist
    #
    # def has_path(self, p1, p2, is_sparse=False):
    #     if p1 not in self.points_to_nodes.keys() or p2 not in self.points_to_nodes.keys():
    #         return False
    #     q = queue.Queue()
    #     visited = {p1: True}
    #     q.put(p1)
    #     while not q.empty():
    #         curr = q.get()
    #         if curr == p2:
    #             return True
    #         else:
    #             if is_sparse:
    #                 keys = self.points_to_nodes[curr].sparse_connections.keys()
    #             else:
    #                 keys = self.points_to_nodes[curr].connections.keys()
    #             for next_n in keys:
    #                 next_p = next_n.point
    #                 if next_p not in visited:
    #                     visited[next_p] = True
    #                     q.put(next_p)
    #     return False
    #
    # def calc_bfs_dist_from_t(self, t):
    #     if t not in self.points_to_nodes.keys():
    #         return False
    #     self.points_to_nodes[t].bfs_dist_from_t = 0
    #     temp_i = 0
    #     q = [(0, temp_i, t)]
    #     heapq.heapify(q)
    #     visited = {t: True}
    #     while len(q) > 0:
    #         c_dist, _, curr = heapq.heappop(q)
    #         curr_n = self.points_to_nodes[curr]
    #         for next_n in curr_n.connections.keys():
    #             next_p = next_n.point
    #             if next_p not in visited:
    #                 next_n.bfs_dist_from_t = c_dist + 1
    #                 next_n.father_in_bfs_dist_from_t = curr_n
    #                 visited[next_p] = True
    #                 temp_i += 1
    #                 heapq.heappush(q, (next_n.bfs_dist_from_t, temp_i, next_p))
    #     return True
    #
    # def calc_real_dist_from_t(self, t):
    #     if t not in self.points_to_nodes.keys():
    #         return False
    #     self.points_to_nodes[t].real_dist_from_t = 0
    #     temp_i = 0
    #     q = [(0, temp_i, t)]
    #     heapq.heapify(q)
    #     done = {}
    #     while len(q) > 0:
    #         c_dist, _, curr = heapq.heappop(q)
    #         done[curr] = True
    #         curr_n = self.points_to_nodes[curr]
    #         for next_n in curr_n.connections.keys():
    #             next_p = next_n.point
    #             if next_p not in done:
    #                 temp_i += 1
    #                 alt = c_dist + next_n.connections[curr_n]
    #                 if next_n.real_dist_from_t is None or alt < next_n.real_dist_from_t:
    #                     next_n.real_dist_from_t = alt
    #                     next_n.father_in_dist_from_t = curr_n
    #                     heapq.heappush(q, (next_n.real_dist_from_t, temp_i, next_p))
    #     return True


def cords_to_2d_points(cords_x, cords_y):
    res = [SS.Point_d(2, [KER.FT(x), KER.FT(y)]) for x in cords_x for y in cords_y]
    return res


def generate_milestones(cd):
    res = []
    if Config.sample_mathod == "eps_net":
        i = 0
        half_points_diff = (Config.edge_len / Config.balls_per_dim) / 2
        l1_cords = [Config.delta + (2 * i - 1) * half_points_diff for i in range(1, Config.balls_per_dim + 1)]
        l2_cords = [Config.delta + (2 * i) * half_points_diff for i in range(Config.balls_per_dim + 1)]
        l1 = cords_to_2d_points(l1_cords, l1_cords)
        l2 = cords_to_2d_points(l2_cords, l2_cords)
        all_points = l1+l2
        for point in all_points:
            if cd.is_point_valid(point):
                res.append(PrmNode(point, "v"+str(i)))
                i += 1
    return res

    # v = []
    # if Config().sr_prm_config['use_grid']:
    #     diff = FT(Config().sr_prm_config['grid_size'])
    #     Config().sr_prm_config['number_of_neighbors_to_connect'] = 8
    #     x = FT(min_x)
    #     while x < FT(max_x):
    #         y = FT(min_y)
    #         while y < FT(max_y):
    #             if cd.is_valid_conf(Point_2(x, y)):
    #                 v.append(xy_to_2n_d_point(x, y))
    #             y += diff
    #         x += diff
    # else:
    #     while len(v) < n:
    #         x = FT(random.uniform(min_x, max_x))
    #         y = FT(random.uniform(min_y, max_y))
    #         if cd.is_valid_conf(Point_2(x, y)):
    #             v.append(xy_to_2n_d_point(x, y))
    # return v


# def make_graph(cd, milestones, origin, destination, create_sparse):
#     milestones += [origin, destination]
#     nn = NeighborsFinder(milestones)
#     g = PrmGraph()
#
#     for milestone in milestones:
#         g.add_node(milestone)
#         # the + 1 to number_of_neighbors is to count for count v as it's neighbor
#         nearest = nn.k_nn(milestone, Config().sr_prm_config['number_of_neighbors_to_connect'])
#         for neighbor in nearest[1:]:  # first point is self and no need for edge from v to itself
#             if cd.path_collision_free(milestone, neighbor):
#                 g.insert_edge(milestone, neighbor)
#
#     return g

def make_graph(cd, milestones, nn):
    g = PrmGraph(milestones)
    for milestone in milestones:
        p = milestone.point
        nearest = nn.neighbors_in_radius(p, Config.connection_radius)
        for neighbor in nearest[1:]:  # first point is self and no need for edge from v to itself
            edge = KER.Segment_2(KER.Point_2(p[0], p[1]), KER.Point_2(neighbor[0], neighbor[1]))
            if cd.is_edge_valid(edge):
                g.insert_edge(milestone, neighbor)

    return g


def generate_path(path, starts, obstacles, destinations, radius):
    # start = time.time()
    cd = Collision_detection.Collision_detector(obstacles, KER.FT(radius))
    milestones = []
    for (i, start_p) in enumerate(starts):
        milestones.append(PrmNode(SS.Point_d(2, [start_p.x(), start_p.y()]), "start"+str(i)))
    for (i, destination_p) in enumerate(destinations):
        milestones.append(PrmNode(SS.Point_d(2, [destination_p.x(), destination_p.y()]), "goal"+str(i)))
    milestones += generate_milestones(cd)
    nn = NeighborsFinder([milestone.point for milestone in milestones])
    g = make_graph(cd, milestones, nn)

    # a = Segment_2(Point_2(0.5, 0.5), Point_2(21, 20.5))
    # print("Segment_2:", a, "is valid:", cd.is_edge_valid(a))
    # a = Segment_2(Point_2(0.5, 0.5), Point_2(1, 0.5))
    # print("Segment_2:", a, "is valid:", cd.is_edge_valid(a))
    # p = Point_2(FT(10.1), FT(10.1))
    # print("Point_2:", p, "is valid:", cd.is_point_valid(p))
    # p = Point_2(FT(1.1), FT(1.1))
    # print("Point_2:", p, "is valid:", cd.is_point_valid(p))
    # origin = two_d_point_to_2n_d_point(origin)
    # destination = two_d_point_to_2n_d_point(destination)
    # max_x, max_y, min_x, min_y = get_min_max(obstacles)
    # if not cd.is_valid_conf(origin) or not cd.is_valid_conf(destination):
    #     print("invalid input")
    #     return False
    # number_of_points_to_find = Config().sr_prm_config['number_of_milestones_to_find']
    # milestones = generate_milestones(cd, number_of_points_to_find, max_x, max_y, min_x, min_y)
    # g = make_graph(cd, milestones, origin, destination, create_sparse)
    if g.has_path(origin, destination, create_sparse):
        g.calc_bfs_dist_from_t(destination)
        g.calc_real_dist_from_t(destination)
        # print_sr_sparse_graph(g)
        return True, g
    else:
        print("failed to find a valid path in prm")
        return False, PrmGraph([])
