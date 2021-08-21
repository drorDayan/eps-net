from config import *
import time
import random
from sr_neighbor_finder import NeighborsFinder
import heapq
import Collision_detection
import CGALPY.Ker as KER
import CGALPY.SS as SS
import gc

Config = Config()


def point_d_to_point_2(x):
    return KER.Point_2(x[0], x[1])


class PrmNode:
    def __init__(self, pt, name):
        self.point = pt
        self.point_2 = point_d_to_point_2(pt)
        self.name = name
        self.out_connections = {}
        self.in_connections = {}
        self.node_conflicts = []
        self.edge_conflicts = []


class PrmEdge:
    def __init__(self, src, dest, eid):
        self.src = src
        self.dest = dest
        self.name = "e"+str(eid)
        self.segment = KER.Segment_2(src.point_2, dest.point_2)
        self.node_conflicts = []
        self.edge_conflicts = set()
        self.cost = sqrt(KER.squared_distance(src.point_2, dest.point_2).to_double())


class PrmGraph:
    def __init__(self, milestones):
        self.edge_id = 0
        self.points_to_nodes = {}
        self.edges = []
        for milestone in milestones:
            if milestone in self.points_to_nodes:
                print("Error, re entry of:", milestone.point, "in", milestone)
            self.points_to_nodes[milestone.point] = milestone

    def insert_edge(self, milestone, neighbor):
        neighbor_node = self.points_to_nodes[neighbor]
        if neighbor_node == milestone:
            return
        if neighbor_node in milestone.out_connections or neighbor_node in milestone.in_connections:
            return
        self.edges.append(PrmEdge(milestone, neighbor_node, self.edge_id))
        milestone.out_connections[neighbor_node] = self.edges[-1]
        neighbor_node.in_connections[milestone] = self.edges[-1]
        self.edge_id += 1

    def calculate_vertices_conflicts(self, nn, robot_radius):
        print("calculate_vertices_conflicts")
        for point, node in self.points_to_nodes.items():
            nearest = nn.neighbors_in_radius(point, KER.FT(2)*robot_radius)
            for neighbor in nearest:  # first point is self and no need for edge from v to itself
                neighbor_node = self.points_to_nodes[neighbor]
                if neighbor_node == node:
                    continue
                node.node_conflicts.append(neighbor_node)
                neighbor_node.node_conflicts.append(node)

    def calculate_edge_to_vertex_conflicts(self, robot_radius, nn):
        print("calculate_edge_to_vertex_conflicts")
        for edge in self.edges:
            sure_s = nn.neighbors_in_radius(edge.src.point, KER.FT(2)*robot_radius)
            sure_d = nn.neighbors_in_radius(edge.dest.point, KER.FT(2)*robot_radius)
            sure_p = set([self.points_to_nodes[point] for point in sure_s+sure_d])

            for node in sure_p:
                node.edge_conflicts.append(edge)
                edge.node_conflicts.append(node)

            x1 = nn.neighbors_in_radius(edge.src.point, KER.FT(2)*robot_radius+Config.connection_radius)
            x2 = nn.neighbors_in_radius(edge.dest.point, KER.FT(2)*robot_radius+Config.connection_radius)
            points = set([self.points_to_nodes[point] for point in x1+x2])
            for node in points-sure_p:
                if KER.squared_distance(point_d_to_point_2(node.point), edge.segment) < KER.FT(4)*robot_radius*robot_radius:
                    node.edge_conflicts.append(edge)
                    edge.node_conflicts.append(node)

    def calculate_edge_to_edge_conflicts(self, robot_radius, nn):  # Dror: this can be improved by looking at edges of "near" vertices
        print("calculate_edge_to_edge_conflicts")
        i = 0
        for edge1 in self.edges:
            if i % 1000 == 0:
                gc.collect()
            print(i)
            i += 1

            sure_s = nn.neighbors_in_radius(edge1.src.point, KER.FT(2)*robot_radius)
            sure_d = nn.neighbors_in_radius(edge1.dest.point, KER.FT(2)*robot_radius)
            sure_p = set([self.points_to_nodes[point] for point in sure_s+sure_d])
            sure_e = set()
            for node in sure_p:
                sure_e.update(list(node.in_connections.values())+list(node.out_connections.values()))

            for edge2 in sure_e:
                if edge1 == edge2 or edge2 in edge1.edge_conflicts:
                    continue
                edge1.edge_conflicts.add(edge2)
                edge2.edge_conflicts.add(edge1)

            maybe_s = nn.neighbors_in_radius(edge1.src.point, KER.FT(2)*robot_radius+KER.FT(2)*Config.connection_radius)
            maybe_d = nn.neighbors_in_radius(edge1.dest.point, KER.FT(2)*robot_radius+KER.FT(2)*Config.connection_radius)
            maybe_p = set([self.points_to_nodes[point] for point in maybe_s+maybe_d])
            maybe_e = set()
            for node in maybe_p:
                maybe_e.update(list(node.in_connections.values())+list(node.out_connections.values()))

            for edge2 in maybe_e-sure_e:
                if edge1 == edge2 or edge2 in edge1.edge_conflicts:
                    continue
                else:
                    if KER.squared_distance(edge1.segment, edge2.segment) < KER.FT(4)*robot_radius*robot_radius:
                        edge1.edge_conflicts.add(edge2)
                        edge2.edge_conflicts.add(edge1)

    def a_star(self, robot_radius, start_points, goal_points):
        goal = tuple([self.points_to_nodes[p] for p in goal_points])

        def h(p):
            res = 0
            for p1, p2 in zip(p, goal):
                res += sqrt(KER.squared_distance(p1.point_2, p2.point_2).to_double())
            return res

        def get_neighbors(points):
            connections_list = [list(p.out_connections.keys())+list(p.in_connections.keys()) for p in points]
            res = []
            for i in range(len(points)):
                is_good = True
                for next_point in connections_list[i]:
                    for j in range(len(points)):
                        if i == j:
                            continue
                        if KER.squared_distance(next_point.point_2, points[j].point_2) < KER.FT(4)*robot_radius*robot_radius:
                            is_good = False
                            break
                    if not is_good:
                        continue
                    seg = points[i].in_connections[next_point] if next_point in points[i].in_connections else points[i].out_connections[next_point]
                    for j in range(len(points)):
                        if i == j:
                            continue
                        if KER.squared_distance(seg.segment, points[j].point_2) < KER.FT(4)*robot_radius*robot_radius:
                            is_good = False
                            break
                    if not is_good:
                        continue
                    res.append((tuple([points[k] if k!=i else next_point for k in range(len(points))]), seg))
            return res

        temp_i = 0
        start = tuple([self.points_to_nodes[p] for p in start_points])

        def get_path(cf):
            c = goal
            path = [[p.point_2 for p in c]]
            while c != start:
                c = cf[c]
                path.append([p.point_2 for p in c])
            path.reverse()
            return path

        q = [(h(start), temp_i, start)]
        heapq.heapify(q)
        came_from = {}
        g_score = {start: 0}
        temp_j = 0
        while len(q) > 0:
            curr_f_score, _, curr = heapq.heappop(q)
            if curr_f_score > (g_score[curr]+h(curr)):
                temp_j += 1
                if temp_j % 100000 == 0:
                    print("temp_j", temp_j)
                    print(len(q), " ", len(came_from))
                continue
            if curr == goal:
                return g_score[curr], get_path(came_from)
            for neighbor, edge in get_neighbors(curr):
                tentative_g_score = g_score[curr] + edge.cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = curr
                    g_score[neighbor] = tentative_g_score
                    temp_i += 1
                    if temp_i % 100000 == 0:
                        print(temp_i)
                        print(len(q), " ", len(came_from))
                        if temp_i % 1000000 == 0:
                            # TODO remove duplications?
                            gc.collect()
                    heapq.heappush(q, (tentative_g_score + h(neighbor), temp_i, neighbor))
        print("error no path found")
        return 99999999999, []


def cords_to_2d_points(cords_x, cords_y):
    res = [SS.Point_d(2, [KER.FT(x), KER.FT(y)]) for x in cords_x for y in cords_y]
    return res


def generate_milestones(cd):
    res = []
    if Config.sample_method == "eps_net":
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

    if Config.sample_method == "grid":
        i = 0
        points_diff = (Config.edge_len / (Config.grid_points_per_dim-1))
        cords = [Config.delta + i * points_diff for i in range(Config.grid_points_per_dim)]
        points = cords_to_2d_points(cords, cords)
        for point in points:
            if cd.is_point_valid(point):
                res.append(PrmNode(point, "v"+str(i)))
                i += 1
        return res

    if Config.sample_method == "random":
        i = 0
        points = [SS.Point_d(2, [KER.FT(random.uniform(Config.delta, 1-Config.delta)), KER.FT(random.uniform(Config.delta, 1-Config.delta))]) for _ in range(Config.num_of_sample_points)]
        for point in points:
            if cd.is_point_valid(point):
                res.append(PrmNode(point, "v"+str(i)))
                i += 1
        return res

    raise ValueError("Invalid configuration")


def make_graph(cd, milestones, nn):
    g = PrmGraph(milestones)
    for milestone in milestones:
        p = milestone.point
        nearest = nn.neighbors_in_radius(p, Config.connection_radius)
        for neighbor in nearest:
            if neighbor == p:
                continue
            edge = KER.Segment_2(KER.Point_2(p[0], p[1]), KER.Point_2(neighbor[0], neighbor[1]))
            if cd.is_edge_valid(edge):
                g.insert_edge(milestone, neighbor)

    return g


def generate_path(path, starts, obstacles, destinations, in_radius):
    start_t = time.time()
    radius = KER.FT(in_radius)
    cd = Collision_detection.Collision_detector(obstacles, radius)
    milestones = []
    for (i, start_p) in enumerate(starts):
        milestones.append(PrmNode(SS.Point_d(2, [start_p.x(), start_p.y()]), "start"+str(i)))
    for (i, destination_p) in enumerate(destinations):
        milestones.append(PrmNode(SS.Point_d(2, [destination_p.x(), destination_p.y()]), "goal"+str(i)))
    milestones += generate_milestones(cd)
    nn = NeighborsFinder([milestone.point for milestone in milestones])
    g = make_graph(cd, milestones, nn)
    print("vertices amount:", len(g.points_to_nodes))
    print("edges amount:", len(g.edges))

    if Config.run_a_star:
        a_star_res, d_path = g.a_star(radius, [SS.Point_d(2, [start_p.x(), start_p.y()]) for start_p in starts],
                                      [SS.Point_d(2, [destination_p.x(), destination_p.y()]) for destination_p in destinations] )
        print("a_star_res:", a_star_res)
    if Config.create_yaml:
        g.calculate_vertices_conflicts(nn, radius)
        g.calculate_edge_to_vertex_conflicts(radius, nn)
        g.calculate_edge_to_edge_conflicts(radius, nn)

        with open(Config.out_file_name, "w") as f:
            f.write("vertices:\n")
            for vertex in g.points_to_nodes.values():
                f.write("  - name: " + vertex.name + "\n")
                f.write("    pos: [" + str(vertex.point[0].to_double()) + ", " + str(vertex.point[1].to_double()) + "]\n")
                f.write("    vertexConflicts: "+str([node.name for node in vertex.node_conflicts])+"\n")
                f.write("    edgeConflicts: "+str([e.name for e in vertex.edge_conflicts])+"\n")
            f.write("edges:\n")
            for edge in g.edges:
                f.write("  - name: " + edge.name + "\n")
                f.write("    from: " + edge.src.name + "\n")
                f.write("    to: " + edge.dest.name + "\n")
                f.write("    edgeConflicts: "+str([e.name for e in edge.edge_conflicts])+"\n")
                f.write("    vertexConflicts: "+str([node.name for node in edge.node_conflicts])+"\n")
            f.flush()
    took = time.time()-start_t
    print("GOODYYY", took)
    if Config.run_a_star:
        for i in d_path:
            path.append(i)
        return a_star_res, took
    return 0
