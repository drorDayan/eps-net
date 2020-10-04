from math import sqrt, ceil
import CGALPY.Ker as KER


class Config(object):
    __instance = None

    def __new__(cls):
        if Config.__instance is None:
            Config.__instance = object.__new__(cls)
        return Config.__instance

    def __init__(self):
        self.eps = 1
        self.delta = 0.05
        self.is_multi_robot = False
        self.sample_mathod = "eps_net"
        alpha = self.eps / sqrt(1 + self.eps ** 2)
        self.edge_len = 1 - 2 * self.delta
        if self.is_multi_robot:
            raise ValueError("Not yet inited")
            self.ball_radius = alpha * self.delta
            self.connection_radius = 0.1  # TODO
        else:
            self.ball_radius = alpha * self.delta  # TODO
            self.connection_radius = KER.FT(0.1)  # TODO
        beta = self.ball_radius
        unrounded_balls_per_dim = self.edge_len / (2 * self.ball_radius)
        print("unrounded number of balls:", unrounded_balls_per_dim ** 2 + (unrounded_balls_per_dim + 1) ** 2)
        self.balls_per_dim = ceil(unrounded_balls_per_dim)
        self.num_of_sample_points = self.balls_per_dim ** 2 + (self.balls_per_dim + 1) ** 2
        print("real number of balls:", self.num_of_sample_points)

