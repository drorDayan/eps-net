from math import sqrt, ceil
from CGALPY.Arr2 import *
from CGALPY.Ker import *
from CGALPY.Pol2 import *
from CGALPY.BSO2 import *
from CGALPY.CH2 import *
from CGALPY.SS import *
from CGALPY.BV import *
from CGALPY.MN2 import *
from CGALPY.PP2 import *
from CGALPY.Tri2 import *


class Config(object):
    __instance = None

    def __new__(cls):
        if Config.__instance is None:
            Config.__instance = object.__new__(cls)
        return Config.__instance

    def __init__(self):
        self.eps = 99999999999
        self.delta = 0.1
        self.is_multi_robot = False
        self.sample_mathod = "eps_net"
        alpha = self.eps / sqrt(1 + self.eps ** 2)
        edge_len = 1 - 2 * self.delta
        if self.is_multi_robot:
            ball_radius = alpha * self.delta
            self.connection_radius = 0.1  # TODO
        else:
            raise ValueError("Not yet inited")
            ball_radius = alpha * self.delta  # TODO
            self.connection_radius = 0.1  # TODO
        beta = ball_radius
        unrounded_balls_per_dim = edge_len / (2 * ball_radius)
        print("unrounded number of balls:", unrounded_balls_per_dim ** 2 + (unrounded_balls_per_dim + 1) ** 2)
        balls_per_dim = ceil(unrounded_balls_per_dim)
        self.num_of_sample_points = balls_per_dim ** 2 + (balls_per_dim + 1) ** 2
        print("real number of balls:", self.num_of_sample_points)

