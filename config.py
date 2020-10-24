from math import sqrt, ceil
import CGALPY.Ker as KER


class Config(object):
    __instance = None

    def __new__(cls):
        if Config.__instance is None:
            Config.__instance = object.__new__(cls)
        return Config.__instance

    def __init__(self,):
        self.eps = 1000
        self.delta = 0.04
        self.is_multi_robot = True
        self.sample_method = "eps_net"
        self.run_a_star = True
        self.create_yaml = False
        self.reset()

    def reset(self):
        self.out_file_name = ("scene5_r006_d004_multi_" if self.is_multi_robot else "single")+str(self.eps)+"_"+str(self.delta)+"_"+self.sample_method+".ymal"
        print("out_file_name:", self.out_file_name)
        alpha = self.eps / sqrt(1 + self.eps ** 2)
        y = self.eps/(2*(2+self.eps))
        self.edge_len = 1 - 2 * self.delta
        if self.is_multi_robot:
            # These may change as we modify bounds in the paper
            self.ball_radius = y * self.delta
            # self.connection_radius = KER.FT(self.delta)*(KER.FT(1+self.eps)/KER.FT(2+self.eps))
            self.connection_radius = KER.FT(self.delta)*(KER.FT(1+self.eps)/KER.FT(2+self.eps))*KER.FT(1.0001)
            # self.connection_radius = KER.FT(self.delta*1.001)
        else:
            self.ball_radius = alpha * self.delta
            self.connection_radius = KER.FT(2*(alpha+sqrt(1-alpha**2))*self.delta)*KER.FT(1.0001)
        unrounded_balls_per_dim = self.edge_len / (2 * self.ball_radius)
        print("unrounded number of balls:", unrounded_balls_per_dim ** 2 + (unrounded_balls_per_dim + 1) ** 2)
        self.balls_per_dim = ceil(unrounded_balls_per_dim)
        self.num_of_sample_points = self.balls_per_dim ** 2 + (self.balls_per_dim + 1) ** 2
        print("real number of balls:", self.num_of_sample_points)
        self.grid_points_per_dim = ceil(sqrt(self.num_of_sample_points))
        print("real number of grid points:", self.grid_points_per_dim**2)

