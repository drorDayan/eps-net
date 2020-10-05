import eps_prm
import config

conf = config.Config()


def generate_path(path, starts, obstacles, destinations, in_radius):
    for eps in [100, 50, 25, 10, 5, 3, 1]:
        conf.eps = eps
        for sample_method in ["eps_net", "grid", "random"]:
            conf.sample_method = sample_method
            # calculate the rest of the configuration and run
            conf.reset()
            eps_prm.generate_path(path, starts, obstacles, destinations, in_radius)
