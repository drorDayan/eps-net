import eps_prm
import config
import gc

conf = config.Config()


def generate_path(path, starts, obstacles, destinations, in_radius):
    res = []
    eps_s = [50, 25, 20, 15, 10, 5, 3, 2.5, 2, 1, 0.75]  # , 0.5, 0.25]
    # conf.sample_method = "grid"
    for eps in eps_s:  # , 1, 0.25, 0.1, 0.01]:
        conf.eps = eps
        print("eps=", eps)
        # for sample_method in ["eps_net", "grid"]:
        #     conf.sample_method = sample_method
            # calculate the rest of the configuration and run
        conf.reset()
        curr_res = eps_prm.generate_path(path, starts, obstacles, destinations, in_radius)
        res.append(curr_res)
        gc.collect()
        print("eps_s= ", eps_s)
        print("res= ", res)
    print("final eps_s= ", eps_s)
    print("final res= ", res)
