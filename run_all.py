import eps_prm
import config
import gc

conf = config.Config()


def generate_path(path, starts, obstacles, destinations, in_radius):
    # eps_s = [9999999, 50, 25, 20, 15, 10, 5, 2.5, 2, 1.5, 1, 0.75]  # , 0.5, 0.25]
    #eps_s = [9999999, 25, 10, 5, 2.5, 1.5, 1]
    eps_s = [9999999, 25, 10, 5, 3, 2.7, 2.5, 2, 1.9, 1.8, 1.7, 1.5, 1.3, 1.2, 1]
    # eps_s = [9999999, 25, 10, 5, 2.5, 2, 1.5, 1]
    conf.delta = 0.02
    # conf.sample_method = "grid"
    sampling_methods = ["eps_net"]#, "random"]
    for sample_method in sampling_methods:
        num_of_runs = 3  # default value
        conf.sample_method = sample_method
        if conf.sample_method == "random":
            num_of_runs = 10
        for _ in range(num_of_runs):
            res = []
            times = []
            for eps in eps_s:  # , 1, 0.25, 0.1, 0.01]:
                conf.eps = eps
                print("eps=", eps)
                # for sample_method in ["eps_net", "grid"]:
                #     conf.sample_method = sample_method
                    # calculate the rest of the configuration and run
                conf.reset()
                curr_res, curr_time = eps_prm.generate_path(path, starts, obstacles, destinations, in_radius)
                res.append(curr_res)
                times.append(curr_time)
                gc.collect()
                print(sample_method, "eps_s= ", eps_s)
                print(sample_method, "res= ", res)
                print(sample_method, "times= ", times)
            print(sample_method, "final eps_s= ", eps_s)
            print(sample_method, "final res= ", res)
            print(sample_method, "final times= ", times)
