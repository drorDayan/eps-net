import math
import yaml
import matplotlib.pyplot as plt

eps = 99999999999
delta = 0.1


def cords_to_2d_points(cords_x, cords_y):
    res = [[x, y] for x in cords_x for y in cords_y]
    return res


def create_fig():
    opt_eps_s = ["$\\infty$", "50", "25", "20", "15", "10", "5", "2.5", "2", "1.5", "1", "0.75"]
    opt_my_res = [1.3742365549272302, 1.3694792029817147, 1.373279226253925, 1.372239349313844,
           1.3683743542186004, 1.3721165441830794, 1.3674488423712654, 1.3633612251566565, 1.3666904755831202,
           1.3017236723780092, 1.30034735466921, 1.2972315620259303]
    opt_opt_res = 1.301
    opt_my_res = [a/opt_opt_res for a in opt_my_res]

    # removed 3 which had res=7.200823126702309 and annotation "1578"
    spiral_eps_s = ["50", "25", "20", "15", "10", "5", "2.5", "2", "1.5", "1", "0.75"]
    spiral_my_res = [7.225211037399447, 7.259783588110346, 7.300494096342531, 7.283739359697042, 7.231285615510996,
                     7.191497727661158, 7.188265871974338, 7.098752845485672, 6.814532142977471, 6.784470805676023,
                     6.733991402604547]
    spiral_opt_res = 3.740546513846127 + 3.7799108897216573
    spiral_my_res = [a/spiral_opt_res for a in spiral_my_res]

    multi_eps_s = ["50", "25", "10", "5"]
    multi_my_res = [6.513, 6.55722, 6.5033, 6.37237]
    multi_opt_res = 6.108
    multi_my_res = [a/multi_opt_res for a in multi_my_res]

    # annotations = ["1201", "1301", "1405", "1513", "1625", "2245", "3121",
    #                "3613", "4325", "9941", "14621"]
    # eps_s = [eps_s[i]+"\n"+annotations[i] for i in range(len(eps_s))]
    annotations = ["631", "701", "738", "774", "855", "1169", "1869", "2180", "2971", "4994", "7307"]
    annotations_pos = [(0.1, 0.4), (0.18, 0.42), (0.265, 0.45), (0.355, 0.43), (0.44, 0.4),
                       (0.51, 0.37), (0.6, 0.37), (0.7, 0.31), (0.785, 0.12), (0.87, 0.095), (0.92, 0.003)]

    opt_annotations = ["4905", "5105", "5517", "5729", "6165", "6849", "9389", "15317", "18629", "25317", "41765", "62309"]
    opt_annotations_pos = [(0.01, 0.82), (0.09, 0.8), (0.18, 0.81), (0.26, 0.8), (0.345, 0.8), (0.42, 0.81),
                       (0.51, 0.86), (0.58, 0.78), (0.665, 0.855), (0.785, 0.6), (0.83, 0.53), (0.91, 0.5)]

    multi_annotations = ["1215", "1315", "1639", "2259"]
    multi_annotations_pos = [(0.1, 0.94), (0.18, 0.91), (0.43, 0.93), (0.51, 0.75)]

    plt.plot(opt_eps_s, opt_my_res, "go", label="2-robots: obstacle-free", linestyle="solid")
    plt.plot(spiral_eps_s, spiral_my_res, "mo", label="2-robots: spiral", linestyle="solid")
    plt.plot(multi_eps_s, multi_my_res, "bo", label="7-robots", linestyle="solid")
    plt.xlabel("$\\varepsilon$")
    plt.ylabel("cost($\\widehat{\\Sigma}$)/cost(OPT$_\\delta$)")
    plt.title("Approximation of optimal $\\delta$-clear solution as a function of $\\varepsilon$")
    plt.axhline(y=1, color="r", label="Optimal $\\delta$-clear")
    plt.legend()
    for i, txt in enumerate(annotations):
        plt.annotate(txt, xy=(spiral_eps_s[i], spiral_my_res[i]),
                     textcoords='axes fraction', xytext=(annotations_pos[i]), color="m")

    for i, txt in enumerate(opt_annotations):
        plt.annotate(txt, xy=(opt_eps_s[i], opt_my_res[i]),
                     textcoords='axes fraction', xytext=(opt_annotations_pos[i]), color="g")

    for i, txt in enumerate(multi_annotations):
        plt.annotate(txt, xy=(multi_eps_s[i], multi_my_res[i]),
                     textcoords='axes fraction', xytext=(multi_annotations_pos[i]), color="b")

    # plt.show()
    plt.savefig("exp_res.pdf")#, bbox_inches='tight', pad_inches=0.2)


if __name__ == "__main__":
    create_fig()
    exit()
    with open("warehouse_2_d004_multi_100000_0.04_eps_net.ymal") as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        fruits_list = yaml.load(file, Loader=yaml.FullLoader)
        print("123")

    alpha = eps/math.sqrt(1+eps**2)
    edge_len = 1-2*delta
    ball_radius = alpha*delta
    beta = ball_radius
    unrounded_balls_per_dim = edge_len/(2*ball_radius)
    print("unrounded number of balls:", unrounded_balls_per_dim**2+(unrounded_balls_per_dim+1)**2)
    balls_per_dim = math.ceil(unrounded_balls_per_dim)
    print("real number of balls:", balls_per_dim**2+(balls_per_dim+1)**2)
    # This is like w but normalized to full balls
    half_points_diff = (edge_len/balls_per_dim)/2
    l1_cords = [delta+(2*i-1)*half_points_diff for i in range(1, balls_per_dim + 1)]
    l2_cords = [delta+(2*i)*half_points_diff for i in range(balls_per_dim + 1)]
    l1 = cords_to_2d_points(l1_cords, l1_cords)
    l2 = cords_to_2d_points(l2_cords, l2_cords)
    print("l1:", l1)
    print("l2:", l2)
    print("balls in each layer", len(l1), len(l2))
