import math
import yaml
import matplotlib.pyplot as plt

eps = 99999999999
delta = 0.1


def cords_to_2d_points(cords_x, cords_y):
    res = [[x, y] for x in cords_x for y in cords_y]
    return res


def create_fig():
    eps_s = ["50", "25", "20", "15", "10", "5", "3", "2.5", "2", "1", "0.75"]
    res = [7.225211037399447, 7.259783588110346, 7.300494096342531, 7.283739359697042, 7.231285615510996,
           7.191497727661158, 7.200823126702309, 7.188265871974338, 7.098752845485672, 6.784470805676023,
           6.733991402604547]
    annotations = ["1201", "1301", "1405", "1513", "1625", "2245", "3121",
                   "3613", "4325", "9941", "14621"]
    eps_s = [eps_s[i]+"\n"+annotations[i] for i in range(len(eps_s))]
    annotations = ["631", "701", "738", "774", "855", "1169", "1578", "1869", "2180", "4994", "7307"]
    annotations_pos = [(0.01, 0.785), (0.1, 0.91), (0.195, 0.968), (0.3, 0.945), (0.4, 0.86), (0.48, 0.8), (0.56, 0.81),
                       (0.66, 0.79), (0.785, 0.63), (0.87, 0.14), (0.93, 0.003)]

    plt.plot(eps_s, res, marker="o")
    plt.xlabel("$\\varepsilon$, number of points in the staggered grid")
    plt.ylabel("cost($\\widehat{\\Sigma}$)")
    plt.title("Cost of the multi-robot trajectories as a function of $\\varepsilon$")
    for i, txt in enumerate(annotations):
        plt.annotate(txt, xy=(eps_s[i], res[i]), textcoords='axes fraction', xytext=(annotations_pos[i]))

    # plt.show()
    plt.savefig("spiral_res.pdf", bbox_inches='tight', pad_inches=0.2)


if __name__ == "__main__":
    create_fig()
    exit()
    with open("roadmapWithConflicts_swap50.yaml") as file:
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
