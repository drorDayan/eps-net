import math


eps = 99999999999
delta = 0.1


def cords_to_2d_points(cords_x, cords_y):
    res = [[x, y] for x in cords_x for y in cords_y]
    return res


if __name__ == "__main__":
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
