import numpy as np
import networkx as nx


def select_node_to_expand(tree, space_range, use_bias):
    space_range = np.asarray(space_range)
    random_point = np.random.rand(space_range.shape[0]) * space_range
    nodes = list(tree.nodes())
    dist_2 = np.sum((nodes - random_point) ** 2, axis=1)
    return nodes[np.argmin(dist_2)], random_point


def sample_new_point(m_g, random_point, d_threshold):
    m_g = np.asarray(m_g)
    random_point = np.asarray(random_point)
    d = np.sum((m_g - random_point) ** 2)
    m_new = m_g + d_threshold / d * (random_point - m_g)
    return tuple(m_new)


def is_collision_free(m_g, m_new, obstacle_map):
    return True


def dist(m_g, m_new):
    m_g = np.array(m_g)
    m_new = np.array(m_new)
    return np.sum((m_new - m_g) ** 2)


def is_target_reached(m_new, target_region):
    frame, _range = target_region

    frame = np.array(frame)
    m_new = np.array(m_new)

    diff = np.abs(frame - m_new)

    if np.all(diff <= _range):
        return False
    else:
        return True


# input:
#   starting_state: starting state of the robot
#   target_region: target region where you want the robot to reach
#                   it consists of a tuple (coordinate of frame of target state, range for each dimension)
#   motion_model: a function defining the path between two points. e.g. for straight path, f(x1,x2) = ax1+bx2
def apply_vanilla_rrt(space_region, starting_state, target_region, obstacle_map, n_samples=1000, use_bias=False):
    tree = nx.DiGraph()
    tree.add_node(starting_state)

    final_state = None
    d_threshold = 0.1

    for i in range(n_samples):
        # select node to expand
        m_g, random_point = select_node_to_expand(tree, space_region, use_bias)

        # sample a new point
        m_new = sample_new_point(m_g, random_point, d_threshold)

        print(m_new)

        # check if path between(m_g,m_new) defined by motion-model is collision free
        is_free = is_collision_free(m_g, m_new, obstacle_map)

        # if path is free, add new node to tree
        if is_free:
            tree.add_weighted_edges_from([(m_g, m_new, dist(m_g, m_new))])
            if is_target_reached(m_new, target_region):
                final_state = m_new

    return tree, final_state


# test
start = (0, 0)
target = ((10, 10), (1, 1))
apply_vanilla_rrt((20, 20), start, target, None, n_samples=10)
