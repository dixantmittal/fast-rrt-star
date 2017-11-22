import numpy as np
from utils import *

def select_node_to_expand(tree, space_range):
    space_range = np.asarray(space_range)
    random_point = np.random.rand(space_range.shape[1]) * (space_range[1] - space_range[0]) + space_range[0]
    nodes = list(tree.nodes())
    d = cartesian_distance(nodes, random_point)
    return nodes[np.asscalar(np.argmin(d))], random_point


def sample_new_point(m_g, random_point, d_threshold):
    m_g = np.asarray(m_g)
    random_point = np.asarray(random_point)

    # get distance to random point
    d = cartesian_distance(m_g, random_point)
    if d <= d_threshold:
        return tuple(random_point)

    # rescale the point
    m_new = m_g + d_threshold * (random_point - m_g) / d
    return tuple(m_new)
