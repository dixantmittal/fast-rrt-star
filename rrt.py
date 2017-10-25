import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def select_node_to_expand(tree, space_range, use_bias):
    space_range = np.asarray(space_range)
    random_point = np.random.rand(space_range.shape[1]) * (space_range[1] - space_range[0]) + space_range[0]
    nodes = list(tree.nodes())
    d = dist(nodes, random_point)
    return nodes[np.asscalar(np.argmin(d))], random_point


def sample_new_point(m_g, random_point, d_threshold):
    m_g = np.asarray(m_g)
    random_point = np.asarray(random_point)
    d = dist(m_g, random_point)
    m_new = m_g + d_threshold * (random_point - m_g) / d
    return tuple(m_new)


def is_obstacle_space(m_new, obstacle_map):
    if obstacle_map is None:
        return False

    for key in obstacle_map.keys():
        if lies_in_area(m_new, obstacle_map[key]):
            return True
    return False


def is_collision_free(m_g, m_new, obstacle_map, granularity):
    if dist(m_g, m_new) <= granularity:
        return True

    if is_obstacle_space(m_new, obstacle_map):
        return False

    return is_collision_free(m_g, tuple((np.asarray(m_g) + np.asarray(m_new)) / 2), obstacle_map, granularity) \
           and is_collision_free(tuple((np.asarray(m_g) + np.asarray(m_new)) / 2), m_new, obstacle_map, granularity)


def dist(m_g, m_new):
    m_g = np.array(m_g)
    m_new = np.array(m_new)

    if m_g.ndim == 1:
        m_g = m_g.reshape(1, -1)

    if m_new.ndim == 1:
        m_new = m_new.reshape(1, -1)

    d = np.power(np.sum((m_new - m_g) ** 2, axis=1), 0.5)
    return d


def lies_in_area(m_new, area):
    frame, _range = area
    frame = np.array(frame)
    m_new = np.array(m_new)

    diff = m_new - frame

    return np.all(diff <= _range) and np.all(diff >= 0)


def apply_vanilla_rrt(space_region, starting_state, target_region, obstacle_map, granularity=0.1, d_threshold=0.5,
                      n_samples=1000, use_bias=False):
    tree = nx.DiGraph()
    tree.add_node(starting_state)

    final_state = None

    for i in range(n_samples):
        # select node to expand
        m_g, random_point = select_node_to_expand(tree, space_region, use_bias)

        # sample a new point
        m_new = sample_new_point(m_g, random_point, d_threshold)

        # check if m_new lies in space_region
        if not lies_in_area(m_new, space_region):
            continue

        # check if path between(m_g,m_new) defined by motion-model is collision free
        is_free = is_collision_free(m_g, m_new, obstacle_map, granularity)

        # if path is free, add new node to tree
        if is_free:
            tree.add_weighted_edges_from([(m_g, m_new, dist(m_g, m_new))])
            if lies_in_area(m_new, target_region):
                print('Target reached at i:', i)
                final_state = m_new
                break

    if final_state is None:
        print("Target not reached.")
    return tree, final_state


# test
start = (0, 0)
target = ((50, 50), (5, 5))
tree, final_state = apply_vanilla_rrt(((0, 0), (100, 100)), start, target, {}, d_threshold=1, n_samples=5000,
                                      granularity=0.2)

# plot the tree
nodes = np.asarray(list(tree.nodes))
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
plt.plot(nodes[:, 0], nodes[:, 1], 'bo', ms=1)

rect = patches.Rectangle(target[0], target[1][0], target[1][1], linewidth=1, edgecolor='r', facecolor='none')
ax.add_patch(rect)
plt.show()
