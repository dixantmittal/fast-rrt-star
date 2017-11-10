import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from datetime import datetime

"""performance is deeply affected by granularity, d_threshold and ball radius, so choose the values according to the 
use case > more ball_radius = better path and more computation time > less granularity = finer check for collision 
and more computation time > more the d_threshold, more rapidly it will explore the space, may result in more 
collisions. """

collision_cache = {}

volume_of_unit_ball = {
    1: 2,
    2: 3.142,
    3: 4.189,
    4: 4.935,
    5: 5.264,
    6: 5.168,
    7: 4.725,
    8: 4.059,
    9: 3.299,
    10: 2.550
}


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
    if d <= d_threshold:
        return tuple(random_point)
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
    if collision_cache.get(m_new, False):
        return False

    if is_obstacle_space(m_new, obstacle_map):
        collision_cache[m_new] = True
        return False

    m_g = np.array(m_g)
    m_new = np.array(m_new)
    d = np.asscalar(dist(m_g, m_new))
    unit_vector = (m_new - m_g) / d
    floor = int(np.floor(d / granularity))

    for i in range(floor):
        _m = m_g + i * granularity * unit_vector

        if collision_cache.get(tuple(_m), False):
            return False

        if is_obstacle_space(_m, obstacle_map):
            collision_cache[tuple(_m)] = True
            return False

    return True


def dist(m_g, m_new):
    m_g = np.array(m_g)
    m_new = np.array(m_new)

    if m_g.ndim == 1:
        m_g = m_g.reshape(1, -1)

    if m_new.ndim == 1:
        m_new = m_new.reshape(1, -1)

    d = np.power(np.sum((m_new - m_g) ** 2, axis=1), 0.5)
    return d


def nearest_neighbours(graph, m_new, r):
    nodes = list(graph.nodes())
    d = dist(nodes, m_new)
    nearest_nodes = np.array(nodes)[d < r]
    return tuple(map(tuple, nearest_nodes))


def lies_in_area(m_new, area):
    frame, _range = area
    frame = np.array(frame)
    m_new = np.array(m_new)

    diff = m_new - frame

    return np.all(diff <= _range) and np.all(diff >= 0)


def find_free_space(space_region, obstacle_map):
    _, space_range = space_region
    l, b = space_range
    space_area = l * b

    obstacle_area = 0
    for obstacle in obstacle_map.values():
        _, obstacle_range = obstacle
        l, b = obstacle_range
        obstacle_area += l * b

    return space_area - obstacle_area


def apply_rrt_star(space_region, starting_state, target_region, obstacle_map, n_samples=1000, granularity=0.1,
                   d_threshold=0.5, use_bias=False):
    tree = nx.DiGraph()
    tree.add_node(starting_state)

    space_dim = len(starting_state)

    final_state = None

    # cost for each vertex
    cost = {starting_state: 0}

    gamma = 1 + np.power(2, space_dim) * (1 + 1.0 / space_dim) * find_free_space(space_region, obstacle_map)

    for i in range(n_samples):
        # print(i)
        # select node to expand
        m_g, random_point = select_node_to_expand(tree, space_region, use_bias)

        # sample a new point
        m_new = sample_new_point(m_g, random_point, d_threshold)

        # check if m_new lies in space_region
        if not lies_in_area(m_new, space_region):
            continue

        # if m_new is not collision free, sample any other point
        if not is_collision_free(m_g, m_new, obstacle_map, granularity):
            continue

        # find k nearest neighbours
        radius = np.minimum(np.power(gamma / volume_of_unit_ball[space_dim] * np.log(i + 1) / (i + 1),
                                     1 / space_dim), d_threshold)
        m_near = nearest_neighbours(tree, m_new, r=radius)

        min_cost = m_g
        d_min_cost = dist(m_g, m_new)

        # look for shortest cost path to m_new
        for m_g in m_near:

            # check if path between(m_g,m_new) defined by motion-model is collision free
            is_free = is_collision_free(m_g, m_new, obstacle_map, granularity)

            # if path is free, add new node to tree
            if is_free:
                d = dist(m_g, m_new)
                c = cost[m_g] + d

                if c < cost[min_cost] + d_min_cost:
                    min_cost = m_g
                    d_min_cost = d

        tree.add_weighted_edges_from([(min_cost, m_new, d_min_cost)])
        cost[m_new] = cost[min_cost] + d_min_cost

        # update m_near's neighbours as well

        for m_g in m_near:

            # check if path between(m_g,m_new) is collision free
            is_free = is_collision_free(m_g, m_new, obstacle_map, granularity)

            # if path is free, add new node to tree
            if is_free:
                d = dist(m_g, m_new)
                c = cost[m_new] + d

                if c < cost[m_g]:
                    tree.remove_edge(list(tree.predecessors(m_g))[0], m_g)
                    tree.add_weighted_edges_from([(m_new, m_g, d)])
                    cost[m_g] = c

        # if target is reached, return the tree and final state
        if lies_in_area(m_new, target_region):
            print('Target reached at i:', i)
            if final_state is None:
                final_state = m_new
            elif cost[m_new] < cost[final_state]:
                final_state = m_new


    if final_state is None:
        print("Target not reached.")
    return tree, final_state


# test
start = (20, 20)
target = ((30, 10), (5, 5))
obstacle = {
    1: ((3, 4), (2, 2)),
    2: ((10, 7), (5, 5)),
    3: ((25, 7), (5, 5))
}

t = datetime.now()
tree, final_state = apply_rrt_star(((0, 0), (40, 40)), start, target, obstacle, d_threshold=3,
                                   n_samples=1000, granularity=0.5)
print('total time taken: ', datetime.now() - t)

# plot the tree
nodes = np.asarray(list(tree.nodes))
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')

target_rect = patches.Rectangle(target[0], target[1][0], target[1][1], linewidth=1, edgecolor='g', facecolor='g')
ax.add_patch(target_rect)

for val in obstacle.values():
    ax.add_patch(patches.Rectangle(val[0], val[1][0], val[1][1], linewidth=1, edgecolor='r', facecolor='r'))

edges = list(tree.edges)
for edge in edges:
    edge = np.array(edge).transpose()
    plt.plot(edge[0], edge[1], 'c-', edge[0], edge[1], 'bo', ms=1)
# plt.plot(nodes[:, 0], nodes[:, 1], 'bo', ms=1)

if final_state is not None:
    path = nx.shortest_path(tree, start, final_state)
    plt.plot(np.array(path)[:, 0], np.array(path)[:, 1], 'g-', ms=2)
plt.show()
