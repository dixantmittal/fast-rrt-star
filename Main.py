from rrt_star import *
from rrg import *
from rrt import *
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from random_data_generator import *

space_region = ((0, 0), (50, 50))
obstacle = get_random_obstacles(2, space_region)

start = get_random_initial_state(space_region, obstacle)
target = get_random_target_state(space_region, obstacle)

t = datetime.now()
rrt_star, rrt_start_final_state = apply_rrt_star(space_region=space_region,
                                                 starting_state=start,
                                                 target_region=target,
                                                 obstacle_map=obstacle,
                                                 d_threshold=3,
                                                 n_samples=5000,
                                                 granularity=0.3)
print('total time taken: ', datetime.now() - t)

t = datetime.now()
rrt, rrt_final_state = apply_rrt(space_region=space_region,
                                 starting_state=start,
                                 target_region=target,
                                 obstacle_map=obstacle,
                                 d_threshold=3,
                                 n_samples=5000,
                                 granularity=0.3)
print('total time taken: ', datetime.now() - t)

# plot the trees
# Plotting takes some time as it has to iterate and create each edge. Could not find a better solution
nodes = np.asarray(list(rrt_star.nodes))
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')

target_rect = patches.Rectangle(target[0], target[1][0], target[1][1], linewidth=1, edgecolor='g', facecolor='g')
ax.add_patch(target_rect)

for val in obstacle.values():
    ax.add_patch(patches.Rectangle(val[0], val[1][0], val[1][1], linewidth=1, edgecolor='r', facecolor='r'))

edges = list(rrt_star.edges)
for edge in edges:
    edge = np.array(edge).transpose()
    plt.plot(edge[0], edge[1], 'c-', edge[0], edge[1], 'bo', ms=1)
# plt.plot(nodes[:, 0], nodes[:, 1], 'bo', ms=1)

if rrt_start_final_state is not None:
    path = nx.shortest_path(rrt_star, start, rrt_start_final_state)
    plt.plot(np.array(path)[:, 0], np.array(path)[:, 1], 'k-', ms=3)

# plot the tree
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')

target_rect = patches.Rectangle(target[0], target[1][0], target[1][1], linewidth=1, edgecolor='g', facecolor='g')
ax.add_patch(target_rect)

for val in obstacle.values():
    ax.add_patch(patches.Rectangle(val[0], val[1][0], val[1][1], linewidth=1, edgecolor='r', facecolor='r'))

edges = list(rrt.edges)
for edge in edges:
    edge = np.array(edge).transpose()
    plt.plot(edge[0], edge[1], 'c-', edge[0], edge[1], 'bo', ms=1)

if rrt_final_state is not None:
    path = nx.shortest_path(rrt, start, rrt_final_state)
    plt.plot(np.array(path)[:, 0], np.array(path)[:, 1], 'k-', ms=3)
plt.show()
