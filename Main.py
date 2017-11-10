from rrt_star import *
from rrg import *
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from random_data_generator import *

# test
space_region = ((0, 0), (50, 50))
obstacle = get_random_obstacles(2, space_region)

start = get_random_initial_state(space_region, obstacle)
target = get_random_target_state(space_region, obstacle)

t = datetime.now()
tree, final_state = apply_rrg(space_region=space_region,
                                   starting_state=start,
                                   target_region=target,
                                   obstacle_map=obstacle,
                                   d_threshold=3,
                                   n_samples=5000,
                                   granularity=0.3)
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
# for edge in edges:
#     edge = np.array(edge).transpose()
#     plt.plot(edge[0], edge[1], 'c-', edge[0], edge[1], 'bo', ms=1)
plt.plot(nodes[:, 0], nodes[:, 1], 'bo', ms=1)

if final_state is not None:
    path = nx.shortest_path(tree, start, final_state)
    plt.plot(np.array(path)[:, 0], np.array(path)[:, 1], 'k-', ms=3)
plt.show()
