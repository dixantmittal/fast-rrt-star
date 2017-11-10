from rrt_star import *
from rrg import *
from rrt import *
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from random_data_generator import *
import time
from plotter import *

space_region = ((0, 0), (50, 50))
obstacle = get_random_obstacles(2, space_region)

start = get_random_initial_state(space_region, obstacle)
target = get_random_target_state(space_region, obstacle)

t = datetime.now()
rrt_star, rrt_star_final_state = apply_rrt_star(space_region=space_region,
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
id = int(time.time())
plot(rrt_star, start, rrt_star_final_state, target, obstacle, 'RRT*', id)
plot(rrt, start, rrt_final_state, target, obstacle, 'RRT', id)
