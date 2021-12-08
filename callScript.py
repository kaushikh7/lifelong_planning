# Script file to run the pacman in 6 different algorithms in the differnet custom layouts generated in the layouts folder.
# Respective scores are stored in respective csv files.

import os

algorithms = ['dstar', 'astar', 'lastar', 'ucs', 'dfs', 'bfs']
map_list = ['custom_layout_1','custom_layout_2','mediumMaze','bigMaze','custom_layout_3']
for j in range(5):
    for i in map_list:
        for algorithm in algorithms:
            s = "python pacman.py -l " + i + " -p SearchAgent -a fn=" + algorithm + ' --frameTime 0'

            os.system(s)
