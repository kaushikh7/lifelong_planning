# Team: Cybot (Team Project Topic 2)
In this project we implemented the “D* Lite”, Sven Koenig and Maxim Likhachev, AAAI 2002,and integrated into the Pacman domain for path-finding problems (from start to a fixed goal location).


# Project Topic : We implement the Life-long planning Project. 
> Implement lifelong A* search (D* lite) described in the following paper: “D* Lite”, 
> Sven Koenig and Maxim Likhachev, AAAI 2002.


## Authors and Contributions:
> Aniruddha Mondal: - Writing Abstract, Writing Introduction, Writing comparison of algorithms, Writing technical approach for algorithms, Writing results and analysis, Implementing BFS
> Ashutosh Garg: - Looking for online sources and planning the course of action for the project, Implementing D* Lite, Implementing lifelong planning A*, conducting t-tests
> Kaushik Hegde Kota: - Looking for online sources and planning the course of action for the project, Implementing D* Lite, Implementing A*, Creating script to calculate metrics for mazes and search algorithms.
> Saurabh Ughade: - Writing comparison of algorithms, Writing technical approach for algorithms, Conclusion, Writing results and analysis, Implementing DFS, Implementing UCS

# Steps to run the project
> The project is built on python 3.8 check the requirements.txt for required dependencies
1) The project can be downloaded from the below link for github
2) The below command can be executed to run the pacman
>python pacman.py -l **Layout** -z .5 -p SearchAgent -a fn=**search_algorithm**
3) Example command 
> python pacman.py -l mediumMaze -z .5 -p SearchAgent -a fn=lastar
4) Use this command to speed up the pacman game
> python pacman.py -l mediumMaze -z .5 -p SearchAgent -a fn=lastar --frameTime 0
5) To run the script file that tests the pacman on multiple search algorithms and different layouts
> Run the following command
> This script calulcates the Layout Name, Search Algorithm, Time Computation, Search Nodes Expanded, Final Score
> python callScript.py
> example. csv file -> ucs_res.csv 
6) To run the ttest and generate the results of the ttest values. We run the following commands with search algorithms as arguements -> python ttest.py arg1 arg2
7) The results and summary of the ttest values are stored in ttest_summary folder in respectively named csv's
> python ttest.py dstar lastar
> example . csv file -> ttest_dstar_vs_lastar_results.csv



## LAYOUT (Layouts Available)
- custom_layout_1
- custom_layout_2
- mediumMaze
- custom_layout_3
- bigMaze

## SEARCH_ALGORITHM (Implemented search Algorithms):
- astar : A* search implementation
- lastar: Life Long A* search implementation
- dstar : DLite* search implementation
- dfs : Depth First Search Implementation
- bfs : Breadth First Search Implementation 
- ucs : Uniform Search Implementation


