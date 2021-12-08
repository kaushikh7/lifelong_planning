import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


columns = ['Layout Name','Algorithm','Computation Time','Search Nodes','Score']
astar = pd.read_csv('astar_res.csv',header = None,names=columns)
bfs =pd.read_csv('bfs_res.csv',header = None,names=columns)
dfs = pd.read_csv('dfs_res.csv',header = None,names=columns)
dstar = pd.read_csv('dstar_res.csv',header = None,names=columns)
lastar = pd.read_csv('lastar_res.csv',header = None,names=columns)
ucs = pd.read_csv('ucs_res.csv',header = None,names=columns)


final_df = pd.concat([astar,bfs,dfs,dstar,lastar,ucs])

final_df.groupby('Algorithm').mean()

g = sns.FacetGrid(final_df, col="Layout Name", height=4, aspect=.8)
g.map(sns.barplot, "Algorithm", "Search Nodes")
g.fig.suptitle('Search Nodes Expanded')
plt.show()


g = sns.FacetGrid(final_df, col="Layout Name", height=7,aspect = 1)
g.map(sns.barplot, "Algorithm", "Computation Time")
g.fig.suptitle('Computation Time')
plt.show()

g = sns.FacetGrid(final_df, col="Layout Name", height=7,aspect = 1)
g.map(sns.barplot, "Algorithm", "Score")
g.fig.suptitle('Final Pacman Score')
plt.show()