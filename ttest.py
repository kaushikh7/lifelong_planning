import researchpy as rp
import pandas as pd
import sys

print('Number of arguments:', len(sys.argv), 'arguments.')
print('Argument List:', str(sys.argv))

def calcTtest(df1,df2,arg1,arg2):

    summary, results = rp.ttest(group1= df1[2], group1_name= arg1,
                                group2= df2[2], group2_name= arg2, paired=True)
    print(summary)
    print(results)

    sum_df = pd.DataFrame(summary)
    sum_df.to_csv(f'ttest_summary/ttest_{arg1}_vs_{arg2}_summary.csv')
    res_df = pd.DataFrame(results)
    res_df.to_csv(f'ttest_summary/ttest_{arg1}_vs_{arg2}_results.csv')
    return

arg1 = sys.argv[1]
arg2 = sys.argv[2]

lastar = pd.read_csv('lastar_res.csv', header=None)
dstar = pd.read_csv('dstar_res.csv', header=None)
astar = pd.read_csv('astar_res.csv', header=None)
bfs = pd.read_csv('bfs_res.csv', header=None)
dfs = pd.read_csv('dfs_res.csv', header=None)
ucs = pd.read_csv('ucs_res.csv', header=None)

df1 = eval(arg1)
df2 = eval(arg2)

calcTtest(df1,df2,arg1,arg2)

