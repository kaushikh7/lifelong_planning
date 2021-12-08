

import util
from game import Actions
from util import PriorityQueueLAS, PriorityQueueWithFunction, manhattanDistance
from global_values import static_list

class SearchProblem:


    def getStartState(self):

        util.raiseNotDefined()

    def isGoalState(self, state):

        util.raiseNotDefined()

    def getSuccessors(self, state):

        util.raiseNotDefined()

    def getCostOfActions(self, actions):

        util.raiseNotDefined()

def depthFirstSearch(problem):

    stack=util.Stack()
    static_list.append("dfs")
    act=findingRoute(problem,"dfs",stack)
    return act



def breadthFirstSearch(problem):

    queue = util.Queue()
    static_list.append("bfs")
    act = findingRoute(problem, "bfs", queue)
    return act


def uniformCostSearch(problem):

    priorityQueue = util.PriorityQueue()
    static_list.append("ucs")
    act = findingRoute(problem,"ucs",priorityQueue)
    return act

def nullHeuristic(state, problem=None):

    return 0

# Dynamic A* search algorithm
def aStarSearch(problem, heuristic):


    def aStarPath():
        def priorityFunction(node):
            state, sequenceOfActs, path_cost = node
            heuristic_cost = heuristic(state, problem)
            return path_cost+heuristic_cost
        frontier = PriorityQueueWithFunction(priorityFunction)
        return commonSearch(frontier)


    def commonSearch(frontier):
        rootNode = problem.dynamicStartState
        setExplored = set()
        actions_sequence = list()
        path_cost = 0
        frontier.push((rootNode, actions_sequence, path_cost))
        while not frontier.isEmpty():
            parent, actions_sequence, path_cost = frontier.pop()
            if parent not in setExplored:
                if problem.getGoalState() == parent:
                    return actions_sequence+[(parent,None)]
                setExplored.add(parent)
                for successor in problem.getSuccessors(parent):
                    state, action, step_cost = successor
                    new_actions_sequence = actions_sequence[:]
                    new_actions_sequence += [(parent, action)]
                    cost = path_cost+step_cost
                    frontier.push((state, new_actions_sequence, cost))

    def planning():
        path = aStarPath()
        if len(path) == 1 and path[0][0] == problem.getGoalState():
            return True
        for index in range(len(path)-1):
            currentState, currentAction = path[index]
            nextState, _ = path[index+1]
            problem.finalPath.append((currentState, currentAction))
            print("--> " + str(nextState),)
            if problem.isObstacle(nextState):
                print ("\nObstacle @ "+ str(nextState))
                print ("Replanning...")
                problem.insertObstacle(nextState)
                problem.dynamicStartState = currentState
                return False
            elif nextState == problem.getGoalState():
                return True

    def main():
        problem.finalPath = []
        problem.dynamicStartState = problem.getStartState()
        stop = False
        print('The goal position is', problem.getGoalState())
        print("The path is: ")
        print(problem.dynamicStartState,)

        while (problem.dynamicStartState != problem.getGoalState())  and not stop:
            stop = planning()
        problem.finalPath.append((problem.getGoalState(), None))
        print ("\nDone Planning")
        actions = []
        states = []

        for index in range(len(problem.finalPath[:-1])):
            currentState, currentAction = problem.finalPath[index]
            nextState, _ = problem.finalPath[index+1]
            if currentState != nextState:
                actions.append(currentAction)
                states.append(currentState)
        problem.drawObstacles()
        problem.printPath(states)
        static_list.append("astar")
        print('Size of the Layout: ', str(problem.height)+'x'+str(problem.width))
        print('Path Length: ', len(actions))
        print('Number of obstacles: ', len(problem.obstacles))
        return actions

    return main()

# The life long A* search implementation
def lifeLongAStarSearch(problem, heuristic):

    def calculateKey(state):
        g_rhs = min(problem.g[state], problem.rhs[state])
        return (g_rhs + heuristic(state, problem), g_rhs)

    def initialize():
        for state in problem.getStates():
            problem.rhs[state] = float('inf')
            problem.g[state] = float('inf')
        problem.rhs[problem.dynamicStartState] = 0
        problem.U.insert(problem.dynamicStartState, calculateKey(problem.dynamicStartState))

    def updateVertex(u):
        if u != problem.dynamicStartState:
            prevKeys = [float('inf')]
            for successor, _, cost in problem.getSuccessors(u):
                prevKeys.append(problem.g[successor]+cost)
            problem.rhs[u] = min(prevKeys)
        problem.U.remove(u)
        if problem.g[u] != problem.rhs[u]:
            problem.U.insert(u, calculateKey(u))

    def computeShortestPath():
        goal = problem.getGoalState()
        while problem.U.topKey() < calculateKey(goal) or problem.rhs[goal] != problem.g[goal]:
            u = problem.U.pop()
            if problem.g[u] > problem.rhs[u]:
                problem.g[u] = problem.rhs[u]
                for successor, _, _ in problem.getSuccessors(u):
                    updateVertex(successor)
            else:
                problem.g[u] = float('inf')
                updateVertex(u)
                for successor, _, _ in problem.getSuccessors(u):
                    updateVertex(successor)


    def shortestPath():
        path = []
        state = (problem.getGoalState(), None)
        path.append(state)
        while state[0] != problem.dynamicStartState:
            minimum = float('inf')
            for successor, action, _ in problem.getSuccessors(state[0]):
                if minimum > problem.g[successor]:
                    minimum = problem.g[successor]
                    state = (successor, Actions.reverseDirection(action))
            path.append(state)
        return path[::-1]

    def planning():
        path = shortestPath()
        if len(path) == 1 and path[0][0] == problem.getGoalState():
            return True
        for index in range(len(path)-1):
            currentState, currentAction = path[index]
            nextState, _ = path[index+1]
            problem.finalPath.append((currentState, currentAction))
            print("--> " + str(nextState),)
            if problem.isObstacle(nextState):
                print("\nObstacle @ "+ str(nextState))
                print("Replanning...")
                problem.insertObstacle(nextState)
                updateVertex(nextState)
                problem.dynamicStartState = currentState
                return False
            elif nextState == problem.getGoalState():
                return True

    def main():
        problem.U = PriorityQueueLAS()
        problem.g = {}
        problem.rhs = {}
        problem.finalPath = []
        problem.dynamicStartState = problem.getStartState()
        initialize()
        stop = False
        print('The goal position is', problem.getGoalState())
        print("The path is: ")
        print(problem.dynamicStartState,)
        while (problem.dynamicStartState != problem.getGoalState())  and not stop:
            initialize()
            computeShortestPath()
            stop = planning()
        problem.finalPath.append((problem.getGoalState(), None))
        print("\nDone Planning")
        actions = []
        states = []
        for index in range(len(problem.finalPath[:-1])):
            currentState, currentAction = problem.finalPath[index]
            nextState, _ = problem.finalPath[index+1]
            if currentState != nextState:
                actions.append(currentAction)
                states.append(currentState)
        problem.drawObstacles()
        problem.printPath(states)
        static_list.append("lastar")
        print('Path Length: ', len(actions))
        print('Size of the Layout: ', str(problem.height)+'x'+str(problem.width))
        print('Number of obstacles: ', len(problem.obstacles))
        return actions
    return main()

def dStarSearch(problem, heuristic):

    def calculateKey(state):
        g_rhs = min(problem.g[state], problem.rhs[state])
        return (g_rhs + manhattanDistance(state, problem.s['start']) + problem.k['m'], g_rhs)

    def initialize():
        problem.U = PriorityQueueLAS()
        problem.g = {}
        problem.rhs = {}
        problem.k = {}
        problem.k['m'] = 0
        problem.s = {}
        problem.s['start'] = problem.getStartState()
        problem.s['goal'] = problem.getGoalState()
        for state in problem.getStates():
            problem.rhs[state] = float('inf')
            problem.g[state] = float('inf')
        problem.rhs[problem.s['goal']] = 0
        problem.U.insert(problem.s['goal'], calculateKey(problem.s['goal']))

    def updateVertex(u):
        if u != problem.s['goal']:
            prevKeys = [float('inf')]
            for successor, _, cost in problem.getSuccessors(u):
                prevKeys.append(problem.g[successor]+cost)
            problem.rhs[u] = min(prevKeys)
        problem.U.remove(u)
        if problem.g[u] != problem.rhs[u]:
            problem.U.insert(u, calculateKey(u))

    def computeShortestPath():
        while problem.rhs[problem.s['start']] != problem.g[problem.s['start']] or problem.U.topKey() < calculateKey(problem.s['start']):
            problem.k['old'] = problem.U.topKey()
            u = problem.U.pop()
            if problem.k['old'] < calculateKey(u):
                problem.U.insert(u, calculateKey(u))
            elif problem.g[u] > problem.rhs[u]:
                problem.g[u] = problem.rhs[u]
                for successor, _, _ in problem.getSuccessors(u):
                    updateVertex(successor)
            else:
                problem.g[u] = float('inf')
                updateVertex(u)
                for successor, _, _ in problem.getSuccessors(u):
                    updateVertex(successor)

    def main():
        initialize()
        problem.finalPath = []
        problem.s['last'] = problem.s['start']
        problem.dynamicAction = None
        computeShortestPath()
        print('The goal position is', problem.getGoalState())
        print("The path is: ")
        print(problem.s['start'],)
        while (problem.s['start'] != problem.s['goal']):
            if problem.g[problem.s['start']] == float('inf'):
                return []
            minimum = float('inf')
            problem.s['successor'] = None
            for successor, action, cost in problem.getSuccessors(problem.s['start']):
                updatedCost = problem.g[successor]+cost
                if updatedCost < minimum:
                    minimum = updatedCost
                    problem.s['successor'] = successor
                    problem.dynamicAction = action
            print("--> " + str(problem.s['successor']),)
            if problem.isObstacle(problem.s['successor']):
                print("\nObstacle @ "+ str(problem.s['successor']))
                print("Replanning...")
                problem.insertObstacle(problem.s['successor'])
                problem.k['m'] += manhattanDistance(problem.s['last'], problem.s['start'])
                problem.s['last'] = problem.s['start']
                updateVertex(problem.s['successor'])
                computeShortestPath()
            else:
                problem.finalPath.append((problem.s['start'], problem.dynamicAction))
                problem.s['start'] = problem.s['successor']
        problem.finalPath.append((problem.s['goal'], None))
        print("\nDone Planning")
        actions = []
        states = []
        for index in range(len(problem.finalPath[:-1])):
            currentState, currentAction = problem.finalPath[index]
            nextState, _ = problem.finalPath[index+1]
            if currentState != nextState:
                actions.append(currentAction)
                states.append(currentState)
        problem.drawObstacles()
        problem.printPath(states)
        static_list.append("dstar")

        print('Size of the Layout: ', str(problem.height)+'x'+str(problem.width))
        print('Path Length: ', len(actions))
        print('Number of obstacles: ', len(problem.obstacles))

        return actions

    return main()


def findingRoute(problem, method, tracker):
    start = problem.getStartState()
    state = [start, [], 0]
    if method == "dfs" or method == "bfs":
        tracker.push(state)
    else:
        tracker.push(state, 0)
    closedSet = []

    while not tracker.isEmpty():
        curState, action, cost = tracker.pop()
        if curState not in closedSet:
            if problem.isGoalState(curState):
                return action
            else:
                neighbour = problem.getSuccessors(curState)

                for latestState, latestAction, latestCost in neighbour:
                    newAction = action + [latestAction]
                    newCost = cost + latestCost
                    if method == "dfs" or method == "bfs":
                        tracker.push((latestState, newAction, newCost))
                    elif method == "ucs":
                        tracker.push((latestState, newAction, newCost), newCost)
                    else:
                        pass
            closedSet.append(curState)


def createAbbreviations():
    astar = aStarSearch
    lastar = lifeLongAStarSearch
    dstar = dStarSearch
    bfs = breadthFirstSearch
    dfs = depthFirstSearch
    ucs = uniformCostSearch
    return astar,lastar,dstar,bfs,dfs,ucs


astar,lastar,dstar,bfs,dfs,ucs = createAbbreviations()