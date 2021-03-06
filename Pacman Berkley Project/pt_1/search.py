# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):

# Explore the states with the help of a stack
    backtrack = util.Stack()
    fringe = util.Stack()
    directionsTo = {}
    pathTo = {}
    actionList = []
    explored = {}

## Dfs
    pathTo[problem.getStartState()] = problem.getStartState()
    fringe.push(problem.getStartState())
    
    while not fringe.isEmpty():
      stateBeingExplored = fringe.pop()
      explored[stateBeingExplored] = 1
      if problem.isGoalState(stateBeingExplored):
        break;
      for successor in problem.getSuccessors(stateBeingExplored):
        if not successor[0] in explored:
          pathTo[successor[0]] = stateBeingExplored
          directionsTo[successor[0]] = successor[1]
          fringe.push(successor[0])

## backtrack to the beginning
    successor = stateBeingExplored
    while not successor == problem.getStartState():
      backtrack.push(directionsTo[successor])
      successor = pathTo[successor]

## reconstruct the path
    while not backtrack.isEmpty():
      actionList.append(backtrack.pop())
    return actionList


def breadthFirstSearch(problem):
## Explore states with the help of the queue
    backtrack = util.Stack()
    fringe = util.Queue()
    directionsTo = {}
    pathTo = {}
    actionList = []

## Bfs
    pathTo[problem.getStartState()] = problem.getStartState()
    fringe.push(problem.getStartState())
    
    while not fringe.isEmpty():
      stateBeingExplored = fringe.pop()
      if problem.isGoalState(stateBeingExplored):
        break;
      for successor in problem.getSuccessors(stateBeingExplored):
        if not successor[0] in pathTo:
          pathTo[successor[0]] = stateBeingExplored
          directionsTo[successor[0]] = successor[1]
          fringe.push(successor[0])

## backtrack to the beginning
    successor = stateBeingExplored
    while not successor == problem.getStartState():
      backtrack.push(directionsTo[successor])
      successor = pathTo[successor]

## reconstruct the path
    while not backtrack.isEmpty():
      actionList.append(backtrack.pop())
    return actionList

def uniformCostSearch(problem):
## Explore the states with the help of the priority queue
## The priority is high for the shorter paths
    backtrack = util.Stack()
    fringe = util.PriorityQueue()
    pathCostTo = {}
    directionsTo = {}
    pathTo = {}
    actionList = []

## Uniform cost search
    pathTo[problem.getStartState()] = problem.getStartState()
    fringe.push(problem.getStartState(), 0)
    pathCostTo[problem.getStartState()] = 0
    
    while not fringe.isEmpty():
      stateBeingExplored = fringe.pop()
      if problem.isGoalState(stateBeingExplored):
        break;
      for successor in problem.getSuccessors(stateBeingExplored):
        if not successor[0] in pathTo or pathCostTo[successor[0]] > (successor[2] + pathCostTo[stateBeingExplored]):
          pathCostTo[successor[0]] = successor[2] + pathCostTo[stateBeingExplored]
          pathTo[successor[0]] = stateBeingExplored
          directionsTo[successor[0]] = successor[1]
          fringe.push(successor[0], pathCostTo[successor[0]])

## backtrack to the beginning
    successor = stateBeingExplored
    while not successor == problem.getStartState():
      backtrack.push(directionsTo[successor])
      successor = pathTo[successor]

## reconstruct the path
    while not backtrack.isEmpty():
      actionList.append(backtrack.pop())
    return actionList


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
## Explore states with the help of priority queue
## priority is high for the states with lower distance+heuristic
    backtrack = util.Stack()
    fringe = util.PriorityQueue()
    pathCostTo = {}
    directionsTo = {}
    pathTo = {}
    actionList = []

## A* search
    pathTo[problem.getStartState()] = problem.getStartState()
    fringe.push(problem.getStartState(), 0)
    pathCostTo[problem.getStartState()] = 0
    
    while not fringe.isEmpty():
      stateBeingExplored = fringe.pop()
      if problem.isGoalState(stateBeingExplored):
        break;
      for successor in problem.getSuccessors(stateBeingExplored):
        if not successor[0] in pathTo or pathCostTo[successor[0]] > (successor[2] + pathCostTo[stateBeingExplored]):
          pathCostTo[successor[0]] = successor[2] + pathCostTo[stateBeingExplored]
          pathTo[successor[0]] = stateBeingExplored
          directionsTo[successor[0]] = successor[1]
          fringe.push(successor[0], pathCostTo[successor[0]]+heuristic(successor[0], problem))

## backtrack to the beginning
    successor = stateBeingExplored
    while not successor == problem.getStartState():
      backtrack.push(directionsTo[successor])
      successor = pathTo[successor]

## reconstruct the path
    while not backtrack.isEmpty():
      actionList.append(backtrack.pop())
    return actionList

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
