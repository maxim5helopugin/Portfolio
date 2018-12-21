# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
      A reflex agent chooses an action at each choice point by examining
      its alternatives via a state evaluation function.

      The code below is provided as a guide.  You are welcome to change
      it in any way you see fit, so long as you don't touch our method
      headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
    ## New evaluation function
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
    ## Define closest gohst and closest food to be far away
        x,y = newPos
        closestGhost = 10000
        closestFood = 10000
    ## Search for the closest ghost
        for ghost in newGhostStates:
            if abs(ghost.getPosition()[0]-x)+abs(ghost.getPosition()[1]-y) < closestGhost:
                closestGhost = abs(ghost.getPosition()[0]-x)+abs(ghost.getPosition()[1]-y)
    ## If it is too close, the state is not desirable
        if closestGhost < 2:
            return 1
    ## If the food is in the successor cell, it is desirable
        if currentGameState.getFood()[x][y]:
            return 10
    ## Otherwise search for the closest food, and reward the distance
        for i in range(0,newFood.height):
            for j in range(0, newFood.width):
                if newFood[j][i]:
                    if abs(j-x)+abs(i-y) < closestFood:
                        closestFood = abs(j-x)+abs(i-y)
        return 10-closestFood

def scoreEvaluationFunction(currentGameState):
    """
      This default evaluation function just returns the score of the state.
      The score is the same one displayed in the Pacman GUI.

      This evaluation function is meant for use with adversarial search agents
      (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
      This class provides some common elements to all of your
      multi-agent searchers.  Any methods defined here will be available
      to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

      You *do not* need to make any changes here, but you can if you want to
      add functionality to all your adversarial search agents.  Please do not
      remove anything, however.

      Note: this is an abstract class: one that should not be instantiated.  It's
      only partially specified, and designed to be extended.  Agent (game.py)
      is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):

    def getAction(self, gameState):

    	# Get minimax action
        def value(gameState, agentIndex, depth):
        # Only change depth for max agents
          if agentIndex == 0:
              depth +=1
        # If max depth or terminal state reached, evaluate it
          if self.depth < depth:
              return self.evaluationFunction(gameState), None;
          if len(gameState.getLegalActions(agentIndex))==0:
            return self.evaluationFunction(gameState), None
        # Otherwise continue the tree
          if agentIndex == 0:
            return maxvalue(gameState, agentIndex, depth)
          if agentIndex > 0:
            return minvalue(gameState, agentIndex, depth)

        # Returns the min value of minimax
        def minvalue(gameState, agentIndex, depth):
          v = float("inf")
          bestaction = None
          for action in gameState.getLegalActions(agentIndex):
            newv = value(gameState.generateSuccessor(agentIndex, action), (agentIndex+1) % gameState.getNumAgents(), depth)
            v = min(v, newv[0])
            if v == newv[0]:
                bestaction = action
          return v, bestaction

        # Returns the max value of minimax
        def maxvalue(gameState, agentIndex, depth):
          v = float("-inf")
          bestaction = None
          for action in gameState.getLegalActions(agentIndex):
            newv = value(gameState.generateSuccessor(agentIndex, action), (agentIndex+1) % gameState.getNumAgents(), depth)
            v = max(v, newv[0])
            if v == newv[0]:
                bestaction = action
          return v, bestaction

        return value(gameState, 0, 0)[1]

class AlphaBetaAgent(MultiAgentSearchAgent):
	# Return the minimax value (alpha-betha)
    def getAction(self, gameState):
    # Return max value prunning on unnecessary subtrees
        def maxvalue(gameState, agentIndex, depth, alpha, beta):
          v = float("-inf")
          bestaction = None
          for action in gameState.getLegalActions(agentIndex):
            newv = value(gameState.generateSuccessor(agentIndex, action), (agentIndex+1) % gameState.getNumAgents(), depth, alpha, beta)
            v = max(v, newv[0])
            if v == newv[0]:
                bestaction = action
            if v > beta:
              break
            alpha = max(alpha, v)
            
          return v, bestaction

        # Return min value prunning on unnecessary subtrees
        def minvalue(gameState, agentIndex, depth, alpha, beta):
          v = float("inf")
          bestaction = None
          for action in gameState.getLegalActions(agentIndex):
            newv = value(gameState.generateSuccessor(agentIndex, action), (agentIndex+1) % gameState.getNumAgents(), depth, alpha, beta)
            v = min(v, newv[0])
            if v == newv[0]:
                bestaction = action
            if v < alpha:
              break
            beta = min(beta, v)
          return v, bestaction

        def value(gameState, agentIndex, depth, alpha, beta):
          # Only change depth for max agents
          if agentIndex == 0:
              depth +=1
        # If max depth or terminal state reached, evaluate it
          if self.depth < depth:
              return self.evaluationFunction(gameState), None;
          if len(gameState.getLegalActions(agentIndex))==0:
            return self.evaluationFunction(gameState), None
        # Otherwise continue the tree
          if agentIndex == 0:
            return maxvalue(gameState, agentIndex, depth, alpha, beta)
          if agentIndex > 0:
            return minvalue(gameState, agentIndex, depth, alpha, beta)

        alpha = float('-inf')
        beta = float('inf')
        return value(gameState, 0, 0, alpha, beta)[1]


class ExpectimaxAgent(MultiAgentSearchAgent):

	#Return expectimax action
    def getAction(self, gameState):
    #Return the max value of expectimax tree
        def maxvalue(gameState, agentIndex, depth):
           v = float("-inf")
           bestaction = None
           for action in gameState.getLegalActions(agentIndex):
             newv = value(gameState.generateSuccessor(agentIndex, action), (agentIndex+1) % gameState.getNumAgents(), depth)
             v = max(v, newv[0])
             if v == newv[0]:
                 bestaction = action
           return v, bestaction

    # Average the value of each min move
        def minvalue(gameState, agentIndex, depth):
          v = 0
          bestaction = None
          numactions = 0
          for action in gameState.getLegalActions(agentIndex):
            newv = value(gameState.generateSuccessor(agentIndex, action), (agentIndex+1) % gameState.getNumAgents(), depth)
            numactions +=1.0
            v += newv[0]
          v = v/numactions
          return v, None

    # Itrate through the tree
        def value(gameState, agentIndex, depth):
          # Only change depth for max agents
          if agentIndex == 0:
              depth +=1
        # If max depth or terminal state reached, evaluate it
          if self.depth < depth:
              return self.evaluationFunction(gameState), None;
          if len(gameState.getLegalActions(agentIndex))==0:
            return self.evaluationFunction(gameState), None
        # Otherwise continue the tree
          if agentIndex == 0:
            return maxvalue(gameState, agentIndex, depth)
          if agentIndex > 0:
            return minvalue(gameState, agentIndex, depth)
        return value(gameState, 0, 0)[1]

def betterEvaluationFunction(currentGameState):
	#Description : first, evaluate 2 metrics : happiness and danger
	#Danger - how close are ghosts. If they are too close - pacman is in great dagner
	#		- however, if pacman ate a capsule, then ghosts are in danger
	#Happiness - how happy is pacman overall. If there are food pellets left, it is unhappy
	#		   - if there are capsules left, it makes it very unhappy
	#		   - and finally, pacman is less happy when the closest food is too far away
	# the sum of happiness and danger constitute the evaluation
	# however, the dynamic score is added - to make pacman decide faster rather than wait

    newFood = currentGameState.getFood()
    newScaredTimes = [ghostState.scaredTimer for ghostState in currentGameState.getGhostStates()]

    x,y = currentGameState.getPacmanPosition()
    closestGhost = 10000
    closestFood = 100

    danger = 0
    happiness = 0


    for ghost in currentGameState.getGhostStates():
        if abs(ghost.getPosition()[0] - x) + abs(ghost.getPosition()[1] - y) < closestGhost:
            closestGhost = abs(ghost.getPosition()[0] - x) + abs(ghost.getPosition()[1] - y)
        if closestGhost < 2 and newScaredTimes[0] == 0:
            danger +=1000
        elif closestGhost < newScaredTimes[0]:
            danger +=16*closestGhost


    for i in range(0,newFood.height):
        for j in range(0, newFood.width):
            if newFood[j][i]:
                if abs(j-x)+abs(i-y)<closestFood:
                    closestFood = abs(j-x)+abs(i-y)
                happiness -=8
            if (j,i) in currentGameState.data.capsules:
                happiness -=255
    happiness -= closestFood
    estimate = -1*danger + 1*happiness + 1*currentGameState.data.score
    return estimate


# Abbreviation
better = betterEvaluationFunction

