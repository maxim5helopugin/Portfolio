# qlearningAgents.py
# ------------------
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


from game import *
from learningAgents import ReinforcementAgent
from featureExtractors import *

import random,util,math

class QLearningAgent(ReinforcementAgent):
    """
      Q-Learning Agent

      Functions you should fill in:
        - computeValueFromQValues
        - computeActionFromQValues
        - getQValue
        - getAction
        - update

      Instance variables you have access to
        - self.epsilon (exploration prob)
        - self.alpha (learning rate)
        - self.discount (discount rate)

      Functions you should use
        - self.getLegalActions(state)
          which returns legal actions for a state
    """
    def __init__(self, **args):
        ReinforcementAgent.__init__(self, **args)
        self.qvalues = {}


## return Q(s,a) from a dictionary if there is one
    def getQValue(self, state, action):
        if (state, action) not in self.qvalues:
            return 0.0
        else:
            return self.qvalues[(state,action)]

## return V(s) -> the max Q(s,a) from the state
    def computeValueFromQValues(self, state):
        actions = self.getLegalActions(state)
        values = []
        if len(actions) == 0:
            return 0.0
        else:
            for action in actions:
                values.append(self.getQValue(state,action))
        return max(values)

# return the action from the state, defined as the best Q(s,a)
    def computeActionFromQValues(self, state):
        actions = self.getLegalActions(state)
        current_max = float("-inf")
        best_action = None
        if len(actions) == 0:
            return best_action
        else:
            for action in actions:
                if self.getQValue(state, action) > current_max:
                    best_action = action
                    current_max = self.getQValue(state, action)
            return best_action

# epsilon - greedy. explore epsilon of time, exploit 1-epsilon of time
    def getAction(self, state):
        legalActions = self.getLegalActions(state)
        action = None
        if(util.flipCoin(self.epsilon)):
        	return random.choice(legalActions)
        else:
        	return self.computeActionFromQValues(state)

## update the q-values of the visited state
    def update(self, state, action, nextState, reward):
        if (state,action) not in self.qvalues:
            self.qvalues[(state,action)] = 0.0
        self.qvalues[(state,action)] = (1-self.alpha)*self.qvalues[(state,action)] + self.alpha*(reward+self.discount*self.computeValueFromQValues(nextState))

    def getPolicy(self, state):
        return self.computeActionFromQValues(state)

    def getValue(self, state):
        return self.computeValueFromQValues(state)


class PacmanQAgent(QLearningAgent):
    "Exactly the same as QLearningAgent, but with different default parameters"

    def __init__(self, epsilon=0.05,gamma=0.8,alpha=0.2, numTraining=0, **args):
        """
        These default parameters can be changed from the pacman.py command line.
        For example, to change the exploration rate, try:
            python pacman.py -p PacmanQLearningAgent -a epsilon=0.1

        alpha    - learning rate
        epsilon  - exploration rate
        gamma    - discount factor
        numTraining - number of training episodes, i.e. no learning after these many episodes
        """
        args['epsilon'] = epsilon
        args['gamma'] = gamma
        args['alpha'] = alpha
        args['numTraining'] = numTraining
        self.index = 0  # This is always Pacman
        QLearningAgent.__init__(self, **args)

    def getAction(self, state):
        """
        Simply calls the getAction method of QLearningAgent and then
        informs parent of action for Pacman.  Do not change or remove this
        method.
        """
        action = QLearningAgent.getAction(self,state)
        self.doAction(state,action)
        return action


class ApproximateQAgent(PacmanQAgent):
    """
       ApproximateQLearningAgent

       You should only have to overwrite getQValue
       and update.  All other QLearningAgent functions
       should work as is.
    """
    def __init__(self, extractor='IdentityExtractor', **args):
        self.featExtractor = util.lookup(extractor, globals())()
        PacmanQAgent.__init__(self, **args)
        self.weights = util.Counter()

    def getWeights(self):
        return self.weights

# returns the dot product of W and F(s,a)
    def getQValue(self, state, action):
        features = self.featExtractor.getFeatures(state, action)
        qval = 0
        weights = self.getWeights()
        for i in features:
            if i not in weights:
                weights[i] = 0
            qval += weights[i]*features[i]
        return qval


# update the vector of weights, depending on the reward and q-val of next state
    def update(self, state, action, nextState, reward):

        difference = reward + self.discount*self.getValue(nextState)-self.getQValue(state, action)
        features = self.featExtractor.getFeatures(state, action)
        for i in features:
            self.weights[i] += self.alpha * difference * features[i]

    def final(self, state):
        "Called at the end of each game."
        # call the super-class final method
        PacmanQAgent.final(self, state)

        # did we finish training?
        if self.episodesSoFar == self.numTraining:
            pass
