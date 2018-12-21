# valueIterationAgents.py
# -----------------------
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


import mdp, util

from learningAgents import ValueEstimationAgent

class ValueIterationAgent(ValueEstimationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A ValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100):
        """
          Your value iteration agent should take an mdp on
          construction, run the indicated number of iterations
          and then act according to the resulting policy.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state, action, nextState)
              mdp.isTerminal(state)
        """
        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations
        self.values = util.Counter() # A Counter is a dict with default 0

#########################################################################
        # for each iteration, inititalize new_values to null
        for i in range(0, iterations):
            new_values = util.Counter()
        ## for each possible start state, get q-values of each state-action pair
            for start_state in self.mdp.getStates():
            	Qs = []
                for possible_action in self.mdp.getPossibleActions(start_state):
                    Qs.append(self.computeQValueFromValues(start_state, possible_action))
                if Qs:
        ## if there are any, get their max
                    new_values[start_state] = max(Qs);
            self.values = new_values


    def getValue(self, state):
        return self.values[state]


    def computeQValueFromValues(self, state, action):
    # compute the sum of q-values with non-deterministic actions
        Qvalue = []
        successor_states = self.mdp.getTransitionStatesAndProbs(state, action)
        for (state_prime,probability) in successor_states:
            Qvalue.append(probability*(self.mdp.getReward(state,action, state_prime)+self.discount*self.getValue(state_prime)))
        return sum(Qvalue)

    def computeActionFromValues(self, state):
## Observe the V values and return the best action, determined by the transition function
        new_action = None
        temp_v = 0
        greatest_v = float("-inf");
        for action in self.mdp.getPossibleActions(state):
            for new_state in self.mdp.getTransitionStatesAndProbs(state, action):
                temp_v +=self.values[new_state[0]]*new_state[1]
            if temp_v>=greatest_v:
                greatest_v = temp_v
                new_action = action
            temp_v = 0
        return new_action

    def getPolicy(self, state):
        return self.computeActionFromValues(state)

    def getAction(self, state):
        "Returns the policy at the state (no exploration)."
        return self.computeActionFromValues(state)

    def getQValue(self, state, action):
        return self.computeQValueFromValues(state, action)
