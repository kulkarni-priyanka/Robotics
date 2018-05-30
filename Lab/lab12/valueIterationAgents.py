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

        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations
        self.values = util.Counter() # A Counter is a dict with default 0

        #print self.mdp.getStates()

        #for s in self.mdp.getStates():
            #print self.values[s]

        state_qval = 0
        for i in range(iterations):
            valuesCopy = self.values.copy()
            for s in self.mdp.getStates():
                #state_qval = self.values[s]
                state_qval = None
                qval_temp = None
                for a in self.mdp.getPossibleActions(s):
                    qval_temp = self.computeQValueFromValues(s,a)
                    #print 'Hello'
                    #print self.getTransitionStatesAndProbs(s,a)
                    if (state_qval == None) or (qval_temp > state_qval):
                        state_qval = qval_temp
                if qval_temp == None:
                    state_qval = 0
                #self.values[s] = state_qval
                valuesCopy[s]  = state_qval
            self.values = valuesCopy


    def getValue(self, state):
        """
          Return the value of the state (computed in __init__).
        """
        return self.values[state]


    def computeQValueFromValues(self, state, action):
        """
          Compute the Q-value of action in state from the
          value function stored in self.values.
        """
        final_qval=0
        #print 'Hello'
        #print self.mdp.getTransitionStatesAndProbs(state,action)

        for t in range(len(self.mdp.getTransitionStatesAndProbs(state,action))):
            tup = self.mdp.getTransitionStatesAndProbs(state,action)
            final_qval += (tup[t-1])[1]*(self.mdp.getReward(state, action, (tup[t-1])[0]) + self.discount*(self.values[(tup[t-1])[0]]))

        return final_qval

    def computeActionFromValues(self, state):
        """
          The policy is the best action in the given state
          according to the values currently stored in self.values.

          You may break ties any way you see fit.  Note that if
          there are no legal actions, which is the case at the
          terminal state, you should return None.
        """
        actions = self.mdp.getPossibleActions(state)

        if len(actions) == 0:
            return None

        max_action = 0
        final_action = ''
        final_qval = 0
        for action in actions:
            final_qval = self.computeQValueFromValues(state,action)
            if final_qval == None:
                print('Hello world')
            if  max_action == 0 or final_qval > max_action:
                max_action = final_qval
                final_action = action

        return final_action

    def getPolicy(self, state):
        return self.computeActionFromValues(state)

    def getAction(self, state):
        "Returns the policy at the state (no exploration)."
        return self.computeActionFromValues(state)

    def getQValue(self, state, action):
        return self.computeQValueFromValues(state, action)
