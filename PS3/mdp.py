import numpy as np
from utils import *


class MDP:

    def __init__(self,
                 env: Environment,
                 goal: tuple,
                 gamma: float = 0.99):
        """
        env is the grid enviroment
        goal is the goal state
        gamma is the discount factor
        """
        self._env = env
        self._goal = goal
        self._gamma = gamma
        self._V = np.zeros(env.shape)
        self._policy = np.zeros(self._env.shape, 'b') #type byte (or numpy.int8)

    def calculate_value_function(self):
        """
        This function uses the Value Iteration algorithm to fill in the
        optimal value function
        """
        return self._V

    def calculate_policy(self):
        """
        Only to be run AFTER Vopt has been calculated.
        
        output:
        policy: a map from each state s to the greedy best action a to execute
        """
        return self._policy

    def policy(self,state:tuple) -> int:
        """
        returns the action according to the policy
        """
        return self._policy[state]

