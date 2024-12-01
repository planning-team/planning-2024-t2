import numpy as np
from utils import *


class VI:
    def __init__(self,
                 env: Environment,
                 goal: tuple):
        """
        env is the grid enviroment, as defined in utils
        goal is the goal state
        """
        self._env = env
        self._goal = goal
        self._G = np.ones(self._env.shape)*1e2
        self._policy = np.zeros(self._env.shape, 'b') #type byte (or numpy.int8)


    def calculate_value_function(self):
        """
        env is the grid enviroment
        goal is the goal state
            
        output:
        G: Optimal cost-to-go
        """
        
        return self._G
        
    def calculate_policy(self):
        """
        G: optimal cot-to-go function (needed to be calcualte in advance)
        
        output:
        policy: a map from each state x to the best action a to execcute
        """    
        return self._policy
        
    def policy(self,state:tuple) -> int:
        """
        returns the action according to the policy
        """
        return self._policy[state]

