import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# this are the set of possible actions admitted in this problem
action_space = []
action_space.append((-1,0))
action_space.append((0,-1))
action_space.append((1,0))
action_space.append((0,1))

class Environment:
    def __init__(self,
                 env: np.ndarray,
                 initial_state: tuple,
                 goal: tuple,
                 epsilon: float = 0.4
                 ):
        """
        grid of the environment
        goal is the goal state
        epsilon is the noise in the state space after the transition function (0 is no noise)
        """
        self._env = env
        self._epsilon = epsilon
        self._state = initial_state
        self._goal = goal

    @property
    def shape(self):
        return self._env.shape

    def reset(self, initial_state: tuple):
        self._state =  initial_state
        return self._state
        
    def plot_enviroment(self, s, goal):
        """
        env is the grid enviroment
        s is the state 
        """
        dims = self._env.shape    
        current_env = np.copy(self._env)
        # plot agent
        current_env[s] = 1.0 #yellow
        # plot goal
        current_env[goal] = 0.3
        return current_env


    def state_consistency_check(self,s):
        """Checks wether or not the proposed state is a valid state, i.e. is in colision or our of bounds"""
        # check for collision
        if s[0] < 0 or s[1] < 0 or s[0] >= self._env.shape[0] or s[1] >= self._env.shape[1] :
            #print('out of bonds')
            return False
        if self._env[s] >= 1.0-1e-4:
            #print('Obstacle')
            return False
        return True


    def transition_function(self,s,a):
        """Transition function for states in this problem
        s: current state, this is a tuple (i,j)
        a: current action, this is a tuple (i,j)
        
        Output:
        new state
        True if correctly propagated
        False if this action can't be executed
        """
        snew = np.array(s) + np.array(a)
        snew = tuple(snew)
        #print('snew',snew)
        if self.state_consistency_check(snew):
            return snew, True
        return s, False


    def probabilistic_transition_function(self,s,a, epsilon = None):
        """Probabilistic Transition function requires:
        s: current state, this is a tuple (i,j)
        a: current action, this is a tuple (i,j)
        epsilon (in [0,1]): This is the probability of carrying out the desired action, in the extreme, 0 indicates a perfect action execution.
        
        Output:
        state_propagated_list: list of propagated states
        prob_list: list of the corresponding state's prob, in the same order
        """
        if epsilon == None:
            epsilon = self._epsilon
        state_propagated_list = []
        prob_list = []
        for action in action_space:
            snew = np.array(s) + np.array(action)
            snew = tuple(snew)
            # This is to ensure that p(Sigma)=1
            prob = epsilon/3
            if action == a:
                prob = 1-epsilon
            state_propagated_list.append(snew)
            prob_list.append(prob)
        # There is no state consistency, it should be done externally when asigning the reward
        return state_propagated_list, prob_list

    def step(self,a, epsilon = None ):
        """Sample Probabilistic Transition function requires:
        a: current action, this is a tuple (i,j)
        epsilon (in [0,1]): This is the probability of carrying out the desired action, in the extreme, 0 indicates a perfect action execution.
        
        Output:
        sampled_state_propagated
        reward: real value of the reward of propagating the state
        safe_propagation: bool TRUE for correct propagation and FALSE for incorrec leading into collision or out of bounds
        success: True if the state is the goal or if the iterations larger than the budget
        """
        if epsilon == None:
            epsilon = self._epsilon
        state_list, prob_list = self.probabilistic_transition_function(self._state,a,epsilon)
        index = np.random.choice(len(prob_list),p=prob_list)
        state = state_list[index]
        safe_propagation = self.state_consistency_check(state_list[index])
        reward = 0.0
        success = False
        if not safe_propagation:
            reward = -1.0
        if state == self._goal:
            success = True
            reward = 1.0
        self._state = state
        return state, reward, safe_propagation, success

