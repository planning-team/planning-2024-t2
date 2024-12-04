import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from utils import *
#import pickle
from mdp import MDP
from vi import VI


data = np.load('data_ps3.npz')
environment_grid = data['environment']


# (row index, colum index). In the image row corresponds to y, and colum to s.
s_ini = (0,0)
goal = (21,17)
epsilon = 0.4 #Propagation probability (see utils)

environment = Environment(environment_grid,s_ini,goal,epsilon)



# Visualization
# ======================================
if 1:
    im = plot_enviroment(environment,x_ini,goal)
    plt.matshow(im)
    plt.show()


# task 1 VI, Gopt
# ======================================
#vi = VI(...) TODO



# task 2 MDP
# ======================================
#mdp = MDP(...) TODO

# Plan visualization
# ======================================
if 0:
    fig = plt.figure()
    imgs = []
    s = s_ini
    environment.reset(s)
    for iters in range(100):
        print('state ',s, ' iters ', iters)
        im = environment.plot_enviroment(s,goal)
        plot = plt.imshow(im)
        imgs.append([plot])
        # TODO, calculate a plan based on the policy calculated in VI or MDP
        #a = vi.policy(s)
        #a = mdp.policy(s)
        # NOTE, ONLY for visualizations, the noise has been reduced, but it should be different than for calculating the MDP
        s, _, safe_propagation, success = environment.step(action_space[a],epsilon=0.001)
        if not safe_propagation:
            print('Collision!!',s)
            break
        print('iters', iters, ' action ', a, ' state ', s)
        if success:
            print('Goal achieved in ', iters)
            im = environment.plot_enviroment(s,goal)
            plot = plt.imshow(im)
            imgs.append([plot])
            break


    ani = animation.ArtistAnimation(fig, imgs, interval=100, blit=True)
    ani.save('plan_vi.mp4')
    #ani.save('plan_mdp.mp4')
    plt.show()

