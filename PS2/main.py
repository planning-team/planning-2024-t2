import numpy as np
import pickle
from environment import State, ManipulatorEnv
from rrt import RRTPlanner
from video_util import animate_plan


# You are free to change any interfaces for your needs.


def main():
    with open("data.pickle", "rb") as handle:
        data = pickle.load(handle)

    start_state = State(np.array(data["start_state"]))
    goal_state = State(np.array(data["goal_state"]))
    env = ManipulatorEnv(obstacles=np.array(data["obstacles"]),
                         initial_state=start_state,
                         collision_threshold=data["collision_threshold"])

    planner = RRTPlanner(env)

    plan = planner.plan(start_state, goal_state)
    print("RRT planner has finished successfully")

    animate_plan(env, plan)


if __name__ == '__main__':
    main()
