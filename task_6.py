import numpy as np
import do_mpc
import casadi
from pyminisim.core import Simulation
from pyminisim.world_map import CirclesWorld
from pyminisim.robot import UnicycleRobotModel
from pyminisim.visual import Renderer, CircleDrawing
from typing import Tuple
import time

# Parameters
x0 =                                        # Initial State, [x, y, theta]
dt =                                        # Model step interval, [s]
sim_dt =                                    # Simulator step interval, [s]
horizon =                                   # Number of receding horizon steps, [#]
goal =                                      # Navigation goal, [x, y]
obstacle =                                  # Obstacle coordinates and radius, [x, y, radius]
safe_distance =                             # Minimum distance to obstacle, [m]

# Create your discrete system model
model = do_mpc.model.Model("discrete")

# State variables
pose_x = model.set_variable(var_type='_x', var_name='pose_x')
pose_y =
pose_theta =

# Control input variables
u_v = model.set_variable(var_type='_u', var_name='u_v')
u_omega =

# Define system equations
model.set_rhs('pose_x', pose_x + u_v * casadi.cos(pose_theta) * dt)
model.set_rhs()
model.set_rhs()

# Setup and obstacle distance function the cost
obstacle_distance = ()
model.set_expression("obstacle_distance", obstacle_distance)

cost = ()
model.set_expression('cost', cost)
# Setup the model
model.setup()

# Create your controller
mpc = do_mpc.controller.MPC(model)

# Set parameters
setup_mpc = {
            'n_robust': 0,
            'n_horizon': horizon,
            't_step': dt,
            'state_discretization': 'discrete',
            'store_full_solution': True,
            "nlpsol_opts": {"ipopt.print_level": 0,
                            "ipopt.sb": "yes",
                            "print_time": 0}
        }
mpc.set_param(**setup_mpc)

# Define cost-function
mterm = model.aux['cost']  # terminal cost
lterm = model.aux['cost']  # stage cost
mpc.set_objective(mterm=mterm, lterm=lterm)

# Set constraints for state and control variables
mpc.bounds['lower', '_x', 'pose_theta'] =
mpc.bounds['upper', '_x', 'pose_theta'] =
mpc.bounds['lower', '_u', 'u_v'] = 
mpc.bounds['upper', '_u', 'u_v'] = 
mpc.bounds['lower', '_u', 'u_omega'] = 
mpc.bounds['upper', '_u', 'u_omega'] = 

# Define rterm (penalty for control input)
mpc.set_rterm(u_v=1e-4)

# Set nonlinear constraints
mpc.set_nl_cons('obstacle', -model.aux['obstacle_distance'], 0)

# Setup controller
mpc.setup()


# Setup pyminisim simulator
def create_sim(
        obstacle: np.ndarray,
        sim_dt: float
) -> Tuple[Simulation, Renderer]:
    robot_model = UnicycleRobotModel(initial_pose=np.array([0., 0., 0.]),
                                     initial_control=np.array([0., np.deg2rad(0.)]))
    sensors = []
    sim = Simulation(sim_dt=sim_dt,
                     world_map=CirclesWorld(circles=obstacle.reshape(1, -1)),
                     robot_model=robot_model,
                     pedestrians_model=None,
                     sensors=sensors,
                     rt_factor=1.)
    renderer = Renderer(simulation=sim,
                        resolution=80.0,
                        screen_size=(500, 500),
                        camera="robot")
    return sim, renderer


def main():
    sim, renderer = create_sim(
        obstacle=obstacle,
        sim_dt=sim_dt
    )
    renderer.initialize()

    renderer.draw("goal", CircleDrawing(goal[:2], 0.1, (255, 0, 0), 0))

    running = True
    sim.step()  # First step can take some time due to Numba compilation

    u_pred = np.array([0., 0.])
    hold_time = sim.sim_dt
    renderer.render()
    mpc.set_initial_guess()

    while running:
        renderer.render()

        if hold_time >= dt:
            x_current = sim.current_state.world.robot.pose
            u_pred = # Fill here
            u_pred = u_pred.flatten()
            hold_time = 0.

            if np.linalg.norm(x_current[:2] - goal) < 0.1:
                running = False
                time.sleep(3)

        sim.step() # Fill here
        sim.world_map.is_occupied(np.array([2.4, 2.4]))
        hold_time += sim.sim_dt

    # Done! Time to quit.
    renderer.close()


if __name__ == "__main__":
    main()
