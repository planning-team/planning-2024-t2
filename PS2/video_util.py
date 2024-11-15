import numpy as np
import matplotlib.pyplot as plt
import cv2

from typing import List, Optional
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from environment import ManipulatorEnv, State


def animate_plan(env: ManipulatorEnv, plan: List[State], video_output_file: Optional[str] = "solve_4R.mp4"):
    """
    Visualizes the plan with pyplot and, optionally, saves it to the video file.

    :param env: Manipulator environment
    :param plan: Plan - sequence of states
    :param video_output_file: If not None, saves animation to this file. Suggested extension is .mp4.
    """
    video_writer = None
    fig = plt.figure()
    ax = plt.gca()
    ax.set_xlim([-4, 4])
    ax.set_ylim([-2.5, 2.5])
    for i, state in enumerate(plan):
        env.state = state
        env.render(plt_show=False)

        canvas = FigureCanvas(fig)
        canvas.draw()
        mat = np.array(canvas.renderer._renderer)
        mat = cv2.cvtColor(mat, cv2.COLOR_RGB2BGR)
        if video_writer is not None:
            video_writer.write(mat)
        elif video_output_file is not None:
            video_writer = cv2.VideoWriter(video_output_file, cv2.VideoWriter_fourcc(*'mp4v'), 10,
                                           (mat.shape[1], mat.shape[0]))
            video_writer.write(mat)

        if i != len(plan) - 1:
            plt.pause(0.05)
            plt.clf()
        else:
            if video_writer is not None:
                video_writer.release()
            plt.show()
