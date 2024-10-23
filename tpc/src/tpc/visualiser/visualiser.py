from typing import List, Tuple, Dict, Union, Callable, Optional
from collections import deque

import imageio
from matplotlib.colors import Colormap
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.gridspec import GridSpec
from matplotlib.artist import Artist
from matplotlib.axes import Axes, Subplot
from matplotlib.patches import Arc
plt.style.use('dark_background')
import numpy as np

class GymnasiumPendulumVisualizer:

    def __init__(self,
                 # state: np.ndarray,
                 # state_estimate: np.ndarray,
                 observation: np.ndarray,
                 observation_estimate: np.ndarray, 
                 # state_error: np.ndarray,
                 # observation_error: np.ndarray,
                 agent_names: List[str],
                 n_frames: Optional[int],
                 plot_theta: bool = True,
                 name: str = "GymnasiumPendulumVisualizer",
        ):
        """
        observation: np.ndarray
            The current observation of the pendulum from the simulator
        observation_estimate: np.ndarray
            The estimated observation of the pendulum from each agent.
            shape: (n_agents, observation_dim)
        """


        if observation_estimate.ndim != 2:
            raise ValueError("observation_estimate must be a 2D array.")
        n_agents: int = len(agent_names)
        if n_agents > 2:
            raise ValueError("Visualizer can only support up to 2 agents.")

        
        cmap_simulator: Callable = lambda: "yellow"
        cmap_agent: Colormap = plt.get_cmap('Set2', n_agents)

        # self.fig, self.axs = plt.subplots(2,2)
        self.plot_theta = plot_theta
        if not self.plot_theta:
            self.fig, self.ax_pendulum = plt.subplots(1,1)
        else:
            # self.fig, self.axs = plt.subplots(2,1)
            self.fig: Figure = plt.figure(figsize=(10, 5))
            gs: GridSpec = self.fig.add_gridspec(4, 2)
            self.ax_pendulum: Axes = self.fig.add_subplot(gs[0:2, :])
            self.ax_theta: Axes = self.fig.add_subplot(gs[2, :])
            self.ax_theta_error: Axes = self.fig.add_subplot(gs[3, :])

            deq_len: int = 100

            self.ax_theta_error.set_xlim([0, deq_len])
            self.ax_theta_error.set_ylim([-np.pi, np.pi])

        self.ax_pendulum.set_xlim([-2.1, 2.1])
        self.ax_pendulum.set_ylim([-2.1, 2.1])

        # self.frames = []
        # width, height = self.fig.get_size_inches()
        width, height = self.fig.canvas.get_width_height()
        self.frames: np.ndarray = np.zeros((n_frames, width, height, 3), dtype=np.uint8)
        self.frame_idx: int = 0

        # self.artists: List[Tuple[plt.Artist, plt.Axes]] = []
        
        # Plot the current position
        self.scat_sim_observation = self.ax_pendulum.scatter(
            x=-np.sin(observation[0]), y=np.cos(observation[0]), 
            c='yellow', marker='o', s=100,
            # label="Simulated Observation",
            animated=False
        )
        # Line representing the pendulum rod
        self.pendulum_line, = self.ax_pendulum.plot(
            [0, -np.sin(observation[0])],
            [0, np.cos(observation[0])],
            # 'b-', linewidth=2
            c=cmap_simulator(), linewidth=2, linestyle='-',
            label="Ground Truth"
        )
        # self.artists.append((self.scat_sim_observation, self.ax_pendulum))

        # self.scat_agent_observation = self.ax_pendulum.scatter(
        self.scat_agent_observation: List[plt.Artist] = []
        self.pendulum_line_estimate: List[plt.Artist] = []
        for agent in range(n_agents):
            self.scat_agent_observation.append(
                self.ax_pendulum.scatter(
                    x=-np.sin(observation_estimate[agent,0]),
                    y=np.cos(observation_estimate[agent,1]), 
                    # c='green', marker='x', s=100,
                    c=cmap_agent(agent), marker='x', s=100,
                    animated=False
                )
            )

            # Line representing the estimated pendulum rod
            pendulum_line_estimate, = self.ax_pendulum.plot(
                [0, -np.sin(observation_estimate[agent, 0])],
                [0, np.cos(observation_estimate[agent, 1])],
                c=cmap_agent(agent), linewidth=2, linestyle='-',
                label=agent_names[agent],
            )
            self.pendulum_line_estimate.append(pendulum_line_estimate)
        

        if self.plot_theta:

            self.sim_theta_history: deque = deque(maxlen=deq_len)
            # self.sim_theta_dot_history: deque = deque(maxlen=deq_len)
            for _ in range(deq_len):
                self.sim_theta_history.append(0)
                # self.sim_theta_dot_history.append(0)
            self.sim_theta_line, = self.ax_theta.plot(
                np.arange(deq_len), np.zeros(deq_len),
                # 'y-', linewidth=2, label="Simulated"
                c=cmap_simulator(), linewidth=2, linestyle='-',
            )
            # self.sim_theta_dot_line, = self.ax_theta.plot(
            #     np.arange(deq_len), np.zeros(deq_len),
            #     'c-', linewidth=2, label="Simulated"
            # )


            self.agent_theta_history: List[deque] = [deque(maxlen=deq_len) for _ in range(n_agents)]
            # self.agent_theta_dot_history: List[deque] = [deque(maxlen=deq_len) for _ in range(n_agents)]

            self.ax_theta.set_xlim([0, deq_len])
            # self.ax_theta.set_ylim([-np.pi*1.5, np.pi*1.5])
            self.ax_theta.set_ylim([-0.1, 2*np.pi*1.3])
            self.ax_theta.set_xlabel("Time")
            self.ax_theta.set_ylabel("Theta")
            self.ax_theta.set_title("Theta vs Time")

            self.agent_theta_lines: List[plt.Artist] = []
            # self.agent_theta_dot_lines: List[plt.Artist] = []
            for agent in range(n_agents):
                for _ in range(deq_len):
                    self.agent_theta_history[agent].append(0)
                    # self.agent_theta_dot_history[agent].append(0)
                theta_line, = self.ax_theta.plot(
                    np.arange(deq_len), np.zeros(deq_len),
                    c=cmap_agent(agent), linewidth=2, linestyle='-',
                    label=f"Agent {agent}"
                )
                # theta_dot_line, = self.ax_theta.plot(
                #     np.arange(deq_len), np.zeros(deq_len),
                #     'b-', linewidth=2, label=f"Agent {agent}"
                # )
                self.agent_theta_lines.append(theta_line)
                # self.agent_theta_dot_lines.append(theta_dot_line)

                self.ax_theta_error.plot(
                    np.arange(deq_len), np.zeros(deq_len),
                    c=cmap_agent(agent), linewidth=2, linestyle='-',
                    label=f"Agent {agent}"
                )


            # self.ax_theta.legend()
            self.ax_pendulum.legend()

        plt.ion()


    @staticmethod
    def theta_to_xy(theta: float, length: float = 1.0) -> Tuple[float, float]:
        """
        Convert the angle theta to x-y coordinates
        """
        return -length * np.sin(theta), length * np.cos(theta)

    def step(self, 
             # state: np.ndarray,
             # state_estimate: np.ndarray,
             observation: np.ndarray,
             observation_estimate: np.ndarray,
             # state_error: np.ndarray,
             # observation_error: np.ndarray,
             # positions_x: List[float], positions_y: List[float]
             ):

        ### Update the artists
        gt_x, gt_y = self.theta_to_xy(observation[0])
        # gt_x, gt_y = -observation[1], observation[0]
        self.scat_sim_observation.set_offsets(np.c_[gt_x, gt_y])
        self.pendulum_line.set_data([0, gt_x], [0, gt_y])
        if self.plot_theta:
            self.sim_theta_history.append(observation[0])
            # self.sim_theta_dot_history.append(observation[1])
            self.sim_theta_line.set_ydata(list(self.sim_theta_history))
            # self.sim_theta_dot_line.set_ydata(list(self.sim_theta_dot_history))

        # Update the pendulum line position
        for agent in range(len(self.scat_agent_observation)):
            estimated_x, estimated_y = self.theta_to_xy(observation_estimate[agent,0])
            # estimated_x, estimated_y = -observation_estimate[agent,1], observation_estimate[agent,0]
            # estimated_x, estimated_y = self.theta_to_xy(observation[0])
            self.scat_agent_observation[agent].set_offsets(np.c_[estimated_x, estimated_y])
            self.pendulum_line_estimate[agent].set_data([0, estimated_x], [0, estimated_y])

            if self.plot_theta:
                self.agent_theta_history[agent].append(observation_estimate[agent, 0])
                # self.agent_theta_dot_history[agent].append(observation_estimate[agent, 1])
                self.agent_theta_lines[agent].set_ydata(list(self.agent_theta_history[agent]))
                # self.agent_theta_dot_lines[agent].set_ydata(list(self.agent_theta_dot_history[agent]))

                self.ax_theta_error.lines[agent].set_ydata(
                    np.array(self.agent_theta_history[agent]) - np.array(self.sim_theta_history)
                )


        # Redraw the figure
        self.fig.canvas.draw_idle()
        plt.pause(0.0001)

        # Capture the current frame
        # self.fig.canvas.draw()
        # image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype='uint8')
        # image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
        # # Resize to fit the frames array
        # # self.frames.append(image)
        # self.frames[self.frame_idx] = image
        # self.frame_idx += 1

    def save_animation(self, filename='pendulum_animation.mp4', fps=30):
        """Save the recorded frames as a video file."""
        pass
        # writer = imageio.get_writer(filename, fps=fps)
        # for frame in self.frames:
        #     writer.append_data(frame)
        # writer.close()
        # print(f"Animation saved as {filename}")
