from typing import List, Tuple, Dict, Union, Callable
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
                 plot_theta: bool = True,
        ):
        """
        observation: np.ndarray
            The current observation of the pendulum from the simulator
        observation_estimate: np.ndarray
            The estimated observation of the pendulum from each agent.
            shape: (n_agents, observation_dim)
        """

        from matplotlib.colors import Colormap
        # colors: Dict[str, Union[str, Colormap]] = {
        #     "simulator": "yellow",
        #     "agent": agent_cmap,
        # }

        if observation_estimate.ndim != 2:
            raise ValueError("observation_estimate must be a 2D array.")
        n_agents: int = observation_estimate.shape[0]
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
            gs: GridSpec = self.fig.add_gridspec(3, 2)
            self.ax_pendulum: Axes = self.fig.add_subplot(gs[0:2, :])
            self.ax_theta: Axes = self.fig.add_subplot(gs[2, :])

        # self.artists: List[Tuple[plt.Artist, plt.Axes]] = []
        
        # Plot the current position
        self.scat_sim_observation = self.ax_pendulum.scatter(
            x=-np.sin(observation[0]), y=np.cos(observation[0]), 
            c='yellow', marker='o', s=100,
            label="Simulated Observation",
            animated=False
        )
        # Line representing the pendulum rod
        self.pendulum_line, = self.ax_pendulum.plot(
            [0, -np.sin(observation[0])],
            [0, np.cos(observation[0])],
            # 'b-', linewidth=2
            c=cmap_simulator(), linewidth=2, linestyle='-'
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
                    label="Estimated Observation",
                    animated=False
                )
            )

            # Line representing the estimated pendulum rod
            pendulum_line_estimate, = self.ax_pendulum.plot(
                [0, -np.sin(observation_estimate[agent, 0])],
                [0, np.cos(observation_estimate[agent, 1])],
                c=cmap_agent(agent), linewidth=2, linestyle='-'
            )
            self.pendulum_line_estimate.append(pendulum_line_estimate)
        
        self.ax_pendulum.set_xlim([-2.1, 2.1])
        self.ax_pendulum.set_ylim([-2.1, 2.1])

        if self.plot_theta:
            from collections import deque
            deq_len: int = 100

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
            self.ax_theta.set_ylim([-np.pi*1.5, np.pi*1.5])
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
            # self.ax_theta.legend()

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
            # estimated_x, estimated_y = self.theta_to_xy(observation[0])
            self.scat_agent_observation[agent].set_offsets(np.c_[estimated_x, estimated_y])
            self.pendulum_line_estimate[agent].set_data([0, estimated_x], [0, estimated_y])

            if self.plot_theta:
                self.agent_theta_history[agent].append(observation_estimate[agent, 0])
                # self.agent_theta_dot_history[agent].append(observation_estimate[agent, 1])
                self.agent_theta_lines[agent].set_ydata(list(self.agent_theta_history[agent]))
                # self.agent_theta_dot_lines[agent].set_ydata(list(self.agent_theta_dot_history[agent]))


        # Redraw the figure
        self.fig.canvas.draw_idle()
        plt.pause(0.0001)
        # plt.pause(0.75)
