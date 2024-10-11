from typing import List, Tuple
import matplotlib.pyplot as plt
plt.style.use('dark_background')
import numpy as np

class AbstractVisualizer:

    def __init__(self,
                 state: np.ndarray,
                 state_estimate: np.ndarray,
                 observation: np.ndarray,
                 observation_estimate: np.ndarray,
                 state_error: np.ndarray,
                 observation_error: np.ndarray,
        ):
        
        # self.fig, self.axs = plt.subplots(1,2)
        self.fig, self.axs = plt.subplots(1,1)
        if not isinstance(self.axs, np.ndarray):
            self.axs = np.array([self.axs])
        self.axs = self.axs.flatten()
        self.artists: List[Tuple[plt.Artist, plt.Axes]] = []
        
        # Plot the current position
        self.scat_pos = self.axs[0].scatter(
            x=pos_x, y=pos_y, 
            c='yellow', marker='x',
            label="current_pos",
            animated=False
        )
        self.artists.append((self.scat_pos, self.axs[0]))
        
        # Plot the target position
        self.axs[0].scatter(
            x=target_x, y=target_y,
            marker='o', edgecolors='r',
            s=100, facecolors='none',
            animated=False
        )
        self.axs[0].set_xlim([-1.1, 1.1])
        self.axs[0].set_ylim([-1.1, 1.1])
        
        # Plot the robot links
        self.ln_links, = self.axs[0].plot(
            positions_x, positions_y,
            'o-', c='white', animated=False
        )
        self.artists.append((self.ln_links, self.axs[0]))
        
        plt.ion()

    def step(self, j: int, theta: np.ndarray,
             pos_x: np.ndarray, pos_y: np.ndarray,
             positions_x: List[float], positions_y: List[float]):
    
        # Update the artists
        self.scat_pos.set_offsets(np.c_[pos_x, pos_y])
        self.ln_links.set_data(positions_x, positions_y)
    
        # Redraw the figure
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

class GymnasiumPendulumVisualizer:

    def __init__(self,
                 # state: np.ndarray,
                 # state_estimate: np.ndarray,
                 observation: np.ndarray,
                 observation_estimate: np.ndarray,
                 # state_error: np.ndarray,
                 # observation_error: np.ndarray,
        ):
        
        # self.fig, self.axs = plt.subplots(1,2)
        # self.fig, self.axs = plt.subplots(2,2)
        self.fig, self.axs = plt.subplots(1,1)
        if not isinstance(self.axs, np.ndarray):
            self.axs = np.array([self.axs])
        self.axs = self.axs.flatten()
        self.artists: List[Tuple[plt.Artist, plt.Axes]] = []
        
        # Plot the current position
        self.scat_sim_observation = self.axs[0].scatter(
            x=observation[0], y=observation[1], 
            c='yellow', marker='o', s=100,
            label="Simulated Observation",
            animated=False
        )
        self.artists.append((self.scat_sim_observation, self.axs[0]))

        self.scat_agent_observation = self.axs[0].scatter(
            x=observation_estimate[0], y=observation_estimate[1], 
            c='green', marker='x', s=100,
            label="Estimated Observation",
            animated=False
        )
        
        # Plot the target position
        # self.axs[0].scatter(
        #     x=target_x, y=target_y,
        #     marker='o', edgecolors='r',
        #     s=100, facecolors='none',
        #     animated=False
        # )
        self.axs[0].set_xlim([-2.1, 2.1])
        self.axs[0].set_ylim([-2.1, 2.1])
        
        # # Plot the robot links
        # self.ln_links, = self.axs[0].plot(
        #     positions_x, positions_y,
        #     'o-', c='white', animated=False
        # )
        # self.artists.append((self.ln_links, self.axs[0]))
        
        plt.ion()

    def step(self, 
             # state: np.ndarray,
             # state_estimate: np.ndarray,
             observation: np.ndarray,
             observation_estimate: np.ndarray,
             # state_error: np.ndarray,
             # observation_error: np.ndarray,
             # positions_x: List[float], positions_y: List[float]
             ):
    
        # Update the artists
        # self.scat_pos.set_offsets(np.c_[pos_x, pos_y])
        # self.ln_links.set_data(positions_x, positions_y)
        self.scat_sim_observation.set_offsets(np.c_[observation[0], observation[1]])
        self.scat_agent_observation.set_offsets(np.c_[observation_estimate[0], observation_estimate[1]])
    
        # Redraw the figure
        self.fig.canvas.draw_idle()
        plt.pause(0.001)
