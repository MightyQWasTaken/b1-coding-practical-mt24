from __future__ import annotations
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt

from terrain import generate_reference_and_limits
import control

# Using Python's csv package instead of using pandas (I learnt pandas over the summer and would like to learn another way of doing this.)
import csv

class Submarine:
    def __init__(self):

        self.mass = 1
        self.drag = 0.1
        self.actuator_gain = 1

        self.dt = 1 # Time step for discrete time simulation

        self.pos_x = 0
        self.pos_y = 0
        self.vel_x = 1 # Constant velocity in x direction
        self.vel_y = 0

        self.pos_y_LastTime = 0 # For Controller


    def transition(self, action: float, disturbance: float):
        self.pos_y_LastTime = self.pos_y
        self.pos_x += self.vel_x * self.dt
        self.pos_y += self.vel_y * self.dt

        force_y = -self.drag * self.vel_y + self.actuator_gain * (action + disturbance)
        acc_y = force_y / self.mass
        self.vel_y += acc_y * self.dt

    def get_depth(self) -> float:
        return self.pos_y
    
    def get_depth_LastTime(self) -> float:
        return self.pos_y_LastTime

    def get_position(self) -> tuple:
        return self.pos_x, self.pos_y
    
    def reset_state(self):
        self.pos_x = 0
        self.pos_y = 0
        self.vel_x = 1
        self.vel_y = 0
    
class Trajectory:
    def __init__(self, position: np.ndarray):
        self.position = position  
        
    def plot(self):
        plt.plot(self.position[:, 0], self.position[:, 1])
        plt.show()

    def plot_completed_mission(self, mission: Mission):
        x_values = np.arange(len(Mission.reference))
        min_depth = np.min(Mission.cave_depth)
        max_height = np.max(Mission.cave_height)

        plt.fill_between(x_values, Mission.cave_height, Mission.cave_depth, color='blue', alpha=0.3)
        plt.fill_between(x_values, Mission.cave_depth, min_depth*np.ones(len(x_values)), 
                         color='saddlebrown', alpha=0.3)
        plt.fill_between(x_values, max_height*np.ones(len(x_values)), Mission.cave_height, 
                         color='saddlebrown', alpha=0.3)
        plt.plot(self.position[:, 0], self.position[:, 1], label='Trajectory')
        plt.plot(Mission.reference, 'r', linestyle='--', label='Reference')
        plt.legend(loc='upper right')
        plt.show()

@dataclass
class Mission:
    def __init__(self):
        # Initialize attributes as empty lists
        self.reference = []
        self.cave_height = []
        self.cave_depth = []


    @classmethod
    def random_mission(cls, duration: int, scale: float):
        (reference, cave_height, cave_depth) = generate_reference_and_limits(duration, scale)
        return cls(reference, cave_height, cave_depth)

    @classmethod
    def from_csv(self, file_name: str):
        self.reference = []
        self.cave_height = []
        self.cave_depth = []

        with open(file_name, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                # Append values to the instance's attributes
                self.reference.append(float(row['reference']))
                self.cave_height.append(float(row['cave_height']))
                self.cave_depth.append(float(row['cave_depth']))
                reference = self.reference
                cave_height = self.cave_height
                cave_depth = self.cave_depth

        return reference, cave_height, cave_depth
        


class ClosedLoop:
    def __init__(self, plant: Submarine):
        self.plant = plant

    def simulate(self,  mission: Mission, disturbances: np.ndarray, gain:float) -> Trajectory:

        T = len(Mission.reference)
        if len(disturbances) < T:
            raise ValueError("Disturbances must be at least as long as mission duration")
        
        positions = np.zeros((T, 2))
        actions = np.zeros(T)
        self.plant.reset_state()

        for t in range(T):
            positions[t] = self.plant.get_position()
            observation_t = self.plant.get_depth()
            observation_LastTime = self.plant.get_depth_LastTime()
            actions[t] = control.pdcontroller(observation_t, observation_LastTime, t, Mission.reference) * gain
            self.plant.transition(actions[t], disturbances[t])

        return Trajectory(positions)
        
    def simulate_with_random_disturbances(self, mission: Mission, gain:float, variance: float = 0.5) -> Trajectory:
        disturbances = np.random.normal(0, variance, len(Mission.reference))
        # disturbances = np.zeros(len(Mission.reference)) #|Used for initial tuning of controller
        return self.simulate(mission, disturbances, gain)
