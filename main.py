import numpy as np
import matplotlib.pyplot as plt

# Class for physics entity (with y pos, velocity, and acceleration)
class Entity:
    def __init__(self, mass, y):
        self.accel = 0
        self.velocity = 0
        self.y = y
        self.mass = mass # kg

    def update(self, thrust):
        self.accel = -9.81 + thrust / self.mass
        self.velocity += self.accel
        self.y += self.velocity

# PID controller
class PID:
    def __init__(self, kp, kd, ki, goal):
        # gains
        self.kp = kp
        self.kd = kd
        self.ki = ki
        # goal
        self.goal = goal
        # errors
        self.integral = 0
        self.derivative = 0
        self.proportional = 0
        # previous error (for derivative calculation)
        self.prev_error = 0
    def __call__(self, pos):
        # calculate error between goal and current state
        error = self.goal - pos
        # update errors
        self.proportional = error
        self.derivative = (error - self.prev_error) / dt
        self.integral += error * dt
        # calculate output
        out = self.kp*self.proportional + self.kd*self.derivative + self.ki*self.integral

        self.prev_error = error
        return out

dt = 1
timer = 0
rocket = Entity(1, 0)

# Tuned using the ziegler-nichols method
ku = 2.02
tu = 4
pid = PID(0.6*ku, 0.075*ku*tu, 1.2*ku/tu, 10)

positions = []
times = []

while timer < 100:
    # update the data
    positions.append(rocket.y)
    times.append(timer)

    # update the rocket
    rocket.update(pid(rocket.y))

    # update the timer
    timer += dt

# visualize rocket height
positions = np.array(positions)
times = np.array(times)

plt.plot(times, positions)
plt.show()
