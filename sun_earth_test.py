import numpy as np
import math

r = np.array([1, 0])
v = np.array([0, 1])

acceleration = np.array([0, 0])
prev_acceleration = np.array([0, 0])
next_acceleration = np.array([0, 0])

Δt = 0.01
earth_mass = 1
sun_mass = 332946
gravity_constant = 1.18638e-4

# update the postion using the current acceleration and the previous 
# acceleration

def update_position(position, velocity acceleration, prev_acceleration, timestep):
    position = (
        position + (velocity * timestep)
        + (4 * acceleration - prev_acceleration) 
        * (timestep ** 2) / 6
    )
    return position


def calc_acceleration(position):
    r_mag = np.linalg.norm(position)  # Compute magnitude of position vector
    acceleration = -gravity_constant * sun_mass * position / r_mag**3
    return acceleration


def update_velocity(new_a):
    next_velocity = (
        self.velocity + 
        ((2 * new_a) + (5 * self.acceleration) - prev_a)
    )
    return next_velocity

def update_accelerations():
    prev_acceleration = acceleration.copy()
    acceleration


"""
prompt for total time
run_simulation()

loop over total time with a division of timestep
at each timestep:

    update the postion using the current acceleration and the previous
    acceleration

    calculate next accelerations using updated position

    update the velocity using previous, current and next acceleration

    update accelerations for the following timestep


"""

total_time = 1

def run_simulation(position, velocity):

    # calculate initial acceleration
    print(calc_acceleration(position))

    for timestep in range(int(total_time / Δt)):
        update_position(acceleration, velocity, prev_acceleration, timestep)

run_simulation(r, v)
