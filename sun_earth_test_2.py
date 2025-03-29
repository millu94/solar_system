import numpy as np
import matplotlib.pyplot as plt

earth_mass = 1
sun_mass = 332946
gravity_constant = 1.18638e-4



Δt = 0.01

def update_position(position, velocity, acceleration, previous_acceleration, timestep):
    next_position = (
        position + (velocity * timestep) +
        (4 * acceleration - previous_acceleration) *
        timestep ** 2  / 6
    )
    return next_position

def calculate_acceleration(next_position):
    r_mag = np.linalg.norm(next_position)
    next_acceleration = -gravity_constant * sun_mass * next_position / r_mag ** 3
    # print(f"new_acceleration: {next_acceleration}")
    
    return next_acceleration

def update_velocity(
        current_velocity, next_acceleration, current_acceleration, 
        previous_acceleration, timestep):
    next_velocity = (
        current_velocity + (
            2 * next_acceleration 
            + 5 * current_acceleration 
            - previous_acceleration
        ) * timestep / 6
    )
    return next_velocity

def update_accelerations(current_acceleration, next_acceleration):
    prev_a = current_acceleration.copy()
    new_a = next_acceleration
    #print(f"prev_a: {prev_a}")
    #print(f"new_a: {new_a}")
    return prev_a, new_a

def run_simulation():

    r = np.array([1, 0])
    v = np.array([0, 6.28])
    a = np.array([0, 0])

    positions = [r.copy()]
   
    # print(f"current acceleration: {a}")
    next_acceleration = calculate_acceleration(r)
    previous_acceleration, current_acceleration = update_accelerations(a, next_acceleration)
    # print(f"initial previous_acceleration: {previous_acceleration}, initial current_acceleration: {current_acceleration}")

    for timestep in range(50):

        print(f"loop number: {timestep}")

        # update position takes the current position, velocity, acceleration; previous acceleration; timestep
        # returns only the next position
        print(f"previous acceleration: {previous_acceleration}")
        r = update_position(r, v, current_acceleration, previous_acceleration, Δt)
        print(f"updated_position: {r}")
        positions.append(r)

        # calculate acceleration takes the next position and only returns the next acceleration
        a = calculate_acceleration(r)

        # update velocity takes current velocity, next acceleration, current acceleration, previous acceleration 
        # and timestep, only returns next velocity
        v = update_velocity(v, a, current_acceleration, previous_acceleration, Δt)

        previous_acceleration, current_acceleration = update_accelerations(current_acceleration, a)

    # print("now running simulation")
    print(positions)
    return np.array(positions)

sun_earth = run_simulation()

# Extract x and y coordinates
x = sun_earth[:, 0]
y = sun_earth[:, 1]

# Plot the orbit
plt.figure(figsize=(8, 8))
plt.plot(x, y, 'b-', label="Planet Orbit")
plt.scatter([0], [0], color='yellow', s=300, label="Sun")  # Sun at (0,0)
plt.xlabel("X Position (AU)")
plt.ylabel("Y Position (AU)")
plt.title("Planetary Orbit Simulation")
plt.grid(True)
plt.axis('equal')  # Ensures the orbit isn't distorted
plt.legend()
plt.show()