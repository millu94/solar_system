import numpy as np
import matplotlib.pyplot as plt

# Gravitational constant
G = 6.67430e-11  # m^3 kg^-1 s^-2
M_sun = 1.989e30  # kg (mass of the Sun)

# Function to compute acceleration due to gravity
def acceleration(pos):
    r = np.linalg.norm(pos)  # Distance from the Sun
    return -G * M_sun * pos / r**3  # Newton's law of gravitation

# Beeman integration function
def beeman_step(r, v, a_prev, dt):
    """Performs a single Beeman integration step."""
    
    # Predict next position using Beeman's formula
    r_next = r + v * dt + (4 * a_prev - a_prev) * (dt**2) / 6

    # Compute new acceleration based on predicted position
    a_next = acceleration(r_next)

    # Predict next velocity using Beeman's velocity update formula
    v_next = v + (2 * a_next + 5 * a_prev - a_prev) * dt / 6

    return r_next, v_next, a_next

# Simulation parameters
dt = 60 * 60 * 24  # Time step of 1 day
num_steps = 365  # Simulate 1 year

# Initial conditions (Earth around Sun)
r = np.array([1.496e11, 0])  # 1 AU from Sun
v = np.array([0, 29780])  # Approx Earth's velocity in m/s
a = acceleration(r)

# Arrays to store results
positions = [r]

# Run the simulation
for _ in range(num_steps):
    r, v, a = beeman_step(r, v, a, dt)
    positions.append(r)

# Convert positions to numpy array for plotting
positions = np.array(positions)

# Plot the orbital path
plt.figure(figsize=(6, 6))
plt.plot(positions[:, 0], positions[:, 1], label="Earth's Orbit")
plt.scatter(0, 0, color='orange', s=200, label="Sun")  # Sun at origin
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.legend()
plt.title("Earth Orbit Simulated with Beeman Algorithm")
plt.grid()
plt.axis("equal")
plt.show()
