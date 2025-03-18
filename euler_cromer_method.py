import numpy as np
import matplotlib.pyplot as plt

# Parameters
m = 1.0       # Mass (kg)
k = 1.0       # Spring constant (N/m)
x0 = 1.0      # Initial position (m)
v0 = 0.0      # Initial velocity (m/s)
dt = 0.01     # Time step (s)
T = 20.0      # Total simulation time (s)

# Force function (Hooke's law)
def force(x):
    return -k * x

# Euler-Cromer method
def euler_cromer(x0, v0, dt, T, force):
    # Initialize arrays to store results
    num_steps = int(T / dt)
    t_values = np.linspace(0, T, num_steps)
    x_values = np.zeros(num_steps)
    v_values = np.zeros(num_steps)
    
    # Set initial conditions
    x_values[0] = x0
    v_values[0] = v0
    
    # Perform Euler-Cromer integration
    for i in range(1, num_steps):
        # Update velocity
        v_values[i] = v_values[i-1] + force(x_values[i-1]) / m * dt
        # Update position
        x_values[i] = x_values[i-1] + v_values[i] * dt
    
    return t_values, x_values, v_values

# Run simulation
t, x, v = euler_cromer(x0, v0, dt, T, force)

# Plot results
plt.figure(figsize=(10, 6))
plt.plot(t, x, label="Position (x)")
plt.plot(t, v, label="Velocity (v)")
plt.xlabel("Time (s)")
plt.ylabel("Position (m) / Velocity (m/s)")
plt.title("Euler-Cromer Method: Mass-Spring System")
plt.legend()
plt.grid()
plt.show()