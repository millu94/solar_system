import numpy as np
import matplotlib.pyplot as plt

# Create a grid of points
x, y, z = np.meshgrid(np.arange(-5, 5, 1),
                      np.arange(-5, 5, 1),
                      np.arange(-5, 5, 1))

# Calculate the vector field
r = np.sqrt(x**2 + y**2 + z**2)
r[r == 0] = 1  # Avoid division by zero
u = x / r**3
v = y / r**3
w = z / r**3

# Plot the vector field
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.quiver(x, y, z, u, v, w, length=0.1, normalize=True)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('Vector Field $\mathbf{r}/r^3$')
plt.show()