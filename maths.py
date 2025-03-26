import numpy as np
import matplotlib.pyplot as plt

a_x, a_y = 1, 1 

x, y = np.meshgrid(np.linspace(-5, 5, 10), np.linspace(-5, 5, 10))

plt.quiver(x, y, a_x, a_y, color='blue', scale=10)
plt.title("Constant Vector Field in the x-y Plane")
plt.xlabel("x")
plt.ylabel("y")
plt.grid()
plt.show()