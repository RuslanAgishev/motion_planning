import numpy as np
import pylab as plt
from skfmm import extension_velocities

N     = 150
X, Y  = np.meshgrid(np.linspace(-1, 1, N), np.linspace(-1, 1, N))
r     = 1.75
dx    = 2.0 / (N - 1)
phi   = (X) ** 2 + (Y+1.85) ** 2 - r ** 2
speed = X+1.25
d, f_ext = extension_velocities(phi, speed, dx)


plt.subplot(131)
plt.title("Zero-contour of phi")
plt.contour(X, Y, phi,[0], colors='black', linewidths=(3))
plt.gca().set_aspect(1)

plt.subplot(132)
plt.title("Interface velocity")
plt.contour(X, Y, phi,[0], colors='black', linewidths=(3))
plt.contourf(X, Y, speed)
plt.gca().set_aspect(1)

plt.subplot(133)
plt.title("Extension velocities")
plt.contour(X, Y, phi,[0], colors='black', linewidths=(3))
plt.contourf(X, Y, f_ext)
plt.gca().set_aspect(1)

plt.show()