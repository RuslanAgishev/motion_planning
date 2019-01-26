import numpy as np
import pylab as plt
import skfmm

X, Y = np.meshgrid(np.linspace(-1,1,501), np.linspace(-1,1,501))
phi = (X+0.8)**2+(Y+0.8)**2 - 0.01
speed = 1+X**2+Y**2

plt.subplot(221)
plt.title("Zero-contour of phi")
plt.contour(X, Y, phi, [0], colors='black', linewidths=(3))
plt.gca().set_aspect(1)
plt.xticks([]); plt.yticks([])

plt.subplot(222)
plt.title("Distance")
plt.contour(X, Y, phi, [0], colors='black', linewidths=(3))
plt.contour(X, Y, skfmm.distance(phi, dx=2.0/500), 15)
plt.gca().set_aspect(1)
plt.xticks([]); plt.yticks([])

plt.subplot(223)
plt.title("Distance with x- \nand y- directions periodic")
plt.contour(X, Y, phi, [0], colors='black', linewidths=(3))
plt.contour(X, Y, skfmm.distance(phi, dx=2.0/500, periodic=True), 15)
plt.gca().set_aspect(1)
plt.xticks([]); plt.yticks([])

plt.subplot(224)
plt.title("Travel time with y- \ndirection periodic ")
plt.contour(X, Y, phi, [0], colors='black', linewidths=(3))
plt.contour(X, Y, skfmm.travel_time(phi, speed, dx=2.0/500, periodic=(1,0)), 15)
plt.gca().set_aspect(1)
plt.xticks([]); plt.yticks([])

plt.show()