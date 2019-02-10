import numpy as np
import time
from scipy.integrate import odeint
from math import *

def MassSpringDamper(state,t,F, mode='critically_damped'):
	x = state[0]
	xd = state[1]
	if mode=='oscillations':
		m = 1.0; k = 2; b = 0 # undapmped: oscillations
	elif mode=='undapmped':
		m = 1.0; k = 2; b = 2*sqrt(m*k)-2 # underdamped
	elif mode=='overdamped':
		m = 1.0; k = 2; b = 2*sqrt(m*k)+2 # overdamped
	else:
		m = 1.0; k = 2; b = 2*sqrt(m*k) # critically damped
	xdd = -(b/m)*xd - (k/m)*x + F/m
	return [xd, xdd]


y0 = [np.pi - 0.1, 0.0]
M = 10
t = np.linspace(0, 10, 101)
mode = 'overdamped'
sol = odeint(MassSpringDamper, y0, t, args=(M,mode,))




import matplotlib.pyplot as plt
plt.plot(t, sol[:, 0], 'b', label='theta(t)')
# plt.plot(t, sol[:, 1], 'g', label='omega(t)')
plt.legend(loc='best')
plt.xlabel('t')
plt.grid()
plt.show()