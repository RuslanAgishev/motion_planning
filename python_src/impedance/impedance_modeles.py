#!/usr/bin/env 

import time
from scipy.integrate import odeint
import numpy as np
from math import *


def MassSpringDamper(state,t,F, mode='critically_damped'):
	x = state[0]
	xd = state[1]
	if mode=='oscillations':
		m = 1.0; k = 2; b = 0 # oscillations
	elif mode=='underdapmped':
		m = 1.0; k = 2; b = 2*sqrt(m*k)-2 # underdamped
	elif mode=='overdamped':
		m = 1.0; k = 2; b = 2*sqrt(m*k)+2 # overdamped
	else:
		m = 1.0; k = 2; b = 2*sqrt(m*k) # critically damped
	xdd = -(b/m)*xd - (k/m)*x + F/m
	return [xd, xdd]


def velocity_imp(hum_vel, imp_pose_prev, imp_vel_prev, time_prev, mode='critically_damped'):
	F_coeff = 12 # 7
	time_step = time.time() - time_prev
	time_prev = time.time()
	t = [0. , time_step]
	F = - hum_vel * F_coeff

	state0_x = [imp_pose_prev[0], imp_vel_prev[0]]
	state_x = odeint(MassSpringDamper, state0_x, t, args=(F[0],mode,))
	state_x = state_x[1]

	state0_y = [imp_pose_prev[1], imp_vel_prev[1]]
	state_y = odeint(MassSpringDamper, state0_y, t, args=(F[1],mode,))
	state_y = state_y[1]

	imp_pose = np.array( [state_x[0], state_y[0]] )
	imp_vel  = np.array( [state_x[1], state_y[1]] )

	# return state[0]
	return imp_pose, imp_vel, time_prev

