import time
from scipy.integrate import odeint
import numpy as np


def MassSpringDamper(state,t,F):
	x = state[0]
	xd = state[1]
	m = 2.0 # Kilograms
	b = 12.6
	k = 20.0 # Newtons per meter
	xdd = -(b/m)*xd - (k/m)*x + F/m
	return [xd, xdd]

def Pendulum(state, t, M):
    theta, omega = state
    J = 1.; b = 10.; k = 0.
    xd = [omega, (M - b*omega - k*np.sin(theta)) / J ]
    return xd


def velocity_imp(hum_vel, imp_pose_prev, imp_vel_prev, time_prev):
	F_coeff = 12 # 7
	time_step = time.time() - time_prev
	time_prev = time.time()
	t = [0. , time_step]
	F = - hum_vel * F_coeff

	state0_x = [imp_pose_prev[0], imp_vel_prev[0]]
	state_x = odeint(MassSpringDamper, state0_x, t, args=(F[0],))
	state_x = state_x[1]

	state0_y = [imp_pose_prev[1], imp_vel_prev[1]]
	state_y = odeint(MassSpringDamper, state0_y, t, args=(F[1],))
	state_y = state_y[1]

	imp_pose = np.array( [state_x[0], state_y[0]] )
	imp_vel  = np.array( [state_x[1], state_y[1]] )

	# return state[0]
	return imp_pose, imp_vel, time_prev


def angular_imp(theta, imp_theta_prev, imp_omega_prev, time_prev):
	M_coeff = 10 # 7
	time_step = time.time() - time_prev
	time_prev = time.time()
	t = [0. , time_step]
	M = - sin(imp_theta_prev - theta) * M_coeff
	state0 = [imp_theta_prev, imp_omega_prev]
	state = odeint(Pendulum, state0, t, args=(M,))
	state = state[1]

	imp_theta = state[0]
	imp_omega = state[1]
	return imp_theta, imp_omega, time_prev