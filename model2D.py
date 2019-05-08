#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3

import numpy as np
import scipy as sp
from scipy.integrate import odeint
import matplotlib.pyplot as plt

class Model2D:
	def __init__(self):
		rospy.init_node('model2D')
		#rospy.Subscriber("input2D", Vector3, self.callback)
		#self.state = rospy.Publisher('state2D',Transform,queue_size = 1)
		#msg_pub = Transform()
		#rospy.spin()
		g = 9.8
		Ixx = 1.43*10.0**(-5.0)   # ** instead of ^ for exponentiation
		m = 0.03

		N = 3001 # steps
		t = np.linspace(0,300,N)

		self.x = np.zeros((N,6))  # index 0-5
		self.u = np.zeros((N,2))  # index 0-1

		self.x[0,:] = np.array([0,0,0,0,0,0]) # init state
		self.u[0,:] = np.array([0,0])
		
		# Solve ODE
		for i in range(1,N):
			# calculate control input
			self.controller(step=i, t=t[i])

			# span for next time step
			tspan = [t[i-1],t[i]] 

			# solve for next step
			output = odeint(self.model, self.x[i-1,:], tspan, args=(self.u[i-1,:],)) 
			self.x[i,:] = output[1,:]  # take the value at time i and drop that at time i-1

		# plot results
		plt.plot(t,self.u[:,0],'m-',label='u1(t)')
		plt.plot(t,self.u[:,1],'k-',label='u2(t)')
		plt.plot(t,self.x[:,0],'r-',label='y(t)')
		plt.plot(t,self.x[:,1],'g-',label='z(t)')
		plt.plot(t,self.x[:,2],'b-',label='phi(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.show()

	# function that returns dz/dt
	def model(self,x,t,u):
		g = 9.8
		Ixx = 1.43*10.0**(-5.0)
		m = 0.03

		y_dot = x[3]   # index 0-5
		z_dot = x[4]
		phi_dot = x[5]
		y_ddot = -g * x[2]
		z_ddot = u[0] / m
		phi_ddot = u[1] / Ixx

		dxdt = [y_dot, z_dot, phi_dot, y_ddot, z_ddot, phi_ddot]
		return dxdt

	def controller(self, step, t):
		x = self.x[step-1,:]
		Ixx = 1.43*10.0**(-5.0)
		kdphi = 4.0
		kpphi = 6.4

		y0 = 2.0
		yh = 5.0
		zh = 10.0
		zt = 8.0

		m = 0.03
		g = 9.8
		kdz = .0
		kpz = 6.4
		kdy = 0.5
		kpy = 0.8

		t1 = 50.0
		t2 = 100.0
		t3 = 150.0
		T = 100.0
		t5 = 300.0

		# Generate the reference
		if t<t1:
			y = y0
			y_dot = 0.0
			z = 0.0
			z_dot = 0.0
		elif t<t2:
			y = y0
			y_dot = 0.0
			z = zt/(t2-t1)*(t-t1)
			z_dot = zt/(t2-t1)
		elif t<t3:
			y = y0+(yh-y0)/(t3-t2)*(t-t2)
			y_dot = (yh-y0)/(t3-t2)
			z = zt+(zh-zt)/(t3-t2)*(t-t2)
			z_dot = (zh-zt)/(t3-t2)
		elif t<(t3+T):
			y = yh
			y_dot = 0.0
			z = zh 
			z_dot = 0.0
		elif t<t5:
			y = yh
			y_dot = 0.0
			z = zh/(t5-t3-T)*(t5-t)
			z_dot = -zh/(t5-t3-T)
		else:
			y = yh
			y_dot = 0.0
			z = 0.0
			z_dot = 0.0

		# Position Controller
		phi = -1/g*(kdy*(y_dot-x[3])+kpy*(y-x[0]))
		phi_dot = -kdy*x[2]-1/g*kpy*(y_dot-x[3])
		phi_ddot = -kdy*x[5]+kpy*x[2]
		self.u1 = m*(g+kdz*(z_dot-x[4])+kpz*(z-x[1]))
		self.phi_des = np.array([phi, phi_dot, phi_ddot])

		# Attitude Controller
		phi_ddot = self.phi_des[2]
		phi_dot = self.phi_des[1]
		phi = self.phi_des[0]
		self.u2 = Ixx*(phi_ddot+kdphi*(phi_dot-x[5])+kpphi*(phi-x[2]))
		self.u[step-1,:] = np.array([self.u1, self.u2])


if __name__ == '__main__':
	try:
		model2D = Model2D()
	except rospy.ROSInterruptException:
		pass
