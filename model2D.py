#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

class Model2D:
	def __init__(self):
		rospy.init_node('model2D')
		rospy.Subscriber("model2D", String, self.callback)
		rospy.spin()

		#q0 = np.array([2, 0])
		#qh = np.array([5, 10])
		#zt = np.array([8]) 
		#T  = np.array([100]) 
		#self.runSimulation(q0, qh, zt, T)


	def callback(self, msg):
		message = str(msg.data)
		data = map(float, message.split())
		q0 = np.array([data[0], data[1]])  # y, z - you cannot track phi in 2d case
		qh = np.array([data[2], data[3]])  # y, z - desired pose
		zt = np.array(data[4])  # take-off height
		T = np.array(data[5])   # hovering time in second; taking value from 50 to 150
		self.runSimulation(q0, qh, zt, T)


	def runSimulation(self, q0, qh, zt, T):
		time.sleep(2)
		print(' ')
		print('2D model is running... Please wait for a moment')
		print(' ')
		time.sleep(1)
		self.N = 3001 # steps
		self.time = np.linspace(0,300,self.N)

		self.xr = np.zeros((self.N,6))
		self.getReference(q0, qh, zt, T) # generate reference trajectory

		self.x = np.zeros((self.N,6))  # index 0-5
		self.u = np.zeros((self.N,2))  # index 0-1

		self.x[0,:] = self.xr[0,:]    # set init state to reference state
		self.u[0,:] = np.array([0,0])

		# simulation starts
		for i in range(1,self.N):
			# calculate control input
			self.positionController(step=i)
			self.attitudeController(step=i)

			# span for next time step
			tspan = [self.time[i-1],self.time[i]] 

			# solve for next step
			output = odeint(self.model, self.x[i-1,:], tspan, args=(self.u[i-1,:],)) 
			self.x[i,:] = output[1,:]  # take the value at time i and drop that at time i-1

		# plot results - you can zoom in to see details in these figures
		plt.figure(1)
		plt.plot(self.time,self.xr[:,0],'r-',label='yr(t)')
		plt.plot(self.time,self.xr[:,1],'g-',label='zr(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('reference trajectory')

		plt.figure(2)
		plt.plot(self.time,self.u[:,0],'m-',label='u1(t)')
		plt.plot(self.time,self.u[:,1],'k-',label='u2(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('control input')

		plt.figure(3)
		plt.plot(self.time,self.x[:,0],'r-',label='y(t)')
		plt.plot(self.time,self.x[:,1],'g-',label='z(t)')
		plt.plot(self.time,self.x[:,2],'b-',label='phi(t)')
		plt.plot(self.time,self.x[:,3],'r--',label='y`(t)')
		plt.plot(self.time,self.x[:,4],'g--',label='z`(t)')
		plt.plot(self.time,self.x[:,5],'b--',label='phi`(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('state evolution')

		plt.show()


	# 2d quadrotor model to be integrated
	def model(self,x,t,u):
		# x[0] - y ; x[1] - z ;  x[2] - phi
		# x[3] - y`; x[4] - z`;  x[5] - phi`
		m = 0.03
		g = 9.8
		Ixx = 1.43*10.0**(-5.0)

		y_dot = x[3]
		z_dot = x[4]
		phi_dot = x[5]
		y_ddot = -g * x[2]
		z_ddot = u[0] / m
		phi_ddot = u[1] / Ixx

		dxdt = [y_dot, z_dot, phi_dot, y_ddot, z_ddot, phi_ddot]
		return dxdt


	def getReference(self, q0, qh, zt, T):
		y0 = q0[0]
		z0 = q0[1]
		yh = qh[0]
		zh = qh[1]

		t1 = 50.0
		t2 = 100.0
		t3 = 300.0

		for step in range(self.N):
			t = self.time[step]
			if t<t1:   # take off
				y = y0
				y_dot = 0.0
				z = zt/t1*t
				z_dot = zt/t1
			elif t<t2:   # go to desired pose
				y = y0+(yh-y0)/(t2-t1)*(t-t1)
				y_dot = (yh-y0)/(t2-t1)
				z = zt+(zh-zt)/(t2-t1)*(t-t1)
				z_dot = (zh-zt)/(t2-t1)
			elif t<(t2+T):  # hover for T second
				y = yh
				y_dot = 0.0
				z = zh 
				z_dot = 0.0
			elif t<t3:   # land
				y = yh
				y_dot = 0.0
				z = zh/(t3-t2-T)*(t3-t)
				z_dot = -zh/(t3-t2-T)
			else:
				y = yh
				y_dot = 0.0
				z = 0.0
				z_dot = 0.0
			self.xr[step,:] = [y, z, 0, y_dot, z_dot, 0] # phi = 0


	def positionController(self, step):
		m = 0.03
		g = 9.8
		kdy = 4.0
		kpy = 4.0
		kdz = 11.0
		kpz = 30.0

		x = self.x[step-1,:] # state
		y = self.xr[step-1,0] # reference
		z = self.xr[step-1,1] # reference
		y_dot = self.xr[step-1,3] # reference
		z_dot = self.xr[step-1,4] # reference

		phi = -1/g*(kdy*(y_dot-x[3])+kpy*(y-x[0]))
		phi_dot = -kdy*x[2]-1/g*kpy*(y_dot-x[3])
		phi_ddot = -kdy*x[5]+kpy*x[2]

		self.u1 = m*(g+kdz*(z_dot-x[4])+kpz*(z-x[1]))
		self.phi_des = np.array([phi, phi_dot, phi_ddot])

	def attitudeController(self, step):
		Ixx = 1.43*10.0**(-5.0)
		kdphi = 4.0
		kpphi = 4.0

		x = self.x[step-1,:]
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
