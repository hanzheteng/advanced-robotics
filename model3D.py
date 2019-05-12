#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3

import numpy as np
import scipy as sp
from scipy.integrate import odeint
import matplotlib.pyplot as plt

class Model3D:
	def __init__(self):
		q0 = np.array([2, 2, 0, np.pi/4])  # x, y, z, psi - you can only track x,y,z,psi in 3d case
		qh = np.array([6, 5, 10, np.pi/3]) # x, y, z, psi - desired pose
		zt = np.array([8])     # take-off height
		T  = np.array([100])    # hovering time in second; taking value from 50 to 150

		self.N = 3001 # steps
		self.time = np.linspace(0,300,self.N)

		self.Xr = np.zeros((self.N,12))
		self.getReference(q0, qh, zt, T) # generate reference trajectory

		self.X = np.zeros((self.N,12))  # index 0-11
		self.U = np.zeros((self.N,4))   # index 0-3

		self.X[0,:] = self.Xr[0,:]    # set init state to reference state
		self.U[0,:] = np.array([0,0,0,0])

		# simulation starts
		for i in range(1,self.N):
			# calculate control input
			self.positionController(step=i)
			self.attitudeController(step=i)

			# span for next time step
			tspan = [self.time[i-1],self.time[i]] 

			# solve for next step
			output = odeint(self.model, self.X[i-1,:], tspan, args=(self.U[i-1,:],)) 
			self.X[i,:] = output[1,:]  # take the value at time i and drop that at time i-1

		# plot results - you can zoom in to see details in these figures
		plt.figure(1)
		plt.plot(self.time,self.Xr[:,0],'r-',label='xr(t)')
		plt.plot(self.time,self.Xr[:,1],'g-',label='yr(t)')
		plt.plot(self.time,self.Xr[:,2],'b-',label='zr(t)')
		plt.plot(self.time,self.Xr[:,8],'k-',label='psi(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('reference trajectory')

		plt.figure(2)
		plt.plot(self.time,self.U[:,0],'m-',label='F(t)')
		plt.plot(self.time,self.U[:,1],'r-',label='M1(t)')
		plt.plot(self.time,self.U[:,2],'g-',label='M2(t)')
		plt.plot(self.time,self.U[:,3],'b-',label='M3(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('control input')

		plt.figure(3)
		plt.plot(self.time,self.X[:,0],'r-',label='x(t)')
		plt.plot(self.time,self.X[:,1],'g-',label='y(t)')
		plt.plot(self.time,self.X[:,2],'b-',label='z(t)')
		plt.plot(self.time,self.X[:,3],'r--',label='x`(t)')
		plt.plot(self.time,self.X[:,4],'g--',label='y`(t)')
		plt.plot(self.time,self.X[:,5],'b--',label='z`(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('translational state evolution')

		plt.figure(4)
		plt.plot(self.time,self.X[:,6],'r-',label='phi(t)')
		plt.plot(self.time,self.X[:,7],'g-',label='theta(t)')
		plt.plot(self.time,self.X[:,8],'b-',label='psi(t)')
		plt.plot(self.time,self.X[:,9],'r--',label='phi`(t)')
		plt.plot(self.time,self.X[:,10],'g--',label='theta`(t)')
		plt.plot(self.time,self.X[:,11],'b--',label='psi`(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('rotational state evolution')

		plt.show()


	# 3d quadrotor model to be integrated
	def model(self,x,t,u):
		# x[0] - x ; x[1] - y ;  x[2] - z
		# x[3] - x`; x[4] - y`;  x[5] - z`
		# x[6] - phi ;  x[7] - theta ;   x[8] - psi
		# x[9] - phi`; x[10] - theta`;  x[11] - psi`
		g = 9.8
		m = 0.03

		r_dot = np.matrix(x[3:6]).transpose()

		Omega_dot = np.matrix(x[9:12]).transpose()

		phi = x[6]
		theta = x[7]
		psi = x[8]
		Rx_phi = np.matrix([[1, 0, 0], [0, np.cos(phi), np.sin(phi)], [0, -np.sin(phi), np.cos(phi)]])
		Ry_theta = np.matrix([[np.cos(theta), 0, -np.sin(theta)], [0, 1, 0], [np.sin(theta), 0, np.cos(theta)]])
		Rz_psi = np.matrix([[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
		Rsb_T = Rx_phi*Ry_theta*Rz_psi
		Rsb = np.transpose(Rsb_T)
		g_vec = np.matrix([[0],[0],[-g]])
		F_vec = np.matrix([[0],[0],[u[0]/m]])
		r_ddot = g_vec + Rsb*F_vec

		Ib = np.matrix([[1.43,0,0],[0,1.43,0], [0,0,2.89]])*10**(-5)
		Ib_inv = np.linalg.inv(Ib)
		R0 = np.matrix([[1, 0 ,-np.sin(theta)],
					    [0 ,np.cos(phi),np.cos(theta)*np.sin(phi)],
					    [0 ,-np.sin(phi),np.cos(theta)*np.cos(phi)]])
		R0_inv = np.linalg.inv(R0)
		Moment = np.matrix(u[1:4]).transpose()
		Omega_ddot = R0_inv*Ib_inv*(Moment - np.cross(R0*Omega_dot,Ib*R0*Omega_dot, axis=0) )

		dxdt = np.array([r_dot, r_ddot, Omega_dot, Omega_ddot]) # 12 dimension
		dxdt = np.reshape(dxdt,12)
		return dxdt


	def getReference(self, q0, qh, zt, T):
		x0   = q0[0]
		y0   = q0[1]
		z0   = q0[2]
		psi0 = q0[3]
		xh   = qh[0]
		yh   = qh[1]
		zh   = qh[2]
		psih = qh[3]

		t1 = 50.0
		t2 = 100.0
		t3 = 300.0

		for step in range(self.N):
			t = self.time[step]
			if t<t1:   # take off
				x = x0
				x_dot = 0
				y = y0
				y_dot = 0
				z = zt/t1*t
				z_dot = zt/t1
				psi = psi0
			elif t<t2:   # go to desired pose
				x = x0+(xh-x0)/(t2-t1)*(t-t1)
				x_dot = (xh-x0)/(t2-t1)
				y = y0+(yh-y0)/(t2-t1)*(t-t1)
				y_dot = (yh-y0)/(t2-t1)
				z = zt+(zh-zt)/(t2-t1)*(t-t1)
				z_dot = (zh-zt)/(t2-t1)
				psi = psih
			elif t<(t2+T):  # hover for T second
				x = xh
				x_dot = 0
				y = yh
				y_dot = 0
				z = zh
				z_dot = 0
				psi = psih
			elif t<t3:   # land
				x = xh
				x_dot = 0
				y = yh
				y_dot = 0
				z = zh/(t3-t2-T)*(t3-t)
				z_dot = -zh/(t3-t2-T)
				psi = psih
			else:
				x = xh
				x_dot = 0
				y = yh
				y_dot = 0 
				z = 0
				z_dot = 0
				psi = psih
			self.Xr[step,:] = [x, y, z, x_dot, y_dot, z_dot, 0, 0, psi, 0, 0, 0]


	def positionController(self, step):
		m = 0.03
		g = 9.8
		kdx = 0.4
		kpx = 0.2
		kdz = 0.4
		kpz = 1.25
		kdy = 0.4
		kpy = 0.2

		# slicing 0:6 takes 0,1,2,3,4,5
		R = self.X[step-1, 0:6] # translational state feedback
		Omega = self.X[step-1,6:12] # rotational state feedback

		x = self.Xr[step-1,0] # reference
		y = self.Xr[step-1,1] # reference
		z = self.Xr[step-1,2] # reference
		x_dot = self.Xr[step-1,3] # reference
		y_dot = self.Xr[step-1,4] # reference
		z_dot = self.Xr[step-1,5] # reference
		psi = self.Xr[step-1,8] # reference

		x_control = kdx*(x_dot-R[3])+kpx*(x-R[0])
		y_control = kdy*(y_dot-R[4])+kpy*(y-R[1])
		x_control_hat = np.cos(Omega[2])*x_control+np.sin(Omega[2])*y_control
		y_control_hat = -np.sin(Omega[2])*x_control+np.cos(Omega[2])*y_control
		z_control = kdz*(z_dot-R[5])+kpz*(z-R[2])
		self.F = np.array([ m*z_control/(np.cos(Omega[1])*np.cos(Omega[0]))+m*g ])
		self.U[step-1,0] = self.F

		phi_des = np.arcsin(-y_control_hat/g)
		theta_des = np.arcsin(x_control_hat/(g*np.cos(Omega[0])))     # np.arcsin
		psi_des = psi
		self.Omega_des = np.array([phi_des, theta_des, psi_des])


	def attitudeController(self, step):
		kdphi = 20000
		kpphi = 70000
		kdtheta = 20000
		kptheta = 70000
		kdpsi = 12000
		kppsi = 60000

		Omega = np.matrix(self.X[step-1,6:9]).transpose()
		Omega_dot = np.matrix(self.X[step-1,9:12]).transpose()
		phi = Omega[0]
		theta = Omega[1]
		psi = Omega[2]
		kd = np.matrix([[kdphi,0,0], [0,kdtheta,0], [0,0,kdpsi]])
		kp = np.matrix([[kpphi,0,0], [0,kptheta,0], [0,0,kppsi]])
		I = np.matrix([[1.43,0,0], [0,1.43,0], [0,0,2.89]])*10**(-5)

		W_inv = np.matrix([[1, 0 ,-np.sin(theta)],
						  [0 ,np.cos(phi),np.cos(theta)*np.sin(phi)],
						  [0 ,-np.sin(phi),np.cos(theta)*np.cos(phi)]])
		omega = W_inv * Omega_dot
		omega_bracket = np.matrix([[0,-omega[2],omega[1]],[omega[2],0,-omega[0]],[-omega[1],omega[0],0]])
		omega_dot = omega_bracket*I*omega
		Omega_des = np.matrix(self.Omega_des).transpose()

		self.M = I*W_inv*(kd*(-Omega_dot)+kp*(Omega_des-Omega))+omega_dot
		self.U[step-1,1:4] = np.array(self.M.transpose())


if __name__ == '__main__':
	try:
		model3D = Model3D()
	except rospy.ROSInterruptException:
		pass
