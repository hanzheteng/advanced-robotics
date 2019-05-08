#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
import control
import control.matlab
import numpy as np

class modelClass:
	def __init__(self):
		rospy.init_node('model2D', anonymous = True)
		rospy.Subscriber("input2D", Vector3, self.callback)
		self.state = rospy.Publisher('state2D',Transform,queue_size = 1)
		msg_pub = Transform()
		rospy.spin()

	def callback(self,data):
		update = data.data
		g = 9.8
		Ixx = 
		m = 0.03

		# Define the system 
		A = [[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[0,0,-g,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
		B = [[0,0],[0,0],[0,0],[0,0],[1/m,0],[0,1/Ixx]]
		C = np.identity(6)
		D = np.zeros((6,2))
		sys = control.ss(A,B,C,D)

		# Propagate the system
		x0 = [0,0,0,0,0,0]
		T = [0,0.1]
		u = [[self.u1,self.u1],[self.u2,self.u2]]
		yout,T,xout = control.matlab.lsim(sys,u,T,x0) 
		stateOutput = yout[1]
		msg_pub.translation.x = stateOutput[0]
		msg_pub.translation.y = stateOutput[1]
		msg_pub.translation.z = stateOutput[2]
		quaternion = quaternion_from_euler(stateOutput[3],stateOutput[4],stateOutput[5])
		state.publish(msg_pub)

if __name__ == '__main__':
	try:
		model = modelClass()
    	except rospy.ROSInterruptException:
		pass
