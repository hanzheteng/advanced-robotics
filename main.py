#!/usr/bin/env python

import rospy
import std_msgs.msg
# need to add check item

def controller2D(q_0,q_h,z_t,T):

	
		
    
if __name__ == '__main__':
        try:
		print 'PLease choose dimension of the system (2 or 3)'
		dimension = input('') # number
		print 'Please set the initial pose q0'
		q_0= input('') # 2 for 2D, 4 for 3D
		print 'Please set the desired hovering pose qh'
		q_h = input('')
		print 'Please set the hovering height zt'
		z_t = input('')	# height in z-dimension
		print 'Please set the hovering time T'
		T = input('') # seconds
		if dimension == 2:
			controller2D(q_0,q_h,z_t,T)
		elif dimension == 3: 
			controller3D(q_0,q_h,z_t,T)
		else: print ("Invalid dimension")
    	except rospy.ROSInterruptException:
		pass