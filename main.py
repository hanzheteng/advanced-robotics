#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String


if __name__ == '__main__':
	rospy.init_node('main')
	pub2d = rospy.Publisher('model2D', String, queue_size = 1)
	pub3d = rospy.Publisher('model3D', String, queue_size = 1)

	print(' ')
	print('Please select the dimension of the system (2 or 3)')
	dimension = input()
	if not (dimension == 2 or dimension == 3):
		print('WRONG input, exiting...')
		sys.exit()

	if dimension == 2:
		print('Please set the initial pose q0 = [y z]')
		print('For example: 2 0 (two integers, seperate by space)')
		q0_s = raw_input()
		q0 = map(int, q0_s.split())
		if not len(q0) == 2:
			print('WRONG input, exiting...')
			sys.exit()

		print('Please set the desired hovering pose qh = [y z]')
		print('For example: 5 10 (two integers, seperate by space)')
		qh_s = raw_input()
		qh = map(int, qh_s.split())
		if not len(qh) == 2:
			print('WRONG input, exiting...')
			sys.exit()

	if dimension == 3:
		print('Please set the initial pose q0 = [x y z yaw]')
		print('For example: 2 2 0 0.79 (four numbers, seperate by space)')
		q0_s = raw_input()
		q0 = map(float, q0_s.split())
		if not len(q0) == 4:
			print('WRONG input, exiting...')
			sys.exit()
		
		print('Please set the desired hovering pose qh = [x y z yaw]')
		print('For example: 5 5 10 1.05 (four numbers, seperate by space)')
		qh_s = raw_input()
		qh = map(float, qh_s.split())
		if not len(qh) == 4:
			print('WRONG input, exiting...')
			sys.exit()

	print('Please set the take-off height zt')
	print('For example: 5 (one positive integer)')
	zt = input()	# height in z-dimension
	if not (zt >= 0):
		print('WRONG input, exiting...')
		sys.exit()

	print('Please set the hovering time T at desired pose')
	print('For example: 100 (one integer between 50 and 150)')
	T = input()
	if not (50 <= T and T <= 150):
		print('WRONG input, exiting...')
		sys.exit()

	data = ''.join([q0_s, ' ', qh_s,' ', str(zt),' ', str(T)])
	msg = String()
	msg.data = data
	if dimension == 2:
		pub2d.publish(msg)
	if dimension == 3:
		pub3d.publish(msg)

	print(' ')
	print(str(dimension) + 'D model is running... Please wait for a moment')
	print(' ')
	
