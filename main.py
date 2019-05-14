#!/usr/bin/env python

import sys
import rospy
import std_msgs.msg


if __name__ == '__main__':
	print('PLease select the dimension of the system (2 or 3)')
	dimension = input()
	if not (dimension == 2 or dimension == 3):
		print('WRONG input, exiting...')
		sys.exit()

	if dimension == 2:
		print('Please set the initial pose q0 = [y z]')
		print('For example: 2 0 (two integers, seperate by space)')
		s = raw_input()
		q0 = map(int, s.split())
		if not len(q0) == 2:
			print('WRONG input, exiting...')
			sys.exit()

		print('Please set the desired hovering pose qh = [y z]')
		print('For example: 5 10 (two integers, seperate by space)')
		s = raw_input()
		qh = map(int, s.split())
		if not len(qh) == 2:
			print('WRONG input, exiting...')
			sys.exit()

	if dimension == 3:
		print('Please set the initial pose q0 = [x y z yaw]')
		print('For example: 2 2 0 0.79 (four numbers, seperate by space)')
		s = raw_input()
		q0 = map(float, s.split())
		if not len(q0) == 4:
			print('WRONG input, exiting...')
			sys.exit()
		
		print('Please set the desired hovering pose qh = [x y z yaw]')
		print('For example: 5 5 10 1.05 (four numbers, seperate by space)')
		s = raw_input()
		qh = map(float, s.split())
		if not len(qh) == 4:
			print('WRONG input, exiting...')
			sys.exit()

	print('Please set the take-off height zt (one positive integer)')
	zt = input()	# height in z-dimension
	if not (zt >= 0):
		print('WRONG input, exiting...')
		sys.exit()

	print('Please set the hovering time T at desired pose (one integer between 50 and 150)')
	T = input()
	if not (50 <= zt <= 150):
		print('WRONG input, exiting...')
		sys.exit()

