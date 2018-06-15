from __future__ import print_function
from random import random 
from math import atan2, cos, sin, sqrt, pi
import sys
import os

cwd = os.getcwd()
sys.path.append('/'.join(cwd.split('/')[0:-1]))

from SummerVentures import SimpleVehicle, ang_diff

def pure_pursuit(x_start, y_start, theta_start):
	Kv = 3
	Kh = 6
	Ki = 0.00001
	d = 1
	dt = 0.1
	A = 5

	vehicle = SimpleVehicle(x_start, y_start, theta_start, 1.0)
	
	x = x_start
	y = y_start
	theta = theta_start

	x_goal = -10
	y_goal = A*sin(x_goal)

	x_diff = x_goal-x
	y_diff = y_goal-y
	
	error = sqrt(x_diff**2 + y_diff**2)-d
	totalError = 0
	T = 0

	while True:
		#############YOUR CODE GOES HERE#############
		
		
		#############################################

if __name__ == '__main__':
	if len(sys.argv) == 6:
		try:
			x_start = int(sys.argv[1])
			y_start = int(sys.argv[2])
			theta_start = int(sys.argv[3])
		except ValueError:
			print('Invalid value. Using random values instead')
	else:
		x_start = 20*random()-10
		y_start = 20*random()-10
		theta_start = 2*pi*random()-pi

	print('Starting position: (%f m, %f m)' % (x_start, y_start))
	print('Starting orientation: %f rad' % (theta_start))

	pure_pursuit(x_start, y_start, theta_start)