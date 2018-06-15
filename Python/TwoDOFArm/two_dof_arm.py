from __future__ import print_function
from math import sin, cos, atan2, sqrt
import sys
import os

cwd = os.getcwd()
sys.path.append('/'.join(cwd.split('/')[0:-1]))

from SummerVentures import SimpleArm, ang_diff

def two_dof_arm():
	Kp = 5

	arm = SimpleArm(0, 0, 1, 1, 0, 0)
	theta1 = arm.theta1
	theta2 = arm.theta2
	l1 = arm.L1
	l2 = arm.L2
	dt = 0.01

	while True:
		#############YOUR CODE GOES HERE#############
		
		
		#############################################

if __name__ == '__main__':
	two_dof_arm()