from __future__ import print_function
from math import sin, cos, atan, acos, pi
import numpy as np
import sys
import os

cwd = os.getcwd()
sys.path.append('/'.join(cwd.split('/')[0:-1]))

from SummerVentures import Arm3d, ang_diff

def rotation_matrix(theta):
	pass

def three_dof_arm():
	arm = Arm3d()
	l2 = arm.L2
	l3 = arm.L3
	theta1 = arm.theta1
	theta2 = arm.theta2
	theta3 = arm.theta3
	dt = 0.01

	while True:
		pass

if __name__ == '__main__':
	three_dof_arm()