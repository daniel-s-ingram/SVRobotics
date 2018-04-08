from math import cos, sin, pi
import matplotlib.pyplot as plt
import numpy as np

class SimpleVehicle:
	def __init__(self, x, y, theta, L):
		self.x = x
		self.y = y
		self.theta = theta
		self.L = L
		self.update_points()

	def update_pose(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta
		self.update_points()

	def update_points(self):
		R = self.rotation_matrix(self.theta)

		self.front = np.array([[self.x],[self.y]]) + np.matmul(R, np.array([[self.L/2.0],[0]]))
		self.back_left = np.array([[self.x],[self.y]]) + np.matmul(R, np.array([[-self.L/2.0],[self.L/4.0]]))
		self.back_right = np.array([[self.x],[self.y]]) + np.matmul(R, np.array([[-self.L/2.0],[-self.L/4.0]]))

	def rotation_matrix(self, theta):
		return np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

	def plot(self, dt, x_goal, y_goal, obstacle=False, x_obstacle=0, y_obstacle=0, obstacle_size=0):
		if obstacle:
			obstacle = plt.Circle((x_obstacle, y_obstacle), radius=0.5*obstacle_size, fc='r')
			plt.gca().add_patch(obstacle)

		plt.plot(x_goal, y_goal, 'g.')
		plt.plot([self.front[0], self.back_left[0]], [self.front[1], self.back_left[1]], 'k-', \
		         [self.back_left[0], self.back_right[0]], [self.back_left[1], self.back_right[1]], 'r-', \
		         [self.back_right[0], self.front[0]], [self.back_right[1], self.front[1]], 'k-')

		plt.xlim(-50, 50)
		plt.ylim(-50, 50)

		plt.ion()
		plt.pause(dt)
		plt.show()

class SimpleArm:
	def __init__(self, x, y, l1, l2, theta1, theta2):
		self.shoulder = np.array([[x], [y]])
		self.L1 = l1
		self.L2 = l2
		self.theta1 = theta1
		self.theta2 = theta2
		self.update_points()

	def update_joints(self, theta1, theta2):
		self.theta1 = theta1
		self.theta2 = theta2
		self.update_points()

	def update_points(self):
		self.elbow = self.shoulder + np.array([[self.L1*cos(self.theta1)], [self.L1*sin(self.theta1)]])
		self.wrist = self.elbow + np.array([[self.L2*cos(self.theta1+self.theta2)], [self.L2*sin(self.theta1+self.theta2)]])

	def plot(self, dt):
		plt.cla()

		plt.plot(self.shoulder[0], self.shoulder[1], 'ro')
		plt.plot(self.elbow[0], self.elbow[1], 'ro')
		plt.plot(self.wrist[0], self.wrist[1], 'ro')

		plt.plot([self.shoulder[0], self.elbow[0]], [self.shoulder[1], self.elbow[1]], 'k-')
		plt.plot([self.elbow[0], self.wrist[0]], [self.elbow[1], self.wrist[1]], 'k-')

		lim = self.L1 + self.L2
		plt.xlim(-lim, lim)
		plt.ylim(-lim, lim)

		plt.ion()
		plt.pause(dt)
		plt.show()

def ang_diff(theta1, theta2):
	return (theta1-theta2+pi)%(2*pi)-pi