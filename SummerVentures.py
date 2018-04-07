from math import cos, sin, pi
import matplotlib.pyplot as plt
import numpy as np

class SimpleVehicle:
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta
		self.update_points()

	def update_pose(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta
		self.update_points()

	def update_points(self):
		R = self.rotation_matrix(self.theta)

		self.front = np.array([[self.x],[self.y]]) + np.matmul(R, np.array([[1],[0]]))
		self.back_left = np.array([[self.x],[self.y]]) + np.matmul(R, np.array([[-1],[0.5]]))
		self.back_right = np.array([[self.x],[self.y]]) + np.matmul(R, np.array([[-1],[-0.5]]))

	def rotation_matrix(self, theta):
		return np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

def plot_vehicle(vehicle, dt, x, y):
	#plt.cla()

	plt.plot(x, y, 'ro')
	plt.plot([vehicle.front[0], vehicle.back_left[0]], [vehicle.front[1], vehicle.back_left[1]], 'k-', \
	         [vehicle.back_left[0], vehicle.back_right[0]], [vehicle.back_left[1], vehicle.back_right[1]], 'r-', \
	         [vehicle.back_right[0], vehicle.front[0]], [vehicle.back_right[1], vehicle.front[1]], 'k-')

	plt.xlim(-50, 50)
	plt.ylim(-50, 50)

	plt.ion()
	plt.pause(dt)
	plt.show()

def ang_diff(theta1, theta2):
	return (theta1-theta2+pi)%(2*pi)-pi