from math import cos, sin, pi, sqrt
import matplotlib.pyplot as plt
import numpy as np

class SimpleVehicle:
	def __init__(self, x, y, theta, L):
		self.x = x
		self.y = y
		self.theta = theta
		self.L = L
		self.goal = [x, y]
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

	def click(self, event):
		self.goal = [event.xdata, event.ydata]

	def plot(self, dt, x_goal, y_goal, xlims=[-50,50], ylims=[-50,50], obstacle=False, x_obstacle=0, y_obstacle=0, obstacle_size=0):
		#plt.cla()

		fig = plt.figure(1)
		fig.canvas.mpl_connect('button_press_event', self.click)

		if obstacle:
			obstacle = plt.Circle((x_obstacle, y_obstacle), radius=0.5*obstacle_size, fc='r')
			plt.gca().add_patch(obstacle)

		plt.plot(x_goal, y_goal, 'g.')
		plt.plot([self.front[0], self.back_left[0]], [self.front[1], self.back_left[1]], 'k-', \
		         [self.back_left[0], self.back_right[0]], [self.back_left[1], self.back_right[1]], 'r-', \
		         [self.back_right[0], self.front[0]], [self.back_right[1], self.front[1]], 'k-')

		plt.xlim(xlims)
		plt.ylim(ylims)

		plt.ion()
		plt.pause(dt)
		plt.show()

class SimpleArm:
	def __init__(self, x, y, L1, L2, theta1, theta2):
		self.shoulder = np.array([[x], [y]])
		self.L1 = L1
		self.L2 = L2
		self.theta1 = theta1
		self.theta2 = theta2
		self.goal = [L1*cos(theta1)+L2*cos(theta1+theta2), L1*sin(theta1), L2*sin(theta1+theta2)]
		self.update_points()

	def update_joints(self, theta1, theta2):
		self.theta1 = theta1
		self.theta2 = theta2
		self.update_points()

	def update_points(self):
		self.elbow = self.shoulder + np.array([[self.L1*cos(self.theta1)], [self.L1*sin(self.theta1)]])
		self.wrist = self.elbow + np.array([[self.L2*cos(self.theta1+self.theta2)], [self.L2*sin(self.theta1+self.theta2)]])

	def click(self, event):
		self.goal = [event.xdata, event.ydata]

	def plot(self, dt):
		plt.cla()

		fig = plt.figure(1)
		fig.canvas.mpl_connect('button_press_event', self.click)

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

class Omni:
	def __init__(self, x, y, L, W, theta, alpha, wheel_radius):
		self.x = x
		self.y = y

		self.L = L
		self.W = W

		self.theta = theta
		self.alpha = alpha

		self.wheel_radius = wheel_radius

		self.update_points()

	def drive(self, v, theta, omega):
		vx = v*cos(theta)
		vy = v*sin(theta)

		self.v1 = np.array([[vx], [vy]]) + np.array([[-self.p1[1,0]*omega], [self.p1[0,0]*omega]])
		self.v2 = np.array([[vx], [vy]]) + np.array([[-self.p2[1,0]*omega], [self.p2[0,0]*omega]])
		self.v3 = np.array([[vx], [vy]]) + np.array([[-self.p3[1,0]*omega], [self.p3[0,0]*omega]])
		self.v4 = np.array([[vx], [vy]]) + np.array([[-self.p4[1,0]*omega], [self.p4[0,0]*omega]])

		self.x = self.x + vx*0.1
		self.y = self.y + vy*0.1
		self.theta = self.theta + omega*0.1

		self.update_points()

	def update_points(self):
		R = self.rotation_matrix(self.theta)

		self.p1 = np.array([[self.x],[self.y]]) + np.matmul(R, np.array([[-0.5*self.W],[0.5*self.L]]))
		self.p2 = np.array([[self.x],[self.y]]) + np.matmul(R, np.array([[0.5*self.W],[0.5*self.L]]))
		self.p3 = np.array([[self.x],[self.y]]) + np.matmul(R, np.array([[0.5*self.W],[-0.5*self.L]]))
		self.p4 = np.array([[self.x],[self.y]]) + np.matmul(R, np.array([[-0.5*self.W],[-0.5*self.L]]))

		self.plot()

	def rotation_matrix(self, theta):
		return np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

	def plot(self):
		plt.cla()

		plt.plot([self.p1[0], self.p2[0]], [self.p1[1], self.p2[1]], 'k-')
		plt.plot([self.p2[0], self.p3[0]], [self.p2[1], self.p3[1]], 'k-')
		plt.plot([self.p3[0], self.p4[0]], [self.p3[1], self.p4[1]], 'k-')
		plt.plot([self.p4[0], self.p1[0]], [self.p4[1], self.p1[1]], 'k-')

		if hasattr(self, 'v1'):
			plt.arrow(self.p1[0,0], self.p1[1,0], self.v1[0,0]/sqrt(self.v1[0,0]**2 + self.v1[1,0]**2), \
				self.v1[1,0]/sqrt(self.v1[0,0]**2 + self.v1[1,0]**2), width=0.01)
			plt.arrow(self.p2[0,0], self.p2[1,0], self.v2[0,0]/sqrt(self.v2[0,0]**2 + self.v2[1,0]**2), \
				self.v2[1,0]/sqrt(self.v1[0,0]**2 + self.v1[1,0]**2), width=0.01)
			plt.arrow(self.p3[0,0], self.p3[1,0], self.v3[0,0]/sqrt(self.v3[0,0]**2 + self.v3[1,0]**2), \
				self.v3[1,0]/sqrt(self.v1[0,0]**2 + self.v1[1]**2), width=0.01)
			plt.arrow(self.p4[0,0], self.p4[1,0], self.v4[0,0]/sqrt(self.v4[0,0]**2 + self.v4[1,0]**2), \
				self.v4[1,0]/sqrt(self.v1[0,0]**2 + self.v1[1]**2), width=0.01)

		plt.xlim(-10, 10)
		plt.ylim(-10, 10)

		plt.ion()
		plt.pause(0.1)
		plt.show()

def ang_diff(theta1, theta2):
	return (theta1-theta2+pi)%(2*pi)-pi