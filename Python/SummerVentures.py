from math import cos, sin, pi, sqrt
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class SimpleVehicle:
	def __init__(self, x=0, y=0, theta=0, L=1):
		self.x = x
		self.y = y
		self.theta = theta
		self.L = L
		self.goal = [x, y]
		self.fig = plt.figure(1)
		self.fig.canvas.mpl_connect('button_press_event', self.click)
		self.update_points()

	def update_pose(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta
		self.update_points()

	def update_points(self):
		self.R = self.rotation_matrix(self.theta)

		self.front = np.array([[self.x],[self.y]]) + np.matmul(self.R, np.array([[self.L/2.0],[0]]))
		self.back_left = np.array([[self.x],[self.y]]) + np.matmul(self.R, np.array([[-self.L/2.0],[self.L/4.0]]))
		self.back_right = np.array([[self.x],[self.y]]) + np.matmul(self.R, np.array([[-self.L/2.0],[-self.L/4.0]]))

	def rotation_matrix(self, theta):
		return np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

	def click(self, event):
		self.goal = [event.xdata, event.ydata]

	def plot(self, dt=0.1, plot_goal=False, goal_pos=[10,10], plot_obstacle=False, obstacle_pos=[0,0], obstacle_size=0, xlims=[-50,50], ylims=[-50,50]):
		#plt.cla()

		if plot_obstacle:
			obstacle = plt.Circle((obstacle_pos[0], obstacle_pos[1]), radius=0.5*obstacle_size, fc='r')
			plt.gca().add_patch(obstacle)

		if plot_goal:
			plt.plot(goal_pos[0], goal_pos[1], 'g.')

		plt.plot([self.front[0], self.back_left[0]], [self.front[1], self.back_left[1]], 'k-', \
		         [self.back_left[0], self.back_right[0]], [self.back_left[1], self.back_right[1]], 'r-', \
		         [self.back_right[0], self.front[0]], [self.back_right[1], self.front[1]], 'k-')

		plt.xlim(xlims)
		plt.ylim(ylims)

		plt.ion()
		plt.pause(dt)
		plt.show()

class SimpleArm:
	def __init__(self, x=0, y=0, L1=1, L2=1, theta1=0, theta2=0):
		self.shoulder = np.array([[x], [y]])

		self.L1 = L1
		self.L2 = L2

		self.goal = [L1*cos(theta1)+L2*cos(theta1+theta2), L1*sin(theta1), L2*sin(theta1+theta2)]

		self.update_joints(theta1, theta2)

	def update_joints(self, theta1, theta2, theta3=0):
		self.theta1 = theta1
		self.theta2 = theta2
		self.theta3 = theta3
		self.update_points()

	def update_points(self):
		self.elbow = self.shoulder + np.array([[self.L1*cos(self.theta1)], [self.L1*sin(self.theta1)]])
		self.wrist = self.elbow + np.array([[self.L2*cos(self.theta1+self.theta2)], [self.L2*sin(self.theta1+self.theta2)]])

	def click(self, event):
		self.goal = [event.xdata, event.ydata]

	def plot(self, dt=0.01):
		plt.cla()

		fig = plt.figure(1)
		fig.canvas.mpl_connect('button_press_event', self.click)

		plt.plot(self.goal[0], self.goal[1], 'g*')

		plt.plot([self.shoulder[0], self.elbow[0]], [self.shoulder[1], self.elbow[1]], 'k-', linewidth=10)
		plt.plot([self.elbow[0], self.wrist[0]], [self.elbow[1], self.wrist[1]], 'k-', linewidth=10)

		plt.plot(self.shoulder[0], self.shoulder[1], 'ro', markersize=20)
		plt.plot(self.elbow[0], self.elbow[1], 'ro', markersize=20)
		plt.plot(self.wrist[0], self.wrist[1], 'ro', markersize=20)

		lim = self.L1 + self.L2
		plt.xlim(0, lim)
		plt.ylim(0, lim)

		plt.ion()
		plt.pause(dt)
		plt.show()

class Arm3d:
	def __init__(self, L1=0, L2=1, L3=1, theta1=0, theta2=0, theta3=0):
		self.shoulder = np.array([[0], [0], [0]])

		self.L1 = L1
		self.L2 = L2
		self.L3 = L3

		self.goal = [1,1,1]

		sim_fig = plt.figure(1)
		self.sim_ax = sim_fig.add_subplot(111, projection='3d')
		plt.subplots_adjust(left=0.3, bottom=0.3, right=0.7)

		z_ax = plt.axes([0.1, 0.1, 0.8, 0.01])
		y_ax = plt.axes([0.1, 0.15, 0.8, 0.01])
		x_ax = plt.axes([0.1, 0.2, 0.8, 0.01])

		self.x_slider = Slider(x_ax, 'x', 0.1, 2.0, valinit=1)
		self.y_slider = Slider(y_ax, 'y', 0.1, 2.0, valinit=1)
		self.z_slider = Slider(z_ax, 'z', 0.1, 2.0, valinit=1)

		self.x_slider.on_changed(self.slider_callback)
		self.y_slider.on_changed(self.slider_callback)
		self.z_slider.on_changed(self.slider_callback)

		self.update_joints(theta1, theta2, theta3)

	def slider_callback(self, val):
		self.goal = [self.x_slider.val, self.y_slider.val, self.z_slider.val]

	def update_joints(self, theta1, theta2, theta3):
		self.theta1 = theta1
		self.theta2 = theta2
		self.theta3 = theta3

		self.R = self.rotation_matrix(theta1)

		self.update_points()

	def update_points(self):
		elbow = self.shoulder + np.array([[self.L2*cos(self.theta2)], [0], [self.L2*sin(self.theta2)]])
		self.elbow = np.matmul(self.R, elbow)
		self.wrist = np.matmul(self.R, elbow + np.array([[self.L3*cos(self.theta2+self.theta3)], [0], [self.L3*sin(self.theta2+self.theta3)]]))

		self.plot()

	def rotation_matrix(self, theta1):
		return np.array([[cos(theta1), -sin(theta1), 0], [sin(theta1), cos(theta1), 0], [0, 0, 1]])

	def plot(self, dt=0.01):
		plt.axes(self.sim_ax)
		plt.cla()

		self.sim_ax.plot([self.shoulder[0, 0], self.elbow[0, 0]], [self.shoulder[1, 0], self.elbow[1, 0]], [self.shoulder[2, 0], self.elbow[2, 0]], \
			'k-', linewidth=10)
		self.sim_ax.plot([self.elbow[0, 0], self.wrist[0, 0]], [self.elbow[1, 0], self.wrist[1, 0]], [self.elbow[2, 0], self.wrist[2, 0]], \
			'k-', linewidth=10)

		self.sim_ax.plot([self.shoulder[0, 0], self.elbow[0, 0], self.wrist[0, 0]], \
			[self.shoulder[1, 0], self.elbow[1, 0], self.wrist[1, 0]], \
			[self.shoulder[2, 0], self.elbow[2, 0], self.wrist[2, 0]], \
			'r.', markersize=40)

		lim = self.L2 + self.L3
		plt.xlim(0, lim)
		plt.ylim(0, lim)
		self.sim_ax.set_zlim(0, lim)

		plt.ion()
		plt.pause(dt)
		plt.show()

class OmniVehicle:
	def __init__(self, x=0, y=0, L=1, W=0.5, theta=0, alpha=0.5, wheel_radiu=0.25):
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

class Quadrotor:
	def __init__(self):
		self.x = 10
		self.y = 10
		self.z = 20
		self.x_vel = 0
		self.y_vel = 0
		self.z_vel = 0
		self.x_acc = 0
		self.y_acc = 0
		self.z_acc = 0

		self.des_x = 10
		self.des_y = 10
		self.des_z = 20
		self.des_x_vel = 0
		self.des_y_vel = 0
		self.des_z_vel = 0
		self.des_x_acc = 0
		self.des_y_acc = 0
		self.des_z_acc = 0

		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.roll_vel = 0
		self.pitch_vel = 0
		self.yaw_vel = 0

		self.des_yaw = 0

		#self.base_size = 0.172
		self.base_size = 2
		self.m = 0.18
		self.g = 9.81
		self.Ixx = 0.00025
		self.Iyy = 0.00232
		self.Izz = 0.0003738

		self.thrust = self.m*self.g
		self.torque_x = 0
		self.torque_y = 0
		self.torque_z = 0

		self.Kp_x = 0.01
		self.Kp_y = 0.01
		self.Kp_z = 0.01
		self.Kp_roll = 0.001
		self.Kp_pitch = 0.001
		self.Kp_yaw = 0.1

		self.dt = 0.1

		fig = plt.figure()
		self.ax = fig.add_subplot(111, projection='3d')

		self.R = self.rotation_matrix(self.roll, self.pitch, self.yaw)

		self.plot()

	def set_desired_state(self, des_pos, des_vel, des_acc):
		self.des_x = des_pos[0]
		self.des_y = des_pos[1]
		self.des_z = des_pos[2]
		self.des_x_vel = des_vel[0]
		self.des_y_vel = des_vel[1]
		self.des_z_vel = des_vel[2]
		self.des_x_acc = des_acc[0]
		self.des_y_acc = des_acc[1]
		self.des_z_acc = des_acc[2]
		self.fly()

	def fly(self):
		self.thrust = self.m*(self.g + self.des_z_acc + self.Kp_z*(self.des_z-self.z))
		self.des_x_acc += self.Kp_x*(self.des_x-self.x)
		self.des_y_acc += self.Kp_y*(self.des_y-self.y)
		self.torque_x = self.Kp_roll*(((self.des_x_acc*sin(self.des_yaw) - self.des_y_acc*cos(self.des_yaw))/self.g) - self.roll)
		self.torque_y = self.Kp_pitch*(((self.des_x_acc*cos(self.des_yaw) - self.des_y_acc*sin(self.des_yaw))/self.g) - self.pitch)
		self.torque_z = self.Kp_yaw*(self.des_yaw-self.yaw)

		self.roll = self.roll + self.roll_vel*self.dt
		self.pitch = self.pitch + self.pitch_vel*self.dt
		self.yaw = self.yaw + self.yaw_vel*self.dt
		self.roll_vel = self.roll_vel + (self.torque_x/self.Ixx)*self.dt
		self.pitch_vel = self.pitch_vel + (self.torque_y/self.Iyy)*self.dt
		self.yaw_vel = self.yaw_vel + (self.torque_z/self.Izz)*self.dt

		self.R = self.rotation_matrix(self.roll, self.pitch, self.yaw)

		acc = (np.matmul(self.R, np.array([[0], [0], [self.thrust]])) - np.array([[0], [0], [self.m*self.g]]))/self.m
		self.x_acc = acc[0,0]
		self.y_acc = acc[1,0]
		self.z_acc = acc[2,0]

		self.x = self.x + self.x_vel*self.dt
		self.y = self.y + self.y_vel*self.dt
		self.z = self.z + self.z_vel*self.dt

		self.x_vel = self.x_vel + self.x_acc*self.dt
		self.y_vel = self.y_vel + self.y_acc*self.dt
		self.z_vel = self.z_vel + self.z_acc*self.dt

		self.plot()

	def rotation_matrix(self, roll, pitch, yaw):
		return np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll)],
				 [sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll), -cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll)],
				 [-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(yaw)]])

	def plot(self):
		plt.cla()

		rotor1_pos = np.array([[self.x], [self.y], [self.z]]) + np.matmul(self.R, np.array([[0.5*self.base_size], [0], [0]]))
		rotor2_pos = np.array([[self.x], [self.y], [self.z]]) + np.matmul(self.R, np.array([[-0.5*self.base_size], [0], [0]]))
		rotor3_pos = np.array([[self.x], [self.y], [self.z]]) + np.matmul(self.R, np.array([[0], [0.5*self.base_size], [0]]))
		rotor4_pos = np.array([[self.x], [self.y], [self.z]]) + np.matmul(self.R, np.array([[0], [-0.5*self.base_size], [0]]))

		self.ax.plot([rotor1_pos[0,0], rotor2_pos[0,0]], [rotor1_pos[1,0], rotor2_pos[1,0]], [rotor1_pos[2,0], rotor2_pos[2,0]], 'k-')
		self.ax.plot([rotor3_pos[0,0], rotor4_pos[0,0]], [rotor3_pos[1,0], rotor4_pos[1,0]], [rotor3_pos[2,0], rotor4_pos[2,0]], 'k-')
		self.ax.plot([rotor1_pos[0,0], rotor2_pos[0,0], rotor3_pos[0,0], rotor4_pos[0,0]],
			[rotor1_pos[1,0], rotor2_pos[1,0], rotor3_pos[1,0], rotor4_pos[1,0]],
			[rotor1_pos[2,0], rotor2_pos[2,0], rotor3_pos[2,0], rotor4_pos[2,0]], 'r.')

		plt.xlim(-20, 20)
		plt.ylim(-20, 20)
		self.ax.set_zlim(0, 40)

		plt.ion()
		plt.pause(0.001)
		plt.show()

class TrajectoryGenerator:
	def __init__(self, start_pos, des_pos, T, start_vel=[0,0,0], des_vel=[0,0,0], start_acc=[0,0,0], des_acc=[0,0,0]):
		self.start_x = start_pos[0]
		self.start_y = start_pos[1]
		self.start_z = start_pos[2]

		self.des_x = des_pos[0]
		self.des_y = des_pos[1]
		self.des_z = des_pos[2]

		self.start_x_vel = start_vel[0]
		self.start_y_vel = start_vel[1]
		self.start_z_vel = start_vel[2]

		self.des_x_vel = des_vel[0]
		self.des_y_vel = des_vel[1]
		self.des_z_vel = des_vel[2]

		self.start_x_acc = start_acc[0]
		self.start_y_acc = start_acc[1]
		self.start_z_acc = start_acc[2]

		self.des_x_acc = des_acc[0]
		self.des_y_acc = des_acc[1]
		self.des_z_acc = des_acc[2]

		self.T = T

	def solve(self):
		A = np.array([[0, 0, 0, 0, 0, 1],
			[self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
			[0, 0, 0, 0, 1, 0],
			[5*self.T**4, 4*self.T**3, 3*self.T**2, 2*self.T, 1, 0],
			[0, 0, 0, 2, 0, 0],
			[20*self.T**3, 12*self.T**2, 6*self.T, 2, 0, 0]])

		b_x = np.array([[self.start_x],
			[self.des_x],
			[self.start_x_vel],
			[self.des_x_vel],
			[self.start_x_acc],
			[self.des_x_acc]])

		b_y = np.array([[self.start_y],
			[self.des_y],
			[self.start_y_vel],
			[self.des_y_vel],
			[self.start_y_acc],
			[self.des_y_acc]])

		b_z = np.array([[self.start_z],
			[self.des_z],
			[self.start_z_vel],
			[self.des_z_vel],
			[self.start_z_acc],
			[self.des_z_acc]])

		self.x_c = np.linalg.solve(A, b_x)
		self.y_c = np.linalg.solve(A, b_y)
		self.z_c = np.linalg.solve(A, b_z)

def ang_diff(theta1, theta2):
	return (theta1-theta2+pi)%(2*pi)-pi