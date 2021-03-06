#!/usr/bin/env python

#The required packages are imported here
from vitarana_drone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32, Float32, Float64
import rospy
import time
import math


class DroneFly():
	"""docstring for DroneFly"""
	def __init__(self):

		rospy.init_node('pluto_fly', disable_signals = True)

		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)

		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)

		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
		rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)

		rospy.Subscriber('/drone_yaw', Float64, self.get_yaw)

		self.cmd = PlutoMsg()

		# Position to hold.
		self.wp_x = -5.36
		self.wp_y = -5.36
		self.wp_z = 30.0

		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		self.cmd.plutoIndex = 0

		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0
		self.drone_yaw = 0.0

		#PID constants for Roll
		self.kp_roll = 203.0
		self.ki_roll = 0.0
		self.kd_roll = 64.0

		#PID constants for Pitch
		self.kp_pitch = 500.0
		self.ki_pitch = 0.0
		self.kd_pitch = 75.0

		#PID constants for Yaw
		self.kp_yaw = 30.0
		self.ki_yaw = 0.0
		self.kd_yaw = 0.0

		#PID constants for Throttle
		self.kp_throt = 705.0
		self.ki_throt = 0.0
		self.kd_throt = 64.0

		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0

		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032

		self.last_alt_error = 0.0
		self.alt_error_sum = 0.0

		self.last_yaw_error = 0.0
		self.yaw_error_sum = 0.0

		self.last_pitch_error = 0.0
		self.pitch_error_sum = 0.0

		self.last_roll_error = 0.0
		self.roll_error_sum = 0.0

		self.correct_angle = 0.0

		rospy.sleep(.1)


	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)


	def position_hold(self):

		rospy.sleep(2)

		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)

		while True:

			self.calc_pid()
			self.publish_plot_data()

		 	pitch_value = int(1500 - self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)

			roll_value = int(1500 - self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)

			throt_value = int(1500 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)

			yaw_value = int(1500 + self.correct_yaw)
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)

			self.pluto_cmd.publish(self.cmd)


	def calc_pid(self):
		self.seconds = time.time()
		current_time = self.seconds - self.last_time
		if(current_time >= self.loop_time):
			self.pid_roll()
			self.pid_pitch()
			self.pid_throt()
			self.pid_yaw()

			self.last_time = self.seconds


	def pid_roll(self):

		time_now = time.time()
		time_change = time_now-self.last_time

		error = self.wp_y - self.drone_y
		self.roll_error_sum += (error*time_change)
		dErr = (error-self.last_roll_error)/time_change

		output = (0.01*self.kp_roll * error) + (0.001*self.ki_roll * self.roll_error_sum) + (0.1*self.kd_roll * dErr)
		self.last_roll_error = error
		self.correct_roll = output

	def pid_pitch(self):

		time_now = time.time()
		time_change = time_now-self.last_time

		error = self.wp_x - self.drone_x
		self.pitch_error_sum += (error*time_change)
		dErr = (error-self.last_pitch_error)/time_change

		output = (0.01*self.kp_pitch * error) + (0.001*self.ki_pitch * self.pitch_error_sum) + (0.1*self.kd_pitch * dErr)
		self.last_pitch_error = error
		self.correct_pitch = output


	def pid_throt(self):

		time_now = time.time()
		time_change = time_now-self.last_time

		error = self.wp_z - self.drone_z
		self.alt_error_sum += (error*time_change)
		dErr = (error-self.last_alt_error)/time_change

		output = (0.01*self.kp_throt * error) + (0.001*self.ki_throt * self.alt_error_sum) + (0.1*self.kd_throt * dErr)
		self.last_alt_error = error
		self.correct_throt = output

	def pid_yaw(self):
		self.correct_angle = math.degrees(math.atan((self.wp_y-self.drone_y)/(self.wp_x-self.drone_x)))

		time_now = time.time()
		time_change = time_now-self.last_time

		error = self.correct_angle-self.drone_yaw
		self.yaw_error_sum += (error*time_change)
		dErr = (error-self.last_yaw_error)/time_change

		output = (0.01*self.kp_yaw * error) + (0.001*self.ki_yaw * self.yaw_error_sum) + (0.1*self.kd_yaw * dErr)
		self.last_yaw_error = error

		self.correct_yaw = output

	def limit(self, input_value, max_value, min_value):

		if input_value >= max_value:
			return max_value
		if input_value <= min_value:
			return min_value
		else:
			return input_value

	def publish_plot_data(self):
		alt_pub = rospy.Publisher('alt_error', Float32, queue_size=10)
		pitch_pub = rospy.Publisher('pitch_error', Float32, queue_size=10)
		roll_pub = rospy.Publisher('roll_error', Float32, queue_size=10)
		zero_line_data_pub = rospy.Publisher('zero_line_data', Float32, queue_size=10)


		alt_error = self.wp_z - self.drone_z
		alt_pub.publish(alt_error)
		throt_pub = rospy.Publisher('throttle_value', Float32, queue_size=10)
		throt_pub.publish(self.correct_throt)

		pitch_error = self.wp_x - self.drone_x
		pitch_pub.publish(pitch_error)

		roll_error = self.wp_y - self.drone_y
		roll_pub.publish(roll_error)

		zero_line_data_pub.publish(0)


	def set_pid_alt(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

		self.kp_throt = pid_val.Kp
		self.ki_throt = pid_val.Ki
		self.kd_throt = pid_val.Kd

	def set_pid_roll(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

		self.kp_roll = pid_val.Kp
		self.ki_roll = pid_val.Ki
		self.kd_roll = pid_val.Kd

	def set_pid_pitch(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

		self.kp_pitch = pid_val.Kp
		self.ki_pitch = pid_val.Ki
		self.kd_pitch = pid_val.Kd

	def set_pid_yaw(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

		self.kp_yaw = pid_val.Kp
		self.ki_yaw = pid_val.Ki
		self.kd_yaw = pid_val.Kd

	def get_pose(self,pose):

		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables

		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z

	def get_yaw(self, drone_yaw):

		#Subscriber function to get drone current yaw

		self.drone_yaw = drone_yaw.data


if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = DroneFly()
		temp.position_hold()
		rospy.spin()
