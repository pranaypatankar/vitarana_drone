#!/usr/bin/env python
from vitarana_drone.msg  import *
from pid_tune.msg import PidTune
#from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32, Float32, Float64
from sensor_msgs.msg import NavSatFix
import rospy
import time
#import math
import tf
 
class Edrone():
    def __init__(self):
      
         rospy.init_node('position_controller') #iniitializing ros nade named position_controller
         self.drone_position = [0.0, 0.0 ,0.0 ,0.0]
         self.setpoint = [19.0 , 72.0, 3.0, 0.0]
         self.altitude = 0.0
         self.altitude_setpoint = 0.0
         self.cmd = edrone_cmd()
         self.cmd.rcRoll = 0
         self.cmd.rcPitch = 0
         self.cmd.rcYaw = 0
         self.cmd.rcThrottle = 0
         self.cmd.aux1 = 0

         self.cmd.aux2 = 0
         self.cmd.aux3 = 0
         self.cmd.aux4 = 0
         self.Kp = [0.0 ,0.0 ,0.0, 0.0]
         self.Ki = [0.0 ,0.0 ,0.0, 0.0]	 
         self.Kd = [0.0 ,0.0 ,0.0, 0.0]

         self.error = [0,  0, 0]
         self.prev_values = [0, 0, 0]
         self.error_sum = [0, 0, 0]

         self.drone_command_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size = 1)
         self.roll_error_pub = rospy.Publisher('/x_error', Float32, queue_size = 1)
         self.pitch_error_pub = rospy.Publisher('/y_error', Float32 , queue_size = 1)
         self.throttle_error_pub = rospy.Publisher('/z_error' , Float32, queue_size = 1)  
         self.zero_error_pub = rospy.Publisher('/z_error', Float32 , queue_size=1)      
        # ------------------------Add other ROS Publishere-----------------------------------------------------
         self.sample_time = 0.060 
        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
         #rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
         #rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
         rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid) 
         rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
         #rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
         rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
         rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)

    def gps_callback(self, msg):
     	 self.drone_position[0] = msg.latitude
     	 self.drone_position[1] = msg.longitude
     	 self.drone_position[2] = msg.altitude

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3
    def altitude_set_pid(self, altitude):
        self.Kp[2] = altitude.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = altitude.Ki * 0.008
        self.Kd[2] = altitude.Kd * 0.3 
    
    def pid(self):

    	self.error[0] = (self.setpoint[0] - self.drone_position[0])*1000000
        self.error[1] = (self.setpoint[1] - self.drone_position[1])*1000000
        self.error[2] = self.setpoint[2] - self.drone_position[2]

        self.roll = 1500 + self.Kp[0]*self.error[0] + self.error_sum[0]*self.Ki[0] + (self.error[0] - self.prev_values[0] )*self.Kd[0]
        #self.out_roll = self.Kp[0]*self.error[0] +  self.sample_time +  self.Kd[0]*(self.error[0] - self.prev_values[0] )
        self.pitch = 1500 + self.Kp[1]*self.error[1] + self.error_sum[1]*self.Ki[1] + (self.error[1] - self.prev_values[1] )*self.Kd[1]
        self.throttle = 1500 + self.Kp[2]*self.error[2] + self.error_sum[2]*self.Ki[2] + (self.error[2] - self.prev_values[2] )*self.Kd[2]

        self.prev_values[0] = self.error[0]
        self.prev_values[1] = self.error[1]
        self.prev_values[2] = self.error[2]

        self.error_sum[0] = self.error[0] + self.error_sum[0]
        self.error_sum[1] = self.error[1] + self.error_sum[1]
        self.error_sum[2] = self.error[2] + self.error_sum[2]

        self.cmd.rcRoll = self.roll
        self.cmd.rcPitch = self.pitch
        self.cmd.rcThrottle = self.throttle

        if(self.cmd.rcRoll > 2000):
        	self.cmd.rcRoll = 2000
        if(self.cmd.rcRoll < 1000):
        	self.cmd.rcRoll = 1000
        if(self.cmd.rcPitch > 2000):
        	self.cmd.rcPitch = 2000
        if(self.cmd.rcPitch < 1000):
        	self.cmd.rcPitch = 1000
        if(self.cmd.rcThrottle > 2000):
        	self.cmd.rcThrottle = 2000
        if(self.cmd.rcThrottle < 1000):
        	self.cmd.rcThrottle = 1000
        
        self.drone_command_pub.publish(self.cmd) 
        self.roll_error_pub.publish(self.error[0]) 
        self.pitch_error_pub.publish(self.error[1])
        self.throttle_error_pub.publish(self.error[2])
        self.zero_error_pub.publish(0)

        #self.setpoint = [19.0000451704 , 72 , 3 , 0] 


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
