#! /usr/bin/env python
'''
Created on November 30, 2019
Version 2.0: May 16, 2022
@author: German Mad. Randomgrid.com
'''
import threading
import serial
import time
import math
import numpy as np
import rospy

import tf

from std_msgs.msg import Float32 # yaw, yaw_dt

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
# from niubot.msg import NiubotRaw
from niubot_core.msg import NiubotState

def _OnLineReceived(line):
    print(line)

class NiubotBringUp(object):
    '''
    Helper class for receiving lines from a serial port
    '''
# /dev/ttyUSB1 /dev/ttyACM0
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, lineHandler = _OnLineReceived):
        '''
        Initializes the receiver class.
        port: The serial port to listen to.
        receivedLineHandler: The function to call when a line was received.
        '''
         # robot constants
        self.radio = 0.03
        self.eje = 0.178
        #de centro A RUEDA 0.89
        #self.ticks_per_rev = 60.0

        #self.m_per_tick = (2.0 * math.pi * self.radio) / self.ticks_per_rev
        #self.rad_per_tick = (2.0 * math.pi) / self.ticks_per_rev

        # vars
        self.ertime = 0.0 # s
        self.dt = 0.0 # s

        #elf.count_left = 0  # ticks
        #self.count_right = 0  # ticks

        self.dir_l = 0  # 1-forward -1-backward 0-Stop
        self.dir_r = 0  # 1-forward -1-backward 0-Stop

        self.pwm_l = 0  # -254 a 254
        self.pwm_r = 0  # -254 a 254

        self.v_l = 0.0  # m/s
        self.v_r = 0.0  # m/s

        self.yaw = 0.0
        self.asqrt = 0.0

        self.ceil = 0.0
        self.ir_l = 0.0
        self.ir_r = 0.0

        self.aX = 0.0
        self.aY = 0.0
        self.aZ = 0.0
        self.gX = 0.0
        self.gY = 0.0
        self.gZ = 0.0
        self.mX = 0.0
        self.mY = 0.0
        self.mZ = 0.0

        self.fov = 0.4
        self.min_range = 0.02
        self.max_range = 0.15

        self.fov_ceil = 0.1
        self.min_range_ceil = 0.02
        self.max_range_ceil = 2.3

        self.v = 0.0
        self.w = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.xdot = 0.0
        self.ydot = 0.0
        self.thetadot = 0.0

        self.phi = 0.0

        self.servo = 0.0

        self.orientation_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.1]# Row major about x, y, z axes float64
        self.angular_velocity_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,.0,0.0,0.0] # Row major about x, y, z float64
        self.linear_acceleration_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,.0,0.0,0.0] # Row major x, y z float64

        self.pose_covariance = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # float64
        self.twist_covariance = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # float64

        self._Port = "/dev/ttyAMA0"
        self._Baudrate = 115200
        self.ReceivedLineHandler = lineHandler
        self._KeepRunning = False

    def Talkers(self):
        rospy.init_node('niubot_core', anonymous=True)

        self.state_pub = rospy.Publisher('/niubot/state', NiubotState, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        self.ceil_pub = rospy.Publisher('/niubot/ceil', Range, queue_size=10)
        self.ir_l_pub = rospy.Publisher('/niubot/ir_l', Range, queue_size=10)
        self.ir_r_pub = rospy.Publisher('/niubot/ir_r', Range, queue_size=10)

        #self.pose_pub = rospy.Publisher('niubot/pose',  PoseWithCovarianceStamped, queue_size=10)
        #self.twist_pub = rospy.Publisher('niubot/twist',  TwistWithCovarianceStamped, queue_size=10)
        #self.yaw_pub = rospy.Publisher('niubot/yaw', Float32, queue_size=10)

        self.odom_broadcaster = tf.TransformBroadcaster()

        self.odom = Odometry()
        self.odom_pub.publish(self.odom)

    def Start(self):
        self._Serial = serial.Serial(port=self._Port, baudrate=self._Baudrate, timeout=1)

        self._KeepRunning = True
        self._ReceiverThread = threading.Thread(target=self._Listen)
        self._ReceiverThread.setDaemon(True)
        self.Talkers()
        self._ReceiverThread.start()

    def Stop(self):
        rospy.loginfo("Stopping serial gateway")
        self._KeepRunning = False
        time.sleep(0.1)
        self._Serial.close()

    def _Listen(self):
         while self._KeepRunning: # not rospy.is_shutdown():
             self.line = self._Serial.readline()
             self.line = str(self.line)
             self.line = self.line.replace("\\r\\n'", "")
             self.line = self.line.replace("b'", "")
             self.msg = self.line.split(',')
             #print(self.msg)
             self.handshake = self.msg[0]

             if len(self.msg) == 23 and self.handshake == "9":
                 self.ertime = rospy.Time.now()
                 self.seq = int(self.msg[1])
                 self.dt = float(self.msg[2]) / 1000.0 # seconds
                 self.yaw = float(self.msg[3]) # en degrees
                 self.asqrt = float(self.msg[4]) # mediam of acc axis
                 self.v_l = float(self.msg[5]) # m/s
                 self.v_r = float(self.msg[6])
                 self.ceil = float(self.msg[7]) / 1000.0 # m
                 self.ir_l = float(self.msg[8]) / 100.0 # m
                 self.ir_r = float(self.msg[9])
                 self.aX = float(self.msg[10])
                 self.aY = float(self.msg[11])
                 self.aZ = float(self.msg[12])
                 self.gX = float(self.msg[13])
                 self.gY = float(self.msg[14])
                 self.gZ = float(self.msg[15])
                 self.mX = float(self.msg[16])
                 self.mY = float(self.msg[17])
                 self.mZ = float(self.msg[18])
                 self.temp = float(self.msg[19])

                 self.state = NiubotState()
                 self.state.header.stamp = self.ertime
                 self.state.header.frame_id = "base_link"
                 self.state.header.seq = self.seq
                 self.state.dt = self.dt
                 self.state.yaw = self.yaw
                 self.state.phi = self.phi
                 self.state.asqrt = self.asqrt
                 self.state.vel_left = self.v_l
                 self.state.vel_right = self.v_r
                 self.state.pwm_l = self.pwm_l
                 self.state.pwm_r = self.pwm_r
                 self.state.servo = self.servo
                 self.state.temp = self.servo

                 self.ir_l_range = Range()
                 self.ir_l_range.header.stamp = self.ertime
                 self.ir_l_range.header.frame_id = "ir_l_link"
                 self.ir_l_range.header.seq = self.seq
                 self.ir_l_range.radiation_type =  self.ir_l_range.INFRARED # ULTRASOUND
                 self.ir_l_range.field_of_view = self.fov_ir
                 self.ir_l_range.min_range = self.min_range_ir
                 self.ir_l_range.max_range = self.max_range_ir
                 self.ir_l_range.range = self.ir_l # en m

                 self.ir_r_range = Range()
                 self.ir_r_range.header.stamp = self.ertime
                 self.ir_r_range.header.frame_id = "ir_r_link"
                 self.ir_r_range.header.seq = self.seq
                 self.ir_r_range.radiation_type =  self.ir_r_range.INFRARED # ULTRASOUND
                 self.ir_r_range.field_of_view = self.fov_ir
                 self.ir_r_range_range.min_range = self.min_range_ir
                 self.ir_r_range.max_range = self.max_range_ir
                 self.ir_r_range_range.range = self.ir_r # en m

                 self.ceil_range = Range()
                 self.ceil_range.header.stamp = self.ertime
                 self.ceil_range.header.frame_id = "ceil_link"
                 self.ceil_range.header.seq = self.seq
                 self.ceil_range.radiation_type =  self.ir_r_range.INFRARED # ULTRASOUND
                 self.ceil_range.field_of_view = self.fov_ceil
                 self.ceil_range_range.min_range = self.min_range_ceil
                 self.ceil_range.max_range = self.max_range_ceil
                 self.ceil_range_range.range = self.ceil # en m

                 self.imu = Imu()
                 self.imu.header.stamp = self.ertime
                 self.imu.header.frame_id = "imu_link"
                 self.imu.header.seq = self.seq

                 q = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(self.yaw))
                 orientation = Quaternion(*q)
                 linear_acceleration = Vector3()
                 linear_acceleration.x = self.aX
                 linear_acceleration.y = self.aY
                 linear_acceleration.z = self.aZ
                 angular_velocity = Vector3()
                 angular_velocity.x = self.gX
                 angular_velocity.y = self.gY
                 angular_velocity.z = self.gZ
                 self.imu.orientation = orientation # quaternion
                 self.imu.linear_acceleration = linear_acceleration # m/s2
                 self.imu.angular_velocity = angular_velocity # rad/s
                 #self.imu.orientation_covariance = self.orientation_covariance
                 #self.imu.linear_acceleration_covariance = self.linear_acceleration_covariance
                 #self.imu.angular_velocity_covariance = self.angular_velocity_covariance

                 self.imu_pub.publish(self.imu)
                 self.ceil_pub.publish(self.ceil_range)
                 self.ir_l_pub.publish(self.ir_l_range)
                 self.ir_r_pub.publish(self.ir_r_range)
                 self.state_pub.publish(self.state)

                 self.update_odometry()

                 # since all odometry is 6DOF we'll need a quaternion created from yaw
                 self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
                 self.odom_quat  = np.asarray(self.odom_quat)

                 #we'll publish the transform over tf
                 self.odom_broadcaster.sendTransform((self.x,self.y,0.0),
                                                         self.odom_quat,
                                                         self.ertime,
                                                         "base_footprint",
                                                         "odom")

                 # next, we'll publish the odometry message over ROS
                 self.odom.header.stamp = self.ertime
                 self.odom.header.frame_id = "odom"
                 self.odom.child_frame_id = "base_footprint"
                 self.odom.header.seq = self.seq

                 # set the position
                 self.odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*self.odom_quat))
                 #self.odom.pose.covariance = self.pose_covariance

                 # set the velocity
                 self.odom.twist.twist = Twist(Vector3(self.xdot, self.ydot, 0.0), Vector3(0.0, 0.0, self.thetadot))
                 #self.odom.twist.covariance = self.twist_covariance

                 # publish the message
                 self.odom_pub.publish(self.odom)

                 self.last_time = self.ertime

             else:
                 self._Serial.flush()
                 print("arduino no transmit good data -> flush", self.msg)
# Odometry
    def update_odometry(self):
        # Compute odometry here
        right_dt = self.v_r * self.dt
        left_dt = self.v_l * self.dt
        center_dt = (left_dt + right_dt) / 2.0

        self.phi = (right_dt - left_dt) / self.eje # ojo phi = (right_dt - left_dt) / self.eje

        x_dt = center_dt * math.cos(self.theta + self.phi) # ojo x_dt = center_dt * math.cos(phi)
        y_dt = center_dt * math.sin(self.theta + self.phi) # ojo y_dt = center_dt * math.sin(phi)--

        self.x = self.x + x_dt
        self.y = self.y + y_dt
        self.theta = self.theta + self.phi

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.xdot = x_dt / self.dt
        self.ydot = y_dt / self.dt
        self.thetadot = phi / self.dt

    def uni_to_diff(self, v, w):
        vr = ((2.0 * v) + (self.eje * w)) / (2.0 * self.radio)
        vl = ((2.0 * v) - (self.eje * w)) / (2.0 * self.radio)
        return vl, vr

    def Write(self, data):
        #info = "Writing to serial port: %s" %data
        #rospy.loginfo(info)
        self._Serial.write(data)

    def callback(self,msg):
        #rospy.loginfo("Received a /cmd_vel message!")
        self.v = msg.linear.x
        self.w = msg.angular.z

        vl, vr = self.uni_to_diff(self.v, self.w)

        self.pwm_l = int(vl * 80) # 0.5 en cmd 40 pwm
        self.pwm_r = int(vr * 80)

        if self.pwm_l > 48: # y lo maximo es 48
            self.pwm_l = 48
        elif self.pwm_l < -48:
            self.pwm_l = -48

        if self.pwm_r > 48:
            self.pwm_r = 48
        elif self.pwm_r < -48:
            self.pwm_r = -48

        caden = str(self.servo)+","+str(self.pwm_l)+","+str(self.pwm_r)+"\n"
        #rospy.loginfo(caden)
        self.Write(str.encode(caden))

    def listener(self):
        rospy.Subscriber("cmd_vel", Twist, self.callback)
        rospy.spin()

if __name__ == '__main__':
    niubotSess = NiubotBringUp()
    niubotSess.Start()
    niubotSess.listener()

    #raw_input("Hit <Enter> to end.")
    time.sleep(1.0)
    while niubotSess._KeepRunning:
        pass

    niubotSess.Stop()
