#! /usr/bin/env python
'''
Created on November 30, 2019

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
from niubot_core.msg import NiubotWheels

def _OnLineReceived(line):
    print(line)

class NiubotBringUp(object):
    '''
    Helper class for receiving lines from a serial port
    '''
# ttyUSB1
    def __init__(self, port="/dev/ttyS0", baudrate=115200, lineHandler = _OnLineReceived):
        '''
        Initializes the receiver class.
        port: The serial port to listen to.
        receivedLineHandler: The function to call when a line was received.
        '''
         # robot constants
        self.radio = 0.03
        self.eje = 0.162
        self.ticks_per_rev = 60.0

        self.m_per_tick = (2.0 * math.pi * self.radio) / self.ticks_per_rev
        self.rad_per_tick = (2.0 * math.pi) / self.ticks_per_rev

        # vars
        self.t = 0.0 # s
        self.dt = 0.0000001 # s

        self.count_left = 0  # ticks
        self.count_right = 0  # ticks

        self.dir_left = 1  # 1-forward -1-backward 0-Stop
        self.dir_right = 1  # 1-forward -1-backward 0-Stop

        self.pwm_left = 0  # -254 a 254
        self.pwm_right = 0  # -254 a 254

        self.vl = 0.0  # m/s
        self.vr = 0.0  # m/s

        self.yaw = 0.0

        self.sonar0 = 0.0
        self.sonar1 = 0.0
        self.sonar2 = 0.0
        self.sonar3 = 0.0
        self.sonar4 = 0.0

        self.aX = 0.0
        self.aY = 0.0
        self.aZ = 0.0
        self.gX = 0.0
        self.gY = 0.0
        self.gZ = 0.0
        self.mX = 0.0
        self.mY = 0.0
        self.mZ = 0.0

        self.fov = 0.5;
        self.min_range = 0.02
        self.max_range = 1.20

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.xdot = 0.0
        self.ydot = 0.0
        self.thetadot = 0.0

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

        self.wheels_pub = rospy.Publisher('/niubot/wheels', NiubotWheels, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        self.sonar0_pub = rospy.Publisher('/niubot/sonar0', Range, queue_size=10)
        self.sonar1_pub = rospy.Publisher('/niubot/sonar1', Range, queue_size=10)
        self.sonar2_pub = rospy.Publisher('/niubot/sonar2', Range, queue_size=10)
        self.sonar3_pub = rospy.Publisher('/niubot/sonar3', Range, queue_size=10)
        self.sonar4_pub = rospy.Publisher('/niubot/sonar4', Range, queue_size=10)
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
                 self.dir_left = int(self.msg[5])
                 self.dir_right = int(self.msg[6])
                 self.count_left = int(self.msg[7])
                 self.count_right = int(self.msg[8])

                 self.sonar0 = float(self.msg[9])
                 self.sonar1 = float(self.msg[10])
                 self.sonar2 = float(self.msg[11])
                 self.sonar3 = float(self.msg[12])
                 self.sonar4 = float(self.msg[13])
                 if self.sonar0 > 1.2:
                     self.sonar0 = 1.2
                 if self.sonar1 > 1.2:
                     self.sonar1 = 1.2
                 if self.sonar2 > 1.2:
                     self.sonar2 = 1.2
                 if self.sonar3 > 1.2:
                     self.sonar3 = 1.2
                 if self.sonar4 > 1.2:
                     self.sonar4 = 1.2

                 if self.sonar0 < 0.02:
                     self.sonar0 = 1.2
                 if self.sonar1 < 0.02:
                     self.sonar1 = 1.2
                 if self.sonar2 < 0.02:
                     self.sonar2 = 1.2
                 if self.sonar3 < 0.02:
                     self.sonar3 = 1.2
                 if self.sonar4 < 0.02:
                     self.sonar4 = 1.2

                 self.aX = float(self.msg[14])
                 self.aY = float(self.msg[15])
                 self.aZ = float(self.msg[16])
                 self.gX = float(self.msg[17])
                 self.gY = float(self.msg[18])
                 self.gZ = float(self.msg[19])
                 self.mX = float(self.msg[20])
                 self.mY = float(self.msg[21])
                 self.mZ = float(self.msg[22])

                 self.wheels = NiubotWheels()
                 self.wheels.header.stamp = self.ertime
                 self.wheels.header.frame_id = "base_link"
                 self.wheels.header.seq = self.seq
                 self.wheels.dt = self.dt
                 self.wheels.dir_left = self.dir_left
                 self.wheels.dir_right = self.dir_right
                 self.wheels.count_left = self.count_left
                 self.wheels.count_right = self.count_right
                 self.wheels.vel_left = (self.dir_left * self.count_left * self.m_per_tick) / self.dt
                 self.wheels.vel_right = (self.dir_right * self.count_right * self.m_per_tick) / self.dt
                 self.wheels.pwm_left = self.pwm_left
                 self.wheels.pwm_right = self.pwm_right

                 self.sonar0_range = Range()
                 self.sonar0_range.header.stamp = self.ertime
                 self.sonar0_range.header.frame_id = "sonar0_link"
                 self.sonar0_range.header.seq = self.seq
                 self.sonar0_range.radiation_type = 0 # ULTRASOUND
                 self.sonar0_range.field_of_view = self.fov
                 self.sonar0_range.min_range = self.min_range
                 self.sonar0_range.max_range = self.max_range
                 self.sonar0_range.range = self.sonar0 # en m

                 self.sonar1_range = Range()
                 self.sonar1_range.header.stamp = self.ertime
                 self.sonar1_range.header.frame_id = "sonar1_link"
                 self.sonar1_range.header.seq = self.seq
                 self.sonar1_range.radiation_type = 0 # ULTRASOUND
                 self.sonar1_range.field_of_view = self.fov
                 self.sonar1_range.min_range = self.min_range
                 self.sonar1_range.max_range = self.max_range
                 self.sonar1_range.range = self.sonar1 # en m

                 self.sonar2_range = Range()
                 self.sonar2_range.header.stamp = self.ertime
                 self.sonar2_range.header.frame_id = "sonar2_link"
                 self.sonar2_range.header.seq = self.seq
                 self.sonar2_range.radiation_type = 0 # ULTRASOUND
                 self.sonar2_range.field_of_view = self.fov
                 self.sonar2_range.min_range = self.min_range
                 self.sonar2_range.max_range = self.max_range
                 self.sonar2_range.range = self.sonar2 # en m

                 self.sonar3_range = Range()
                 self.sonar3_range.header.stamp = self.ertime
                 self.sonar3_range.header.frame_id = "sonar3_link"
                 self.sonar3_range.header.seq = self.seq
                 self.sonar3_range.radiation_type = 0 # ULTRASOUND
                 self.sonar3_range.field_of_view = self.fov
                 self.sonar3_range.min_range = self.min_range
                 self.sonar3_range.max_range = self.max_range
                 self.sonar3_range.range = self.sonar3 # en m

                 self.sonar4_range = Range()
                 self.sonar4_range.header.stamp = self.ertime
                 self.sonar4_range.header.frame_id = "sonar4_link"
                 self.sonar4_range.header.seq = self.seq
                 self.sonar4_range.radiation_type = 0 # ULTRASOUND
                 self.sonar4_range.field_of_view = self.fov
                 self.sonar4_range.min_range = self.min_range
                 self.sonar4_range.max_range = self.max_range
                 self.sonar4_range.range = self.sonar4 # en m

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
                 self.sonar0_pub.publish(self.sonar0_range)
                 self.sonar1_pub.publish(self.sonar1_range)
                 self.sonar2_pub.publish(self.sonar2_range)
                 self.sonar3_pub.publish(self.sonar3_range)
                 self.sonar4_pub.publish(self.sonar4_range)
                 self.wheels_pub.publish(self.wheels)

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
        right_dt = self.count_right * self.dir_right * self.m_per_tick
        left_dt = self.count_left * self.dir_left * self.m_per_tick
        center_dt = (left_dt + right_dt) / 2.0

        phi = (right_dt - left_dt) / self.eje # ojo phi = (right_dt - left_dt) / self.eje

        x_dt = center_dt * math.cos(self.theta + phi) # ojo x_dt = center_dt * math.cos(phi)
        y_dt = center_dt * math.sin(self.theta + phi) # ojo y_dt = center_dt * math.sin(phi)--

        self.x = self.x + x_dt
        self.y = self.y + y_dt
        self.theta = self.theta + phi

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

        self.vl, self.vr = self.uni_to_diff(self.v, self.w)
        #cost_vl = self.vl - self.wheels.vel_left
        #cost_vr = self.vr - self.wheels.vel_right
        #self.pwm_left -= int(cost_vl * 10)
        #self.pwm_right -= int(cost_vr * 10)
        self.pwm_left = int(self.vl * 10)
        self.pwm_right = int(self.vr * 10)

        if self.pwm_left > 126:
            self.pwm_left = 126
        elif self.pwm_left < -126:
            self.pwm_left = -126

        if self.pwm_right > 126:
            self.pwm_right = 126
        elif self.pwm_right < -126:
            self.pwm_right = -126

        caden = "0,"+str(self.pwm_left)+","+str(self.pwm_right)+"\n"
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
