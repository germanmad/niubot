#!/usr/bin/env python

import rospy
#from std_msgs.msg import String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class Server:
    def __init__(self):
        self.vel_lin = rospy.get_param("vel_lin")
        self.vel_ang = rospy.get_param("vel_ang")
        self.avance_lapse = rospy.get_param('avance_lapse')
        self.giro_lapse = rospy.get_param('giro_lapse')
        self.limit = rospy.get_param('limit')

        self.ir_l = 0.0
        self.ir_r = 0.0

        self.vel_lin = 0.55
        self.vel_ang = 1.2
        self.giro_lapse = 1.0
        self.avance_lapse = 1.0
        self.limit = 1.8
        #print(self.limit)

    def get_ir_l(self,msg):
        self.ir_l = msg.ir_l
    def get_ir_r(self,msg):
        self.ir_r = msg.ir_r

    self.overlap()

    def overlap(self):
        twist = Twist()
        get_ir_l(self,msg)
        get_ir_r(self,msg)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0

        if (self.ir_l < self.limit) or (self.ir_r < self.limit):

        if self.ir_l < self.ir_r:
            twist.linear.x = 0.0
            twist.angular.z = -self.vel_ang
            self.pub.publish(twist)
            rospy.sleep(self.giro_lapse)
        else if self.ir_l > self.ir_r:
            twist.linear.x = 0.0
            twist.angular.z = self.vel_ang
            self.pub.publish(twist)
            rospy.sleep(self.giro_lapse)
        else:
            twist.linear.x = 0.55
            twist.angular.z = 0.0
            self.pub.publish(twist)
            rospy.sleep(self.avance_lapse)

def projection():
    rospy.init_node('avoid_explorer', anonymous=True)
    server = Server()
    server.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber("/niubot/ir_l", Range, server.ir_l)
    rospy.Subscriber("/niubot/ir_r", Range, server.ir_r)
    #rospy.Subscriber("/niubot/wheels", NiubotWheels, server.wheels)

    rospy.spin()

if __name__ == '__main__':
    projection()

# def sonars_explorer():
#     rospy.init_node('sonars_cmd', anonymous=True)
#     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     rospy.Subscriber("/niubot/sonar0", Range, callback_s0)
#     rospy.Subscriber("/niubot/sonar1", Range, callback_s1)
#     rospy.Subscriber("/niubot/imu", Imu, callback_imu)
#     rospy.Subscriber("/niubot/wheels", NiubotWheels, callback_wheels)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         #calculate twist with sonars
#         twist = Twist()
#         twist.linear.x = 0.3; twist.linear.y = 0; twist.linear.z = 0
#         twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
#         pub.publish(twist)
#         rate.sleep()
#
# def callback_s0(data):
#     sonars0= data.range
#     print("sonar0 - " + str(sonars0))
#     #rospy.loginfo(rospy.get_caller_id() + "range 0 is %s",num, data.range)
#
# def callback_s1(data):
#     sonars1= data.range
#     print("sonar1 - " + str(sonars1))
#     #rospy.loginfo(rospy.get_caller_id() + "range 0 is %s",num, data.range)
#
# def callback_imu(data):
#     orientation = data.orientation.z
#     print("orientacion" + str(orientation))
#     #rospy.loginfo(rospy.get_caller_id() + "orientation is %s", orientation)
#
# def callback_wheels(data):
#     wheel_l = data.vel_left
#     wheel_r = data.vel_right
#     print(wheel_l)
#     #rospy.loginfo(rospy.get_caller_id() + "wheel_l is %s", wheel_l)
#     #rospy.loginfo(rospy.get_caller_id() + "wheel_r is %s", wheel_r)
#
# if __name__ == '__main__':
#
#     try:
#         sonars_explorer()
#     except rospy.ROSInterruptException:
#         pass
