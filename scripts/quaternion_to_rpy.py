#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion

class QuaternionToRpy:
    def __init__(self):
        rospy.init_node("quaternion_to_rpy", anonymous=True)
        rospy.Subscriber( "batting_mocap/pose", PoseStamped, self.calc_rpy_callback)
        self.pub_rpy = rospy.Publisher("batting/xyzrpy", Twist, queue_size=10)
        self.r = rospy.Rate(10) # 10hz
        self.count = 1

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def calc_rpy_callback(self, msg):
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        rospy.loginfo("roll = %f, pitch = %f, yaw = %f", roll, pitch, yaw)

        rpy_msg = Twist()
        rpy_msg.linear.x = msg.pose.position.x
        rpy_msg.linear.y = msg.pose.position.y
        rpy_msg.linear.z = msg.pose.position.z
        rpy_msg.angular.x = roll
        rpy_msg.angular.y = pitch
        rpy_msg.angular.z = yaw
        self.pub_rpy.publish(rpy_msg)

        x = rpy_msg.linear.x * 1000
        y = rpy_msg.linear.y * 1000
        z = rpy_msg.linear.z * 1000

        fp = open("/tmp/memo_batting.txt", "a")
        str = "(setq *head-coords%d* (make-cascoords :pos (float-vector %f %f %f) :rpy (list %f %f %f)))\n" % (self.count, x, y, z, yaw, pitch, roll)
        print str
        fp.write(str)
        fp.close()
        self.count += 1


if __name__ == '__main__':
    try:
        node = QuaternionToRpy()
        node.execute()
    except rospy.ROSInterruptException: pass
