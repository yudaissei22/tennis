#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Vector3

class PoseToVelocity:
    def __init__(self):
        rospy.init_node("pose_to_velocity", anonymous=True)
        rospy.Subscriber( "batting_mocap/pose", PoseStamped, self.calc_vel_callback)
        self.pub_velocity = rospy.Publisher("batting/velocity", Vector3, queue_size=10)
        self.pub_speed = rospy.Publisher("batting/speed", Float64, queue_size=10)
        self.r = rospy.Rate(10) # 10hz
        self.old_pos = None
        self.old_t = None

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def calc_vel_callback(self, msg):
        if self.old_pos == None:
            self.old_pos = Vector3()
            self.old_pos.x = msg.pose.position.x
            self.old_pos.y = msg.pose.position.y
            self.old_pos.z = msg.pose.position.z
            self.old_t = msg.header.stamp.secs + float(msg.header.stamp.nsecs)/1000000000
            rospy.loginfo("old_t = %f", self.old_t)
            return

        new_pos = Vector3()
        new_pos.x = msg.pose.position.x
        new_pos.y = msg.pose.position.y
        new_pos.z = msg.pose.position.z
        new_t = msg.header.stamp.secs + float(msg.header.stamp.nsecs)/1000000000

        dx = new_pos.x - self.old_pos.x
        dy = new_pos.y - self.old_pos.y
        dz = new_pos.z - self.old_pos.z
        dt = new_t - self.old_t
        if dx == 0 or dy == 0 or dz == 0:
            return
        rospy.loginfo("dx = %f, dy = %f, dz = %f\ndt = %f", dx, dy, dz, dt)

        vel_msg = Vector3()
        vel_msg.x = dx / dt
        vel_msg.y = dy / dt
        vel_msg.z = dz / dt
        self.pub_velocity.publish(vel_msg)

        speed_msg = Float64()
        speed_msg.data = numpy.sqrt(vel_msg.x*vel_msg.x + vel_msg.y*vel_msg.y + vel_msg.z*vel_msg.z)
        self.pub_speed.publish(speed_msg)

        self.old_pos.x = new_pos.x
        self.old_pos.y = new_pos.y
        self.old_pos.z = new_pos.z
        self.old_t = new_t

if __name__ == '__main__':
    try:
        node = PoseToVelocity()
        node.execute()
    except rospy.ROSInterruptException: pass
