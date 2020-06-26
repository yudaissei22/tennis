#!/usr/bin/env python
import rospy
from roseus.srv import StringString
import std_msgs


class Ball:
    def __init__(self):
        pass
    # x, y, z: initial position
    # fx, fy, fz: external force in world frame
    # tm: time to apply external force to the ball
    def fly(self, x = 20.0, y = -2.0, z = 1.0, fx = -10, fy = 0.6, fz = 8.3, tm = 0.073):
        rospy.wait_for_service('choreonoid_service')
        choreonoid_service = rospy.ServiceProxy('choreonoid_service', StringString)
        res = choreonoid_service.call(str="resetPosition('TennisBall', [{}, {}, {}], [0, 0, 0], 0.2)".format(x, y, z))
        print("{}".format(res.str))
        res = choreonoid_service.call(str="addExternalForce('TennisBall', 'TennisBall', [0.0, 0.0, 0.0], [{}, {}, {}], {})".format(fx, fy, fz, tm))
        print("{}".format(res.str))

ball = Ball()
ball.fly()
