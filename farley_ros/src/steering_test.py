#!/usr/bin/python
#
# A simple steering controller test script.

import roslib; roslib.load_manifest('farley_ros')
import rospy

from steering_control import SteeringControl

import math

rospy.init_node('steering_test')

rate = rospy.Rate(20)

c = SteeringControl()
c.setVelocity(0.1)
rospy.sleep(2)  # Give horizon some time to start

c.setWaypoint(0.5,-0.5,0)
c.start()

# Wait for real pose data to be received
while (c.curPose is None) and not rospy.is_shutdown():
  rate.sleep()

# Wait to cross the waypoint x value
while not rospy.is_shutdown() and c.curPose.X < 0.5:
  rate.sleep()

c.setWaypoint(0.5, 0.5, math.pi/2)
while not rospy.is_shutdown() and c.curPose.Y < 0.5:
  rate.sleep()

c.setWaypoint(-0.5, 0.5, math.pi)
while not rospy.is_shutdown() and c.curPose.X > -0.5:
  rate.sleep()

c.setWaypoint(-0.5,-0.5, -math.pi/2)
while not rospy.is_shutodwn() and c.curPose.Y > -0.5:
  rate.sleep()

c.stop()
print("Test Complete!")
rospy.spin()
