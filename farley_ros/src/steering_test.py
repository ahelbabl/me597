#!/usr/bin/python
#
# A simple steering controller test script.

import roslib; roslib.load_manifest('farley_ros')
import rospy

from steering_control import SteeringControl

rospy.init_node('steering_test')

c = SteeringControl()
c.setSteeringAngle(0.0)
c.setVelocity(0.0)
c.start()

rospy.spin()
