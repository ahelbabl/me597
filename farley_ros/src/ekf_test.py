#!/usr/bin/python
#
# Sandbox script for EKF testing.

import roslib; roslib.load_manifest('farley_ros')
import rospy

from controller_estimator import ControllerEstimator

rospy.init_node('ekf_test')

c = ControllerEstimator()
c.setVelocity(0.0)
c.setSteeringAngle(0.0)
c.start()

rospy.spin()

