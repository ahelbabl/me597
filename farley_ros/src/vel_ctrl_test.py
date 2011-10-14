#!/usr/bin/python
#
# Test the velocity controller.
# Just runs with a constant setpoint.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy

from geometry_msgs.msg import Twist

from velocity_control import VelocityControl
import os

rospy.init_node('vel_ident')

c = VelocityControl()
c.setVelocity(0.2);
c.execute()

