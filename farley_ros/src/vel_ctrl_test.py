#!/usr/bin/python
#
# Test the velocity controller.
# Just runs with a constant setpoint.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy

from geometry_msgs.msg import Twist

from velocity_control import VelocityControl
import os
import matplotlib.pyplot as p

rospy.init_node('vel_ident')

c = VelocityControl()
c.setVelocity(0.2);

rospy.sleep(2.0)
c.start();
rospy.sleep(120.0);
c.stop();
p.plot(c.velRecord)
p.show()
rospy.spin()
