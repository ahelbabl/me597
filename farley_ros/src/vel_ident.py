#!/usr/bin/python
#
# Implements a simple proportional velocity controller, for use in drive
# system identifcation, for more sophisticated velocity control design.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy

from geometry_msgs.msg import Twist
from clearpath_horizon.msg import RawEncoders
from velocity_control import Controller

rospy.init_node('vel_ident')

c = Controller()

def encodersCb(encoders):
  if len(encoders.ticks) != 1:
    raise Exception("Robot should only have 1 encoder, but found {0}".
        format(len(encoders.ticks)))
  c.measurement(encoders.ticks[0])

rospy.Subscriber('/clearpath/robots/default/data/raw_encoders', 
    RawEncoders, encodersCb)

c.execute()

