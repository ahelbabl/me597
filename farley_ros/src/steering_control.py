#!/usr/bin/bash
#
# Implements a nonlinear steering controller, extending VelocityController for 
# speed control functionality.

import roslib
roslib.load_manifest('farley_ros')
import rospy
from indoor_pos.msg import ips_msg

from velocity_control import VelocityControl

class SteeringControl(VelocityControl):
  def __init__(self, ts=0.05):
    VelocityControl.__init__(self, ts=ts)
    
  def _posCb(self):
    pass

