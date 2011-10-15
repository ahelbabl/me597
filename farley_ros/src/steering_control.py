#!/usr/bin/bash
#
# Implements a nonlinear steering controller, extending VelocityController for 
# speed control functionality.

import roslib
roslib.load_manifest('farley_ros')
import rospy
from indoor_pos.msg import ips_msg
from geometry_msgs.msg import Twist

from velocity_control import VelocityControl
import math

class SteeringControl(VelocityControl):
  def __init__(self, maxDelta=0.1, ts=0.05):
    VelocityControl.__init__(self, ts=ts)
    rospy.Subscriber('indoor_pos', ips_msg, self._poseCb)
    self.maxDelta = maxDelta

    # Next waypoint
    self.xRef = 0.0
    self.yRef = 0.0

    # Steering reference and control signals: 
    self.steerCtrl = 0.0  # [-100,100]

  def setWaypoint(self, x, y):
    """ Set the target waypoint """
    self.xRef = x
    self.yRef = y
   
  def _publish(self):
    """ Publishes the current control signals """
    if not self.running:
        return
    velCmd = Twist()
    velCmd.linear.x = self.velCtrl
    velCmd.angular.z = self.steerCtrl
    self.cmdPub.publish(velCmd)

  def _poseCb(self, pose):
    """ Update steering command using current pose """
    # Position error
    dx = self.xRef - pose.X
    dy = self.yRef - pose.Y

    # Heading to waypoint
    hRef = math.atan2(dy, dx)
    # Heading error
    dh = hRef - pose.Yaw
    
    # Ctrl signal is just to set steering angle to heading error.
    # Map radians -> steering power
    self.steerCtrl = self._saturate(dh, self.maxDelta)

    self._publish()
    return
