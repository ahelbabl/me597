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
  def __init__(self, maxDelta=0.1, ts=0.05, speed=0.20):
    VelocityControl.__init__(self, ts=ts)
    rospy.Subscriber('indoor_pos', ips_msg, self._poseCb)
    self.maxDelta = maxDelta
    self.speed = 0.20

    # Next waypoint
    self.xRef = 0.0
    self.yRef = 0.0
    self.hRef = 0.0

    # Steering reference and control signals: 
    self.steerCtrl = 0.0  # [-100,100]

  def setSteeringAngle(self, ang):
    """ Set the steering angle to ang (radian) """
    out = 266.7 * ang - 7.0;
    self.steerCtrl = self._saturate(out, 100)
   
  def setWaypoint(self, x, y, head):
    """ Set the target waypoint """
    self.xRef = x
    self.yRef = y
    self.hRef = head

  def _publish(self):
    """ Publishes the current control signals """
    if not self.running:
        return
    velCmd = Twist()
    velCmd.linear.x = self.velCtrl
    velCmd.angular.z = self.steerCtrl
    self.cmdPub.publish(velCmd)

  def _wrapAngle(self, ang):
    """ Wrap an angle (radian) to [-pi, pi] """
    while ang > math.pi:
        ang -= math.pi
    while ang < -math.pi:
        ang += math.pi
    return ang

  def _halt(self):
    self.setVelocity(0)
    self.setSteeringAngle(0)

  def _poseCb(self, pose):
    """ Update steering command using current pose """
    if (pose.X == 0) and (pose.Y == 0) and (pose.Yaw == 0):
      self._halt()
      print("Bad pose!")
      return
    if (pose.X > 10) and (pose.Y > 10) and (pose.Yaw > 500):
      self._halt()
      print("Bad pose!")
      return
    self.setVelocity(self.speed)
    print("Pose: x: {0}, y: {1}, h: {2}".format(pose.X, pose.Y, pose.Yaw))
   
    # Crosstrack err gain 
    gain=2.5

    # Calculate heading error
    ang = pose.Yaw * math.pi / 180
    eHead = self.hRef - ang # calculate this
    
    # Calculate crosstrack error.  Derived by wizards.
    ex = pose.X * (1 - math.cos(ang)*math.cos(ang))
    ey = pose.Y * (1 - math.sin(ang)*math.sin(ang))
    eCrosstrack = math.sqrt(ex*ex + ey*ey)
    # The sign of the z component of cross(target heading, crosstrack direction)
    # give the sign of the steering angle required to correct crosstrack.
    sign = cos(self.hRef)*ey - sin(self.hRef)*ex
    sign = sign / abs(sign)
    eCrosstrack = sign * eCrosstrack

    # Desired steering angle: 
    delta = eHead + math.atan2(gain * eCrosstrack, self.velRef)
    self.setSteeringAngle(self._wrapAngle(delta))

    # Don't publish - we'll leave that to the velocity control
    # (Don't want to try to update too frequently)
    return
