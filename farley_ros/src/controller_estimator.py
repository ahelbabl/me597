#!/usr/bin/python
#
# A monolithic velocity + steering controller and EKF state estimator.
# All of these functions are combined due to the tight relationship
# between estimation and control, in terms both of timing and information
# sharing.

import math
import numpy as np
import os

import roslib; roslib.load_manifest('farley_ros')
import rospy
from geometry_msgs.msg import Twist
from indoor_pos.msg import ips_msg

from speedometer import Speedometer

class ControllerEstimator:
  def __init__(self, kp=143.9, ki=857.8, ts=0.05, kSteer=0.5, ts=0.05, speed=0.20):
    self.cmdPub = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
    self.spd = Speedometer(self._velocityCb)
    rospy.Subscriber('indoor_pos', ips_msg, self._poseCb)

    self.running = False
    self.ts = ts

    # Proportional control constants
    self.kp = kp;
    self.ki = ki; 
    self.integrator = 0.0;
    self.lastErr = 0.0;  # (needed for trapezoidal integrator)

    # Reference and control signals:
    self.velRef = 0.0  # [m/s]
    self.velCtrl = 0.0 # [-100,100]

    # Proportional control constant for steering angle.
    self.kSteer = kSteer

    # Next waypoint
    self.xRef = 0.0
    self.yRef = 0.0
    self.hRef = 0.0

    # Steering reference and control signals: 
    self.steerCtrl = 0.0  # [-100,100]

    # Record pose data for inspectimification
    self.velRecord = np.array([0])
    self.xRecord = np.array([0])
    self.yRecord = np.array([0])

  def setVelocity(self, ref):
    """ Set the velocity reference """
    self.velRef = ref

  def setSteeringAngle(self, ang):
    """ Set the steering angle to ang (radian) """
    out = 266.7 * ang - 7.0;
    self.steerCtrl = self._saturate(out, 100)
   
  def setWaypoint(self, x, y, head):
    """ Set the target waypoint """
    self.xRef = x
    self.yRef = y
    self.hRef = head

  def start(self):
    self.running = True

  def stop(self):
    self.running = False

  def _publish(self):
    """ Publishes the current control signals """
    if not self.running:
        return
    velCmd = Twist()
    velCmd.linear.x = self.velCtrl
    velCmd.angular.z = self.steerCtrl
    self.cmdPub.publish(velCmd)

  def _saturate(self, val, limit):
    """ Returns val, saturated into [-limit,+limit] """
    if val > limit:
       return limit
    if val < -limit:
       return -limit
    return val

  def _velocityCb(self, vel, time):
    """ Accepts new measurement data, and adjusts the control signal """
    if not self.running:
        return

    self.velRecord = np.concatenate((self.velRecord, np.array([vel])))

    # Calculate the control signal:
    err = self.velRef - vel
    p = self.kp * err

    self.integrator += self.ki * 0.5 * self.ts * (self.lastErr + err)
    self.integrator = self._saturate(self.integrator, 100)

    self.velCtrl = self._saturate(p + self.integrator, 100)

    self.lastErr = err
    self._publish()
    return

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
    if not self.running:
        return

    self.xRecord = np.concatenate((self.xRecord, np.array([pose.X])))
    self.yRecord = np.concatenate((self.yRecord, np.array([pose.Y])))
    self.outfile.write('{0} {1} {2} {3}\n'.format(
          pose.header.stamp.to_sec(), pose.X, pose.Y, pose.Yaw))
   
    # Calculate heading error
    ang = (pose.Yaw * math.pi / 180)
    eHead = self.hRef - ang
    
    # Calculate crosstrack error.  Derived by wizards.
    ex = (self.xRef - pose.X) * (1 - math.cos(ang)*math.cos(ang))
    ey = (self.yRef - pose.Y) * (1 - math.sin(ang)*math.sin(ang))
    eCrosstrack = math.sqrt(ex*ex + ey*ey)
    # The sign of the z component of cross(target heading, crosstrack direction)
    # give the sign of the steering angle required to correct crosstrack.
    sign = math.cos(self.hRef)*ey - math.sin(self.hRef)*ex
    sign = sign / abs(sign)
    eCrosstrack = sign * eCrosstrack

    # Desired steering angle: 
    delta = eHead + math.atan2(self.kSteer * eCrosstrack, self.velRef)

    #print("Pose: x: {0}, y: {1}, h: {2}, eh: {3}".format(pose.X, pose.Y, pose.Yaw,eHead)) 
    self.setSteeringAngle(self._wrapAngle(delta))

    # Don't publish - we'll leave that to the velocity control
    # (Don't want to try to update too frequently)
    return

