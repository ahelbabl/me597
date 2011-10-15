#!/usr/bin/python
#
# Implements a good PI controller for the robot.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy
from geometry_msgs.msg import Twist

from speedometer import Speedometer

import numpy as np

class VelocityControl:
  def __init__(self, kp=143.9, ki=857.8, ts=0.05):
    self.cmdPub = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
    self.ts = ts
    self.spd = Speedometer(self._velocityCb)
    self.running = False

    # Proportional control constant
    self.kp = kp;
    self.ki = ki; 
    self.integrator = 0.0;
    self.lastErr = 0.0;  # (needed for trapezoidal integrator)

    # Reference and control signals:
    self.velRef = 0.0  # [m/s]
    self.velCtrl = 0.0 # [-100,100]

    # Record data in np arrays for plotting 
    self.velRecord = np.array([0])

  def setVelocity(self, ref):
    """ Set the velocity reference """
    self.velRef = ref

  def start(self):
    self.running = True

  def stop(self):
    self.running = False

  def _publish(self):
    """ Publishes the current control signal """
    if not self.running:
        return
    velCmd = Twist()
    velCmd.linear.x = self.velCtrl
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

    print("v: {0}, e: {4}, p: {1}, i: {2}, u: {3}".format(
        vel, p, self.integrator, self.velCtrl, err))

    self.lastErr = err
    self._publish()
    return
