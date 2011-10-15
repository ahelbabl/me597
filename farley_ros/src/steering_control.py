#!/usr/bin/python
#
# Implements a good PI controller for the robot.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy
from geometry_msgs.msg import Twist

from velocity_control import VelocityControl

import numpy as np

class SteeringControl:
  def __init__(self, px,py velocity=0.2):
    self.cmdPub = rospy.Publisher('/clearpath/robots/default/cmd_steer', Twist)
    self.ts = ts

    #apply speed controller
    self.v = VelocityControl()
    self.v.setVelocity(velocity)
    self.velocity = velocity
    
    # set start position (remove this once we get position data
    self.px = px;
    self.py = py;

    # Reference and control signals:
    # Record data in np arrays for plotting 
#    self.velRecord = np.array([0])

  def setVelocity(self, ref):
    """ Set the velocity reference """
    self.v.setVelocity = ref

  def start(self):
    self.running = True
    self.v.start()

  def stop(self):
    self.running = False
    self.v.stop()

#need to understand how this works
  def _publish(self):
    """ Publishes the current control signal """
    if not self.running:
        return
    velCmd = Twist()
    velCmd.angular.z = self.delta
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
