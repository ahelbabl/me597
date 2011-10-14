#!/usr/bin/python
#
# Implements a good PI controller for the robot.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy
from geometry_msgs.msg import Twist

from speedometer import Speedometer

class VelocityControl:
  def __init__(self, kp=143.9, ki=100, rate=20):
    """ Default Kp and Ki have been designed with Simulink for a good 
        step response. """
    self.cmdPub = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
    self.rate = rospy.Rate(rate)
    self.spd = Speedometer(self._velocityCb)

    # Proportional control constant
    self.kp = kp;
    self.ki = ki; 
    self.integrator = 0.0;
    self.lastErr = 0.0;  # (needed for trapezoidal integrator)

    # Reference and control signals:
    self.velRef = 0.0  # [m/s]
    self.velCtrl = 0.0 # [-100,100]

  def _publish(self):
    """ Publishes the current control signal """
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

  def setVelocity(self, ref):
    """ Set the velocity reference """
    self.velRef = ref

  def execute(self):
    """ Executes the control loop. """
    # Vel cmd must be sent regularly, even if not changed (due to dead-man)
    while not rospy.is_shutdown():
      self.rate.sleep()
      self._publish()

  def _velocityCb(self, vel, time):
    """ Accepts new measurement data, and adjusts the control signal """
    # Calculate the control signal:
    err = self.velRef - vel
    p = self.kp * err

    self.integrator += 0.5 * self.ki * (self.lastErr + err)
    self.integrator = self._saturate(self.integrator, 100)

    self.velCtrl = self._saturate(p + self.integrator, 100)

    print("v: {0}, e: {4}, p: {1}, i: {2}, u: {3}".format(
        vel, p, self.integrator, self.velCtrl, err))

    self.lastErr = err
    self._publish()
    return
