#!/usr/bin/python
#
# Provides velocity control for the robot.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy
from geometry_msgs.msg import Twist

class VelocityControl:
  def __init__(self):
    self.cmdPub = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
    self.rate = rospy.Rate(20);
    self.spd = Speedometer(self._velocityCb)

    # Proportional control constant
    self.kp = 0.002;
    self.ki = 0.001; 
    self.integrator = 0.0;

    # Reference and control signals:
    self.velRef = 2000.0  # [tics/s]
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

  def execute(self):
    """ Executes the control loop. """
    # Vel cmd must be sent regularly, even if not changed (due to dead-man)
    while not rospy.is_shutdown():
      self.rate.sleep()
      self._publish()

  def _velocityCb(self, vel, time):
    """ Accepts new measurement data, and adjusts the control signal """
    # Calculate and apply control signal:
    p = self.kp * (self.velRef - vel)
    self.integrator += self.ki * (self.velRef - vel)
    self.integrator = self._saturate(self.integrator, 100)
    self.velCtrl = self._saturate(p + self.integrator, 100);

    self._publish()
    return
