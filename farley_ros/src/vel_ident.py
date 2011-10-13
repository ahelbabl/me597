#!/usr/bin/python
#
# Implements a simple proportional velocity controller, for use in drive
# system identifcation, for more sophisticated velocity control design.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy

from geometry_msgs.msg import Twist
from clearpath_horizon.msg import RawEncoders

rospy.init_node('vel_ident')

def saturate(val, limit):
  """ Returns val, saturated into [-limit,+limit] """
  if val > limit:
     return limit
  if val < -limit:
     return -limit
  return val

class Controller:
  def __init__(self):
    self.cmdPub = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
    self.rate = rospy.Rate(20);

    # Proportional control constant
    self.kp = 0.002;
    self.ki = 0.001; 
    self.integrator = 0.0;

    # Reference and control signals:
    self.velRef = 2000.0  # [tics/s]
    self.velCtrl = 0.0 # [-100,100]

    # Velocity measurement state:
    self.lastMsmtTime = None 
    self.lastTicks = 0 # [ticks]

  def _publish(self):
    """ Publishes the current control signal """
    velCmd = Twist()
    velCmd.linear.x = self.velCtrl
    self.cmdPub.publish(velCmd)

  def execute(self):
    """ Executes the control loop. """
    # Vel cmd must be sent regularly, even if not changed (due to dead-man)
    while not rospy.is_shutdown():
      self.rate.sleep()
      self._publish()

  def measurement(self, ticks):
    """ Accepts new measurement data, and adjusts the control signal """
    if self.lastMsmtTime is None:
      # First measurement, have no vel data.
      self.lastMsmtTime = rospy.get_rostime()
      self.lastTicks = ticks
      return

    # Calculate current speed:
    curTime = rospy.get_rostime()
    dt = (curTime - self.lastMsmtTime).to_sec()
    vel = float(ticks - self.lastTicks) / dt

    # Calculate and apply control signal:
    p = self.kp * (self.velRef - vel)
    self.integrator += self.ki * (self.velRef - vel)
    self.integrator = saturate(self.integrator, 100)
    self.velCtrl = saturate(p + self.integrator, 100);

    self._publish()
    print("dt: {0}, y: {1}, u:{2}".format(dt, vel, self.velCtrl))
    
    self.lastMsmtTime = curTime
    self.lastTicks = ticks
    return

c = Controller()

def encodersCb(encoders):
  if len(encoders.ticks) != 1:
    raise Exception("Robot should only have 1 encoder, but found {0}".
        format(len(encoders.ticks)))
  c.measurement(encoders.ticks[0])

rospy.Subscriber('/clearpath/robots/default/data/raw_encoders', 
    RawEncoders, encodersCb)

c.execute()

