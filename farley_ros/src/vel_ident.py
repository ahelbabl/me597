#!/usr/bin/python
#
# Implements a simple proportional velocity controller, for use in drive
# system identifcation, for more sophisticated velocity control design.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy

from geometry_msgs.msg import Twist
from clearpath_horizon.msg import RawEncoders

rospy.init_node('vel_ident')

class Controller:
  def __init__(self):
    self.cmdPub = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
    self.rate = rospy.Rate(20);

    # Proportional control constant
    self.kp = 0.5;

    # Reference and control signals:
    self.velRef = 0.0  # [tics/s]
    self.velCtrl = 0.0 # [-100,100]

    # Velocity measurement state:
    self.lastMsmtTime = 0 # [ns]
    self.lastTicks = 0    # [ticks]

  def _publish():
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
    if self.lastMsmt == 0:
      # First measurement, have no vel data.
      self.lastMsmtTime = rospy.get_rostime().nsecs
      self.lastTicks = ticks
      return

    # Calculate current speed:
    curTime = rospy.get_rostime().nsecs
    dt = 0.000001 * (curTime - self.lastMsmtTime) #[ns->s]
    vel = float(ticks - self.lastTicks) / dt

    # Calculate and apply control signal:
    self.velCtrl = kp * (self.velRef - self.velCtrl)
    self._publish()
    print("y: {0}, u:{1}".format(vel, self.velCtrl))
    
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


