#!/usr/bin/python
#
# Provides a class which subscribes to encoder data, which it uses
# to compute velocity.

import roslib; roslib.load_manifest('farley_ros')
import rospy
from clearpath_horizon.msg import RawEncoders

class Speedometer:
  def __init__(self, speedCb):
    """ Create a Speedometer.  Whenever encoder data is received,
        speedCb will be called with the current speed passed as the first param
        and the measurement time as the second parameter """
    self.speedCb = speedCb
    self.lastTime = None
    self.lastTicks = 0
    rospy.Subscriber('/clearpath/robots/default/data/raw_encoders',
        RawEncoders, self._encodersCb) 
    
    self.filt = [1.0/3, 1.0/3, 1.0/3]
    self.buf = [0, 0, 0]
  
  def _encodersCb(self, encoders):
    if len(encoders.ticks) != 1:
      raise Exception(
          "Expected 1 encoder, found {0}".format(len(encoders.ticks)))

    # First data point initializes, doesn't produce speed data.
    if self.lastTime is None:
      self.lastTime = encoders.header.stamp
      self.lastTicks = encoders.ticks[0]
      return

    # Conveniences:
    curTime = encoders.header.stamp
    curTicks = encoders.ticks[0]
  
    # Calculate current velocity 
    dt = (curTime - self.lastTime).to_sec()
    vel = float(curTicks - self.lastTicks) / dt
    vel = vel * 11.519E-6

    # Shift into filter buffer
    self.buf = [vel] + self.buf[:-1]
    filtered = 0.0
    for i in range(len(self.buf)):
      filtered += self.filt[i] * self.buf[i]

    self.speedCb(filtered, curTime)
    self.lastTime = curTime
    self.lastTicks = curTicks

