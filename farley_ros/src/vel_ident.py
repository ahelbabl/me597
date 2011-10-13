#!/usr/bin/python
#
# Subjects the robot to a square wave, so that we can derive motor model constants
# empirically.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy

from geometry_msgs.msg import Twist
from clearpath_horizon.msg import RawEncoders

rospy.init_node('vel_ident')

def encodersCb(encoders):
  if len(encoders.ticks) != 1:
    raise Exception("Robot should only have 1 encoder, but found {0}".
        format(len(encoders.ticks)))

rospy.Subscriber('/clearpath/robots/default/data/raw_encoders', 
    RawEncoders, encodersCb)

cmdVel = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
startTime = rospy.get_rostime()
rate = rospy.Rate(20)

while not rospy.is_shutdown():
  cmd = Twist()
  if int( 0.25 * (rospy.get_rostime() - startTime).secs ) % 2 == 1:
    cmd.linear.x = 50.0
  cmdVel.publish(cmd) 
  rate.sleep()

