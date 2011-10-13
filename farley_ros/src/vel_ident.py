#!/usr/bin/python
#
# Subjects the robot to a square wave, so that we can derive motor model constants
# empirically.

import roslib; roslib.load_manifest('clearpath_horizon')
import rospy

from geometry_msgs.msg import Twist

from speedometer import Speedometer

rospy.init_node('vel_ident')

cmdVel = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
startTime = rospy.get_rostime()
rate = rospy.Rate(20)

def speedCb(spd, time):
  print("t: {0}, v: {1}".format((time - startTime).to_sec(), spd))

spd = Speedometer(speedCb)

while not rospy.is_shutdown():
  cmd = Twist()
  if int( 0.25 * (rospy.get_rostime() - startTime).to_sec() ) % 2 == 1:
    cmd.linear.x = 50.0
  cmdVel.publish(cmd) 
  rate.sleep()

