#!/usr/bin/python
#
# Subjects the robot to a square wave, so that we can derive motor model constants
# empirically.

import roslib; roslib.load_manifest('farley_ros')
import rospy

from geometry_msgs.msg import Twist

from speedometer import Speedometer
import os

rospy.init_node('vel_ident')

cmdVel = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
startTime = rospy.get_rostime()
rate = rospy.Rate(20)
outfile = open('{0}/response.dat'.format(os.environ['HOME']), 'w')
curCmd = 0.0

def speedCb(spd, time):
  outfile.write("{0} {1} {2}\n".format( (time-startTime).to_sec(), curCmd, spd))
  print("v = {0}, u = {1}".format(spd, curCmd))

spd = Speedometer(speedCb)

while not rospy.is_shutdown():
  cmd = Twist()
  curCmd = 0.0
  if int( 0.5 * (rospy.get_rostime() - startTime).to_sec() ) % 2 == 1:
    curCmd = 100.0
  if int( 0.25 * (rospy.get_rostime() - startTime).to_sec() ) % 2 == 1:
    curCmd = -curCmd

  cmd.linear.x = curCmd
  cmdVel.publish(cmd) 
  rate.sleep()

