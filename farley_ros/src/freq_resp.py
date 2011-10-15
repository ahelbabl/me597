#!/usr/bin/python
#
# Subjects the robot to a sequence of sinusoidal inputs, for the purposes of 
# measuring frequency response.
#
# Note that our nyquist frequency is 10Hz.

import roslib; roslib.load_manifest('farley_ros')
import rospy

from geometry_msgs.msg import Twist

from speedometer import Speedometer
import os
import math

rospy.init_node('freq_resp')

cmdVel = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
startTime = rospy.get_rostime()
curCmd = 0.0

# Frequencies to test
freqs = [1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 7.5, 10.0]
curFreq = 0
rate = rospy.Rate(20)

outfile = open('/dev/null', 'w')

# Setup callback for recording speed data as it is received.
def speedCb(spd, time):
  outfile.write("{0} {1} {2}\n".format( (time-startTime).to_sec(), curCmd, spd))
  print("v = {0}, u = {1}".format(spd, curCmd))
spd = Speedometer(speedCb)

# Wait 5s for system startup
rospy.sleep(5.0)
curStartTime = rospy.get_rostime()
outfile.close()
outfile = open(
    '{0}/sin_w{1}.dat'.format(os.environ['HOME'], freqs[0]), 'w')

while not rospy.is_shutdown():
  curTime = rospy.get_rostime()

  # Each test lasts 10s.  Then, system rests for 2s.  Then, next test.
  if (curTime-curStartTime).to_sec() < 10:
    curCmd = 100.0 * math.sin((curTime-curStartTime).to_sec() * freqs[curFreq])
  elif (curTime-curStartTime).to_sec() < 12:
    outfile.close()
    outfile = open('/dev/null','w')
    curCmd = 0.0
  else:
    curFreq += 1
    if curFreq >= len(freqs):
      rospy.signal_shutdown("Test complete")
      break
    curStartTime = rospy.get_rostime()
    outfile.close()
    outfile = open(
        '{0}/sin_w{1}.dat'.format(os.environ['HOME'], freqs[curFreq]), 'w')
    curCmd = 0.0

  cmd = Twist()
  cmd.linear.x = curCmd
  cmdVel.publish(cmd) 
  rate.sleep()

rospy.spin()

