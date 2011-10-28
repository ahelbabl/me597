#!/usr/bin/python
# Simple test node which gives the robot commands which result in it 
# driving in a circle.

import roslib; roslib.load_manifest('farley_ros')
import rospy

from geometry_msgs.msg import Twist

UPDATE_HZ = 20

rospy.init_node('circler')
cmd_pub = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)

rate = rospy.Rate(UPDATE_HZ)

start = rospy.get_rostime().secs

while not rospy.is_shutdown():
  rate.sleep()
  if (rospy.get_rostime().secs - start) < 2:
    continue

  cmd = Twist()
  cmd.linear.x = 40;
  cmd.angular.z = 50;
  cmd_pub.publish(cmd)

