#!/usr/bin/python
#
# A simple steering controller test script.

import roslib; roslib.load_manifest('farley_ros')
import rospy

import math
import matplotlib.pyplot as p

from steering_control import SteeringControl

rospy.init_node('steering_test')

rate = rospy.Rate(20)

c = SteeringControl()
c.setVelocity(0.1)
rospy.sleep(2)  # Give horizon some time to start

waypoints = [ [0.7,-0.5, 0], 
              [0.7, 0.5, math.pi/2],
              [-0.3,0.5, math.pi],
              [-0.3,-0.5,-math.pi/2] ]
cw = 0  #(curWaypoint)

c.setWaypoint(waypoints[0][0], waypoints[0][1], waypoints[0][2])
c.start()

# Wait for real pose data to be received
while (c.curPose is None) and not rospy.is_shutdown():
  rate.sleep()

while not rospy.is_shutdown():
  if (abs(c.curPose.X-waypoints[cw][0]) < 0.5) and (abs(c.curPose.Y-waypoints[cw][1]) < 0.5):
     print("Hit waypoint!")
     cw += 1
     if (cw >= len(waypoints)):
        break
     c.setWaypoint(waypoints[cw][0], waypoints[cw][1], waypoints[cw][2])
  rate.sleep()

c.stop()
print("Test Complete!")
p.plot(c.xRecord, c.yRecord)
p.show()
rospy.spin()
