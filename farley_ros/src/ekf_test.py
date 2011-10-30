#!/usr/bin/python
#
# Sandbox script for EKF testing.

import roslib; roslib.load_manifest('farley_ros')
import rospy

from controller_estimator import ControllerEstimator

rospy.init_node('ekf_test')

c = ControllerEstimator()
c.setVelocity(0.2)
c.setSteeringAngle(0.35)
c.start()

rospy.spin()

out = open('/home/iain/state.dat', 'w')
for s in c.stateRecord:
  for i in range(s.shape[0]):
    out.write('{0} '.format(s[i,0]))
  out.write('\n')
out.close()

#rospy.sleep(5)
#
#while not rospy.is_shutdown():
#  spd = 0.2;
#  if (int(0.5 * rospy.get_rostime().to_sec()) % 2 == 1):
#    spd *= -1;
#  c.setVelocity(spd)
#  rospy.sleep(2)

