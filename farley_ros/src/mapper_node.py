#!/usr/bin/python
#
# A node which uses laser scan data and state estimates to construct 
# an occupancy map.

import roslib; roslib.load_manifest('farley_ros')
import rospy

from mapper import Mapper
import os

rospy.init_node('mapper')

m = Mapper()

rospy.spin()

g = m.grid

out = open('{0}/map.dat'.format(os.environ['HOME']), 'w')
for i in range(m.grid.shape[0]):
  for j  in range(m.grid.shape[1]):
    out.write('{0} '.format(m.grid[i,j]))
  out.write('\n')
out.close()

