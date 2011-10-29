#!/usr/bin/python
#
# A node which uses laser scan data and state estimates to construct 
# an occupancy map.

import roslib; roslib.load_manifest('farley_ros')
import rospy

from mapper import Mapper

rospy.init_node('mapper')

m = Mapper()

rospy.spin()
