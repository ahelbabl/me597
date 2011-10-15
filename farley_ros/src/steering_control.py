#!/usr/bin/bash
#
# Implements a nonlinear steering controller, extending VelocityController for 
# speed control functionality.

import roslib
roslib.load_manifest('farley_ros')
import rospy
from indoor_pos.msg import ips_msg
from geometry_msgs.msg import Twist

from velocity_control import VelocityControl
import math

class SteeringControl(VelocityControl):
  def __init__(self, maxDelta=0.1, ts=0.05):
    VelocityControl.__init__(self, ts=ts)
    rospy.Subscriber('indoor_pos', ips_msg, self._poseCb)
    self.maxDelta = maxDelta

    # Next waypoint
    self.xRef = 0.0
    self.yRef = 0.0

    # Steering reference and control signals: 
    self.steerCtrl = 0.0  # [-100,100]

  def setWaypoint(self, x, y):
    """ Set the target waypoint """
    self.xRef = x
    self.yRef = y

  def setSteeringAngle(self, ang):
    """ Set the steering angle to ang (radian) """
    out = 266.7 * ang - 7.0;
    self.steerCtrl = self._saturate(out, 100)
   
  def _publish(self):
    """ Publishes the current control signals """
    if not self.running:
        return
    velCmd = Twist()
    velCmd.linear.x = self.velCtrl
    velCmd.angular.z = self.steerCtrl
    self.cmdPub.publish(velCmd)

  def _poseCb(self, pose):
    """ Update steering command using current pose """
    print("Pose: x: {0}, y: {1}, h: {2}".format(pose.X, pose.Y, pose.Yaw))
    
    #system model variables
    Tmax=5     #length of time
    dt=0.001   #Time step
    delta_max=(25*math.pi)/180
    vel= VelocityControl()  #unit????
    vel.setVelocity(5)
    gain=2.5
    l=1     #update this!!!!
    #initialize vars
    e_cur=0 ;e_next=0
    psi_cur=pose.Yaw ;psi_next=0

    x=pose.X; y=pose.Y
    for i in range(len(Tmax/dt))
	delta=psi_cur+math.atan2(k*e_cur,vel))
	ederiv=vel*math.sin(psi_cur)-delta
	psideriv=-(vel*math.sin(delta))/l
	#update states
	e_next=e_cur+dt*ederiv
	psi_next=psi_cur+dt*psideriv
	#position update
	x=pose.X; y=pose.Y

    # Don't publish - we'll leave that to the velocity control
    # (Don't want to try to update too frequently)
    return
