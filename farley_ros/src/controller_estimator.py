#!/usr/bin/python
#
# A monolithic velocity + steering controller and EKF state estimator.
# All of these functions are combined due to the tight relationship
# between estimation and control, in terms both of timing and information
# sharing.

import math
import numpy as np
import os

import roslib; roslib.load_manifest('farley_ros')
import rospy
from geometry_msgs.msg import Twist
from indoor_pos.msg import ips_msg

from speedometer import Speedometer

class ControllerEstimator:
  def __init__(self):
    self.cmdPub = rospy.Publisher('/clearpath/robots/default/cmd_vel', Twist)
    self.spd = Speedometer(self._velocityCb)
    rospy.Subscriber('indoor_pos', ips_msg, self._poseCb)

    self.running = False
    self.ts = 0.05 # Sample time [s]
    self.length = 0.265 # Robot length [m]

    # Velocity control state:

    # Proportional control constants
    self.kp = 143.9
    self.ki = 857.8 
    self.integrator = 0.0
    self.lastErr = 0.0  # (needed for trapezoidal integrator)

    # Reference and control signals:
    self.velRef = None  # [m/s]
    self.velCtrl = None # [-100,100]

    # Steering control state:

    # Proportional control constant for steering angle.
    self.kSteer = 0.5

    # Next waypoint
    self.xRef = None
    self.yRef = None
    self.hRef = None

    # Steering reference and control signals: 
    self.steerCtrl = None # [-100,100]

    # Record pose data for inspectimification
    self.velRecord = np.array([0])
    self.xRecord = np.array([0])
    self.yRecord = np.array([0])

    # EKF State:
    # Latest measurements.  Set to None when consumed for EKF estimate. 
    self.lastPose = None
    self.lastVel = None

    # Process covariance:
    self.Q = np.array([[0.0005, 0, 0, 0],  
                       [0, 0.15, 0, 0],
                       [0, 0, 0.07, 0],
                       [0, 0, 0, 0.07]])
    # Measurement covariance:
    self.R = np.array([[0.0035, 0, 0, 0],
                       [0, 0.7, 0, 0],
                       [0, 0, 0.7, 0],
                       [0, 0, 0, 0.7]])

    # Zero initial state
    self.stateEst = np.zeros((4,1))
    self.velOutDelay = [0 0 0 0]
    # ... with low confidence
    self.P = np.ones((4,4))

    # Velocity model params:
    Km = 0.003267973309
    am = 6.779750386209
    self.velNum = (Km*am*self.ts)/(am*self.ts + 2)
    self.velDen = (am*self.ts-2)/(am*self.ts+2)

  def setVelocity(self, ref):
    """ Set the velocity reference """
    if ref is None:
      self.velRef = self.velCtrl = None
    else:
      self.velRef = ref

  def setSteeringAngle(self, ang):
    """ Set the steering angle to ang (radian) """
    if ang is None:
      self.steerCtrl = None
    else:
      out = 266.7 * ang - 7.0;
      self.steerCtrl = self._saturate(out, 100)
   
  def setWaypoint(self, x, y, head):
    """ Set the target waypoint """
    if (x is None) or (y is None) or (head is None):
      self.xRef = self.yRef = self.hRef = None
    else:
      self.xRef = x
      self.yRef = y
      self.hRef = head

  def start(self):
    self.running = True

  def stop(self):
    self.running = False

  def _publish(self):
    """ Publishes the current control signals """
    if not self.running:
      return
    velCmd = Twist()
    if not self.velCtrl is None:
      velCmd.linear.x = self.velCtrl
    if not self.steerCtrl is None:
      velCmd.angular.z = self.steerCtrl
    self.velOutDelay = self.velOutDelay[2:] + [self.velCtrl]
    self.cmdPub.publish(velCmd)

  def _saturate(self, val, limit):
    """ Returns val, saturated into [-limit,+limit] """
    if val > limit:
       return limit
    if val < -limit:
       return -limit
    return val

  def _wrapAngle(self, ang):
    """ Wrap an angle (radian) to [-pi, pi] """
    while ang > math.pi:
        ang -= math.pi
    while ang < -math.pi:
        ang += math.pi
    return ang

  def _halt(self):
    self.setVelocity(0)
    self.setSteeringAngle(0)

  def _ekfUpdate(self):
    if (self.lastVel is None) or (self.lastPose is None):
      # Not enough info to iterate kalman
      return

    # Measurements:
    m = np.array([[self.lastVel], 
            [self.lastPose.Yaw], 
            [self.lastPose.X], 
            [self.lastPose.Y]])

    # A priori state estimate:
    xp = np.zeros((4,1))
    xp[0] = self.velNum*(self.velOutDelay[-4] + self.velOutDelay[-3]) -
        self.velDen*self.stateEst[0]
    xp[1] = self.stateEst[1] + 
        self.stateEst[0]*math.sin(self.steerCtrl)*self.ts / self.length
    xp[2] = self.stateEst[2] +
        self.stateEst[0]*math.cos(self.stateEst[1])*self.ts
    xp[3] = self.stateEst[3] +
        self.stateEst[0]*math.sin(self.stateEst[1])*self.ts

    # Jacobian of current state w.r.t previous state:
    A = np.zeros((4,4))
    A[0,0] = -self.velDen
    A[1,0] = self.ts * math.sin(self.steerctrl) / self.length
    A[1,1] = 1
    A[2,0] = self.ts * math.cos(self.stateEst[1])
    A[2,1] = -self.ts * self.stateEst[0] * math.sin(self.steerCtrl)
    A[2,2] = 1
    A[3,0] = self.ts * math.sin(self.steerCtrl)
    A[3,1] = self.ts * self.stateEst[0] * math.cos(self.steerCtrl)
    A[3,3] = 1

    # A priori covariance:
    Pp = A * self.P * A.transpose() + self.Q

    # Kalman Gain:
    K = Pp * (Pp + self.R).inv()

    # State estimates:
    self.stateEst = xp + K*(m - xp)
    self.P = (np.identity(4) - K)*Pp

  def _velocityCb(self, vel, time):
    """ Accepts new measurement data, and adjusts the control signal """
    if not self.running:
      return
    if self.velRef is None:
      return

    self.velRecord = np.concatenate((self.velRecord, np.array([vel])))

    # Calculate the control signal:
    err = self.velRef - vel
    p = self.kp * err

    self.integrator += self.ki * 0.5 * self.ts * (self.lastErr + err)
    self.integrator = self._saturate(self.integrator, 100)

    self.velCtrl = self._saturate(p + self.integrator, 100)

    self.lastErr = err
    self._publish()
    return

  def _poseCb(self, pose):
    """ Update steering command using current pose """
    self.lastPose = pose

    if not self.running:
      return
    if (self.xRef is None) or (self.yRef is None) or (self.hRef is None):
      return

    self.xRecord = np.concatenate((self.xRecord, np.array([pose.X])))
    self.yRecord = np.concatenate((self.yRecord, np.array([pose.Y])))
    self.outfile.write('{0} {1} {2} {3}\n'.format(
          pose.header.stamp.to_sec(), pose.X, pose.Y, pose.Yaw))
   
    # Calculate heading error
    ang = (pose.Yaw * math.pi / 180)
    eHead = self.hRef - ang
    
    # Calculate crosstrack error.  Derived by wizards.
    ex = (self.xRef - pose.X) * (1 - math.cos(ang)*math.cos(ang))
    ey = (self.yRef - pose.Y) * (1 - math.sin(ang)*math.sin(ang))
    eCrosstrack = math.sqrt(ex*ex + ey*ey)
    # The sign of the z component of cross(target heading, crosstrack direction)
    # give the sign of the steering angle required to correct crosstrack.
    sign = math.cos(self.hRef)*ey - math.sin(self.hRef)*ex
    sign = sign / abs(sign)
    eCrosstrack = sign * eCrosstrack

    # Desired steering angle: 
    delta = eHead + math.atan2(self.kSteer * eCrosstrack, self.velRef)

    #print("Pose: x: {0}, y: {1}, h: {2}, eh: {3}".format(pose.X, pose.Y, pose.Yaw,eHead)) 
    self.setSteeringAngle(self._wrapAngle(delta))

    # Don't publish - we'll leave that to the velocity control
    # (Don't want to try to update too frequently)
    return

