# A controller which ControllerEstimator, and which overrides the
# steering controller in order to steer in the direction of the negative
# gradient of a cost map.

import math as m
import numpy as np
from collections import namedtuple
from controller_estimator import *

Range = namedtuple('Range', 'min max incr')

class GradientDescender(ControllerEstimator):
  def __init__(self, costFile, xRange, yRange):
    """ costFile - path to a comma separated file storing a precomputed costmap
        xRange - defines the x positions corresponding to costmap row indices
        yRange - defines the y positions corresponding to costmap col indices """
    ControllerEstimator.__init__(self)
    self.xRange = xRange
    self.yRange = yRange
 
    # Read the costmap 
    mapFile = open(costFile, 'r')
    # Tokenize the file:
    splitMap = [[i.strip() for i in line.split(',')] for line in mapFile]
    # Trim an empty row (which happens when file ends with a newline):
    if splitMap[-1][0] == '':
      splitMap = splitMap[:-1]
    # Parse tokens as int and store in numpy array.
    self.costMap = np.array([[int(j) for j in i] for i in splitMap])

    self.costGrad = np.gradient(self.costMap)

  def _steerCtrl(self, X, Y, ang):
    """ A simple proportional steering controller, which takes the direction
        of the negative cost gradient at the current position as its reference
        position. """
    # Find indices of current cost map cell
    i = int(round((X - self.xRange.min) / self.xRange.incr))
    j = int(round((Y - self.yRange.min) / self.yRange.incr))

    # Range check on cost map
    if (i<0) or (i >= self.costMap.shape[0]) or \
       (j<0) or (j >= self.costMap.shape[1]):
      self._halt()
      return

    ref = m.atan2(-self.costGrad[1][i,j], -self.costGrad[0][i,j])
    self.setSteeringAngle(self._wrapAngle(ref - ang))
    
