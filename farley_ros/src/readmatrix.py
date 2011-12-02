#!/usr/bin/python
#
# Read a matrix from a comma-separated file.

import numpy as np

mapFile = open('cost.map','r')
# Tokenize the file:
splitMap = [[i.strip() for i in line.split(',')] for line in mapFile]
# Trim an empty row (which happens when file ends with a newline):
if splitMap[-1][0] == '':
  splitMap = splitMap[:-1]
# Parse tokens as int and store in numpy array.
costMap = np.array([[int(j) for j in i] for i in splitMap])
print(costMap)
