"""
MAPPING PROTOCOL
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage

MAP_SIZE = np.array([500, 500])
mazeMap = np.zeros(MAP_SIZE)
MAP_MAX = 10;  # max element magnitude

worldX = 0;  # X position of robot relative to starting pos [cm]
worldY = 0;  # Y position of robot relative to starting pos [cm]
MAP_X_OFFSET = MAP_SIZE[0]/2
MAP_Y_OFFSET = MAP_SIZE[1] - 1
MAP_SCALE = 1  # grid squares per cm
mapX = MAP_SCALE * worldX + MAP_X_OFFSET  # X index of robot in map matrix 
mapY = MAP_SCALE * worldY + MAP_Y_OFFSET  # Y index of robot in map matrix
START_PT = (mapX, mapY)  # start point of map

OVERLAY_TOL = 10  # max distance to shift maps while overlaying
ANGLE_TOL = 5 # max angle to shift maps while overlaying

# updateMap: add surroundings (LIDAR readings) to local map at estimated position
def updateMap(lidarAngle, lidarDist):
    global mazeMap
    deltaX = lidarDist * np.cos(lidarAngle)
    deltaY = lidarDist * np.sin(lidarAngle)
    if (mazeMap[mapY + deltaY, mapX + deltaX] < MAP_MAX):
        mazeMap[mapY + deltaY, mapX + deltaX] += 1

# overlayMaps: for array of position and orientation adjustments of map 1, determine best orientation and position for alignment with map 2
# Want to maximize values in map (common features), then normalize
# WANT TO MAXIMIZE CONTRAST!!
def overlayMaps(otherMap):
    global mazeMap
    # Store overlay results
    overlayQuality = np.zeros([OVERLAY_TOL*2+1, OVERLAY_TOL*2+1, ANGLE_TOL*2+1])
    for x in range(-OVERLAY_TOL, OVERLAY_TOL+1):
        for y in range(-OVERLAY_TOL, OVERLAY_TOL+1):
            for theta in range(-ANGLE_TOL, ANGLE_TOL+1):
                # Shift incoming map by (x, y) offset
                otherMapShifted = shiftMap(otherMap, x, y,theta)
                # Add shifted maps
                tempMap = otherMapShifted + mazeMap
                # Record overlay quality
                overlayQuality[y+OVERLAY_TOL, x+OVERLAY_TOL, theta+ANGLE_TOL] = np.count_nonzero(tempMap > MAP_MAX)

    # Find optimal offsets for overlay
    indexFlat = np.argmax(overlayQuality)
    (yInd, xInd, thetaInd) = np.unravel_index(np.argmax(overlayQuality, axis=None), overlayQuality.shape)
    print([xInd, yInd, thetaInd])
    offsetX = -OVERLAY_TOL + xInd
    offsetY = -OVERLAY_TOL + yInd
    offsetTheta = -ANGLE_TOL + thetaInd
    otherMapShifted = shiftMap(otherMap, offsetX, offsetY, offsetTheta)
    # Combine maps: average overlapped cells and directly insert new cells
    for x in range(MAP_SIZE[1]):
        for y in range(MAP_SIZE[0]):
            if (mazeMap[y, x] > 0 and otherMapShifted[y, x] > 0):
                mazeMap[y, x] = (mazeMap[y, x] + otherMapShifted[y, x])/2
            else:
                mazeMap[y, x] = mazeMap[y, x] + otherMapShifted[y, x]

# Shift elements in map matrix by specified amount
# Fill empty elements with zeros
def shiftMap(map, xOffset, yOffset, theta):
    rotMap = ndimage.rotate(map, angle=theta, order=0)
    newW = np.size(rotMap, 1)
    bufferW = int((newW-MAP_SIZE[1])/2)
    newH = np.size(rotMap, 0)
    bufferH = int((newH-MAP_SIZE[0])/2)
    rotMap = rotMap[bufferH:bufferH+MAP_SIZE[0], bufferW:bufferW+MAP_SIZE[1]]
    shiftedMap = np.zeros(MAP_SIZE + [abs(yOffset), abs(xOffset)])
    if (xOffset > 0):
        placeX = xOffset
        choseX = 0
    else:
        placeX = 0
        choseX = abs(xOffset)
    if (yOffset > 0):
        placeY = yOffset
        choseY = 0
    else:
        placeY = 0
        choseY = abs(yOffset)
    shiftedMap[placeY:placeY+MAP_SIZE[0], placeX:placeX+MAP_SIZE[1]] = rotMap
    return shiftedMap[choseY:choseY+MAP_SIZE[0], choseX:choseX+MAP_SIZE[1]]

# Maze solving algorithm: DFS
# Send robots towards nearest unexplored part of maze
# queue of way points


# Test cases
def test():
    global mazeMap

    plt.figure(1)
    leftEye = np.concatenate((np.zeros([200, 149]), np.ones([200, 2])*10, np.zeros([200, 349])), axis=1)
    mouth = np.concatenate((np.zeros([2, 100]), np.ones([2, 300])*10, np.zeros([2, 100])), axis=1)
    sampleMap = np.concatenate((np.zeros([95, 500]), leftEye, np.zeros([50, 500]), mouth, np.zeros([153, 500])), axis=0)
    plt.imshow(sampleMap, interpolation='nearest')

    plt.figure(2)
    rightEye = np.concatenate((np.zeros([200, 349]), np.ones([200, 2])*10, np.zeros([200, 149])), axis=1)
    mouth = np.concatenate((np.zeros([2, 95]), np.ones([2, 300])*10, np.zeros([2, 105])), axis=1)
    mazeMap = np.concatenate((np.zeros([90, 500]), rightEye, np.zeros([50, 500]), mouth, np.zeros([158, 500])), axis=0)
    mazeMap = ndimage.rotate(mazeMap, angle=3, order=0)
    newW = np.size(mazeMap, 1)
    bufferW = int((newW-MAP_SIZE[1])/2)
    newH = np.size(mazeMap, 0)
    bufferH = int((newH-MAP_SIZE[0])/2)
    mazeMap = mazeMap[bufferH:bufferH+MAP_SIZE[0], bufferW:bufferW+MAP_SIZE[1]]
    plt.imshow(mazeMap, interpolation='nearest')

    plt.figure(3)
    overlayMaps(sampleMap)
    plt.imshow(mazeMap, interpolation='nearest')
    plt.show()

test()