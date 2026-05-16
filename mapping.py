"""
MAPPING PROTOCOL
"""

try:
    from ulab import numpy as np
except ImportError:
    import numpy as np

np.set_printoptions(threshold=50) # printout for debugging

NUM_RATS = 3  # Number of rats in network
RAT_SIZE = 5  # Size of each rat (for traversal clearance) (treat as square with sides 2*RAT_SIZE)

MAP_SIZE = [50, 50]
mazeMap = np.zeros(MAP_SIZE, dtype=np.uint8)
MAP_MAX = 10;  # max element magnitude

MAP_X_OFFSET = MAP_SIZE[1]/2
MAP_Y_OFFSET = MAP_SIZE[0] - 1
MAP_SCALE = 0.5  # grid squares per cm

mapX = 0
mapY = 0
#  START_PT = (mapX, mapY)  # start point of map

OVERLAY_TOL = 10  # max distance to shift maps while overlaying
# ANGLE_TOL = 0 # max angle to shift maps while overlaying

# Store map overlay offsets after calculating for first time
# Initialize to values greater than OVERLAY_TOL, to indicate they haven't been set yet
storedOffsets = np.ones([3, NUM_RATS], dtype=np.uint8) * (OVERLAY_TOL + 1)

# Use world (physical) coordinates and map matrix index coordinates
def updateMapPos(worldX, worldY):
    global mapX, mapY
    mapX = int(np.round(MAP_SCALE * worldX) + MAP_X_OFFSET)
    mapY = int(np.round(MAP_SCALE * worldY) + MAP_Y_OFFSET)

def convertMapIndToWorldCoords(xInd, yInd):
    return (xInd - MAP_X_OFFSET) / MAP_SCALE, (yInd - MAP_Y_OFFSET) / MAP_SCALE

# updateMap: add surroundings (LIDAR readings) to local map at estimated position
# lidarAngle is relative to rat orientation
def updateMap(lidarAngle, lidarDist, worldDir, fellowRat):
    global mazeMap
    if (not fellowRat):
        deltaX = int((lidarDist * np.cos(lidarAngle + worldDir)) * MAP_SCALE)
        deltaY = int((lidarDist * np.sin(lidarAngle + worldDir)) * MAP_SCALE)
        if (mazeMap[mapY + deltaY, mapX + deltaX] < MAP_MAX):
            mazeMap[mapY + deltaY, mapX + deltaX] += 1
    else:
        print("Other rat in the way")

# overlayMaps: for array of position and orientation adjustments of map 1, determine best orientation and position for alignment with map 2
# Want to maximize values in map (common features), then normalize
# WANT TO MAXIMIZE CONTRAST!!
def overlayMaps(otherMap, ratID):
    global mazeMap
    offsetX = 0
    offsetY = 0
    otherMapShifted = np.empty(otherMap.shape, dtype=np.uint8)
    # Store overlay results (if not previously calculated)
    if (storedOffsets[0, ratID] > OVERLAY_TOL):
        overlayQuality = np.zeros([OVERLAY_TOL*2+1, OVERLAY_TOL*2+1], dtype=np.uint8)
        overlaySensitivity = np.max(mazeMap) * 1.25 # Cutoff for counting sum of map values during overlay quality check
        for x in range(-OVERLAY_TOL, OVERLAY_TOL+1):
            for y in range(-OVERLAY_TOL, OVERLAY_TOL+1):
                # Shift incoming map by (x, y) offset
                otherMapShifted = shiftMap(otherMap, x, y, 0)
                # Add shifted maps
                tempMap = otherMapShifted + mazeMap
                # Record overlay quality
                overlayQuality[y+OVERLAY_TOL, x+OVERLAY_TOL] = np.sum(tempMap > overlaySensitivity)

        # Find optimal offsets for overlay
        indexFlat = np.argmax(overlayQuality)
        yInd = indexFlat // overlayQuality.shape[1]
        xInd = indexFlat % overlayQuality.shape[1]
        offsetX = -OVERLAY_TOL + xInd
        offsetY = -OVERLAY_TOL + yInd
        # offsetTheta = -ANGLE_TOL + thetaInd
        storedOffsets[0, ratID] = offsetX
        storedOffsets[1, ratID] = offsetY
        storedOffsets[2, ratID] = 0
    else:
        offsetX = storedOffsets[0, ratID]
        offsetY = storedOffsets[1, ratID]
        # offsetTheta = storedOffsets[2, ratID]
    otherMapShifted = shiftMap(otherMap, offsetX, offsetY, 0)
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
    # Rotate and crop map
    # rotMap = scipy.ndimage.rotate(map, angle=theta, order=0)
    # newW = np.size(rotMap, 1)
    # bufferW = int((newW-MAP_SIZE[1])/2)
    # newH = np.size(rotMap, 0)
    # bufferH = int((newH-MAP_SIZE[0])/2)
    # rotMap = rotMap[bufferH:bufferH+MAP_SIZE[0], bufferW:bufferW+MAP_SIZE[1]]

    rotMap = map

    # Shift map by specified offsets
    # Crop to MAP_SIZE and fill empty cells with zeros
    shiftedMap = np.zeros([MAP_SIZE[0] + abs(yOffset), MAP_SIZE[1] + abs(xOffset)], dtype=np.uint8)
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

# OPTIONAL: Map rotation (custom rotation function)

# Maze solving algorithm: DFS
# Send robots towards nearest unexplored part of maze
# Prefer waypoints in direction dirPref (relative to world horizontal)
# Builds queue of way points
def explore(dirPref):
    # Valid positions rat can occupy, pixels without any walls within RAT_SIZE
    ratGridSize = int(np.ceil(RAT_SIZE * MAP_SCALE))
    validPos = np.zeros(MAP_SIZE, dtype=np.bool)
    for x in range(ratGridSize, MAP_SIZE[1]-ratGridSize):
        for y in range(ratGridSize, MAP_SIZE[0]-ratGridSize):
            if (np.sum(mazeMap[y-ratGridSize:y+ratGridSize+1, x-ratGridSize:x+ratGridSize+1]) == 0):
                validPos[y, x] = 1
    print(validPos*1)

    # Queue of waypoints to explore
    junctions = np.zeros(MAP_SIZE, dtype=np.uint8)
    for x in range(1, MAP_SIZE[1]-1):
        for y in range(1, MAP_SIZE[0]-1):
            if (validPos[y, x] == 1):
                junctions[y, x] = validPos[y, x+1] + validPos[y+1, x] + validPos[y, x-1] + validPos[y-1, x]
    print(junctions)

def pathFinder(curPos, junctions, dirPref=None):
    junctions[curPos[1], curPos[0]] = 0  # Mark current spot as invalid, so we don't double back to it
    localMap = mazeMap[curPos[1]-1:curPos[1]+2:, curPos[0]-1:curPos[0]+2]

    # Base case 1: reached dead end
    if (np.sum(localMap) == 0):
        return []
    
    if dirPref == None:
        # Find location of max junction nearby
        localMaxInd = np.argmax(localMap)
        targetPos = [curPos[0] + (localMaxInd%3-1), curPos[1] + (localMaxInd//3 - 1)]
    else:
        dirToLook = [np.cos(dirPref) > 0.5, np.sin(dirPref) > 0.5]  # [x dir, y dir]
        targetPos = [curPos[0] + dirToLook[0], curPos[1] - dirToLook[1]]  # [xInd, yInd]

    # Base case 2: there is a neighboring 4-way junction to go to
    if (junctions[targetPos[1], targetPos[0]] == 4):
        targetWorldX, targetWorldY = convertMapIndToWorldCoords(targetPos[0], targetPos[1])
        return [(targetWorldX, targetWorldY)]
    # Recusive step 1: if target pos is invalid, search for path without direction preference
    elif (junctions[targetPos[1], targetPos[0]] == 0):
        return pathFinder(curPos, junctions)
    # Recursive step 2: move to target location, search for path from there
    else:
        restOfPath = pathFinder(targetPos, junctions)
        # If path leads to dead end, try another path from the current location
        # (Path leading to dead end has been filled with zeros during the 
        # traversal, so calling function again will find a new path from curPos)
        if (len(restOfPath) == 0): 
            return pathFinder(curPos, junctions)
        # Valid path to opening
        else:
            return [(targetWorldX, targetWorldY)] + restOfPath

# Test cases
def test(testNum):
    global mazeMap

    if (testNum == 1):  # Overlay test
        # sampleMap = np.concatenate((np.zeros([19, 100]), 
        #                             np.concatenate((np.zeros([40, 39]), np.ones([40, 2])*10, np.zeros([40, 59])), axis=1),
        #                             np.zeros([10, 100]), 
        #                             np.concatenate((np.zeros([2, 20]), np.ones([2, 60])*10, np.zeros([2, 20])), axis=1), 
        #                             np.zeros([29, 100])), axis=0)
        # try:
        #     plt.figure(1)
        #     plt.imshow(sampleMap, interpolation='nearest')
        # except:
        #     print("")

        # mazeMap = np.concatenate((np.zeros([19, 100]), 
        #                           np.concatenate((np.zeros([40, 59]), np.ones([40, 2])*10, np.zeros([40, 39])), axis=1),
        #                           np.zeros([10, 100]), 
        #                           np.concatenate((np.zeros([2, 20]), np.ones([2, 60])*10, np.zeros([2, 20])), axis=1), 
        #                           np.zeros([29, 100])), axis=0)

        #     overlayMaps(sampleMap, 0)
        
        # try:
        #     plt.figure(2)
        #     plt.imshow(mazeMap, interpolation='nearest')

        #     plt.figure(3)
        #     plt.imshow(mazeMap, interpolation='nearest')
        #     plt.show()
        # except:
        #     print("No plots")

        sampleMap = np.zeros(MAP_SIZE, dtype=np.uint8)
        sampleMap[21:27, 26] = 1
        mazeMap[20:26, 25] = 1

        # print(mazeMap)
        # print(sampleMap)
        overlayMaps(sampleMap, 0)
        # print(mazeMap)

        if (mazeMap[25, 25] == 1 and mazeMap[26, 26] == 0):
            print("Overlay Test Succeeded")
        else:
            print("Overlay Test Failed")

    elif(testNum == 2):  # Explore test
        mazeMap[10:40, 15] = 1
        mazeMap[10:40, 23] = 1
        explore(0)

test(2)


# Discard lidar readings if photoresistor picks up another rat (painted certain color)