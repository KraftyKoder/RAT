"""
MAPPING PROTOCOL
"""

try:
    from ulab import numpy as np
    np.set_printoptions(threshold=50) # printout for debugging

except ImportError:
    import numpy as np
    np.set_printoptions(edgeitems=25, linewidth=200, threshold=50) # printout for debugging
    import matplotlib.pyplot as plt

import testMaps


NUM_RATS = 2  # Number of rats in network
RAT_SIZE = 3  # Size of each rat (for traversal clearance) (treat as square with sides 2*RAT_SIZE)
goalFound = False
goalLoc = [0, 0] # Target location in map indices [x, y]

MAP_SIZE = [50, 50]
mazeMap = np.zeros(MAP_SIZE, dtype=np.uint8)
MAP_MAX = 7;  # max element magnitude

MAP_X_OFFSET = MAP_SIZE[1]/2
MAP_Y_OFFSET = MAP_SIZE[0]/2
MAP_SCALE = 0.5  # grid squares per cm

mapX = 0
mapY = 0
#  START_PT = (mapX, mapY)  # start point of map

OVERLAY_TOL = 10  # max distance to shift maps while overlaying
# ANGLE_TOL = 0 # max angle to shift maps while overlaying

EXPLORE_SENS = 5
PATH_LIMIT = 100
PATH_SMOOTHNESS = 5

# Store map overlay offsets after calculating for first time
# Initialize to values greater than OVERLAY_TOL, to indicate they haven't been set yet
storedOffsets = np.ones([3, NUM_RATS], dtype=np.uint8) * (OVERLAY_TOL + 1)

# Use world (physical) coordinates and map matrix index coordinates
def updateMapPos(worldX, worldY):
    global mapX, mapY
    mapX, mapY = convertWorldCoordstoMapInd(worldX, worldY)

def getMapPos():
    return mapX, mapY

def convertWorldCoordstoMapInd(worldX, worldY):
    return int(np.around(np.array([MAP_SCALE * worldX]))[0] + MAP_X_OFFSET), int(np.around(np.array([-MAP_SCALE * worldY]))[0] + MAP_Y_OFFSET)

def convertMapIndToWorldCoords(xInd, yInd):
    return (xInd - MAP_X_OFFSET) / MAP_SCALE, (yInd - MAP_Y_OFFSET) / -MAP_SCALE

# updateMap: add surroundings (LIDAR readings) to local map at estimated position
# lidarAngle is relative to rat orientation
def updateMap(lidarBuf, fellowRat, npts):
    global mazeMap

    LIDAR_ANGLE_DEADBAND = np.radians(30)
    LIDAR_DIST_CUTOFF = 30

    for i in range(npts):
        if (fellowRat[i] != None):
            if (not fellowRat[i]):
                lA = lidarBuf[i, 0]
                lD = lidarBuf[i, 1]
                wX = lidarBuf[i, 2]
                wY = lidarBuf[i, 3]
                wA = lidarBuf[i, 4]

                if (lD <= RAT_SIZE / MAP_SCALE): continue   # Small reading, ignore
                if (lD >= LIDAR_DIST_CUTOFF): continue      # ignore distances above certain value
                if (np.pi - abs(lA) < LIDAR_ANGLE_DEADBAND): continue    # ignore values in dead band range behind rat

                deltaX = int((lD * np.cos(lA + wA)) * MAP_SCALE)
                deltaY = int((lD * np.sin(lA + wA)) * -MAP_SCALE)

                mX, mY = convertWorldCoordstoMapInd(wX, wY)

                # Potential filtering/error correction here
                try:
                    if (mazeMap[mY + deltaY, mX + deltaX] < MAP_MAX):
                        mazeMap[mY + deltaY, mX + deltaX] += 1
                except IndexError:
                    print("Point outside map")
            else:
                print("Other rat in the way")

# Filter noise in map
def filterMap():
    print("hi")

# overlayMaps: for array of position and orientation adjustments of map 1, determine best orientation and position for alignment with map 2
# Want to maximize values in map (common features), then normalize
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
def explore(dirPref=None):
    # Valid positions rat can occupy, pixels without any walls within RAT_SIZE
    ratGridSize = int(np.ceil(RAT_SIZE * MAP_SCALE))
    validPos = np.zeros(MAP_SIZE, dtype=np.bool)
    for x in range(ratGridSize, MAP_SIZE[1]-ratGridSize):
        for y in range(ratGridSize, MAP_SIZE[0]-ratGridSize):
            if (np.sum(mazeMap[y-ratGridSize:y+ratGridSize+1, x-ratGridSize:x+ratGridSize+1]) == 0):
                validPos[y, x] = 1
    #print("Valid Spots")
    #print(validPos*1)

    # Queue of waypoints to explore
    junctions = np.zeros(MAP_SIZE, dtype=np.uint8)
    for x in range(1, MAP_SIZE[1]-1):
        for y in range(1, MAP_SIZE[0]-1):
            if (validPos[y, x]):
                junctions[y, x] = validPos[y, x+1]*1 + validPos[y+1, x]*1 + validPos[y, x-1]*1 + validPos[y-1, x]*1
    #print("Junctions")
    #print(junctions)

    if (goalFound):
        junctions[goalLoc[1], goalLoc[0]] = 5

    #return pathFinder([mapX, mapY], junctions, 1, dirPref=dirPref)
    rawPath = pathFinderIterative([mapX, mapY], junctions, dirPref=dirPref)
    return pathSmoother(rawPath)


# Recursive DFS function to find path from current position to open section of maze or to target
def pathFinder(curPos, junctions, callNum, dirPref=None):
    # Get current state
    curVal = junctions[curPos[1], curPos[0]]
    junctions[curPos[1], curPos[0]] = 0  # Mark current spot as invalid, so we don't double back to it
    # print(junctions)

    # Get neighboring states 
    # (Ensure indices are in bounds)
    openIndLowX = curPos[0]-EXPLORE_SENS
    openIndLowX *= (openIndLowX > 0)
    openIndHighX = curPos[0]+EXPLORE_SENS+1
    if (openIndHighX > MAP_SIZE[1]):
        openIndHighX = MAP_SIZE[1]
    openIndLowY = curPos[1]-EXPLORE_SENS
    openIndLowY *= (openIndLowY > 0)
    openIndHighY = curPos[1]+EXPLORE_SENS+1
    if (openIndHighY > MAP_SIZE[0]):
        openIndHighY = MAP_SIZE[0]

    localMap = junctions[curPos[1]-1:curPos[1]+2:, curPos[0]-1:curPos[0]+2]
    openMap = junctions[openIndLowY:openIndHighY, openIndLowX:openIndHighX]
    
    # Base case 1: reached dead end
    if (np.sum(localMap) == 0):
        return np.empty([0, 2])
    
    # Base case 2: open section of map or target is found, or max path length reached
    # (open criteria: at least half of elements within EXPLORE_SENS of current position are 4-way junctions)
    if (not goalFound and curVal == 4 and np.sum(openMap) > (np.size(openMap)/2 * 4) or 
            (goalFound and curVal == 5) or
            callNum == PATH_LIMIT):
        targetWorldX, targetWorldY = convertMapIndToWorldCoords(curPos[0], curPos[1])
        return np.array([[targetWorldX, targetWorldY]])

    # Next location to traverse to
    # If direction preference given, set next target to that location
    # If goalFound, set direction preference towards goal
    if goalFound:
        delX = goalLoc[0] - curPos[0]
        delY = curPos[1] - goalLoc[1]
        dirPref = np.arctan2(delY, delX)
    if dirPref != None:
        dirToLook = [np.cos(dirPref) > 0.5, np.sin(dirPref) > 0.5]  # [x dir, y dir]
        targetPos = (curPos[0] + dirToLook[0], curPos[1] - dirToLook[1])  # [xInd, yInd]    
    # If no direction preference given or preferred direction is invalid, go to max neighboring junction
    if (dirPref == None or junctions[targetPos[1], targetPos[0]] == 0):
        localMaxInd = np.argmax(localMap)
        targetPos = [curPos[0] + (localMaxInd%3-1), curPos[1] + (localMaxInd//3 - 1)]
    
    # Recursive step: move to target location, search for path from there
    restOfPath = pathFinder(targetPos, junctions, callNum + 1, dirPref)
    # If path leads to dead end, try another path from the current location
    # (Path leading to dead end has been filled with zeros during the 
    # traversal, so calling function again will find a new path from curPos)
    if (np.size(restOfPath) == 0):
        return pathFinder(curPos, junctions, callNum + 1, dirPref)
    else:
        targetWorldX, targetWorldY = convertMapIndToWorldCoords(curPos[0], curPos[1])
        return np.concatenate((np.array([[targetWorldX, targetWorldY]]), restOfPath), axis=0)  # Valid path to opening
    

def pathFinderIterative(startPos, junctions, dirPref=None):
    callNum = 1 # Track number of iterations
    path = np.array([startPos], dtype=np.uint8) # To store traversal points

    while (callNum < PATH_LIMIT and path.size != 0):
        # Get current state
        #print(path.size)
        curPos = path[-1, :]
        #print(curPos)
        curVal = junctions[curPos[1], curPos[0]]
        junctions[curPos[1], curPos[0]] = 0  # Mark current spot as invalid, so we don't double back to it
        #print(junctions)

        # Get neighboring states 
        # (Ensure indices are in bounds)
        openIndLowX = curPos[0]-EXPLORE_SENS
        openIndLowX *= (openIndLowX > 0)
        openIndHighX = curPos[0]+EXPLORE_SENS+1
        if (openIndHighX > MAP_SIZE[1]):
            openIndHighX = MAP_SIZE[1]
        openIndLowY = curPos[1]-EXPLORE_SENS
        openIndLowY *= (openIndLowY > 0)
        openIndHighY = curPos[1]+EXPLORE_SENS+1
        if (openIndHighY > MAP_SIZE[0]):
            openIndHighY = MAP_SIZE[0]

        localMap = junctions[curPos[1]-1:curPos[1]+2:, curPos[0]-1:curPos[0]+2]
        openMap = junctions[openIndLowY:openIndHighY, openIndLowX:openIndHighX]
        
        # Base case 1: reached dead end
        if (np.sum(localMap) == 0):
            path = path[0:-1, :]  # Remove current point in path, back track to previous point
            print("Dead end")
            continue
    
        # Base case 2: open section of map or target is found
        # (open criteria: at least half of elements within EXPLORE_SENS of current position are 4-way junctions)
        if (not goalFound and curVal == 4 and np.sum(openMap) > (np.size(openMap)/2 * 4) or 
                (goalFound and curVal == 5)):
            print("Target found")
            break

        # Next location to traverse to
        # If direction preference given, set next target to that location
        # If goalFound, set direction preference towards goal
        if goalFound:
            delX = goalLoc[0] - curPos[0]
            delY = curPos[1] - goalLoc[1]
            dirPref = np.arctan2(delY, delX)
        if dirPref != None:
            dirToLook = [np.cos(dirPref) > 0.5, np.sin(dirPref) > 0.5]  # [x dir, y dir]
            targetPos = [curPos[0] + dirToLook[0], curPos[1] - dirToLook[1]]  # [xInd, yInd]    
        # If no direction preference given or preferred direction is invalid, go to max neighboring junction
        if (dirPref == None or junctions[targetPos[1], targetPos[0]] == 0):
            localMaxInd = np.argmax(localMap)
            targetPos = [curPos[0] + (localMaxInd%3-1), curPos[1] + (localMaxInd//3 - 1)]
        
        # Recursive step: move to target location, search for path from there
        path = np.concatenate((path, np.array([targetPos],dtype=np.uint8)), axis=0)
        callNum += 1

    pathLength = path.shape[0]
    #print(pathLength)
    # If empty path (no possible routes)
    if pathLength == 0:
        return np.empty([0,2])
    else:  # Convert path from map indices to world coordinates
        pathWorldFrame = np.empty([pathLength, 2])
        for i in range(pathLength):
            xPos, yPos = convertMapIndToWorldCoords(path[i, 0], path[i, 1])
            pathWorldFrame[i] = [xPos, yPos]
        return pathWorldFrame


# Moving average smoothing of waypoints
def pathSmoother(wayPts):
    numPts = wayPts.shape[0]
    # If path short, do directly to target
    if (numPts < PATH_SMOOTHNESS):
        return wayPts
    smoothPath = np.empty([0,2])
    for i in range(0, numPts-PATH_SMOOTHNESS+1, PATH_SMOOTHNESS//2):
        xAvg = 0
        yAvg = 0
        for j in range(i, i+PATH_SMOOTHNESS):
            xAvg += wayPts[j, 0]
            yAvg += wayPts[j, 1]
        xAvg /= PATH_SMOOTHNESS
        yAvg /= PATH_SMOOTHNESS
        smoothPath = np.concatenate((smoothPath, np.array([[xAvg, yAvg]])), axis=0)
    
    # Include last point (target)
    smoothPath = np.concatenate((smoothPath, np.array([wayPts[numPts-1]])), axis=0)
    return smoothPath

def printPath(path):
    pathMap = np.zeros(MAP_SIZE, dtype=np.uint8)
    for i in range(path.shape[0]):
        mX, mY = convertWorldCoordstoMapInd(path[i,0], path[i,1])
        pathMap[mY, mX] = 9
    print(pathMap)

# Functions for dedugging and testing
# (Not part of final code)

def plotMazeMap():
    try:
        for i in range(MAP_SIZE[0]):
            for j in range(MAP_SIZE[1]):
                numWalls = mazeMap[i, j]
                for n in range(0, numWalls):
                    x, y = convertMapIndToWorldCoords(j, i)
                    plt.scatter(x, y, color="black", alpha=0.25, marker="s")
        if (goalFound):
            x, y = convertMapIndToWorldCoords(goalLoc[0], goalLoc[1])
            plt.scatter(x, y, color="red", marker="X")
    except:
        print("Can't plot here")

def plotPath(wayPts, smoothPts):
    try:
        plt.figure()
        plt.xlim([-50, 50])
        plt.ylim([0, 100])
        plt.scatter(wayPts[:,0], wayPts[:,1], color="orange", alpha=0.5)
        plt.plot(smoothPts[:,0], smoothPts[:,1])
        plotMazeMap()
        plt.show()
    except:
        print("Can't plot here")

# Test cases
def test(testNum):
    global mazeMap

    if (testNum == 1):  # Overlay test
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

    elif(testNum == 2):  # Explore: valid pos and junction test
        mazeMap[10:40, 15] = 1
        mazeMap[10:40, 23] = 1
        explore()
    elif (testNum == 3):  # Explore: path finder test
        global mapX, mapY
        mazeMap = testMaps.map1
        mapX = 23
        mapY = 45
        wayPts = explore(np.pi/2)
        print("Waypoints:" + str(wayPts))
        print("Number of waypoints: " + str(len(wayPts)))
        smoothPts = pathSmoother(wayPts)
        print("Smooth Path:" + str(smoothPts))
        print("Number of smooth points: " + str(len(smoothPts)))
        plotPath(wayPts, smoothPts)

        global goalFound, goalLoc
        goalFound = True
        goalLoc = [45, 5]
        mazeMap = testMaps.map1
        mapX = 23
        mapY = 45
        wayPts = explore()
        print("Waypoints:" + str(wayPts))
        print("Number of waypoints: " + str(len(wayPts)))
        smoothPts = pathSmoother(wayPts)
        print("Smooth Path:" + str(smoothPts))
        print("Number of smooth points: " + str(len(smoothPts)))
        plotPath(wayPts, smoothPts)

#test(3)


# Discard lidar readings if photoresistor picks up another rat (painted certain color)