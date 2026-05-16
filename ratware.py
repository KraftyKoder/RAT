"""
Sensor and Motor Driver Code
"""

try:
    from ulab import numpy as np
except ImportError:
    import numpy as np

import mapping

worldX = 0  # X position of robot relative to starting pos [cm]
worldY = 0  # Y position of robot relative to starting pos [cm]
worldDir = 0  # Orientation [deg] relative to global horizontal

lidarAngle = 0
lidarDist = 0
fellowRatDetected = False


# When lidar reading received, call mapping.updateMap
# If fellowRatDetected, call explore (dirPref = opposite of worldDir + lidarAngle, otherwise dirPref = worldDir)

# When you move, update map pos: mapping.updateMapPos()

# Command motors to travel in straight line to target position
def goToTarget(targetX, targetY):
    deltaX = targetX - worldX
    deltaY = targetY - worldY
    turnAngle = np.rad2deg(np.atan2(deltaX, deltaY)) - worldDir

def getWorldCoords():
    return worldX, worldY

    