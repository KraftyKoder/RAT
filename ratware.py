"""
SENSOR AND MOTOR DRIVER CODE
"""

try:
    from ulab import numpy as np
    from machine import Pin, PWM, ADC, SoftI2C
    import time
except ImportError:
    import numpy as np

import mapping
from vl53l0x import VL53L0X
from MPU6050 import MPU6050


LIDAR_SDA_PIN = 2
LIDAR_SCL_PIN = 1
IMU_SDA_PIN = 3
IMU_SCL_PIN = 8

SERVO_PIN = 41
ENC1A = 15
ENC1B = 16
ENC2A = 17
ENC2B = 18
#NSLEEP = 10
#LED = 13

RMOTORFWD = 6
RMOTORBK = 7
DRIVERSLEEP = 14
LMOTORFWD = 5
LMOTORBK = 4
MOTORPWRCAP = 0.9 * 65535


# L1_ADDR = 0x30
# L2_ADDR = 0x31

SRV_MAX = np.radians(182)
SRV_MIN = np.radians(-178)
SRV_MIN_NS = 1425 * 1000   # >= 500
SRV_MAX_NS = 1575 * 1000   # <= 2500
SRV_STOP_NS = 1500 * 1000    # tune
SRV_STEP = 5
SRV_MS = 40
SRV_BLIND_TIME = 500
SRV_TIMEOUT = 7500

XSHUT1 = 42
#XSHUT2 = 43

WHEEL_D = 8.68   # [cm]
ENC_CPR = 360
WHEELBASE = 10.4   # [cm]

#GYRO_W = 0.98
GYRO_W = 1
MAG_W = 0.02
ACCEL_DB = 0.05

# IMU Output Ranges
LSM6DS_ACCEL_RANGE_4_G = 0.122
LSM6DS_GYRO_RANGE_500_DPS = 17.50
LSM6DS_RATE_104_HZ = 104
SENSORS_DPS_TO_RADS = np.pi / 180.0
SENSORS_GRAVITY_STANDARD = 9.81  # m/sec

MAX_PTS = 128

# I2C Sensor Addresses
i2cLidar = None
i2cIMU = None
i2cMag = None 
srv = None
imu = None
mag = None
l1 = None
l2 = None

led = None

lidarBuf = np.empty([MAX_PTS, 5])  # [sweep time/angle, lidarDist, worldX, worldY, worldDir] at time of measurement
npts = 0
LIDAR_OFFSET = -4
LIDAR_FORWARD_SWEEP_OFFSET = np.radians(45)
LIDAR_BACKWARD_SWEEP_OFFSET = np.radians(-18)
LIDAR_CAL = 4

srvDir = 1
srvT = 0
swpDone = False
srvStartTime = 0
srvEndTime = 0
srvPauseTime = 0
firstSweep = True

mox = 0
moy = 0
msx = 1
msy = 1

e1 = 0
e2 = 0

worldX = 0.0  # X position of robot relative to starting pos [cm]
worldY = 0.0  # Y position of robot relative to starting pos [cm]
worldDir = 0  # Orientation [rad] relative to global horizontal
WORLD_DIR_OFFSET = 0  # offset between true north and maze horizontal
vx = 0
vy = 0
e1p = 0
e2p = 0

clock = 0

fellowRatDetected = np.zeros([MAX_PTS])
otherRatPos = np.empty([mapping.NUM_RATS, 2])

rmotorFwd = None
rmotorBk = None
driverSleep = None
lmotorFwd = None
lmotorBk = None
rMotorPwr = 0
lMotorPwr = 0
motorT = 0

# Position errors (for PID control)
prevTurnAngle = 0
turnAngle = 0
turnAngleSum = 0
K_P = 2500
K_D = 7500
K_I = 100

path = np.empty([0, 2])
PATH_RATE = 10 * 1000
lastPathTime = 0

TARGET_TOL = 2  # Target tolerance [cm]
RAT_DETECTION_TOL = 10  # Rat detection tolerance [cm]
OVERLAP_SENS = 0  # Max wall detections allowed within robot bounding box

MOTOR_SPEED = 0.4 * 65535  # rat nominal travel speed duty cycle
TIME_STEP = 0.01  # control time step [sec]

def encHandler(enc):
    if enc == enc1A: isr1()
    if enc == enc2A: isr2()

def isr1():
    global e1
    if (enc1A.value() == enc1B.value()):
        e1 += 1
    else:
        e1 += -1

def isr2(): 
    global e2
    if (enc2A.value() == enc2B.value()):
        e2 += 1
    else:
        e2 += -1

# Initialize sensors and positions
def initRatware():
    global worldX, worldY, worldDir, led, nsleep, i2cLidar, i2cIMU, i2cMag
    global imu, mag, l1, l2, srv, enc1A, enc1B, enc2A, enc2B, clock, motorT, srvT
    global rmotorFwd, rmotorBk, lmotorFwd, lmotorBk, prevTurnAngle, turnAngle, turnAngleSum, driverSleep

    worldX = 0
    worldY = 0
    worldDir = np.pi/2
    mapping.updateMapPos(worldX, worldY)
    clock = time.ticks_ms()
    motorT = clock
    srvT = clock

    #led = Pin(LED, Pin.OUT)
    #nsleep = Pin(NSLEEP, Pin.OUT)
    #nsleep.on()

    i2cLidar = SoftI2C(scl=Pin(LIDAR_SCL_PIN), sda=Pin(LIDAR_SDA_PIN), freq=400000)
    devices = i2cLidar.scan()
    print("LIDAR: ")
    print(devices)
    l1 = VL53L0X(i2cLidar)
    #l1.start_continuous()
    #l2 = devices[1]

    # xshut1 = Pin(XSHUT1)
    # xshut1.on()

    # i2cIMU = SoftI2C(scl=Pin(IMU_SCL_PIN), sda=Pin(IMU_SDA_PIN), freq=400000)
    # devices = i2cIMU.scan()
    # print("IMU: ")
    # print(devices)
    # imu = devices[0]

    imu = MPU6050()

    # mag = devices[1]

    srv = PWM(Pin(SERVO_PIN), freq=50, duty_ns=SRV_STOP_NS)
    srv.duty_ns(SRV_STOP_NS)
    time.sleep(0.5)

    enc1A = Pin(ENC1A, Pin.IN, Pin.PULL_UP)
    enc1B = Pin(ENC1B, Pin.IN, Pin.PULL_UP)
    enc2A = Pin(ENC2A, Pin.IN, Pin.PULL_UP)
    enc2B = Pin(ENC2B, Pin.IN, Pin.PULL_UP)

    enc1A.irq(trigger=Pin.IRQ_FALLING or Pin.IRW_RISING, handler=lambda a:encHandler(a))
    enc2A.irq(trigger=Pin.IRQ_FALLING or Pin.IRW_RISING, handler=lambda a:encHandler(a))


    rmotorFwd = PWM(Pin(RMOTORFWD), freq=50, duty_ns=0)
    rmotorBk = PWM(Pin(RMOTORBK), freq=50, duty_ns=0)
    driverSleep = Pin(DRIVERSLEEP, Pin.OUT)
    driverSleep.on()
    lmotorFwd = PWM(Pin(LMOTORFWD), freq=50, duty_ns=0)
    lmotorBk = PWM(Pin(LMOTORBK), freq=50, duty_ns=0)
    motorOff()
    prevTurnAngle = 0
    turnAngle = 0
    turnAngleSum = 0

    clock = time.ticks_ms()

    global npts, swpDone, srvStartTime, srvEndTime, srvPauseTime, srvDir, lidarBuf, fellowRatDetected, firstSweep
    npts = 0
    swpDone = False
    srvStartTime = time.ticks_ms()
    srvEndTime = time.ticks_ms()
    srvPauseTime = 0
    lidarBuf = np.zeros([MAX_PTS, 5])
    fellowRatDetected = np.zeros([MAX_PTS])
    firstSweep = True

    print("Ratware initialization successful")

# Command motors to travel in straight line to target position
def scurry(targetX, targetY):
    global prevTurnAngle, turnAngle, turnAngleSum, motorT
    # Position error
    prevTurnAngle = turnAngle
    deltaX = targetX - worldX
    deltaY = targetY - worldY
    turnAngle = np.arctan2(deltaY, deltaX) - worldDir
    turnAngleSum += turnAngle
    now = time.ticks_ms()
    dt = time.ticks_diff(now, motorT)
    motorT = now

    print("POSE: " + str(worldX) + "|" + str(worldY) + "|" + str(np.degrees(worldDir)))
    
    # Estimated position after TIME_STEP
    REAL_MOTOR_SPEED = 4 # [cm/s]
    estimateX = REAL_MOTOR_SPEED * TIME_STEP * np.cos(worldDir+turnAngle) + worldX
    estimateY = REAL_MOTOR_SPEED * TIME_STEP * np.sin(worldDir+turnAngle) + worldY

    # Check if robot can move in desired direction
    # if (validPostion(estimateX, estimateY)):
    #     # If so, drive motors towards target
    #     turnPwr = K_P * turnAngle + K_D * (turnAngle - prevTurnAngle)/dt + K_I * turnAngleSum
    #     rMotorPwr = MOTOR_SPEED + turnPwr
    #     lMotorPwr = MOTOR_SPEED - turnPwr
    #     #print(str(np.degrees(turnAngle)) + "|" + str(turnPwr) + "|" + str(rMotorPwr))
    # else: 
    #     # else, regenerate path
    #     # path = mapping.explore()
    #     # print("Invalid pos")
    #     # motorOff()
    #     return
    
    turnPwr = K_P * turnAngle + K_D * (turnAngle - prevTurnAngle)/dt + K_I * turnAngleSum
    rMotorPwr = MOTOR_SPEED + turnPwr
    lMotorPwr = MOTOR_SPEED - turnPwr
    
    global rmotorFwd, rmotorBk, lmotorFwd, lMotorBk

    if (rMotorPwr > MOTORPWRCAP): rMotorPwr = MOTORPWRCAP
    if (lMotorPwr > MOTORPWRCAP): lMotorPwr = MOTORPWRCAP
    if (rMotorPwr < -MOTORPWRCAP): rMotorPwr = -MOTORPWRCAP
    if (rMotorPwr < -MOTORPWRCAP): lMotorPwr = -MOTORPWRCAP

    if rMotorPwr > 0:
        rmotorFwd.duty_u16(int(rMotorPwr))
        rmotorBk.duty_u16(0)
    elif rMotorPwr < 0:
        rmotorFwd.duty_u16(0)
        rmotorBk.duty_u16(int(abs(rMotorPwr)))
    if lMotorPwr > 0:
        lmotorFwd.duty_u16(int(lMotorPwr))
        lmotorBk.duty_u16(0)
    elif lMotorPwr < 0:
        lmotorFwd.duty_u16(0)
        lmotorBk.duty_u16(int(abs(lMotorPwr)))
        


# Check if specified position is valid
# (Clearance for rat from edges of map, and no walls overlapping with rat)
def validPostion(x, y):
    mapX, mapY = mapping.convertWorldCoordstoMapInd(x, y)
    if (mapX < mapping.RAT_SIZE or mapX > mapping.MAP_SIZE[1] - mapping.RAT_SIZE or
        mapY < mapping.RAT_SIZE or mapY > mapping.MAP_SIZE[0] - mapping.RAT_SIZE or
        np.sum(mapping.mazeMap[mapY-mapping.RAT_SIZE:mapY+mapping.RAT_SIZE+1, mapX-mapping.RAT_SIZE:mapX+mapping.RAT_SIZE+1]) > OVERLAP_SENS):
        return False
    else:
        return True

# Check if robot has reached target point (with certain tolerance)
# Remove point from path if target reached
def checkTargetReached(targetX, targetY):
    global prevTurnAngle, turnAngle, turnAngleSum, path
    reached = abs(worldX - targetX) < TARGET_TOL and abs(worldY - targetY) < TARGET_TOL
    if reached:
        # Reset PID errors
        prevTurnAngle = 0
        turnAngle = 0
        turnAngleSum = 0

        print(path)
        # Remove waypoint from path
        if (path.shape[0] > 1):
            path = path[1:,:]
        else:
            path = np.empty([0, 2])
        print(path)
        
        print("Target reached")
    return reached

def getWorldCoords():
    return worldX, worldY

# Check if lidar reading coincides with another RAT, from lidarBuf data
def checkForFellowRat():
    global fellowRatDetected
    for i in range(npts):
        fellowRatDetected[i] = False
        lA = lidarBuf[i, 0]
        lD = lidarBuf[i, 1]
        wX = lidarBuf[i, 2]
        wY = lidarBuf[i, 3]
        wA = lidarBuf[i, 4]
        detectX = lD * np.cos(lA + wA) + wX
        detectY = lD * np.sin(lA + wA) + wY
        for i in range(mapping.NUM_RATS):
            ratX = otherRatPos[i, 0]
            ratY = otherRatPos[i, 1]
            if (abs(detectX - ratX) < RAT_DETECTION_TOL and abs(detectY - ratY) < RAT_DETECTION_TOL):
                fellowRatDetected[i] = True
                path = mapping.explore(np.radians(180)+wA+lA)  # Generate path avoiding other rat

# Read value from Lidar sensor and add point to map
def readLidar(lidar):
    lidarDist1 = lidar.read_range_single_millimeters() / 10 + LIDAR_OFFSET
    #print(lidarDist1)
    return lidarDist1
    # lidarDist2 = lidar.read_range_single_millimeters() / 10 + LIDAR_OFFSET
    #return (lidarDist1 + lidarDist2) / 2

def magHdg():
    registerValues = np.zeros([7], np.uint8)
    xyzMeasurement = i2cMag.readfrom_into(mag, registerValues)

    rx = registerValues[0]
    rx = (rx << 8) | registerValues[1]
    rx = (rx << 2) | (registerValues[6] >> 6)
    ry = registerValues[2]
    ry = (ry << 8) | registerValues[3]
    ry = (ry << 2) | ((registerValues[6] >> 4) & 0x03)
    rz = registerValues[4]
    rz = (rz << 8) | registerValues[5]
    rz = (rz << 2) | ((registerValues[6] >> 2) & 0x03)

    mx = (rx - 131072) / 131072
    my = (ry - 131072) / 131072
    mx = (mx - mox) * msx
    my = (my - moy) * msy
    h = np.arctan2(my, mx) * 180.0 / np.pi
    if (h < 0): h += 360.0
    return h

def readIMU():
    buffer = np.zeros([14], dtype=np.uint8)
    i2cIMU.readfrom_into(imu, buffer)

    rawGyroX = buffer[3] << 8 or buffer[2]
    rawGyroY = buffer[5] << 8 or buffer[4]
    rawGyroZ = buffer[7] << 8 or buffer[6]

    rawAccX = buffer[9] << 8 or buffer[8]
    rawAccY = buffer[11] << 8 or buffer[10]
    rawAccZ = buffer[13] << 8 or buffer[12]

    gyro_scale = LSM6DS_GYRO_RANGE_500_DPS  # milli-dps / bit

    gyroX = rawGyroX * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0
    gyroY = rawGyroY * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0
    gyroZ = rawGyroZ * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0

    accel_scale = LSM6DS_ACCEL_RANGE_4_G # milli-g / bit
 
    accX = rawAccX * accel_scale * SENSORS_GRAVITY_STANDARD / 1000 * 100  # [cm/sec^2]
    accY = rawAccY * accel_scale * SENSORS_GRAVITY_STANDARD / 1000 * 100
    accZ = rawAccZ * accel_scale * SENSORS_GRAVITY_STANDARD / 1000 * 100

    return accX, accY, accZ, gyroX, gyroY, gyroZ

# Update world coordinates based on encoder readings
# Update map coordinates after
def updateRatPose():
    global clock, e1p, e2p, worldX, worldY, worldDir, vx, vy
    now = time.ticks_ms()
    dt = time.ticks_diff(now, clock) / 1000000
    if (dt <= 0 or dt > 0.5):
        return

    c1 = e1
    c2 = e2
    d1 = (c1 - e1p) * (np.pi * WHEEL_D / ENC_CPR)
    d2 = - (c2 - e2p) * (np.pi * WHEEL_D / ENC_CPR)
    e1p = c1
    e2p = c2

    dc = (d1 + d2) / 2.0

    #accX, accY, accZ, gyroX, gyroY, gyroZ = readIMU()
    acc = imu.read_accel_data()
    accX = acc["x"]
    accY = acc["y"]
    accZ = acc["z"]

    gyro = imu.read_gyro_data()
    gyroX = gyro["x"]
    gyroY = gyro["y"]
    gyroZ = gyro["z"]

    gh = worldDir + gyroZ * dt * np.pi / 180
    #mh = magHdg() + WORLD_DIR_OFFSET
    #while (mh - gh > 180.0):  mh -= 2*np.pi
    #while (gh - mh > 180.0):  mh += 2*np.pi

    worldDir = GYRO_W * gh #+ MAG_W * mh
    if (worldDir < 0):    worldDir += 2*np.pi
    if (worldDir >= 2*np.pi): worldDir -= 2*np.pi

    worldX += dc * np.cos(worldDir)   # CHECK IF TRIG FUNCTIONS RIGHT??
    worldY += dc * np.sin(worldDir)

    if (abs(accX) > ACCEL_DB):
        vx += accX * dt
    if (abs(accY) > ACCEL_DB):
        vy += accY * dt
    
    mapping.updateMapPos(worldX, worldY)

def sweep():
    global srvT, lidarBuf, srvEndTime, swpDone, npts, srvDir
    now = time.ticks_ms()

    # If servo spinning too long, halt as precautions
    if (time.ticks_diff(now, srvStartTime) - srvPauseTime > SRV_TIMEOUT):
        srvOff()
        raise ValueError("SERVO TIMEOUT ERROR")

    if (time.ticks_diff(now, srvT) < SRV_MS or (time.ticks_diff(now, srvEndTime) - srvPauseTime < SRV_BLIND_TIME and not firstSweep)):
        return
    srvT = now

    d1 = readLidar(l1)
    #d2 = readLidar(l2)

    if (npts < MAX_PTS and not firstSweep):
        if (d1 > LIDAR_CAL):  # valid reading, not at calibration boundary
            timeStamp = time.ticks_diff(now, srvStartTime)
            if (srvPauseTime != 0):
                timeStamp -= srvPauseTime
            lidarBuf[npts] = [timeStamp, d1, worldX, worldY, worldDir]
            npts += 1
        # if (d2 > LIDAR_CAL):
        #     lidarBuf[npts] = [now - srvStartTime, -d2, worldX, worldY, worldDir]
        #     npts += 1
    
    if not (d1 < LIDAR_CAL):  # Zero calibration point reached
        driveServo()
    else:
        srvOff()
        swpDone = True
        srvDir = -srvDir
        driveServo()
        srvEndTime = time.ticks_ms()

def driveServo():
    global srv, srvDir
    if (srvDir == 1):
        srv.duty_ns(SRV_MAX_NS)
    else:
        srv.duty_ns(SRV_MIN_NS)

def srvOff():
    global srv
    srv.duty_ns(SRV_STOP_NS)

def motorOff():
    rmotorFwd.duty_u16(0)
    rmotorBk.duty_u16(0)
    lmotorFwd.duty_u16(0)
    lmotorBk.duty_u16(0)

def processLidarBuffer():
    global npts, swpDone, srvStartTime, srvEndTime, srvPauseTime, lidarBuf, srvDir, fellowRatDetected, firstSweep

    # Convert first column of buffer from time stamps to lidar angles
    prevSrvDir = -srvDir  # Note: need to flip srvDir for preceding sweep
    swpDuration = time.ticks_diff(srvEndTime, srvStartTime) - srvPauseTime
    swpSpeed = prevSrvDir * (SRV_MAX - SRV_MIN) / swpDuration
    lidarBuf[:, 0] = lidarBuf[:, 0] * swpSpeed
    if (prevSrvDir == 1):  # CCW
        lidarBuf[:, 0] += SRV_MIN + LIDAR_FORWARD_SWEEP_OFFSET
    else:  # CW
        lidarBuf[:, 0] += SRV_MAX + LIDAR_BACKWARD_SWEEP_OFFSET

    # Print buffer
    print(lidarBuf)

    # Discard first sweep
    if (firstSweep):
        firstSweep = False
        resetLidarForSweep()
        return

    # Update map with buffer
    #checkForFellowRat()
    mapping.updateMap(lidarBuf, fellowRatDetected, npts)
    resetLidarForSweep()

def resetLidarForSweep():
    # Reset lidar buffer for next sweep
    global npts, swpDone, srvStartTime, srvPauseTime, lidarBuf, fellowRatDetected
    npts = 0
    swpDone = False
    srvStartTime = time.ticks_ms()
    srvPauseTime = 0
    lidarBuf = np.zeros([MAX_PTS, 5])
    fellowRatDetected = np.zeros([MAX_PTS])

# Scurry PID Tuning
# initRatware()

# path = np.array([[25, 100]])
# startT = time.ticks_ms()
# time.sleep(2)

# while True:
#     if path.size == 0:
#         motorOff()
#         break
#     scurry(path[0, 0], path[0, 1])
#     updateRatPose()
#     reached = checkTargetReached(path[0, 0], path[0, 1])
#     if (time.ticks_diff(time.ticks_ms(), startT) > 15000):
#         motorOff()
#         break
#     time.sleep(TIME_STEP)
