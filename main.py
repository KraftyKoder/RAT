"""
  SOFTWARE FOR RAT BASTARD (Rolling Autonomous Tumbler)
  Project by Krrish Kainth, Ashutosh Kandala, Colby Ye, John Neilon, Ethan Daza
  Capstone 2026
"""

try:
  import time
except:
  print("ESP Module not available")

import ratware
import comms
import mapping

def init():
  comms.initComms()
  ratware.initRatware()

  # Read in surroundings for INIT_SCAN_TIME
  # for i in range(0, INIT_SCAN_TIME * 1/ratware.TIME_STEP):
  #   ratware.sweep()
  #   time.sleep(ratware.TIME_STEP)

  print("RAT INITIALIZATION SUCCESS")

def loop():
  while True:
    ratware.clock = time.ticks_ms()
    if (ratware.path.size == 0 and time.ticks_diff(ratware.clock, ratware.lastPathTime) > ratware.PATH_RATE):
      print("Making path")
      ratware.srvOff()
      ti = time.ticks_ms()
      ratware.path = mapping.explore(ratware.worldDir)
      ratware.lastPathTime = time.ticks_ms()
      print("path made in [ms]:")
      #print(mapping.printPath(ratware.path))
      ratware.driveServo()
      tf = time.ticks_ms()
      ratware.srvPauseTime += time.ticks_diff(tf, ti)
      print(ratware.srvPauseTime)
      
    if (ratware.path.shape[0] != 0):
      ratware.scurry(ratware.path[0,0], ratware.path[0,1])
      ratware.updateRatPose()
      ratware.checkTargetReached(ratware.path[0,0], ratware.path[0,1])
    
    ratware.isr1()
    ratware.isr2()

    ratware.sweep()
    if (ratware.swpDone):
      ratware.processLidarBuffer()

    if (time.ticks_diff(ratware.clock, comms.lastSqueakTime) > comms.SQUEAK_RATE):
      ratware.srvOff()
      ti = time.ticks_ms()
      comms.squeak()
      comms.lastSqueakTime = time.ticks_ms()
      print(mapping.mazeMap)
      ratware.driveServo()
      tf = time.ticks_ms()
      ratware.srvPauseTime += time.ticks_diff(tf, ti)
    
    time.sleep(ratware.TIME_STEP)

init()
loop()