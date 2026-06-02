"""
INTER-ROBOT COMMUNICATION PROTOCOL
"""

try:
    from ulab import numpy as np
    import espnow
    import network
    from machine import UART
    import time
    import deflate, io
except ImportError:
    import numpy as np

from mapping import NUM_RATS, MAP_SIZE, mazeMap, overlayMaps
from ratware import worldX, worldY, otherRatPos

# MAC Addresses of all RAT ESP boards
broadcastAddys = [b'\x90\x70\x69\x07\x11\x64', 
                  b'\xe4\x65\xb8\x6f\x2d\x50']
NUM_RATS = 2
SQUEAK_RATE = 10 * 1000  # Time between messages [ms]
MY_ID = 0  # Personal Rat ID (corresponds to index in broadcastAddys array)
MAX_MESSAGE_SIZE = 200  # Max ints that can be broadcast per message
enow = None  # ESP-Now object

# Store data from senders

otherMap = np.empty(MAP_SIZE).flatten()
otherRatID = None
otherRatMessages = 0
lastSqueakTime = 0

## MESSAGE STRUCTURE
# Index 0: indicates receive error (ex. receiving data from another rat), triggers sender to retry message after delay
# Index 1: sender's global position
# Index 2+: sender's maze map 

def initComms():
  global enow

  # Start Serial Monitor
  uart = UART(1, 115200)
 
  # Setup WiFi and ESP-Now
  sta = network.WLAN(network.WLAN.IF_STA)
  sta.active(True)
  #sta.config(channel=6, protocol=network.WLAN.PROTOCOL_LR)  # Long range mode
  enow = espnow.ESPNow()
  #enow.config(rate=espnow.RATE_LORA_500K)
  enow.active(True)

  for i in range(0, NUM_RATS):
    if (i != MY_ID):
      # Register peer
      enow.add_peer(broadcastAddys[i])

  # Set function to be called whenever new message is received
  enow.irq(onDataRecv)

  print("Comms Initialization Successful")

def encodeMessage(index, data):
  # stream = io.BytesIO()
  # with deflate.DeflateIO(stream, deflate.ZLIB) as d:
  #   d.write(pos.tobytes() + map.tobytes())
  # mapCompressed = stream.getvalue()
  # print(mapCompressed)

  return np.array([index], dtype=np.uint8).tobytes() + data.tobytes()

def decodeMessage(message):
  # with deflate.DeflateIO(io.BytesIO(message), deflate.ZLIB) as d:
  #   messageUncompressed = d.read()
  index = np.array(message[0:1], dtype=np.uint8)
  data = np.array(message[1:], dtype=np.uint8)
  return index, data

# Sends maze map and global position to all other rats
# If otherID specified, only send data to that rat
def squeak(otherID=None):
  for i in range(0, NUM_RATS):
    try:
      if ((otherID == None and i != MY_ID) or (otherID != None and i == otherID)):
        # Send current position
        myMessage = encodeMessage(1, np.array([int(worldX), int(worldY)], dtype=np.uint8))
        enow.send(broadcastAddys[i], myMessage)
        #print("Index: " + str(decodeMessage(myMessage)[0]) + " | Message: " + str(decodeMessage(myMessage)[1]))
        # Send map in chunks
        for n in range(0, mazeMap.size/MAX_MESSAGE_SIZE):
          endInd = (n+1)*MAX_MESSAGE_SIZE
          if (endInd > mazeMap.size):  # Avoid index overflow, if remaining data less than MAX_MESSAGE_SIZE
            endInd = mazeMap.size
          myMessage = encodeMessage(n+2, mazeMap.flatten()[n*MAX_MESSAGE_SIZE:endInd])
          enow.send(broadcastAddys[i], myMessage)
          sentMap = decodeMessage(myMessage)[1]
          #print("Index: " + str(decodeMessage(myMessage)[0]) + " | Message: " + str(sentMap) + " (size: " + str(sentMap.size) + ")")
        print("All messages sent to Rat " + str(i))
    except OSError:
      print("Failed to send all messages to Rat " + str(i))


def onDataRecv(enow):
  print("Message received!")
  while True:  # Read out all messages waiting in the buffer
    mac, message = enow.irecv(0)
    if mac is None:  # Don't wait if no messages left
        break
    # Read message
    otherID = findID(mac)
    index, data = decodeMessage(message)

    if (index == 0):  # Sender rat was busy when reading squeak, resend after delay
      time.sleep(SQUEAK_RATE/10)
      squeak(otherID)
      continue

    if (otherRatID == None): # Set current sender
      otherRatID = otherID
    
    # Only collect data from one rat at a time, trigger other rats to resend data
    if (otherRatID == otherID):
      otherRatMessages += 1
      if (index == 1):
        otherRatPos[otherID, :] = data
      else:
        otherMap[(index-2)*MAX_MESSAGE_SIZE:(index-1)*MAX_MESSAGE_SIZE] = data
    else:
      enow.send(mac, encodeMessage(0, 0))  # Receiver busy error (index 0)

  # All data packets received from otherRatID
  if (otherRatMessages == 1 + np.ceil(MAP_SIZE[0]*MAP_SIZE[1]/MAX_MESSAGE_SIZE)):
    overlayMaps(np.reshape(otherMap, MAP_SIZE))
    print("Successfully read data from Rat " + str(otherRatID))
    # Reset otherRat variables
    otherRatID = None
    otherMap = np.empty(MAP_SIZE).flatten()
    otherRatMessages = 0

def findID(mac):
  for i in range(0, NUM_RATS):
    if (broadcastAddys[i] == mac):
      return i
  return None