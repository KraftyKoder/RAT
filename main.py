/#
  SOFTWARE FOR RAT BASTARD (Rolling Autonomous Tumbler)
  Written by Krrish Kainth
  Capstone 2026

  (Portion of code for ESP-Now communication from Rui Santos' tutorial @ https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
   and https://forum.arduino.cc/t/esp32-wifi-and-promiscuous-mode/1370360)
#/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// MAC Addresses of all RAT ESP boards
uint8_t broadcastAddys[][6] = {
                                {0x6c, 0xc8, 0x40, 0x8a, 0x9e, 0x3c}, 
                                {0x98, 0xa3, 0x16, 0x8e, 0x7c, 0x54}, 
                                {0xe4, 0x65, 0xb8, 0x6f, 0x2e, 0x10}
                              };
uint8_t myMACAdd[6];
const int NUM_RATS = sizeof(broadcastAddys)/6;
const int SQUEAK_RATE = 2000;

typedef struct data_packet {
  int senderID;
  int mazeMap[500][500];
} data_packet;

data_packet myMessage;

esp_now_peer_info_t friendInfo;
int latestRSSI = 0;
int rssiTable[NUM_RATS] = {0};
int myID = 0;

int mazeMap[500][500] = {0};
int MAP_MAX = 1000;  // max element magnitude
float worldX = 0;  // X position of robot relative to starting pos [cm]
float worldY = 0;  // Y position of robot relative to starting pos [cm]
int MAP_X_OFFSET = 250;
int MAP_Y_OFFSET = 499;
int MAP_SCALE = 1;  // grid squares per cm
int mapX = MAP_SCALE * worldX + MAP_X_OFFSET; // X index of robot in map matrix 
int mapY = MAP_SCALE * worldY + MAP_Y_OFFSET; // Y index of robot in map matrix

int OVERLAY_TOL = 10; // max distance to shift maps while overlaying

// Called whenever data sent
void OnDataSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Squeak Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Called whenever data received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&myMessage, incomingData, sizeof(myMessage));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Sender ID: ");
  Serial.println(myMessage.senderID);
  Serial.print("Signal Strength: ");
  latestRSSI = info->rx_ctrl->rssi;
  Serial.println(latestRSSI);
  rssiTable[myMessage.senderID] = latestRSSI;

  // Overlay received map with current global map, update global map with new info
}

// updateMap: add surroundings (LIDAR readings) to local map at estimated position, normalize map (?)
void updateMap(float lidarAngle, float lidarDist) {
  deltaX = lidarDist * cos(lidarAngle);
  deltaY = lidarDist * sin(lidarAngle);
  mazeMap[mapX + deltaX][mapY + deltaY] += 1;
  // RETHINK NORMALIZATION (dont want other values to decrease if many readings taken at one spot)
  // maybe just normalize cells around the measurement location
  //if (max(mazeMap) > MAP_MAX)
  //{
  //  mazeMap /= max(mazeMap) * MAP_MAX;
  //}
}

// overlayMaps: for array of position and orientation adjustments of map 1, determine best orientation and position for alignment with map 2
// Want to maximize values in map (common features), then normalize
// WANT TO MAXIMIZE CONTRAST!!
void overlayMaps(const int[][]* otherMap) {
  // Determining map overlay offset
  int overlayQuality[OVERLAY_TOL*2+1][OVERLAY_TOL*2+1] = {0};
  for (int x = -OVERLAY_TOL; x <= OVERLAY_TOL; x++) {
    for (int y = -OVERLAY_TOL; y <= OVERLAY_TOL; y++) {
      // shift contents of other map by x, y
      // overlap with my map
      // quantify quality of overlay
      // (maybe count number of elements with magnitude 2*MAP_MAX)
    }
  }

  // combine maps while normalizing overlap region
}

// Maze solving algorithm: DFS
// Send robots towards nearest unexplored part of maze
// queue of way points


void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
 
  // Setup WiFi and ESP-Now
  WiFi.mode(WIFI_MODE_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // put your setup code here, to run once:
  esp_err_t macReadStatus = esp_wifi_get_mac(WIFI_IF_STA, myMACAdd);
  if (macReadStatus == ESP_OK) {
    Serial.println("I got my MAC Address!");
  } else {
    Serial.println("I don't know my MAC Address :(");
  }

  // Compare MAC of this board to full list to determine myID
  for (int i = 0; i < NUM_RATS; i++) {
    if (memcmp(broadcastAddys[i], myMACAdd, 6)) {
      myID = i;
      break;
    }
  }

  // Setup send and receive functions
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSend));
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  

  for (int i = 0; i < NUM_RATS; i++) {
    //if (i != myID) {
      // Register peer
      memcpy(friendInfo.peer_addr, broadcastAddys[i], 6);
      friendInfo.channel = 0;
      friendInfo.encrypt = false;
      
      // Add peer        
      if (esp_now_add_peer(&friendInfo) != ESP_OK){
        Serial.println("Failed to add friend");
        return;
      }
    //}
  }


}

void loop() {
  // Estimate global position and orientation based on motor encoders + IMUs (Ash)
  // Outputs x, y, theta relative to starting position (world coordinates)
  worldX = ;
  worldY = ;

  // Convert position to map coordinates (row coln indices in map matrix)
  mapX = MAP_SCALE * worldX + MAP_X_OFFSET;
  mapY = MAP_SCALE * worldY + MAP_Y_OFFSET;

  // Collect lidar readings of surroundings
  // Output: angle and distance from current position and orientation
  lidarAngle = ;
  lidarDist = ;

  // update local map via combination of global position and LIDAR readings
  updateMap(lidarAngle, lidarDist);

  // decision making for which direction to travel in next (maze solving)

  // Every SQUEAK_RATE, send updated global map to all rats
  // CREATE MESSAGE
  myMessage.senderID = myID;
  myMessage.mazeMap = {0};

  // Send message via ESP-NOW
  for (int i = 0; i < NUM_RATS; i++) {
    if (i != myID) {
      esp_err_t result = esp_now_send(broadcastAddys[i], (uint8_t *) &myMessage, sizeof(myMessage));
      if (result == ESP_OK) {
        Serial.println("Successfully squeaked to ESP " + String(i));
      }
      else {
        Serial.println("Could not send squeak to ESP " + String(i));
      }
    }
  }

  delay(2000);
}
