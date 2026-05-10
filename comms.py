"""
INTER-ROBOT COMMUNICATION PROTOCOL
"""



# MAC Addresses of all RAT ESP boards
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