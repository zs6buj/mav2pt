/*
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

*/
#include <queue> 
#include "frsky_sport.h"
#include "mavlite.h"

using namespace std; 

enum PassthroughPacketType : uint8_t {
    TYPE_TEXT =          0,  // 0x5000 status text (dynamic)
    TYPE_ATTITUDE =      1,  // 0x5006 Attitude and range (dynamic)
    TYPE_GPS =           2,  // 0x800 GPS lon
    TYPE_VEL_YAW =       3,  // 0x5005 Vel and Yaw
    TYPE_AP_STATUS =     4,  // 0x5001 AP status
    TYPE_GPS_STATUS =    5,  // 0x5002 GPS status
    TYPE_HOME =          6,  // 0x5004 Home
    TYPE_BATT_2 =        7,  // 0x5008 Battery 2 status
    TYPE_BATT_1 =        8,  // 0x5008 Battery 1 status
    TYPE_PARAM =         9,  // 0x5007 parameters
    TYPE_MAV =           10, // mavlite
};

//char name[] = "WP_RADIUS";
char name[] = "LAND_SPEED";
//float value = 55;
float value = 50;

uint32_t mav_tx_count = 0;
uint32_t mav_rx_count = 0;
uint32_t tx_count = 0;
uint32_t rx_count = 0;

//int sock_handle = 0;
//struct sockaddr_in serv_addr; 
bool connected;
bool reboot;

int16_t bytes_read;
uint8_t buffer[100];
uint32_t packet_stats[11];
  
bool flipflop = true;
uint8_t send_frame = 0;
uint8_t poll_frame_idx = 0;
uint8_t send_message_idx = 0;
const uint8_t poll_frame[5] { 0x00, 0x67, 0x0D, 0x34, 0x1B };

sport_parse_state_t sport_parse_state;

queue <sport_packet_t> rx_queue;
queue <sport_packet_t> tx_queue;

mavlite_message_t txmsg;
mavlite_status_t txstatus;

mavlite_message_t rxmsg;
mavlite_status_t rxstatus;

void dump_sport_frame(int sock, uint8_t frame, uint16_t appid, uint32_t data);
bool mavlite_send_message(mavlite_message_t &txmsg, mavlite_status_t &status);
void log_packet(uint8_t* buffer, uint8_t len);
