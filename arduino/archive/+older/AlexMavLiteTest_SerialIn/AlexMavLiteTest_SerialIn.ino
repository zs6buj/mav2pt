/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
    Frsky SPort bidirectional telemetry test tool
    author: Alex Apostoli

    The idea is to simulate a "virtual" RX/TX connection where this tool does the polling just
    like a genuine frsky RX. It also simulates responses to the polling by sending bytes and decoding
    incoming ones.
    
    We assume a running SITL ArduPlane with following settings:
    
    SERIAL2_PROTOCOL = 10
    FRSKY_DNLINK1_ID = 20
    FRSKY_DNLINK2_ID = 7
    FRSKY_UPLINK_ID = 13

    The test will poll 5 frsky sensors in the following order
    0x00, 0x67, 0x0D, 0x34, 0x1B

    the assumption is this represents a common setup where ardupilot responds to uplink sensor polling (0x0D)
    and downlink sensor polling (0x67, 0x34, 0x1B) while not responding to external sensors polling bytes (0x00)

    When responding to 0x0D we send uplink packets (to the FC) and expect responses when we poll for 0x67, 0x34, 0x1B.
    The scheduler will interleave passthrough packets with mavlite ones, the expected result is something like:

    MAV:  send msgid=76, txcount=5, rxcount=4
    0x0D: 7E 0D 30 00 0F 4C F1 00 03 7F 
    0x34: recv: 0x5006 - 17.9%
    0x1B: recv: 0x5000 - 26.3%
    0x00: 
    0x67: recv: 7E 67 32 02 41 44 49 55 53 54 , mav 29.3%
    0x0D: 7E 0D 30 01 00 00 00 00 00 CE 
    0x34: recv: 0x5005 - 5.1%
    0x1B: recv: 0x5000 - 26.7%
    0x00: 
    0x67: recv: 7E 67 32 03 98 00 00 00 00 32 , mav 29.5%
    MAV:  recv msgid=22, txcount=6, rxcount=4
    0x0D: 7E 0D 30 02 00 00 00 00 00 CD 
    0x34: recv: 0x5006 - 17.7%
    0x1B: recv: 0x5000 - 27.0%
    0x00: 
    0x67: recv: 0x5003 - 3.1%
    0x0D: 7E 0D 30 03 80 3F 16 00 00 F6 
    0x34: recv: 7E 34 32 00 03 4D F1 00 00 8B , mav 29.2%
    0x1B: recv: 0x5000 - 27.3%
    0x00: 
    0x67: recv: 7E 67 32 01 43 00 00 00 00 89 , mav 29.9%
    MAV:  recv msgid=77, txcount=6, rxcount=5

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <arpa/inet.h> 
#include <unistd.h> 
#include <sys/socket.h> 
*/

#include "mavlite_test.h"
#define Log                 Serial         // USB
#define frSerial            Serial1   
  
uint32_t   frBaud = 57600;
uint8_t    frRx = 13;
uint8_t    frTx = 4;
bool       frInvert = true; // same as mav2pt in ground mode
int16_t    lth = 0;

int16_t    fr_idx = 0;
const      uint8_t fr_max = 20;
//=========================================================================
void setup() {
  Log.begin(115200);
  delay(2500);
  Log.println("Starting ..."); 
  
  frSerial.begin(frBaud, SERIAL_8N1, frRx, frTx, frInvert); 
}
//=========================================================================
void loop() {
    buffer[0] = 0x7E;
    buffer[1] = poll_frame[poll_frame_idx];
    // send the polling frame 0x7E + sensor id byte

    frSerial.write(buffer, 2);
    
    //send(sock_handle , buffer , 2 , 0);

    
    switch(poll_frame[poll_frame_idx]) {
        case 0x0D:  // poll and send
            send_data();
            break;
        case 0x00:  // poll and receive
        case 0x1B:  
        case 0x34:
        case 0x67:
            receive_data();
            break;
    }
    poll_frame_idx = (poll_frame_idx+1)%5;
}
//=========================================================================
byte read_byte() {

    byte b;
    
      if (lth == 0) {
        while (lth==0) {
          //Log.print(".");
          lth=frSerial.available();
        }
         //Log.printf(" lth=%3d\n",lth); 
      } 
  
      b = frSerial.read();
      lth--;
        
      return b;
}

//=========================================================================
int16_t read_frsky_frame(uint8_t *_buf) {

  int16_t byt = 0;
  static uint8_t prevbyt = 0;
  static bool ft = true;
  
  //Log.println("\nread_frsky_frame()");

    while (!frSerial.available()) {  // wait for data
      //Log.print(".");
      delay(0); // yield to rtos 
    }
      
    while (frSerial.available())   { 

        byt = read_byte();          
         
        if (fr_idx > fr_max -1) {
          Log.println("read buff overflow ignored");
          fr_idx--;
        }

        if (ft) {
          ft = false;
          while (byt != 0x7E) {  // find first time start / stop char 
            byt = read_byte();   
          }
        }
         
        if (byt == 0x7E) {       // this will be the start of the next frame, so go process the one in buffer
          *(_buf) = byt;         // [0]   
          //Log.printf("idx=%d byt=%2X Found 0x7E \n", 0, byt);         
          int16_t idx = fr_idx;  
          fr_idx = 1;            // [1]
          return idx;            // the only exit
        } 
        
        //Log.printf("idx=%d byt=%2X \n", fr_idx, byt);
        *(_buf + fr_idx) = byt;           
        fr_idx++;  
        prevbyt=byt;
    }  
    return -1;   // avoid blocking
}
//=========================================================================
bool receive_data() {
    Log.printf("0x%02X: ", buffer[1]);
    if (poll_frame_idx == 0 || poll_frame_idx == 0x0D) {
    Log.printf("\n");
        return false;
    }
    // read response
    bytes_read = read_frsky_frame(&buffer[0]);

    if (bytes_read == -1) {
        Log.printf("\n");
        return false;
    } 
    
    Log.printf("bytes_read=%d : ", bytes_read);
    
    for (int i = 0; i < bytes_read ; i++) {  
      Log.printf("<%2X", buffer[i]);
    }
    Log.println();
    
    // bytes_read = recv(sock_handle, &buffer[2], 20, 0);
    // process packet only if we actually did read some bytes
    

    Log.printf("recv: "); 
    for (uint8_t i=0; i<bytes_read+2; i++) {
        if (parse_sport_telemetry_data(buffer[i], sport_parse_state)) {
            // successfully parsed packet, let's check the CRC
            rx_count++;
            if (should_process_sport_packet(sport_parse_state.telemetry_rx_buffer)) {
              
                // is this a mavlite packet?
                if (buffer[2] == 0x32) {
                    Log.print(" Found 0x32 ");
                    log_packet(buffer, bytes_read+2);
                    for (uint8_t j=0; j<6; j++) {
                     //   Log.printf(" %2X", buffer[j]);                    
                        mavlite_rx_parse(sport_parse_state.telemetry_rx_buffer[j+2],j, rxmsg, rxstatus);
                    }
                    packet_stats[TYPE_MAV]++;
                    Log.printf(", mav %02.01f%%\n", 100*packet_stats[TYPE_MAV]*1.0f/rx_count);

                    if (rxstatus.parse_state == ParseState::MESSAGE_RECEIVED) {
                    Log.printf("MAV:  recv msgid=%d, txcount=%u, rxcount=%u\n", rxmsg.msgid, mav_tx_count, mav_rx_count++); 
                    }
                } else {
                    // log the passthrough DIY packet type
                    uint16_t key = buffer[4] << 8 | buffer[3];
                    uint8_t key_idx = 0;
                    switch(key) {
                        case 0x5000:    //status text (dynamic)
                            key_idx = TYPE_TEXT;
                            break;
                        case 0x5006:    //Attitude and range (dynamic)
                            key_idx = TYPE_ATTITUDE;
                            break;
                        case 0x800:    //GPS
                            key_idx = TYPE_GPS;
                            break;
                        case 0x5005:    //vel and yaw
                            key_idx = TYPE_VEL_YAW;
                            break;
                        case 0x5001:    //AP status
                            key_idx = TYPE_AP_STATUS;
                            break;
                        case 0x5002:    //GPS status
                            key_idx = TYPE_GPS_STATUS;
                            break;
                        case 0x5004:    //home status
                            key_idx = TYPE_HOME;
                            break;
                        case 0x5008:    //bat2 status
                            key_idx = TYPE_BATT_2;
                            break;
                        case 0x5003:    //bat1 status
                            key_idx = TYPE_BATT_1;
                            break;
                        case 0x5007:    //params
                            key_idx = TYPE_PARAM;
                            break;
                    }
                    packet_stats[key_idx]++;
                Log.printf("0x%04X - %02.1f%%\n", key, 100*packet_stats[key_idx]*1.0f/rx_count);
                }
            }
        }
    }
    return true;
}
//=========================================================================
void send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data)
{
    uint8_t buf[8];                 // packet buffer
    uint8_t buf2[sizeof(buf)*2+1];  // byte stuffing buffer

    buf[0] = frame;
    buf[1] = appid & 0xFF;
    buf[2] = appid >> 8;
    memcpy(&buf[3], &data, 4);

    uint16_t sum = 0;
    for (uint8_t i=0; i<sizeof(buf)-1; i++) {
        sum += buf[i];
        sum += sum >> 8;
        sum &= 0xFF;
    }
    sum = 0xff - ((sum & 0xff) + (sum >> 8));
    buf[7] = (uint8_t)sum;

    // perform byte stuffing per SPort spec
    uint8_t len = 0;

    for (uint8_t i=0; i<sizeof(buf); i++) {
        uint8_t c = buf[i];
        if (c == FRAME_DLE || buf[i] == FRAME_HEAD) {
            buf2[len++] = FRAME_DLE;
            buf2[len++] = c ^ FRAME_XOR;
        } else {
            buf2[len++] = c;
        }
    }
    Log.printf("7E 0D ");
    for (uint8_t i=0; i<len; i++) {
        Log.printf("%02X ",buf2[i]);
    }
    Log.printf("\n");

    frSerial.write(buf2, len);
    
  //  if (sock > 0) {
  //     send(sock, buf2, len, 0);
  //  }
}
//=========================================================================
bool mavlite_send_message(mavlite_message_t &txmsg, mavlite_status_t &status)
{
    // prevent looping forever
    uint8_t packet_count = 0;
    while (status.parse_state != ParseState::MESSAGE_RECEIVED && packet_count++ < MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN)) {
        sport_packet_t packet {};
        for (uint8_t i=0; i<6; i++) {
            mavlite_tx_encode(packet.raw[i+2],i, txmsg, status);
        }
        if (status.parse_state == ParseState::ERROR) {
            break;
        }
        // queue the packet for sending
        tx_queue.push(packet);
    }
    status.parse_state = ParseState::IDLE;
    return true;
}
//=========================================================================
bool should_process_sport_packet(const uint8_t *packet)
{
    //check CRC
    int16_t crc = 0;
    for (uint8_t i=1; i<SPORT_PACKET_SIZE; ++i) {
        crc += packet[i]; // 0-1FE
        crc += crc >> 8;  // 0-1FF
        crc &= 0x00ff;    // 0-FF
    }
    return (crc == 0x00ff);
}
//=========================================================================
void queue_sport_rx_packet(const uint8_t *packet)
{
    if (!should_process_sport_packet(packet)) {
        Log.printf("crc failed\n");
        return;
    }

    const sport_packet_t sp {
        sport_parse_state.telemetry_rx_buffer[0],
        sport_parse_state.telemetry_rx_buffer[1],
        le16toh_ptr(&sport_parse_state.telemetry_rx_buffer[2]),
        le32toh_ptr(&sport_parse_state.telemetry_rx_buffer[4])
    };

    rx_queue.push(sp);
}
//=========================================================================
void log_packet(uint8_t* buffer, uint8_t len) {
    for (uint8_t i=0; i<len; i++) {
        Log.printf("%02X ",buffer[i]&0xFF);
    }
}

//=========================================================================
void send_data() {
    if (tx_queue.empty()) {
        mavlite_init(txmsg, txstatus);
        switch(send_message_idx) {
            case 0:
                txmsg.msgid=20; //20 read, 23 set, 22 value
                mavlite_msg_set_string(txmsg,name,0);
                break;
            case 1:
                txmsg.msgid=23; //20 read, 23 set, 22 value
                mavlite_msg_set_float(txmsg,value,0);
                mavlite_msg_set_string(txmsg,name,4);
                break;
            case 2:
                if (reboot) {
                    txmsg.msgid=76;
                    uint16_t cmd_id = 246;          // reboot
                    uint8_t param_count = 2;
                    uint8_t cmd_confirmation = 0;
                    uint8_t cmd_options = 0;
                    float param1 = 1;
                    float param2 = 0;
                    bit8_pack(cmd_options,param_count,3,0);
                    bit8_pack(cmd_options,cmd_confirmation,5,3);
                    mavlite_msg_set_uint16(txmsg,cmd_id,0);
                    mavlite_msg_set_uint8(txmsg,cmd_options,2);
                    mavlite_msg_set_float(txmsg,param1,3);
                    mavlite_msg_set_float(txmsg,param2,3+4);
                } else {
                    txmsg.msgid=76;
                    uint16_t cmd_id = 241;          // calibrate ground pressure
                    uint8_t param_count = 3;
                    uint8_t cmd_confirmation = 0;
                    uint8_t cmd_options = 0;
                    float param1 = 0;
                    float param2 = 0;
                    float param3 = 1;
                    bit8_pack(cmd_options,param_count,3,0);
                    bit8_pack(cmd_options,cmd_confirmation,5,3);
                    mavlite_msg_set_uint16(txmsg,cmd_id,0);
                    mavlite_msg_set_uint8(txmsg,cmd_options,2);
                    mavlite_msg_set_float(txmsg,param1,3);
                    mavlite_msg_set_float(txmsg,param2,3+4);
                    mavlite_msg_set_float(txmsg,param3,3+8);
                }
                break;
        }
        send_message_idx = (send_message_idx+1)%3;

        mavlite_send_message(txmsg, txstatus);
        Log.printf("MAV:  send msgid=%d, txcount=%u, rxcount=%u\n", txmsg.msgid, mav_tx_count++, mav_rx_count); 
    }
    Log.printf("0x%02X: send: ", poll_frame[poll_frame_idx]); 
    if (send_frame == 0) {
        if (!tx_queue.empty()) {
            sport_packet_t packet = tx_queue.front();
            send_sport_frame(0x30, packet.appid, packet.data);
            tx_queue.pop();
            tx_count++;
        }
    } else {
        Log.printf("\n"); 
    }
    //send_frame = (send_frame+1)%3; // simulate some delay from the sending party and skip a couple frames each iteration
}
