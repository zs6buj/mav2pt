/*****************************************************************************************************************

     Mav2PT  (Mav2Passthru) Protocol Translator

 
     License and Disclaimer

 
  This software is provided under the GNU v2.0 License. All relevant restrictions apply. In case there is a conflict,
  the GNU v2.0 License is overriding. This software is provided as-is in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  General Public License for more details. In no event will the authors and/or contributors be held liable for any 
  damages arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose, including commercial 
  applications, and to alter it and redistribute it freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software.
  2. If you use this software in a product, an acknowledgment in the product documentation would be appreciated.
  3. Altered versions must be plainly marked as such, and must not be misrepresented as being the original software.
  4. This notice may not be removed or altered from any distribution.  

  By downloading this software you are agreeing to the terms specified in this page and the spirit of thereof.
    
    ******************************************************************************************************************

    Author: Eric Stockenstrom
    
        
    Inspired by original S.Port firmware by Rolf Blomgren
    
    Acknowledgements and thanks to Craft and Theory (http://www.craftandtheoryllc.com/) for
    the Mavlink / Frsky Passthru protocol

    Many thanks to yaapu for advice and testing, and his excellent LUA script

    Thanks also to athertop for advice and testing, and to florent for advice on working with FlightDeck

    Acknowledgement and thanks to the author of, and contributors to, mavESP8266 serial/Wifi bridge
    
    *****************************************************************************

    See: https://github.com/zs6buj/MavlinkToPassthru/wiki
    
    While the DragonLink and Orange UHF Long Range RC and telemetry radio systems deliver 
    a two-way Mavlink link, the FrSky Taranis and Horus hand-held RC controllers expect
    to receive FrSky S.Port protocol telemetry for display on their screen.  Excellent 
    firmware is available to convert Mavlink to the native S.Port protocol, however the 
    author is unaware of a suitable solution to convert to the Passthru protocol. 

    This protocol translator has been especially tailored to work with the excellent LUA 
    display interface from yaapu for the FrSky Horus, Taranis and QX7 controllers. 
    https://github.com/yaapu/FrskyTelemetryScript . The translator also works with the 
    popular FlightDeck product.
    
    The firmware translates APM or PX4 Mavlink telemetry to FrSky S.Port passthru telemetry, 
    and is designed to run on an ESP32, Teensy 3.2, or cheap STM32F103 board (with a signal 
    inverter/converter). The ESP32 implementation supports Bluetooth, WiFI and SD card I/O 
    into and out of the translator, so for example, Mavlink telemetry can be fed directly into Mission
    Planner or QGround Control.

    The performance of the translator on the ESP32 platform is superior to that of the other boards.
    However, the Teensy 3.x is much smaller and fits neatly into the back bay of a Taranis or Horus
    transmitter. The STM32F103C boards are more affordable, but require external inverters/converters 
    for single wire S.Port connection.

    The PLUS version adds additional sensor IDs to Mavlink Passthru protocol DIY range

    The translator can work in one of three modes: Ground_Mode, Air_Mode or Relay_Mode

    Ground_Mode
    In ground mode, it is located in the back of the Taranis/Horus. Since there is no FrSky receiver
    to provide sensor polling, a routine in the firmware emulates FrSky receiver sensor polling. (It
    pretends to be a receiver for polling purposes). 
   
    Un-comment this line       #define Ground_Mode      like this.

    Air_Mode
    In air mode, it is located on the aircraft between the FC and a Frsky receiver. It translates 
    Mavlink out of a Pixhawk and feeds passthru telemetry to the Frsky receiver, which sends it 
    to the Taranis on the ground. In this situation it responds to the FrSky receiver's sensor 
    polling. The APM firmware can deliver passthru telemetry directly without this translator, but as 
    of July 2019 the PX4 Pro firmware cannot, and therefor requires this translator. 
   
    Un-comment this line      #define Air_Mode    like this
   
    Relay_Mode
    Consider the situation where an air-side LRS UHF tranceiver (trx) (like the DragonLink or Orange), 
    communicates with a matching ground-side UHF trx located in a "relay" box using Mavlink 
    telemetry. The UHF trx in the relay box feeds Mavlink telemtry into our passthru translator, and 
    the ctranslator feeds FrSky passthru telemtry into the FrSky receiver (like an XSR), also 
    located in the relay box. The XSR receiver (actually a tranceiver - trx) then communicates on 
    the public 2.4GHz band with the Taranis on the ground. In this situation the translator need not 
    emulate sensor polling, as the FrSky receiver will provide it. However, the translator must 
    determine the true rssi of the air link and forward it, as the rssi forwarded by the FrSky 
    receiver in the relay box will incorrectly be that of the short terrestrial link from the relay
    box to the Taranis.  To enable Relay_Mode :
    Un-comment this line      #define Relay_Mode    like this

    From version 2.12 the target mpu is selected automatically

    Battery capacities in mAh can be 
   
    1 Requested from the flight controller via Mavlink
    2 Defined within this firmware  or 
    3 Defined within the LUA script on the Taranis/Horus. This is the prefered method.
     
    N.B!  The dreaded "Telemetry Lost" enunciation!

    The popular LUA telemetry scripts use RSSI to determine that a telemetry connection has been successfully established 
    between the 'craft and the Taranis/Horus. Be sure to set-up RSSI properly before testing the system.


  ***************************************************************************************************************** 


   Connections to ESP32 and ESP8266 boards depend on the board variant

    Go to config.h tab and look for "S E L E C T   E S P   B O A R D   V A R I A N T" 

    
   Connections to Teensy3.2 are:
    0) USB                         Flashing and serial monitor for debug
    1) SPort S     -->tx1 Pin 1    S.Port out to XSR  or Taranis bay, bottom pin
    2) Mavlink_In  <--rx2 Pin 9    Mavlink source to Teensy - FC_Mav_rxPin
    3) Mavlink_In  -->tx2 Pin 10   Mavlink source to Taranis
    4) Mavlink_Out <--rx3 Pin 7    Optional feature - see #defined
    5) Mavlink_Out -->tx3 Pin 8    Optional feature - see #defined
    6) MavStatusLed       Pin 13   BoardLed
    7) BufStatusLed  1
    8) Vcc 3.3V !
    9) GND

   Connections to Blue Pill STM32F103C  are:
   
    0) USB/TTL     UART0   -->tx1 Pin A9   Flashing and serial monitor for debug
    0) USB/TTL     UART0   -->rx1 Pin A10 
    
    1) SPort S     UART1   -->tx2 Pin A2   Serial1 to inverter, convert to single wire then to S.Port
    2) SPort S     UART1   <--rx2 Pin A3   Serial1 To inverter, convert to single wire then to S.Port
    3) Mavlink_In  UART2   <--rx3 Pin B11  Serial2 Mavlink source to STM32 - FC_Mav_rxPin 
    4) Mavlink_In  UART2   -->tx3 Pin B10  Serial2 Mavlink source STM32 
    5) MavStatusLed               Pin C13  BoardLed
    6) BufStatusLed               Pin C14     
    7) Vcc 3.3V !
    8) GND

   Connections to Maple Mini STM32F103C are:
    0) USB                          Flashing and serial monitor for debug
    1) SPort S     -->tx1 Pin A10   Serial1 to inverter, convert to single wire then to S.Port
    2) SPort S     <--rx1 Pin A9    Serial1 To inverter, convert to single wire then to S.Port
    3) Mavlink_In  <--rx2 Pin A3    8  Serial2 Mavlink source to STM32 - FC_Mav_rxPin  
    4) Mavlink_In  -->tx2 Pin A2       Serial2 Mavlink from STM32 
    5) MavStatusLed       Pin B1    33     
    6) BufStatusLed       Pin       34 
    7) Vcc 3.3V !
    8) GND   
    
*/
//*****************************************************************************************************


#undef F                         // F defined in c_library_v2\mavlink_sha256.h AND teensy3/WString.h
#include "config.h"
#include <CircularBuffer.h>

#include <mavlink_types.h>
#include <common/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>

uint8_t   MavLedState = LOW; 
uint8_t   BufLedState = LOW; 
 
uint32_t  hb_count=0;
uint32_t  sens_buf_full_count = 0;

bool      ap_bat_paramsReq = false;
bool      ap_bat_paramsRead=false; 
bool      parm_msg_shown = false;
bool      ap_paramsList=false;
uint8_t   paramsID=0;

bool      homGood = false;      
bool      mavGood = false;
bool      rssiGood = false;
bool      rssi35 = false;
bool      rssi65 = false;
bool      rssi109 = false;

bool      wifiSuGood = false;
bool      wifiSuDone = false;
bool      timeGood = false;
bool      ftGetBaud = true;
uint8_t   sdStatus = 0; // 0=no reader, 1=reader found, 2=SD found, 3=open for append 4 = open for read, 9=failed

uint32_t  hb_millis=0;
uint32_t  sport_millis=0;  
uint32_t  fchb_millis=0;
uint32_t  rds_millis=0;
uint32_t  acc_millis=0;
uint32_t  em_millis=0;
uint32_t  sp_millis=0;
uint32_t  mav_led_millis=0;
uint32_t  health_millis = 0;
uint32_t  rssi_millis = 0;

uint32_t  now_millis = 0;
uint32_t  prev_millis = 0;

float   lon1,lat1,lon2,lat2,alt1,alt2;  
//************************************
// 4D Location vectors
 struct Location {
  float lat; 
  float lon;
  float alt;
  float hdg;
  };
volatile struct Location hom     = {
  0,0,0,0};   // home location

volatile struct Location cur      = {
  0,0,0,0};   // current location  
   
struct Loc2D {
  float     lat; 
  float     lon;
  };
  
 Loc2D WP[Max_Waypoints]; 

//************************************
struct Battery {
  float    mAh;
  float    tot_mAh;
  float    avg_dA;
  float    avg_mV;
  uint32_t prv_millis;
  uint32_t tot_volts;      // sum of all samples
  uint32_t tot_mW;
  uint32_t samples;
  bool ft;
  };
  
struct Battery bat1     = {
  0, 0, 0, 0, 0, 0, 0, true};   

struct Battery bat2     = {
  0, 0, 0, 0, 0, 0, 0, true};   

    
// ****************************************** M A V L I N K *********************************************

mavlink_message_t   F2Rmsg, R2Gmsg, G2Fmsg;

//uint8_t           FCbuf[MAVLINK_MAX_PACKET_LEN];
//uint8_t           GCSbuf[MAVLINK_MAX_PACKET_LEN]; 

uint8_t             FCbuf[300];
uint8_t             GCSbuf[300]; 

bool                GCS_available = false;
uint16_t            len;

// Mavlink Messages

// Mavlink Header
uint8_t    ap_sysid;
uint8_t    ap_compid;
uint8_t    ap_targcomp;

uint8_t    ap_targsys;     //   System ID of target system - outgoing to FC

uint8_t    mvType;

// Message #0  HEARTHBEAT 
uint8_t    ap_type_tmp = 0;              // hold the type until we know HB not from GCS or Tracker
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 0;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;
bool       px4_flight_stack = false;
uint8_t    px4_main_mode = 0;
uint8_t    px4_sub_mode = 0;

// Message #0  GCS HEARTHBEAT 

uint8_t    gcs_type = 0;
uint8_t    gcs_autopilot = 0;
uint8_t    gcs_base_mode = 0;
uint32_t   gcs_custom_mode = 0;
uint8_t    gcs_system_status = 0;
uint8_t    gcs_mavlink_version = 0;

// Message #0  Outgoing HEARTHBEAT 
uint8_t    apo_sysid;
uint8_t    apo_compid;
uint8_t    apo_targcomp;
uint8_t    apo_mission_type;              // Mav2
uint8_t    apo_type = 0;
uint8_t    apo_autopilot = 0;
uint8_t    apo_base_mode = 0;
uint32_t   apo_custom_mode = 0;
uint8_t    apo_system_status = 0;

// Message # 1  SYS_status 
uint32_t   ap_onboard_control_sensors_health;  //Bitmap  0: error. Value of 0: error. Value of 1: healthy.
uint16_t   ap_voltage_battery1 = 0;    // 1000 = 1V
int16_t    ap_current_battery1 = 0;    //  10 = 1A
uint8_t    ap_ccell_count1= 0;

// Message # 2  SYS_status 
uint64_t  ap_time_unix_usec;          // us  Timestamp (UNIX epoch time).
uint32_t  ap_time_boot_ms;            // ms  Timestamp (time since system boot)

// Message #20 PARAM_REQUEST_READ    // outgoing request to read the onboard parameter with the param_id string id
uint8_t  gcs_targsys;            //   System ID
char     gcs_req_param_id[16];   //  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
int16_t  gcs_req_param_index;    //  Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
// param_index . Send -1 to use the param ID field as identifier (else the param id will be ignored)

float ap_bat1_capacity;
float ap_bat2_capacity;

// Message #21 PARAM_REQUEST_LIST 
//  Generic Mavlink Header defined above
  
// Message #22 PARAM_VALUE
char     ap_param_id [16]; 
float    ap_param_value;
uint8_t  ap_param_type;  
uint16_t ap_param_count;              //  Total number of onboard parameters
int16_t ap_param_index;              //  Index of this onboard parameter

// Message #24  GPS_RAW_INT 
uint8_t    ap_fixtype = 3;            // 0= No GPS, 1=No Fix, 2=2D Fix, 3=3D Fix, 4=DGPS, 5=RTK_Float, 6=RTK_Fixed, 7=Static, 8=PPP
uint8_t    ap_sat_visible = 0;        // numbers of visible satelites
int32_t    ap_lat24 = 0;              // 7 assumed decimal places
int32_t    ap_lon24 = 0;              // 7 assumed decimal places
int32_t    ap_amsl24 = 0;             // 1000 = 1m
uint16_t   ap_eph;                    // GPS HDOP horizontal dilution of position (unitless)
uint16_t   ap_epv;                    // GPS VDOP vertical dilution of position (unitless)
uint16_t   ap_vel;                    // GPS ground speed (m/s * 100) cm/s
uint16_t   ap_cog;                    // Course over ground in degrees * 100, 0.0..359.99 degrees
// mav2
int32_t    ap_alt_ellipsoid;          // mm    Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
uint32_t   ap_h_acc;                  // mm    Position uncertainty. Positive for up.
uint32_t   ap_v_acc;                  // mm    Altitude uncertainty. Positive for up.
uint32_t   ap_vel_acc;                // mm    Speed uncertainty. Positive for up.
uint32_t   ap_hdg_acc;                // degE5   Heading / track uncertainty

// Message #27 RAW IMU 
int32_t   ap_accX = 0;
int32_t   ap_accY = 0;
int32_t   ap_accZ = 0;

// Message #29 SCALED_PRESSURE
float      ap_press_abs;         // Absolute pressure (hectopascal)
float      ap_press_diff;        // Differential pressure 1 (hectopascal)
int16_t    ap_temperature;       // Temperature measurement (0.01 degrees celsius)

// Message ATTITUDE ( #30 )
float ap_roll;                   // Roll angle (rad, -pi..+pi)
float ap_pitch;                  // Pitch angle (rad, -pi..+pi)
float ap_yaw;                    // Yaw angle (rad, -pi..+pi)
float ap_rollspeed;              // Roll angular speed (rad/s)
float ap_pitchspeed;             // Pitch angular speed (rad/s)
float ap_yawspeed;               // Yaw angular speed (rad/s)

// Message GLOBAL_POSITION_INT ( #33 ) (Filtered)
int32_t ap_lat33;          // Latitude, expressed as degrees * 1E7
int32_t ap_lon33;          // Longitude, expressed as degrees * 1E7
int32_t ap_amsl33;         // Altitude above mean sea level (millimeters)
int32_t ap_alt_ag;         // Altitude above ground (millimeters)
int16_t ap_vx;             // Ground X Speed (Latitude, positive north), expressed as m/s * 100
int16_t ap_vy;             // Ground Y Speed (Longitude, positive east), expressed as m/s * 100
int16_t ap_vz;             // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
uint16_t ap_gps_hdg;           // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees

// Message #35 RC_CHANNELS_RAW
uint8_t ap_rssi;
uint8_t ap_rssi35;

// Message #36 Servo_Output
bool      ap_servo_flag = false;  // true when servo_output record received
uint8_t   ap_port; 
uint16_t  ap_servo_raw[16];       // 16 channels, [0] thru [15] 

// Message #39 Mission_Item
//  Generic Mavlink Header defined above
uint16_t  ap_ms_seq;            // Sequence
uint8_t   ap_ms_frame;          // The coordinate system of the waypoint.
uint16_t  ap_ms_command;        // The scheduled action for the waypoint.
uint8_t   ap_ms_current;        // false:0, true:1
uint8_t   ap_ms_autocontinue;   //  Autocontinue to next waypoint
float     ap_ms_param1;         // PARAM1, see MAV_CMD enum
float     ap_ms_param2;         // PARAM2, see MAV_CMD enum
float     ap_ms_param3;         // PARAM3, see MAV_CMD enum
float     ap_ms_param4;         // PARAM4, see MAV_CMD enum
float     ap_ms_x;              // PARAM5 / local: X coordinate, global: latitude
float     ap_ms_y;              // PARAM6 / local: Y coordinate, global: longitude
float     ap_ms_z;              // PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
uint8_t   ap_mission_type;      // MAV_MISSION_TYPE - Mavlink 2

// Message #40 Mission_Request
//  Generic Mavlink Header defined above
//uint8_t   ap_mission_type;  

// Message #42 Mission_Current
//  Generic Mavlink Header defined above
bool ap_ms_current_flag = false;

// Message #43 Mission_Request_List
//  Generic Mavlink Header defined above
bool ap_ms_list_req = false;

// Message #44 Mission_Count
//  Generic Mavlink Header defined above
uint8_t   ap_mission_count = 0;
bool      ap_ms_count_ft = true;

// Message #51 Mission_Request_Int    From GCS to FC - Request info on mission seq #

uint8_t    gcs_target_system;    // System ID
uint8_t    gcs_target_component; // Component ID
uint16_t   gcs_seq;              // Sequence #
uint8_t    gcs_mission_type;      

// Message #62 Nav_Controller_Output
float     ap_nav_roll;           // Current desired roll
float     ap_nav_pitch;          // Current desired pitch
int16_t   ap_nav_bearing;        // Current desired heading
int16_t   ap_target_bearing;     // Bearing to current waypoint/target
uint16_t  ap_wp_dist;            // Distance to active waypoint
float     ap_alt_error;          // Current altitude error
float     ap_aspd_error;         // Current airspeed error
float     ap_xtrack_error;       // Current crosstrack error on x-y plane

// Message #65 RC_Channels
bool      ap_rc_flag = false;    // true when rc record received
uint8_t   ap_chcnt; 
uint16_t  ap_chan_raw[18];       // 16 + 2 channels, [0] thru [17] 

//uint16_t ap_chan16_raw;        // Used for RSSI uS 1000=0%  2000=100%
uint8_t  ap_rssi65;              // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown

// Message #73 Mission_Item_Int
uint8_t   ap73_target_system;    
uint8_t   ap73_target_component;    
uint16_t  ap73_seq;             // Waypoint ID (sequence number)
uint8_t   ap73_frame;           // MAV_FRAME The coordinate system of the waypoint.
uint16_t  ap73_command;         // MAV_CMD The scheduled action for the waypoint.
uint8_t   ap73_current;         // false:0, true:1
uint8_t   ap73_autocontinue;    // Autocontinue to next waypoint
float     ap73_param1;          // PARAM1, see MAV_CMD enum
float     ap73_param2;          // PARAM2, see MAV_CMD enum
float     ap73_param3;          // PARAM3, see MAV_CMD enum
float     ap73_param4;          // PARAM4, see MAV_CMD enum
int32_t   ap73_x;               // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
int32_t   ap73_y;               // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
float     ap73_z;               // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
uint8_t   ap73_mission_type;    // Mav2   MAV_MISSION_TYPE  Mission type.

// Message #74 VFR_HUD  
float    ap_hud_air_spd;
float    ap_hud_grd_spd;
int16_t  ap_hud_hdg;
uint16_t ap_hud_throt;
float    ap_hud_bar_alt;   
float    ap_hud_climb;        

// Message #109 RADIO_status (Sik radio firmware)
uint8_t ap_rssi109;             // local signal strength
uint8_t ap_remrssi;             // remote signal strength
uint8_t ap_txbuf;               // how full the tx buffer is as a percentage
uint8_t ap_noise;               // background noise level
uint8_t ap_remnoise;            // remote background noise level
uint16_t ap_rxerrors;           // receive errors
uint16_t ap_fixed;              // count of error corrected packets

// Message  #125 POWER_status 
uint16_t  ap_Vcc;                 // 5V rail voltage in millivolts
uint16_t  ap_Vservo;              // servo rail voltage in millivolts
uint16_t  ap_flags;               // power supply status flags (see MAV_POWER_status enum)
/*
 * MAV_POWER_status
Power supply status flags (bitmask)
1   MAV_POWER_status_BRICK_VALID  main brick power supply valid
2   MAV_POWER_status_SERVO_VALID  main servo power supply valid for FMU
4   MAV_POWER_status_USB_CONNECTED  USB power is connected
8   MAV_POWER_status_PERIPH_OVERCURRENT peripheral supply is in over-current state
16  MAV_POWER_status_PERIPH_HIPOWER_OVERCURRENT hi-power peripheral supply is in over-current state
32  MAV_POWER_status_CHANGED  Power status has changed since boot
 */

// Message  #147 BATTERY_status 
uint8_t      ap_battery_id;       
uint8_t      ap_battery_function;
uint8_t      ap_bat_type;  
int16_t      ap_bat_temperature;    // centi-degrees celsius
uint16_t     ap_voltages[10];       // cell voltages in millivolts 
int16_t      ap_current_battery;    // in 10*milliamperes (1 = 10 milliampere)
int32_t      ap_current_consumed;   // mAh
int32_t      ap_energy_consumed;    // HectoJoules (intergrated U*I*dt) (1 = 100 Joule)
int8_t       ap_battery_remaining;  // (0%: 0, 100%: 100)
int32_t      ap_time_remaining;     // in seconds
uint8_t      ap_charge_state;     

// Message #166 RADIO see #109


// Message #173 RANGEFINDER 
float ap_range; // m

// Message #181 BATTERY2 
uint16_t   ap_voltage_battery2 = 0;    // 1000 = 1V
int16_t    ap_current_battery2 = 0;    //  10 = 1A
uint8_t    ap_cell_count2 = 0;

// Message #253 STATUSTEXT
 uint8_t   ap_severity;
 char      ap_text[60];  // 50 plus padding
 uint8_t   ap_txtlth;
 bool      ap_simple=0;
 

//***************************************************************
// FrSky Passthru Variables
uint32_t  fr_payload;

// 0x800 GPS
uint8_t ms2bits;
uint32_t fr_lat = 0;
uint32_t fr_lon = 0;

// 0x5000 Text Msg
uint32_t fr_textmsg;
char     fr_text[60];
uint8_t  fr_severity;
uint8_t  fr_txtlth;
char     fr_chunk[4];
uint8_t  fr_chunk_num;
uint8_t  fr_chunk_pntr = 0;  // chunk pointer
char     fr_chunk_print[5];

// 0x5001 AP Status
uint8_t fr_flight_mode;
uint8_t fr_simple;

uint8_t fr_land_complete;
uint8_t fr_armed;
uint8_t fr_bat_fs;
uint8_t fr_ekf_fs;

// 0x5002 GPS Status
uint8_t fr_numsats;
uint8_t fr_gps_status;           // part a
uint8_t fr_gps_adv_status;       // part b
uint8_t fr_hdop;
uint32_t fr_amsl;

uint8_t neg;

//0x5003 Batt
uint16_t fr_bat1_volts;
uint16_t fr_bat1_amps;
uint16_t fr_bat1_mAh;

// 0x5004 Home
uint16_t fr_home_dist;
int16_t  fr_home_angle;       // degrees
int16_t  fr_home_arrow;       // 0 = heading pointing to home, unit = 3 degrees
int16_t  fr_home_alt;

short fr_pwr;

// 0x5005 Velocity and yaw
uint32_t fr_velyaw;
float fr_vy;    // climb in decimeters/s
float fr_vx;    // groundspeed in decimeters/s
float fr_yaw;   // heading units of 0.2 degrees

// 0x5006 Attitude and range
uint16_t fr_roll;
uint16_t fr_pitch;
uint16_t fr_range;

// 0x5007 Parameters  
uint8_t  fr_param_id ;
uint32_t fr_param_val;
uint32_t fr_frame_type;
uint32_t fr_bat1_capacity;
uint32_t fr_bat2_capacity;
uint32_t fr_mission_count;
bool     fr_paramsSent = false;

//0x5008 Batt2
float fr_bat2_volts;
float fr_bat2_amps;
uint16_t fr_bat2_mAh;

//0x5009 Servo_raw         // 4 ch per frame
uint8_t  frPort; 
int8_t   fr_sv[5];       

//0x5010 HUD
float    fr_air_spd;       // dm/s
uint16_t fr_throt;         // 0 to 100%
float    fr_bar_alt;       // metres

//0x500B Missions       
uint16_t  fr_ms_seq;                // WP number
uint16_t  fr_ms_dist;               // To next WP  
float     fr_ms_xtrack;             // Cross track error in metres
float     fr_ms_target_bearing;     // Direction of next WP
float     fr_ms_cog;                // Course-over-ground in degrees
int8_t    fr_ms_offset;             // Next WP bearing offset from COG

//0xF103
uint32_t fr_rssi;

//**************************** Ring and Sensor Buffers *************************

// Give the ESP32 more space, because it has much more RAM
#ifdef ESP32
  CircularBuffer<mavlink_message_t, 30> MavRingBuff;
#else 
  CircularBuffer<mavlink_message_t, 10> MavRingBuff;
#endif

// Scheduler buffer
 typedef struct  {
  uint16_t   id;
  uint8_t    subid;
  uint32_t   millis; // mS since boot
  uint32_t   payload;
  bool       inuse;
  } sb_t;

// Give the sensor table more space when status_text messages sent three times
#if defined Send_status_Text_3_Times
   const uint16_t sb_rows = 300;  // possible unsent sensor ids at any moment 
#else 
   const uint16_t sb_rows = 130;  
#endif

  sb_t sr, sb[sb_rows];
  char safety_padding[10];
  uint16_t sb_unsent;  // how many rows in-use
     
// OLED declarations *************************

#define max_col  22
#define max_row   8
// 8 rows of 21 characters

struct OLED_line {
  char OLx[max_col];
  };
  
 OLED_line OL[max_row]; 

uint8_t row = 0;
uint8_t col = 0;

// BT support declarations *****************
#if (FC_Mavlink_IO == 1) || (GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 3) // Bluetooth
BluetoothSerial SerialBT;
#endif

// ******************************************
void setup()  {
 
  Debug.begin(115200);
  delay(2500);
  Debug.print("Starting .... ");    
  
  String sketch_path = __FILE__;
  String ino_name = sketch_path.substring(sketch_path.lastIndexOf('/')+1);
  Debug.println(ino_name.substring(0, ino_name.lastIndexOf('.')));

  #if (Target_Board == 3) || (Target_Board == 4) 
    Wire.begin(SDA, SCL);
    display.begin(SSD1306_SWITCHCAPVCC, i2cAddr);  
    display.clearDisplay();
  
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
  

    OledPrintln("Starting .... ");
  #endif
/*
  display.setFont(Dialog_plain_8);     //  col=24 x row 8  on 128x64 display
  display.setFont(Dialog_plain_16);    //  col=13 x row=4  on 128x64 display
*/

  Debug.print("Target Board is ");
  #if (Target_Board == 0) // Teensy3x
    Debug.println("Teensy 3.x");
    OledPrintln("Teensy 3.x");
  #elif (Target_Board == 1) // Blue Pill
    Debug.println("Blue Pill STM32F103C");
    OledPrintln("Blue Pill STM32F103C");
  #elif (Target_Board == 2) //  Maple Mini
    Debug.println("Maple Mini STM32F103C");
    OledPrintln("Maple Mini STM32F103C");
  #elif (Target_Board == 3) //  ESP32 Board
    Debug.print("ESP32 / Variant is ");
    OledPrintln("ESP32 / Variant is");
    #if (ESP32_Variant == 1)
      Debug.println("Dev Module");
      OledPrintln("Dev Module");
    #endif
    #if (ESP32_Variant == 2)
      Debug.println("Wemos® LOLIN ESP32-WROOM-32");
      OledPrintln("Wemos® LOLIN");
    #endif
    #if (ESP32_Variant == 3)
      Debug.println("Dragonlink V3 slim with internal ESP32");
      OledPrintln("Dragonlink V3 ESP32");
    #endif
    #if (ESP32_Variant == 4)
      Debug.println("Heltec Wifi Kit 32");
      OledPrintln("Heltec Wifi Kit 32");
    #endif
    
  #elif (Target_Board == 4) //  ESP8266
    Debug.println("ESP8266 / Variant is ");
    OledPrintln("ESP8266 / Variant is");  
    #if (ESP8266_Variant == 1)
      Debug.println("Lonlin Node MCU 12F");
      OledPrintln("Node MCU 12");
    #endif  
  #endif

  #ifdef Ground_Mode
    Debug.println("Ground Mode");
    OledPrintln("Ground Mode");
  #endif
  #ifdef Air_Mode
    Debug.println("Air Mode");
    OledPrintln("Air Mode");
  #endif
  #ifdef Relay_Mode
    Debug.println("Relay Mode");
    OledPrintln("Relay Mode");
  #endif

  #if (Battery_mAh_Source == 1)  
    Debug.println("Battery_mAh_Source = 1 - Get battery capacities from the FC");
    OledPrintln("mAh from FC");
  #elif (Battery_mAh_Source == 2)
    Debug.println("Battery_mAh_Source = 2 - Define battery capacities in this firmware");  
    OledPrintln("mAh defined in fw");
  #elif (Battery_mAh_Source == 3)
    Debug.println("Battery_mAh_Source = 3 - Define battery capacities in the LUA script");
    OledPrintln("Define mAh in LUA");     
  #else
    #error You must define at least one Battery_mAh_Source. Please correct.
  #endif            

  #if (SPort_Serial == 1) 
    Debug.println("Using Serial_1 for S.Port");     
    OledPrintln("S.PORT is Serial1");  
  #else
    Debug.println("Using Serial_3 for S.Port");
    OledPrintln("S.PORT is Serial3");       
  #endif  
  
  #ifndef RSSI_Override
    Debug.println("RSSI Automatic Select");
    OledPrintln("RSSI Auto Select");     
  #else
    Debug.println("RSSI Override for testing = 70%");
    OledPrintln("RSSI Override = 70%");              // for debugging          
  #endif

  #if (FC_Mavlink_IO == 0)  // Serial
    Debug.println("Mavlink Serial In");
    OledPrintln("Mavlink Serial In");
  #endif  

  #if (GCS_Mavlink_IO == 0)  // Serial
    Debug.println("Mavlink Serial Out");
    OledPrintln("Mavlink Serial Out");
  #endif  

  #if (FC_Mavlink_IO == 1)  // Bluetooth
    Debug.println("Mavlink Bluetooth In");
    OledPrintln("Mavlink BT In");
  #endif  

  #if (GCS_Mavlink_IO == 1)  || (GCS_Mavlink_IO == 3) // Bluetooth
    Debug.println("Mavlink Bluetooth Out");
    OledPrintln("Mavlink BT Out");
  #endif  

  #if (FC_Mavlink_IO == 2)  // WiFi
    Debug.println("Mavlink WiFi In");
    OledPrintln("Mavlink WiFi In");
  #endif  

 #if (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)  // WiFi
    Debug.print("Mavlink WiFi Out - ");
    OledPrintln("Mavlink WiFi Out");
 #endif

 #if ((FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)) 
   #if (WiFi_Protocol == 1)
     Debug.println("Protocol is TCP/IP");
     OledPrintln("Protocol is TCP/IP");
   #elif  (WiFi_Protocol == 2)
     Debug.println("Protocol is UDP");
     OledPrintln("Protocol is UDP");
   #endif
 #endif
 
 #if (FC_Mavlink_IO == 3)  // SD / TF
    Debug.println("Mavlink SD In");
    OledPrintln("Mavlink SD In");
 #endif  

 #if defined GCS_Mavlink_SD
    Debug.println("Mavlink SD Out");
    OledPrintln("Mavlink SD Out");
 #endif

 Debug.println("Waiting for telemetry"); 
 OledPrintln("Waiting for telem");

// **************************** Setup Bluetooth *******************************  
  #if (FC_Mavlink_IO == 1) || (GCS_Mavlink_IO == 1)  || (GCS_Mavlink_IO == 3) // Bluetooth
   
  #ifdef BT_Master_Mode
    SerialBT.begin("Mav2PT", true);            
  #else
      SerialBT.begin("Mav2PT");   
  #endif 
      
    bool bt_connected;

    bt_connected = SerialBT.connect(BT_Slave_Name);
    if(bt_connected) {
        Debug.println("Bluetooth connected!");
        OledPrintln("Bluetooth connected!");
    }    
      
  #endif 
  
  // ************************* Setup WiFi ****************************  
  #if ((FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)) //  WiFi

    pinMode(startWiFiPin, INPUT_PULLUP);

  #endif 

 // ************************* Setup SD Card ************************** 
  #if ((FC_Mavlink_IO == 3) || defined GCS_Mavlink_SD)  // SD Card

    void listDir(fs::FS &fs, const char *, uint8_t);  // Fwd declare
    
    if(!SD.begin()){   
        Debug.println("No SD card reader found. Ignoring SD!"); 
        OledPrintln("No SD reader");
        OledPrintln("Ignoring!");
        sdStatus = 0; // 0=no reader, 1=reader found, 2=SD found, 3=open for append 
                      // 4=open for read, 5=eof detected, 9=failed
    } else {
      Debug.println("SD card reader mount OK");
      OledPrintln("SD drv mount OK");
      uint8_t cardType = SD.cardType();
      sdStatus = 1;
      if(cardType == CARD_NONE){
          Serial.println("No SD card found");
          OledPrintln("No SD card");
          OledPrintln("Ignoring!");      
      } else {
        Debug.println("SD card found");
        OledPrintln("SD card found");
        sdStatus = 2;

        Debug.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
        Debug.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

        listDir(SD, "/", 2);

        #if (FC_Mavlink_IO == 3) //  FC side SD in only
        
          std::string S = "";  
          char c = 0x00;
           Debug.println("Enter the number of the SD file to read, and press Send");
           while (c != 0xA) { // line feed
            if (Debug.available())  {
              c = Debug.read();
              S+=c;
             }
             delay(50);
           }
           
           int i;
           // object from the class stringstream 
           std::istringstream myInt(S); 
           myInt >> i;
           Debug.print(i); Debug.print(" ");
      /*
           for (int j= 0 ; fnCnt > j ; j++)  {
      //     cout << i << fnPath[j] << "\n";
             Debug.print(j); Debug.print(" "); Debug.println(fnPath[j].c_str());
           }
      */     

          sprintf(cPath, "%s", fnPath[i].c_str());  // Select the path
          Debug.print(cPath); Debug.println(" selected "); 
          Debug.println("Reading SD card");
          OledPrintln("Reading SD card");
          file = SD.open(cPath);
          if(!file){
            Debug.printf("Can't open file: %s\n", cPath);
            Debug.println(" for reading");
            sdStatus = 9;  // error
          } else {
            sdStatus = 4;
          }

          
      #endif
               
        // The SD is initialised/opened for write in Main Loop after timeGood
        // because the path/file name includes the date-time

     }  
   }
  #endif
  
// ************************ Setup Serial ******************************
  #if(Target_Board != 4)  
    FrSkySPort_Init();
  #endif

  #if (FC_Mavlink_IO == 0)    //  Serial
  
    #if defined AutoBaud
      mvBaudFC_var = GetBaud(FC_Mav_rxPin);
    #endif  
    
    #if (Target_Board == 3)
      mvSerialFC.begin(mvBaudFC_var, SERIAL_8N1, FC_Mav_rxPin, FC_Mav_txPin);   //  rx,tx, cts, rts
    #else
      mvSerialFC.begin(mvBaudFC_var);    
    #endif
    
  #endif
  
  #if (GCS_Mavlink_IO == 0)   //  Serial
    mvSerialGCS.begin(mvBaudGCS);
  #endif 

// *********************************************************************
  mavGood = false;
  homGood = false;     
  hb_count = 0;
  hb_millis=millis();
  sport_millis = millis();
  fchb_millis=millis();
  acc_millis=millis();
  rds_millis=millis();
  em_millis=millis();
  health_millis = millis();
  
  pinMode(MavStatusLed, OUTPUT); 
  if (BufStatusLed != 99) {
    pinMode(BufStatusLed, OUTPUT); 
  } 
   
}
// *********************************************************************
void loop() {            // For WiFi only
  
#if (FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)

  SenseWiFiPin();

  #if (WiFi_Protocol == 1)  // TCP  
    if (wifiSuGood) {
      wifi = server.available();              // listen for incoming clients 
      if(wifi) {
        Debug.println("New client connected"); 
        OledPrintln("New client ok!");      
        while (wifi.connected()) {            // loop while the client's connected
          main_loop(); 
        }
      wifi.stop();
      Debug.println("Client disconnected");
      OledPrintln("Client discnnct!");      
      } else {
         main_loop();
     } 
    }  else { 
       main_loop();
    }  
  #endif  
  
  #if (WiFi_Protocol == 2)  // UDP  
    main_loop();       
  #endif  
     
#else 
  main_loop();
#endif
  
}
// *******************************************************************************************
//********************************************************************************************
void main_loop() {
  
  SenseWiFiPin();
  
  if (!Read_FC_To_RingBuffer()) {  //  check for SD eof
    if (sdStatus == 5) {
      Debug.println("End of SD file");
      OledPrintln("End of SD file");  
      sdStatus = 0;  // closed after reading   
    }
  }
  
  bool rssiOverride = false;;
  #ifdef RSSI_Override
    rssiOverride = true;
  #endif

  if (((rssiGood) || ((rssiOverride) && mavGood)) && (millis() - rssi_millis > 800)) {
    #if defined Ground_Mode || defined Relay_Mode      // In Air_Mode the FrSky receiver provides rssi
      PackSensorTable(0xF101, 0);   // 0xF101 RSSI 
      rssi_millis = millis(); 
    #endif 
  }
 
  if (millis() - sport_millis > 1) {   // main timing loop for S.Port
    RB_To_Decode_To_SPort_and_GCS();
  }

  
  Read_From_GCS();  

  if (GCS_available) {
    Decode_GCS_To_FC();
    Write_To_FC(G2Fmsg.msgid);  
    GCS_available = false;                      
   }
  
  if(mavGood && (millis() - hb_millis) > 6000)  {   // if no heartbeat from APM in 6s then assume mav not connected
    mavGood=false;
    Debug.println("Heartbeat timed out! Mavlink not connected"); 
    OledPrintln("Mavlink lost!");       
    hb_count = 0;
   } 
   
  #ifdef Data_Streams_Enabled 
  if(mavGood) {                      // If we have a link, request data streams from MavLink every 5s
    if(millis()-rds_millis > 5000) {
    rds_millis=millis();
    Debug.println("Requesting data streams"); 
    OledPrintln("Reqstg datastreams");    
    RequestDataStreams();   // must have Teensy Tx connected to Taranis/FC rx  (When SRx not enumerated)
    }
  }
  #endif 

  if(millis()- fchb_millis > 2000) {  // Mav2PT heartbeat to FC every 2 seconds
    fchb_millis=millis();
    #if defined Mav_Debug_Mav2PT_Heartbeat
      Debug.println("Sending mav2pt hb to FC");  
    #endif    
    Send_FC_Heartbeat();   // must have Mav2PT tx pin connected to Telem radio rx pin  
  }

  
  #if defined Request_Missions_From_FC || defined Request_Mission_Count_From_FC
  if (mavGood) {
    if (!ap_ms_list_req) {
      RequestMissionList();  //  #43
      ap_ms_list_req = true;
    }
  }
  #endif

  #if (Battery_mAh_Source == 1)  // Get battery capacity from the FC
  // Request battery capacity params 
  if (mavGood) {
    if (!ap_bat_paramsReq) {
      Param_Request_Read(356);    // Request Bat1 capacity   do this twice in case of lost frame
      Param_Request_Read(356);    
      Param_Request_Read(364);    // Request Bat2 capacity
      Param_Request_Read(364);    
      Debug.println("Battery capacities requested");
      OledPrintln("Bat mAh from FC");    
      ap_bat_paramsReq = true;
    } else {
      if (ap_bat_paramsRead &&  (!parm_msg_shown)) {
        parm_msg_shown = true; 
        Debug.println("Battery params successfully read"); 
        OledPrintln("Bat params read ok"); 
      }
    } 
  }
  #endif 
  
  #ifdef Mav_List_Params
    if(mavGood && (!ap_paramsList)) {
      Request_Param_List();
      ap_paramsList = true;
    }
  #endif 

  #if defined GCS_Mavlink_SD
    void OpenSDForWrite();  // fwd define
    if ((timeGood) && (sdStatus == 2)) OpenSDForWrite();
  #endif
  
  ServiceStatusLeds();
  
}
// *******************************************************************************
//********************************************************************************

bool Read_FC_To_RingBuffer() {

 
  #if (FC_Mavlink_IO == 0) // Serial
  mavlink_status_t status;
  while(mvSerialFC.available()) { 
    uint16_t c = mvSerialFC.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &F2Rmsg, &status)) {  // Read a frame
       #ifdef  Debug_FC_Down
         Debug.println("Serial passed to RB from FC side :");
         PrintMavBuffer(&F2Rmsg);
      #endif              
      MavToRingBuffer();       
    }
  }
  return true;  // moot 
  #endif 

  #if (FC_Mavlink_IO == 1) // Bluetooth

     bool msgRcvdBT = Read_Bluetooth(&F2Rmsg);

     if (msgRcvdBT) {

        MavToRingBuffer();      

        #ifdef  Debug_FC_Down   
          Debug.print("BT passed to RB from FC side: msgRcvdBT=" ); Debug.println(msgRcvdBT);
          PrintMavBuffer(&F2Rmsg);
        #endif      
      }
  return true;  
  #endif    

  #if (FC_Mavlink_IO == 2)  //  WiFi
    
    #if (WiFi_Protocol == 1) // TCP from FC
    
      bool msgRcvdWF = Read_TCP(&F2Rmsg);

      if (msgRcvdWF) {
        
        MavToRingBuffer();  
          
        #ifdef  Debug_FC_Down    
          Debug.print("Passed down from FC WiFi TCP to F2Rmsg: msgRcvdWF=" ); Debug.println(msgRcvdWF);
          PrintMavBuffer(&G2Fmsg);
        #endif      
      }
    return true;  
    #endif
      
    #if (WiFi_Protocol == 2) // UDP from FC
    
      bool msgRcvdWF = Read_UDP(&F2Rmsg);

      if (msgRcvdWF) {
        
        MavToRingBuffer();   
         
        #ifdef  Debug_FC_Down   
          Debug.print("Passed down from FC WiFi UDP to F2Rmsg: msgRcvdWF=" ); Debug.println(msgRcvdWF);
          PrintMavBuffer(&F2Rmsg);
        #endif      
      }
    return true;     
    #endif 
    
  #endif 
  
  #if (FC_Mavlink_IO == 3)  //  SD
    mavlink_status_t status;
    if (sdStatus == 4) {      //  if open for read
      while (file.available()) {
        uint8_t c = file.read();
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &F2Rmsg, &status)) {  // Parse a frame
          #ifdef  Debug_FC_Down
            Debug.println("SD passed to RB from FC side :");
            PrintMavBuffer(&F2Rmsg);
          #endif              
          MavToRingBuffer(); 
          delay(sdReadDelay);
          return true;    // all good 
        }
      } 
      file.close();
      sdStatus = 5;  // closed after reading
      return false;  // eof
    }     
  #endif 
}
//********************************************************************************

void RB_To_Decode_To_SPort_and_GCS() {

  if (!MavRingBuff.isEmpty()) {
    R2Gmsg = (MavRingBuff.shift());  // Get a mavlink message from front of queue
    #if defined Mav_Debug_RingBuff
  //   Debug.print("Mavlink ring buffer R2Gmsg: ");  
  //    PrintMavBuffer(&R2Gmsg);
      Debug.print("Ring queue = "); Debug.println(MavRingBuff.size());
    #endif
    
    Send_From_RingBuf_To_GCS();
    
    DecodeOneMavFrame();  // Decode a Mavlink frame from the ring buffer 

  }
                              //*** Decoded Mavlink to S.Port  ****
  #ifdef Ground_Mode
  if(mavGood  && ((millis() - em_millis) > 10)) {   
     Emulate_ReadSPort();                // Emulate the sensor IDs received from XRS receiver on SPort
     em_millis=millis();
    }
  #endif
     
  #if defined Air_Mode || defined Relay_Mode
  if(mavGood && ((millis() - sp_millis) > 2)) {   // zero does not work for Teensy 3.2, down to zero for Blue Pill
     ReadSPort();                       // Receive sensor IDs from XRS receiver, slot in ours, and send 
     sp_millis=millis();
    }
  #endif   
}  
//********************************************************************************
void Read_From_GCS() {
  
 #if (GCS_Mavlink_IO == 0) // Serial 
  mavlink_status_t status;
  while(mvSerialGCS.available()) { 
    uint8_t c = mvSerialGCS.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &G2Fmsg, &status)) {  // Read a frame from GCS  
    GCS_available = true;  // Record waiting
    #ifdef  Debug_GCS_Up
      Debug.println("Passed up from GCS Serial to G2Fmsg:");
      PrintMavBuffer(&G2Fmsg);
    #endif     
    }
  } 
 #endif 

 #if (GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 3) // Bluetooth
 
     bool msgRcvdBT = Read_Bluetooth(&G2Fmsg);

     if (msgRcvdBT) {
        GCS_available = true;  // Record waiting to go to FC 
        #ifdef  Debug_GCS_Up    
          Debug.print("Passed up from GCS BT to G2Fmsg: msgRcvdBT=" ); Debug.println(msgRcvdBT);
          PrintMavBuffer(&G2Fmsg);
        #endif      
      }

 #endif    

 #if (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)  //  WiFi
    
    #if (WiFi_Protocol == 1) // TCP 
    
      bool msgRcvdWF = Read_TCP(&G2Fmsg);

      if (msgRcvdWF) {
        GCS_available = true;  // Record waiting to go to FC 

        #ifdef  Debug_GCS_Up    
          Debug.print("Passed up from GCS WiFi TCP to G2Fmsg: msgRcvdWF=" ); Debug.println(msgRcvdWF);
          PrintMavBuffer(&G2Fmsg);
        #endif      
      }

    #endif
      
    #if (WiFi_Protocol == 2) // UDP from GCS
      bool msgRcvdWF = Read_UDP(&G2Fmsg);

      if (msgRcvdWF) {
        GCS_available = true;  // Record waiting to go to FC 

        #ifdef  Debug_GCS_Up    
          Debug.print("Passed up from GCS WiFi UDP to G2Fmsg: msgRcvdWF=" ); Debug.println(msgRcvdWF);
          PrintMavBuffer(&G2Fmsg);
        #endif      
      }
       
    #endif 
    
 #endif 
}
//********************************************************************************
#if ((FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)) && (WiFi_Protocol == 1)  //  WiFi  TCP

bool Read_TCP(mavlink_message_t* msgidx)  {
    if (!wifiSuGood) return false;  
    bool msgRcvd = false;
    mavlink_status_t _status;
    
    len = wifi.available();
    uint16_t tcp_count = len;
    if(tcp_count > 0) {

        while(tcp_count--)  {
            int result = wifi.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgidx, &_status);
                if(msgRcvd) {

                    if(!hb_heard_from) {
                        if(msgidx->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from     = true;
                            hb_system_id      = msgidx->sysid;
                            hb_comp_id        = msgidx->compid;
                            hb_seq_expected   = msgidx->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgidx->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkLinkErrors(msgidx);
                    }
                 
                    break;
                }
            }
        }
    }
    
    return msgRcvd;
}
#endif
//********************************************************************************

#if ((FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)) && (WiFi_Protocol == 2) //  WiFi  UDP

bool Read_UDP(mavlink_message_t* msgidx)  {
    if (!wifiSuGood) return false;  
    bool msgRcvd = false;
    mavlink_status_t _status;
    
    len = udp.parsePacket();
    int udp_count = len;
    if(udp_count > 0) {

        while(udp_count--)  {

            int result = udp.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgidx, &_status);
                if(msgRcvd) {
                  
                    udp_remoteIP = udp.remoteIP();  // remember which remote client sent this packet so we can target it
                    DisplayRemoteIP();
                    if(!hb_heard_from) {
                        if(msgidx->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from      = true;
                            hb_system_id       = msgidx->sysid;
                            hb_comp_id         = msgidx->compid;
                            hb_seq_expected   = msgidx->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgidx->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkLinkErrors(msgidx);
                    }
                    
                    break;
                }
            }
        }
    }
    
    return msgRcvd;
}
#endif
//********************************************************************************
#if ((FC_Mavlink_IO == 1) || (GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 3))

bool Read_Bluetooth(mavlink_message_t* msgidx)  {
    
    bool msgRcvd = false;
    mavlink_status_t _status;
    
    len = SerialBT.available();
    uint16_t bt_count = len;
    if(bt_count > 0) {

        while(bt_count--)  {
            int result = SerialBT.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgidx, &_status);
                if(msgRcvd) {

                    if(!hb_heard_from) {
                        if(msgidx->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from     = true;
                            hb_system_id      = msgidx->sysid;
                            hb_comp_id        = msgidx->compid;
                            hb_seq_expected   = msgidx->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgidx->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkLinkErrors(msgidx);
                    }
                 
                    break;
                }
            }
        }
    }
    
    return msgRcvd;
}
#endif
//********************************************************************************
#if ((FC_Mavlink_IO == 1) || (GCS_Mavlink_IO == 1)) || ((FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)) // BT or WiFi
void checkLinkErrors(mavlink_message_t* msgidx)   {

    //-- Don't bother if we have not heard from the link (and it's the proper sys/comp ids)
    if(!hb_heard_from || msgidx->sysid != hb_system_id || msgidx->compid != hb_comp_id) {
        return;
    }
    uint16_t seq_received = (uint16_t)msgidx->seq;
    uint16_t packet_lost_count = 0;
    //-- Account for overflow during packet loss
    if(seq_received < hb_seq_expected) {
        packet_lost_count = (seq_received + 255) - hb_seq_expected;
    } else {
        packet_lost_count = seq_received - hb_seq_expected;
    }
    hb_seq_expected = msgidx->seq + 1;
    link_status.packets_lost += packet_lost_count;
}
#endif
//********************************************************************************

void Decode_GCS_To_FC() {
  #if (GCS_Mavlink_IO == 0) || (GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 2)  || (GCS_Mavlink_IO == 3) // if any GCS I/O requested
   #if defined Mav_Print_All_Msgid
     Debug.printf("GCS to FC - msgid = %3d \n",  G2Fmsg.msgid);
   #endif
   switch(G2Fmsg.msgid) {
       case MAVLINK_MSG_ID_HEARTBEAT:    // #0   
          #if defined Mav_Debug_All || defined Debug_GCS_Upxxxx || defined Mav_Debug_GCS_Heartbeat
            gcs_type = mavlink_msg_heartbeat_get_type(&G2Fmsg); 
            gcs_autopilot = mavlink_msg_heartbeat_get_autopilot(&G2Fmsg);
            gcs_base_mode = mavlink_msg_heartbeat_get_base_mode(&G2Fmsg);
            gcs_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&G2Fmsg);
            gcs_system_status = mavlink_msg_heartbeat_get_system_status(&G2Fmsg);
            gcs_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&G2Fmsg);
  

            Debug.print("Mavlink to FC: #0 Heartbeat: ");           
            Debug.print("gcs_type="); Debug.print(ap_type);   
            Debug.print("  gcs_autopilot="); Debug.print(ap_autopilot); 
            Debug.print("  gcs_base_mode="); Debug.print(ap_base_mode); 
            Debug.print(" gcs_custom_mode="); Debug.print(ap_custom_mode);
            Debug.print("  gcs_system_status="); Debug.print(ap_system_status); 
            Debug.print("  gcs_mavlink_version="); Debug.print(ap_mavlink_version);      
            Debug.println();
          #endif   
          break;
          
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:  // #20  from GCS
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Debug_Param_Request_Read 
            gcs_target_system = mavlink_msg_param_request_read_get_target_system(&G2Fmsg);        
            mavlink_msg_param_request_read_get_param_id(&G2Fmsg, gcs_req_param_id);
            gcs_req_param_index = mavlink_msg_param_request_read_get_param_index(&G2Fmsg);                  

            Debug.print("Mavlink to FC: #20 Param_Request_Read: ");           
            Debug.print("gcs_target_system="); Debug.print(gcs_target_system);   
            Debug.print("  gcs_req_param_id="); Debug.print(gcs_req_param_id);          
            Debug.print("  gcs_req_param_index="); Debug.print(gcs_req_param_index);    
            Debug.println();    

            Param_Request_Read(gcs_req_param_index); 
            
          #endif
          break;
          
         case MAVLINK_MSG_ID_MISSION_REQUEST_INT:  // #51 
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Mav_Debug_Mission
            gcs_target_system = mavlink_msg_mission_request_int_get_target_system(&G2Fmsg);
            gcs_target_component = mavlink_msg_mission_request_int_get_target_component(&G2Fmsg);
            gcs_seq = mavlink_msg_mission_request_int_get_seq(&G2Fmsg); 
            gcs_mission_type = mavlink_msg_mission_request_int_get_seq(&G2Fmsg);                     

            Debug.print("Mavlink to FC: #51 Mission_Request_Int: ");           
            Debug.print("gcs_target_system="); Debug.print(gcs_target_system);   
            Debug.print("  gcs_target_component="); Debug.print(gcs_target_component);          
            Debug.print("  gcs_seq="); Debug.print(gcs_seq);    
            Debug.print("  gcs_mission_type="); Debug.print(gcs_mission_type);    // Mav2
            Debug.println();
          #endif
          break;        
        default:
          if (!mavGood) break;
          #if defined Debug_All || defined Debug_GCS_Up || defined Debug_GCS_Unknown
            Debug.print("Mavlink to FC: ");
            Debug.print("Unknown Message ID #");
            Debug.print(G2Fmsg.msgid);
            Debug.println(" Ignored"); 
          #endif

          break;
   }
   #endif
}
//********************************************************************************
void Write_To_FC(uint32_t msg_id) {
    
      #if (FC_Mavlink_IO == 0)  // Serial to FC
        len = mavlink_msg_to_send_buffer(FCbuf, &G2Fmsg);
        mvSerialFC.write(FCbuf,len);  
         
        #if defined  Debug_FC_Up || defined Debug_GCS_Up
        if (msg_id) {    //  dont print heartbeat - too much info
          Debug.println("Written to FC Serial from G2Fmsg:");
          PrintMavBuffer(&G2Fmsg);
        }  
        #endif
      
      #endif
  
      #if (FC_Mavlink_IO == 1)  // Bluetooth to FC

        bool msgSent = Send_Bluetooth(&G2Fmsg);  // to FC
          
          #ifdef  Debug_FC_Up
            Debug.print("Sent to FC Bluetooth from G2Fmsg: msgSent="); Debug.println(msgSent);
            PrintMavBuffer(&R2Gmsg);
          #endif
        
      #endif

      #if (FC_Mavlink_IO == 2)  // WiFi to FC

       if (wifiSuGood) {
        
        #if (WiFi_Protocol == 1) // TCP  
        
          bool msgSent = Send_TCP(&G2Fmsg);  // to FC
          
          #ifdef  Debug_GCS_Up
            Debug.print("Sent to FC WiFi TCP from G2Fmsg: msgSent="); Debug.println(msgSent);
            PrintMavBuffer(&R2Gmsg);
          #endif
          
        #endif
        
        #if (WiFi_Protocol == 2) // UDP 

          bool msgRead = Send_UDP(&G2Fmsg);  // to FC
          
          #ifdef  Debug_GCS_Up
            Debug.print("Sent to FC WiFi UDP from G2Fmsg: magRead="); Debug.println(msgRead);
            PrintMavBuffer(&G2Fmsg);
          #endif      
                  
        #endif                                                             
        }
      #endif       
}  
//********************************************************************************

void MavToRingBuffer() {

      // MAIN Queue
      if (MavRingBuff.isFull()) {
        BufLedState = HIGH;
        Debug.println("MavRingBuff full. Dropping records!");
     //   OledPrintln("Mav buffer full!"); 
      }
       else {
        BufLedState = LOW;
        MavRingBuff.push(F2Rmsg);
        #if defined Mav_Debug_RingBuff
          Debug.print("Ring queue = "); 
          Debug.println(MavRingBuff.size());
        #endif
      }
  }
  
//********************************************************************************

void Send_From_RingBuf_To_GCS() {   // Down to GCS (or other) from Ring Buffer
#if (GCS_Mavlink_IO == 0) || (GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 2)  || (GCS_Mavlink_IO == 3) || defined GCS_Mavlink_SD
    
    #if (GCS_Mavlink_IO == 0)  // Serial
      len = mavlink_msg_to_send_buffer(GCSbuf, &R2Gmsg);
      #ifdef  Debug_GCS_Down
        Debug.println("Passed down from Ring buffer to GCS by Serial:");
        PrintMavBuffer(&R2Gmsg);
      #endif
       mvSerialGCS.write(GCSbuf,len);  
    #endif

    #if (GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 3)  // Bluetooth
      len = mavlink_msg_to_send_buffer(GCSbuf, &R2Gmsg);     
      #ifdef  Debug_GCS_Down
        Debug.println("Passed down from Ring buffer to GCS by Bluetooth:");
        PrintMavBuffer(&R2Gmsg);
      #endif
      if (SerialBT.hasClient()) {
        SerialBT.write(GCSbuf,len);
      }
    #endif

    #if  (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3) //  WiFi
 
      if (wifiSuGood) {
        
        #if (WiFi_Protocol == 1) // TCP  
        
          bool sentOK = Send_TCP(&R2Gmsg);  // to GCS
          
          #ifdef  Debug_GCS_Down
            Debug.print("Passed down from Ring buffer to GCS by WiFi TCP: sentOk="); Debug.println(sentOk);
            PrintMavBuffer(&R2Gmsg);
          #endif
          
        #endif
        
        #if (WiFi_Protocol == 2) // UDP 

          bool msgSent = Send_UDP(&R2Gmsg);  // to GCS
          
          #ifdef  Debug_GCS_Down
            Debug.print("Passed down from Ring buffer to GCS by WiFi UDP: msgSent="); Debug.println(msgSent);
            PrintMavBuffer(&R2Gmsg);
          #endif      
                  
        #endif                                                             
              
      }
    #endif
    
    #if  defined GCS_Mavlink_SD //  SD Card
      if (sdStatus == 3) {  //  if open for write
        File file = SD.open(cPath, FILE_APPEND);
        if(!file){
           Debug.println("Failed to open file for appending");
           sdStatus = 9;
           return;
          }

       memcpy(GCSbuf, (void*)&ap_time_unix_usec, sizeof(uint64_t));
       len=mavlink_msg_to_send_buffer(GCSbuf+sizeof(uint64_t), &R2Gmsg);

       if(file.write(GCSbuf, len+18)){   // 8 bytes plus some head room   
          } else {
          Debug.println("Append failed");
         }
         
        file.close();
        
        #ifdef  Debug_SD
          Debug.println("Passed down from Ring buffer to SD:");
          PrintMavBuffer(&R2Gmsg);
        #endif        
      }  
    #endif    
   
 #endif 
}


//********************************************************************************
#if ((FC_Mavlink_IO == 1) || (GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 3)) 

bool Send_Bluetooth(mavlink_message_t* msgidx) {

    bool msgSent = false;
    uint8_t buf[300];
     
    uint16_t len = mavlink_msg_to_send_buffer(buf, msgidx);
  
    size_t sent = SerialBT.write(buf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    return msgSent;
}
#endif
//**********************************************************************************
#if ((FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)) && (WiFi_Protocol == 1) //  WiFi  TCP/IP

bool Send_TCP(mavlink_message_t* msgidx) {
    if (!wifiSuGood) return false;  
    bool msgSent = false;
    uint8_t buf[300];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msgidx);
  
    size_t sent =  wifi.write(buf,len);  

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    return msgSent;
}
#endif
//********************************************************************************
#if ((FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)) && (WiFi_Protocol == 2) //  WiFi  UDP

bool Send_UDP(mavlink_message_t* msgidx) {
    if (!wifiSuGood) return false;  
    bool msgSent = false;
    uint8_t buf[300];

    udp.beginPacket(udp_remoteIP, udp_remotePort);

    uint16_t len = mavlink_msg_to_send_buffer(buf, msgidx);
  
    size_t sent = udp.write(buf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    udp.endPacket();
    return msgSent;
}
#endif

//********************************************************************************

uint32_t bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth); // Forward define

void DecodeOneMavFrame() {
  
   #if defined Mav_Print_All_Msgid
     uint16_t sz = sizeof(R2Gmsg);
     Debug.printf("FC to QGS - msgid = %3d Msg size =%3d\n",  R2Gmsg.msgid, sz);
   #endif

   switch(R2Gmsg.msgid) {
    
        case MAVLINK_MSG_ID_HEARTBEAT:    // #0   http://mavlink.org/messages/common
          ap_type_tmp = mavlink_msg_heartbeat_get_type(&R2Gmsg);   // Alex - don't contaminate the ap-type variable
          if (ap_type_tmp == 5 || ap_type_tmp == 6 || ap_type_tmp == 27) break;      
          // Ignore heartbeats from GCS (6) or Ant Trackers(5) or ADSB (27))
          ap_type = ap_type_tmp;
          ap_autopilot = mavlink_msg_heartbeat_get_autopilot(&R2Gmsg);
          ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&R2Gmsg);
          ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&R2Gmsg);
          
          px4_main_mode = bit32Extract(ap_custom_mode,16, 8);
          px4_sub_mode = bit32Extract(ap_custom_mode,24, 8);
          px4_flight_stack = (ap_autopilot == MAV_AUTOPILOT_PX4);
            
          ap_system_status = mavlink_msg_heartbeat_get_system_status(&R2Gmsg);
          ap_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&R2Gmsg);
          hb_millis=millis(); 

          if ((ap_base_mode >> 7) && (!homGood)) 
            MarkHome();  // If motors armed for the first time, then mark this spot as home

          hb_count++; 
   
          if(!mavGood) {
            Debug.print("hb_count=");
            Debug.print(hb_count);
            Debug.println("");

            if(hb_count >= 3) {        // If  3 heartbeats from MavLink then we are connected
              mavGood=true;
              Debug.println("mavgood=true");
               OledPrintln("Mavlink good !");      
              }
            }

          PackSensorTable(0x5001, 0);
          if ((!(hb_count % 10)) || (!mavGood)) {
        //    Debug.print("hb_count % 50="); Debug.println((hb_count % 50));
            PackSensorTable(0x5007, 0);
            }

          #if defined Mav_Debug_All || defined Mav_Debug_FC_Heartbeat
            Debug.print("Mavlink from FC #0 Heartbeat: ");           
            Debug.print("ap_type="); Debug.print(ap_type);   
            Debug.print("  ap_autopilot="); Debug.print(ap_autopilot); 
            Debug.print("  ap_base_mode="); Debug.print(ap_base_mode); 
            Debug.print(" ap_custom_mode="); Debug.print(ap_custom_mode);
            Debug.print("  ap_system_status="); Debug.print(ap_system_status); 
            Debug.print("  ap_mavlink_version="); Debug.print(ap_mavlink_version);   

            if (px4_flight_stack) {         
              Debug.print(" px4_main_mode="); Debug.print(px4_main_mode); 
              Debug.print(" px4_sub_mode="); Debug.print(px4_sub_mode);  
              Debug.print(" ");Debug.print(PX4FlightModeName(px4_main_mode, px4_sub_mode));  
           }
            
            Debug.println();
          #endif

          
          break;
        case MAVLINK_MSG_ID_SYS_STATUS:   // #1
          if (!mavGood) break;

          ap_onboard_control_sensors_health = mavlink_msg_sys_status_get_onboard_control_sensors_health(&R2Gmsg);
          ap_voltage_battery1= Get_Volt_Average1(mavlink_msg_sys_status_get_voltage_battery(&R2Gmsg));        // 1000 = 1V  i.e mV
          ap_current_battery1= Get_Current_Average1(mavlink_msg_sys_status_get_current_battery(&R2Gmsg));     //  100 = 1A, i.e dA
          if(ap_voltage_battery1> 21000) ap_ccell_count1= 6;
            else if (ap_voltage_battery1> 16800 && ap_ccell_count1!= 6) ap_ccell_count1= 5;
            else if(ap_voltage_battery1> 12600 && ap_ccell_count1!= 5) ap_ccell_count1= 4;
            else if(ap_voltage_battery1> 8400 && ap_ccell_count1!= 4) ap_ccell_count1= 3;
            else if(ap_voltage_battery1> 4200 && ap_ccell_count1!= 3) ap_ccell_count1= 2;
            else ap_ccell_count1= 0;
          
          #if defined Mav_Debug_All || defined Mav_Debug_SysStatus || defined Debug_Batteries
            Debug.print("Mavlink from FC #1 Sys_status: ");     
            Debug.print(" Sensor health=");
            Debug.print(ap_onboard_control_sensors_health);   // 32b bitwise 0: error, 1: healthy.
            Debug.print(" Bat volts=");
            Debug.print((float)ap_voltage_battery1/ 1000, 3);   // now V
            Debug.print("  Bat amps=");
            Debug.print((float)ap_current_battery1/ 100, 1);   // now A
              
            Debug.print("  mAh="); Debug.print(bat1.mAh, 6);    
            Debug.print("  Total mAh="); Debug.print(bat1.tot_mAh, 3);  // Consumed so far, calculated in Average module
         
            Debug.print("  Bat1 cell count= "); 
            Debug.println(ap_ccell_count1);
          #endif

          #if defined Send_Sensor_Health_Messages
          if ((millis() - health_millis) > 5000) {
            health_millis = millis();
            if ( bit32Extract(ap_onboard_control_sensors_health, 5, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  // severity = 2
              strcpy(ap_text, "Bad GPS Health");
              PackMultipleTextChunks_5000(0x5000);
            } else
          
            if ( bit32Extract(ap_onboard_control_sensors_health, 0, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad Gyro Health + 0x00 +0x00");
              PackMultipleTextChunks_5000(0x5000);
            } else 
                 
            if ( bit32Extract(ap_onboard_control_sensors_health, 1, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad Accel Health");
              PackMultipleTextChunks_5000(0x5000);
            } else
          
            if ( bit32Extract(ap_onboard_control_sensors_health, 2, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad Compass Health");
              PackMultipleTextChunks_5000(0x5000);
            } else
            
            if ( bit32Extract(ap_onboard_control_sensors_health, 3, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  // severity = 2
              strcpy(ap_text, "Bad Baro Health");
              PackMultipleTextChunks_5000(0x5000);
            } else
          
            if ( bit32Extract(ap_onboard_control_sensors_health, 8, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad LiDAR Health");
              PackMultipleTextChunks_5000(0x5000);
            } else 
                 
            if ( bit32Extract(ap_onboard_control_sensors_health, 6, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad OptFlow Health + 0x00 +0x00");
              PackMultipleTextChunks_5000(0x5000);
            } else
          
            if ( bit32Extract(ap_onboard_control_sensors_health, 22, 1) ) {
             ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad or No Terrain Data");
              PackMultipleTextChunks_5000(0x5000);
            } else

            if ( bit32Extract(ap_onboard_control_sensors_health, 20, 1) ) {
             ap_severity = MAV_SEVERITY_CRITICAL;  
             strcpy(ap_text, "Geofence Breach");
             PackMultipleTextChunks_5000(0x5000);
           } else  
                 
            if ( bit32Extract(ap_onboard_control_sensors_health, 21, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad AHRS");
              PackMultipleTextChunks_5000(0x5000);
            } else
          
           if ( bit32Extract(ap_onboard_control_sensors_health, 16, 1) ) {
             ap_severity = MAV_SEVERITY_CRITICAL;  
             strcpy(ap_text, "No RC Receiver");
             PackMultipleTextChunks_5000(0x5000);
           } else  

           if ( bit32Extract(ap_onboard_control_sensors_health, 24, 1) ) {
             ap_severity = MAV_SEVERITY_CRITICAL;  
             strcpy(ap_text, "Bad Logging");
             PackMultipleTextChunks_5000(0x5000);
           } 
         }                  
         #endif     
             
         PackSensorTable(0x5003, 0);
         
         break;
        case MAVLINK_MSG_ID_SYSTEM_TIME:   // #2
          if (!mavGood) break;
          ap_time_unix_usec= (mavlink_msg_system_time_get_time_unix_usec(&R2Gmsg));    // us
          ap_time_boot_ms= (mavlink_msg_system_time_get_time_boot_ms(&R2Gmsg));        //  ms
          if ( ap_time_unix_usec != 0 ) {
            timeGood = true;
          }
          #if defined Mav_Debug_All || defined Mav_Debug_System_Time
            Debug.print("Mavlink from FC #2 System_Time: ");        
            Debug.print(" Unix secs="); Debug.print((float)(ap_time_unix_usec/1E6), 6);  
            Debug.print("  Boot secs="); Debug.println((float)(ap_time_boot_ms/1E3), 0);   
          #endif
          break;                   
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:   // #20 - OUTGOING TO UAV
          if (!mavGood) break;
          break;     
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:   // #21 - OUTGOING TO UAV
          if (!mavGood) break;
          break;  
        case MAVLINK_MSG_ID_PARAM_VALUE:          // #22
          if (!mavGood) break;        
          len=mavlink_msg_param_value_get_param_id(&R2Gmsg, ap_param_id);
          ap_param_value=mavlink_msg_param_value_get_param_value(&R2Gmsg);
          ap_param_count=mavlink_msg_param_value_get_param_count(&R2Gmsg);
          ap_param_index=mavlink_msg_param_value_get_param_index(&R2Gmsg); 

          switch(ap_param_index) {      // if #define Battery_mAh_Source !=1 these will never arrive
            case 356:         // Bat1 Capacity
              ap_bat1_capacity = ap_param_value;
              #if defined Mav_Debug_All || defined Debug_Batteries
                Debug.print("Mavlink from FC #22 Param_Value: ");
                Debug.print("bat1 capacity=");
                Debug.println(ap_bat1_capacity);
              #endif
              break;
            case 364:         // Bat2 Capacity
              ap_bat2_capacity = ap_param_value;
              ap_bat_paramsRead = true;
              #if defined Mav_Debug_All || defined Debug_Batteries
                Debug.print("Mavlink from FC #22 Param_Value: ");
                Debug.print("bat2 capacity=");
                Debug.println(ap_bat2_capacity);
              #endif             
              break;
          } 
             
          #if defined Mav_Debug_All || defined Mav_Debug_Params || defined Mav_List_Params
            Debug.print("Mavlink from FC #22 Param_Value: ");
            Debug.print("param_id=");
            Debug.print(ap_param_id);
            Debug.print("  param_value=");
            Debug.print(ap_param_value, 4);
            Debug.print("  param_count=");
            Debug.print(ap_param_count);
            Debug.print("  param_index=");
            Debug.println(ap_param_index);
          #endif       
          break;    
        case MAVLINK_MSG_ID_GPS_RAW_INT:          // #24
          if (!mavGood) break;        
          ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&R2Gmsg);                   // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
          ap_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&R2Gmsg);    // number of visible satellites
          if(ap_fixtype > 2)  {
            ap_lat24 = mavlink_msg_gps_raw_int_get_lat(&R2Gmsg);
            ap_lon24 = mavlink_msg_gps_raw_int_get_lon(&R2Gmsg);
            ap_amsl24 = mavlink_msg_gps_raw_int_get_alt(&R2Gmsg);                    // 1m =1000 
            ap_eph = mavlink_msg_gps_raw_int_get_eph(&R2Gmsg);                       // GPS HDOP 
            ap_epv = mavlink_msg_gps_raw_int_get_epv(&R2Gmsg);                       // GPS VDOP 
            ap_vel = mavlink_msg_gps_raw_int_get_vel(&R2Gmsg);                       // GPS ground speed (m/s * 100)
            ap_cog = mavlink_msg_gps_raw_int_get_cog(&R2Gmsg);                       // Course over ground (NOT heading) in degrees * 100
     // mav2
           ap_alt_ellipsoid = mavlink_msg_gps_raw_int_get_alt_ellipsoid(&R2Gmsg);    // mm    Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
           ap_h_acc = mavlink_msg_gps_raw_int_get_h_acc(&R2Gmsg);                    // mm    Position uncertainty. Positive for up.
           ap_v_acc = mavlink_msg_gps_raw_int_get_v_acc(&R2Gmsg);                    // mm    Altitude uncertainty. Positive for up.
           ap_vel_acc = mavlink_msg_gps_raw_int_get_vel_acc(&R2Gmsg);                // mm    Speed uncertainty. Positive for up.
           ap_hdg_acc = mavlink_msg_gps_raw_int_get_hdg_acc(&R2Gmsg);                // degE5   Heading / track uncertainty       

           cur.lat =  (float)ap_lat24 / 1E7;
           cur.lon = (float)ap_lon24 / 1E7;
           cur.alt = ap_amsl24 / 1E3;
           
          }
          #if defined Mav_Debug_All || defined Mav_Debug_GPS_Raw
            Debug.print("Mavlink from FC #24 GPS_RAW_INT: ");  
            Debug.print("ap_fixtype="); Debug.print(ap_fixtype);
            if (ap_fixtype==0) Debug.print(" No GPS");
              else if (ap_fixtype==1) Debug.print(" No Fix");
              else if (ap_fixtype==2) Debug.print(" 2D Fix");
              else if (ap_fixtype==3) Debug.print(" 3D Fix");
              else if (ap_fixtype==4) Debug.print(" DGPS/SBAS aided");
              else if (ap_fixtype==5) Debug.print(" RTK Float");
              else if (ap_fixtype==6) Debug.print(" RTK Fixed");
              else if (ap_fixtype==7) Debug.print(" Static fixed");
              else if (ap_fixtype==8) Debug.print(" PPP");
              else Debug.print(" Unknown");

            Debug.print("  sats visible="); Debug.print(ap_sat_visible);
            Debug.print("  latitude="); Debug.print((float)(ap_lat24)/1E7, 7);
            Debug.print("  longitude="); Debug.print((float)(ap_lon24)/1E7, 7);
            Debug.print("  gps alt amsl="); Debug.print((float)(ap_amsl24)/1E3, 1);
            Debug.print("  eph (hdop)="); Debug.print((float)ap_eph);                 // HDOP
            Debug.print("  epv (vdop)="); Debug.print((float)ap_epv);
            Debug.print("  vel="); Debug.print((float)ap_vel / 100, 3);           // GPS ground speed (m/s)
            Debug.print("  cog="); Debug.print((float)ap_cog / 100, 1);           // Course over ground in degrees
            //  mav2
            Debug.print("  alt_ellipsoid)="); Debug.print(ap_alt_ellipsoid / 1000, 2);      // alt_ellipsoid in mm
            Debug.print("  h_acc="); Debug.print(ap_h_acc);                       // Position uncertainty in mm. Positive for up.
            Debug.print("  v_acc="); Debug.print(ap_v_acc);                       // Altitude uncertainty in mm. Positive for up.
            Debug.print("  ap_vel_acc="); Debug.print(ap_vel_acc);                // Speed uncertainty. Positive for up.
            Debug.print("  ap_hdg_acc="); Debug.print(ap_hdg_acc);                // degE5   Heading / track uncertainty 
            Debug.println();
          #endif 

           PackSensorTable(0x800, 0);   // 0x800 Lat
           PackSensorTable(0x800, 1);   // 0x800 Lon
           PackSensorTable(0x5002, 0);  // 0x5002 GPS Status
           PackSensorTable(0x5004, 0);  // 0x5004 Home         
              
          break;
        case MAVLINK_MSG_ID_RAW_IMU:   // #27
        #if defined Decode_Non_Essential_Mav
          if (!mavGood) break;        
          ap_accX = mavlink_msg_raw_imu_get_xacc(&R2Gmsg);                 
          ap_accY = mavlink_msg_raw_imu_get_yacc(&R2Gmsg);
          ap_accZ = mavlink_msg_raw_imu_get_zacc(&R2Gmsg);
          #if defined Mav_Debug_All || defined Mav_Debug_Raw_IMU
            Debug.print("Mavlink from FC #27 Raw_IMU: ");
            Debug.print("accX="); Debug.print((float)ap_accX / 1000); 
            Debug.print("  accY="); Debug.print((float)ap_accY / 1000); 
            Debug.print("  accZ="); Debug.println((float)ap_accZ / 1000);
          #endif 
        #endif             
          break; 
    
        case MAVLINK_MSG_ID_SCALED_PRESSURE:         // #29
        #if defined Decode_Non_Essential_Mav
          if (!mavGood) break;        
          ap_press_abs = mavlink_msg_scaled_pressure_get_press_abs(&R2Gmsg);
          ap_temperature = mavlink_msg_scaled_pressure_get_temperature(&R2Gmsg);
          #if defined Mav_Debug_All || defined Mav_Debug_Scaled_Pressure
            Debug.print("Mavlink from FC #29 Scaled_Pressure: ");
            Debug.print("  press_abs=");  Debug.print(ap_press_abs,1);
            Debug.print("hPa  press_diff="); Debug.print(ap_press_diff, 3);
            Debug.print("hPa  temperature=");  Debug.print((float)(ap_temperature)/100, 1); 
            Debug.println("C");             
          #endif 
        #endif                          
          break;  
        case MAVLINK_MSG_ID_ATTITUDE:                // #30
          if (!mavGood) break;   

          ap_roll = mavlink_msg_attitude_get_roll(&R2Gmsg);              // Roll angle (rad, -pi..+pi)
          ap_pitch = mavlink_msg_attitude_get_pitch(&R2Gmsg);            // Pitch angle (rad, -pi..+pi)
          ap_yaw = mavlink_msg_attitude_get_yaw(&R2Gmsg);                // Yaw angle (rad, -pi..+pi)
          ap_rollspeed = mavlink_msg_attitude_get_rollspeed(&R2Gmsg);    // Roll angular speed (rad/s)
          ap_pitchspeed = mavlink_msg_attitude_get_pitchspeed(&R2Gmsg);  // Pitch angular speed (rad/s)
          ap_yawspeed = mavlink_msg_attitude_get_yawspeed(&R2Gmsg);      // Yaw angular speed (rad/s)           

          ap_roll = RadToDeg(ap_roll);   // Now degrees
          ap_pitch = RadToDeg(ap_pitch);
          ap_yaw = RadToDeg(ap_yaw);
          
          #if defined Mav_Debug_All || defined Mav_Debug_Attitude   
            Debug.print("Mavlink from FC #30 Attitude: ");      
            Debug.print(" ap_roll degs=");
            Debug.print(ap_roll, 1);
            Debug.print(" ap_pitch degs=");   
            Debug.print(ap_pitch, 1);
            Debug.print(" ap_yaw degs=");         
            Debug.println(ap_yaw, 1);
          #endif             
      
          PackSensorTable(0x5006, 0 );  // 0x5006 Attitude      

          break;  
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:     // #33
          if ((!mavGood) || (ap_fixtype < 3)) break;  
          ap_lat33 = mavlink_msg_global_position_int_get_lat(&R2Gmsg);             // Latitude, expressed as degrees * 1E7
          ap_lon33 = mavlink_msg_global_position_int_get_lon(&R2Gmsg);             // Pitch angle (rad, -pi..+pi)
          ap_amsl33 = mavlink_msg_global_position_int_get_alt(&R2Gmsg);          // Altitude above mean sea level (millimeters)
          ap_alt_ag = mavlink_msg_global_position_int_get_relative_alt(&R2Gmsg); // Altitude above ground (millimeters)
          ap_vx = mavlink_msg_global_position_int_get_vx(&R2Gmsg);               //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
          ap_vy = mavlink_msg_global_position_int_get_vy(&R2Gmsg);               //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
          ap_vz = mavlink_msg_global_position_int_get_vz(&R2Gmsg);               // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
          ap_gps_hdg = mavlink_msg_global_position_int_get_hdg(&R2Gmsg);         // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees        
 
          cur.lat = (float)ap_lat33 / 1E7;
          cur.lon = (float)ap_lon33 / 1E7;
          cur.alt = ap_amsl33 / 1E3;
          cur.hdg = ap_gps_hdg / 100;

          #if defined Mav_Debug_All || defined Mav_Debug_GPS_Int
            Debug.print("Mavlink from FC #33 GPS Int: ");
            Debug.print(" ap_lat="); Debug.print((float)ap_lat33 / 1E7, 6);
            Debug.print(" ap_lon="); Debug.print((float)ap_lon33 / 1E7, 6);
            Debug.print(" ap_amsl="); Debug.print((float)ap_amsl33 / 1E3, 0);
            Debug.print(" ap_alt_ag="); Debug.print((float)ap_alt_ag / 1E3, 1);           
            Debug.print(" ap_vx="); Debug.print((float)ap_vx / 100, 2);
            Debug.print(" ap_vy="); Debug.print((float)ap_vy / 100, 2);
            Debug.print(" ap_vz="); Debug.print((float)ap_vz / 100, 2);
            Debug.print(" ap_gps_hdg="); Debug.println((float)ap_gps_hdg / 100, 1);
          #endif  
                
          break;  
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:         // #35
          if (!mavGood) break; 
          ap_rssi35 = mavlink_msg_rc_channels_raw_get_rssi(&R2Gmsg);
          rssi35 = true;  
               
          if ((!rssi65) && (!rssi109)) { // If no #65 and no #109 received, then use #35
            rssiGood=true;            
            ap_rssi = ap_rssi35 / 2.55;  // 0xff -> 100%
            #if defined Mav_Debug_All || defined Debug_Rssi || defined Mav_Debug_RC
              #ifndef RSSI_Override
                Debug.print("Auto RSSI_Source===>  ");
              #endif
            #endif     
          }

          #if defined Mav_Debug_All || defined Debug_Rssi || defined Mav_Debug_RC
            Debug.print("Mavlink from FC #35 RC_Channels_Raw: ");                        
            Debug.print("  ap_rssi35=");  Debug.print(ap_rssi35);   // 0xff -> 100%
            Debug.print("  rssiGood=");  Debug.println(rssiGood); 
          #endif                    
          break;  
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW :          // #36
          if (!mavGood) break; 
      
          ap_port = mavlink_msg_servo_output_raw_get_port(&R2Gmsg);
          ap_servo_raw[0] = mavlink_msg_servo_output_raw_get_servo1_raw(&R2Gmsg);   
          ap_servo_raw[1] = mavlink_msg_servo_output_raw_get_servo2_raw(&R2Gmsg);
          ap_servo_raw[2] = mavlink_msg_servo_output_raw_get_servo3_raw(&R2Gmsg);   
          ap_servo_raw[3] = mavlink_msg_servo_output_raw_get_servo4_raw(&R2Gmsg);  
          ap_servo_raw[4] = mavlink_msg_servo_output_raw_get_servo5_raw(&R2Gmsg);   
          ap_servo_raw[5] = mavlink_msg_servo_output_raw_get_servo6_raw(&R2Gmsg);
          ap_servo_raw[6] = mavlink_msg_servo_output_raw_get_servo7_raw(&R2Gmsg);   
          ap_servo_raw[7] = mavlink_msg_servo_output_raw_get_servo8_raw(&R2Gmsg); 
          /* 
           *  not supported right now
          ap_servo_raw[8] = mavlink_R2Gmsg_servo_output_raw_get_servo9_raw(&R2Gmsg);   
          ap_servo_raw[9] = mavlink_R2Gmsg_servo_output_raw_get_servo10_raw(&R2Gmsg);
          ap_servo_raw[10] = mavlink_R2Gmsg_servo_output_raw_get_servo11_raw(&R2Gmsg);   
          ap_servo_raw[11] = mavlink_R2Gmsg_servo_output_raw_get_servo12_raw(&R2Gmsg); 
          ap_servo_raw[12] = mavlink_R2Gmsg_servo_output_raw_get_servo13_raw(&R2Gmsg);   
          ap_servo_raw[13] = mavlink_R2Gmsg_servo_output_raw_get_servo14_raw(&R2Gmsg);
          ap_servo_raw[14] = mavlink_R2Gmsg_servo_output_raw_get_servo15_raw(&R2Gmsg);   
          ap_servo_raw[15] = mavlink_R2Gmsg_servo_output_raw_get_servo16_raw(&R2Gmsg);
          */       
      
          #if defined Mav_Debug_All ||  defined Mav_Debug_Servo
            Debug.print("Mavlink from FC #36 servo_output: ");
            Debug.print("ap_port="); Debug.print(ap_port); 
            Debug.print(" PWM: ");
            for (int i=0 ; i < 8; i++) {
              Debug.print(" "); 
              Debug.print(i+1);
              Debug.print("=");  
              Debug.print(ap_servo_raw[i]);   
            }                         
            Debug.println();     
          #endif  
          
          #if defined PlusVersion
            PackSensorTable(0x50F1, 0);   // 0x50F1  SERVO_OUTPUT_RAW
          #endif  
                     
          break;  
        case MAVLINK_MSG_ID_MISSION_ITEM :          // #39
          if (!mavGood) break;
            ap_ms_seq = mavlink_msg_mission_item_get_seq(&R2Gmsg);
            ap_ms_frame = mavlink_msg_mission_item_get_frame(&R2Gmsg);                // The coordinate system of the waypoint.
            ap_ms_command = mavlink_msg_mission_item_get_command(&R2Gmsg);            // The scheduled action for the waypoint.
            ap_ms_current = mavlink_msg_mission_item_get_current(&R2Gmsg);            // false:0, true:1
            ap_ms_autocontinue = mavlink_msg_mission_item_get_autocontinue(&R2Gmsg);  //  Autocontinue to next waypoint
            ap_ms_param1 = mavlink_msg_mission_item_get_param1(&R2Gmsg);              // PARAM1, see MAV_CMD enum
            ap_ms_param2 = mavlink_msg_mission_item_get_param2(&R2Gmsg);              // PARAM2, see MAV_CMD enum
            ap_ms_param3 = mavlink_msg_mission_item_get_param3(&R2Gmsg);              // PARAM3, see MAV_CMD enum
            ap_ms_param3 = mavlink_msg_mission_item_get_param4(&R2Gmsg);              // PARAM4, see MAV_CMD enum
            ap_ms_x = mavlink_msg_mission_item_get_x(&R2Gmsg);                        // PARAM5 / local: X coordinate, global: latitude
            ap_ms_y = mavlink_msg_mission_item_get_y(&R2Gmsg);                        // PARAM6 / local: Y coordinate, global: longitude
            ap_ms_z = mavlink_msg_mission_item_get_z(&R2Gmsg);                        // PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
            ap_mission_type = mavlink_msg_mission_item_get_z(&R2Gmsg);                // MAV_MISSION_TYPE
                     
            #if defined Mav_Debug_All || defined Mav_Debug_Mission
              Debug.print("Mavlink from FC #39 Mission Item: ");
              Debug.print("ap_ms_seq="); Debug.print(ap_ms_seq);  
              Debug.print(" ap_ms_frame="); Debug.print(ap_ms_frame);   
              Debug.print(" ap_ms_command="); Debug.print(ap_ms_command);   
              Debug.print(" ap_ms_current="); Debug.print(ap_ms_current);   
              Debug.print(" ap_ms_autocontinue="); Debug.print(ap_ms_autocontinue);  
              Debug.print(" ap_ms_param1="); Debug.print(ap_ms_param1, 7);   
              Debug.print(" ap_ms_param2="); Debug.print(ap_ms_param2, 7);   
              Debug.print(" ap_ms_param3="); Debug.print(ap_ms_param3, 7);  
              Debug.print(" ap_ms_param4="); Debug.print(ap_ms_param4, 7); 
              Debug.print(" ap_ms_x="); Debug.print(ap_ms_x, 7);   
              Debug.print(" ap_ms_y="); Debug.print(ap_ms_y, 7);   
              Debug.print(" ap_ms_z="); Debug.print(ap_ms_z,0); 
              Debug.print(" ap_mission_type="); Debug.print(ap_mission_type); 
              Debug.println();    
            #endif
            
            if (ap_ms_seq > Max_Waypoints) {
              Debug.println(" Max Waypoints exceeded! Waypoint ignored.");
              break;
            }

             WP[ap_ms_seq-1].lat = ap_ms_x;     //  seq = 1 goes into slot [0]
             WP[ap_ms_seq-1].lon = ap_ms_y;
             
          break;                    
        case MAVLINK_MSG_ID_MISSION_CURRENT:         // #42 should come down regularly as part of EXTENDED_status group
          if (!mavGood) break;   
            ap_ms_seq =  mavlink_msg_mission_current_get_seq(&R2Gmsg);  
            
            #if defined Mav_Debug_All || defined Mav_Debug_Mission
            if (ap_ms_seq) {
              Debug.print("Mavlink from FC #42 Mission Current: ");
              Debug.print("ap_mission_current="); Debug.println(ap_ms_seq);   
            }
            #endif 
              
            if (ap_ms_seq > 0) ap_ms_current_flag = true;     //  Ok to send passthru frames 
  
          break; 
        case MAVLINK_MSG_ID_MISSION_COUNT :          // #44   received back after #43 Mission_Request_List sent
        #if defined Request_Missions_From_FC || defined Request_Mission_Count_From_FC
          if (!mavGood) break;  
            ap_mission_count =  mavlink_msg_mission_count_get_count(&R2Gmsg); 
            #if defined Mav_Debug_All || defined Mav_Debug_Mission
              Debug.print("Mavlink from FC #44 Mission Count: ");
              Debug.print("ap_mission_count="); Debug.println(ap_mission_count);   
            #endif
            #if defined Request_Missions_From_FC
            if ((ap_mission_count > 0) && (ap_ms_count_ft)) {
              ap_ms_count_ft = false;
              RequestAllWaypoints(ap_mission_count);  // # multiple #40, then wait for them to arrive at #39
            }
            #endif
          break; 
        #endif
        
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:   // #62
          if (!mavGood) break;    
            ap_nav_roll =  mavlink_msg_nav_controller_output_get_nav_roll(&R2Gmsg);             // Current desired roll
            ap_nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(&R2Gmsg);            // Current desired pitch
            ap_nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(&R2Gmsg);        // Current desired heading
            ap_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&R2Gmsg);  // Bearing to current waypoint/target
            ap_wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&R2Gmsg);                // Distance to active waypoint
            ap_alt_error = mavlink_msg_nav_controller_output_get_alt_error(&R2Gmsg);            // Current altitude error
            ap_aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&R2Gmsg);          // Current airspeed error
            ap_xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&R2Gmsg);      // Current crosstrack error on x-y plane

            #if defined Mav_Debug_All || defined Mav_Debug_Waypoints
              Debug.print("Mavlink from FC #62 Nav_Controller_Output - (+Waypoint): ");
              Debug.print("ap_nav_roll="); Debug.print(ap_nav_roll, 3);  
              Debug.print(" ap_nav_pitch="); Debug.print(ap_nav_pitch, 3);   
              Debug.print(" ap_nav_bearing="); Debug.print(ap_nav_bearing);   
              Debug.print(" ap_target_bearing="); Debug.print(ap_target_bearing);   
              Debug.print(" ap_wp_dist="); Debug.print(ap_wp_dist);  
              Debug.print(" ap_alt_error="); Debug.print(ap_alt_error, 2);   
              Debug.print(" ap_aspd_error="); Debug.print(ap_aspd_error, 2);   
              Debug.print(" ap_xtrack_error="); Debug.print(ap_xtrack_error, 2);  
              Debug.println();    
            #endif
            
            #if defined PlusVersion  
              PackSensorTable(0x5009, 0);  // 0x5009 Waypoints  
            #endif       
        
          break;     
        case MAVLINK_MSG_ID_RC_CHANNELS:             // #65
          if (!mavGood) break; 

            ap_chcnt = mavlink_msg_rc_channels_get_chancount(&R2Gmsg);
            ap_chan_raw[0] = mavlink_msg_rc_channels_get_chan1_raw(&R2Gmsg);   
            ap_chan_raw[1] = mavlink_msg_rc_channels_get_chan2_raw(&R2Gmsg);
            ap_chan_raw[2] = mavlink_msg_rc_channels_get_chan3_raw(&R2Gmsg);   
            ap_chan_raw[3] = mavlink_msg_rc_channels_get_chan4_raw(&R2Gmsg);  
            ap_chan_raw[4] = mavlink_msg_rc_channels_get_chan5_raw(&R2Gmsg);   
            ap_chan_raw[5] = mavlink_msg_rc_channels_get_chan6_raw(&R2Gmsg);
            ap_chan_raw[6] = mavlink_msg_rc_channels_get_chan7_raw(&R2Gmsg);   
            ap_chan_raw[7] = mavlink_msg_rc_channels_get_chan8_raw(&R2Gmsg);  
            ap_chan_raw[8] = mavlink_msg_rc_channels_get_chan9_raw(&R2Gmsg);   
            ap_chan_raw[9] = mavlink_msg_rc_channels_get_chan10_raw(&R2Gmsg);
            ap_chan_raw[10] = mavlink_msg_rc_channels_get_chan11_raw(&R2Gmsg);   
            ap_chan_raw[11] = mavlink_msg_rc_channels_get_chan12_raw(&R2Gmsg); 
            ap_chan_raw[12] = mavlink_msg_rc_channels_get_chan13_raw(&R2Gmsg);   
            ap_chan_raw[13] = mavlink_msg_rc_channels_get_chan14_raw(&R2Gmsg);
            ap_chan_raw[14] = mavlink_msg_rc_channels_get_chan15_raw(&R2Gmsg);   
            ap_chan_raw[15] = mavlink_msg_rc_channels_get_chan16_raw(&R2Gmsg);
            ap_chan_raw[16] = mavlink_msg_rc_channels_get_chan17_raw(&R2Gmsg);   
            ap_chan_raw[17] = mavlink_msg_rc_channels_get_chan18_raw(&R2Gmsg);
            ap_rssi65 = mavlink_msg_rc_channels_get_rssi(&R2Gmsg);   // Receive RSSI 0: 0%, 254: 100%, 255: invalid/unknown     
   
            rssi65 = true;  
             
            if (!rssi109) { // If no #109 received, then use #65
              rssiGood=true;            
              ap_rssi = ap_rssi65 / 2.55;  // 0xff -> 100%
              #if defined Mav_Debug_All || defined Debug_Rssi || defined Mav_Debug_RC
                #ifndef RSSI_Override
                  Debug.print("Auto RSSI_Source===>  ");
                #endif
              #endif     
              }
             
            #if defined Mav_Debug_All || defined Debug_Rssi || defined Mav_Debug_RC
              Debug.print("Mavlink from FC #65 RC_Channels: ");
              Debug.print("ap_chcnt="); Debug.print(ap_chcnt); 
              Debug.print(" PWM: ");
              for (int i=0 ; i < ap_chcnt ; i++) {
                Debug.print(" "); 
                Debug.print(i+1);
                Debug.print("=");  
                Debug.print(ap_chan_raw[i]);   
              }                         
              Debug.print("  ap_rssi65=");  Debug.print(ap_rssi65);   // 0xff -> 100%
              Debug.print("  rssiGood=");  Debug.println(rssiGood);         
            #endif             
          break;      
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:     // #66 - OUTGOING TO UAV
          if (!mavGood) break;       
          break; 
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:       // #73   received back after #51 Mission_Request_Int sent
        #if defined Mav_Debug_All || defined Mav_Debug_Mission
          if (!mavGood) break; 
          ap73_target_system =  mavlink_msg_mission_item_int_get_target_system(&R2Gmsg);   
          ap73_target_component =  mavlink_msg_mission_item_int_get_target_component(&R2Gmsg);   
          ap73_seq = mavlink_msg_mission_item_int_get_seq(&R2Gmsg);           // Waypoint ID (sequence number)
          ap73_frame = mavlink_msg_mission_item_int_get_frame(&R2Gmsg);       // MAV_FRAME The coordinate system of the waypoint.
          ap73_command = mavlink_msg_mission_item_int_get_command(&R2Gmsg);   // MAV_CMD The scheduled action for the waypoint.
          ap73_current = mavlink_msg_mission_item_int_get_current(&R2Gmsg);   // false:0, true:1
          ap73_autocontinue = mavlink_msg_mission_item_int_get_autocontinue(&R2Gmsg);   // Autocontinue to next waypoint
          ap73_param1 = mavlink_msg_mission_item_int_get_param1(&R2Gmsg);     // PARAM1, see MAV_CMD enum
          ap73_param2 = mavlink_msg_mission_item_int_get_param2(&R2Gmsg);     // PARAM2, see MAV_CMD enum
          ap73_param3 = mavlink_msg_mission_item_int_get_param3(&R2Gmsg);     // PARAM3, see MAV_CMD enum
          ap73_param4 = mavlink_msg_mission_item_int_get_param4(&R2Gmsg);     // PARAM4, see MAV_CMD enum
          ap73_x = mavlink_msg_mission_item_int_get_x(&R2Gmsg);               // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
          ap73_y = mavlink_msg_mission_item_int_get_y(&R2Gmsg);               // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
          ap73_z = mavlink_msg_mission_item_int_get_z(&R2Gmsg);               // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
          ap73_mission_type = mavlink_msg_mission_item_int_get_mission_type(&R2Gmsg); // Mav2   MAV_MISSION_TYPE  Mission type.
 
          Debug.print("Mavlink from FC #73 Mission_Item_Int: ");
          Debug.print("target_system ="); Debug.print(ap73_target_system);   
          Debug.print("  target_component ="); Debug.print(ap73_target_component);   
          Debug.print(" _seq ="); Debug.print(ap73_seq);   
          Debug.print("  frame ="); Debug.print(ap73_frame);     
          Debug.print("  command ="); Debug.print(ap73_command);   
          Debug.print("  current ="); Debug.print(ap73_current);   
          Debug.print("  autocontinue ="); Debug.print(ap73_autocontinue);   
          Debug.print("  param1 ="); Debug.print(ap73_param1, 2); 
          Debug.print("  param2 ="); Debug.print(ap73_param2, 2); 
          Debug.print("  param3 ="); Debug.print(ap73_param3, 2); 
          Debug.print("  param4 ="); Debug.print(ap73_param4, 2);                                          
          Debug.print("  x ="); Debug.print(ap73_x);   
          Debug.print("  y ="); Debug.print(ap73_y);   
          Debug.print("  z ="); Debug.print(ap73_z, 4);    
          Debug.print("  mission_type ="); Debug.println(ap73_mission_type);                                                    

          break; 
        #endif
                               
        case MAVLINK_MSG_ID_VFR_HUD:                 //  #74
          if (!mavGood) break;      
          ap_hud_air_spd = mavlink_msg_vfr_hud_get_airspeed(&R2Gmsg);
          ap_hud_grd_spd = mavlink_msg_vfr_hud_get_groundspeed(&R2Gmsg);      //  in m/s
          ap_hud_hdg = mavlink_msg_vfr_hud_get_heading(&R2Gmsg);              //  in degrees
          ap_hud_throt = mavlink_msg_vfr_hud_get_throttle(&R2Gmsg);           //  integer percent
          ap_hud_bar_alt = mavlink_msg_vfr_hud_get_alt(&R2Gmsg);              //  m
          ap_hud_climb = mavlink_msg_vfr_hud_get_climb(&R2Gmsg);              //  m/s

          cur.hdg = ap_hud_hdg;
          
         #if defined Mav_Debug_All || defined Mav_Debug_Hud
            Debug.print("Mavlink from FC #74 VFR_HUD: ");
            Debug.print("Airspeed= "); Debug.print(ap_hud_air_spd, 2);                 // m/s    
            Debug.print("  Groundspeed= "); Debug.print(ap_hud_grd_spd, 2);            // m/s
            Debug.print("  Heading= ");  Debug.print(ap_hud_hdg);                      // deg
            Debug.print("  Throttle %= ");  Debug.print(ap_hud_throt);                 // %
            Debug.print("  Baro alt= "); Debug.print(ap_hud_bar_alt, 0);               // m                  
            Debug.print("  Climb rate= "); Debug.println(ap_hud_climb);                // m/s
          #endif  

          PackSensorTable(0x5005, 0);  // 0x5005 VelYaw

          #if defined PlusVersion
            PackSensorTable(0x50F2, 0);  // 0x50F2 VFR HUD
          #endif
            
          break; 
        case MAVLINK_MSG_ID_RADIO_STATUS:         // #109
          if (!mavGood) break;

            ap_rssi109 = mavlink_msg_radio_status_get_rssi(&R2Gmsg);         // air signal strength
            ap_remrssi = mavlink_msg_radio_status_get_remrssi(&R2Gmsg);      // remote signal strength
            ap_txbuf = mavlink_msg_radio_status_get_txbuf(&R2Gmsg);          // how full the tx buffer is as a percentage
            ap_noise = mavlink_msg_radio_status_get_noise(&R2Gmsg);          // remote background noise level
            ap_remnoise = mavlink_msg_radio_status_get_remnoise(&R2Gmsg);    // receive errors
            ap_rxerrors = mavlink_msg_radio_status_get_rxerrors(&R2Gmsg);    // count of error corrected packets
            ap_fixed = mavlink_msg_radio_status_get_fixed(&R2Gmsg);
            rssi109 = true;  
              
            // If we get #109 then it must be a SiK fw radio, so use this record for rssi
            rssiGood=true;            
            ap_rssi = ap_rssi109;   //  Percent
            
            #if defined SiK_Rssi_Percent
              ap_rssi = ap_rssi109;          //  Percent
            #else
              ap_rssi = ap_rssi109 / 2.55;   //  0xff -> 100%            
            #endif
            
            #if defined Mav_Debug_All || defined Debug_Rssi || defined Mav_Debug_RC
              #ifndef RSSI_Override
                Debug.print("Auto RSSI_Source===>  ");
              #endif
            #endif     

            #if defined Mav_Debug_All || defined Debug_Radio_Status || defined Debug_Rssi
              Debug.print("Mavlink from FC #109 Radio: "); 
              Debug.print("ap_rssi109="); Debug.print(ap_rssi109);
              Debug.print("  remrssi="); Debug.print(ap_remrssi);
              Debug.print("  txbuf="); Debug.print(ap_txbuf);
              Debug.print("  noise="); Debug.print(ap_noise); 
              Debug.print("  remnoise="); Debug.print(ap_remnoise);
              Debug.print("  rxerrors="); Debug.print(ap_rxerrors);
              Debug.print("  fixed="); Debug.print(ap_fixed);  
              Debug.print("  rssiGood=");  Debug.println(rssiGood);                                
            #endif 

          break;     
           
        case MAVLINK_MSG_ID_SCALED_IMU2:       // #116   https://mavlink.io/en/messages/common.html
          if (!mavGood) break;       
          break;
           
        case MAVLINK_MSG_ID_POWER_STATUS:      // #125   https://mavlink.io/en/messages/common.html
          #if defined Decode_Non_Essential_Mav
            if (!mavGood) break;  
            ap_Vcc = mavlink_msg_power_status_get_Vcc(&R2Gmsg);         // 5V rail voltage in millivolts
            ap_Vservo = mavlink_msg_power_status_get_Vservo(&R2Gmsg);   // servo rail voltage in millivolts
            ap_flags = mavlink_msg_power_status_get_flags(&R2Gmsg);     // power supply status flags (see MAV_POWER_status enum)
            #ifdef Mav_Debug_All
              Debug.print("Mavlink from FC #125 Power Status: ");
              Debug.print("Vcc= "); Debug.print(ap_Vcc); 
              Debug.print("  Vservo= ");  Debug.print(ap_Vservo);       
              Debug.print("  flags= ");  Debug.println(ap_flags);       
            #endif  
          #endif              
            break; 
         case MAVLINK_MSG_ID_BATTERY_STATUS:      // #147   https://mavlink.io/en/messages/common.html
          if (!mavGood) break;       
          ap_battery_id = mavlink_msg_battery_status_get_id(&R2Gmsg);  
          ap_current_battery = mavlink_msg_battery_status_get_current_battery(&R2Gmsg);      // in 10*milliamperes (1 = 10 milliampere)
          ap_current_consumed = mavlink_msg_battery_status_get_current_consumed(&R2Gmsg);    // mAh
          ap_battery_remaining = mavlink_msg_battery_status_get_battery_remaining(&R2Gmsg);  // (0%: 0, 100%: 100)  

          if (ap_battery_id == 0) {  // Battery 1
            fr_bat1_mAh = ap_current_consumed;                       
          } else if (ap_battery_id == 1) {  // Battery 2
              fr_bat2_mAh = ap_current_consumed;                              
          } 
             
          #if defined Mav_Debug_All || defined Debug_Batteries
            Debug.print("Mavlink from FC #147 Battery Status: ");
            Debug.print(" bat id= "); Debug.print(ap_battery_id); 
            Debug.print(" bat current mA= "); Debug.print(ap_current_battery*10); 
            Debug.print(" ap_current_consumed mAh= ");  Debug.print(ap_current_consumed);   
            if (ap_battery_id == 0) {
              Debug.print(" my di/dt mAh= ");  
              Debug.println(Total_mAh1(), 0);  
            }
            else {
              Debug.print(" my di/dt mAh= ");  
              Debug.println(Total_mAh2(), 0);   
            }    
        //  Debug.print(" bat % remaining= ");  Debug.println(ap_time_remaining);       
          #endif                        
          
          break;    
        case MAVLINK_MSG_ID_SENSOR_OFFSETS:    // #150   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;        
          break; 
        case MAVLINK_MSG_ID_MEMINFO:           // #152   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;        
          break;   
        case MAVLINK_MSG_ID_RADIO:             // #166   See #109 RADIO_status
        
          break; 
        case MAVLINK_MSG_ID_RANGEFINDER:       // #173   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;       
          ap_range = mavlink_msg_rangefinder_get_distance(&R2Gmsg);  // distance in meters

          #if defined Mav_Debug_All || defined Mav_Debug_Range
            Debug.print("Mavlink from FC #173 rangefinder: ");        
            Debug.print(" distance=");
            Debug.println(ap_range);   // now V
          #endif  

          PackSensorTable(0x5006, 0);  // 0x5006 Rangefinder
             
          break;            
        case MAVLINK_MSG_ID_AHRS2:             // #178   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;       
          break;  
        case MAVLINK_MSG_ID_BATTERY2:          // #181   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;
          ap_voltage_battery2 = Get_Volt_Average2(mavlink_msg_battery2_get_voltage(&R2Gmsg));        // 1000 = 1V
          ap_current_battery2 = Get_Current_Average2(mavlink_msg_battery2_get_current_battery(&R2Gmsg));     //  100 = 1A
          if(ap_voltage_battery2 > 21000) ap_cell_count2 = 6;
            else if (ap_voltage_battery2 > 16800 && ap_cell_count2 != 6) ap_cell_count2 = 5;
            else if(ap_voltage_battery2 > 12600 && ap_cell_count2 != 5) ap_cell_count2 = 4;
            else if(ap_voltage_battery2 > 8400 && ap_cell_count2 != 4) ap_cell_count2 = 3;
            else if(ap_voltage_battery2 > 4200 && ap_cell_count2 != 3) ap_cell_count2 = 2;
            else ap_cell_count2 = 0;
   
          #if defined Mav_Debug_All || defined Debug_Batteries
            Debug.print("Mavlink from FC #181 Battery2: ");        
            Debug.print(" Bat volts=");
            Debug.print((float)ap_voltage_battery2 / 1000, 3);   // now V
            Debug.print("  Bat amps=");
            Debug.print((float)ap_current_battery2 / 100, 1);   // now A
              
            Debug.print("  mAh="); Debug.print(bat2.mAh, 6);    
            Debug.print("  Total mAh="); Debug.print(bat2.tot_mAh, 3);
         
            Debug.print("  Bat cell count= "); 
            Debug.println(ap_cell_count2);
          #endif

          PackSensorTable(0x5008, 0);   // 0x5008 Bat2       
                   
          break;
          
        case MAVLINK_MSG_ID_AHRS3:             // #182   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;       
          break;
        case MAVLINK_MSG_ID_STATUSTEXT:        // #253      
          ap_severity = mavlink_msg_statustext_get_severity(&R2Gmsg);
          len=mavlink_msg_statustext_get_text(&R2Gmsg, ap_text);

          #if defined Mav_Debug_All || defined Mav_Debug_StatusText
            Debug.print("Mavlink from FC #253 Statustext pushed onto MsgRingBuff: ");
            Debug.print(" Severity="); Debug.print(ap_severity);
            Debug.print(" "); Debug.print(MavSeverity(ap_severity));
            Debug.print("  Text= ");  Debug.print(" |"); Debug.print(ap_text); Debug.println("| ");
          #endif

          PackSensorTable(0x5000, 0);         // 0x5000 StatusText Message
          
          break;                                      
        default:
          if (!mavGood) break;
          #if defined Mav_Debug_All || defined Mav_Debug_Unknown_Msgs
            Debug.print("Mavlink from FC: ");
            Debug.print("Unknown Message ID #");
            Debug.print(R2Gmsg.msgid);
            Debug.println(" Ignored"); 
          #endif

          break;
      }
}

//***************************************************
void MarkHome()  {
  
  homGood = true;
  hom.lat = cur.lat;
  hom.lon = cur.lon;
  hom.alt = cur.alt;
  hom.hdg = cur.hdg;

  #if defined Mav_Debug_All || defined Mav_Debug_GPS_Int
    Debug.print("******************************************Mavlink in #33 GPS Int: Home established: ");       
    Debug.print("hom.lat=");  Debug.print(hom.lat, 7);
    Debug.print(" hom.lon=");  Debug.print(hom.lon, 7 );        
    Debug.print(" hom.alt="); Debug.print(hom.alt, 1);
    Debug.print(" hom.hdg="); Debug.println(hom.hdg);                   
 #endif  
}
//***************************************************
void Send_FC_Heartbeat() {
  
  apo_sysid = 20;                                // ID 20 for this aircraft
  apo_compid = 1;                                //  autopilot1

  apo_type = MAV_TYPE_GCS;                       // 6 Pretend to be a GCS
  apo_autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;   // 3 AP Mega
  apo_base_mode = 0;
  apo_system_status = MAV_STATE_ACTIVE;         // 4
   
  mavlink_msg_heartbeat_pack(apo_sysid, apo_compid, &G2Fmsg, apo_type, apo_autopilot, apo_base_mode, apo_system_status, 0); 
  Write_To_FC(0); 
}
//***************************************************
 void Param_Request_Read(int16_t param_index) {
  ap_sysid = 20;                        // ID 20 for this aircraft
  ap_compid = 1;                        //  autopilot1

  mavlink_msg_param_request_read_pack(ap_sysid, ap_compid, &G2Fmsg,
                   ap_targsys, ap_targcomp, ap_param_id, param_index);
                
  Write_To_FC(20);             
 }

//***************************************************
 void Request_Param_List() {

  ap_sysid = 20;                        // ID 20 for this aircraft
  ap_compid = 1;                        //  autopilot1
  
  mavlink_msg_param_request_list_pack(ap_sysid,  ap_compid, &G2Fmsg,
                    ap_targsys,  ap_targcomp);
              
  Write_To_FC(21);
                    
 }
//***************************************************
#ifdef Request_Missions_From_FC
void RequestMission(uint16_t ms_seq) {    //  #40
  ap_sysid = 0xFF;
  ap_compid = 0xBE;
  ap_targsys = 1;
  ap_targcomp = 1; 
  ap_mission_type = 0;   // Mav2  0 = Items are mission commands for main mission
  
  mavlink_msg_mission_request_pack(ap_sysid, ap_compid, &G2Fmsg,
                               ap_targsys, ap_targcomp, ms_seq, ap_mission_type);

  Write_To_FC(40);
  #if defined Mav_Debug_All || defined Mav_Debug_Mission
    Debug.print("Mavlink to FC #40 Request Mission:  ms_seq="); Debug.println(ms_seq);
  #endif  
}
#endif 
 
//***************************************************
#if defined Request_Missions_From_FC || defined Request_Mission_Count_From_FC
void RequestMissionList() {   // #43   get back #44 Mission_Count
  ap_sysid = 0xFF;
  ap_compid = 0xBE;
  ap_targsys = 1;
  ap_targcomp = 1; 
  ap_mission_type = 0;   // Mav2  0 = Items are mission commands for main mission
  
  mavlink_msg_mission_request_list_pack(ap_sysid, ap_compid, &G2Fmsg,
                               ap_targsys, ap_targcomp, ap_mission_type);
                             
  Write_To_FC(43);
  #if defined Mav_Debug_All || defined Mav_Debug_Mission
    Debug.println("Mavlink to FC #43 Request Mission List (count)");
  #endif  
}
#endif
//***************************************************
#ifdef Request_Missions_From_FC
void RequestAllWaypoints(uint16_t ms_count) {
  for (int i = 0; i < ms_count; i++) {  //  Mission count = next empty WP, i.e. one too high
    RequestMission(i); 
  }
}
#endif
//***************************************************
#ifdef Data_Streams_Enabled    
void RequestDataStreams() {    //  REQUEST_DATA_STREAM ( #66 ) DEPRECATED. USE SRx, SET_MESSAGE_INTERVAL INSTEAD

  ap_sysid = 0xFF;
  ap_compid = 0xBE;
  ap_targsys = 1;
  ap_targcomp = 1;

  const int maxStreams = 7;
  const uint8_t mavStreams[] = {
  MAV_DATA_STREAM_RAW_SENSORS,
  MAV_DATA_STREAM_EXTENDED_status,
  MAV_DATA_STREAM_RC_CHANNELS,
  MAV_DATA_STREAM_POSITION,
  MAV_DATA_STREAM_EXTRA1, 
  MAV_DATA_STREAM_EXTRA2,
  MAV_DATA_STREAM_EXTRA3
  };

  const uint16_t mavRates[] = { 0x04, 0x0a, 0x04, 0x0a, 0x04, 0x04, 0x04};
 // req_message_rate The requested interval between two messages of this type

  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(ap_sysid, ap_compid, &G2Fmsg,
        ap_targsys, ap_targcomp, mavStreams[i], mavRates[i], 1);    // start_stop 1 to start sending, 0 to stop sending   
                          
  Write_To_FC(66);
    }
 // Debug.println("Mavlink to FC #66 Request Data Streams:");
}
#endif

//***************************************************
void ServiceStatusLeds() {
  ServiceMavStatusLed();
  if (BufStatusLed != 99) {
    ServiceBufStatusLed();
  }
}
void ServiceMavStatusLed() {
  if (mavGood) {

      MavLedState = HIGH;
      digitalWrite(MavStatusLed, MavLedState); 
  }
    else {
      BlinkMavLed(500);
    }
  digitalWrite(MavStatusLed, MavLedState); 
}

void ServiceBufStatusLed() {
  digitalWrite(BufStatusLed, BufLedState);  
}

void BlinkMavLed(uint32_t period) {
  uint32_t cMillis = millis();
     if (cMillis - mav_led_millis >= period) {    // blink period
        mav_led_millis = cMillis;
        if (MavLedState == LOW) {
          MavLedState = HIGH; }   
        else {
          MavLedState = LOW;  } 
      }
}

//***************************************************
void DisplayByte(byte b) {
  if (b<=0xf) Debug.print("0");
  Debug.print(b,HEX);
  Debug.print(" ");
}
//***************************************************

void PrintMavBuffer(const void *object){

    const unsigned char * const bytes = static_cast<const unsigned char *>(object);
  int j;

uint8_t   tl;

uint8_t mavNum;

//Mavlink 1 and 2
uint8_t mav_magic;               // protocol magic marker
uint8_t mav_len;                 // Length of payload

//uint8_t mav_incompat_flags;    // MAV2 flags that must be understood
//uint8_t mav_compat_flags;      // MAV2 flags that can be ignored if not understood

uint8_t mav_seq;                // Sequence of packet
//uint8_t mav_sysid;            // ID of message sender system/aircraft
//uint8_t mav_compid;           // ID of the message sender component
uint8_t mav_msgid;            
/*
uint8_t mav_msgid_b1;           ///< first 8 bits of the ID of the message 0:7; 
uint8_t mav_msgid_b2;           ///< middle 8 bits of the ID of the message 8:15;  
uint8_t mav_msgid_b3;           ///< last 8 bits of the ID of the message 16:23;
uint8_t mav_payload[280];      ///< A maximum of 255 payload bytes
uint16_t mav_checksum;          ///< X.25 CRC
*/

  
  if ((bytes[0] == 0xFE) || (bytes[0] == 0xFD)) {
    j = -2;   // relative position moved forward 2 places
  } else {
    j = 0;
  }
   
  mav_magic = bytes[j+2];
  if (mav_magic == 0xFE) {  // Magic / start signal
    mavNum = 1;
  } else {
    mavNum = 2;
  }
/* Mav1:   8 bytes header + payload
 * magic
 * length
 * sequence
 * sysid
 * compid
 * msgid
 */
  
  if (mavNum == 1) {
    Debug.print("mav1: /");

    if (j == 0) {
      DisplayByte(bytes[0]);   // CRC1
      DisplayByte(bytes[1]);   // CRC2
      Debug.print("/");
      }
    mav_magic = bytes[j+2];   
    mav_len = bytes[j+3];
 //   mav_incompat_flags = bytes[j+4];;
 //   mav_compat_flags = bytes[j+5];;
    mav_seq = bytes[j+6];
 //   mav_sysid = bytes[j+7];
 //   mav_compid = bytes[j+8];
    mav_msgid = bytes[j+9];

    //Debug.print(TimeString(millis()/1000)); Debug.print(": ");
  
    Debug.print("seq="); Debug.print(mav_seq); Debug.print("\t"); 
    Debug.print("len="); Debug.print(mav_len); Debug.print("\t"); 
    Debug.print("/");
    for (int i = (j+2); i < (j+10); i++) {  // Print the header
      DisplayByte(bytes[i]); 
    }
    
    Debug.print("  ");
    Debug.print("#");
    Debug.print(mav_msgid);
    if (mav_msgid < 100) Debug.print(" ");
    if (mav_msgid < 10)  Debug.print(" ");
    Debug.print("\t");
    
    tl = (mav_len+10);                // Total length: 8 bytes header + Payload + 2 bytes CRC
 //   for (int i = (j+10); i < (j+tl); i++) {  
    for (int i = (j+10); i <= (tl); i++) {    
     DisplayByte(bytes[i]);     
    }
    if (j == -2) {
      Debug.print("//");
      DisplayByte(bytes[mav_len + 8]); 
      DisplayByte(bytes[mav_len + 9]); 
      }
    Debug.println("//");  
  } else {

/* Mav2:   10 bytes
 * magic
 * length
 * incompat_flags
 * mav_compat_flags 
 * sequence
 * sysid
 * compid
 * msgid[11] << 16) | [10] << 8) | [9]
 */
    
    Debug.print("mav2:  /");
    if (j == 0) {
      DisplayByte(bytes[0]);   // CRC1
      DisplayByte(bytes[1]);   // CRC2 
      Debug.print("/");
    }
    mav_magic = bytes[2]; 
    mav_len = bytes[3];
//    mav_incompat_flags = bytes[4]; 
  //  mav_compat_flags = bytes[5];
    mav_seq = bytes[6];
//    mav_sysid = bytes[7];
   // mav_compid = bytes[8]; 
    mav_msgid = (bytes[11] << 16) | (bytes[10] << 8) | bytes[9]; 

    //Debug.print(TimeString(millis()/1000)); Debug.print(": ");

    Debug.print("seq="); Debug.print(mav_seq); Debug.print("\t"); 
    Debug.print("len="); Debug.print(mav_len); Debug.print("\t"); 
    Debug.print("/");
    for (int i = (j+2); i < (j+12); i++) {  // Print the header
     DisplayByte(bytes[i]); 
    }

    Debug.print("  ");
    Debug.print("#");
    Debug.print(mav_msgid);
    if (mav_msgid < 100) Debug.print(" ");
    if (mav_msgid < 10)  Debug.print(" ");
    Debug.print("\t");

 //   tl = (mav_len+27);                // Total length: 10 bytes header + Payload + 2B CRC +15 bytes signature
    tl = (mav_len+22);                  // This works, the above does not!
    for (int i = (j+12); i < (tl+j); i++) {   
       if (i == (mav_len + 12)) {
        Debug.print("/");
      }
      if (i == (mav_len + 12 + 2+j)) {
        Debug.print("/");
      }
      DisplayByte(bytes[i]); 
    }
    Debug.println();
  }

   Debug.print("Raw: ");
   for (int i = 0; i < 40; i++) {  //  unformatted
      DisplayByte(bytes[i]); 
    }
   Debug.println();
  
}
//***************************************************
//***************************************************
float RadToDeg (float _Rad) {
  return _Rad * 180 / PI;  
}
//***************************************************
float DegToRad (float _Deg) {
  return _Deg * PI / 180;  
}
//***************************************************
String MavSeverity(uint8_t sev) {
 switch(sev) {
    
    case 0:
      return "EMERGENCY";     // System is unusable. This is a "panic" condition. 
      break;
    case 1:
      return "ALERT";         // Action should be taken immediately. Indicates error in non-critical systems.
      break;
    case 2:
      return "CRITICAL";      // Action must be taken immediately. Indicates failure in a primary system.
      break; 
    case 3:
      return "ERROR";         //  Indicates an error in secondary/redundant systems.
      break; 
    case 4:
      return "WARNING";       //  Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
      break; 
    case 5:
      return "NOTICE";        //  An unusual event has occured, though not an error condition. This should be investigated for the root cause.
      break;
    case 6:
      return "INFO";          //  Normal operational messages. Useful for logging. No action is required for these messages.
      break; 
    case 7:
      return "DEBUG";         // Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
      break; 
    default:
      return "UNKNOWN";                                          
   }
}
//***************************************************
String PX4FlightModeName(uint8_t main, uint8_t sub) {
 switch(main) {
    
    case 1:
      return "MANUAL"; 
      break;
    case 2:
      return "ALTITUDE";        
      break;
    case 3:
      return "POSCTL";      
      break; 
    case 4:
 
      switch(sub) {
        case 1:
          return "AUTO READY"; 
          break;    
        case 2:
          return "AUTO TAKEOFF"; 
          break; 
        case 3:
          return "AUTO LOITER"; 
          break;    
        case 4:
          return "AUTO MISSION"; 
          break; 
        case 5:
          return "AUTO RTL"; 
          break;    
        case 6:
          return "AUTO LAND"; 
          break; 
        case 7:
          return "AUTO RTGS"; 
          break;    
        case 8:
          return "AUTO FOLLOW ME"; 
          break; 
        case 9:
          return "AUTO PRECLAND"; 
          break; 
        default:
          return "AUTO UNKNOWN";   
          break;
      } 
      
    case 5:
      return "ACRO";
    case 6:
      return "OFFBOARD";        
      break;
    case 7:
      return "STABILIZED";
      break;        
    case 8:
      return "RATTITUDE";        
      break; 
    case 9:
      return "SIMPLE";  
    default:
      return "UNKNOWN";
      break;                                          
   }
}
//***************************************************
uint8_t PX4FlightModeNum(uint8_t main, uint8_t sub) {
 switch(main) {
    
    case 1:
      return 0;  // MANUAL 
    case 2:
      return 1;  // ALTITUDE       
    case 3:
      return 2;  // POSCTL      
    case 4:
 
      switch(sub) {
        case 1:
          return 12;  // AUTO READY
        case 2:
          return 13;  // AUTO TAKEOFF 
        case 3:
          return 14;  // AUTO LOITER  
        case 4:
          return 15;  // AUTO MISSION 
        case 5:
          return 16;  // AUTO RTL 
        case 6:
          return 17;  // AUTO LAND 
        case 7:
          return 18;  //  AUTO RTGS 
        case 8:
          return 19;  // AUTO FOLLOW ME 
        case 9:
          return 20;  //  AUTO PRECLAND 
        default:
          return 31;  //  AUTO UNKNOWN   
      } 
      
    case 5:
      return 3;  //  ACRO
    case 6:
      return 4;  //  OFFBOARD        
    case 7:
      return 5;  //  STABILIZED
    case 8:
      return 6;  //  RATTITUDE        
    case 9:
      return 7;  //  SIMPLE 
    default:
      return 11;  //  UNKNOWN                                        
   }
}
//***************************************************
void ShowPeriod() {
  Debug.print("Period ms=");
  now_millis=millis();
  Debug.print(now_millis-prev_millis);
  Debug.print("\t");
  prev_millis=now_millis;
}
//***************************************************
void OledPrintln(String S) {
#if (Target_Board == 3) || (Target_Board == 4) 
  if (row>(max_row-1)) {  // last line    0 thru max_row-1
    
    display.clearDisplay();
    display.setCursor(0,0);
    
    for (int i = 0; i < (max_row-1); i++) {     // leave space for new line at the bottom
                                                //   if (i > 1) {   // don't scroll the 2 heading lines
      if (i >= 0) {         
        memset(OL[i].OLx, '\0', sizeof(OL[i].OLx));  // flush          
        strncpy(OL[i].OLx, OL[i+1].OLx, sizeof(OL[i+1].OLx));
      }
      display.println(OL[i].OLx);
    }
    display.display();
    row=max_row-1;
  }
  display.println(S);
  display.display();

  uint8_t lth = strlen(S.c_str());
  

    
  for (int i=0 ; i < lth ; i++ ) {
    OL[row].OLx[col] = S[i];
    col++;
    if (col > max_col-1) break;
  }
  
  for (col=col ; col < max_col-1; col++) {   //  flush to eol
    OL[row].OLx[col] = '\0';
  }
  col = 0;
  row++;
  
 // strncpy(OL[row].OLx, S.c_str(), max_col-1 );  
 // row++;
#endif  
}
//***************************************************
void OledPrint(String S) {
#if (Target_Board == 3) || (Target_Board == 4) 
  if (row>(max_row-1)) {  // last line    0 thru max_row-1
    
    display.clearDisplay();
    display.setCursor(0,0);
    
    for (int i = 0; i < (max_row-1); i++) {     // leave space for new line at the bottom
                                                //   if (i > 1) {   // don't scroll the 2 heading lines
      if (i >= 0) {         
        memset(OL[i].OLx, '\0', sizeof(OL[i].OLx));  // flush          
        strncpy(OL[i].OLx, OL[i+1].OLx, sizeof(OL[i+1].OLx));
      }
      display.print(OL[i].OLx);
    }
    display.display();
    row=max_row-1;
  }
  display.print(S);
  display.display();

  uint8_t lth = strlen(S.c_str());
  
  for (int i=0 ; i < lth ; i++ ) {
    OL[row].OLx[col] = S[i];
    col++;
    if (col > max_col-1) {
      break;
    }
  } 

  for (int i = col ; i < max_col-1; i++) {   //  flush to eol
    OL[row].OLx[i] = '\0';
  }
  
  if (col > max_col-1) {
    col = 0;
    row++;
   } 
#endif  
}

//************************************************************
 #if ((FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)) //  WiFi
 
  void SetupWiFi() { 

    bool apMode = false;  // used when STA fails to connect
    
     //*******************************  S T A T I O N   *****************************
    #if (WiFi_Mode == 2)  // STA
      uint8_t retry = 0;
      Debug.print("Trying to connect to ");  
      Debug.print(STAssid); 
      OledPrintln("WiFi trying ..");

      WiFi.disconnect(true);   // To circumvent "wifi: Set status to INIT" error bug
      delay(500);
      WiFi.mode(WIFI_STA);
      delay(500);
      
      WiFi.begin(STAssid, STApw);
      while (WiFi.status() != WL_CONNECTED){
        retry++;
        if (retry > 10) {
          Debug.println();
          Debug.println("Failed to connect in STA mode");
          OledPrintln("Failed in STA Mode");
          wifiSuDone = true;
          
          #ifdef AutoAP  
            apMode = true;            // Rather go establish an AP instead
            Debug.println("Starting AP instead.");
            OledPrintln("Starting AP instead");  
            //new from Target0815:
            Debug.println("WiFi-Reset ...");
            #if defined Target_Board ==  3
              WiFi.mode(WIFI_MODE_NULL);
            #else
              WiFi.mode(WIFI_OFF);  // esp8266 target board 4
            #endif                 
            delay(500);             
          #endif  
          
          break;
        }
        delay(500);
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        localIP = WiFi.localIP();
        Debug.println();
        Debug.println("WiFi connected!");
        Debug.print("Local IP address: ");
        Debug.print(localIP);

        #if   (WiFi_Protocol == 1)   // TCP
          Debug.print("  port: ");
          Debug.println(tcp_localPort);    //  UDP port is printed lower down
        #else 
          Debug.println();
        #endif 
         
        wifi_rssi = WiFi.RSSI();
        Debug.print("WiFi RSSI:");
        Debug.print(wifi_rssi);
        Debug.println(" dBm");

        OledPrintln("Connected!");
        OledPrintln(localIP.toString());
        
        #if (WiFi_Protocol == 1)   // TCP
          server.begin();                     // Not need now
          Debug.println("TCP server started");
          OledPrintln("TCP server started");
        #endif

        #if (WiFi_Protocol == 2)  // UDP
          udp.begin(udp_localPort);
          Debug.printf("UDP started, listening on IP %s, UDP port %d\n", localIP.toString().c_str(), udp_localPort);
          OledPrint("UDP port = ");  OledPrintln(String(udp_localPort));
        #endif
        
        wifiSuDone = true;
        wifiSuGood = true;
      } 
    #endif

     //*******************************  Access Point   *****************************
    #if (WiFi_Mode == 1)  // AP
      apMode = true;
    #endif

    if (apMode)   {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(APssid, APpw, APchannel);
      localIP = WiFi.softAPIP();
      Debug.print("AP IP address: ");
      Debug.print (localIP); 
      Debug.print("  SSID: ");
      Debug.println(String(APssid));
      OledPrintln("WiFi AP SSID =");
      OledPrintln(String(APssid));
      
      #if (WiFi_Protocol == 1)      // TCP
        server.begin();             //  Server for TCP/IP traffic
        Debug.printf("TCP/IP started, listening on IP %s, TCP port %d\n", localIP.toString().c_str(), tcp_localPort);
        OledPrint("TCP port = ");  OledPrintln(String(tcp_localPort));
      #endif  

      #if (WiFi_Protocol == 2)      // UDP
        udp.begin(udp_localPort);
        Debug.printf("UDP started, listening on IP %s, UDP port %d\n", WiFi.softAPIP().toString().c_str(), udp_localPort);
        OledPrint("UDP port = ");  OledPrintln(String(udp_localPort));

      #endif 
      
      wifiSuGood = true;
 
    }           

      
    #ifndef Start_WiFi  // if not button override
      delay(2000);      // debounce button press
    #endif  

    wifiSuDone = true;
 }   


  #if (WiFi_Protocol == 2)  //  Display the remote UDP IP the first time we get it
  void DisplayRemoteIP() {
    if (FtRemIP)  {
      FtRemIP = false;
      Debug.print("Client connected: Remote UDP IP: "); Debug.print(udp_remoteIP);
      Debug.print("  Remote  UDP port: "); Debug.println(udp_remotePort);
      OledPrintln("Client connected");
      OledPrintln("Remote UDP IP =");
      OledPrintln(udp_remoteIP.toString());
      OledPrintln("Remote UDP port =");
      OledPrintln(String(udp_remotePort));
     }
  }
  #endif
  
 #endif  


//***************************************************

#if ((FC_Mavlink_IO == 3) || defined GCS_Mavlink_SD)  // SD Card
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Debug.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Debug.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Debug.println("Not a directory");
        return;
    }
    
    File file = root.openNextFile();
    
    int i = 0;  
    while(file){
      if(file.isDirectory()){    
        Debug.print("  DIR : ");
        Debug.println(file.name());
        if(levels){
          listDir(fs, file.name(), levels -1);   //  Recursive :)
          }
      } else { 
        std::string myStr (file.name());  
        if (myStr.compare(0,14,"/System Volume") != 0)  {
      
          fnPath[i] = myStr;
        
          Debug.print("  FILE: "); Debug.print(i); Debug.print(" ");
          Debug.print(file.name());
          Debug.print("  SIZE: ");
          Debug.println(file.size());
          i++;    
        }
      }
      file = root.openNextFile();
    }
    fnCnt = i-1;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Initialising file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File initialised");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

//  Not used
void appendFile(fs::FS &fs, const char * path,  const char * message){
    Debug.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Debug.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Debug.println("Message appended");
    } else {
        Debug.println("Append failed");
    }
    file.close();
}

void deleteFile(fs::FS &fs, const char * path){
    Debug.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Debug.println("File deleted");
    } else {
        Debug.println("Delete failed");
    }
}

#endif
//***************************************************
 //***************************************************
 #if ((FC_Mavlink_IO == 3) || defined GCS_Mavlink_SD)  // SD Card
 void decomposeEpoch(uint32_t epch, DateTime_t &dt){

  uint8_t yr;
  uint8_t mth, mthDays;
  uint32_t w_epoch;
  unsigned long days;

  w_epoch = epch;
  dt.ss = w_epoch % 60;
  w_epoch /= 60;      // = mins
  dt.mm = w_epoch % 60;
  w_epoch /= 60;      // = hrs
  dt.hh = w_epoch % 24;
  w_epoch /= 24;      // = days
  dt.dow = ((w_epoch + 4) % 7) + 1;  // Sunday is day 1 
  
  yr = 0;  
  days = 0;
  while((unsigned)(days += (Leap_yr(yr) ? 366 : 365)) <= w_epoch) {
    yr++;
  }
  dt.yr = yr; // yr is offset from 1970 
  
  days -= Leap_yr(yr) ? 366 : 365;
  w_epoch  -= days; // now it is days in this yr, starting at 0
  
  days=0;
  mth=0;
  mthDays=0;
  for (mth=0; mth<12; mth++) {
    if (mth==1) { // february
      if (Leap_yr(yr)) {
        mthDays=29;
      } else {
        mthDays=28;
      }
    } else {
      mthDays = mthdays[mth];
    }
    
    if (w_epoch >= mthDays) {
      w_epoch -= mthDays;
    } else {
        break;
    }
  }
  dt.mth = mth + 1;  // jan is mth 1  
  dt.day = w_epoch + 1;     // day of mth
}
//****************************************************
bool Leap_yr(uint16_t y) {
return ((1970+y)>0) && !((1970+y)%4) && ( ((1970+y)%100) || !((1970+y)%400) );  
}
//***************************************************

String DateTimeString (DateTime_t &ep){
  String S = "";
  ep.yr += 1970;
  S += String(ep.yr);
  if (ep.mth<10) S += "0";
  S += String(ep.mth);
  if (ep.day<10) S += "0";
  S += String(ep.day);
  if (ep.hh<10) S += "0";
  S += String(ep.hh);
  if (ep.mm<10) S += "0";
  S += String(ep.mm);
  if (ep.ss<10) S += "0";
  S += String(ep.ss);
  return S;
}
#endif
//***************************************************
#if defined GCS_Mavlink_SD

void OpenSDForWrite() {
  
  //  deleteFile(SD, "/mav2passthu.tlog");
  
    uint32_t time_unix_sec = (ap_time_unix_usec/1E6) + (Time_Zone * 3600);   // add time zone adjustment decs
    if (daylightSaving) ap_time_unix_usec -= 3600;   // deduct an hour
    decomposeEpoch(time_unix_sec, dt_tm);

    String sPath = "/mav2pt"  + DateTimeString(dt_tm) + ".tlog";
    Debug.print("  Path: "); Serial.println(sPath); 

    strcpy(cPath, sPath.c_str());
    writeFile(SD, cPath , "Mavlink to FrSky Passthru by zs6buj");
    OledPrintln("Writing Tlog");
    sdStatus = 3;      
}
#endif
// *********************************************************************
void SenseWiFiPin() {
 #if ((FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3)) //  WiFi
 
   #if defined Start_WiFi
    if (!wifiSuDone) {
      SetupWiFi();
    }
    return;
  #endif
  
  WiFiPinState = digitalRead(startWiFiPin);
  if ((WiFiPinState == 0) && (!wifiSuDone)) {
    SetupWiFi();
    }
 #endif   
}

uint32_t Get_Volt_Average1(uint16_t mV)  {

  if (bat1.avg_mV < 1) bat1.avg_mV = mV;  // Initialise first time

 // bat1.avg_mV = (bat1.avg_mV * 0.9) + (mV * 0.1);  // moving average
  bat1.avg_mV = (bat1.avg_mV * 0.6666) + (mV * 0.3333);  // moving average
  Accum_Volts1(mV);  
  return bat1.avg_mV;
}
//***********************************************************************
uint32_t Get_Current_Average1(uint16_t dA)  {   // in 10*milliamperes (1 = 10 milliampere)
  
  Accum_mAh1(dA);  
  
  if (bat1.avg_dA < 1){
    bat1.avg_dA = dA;  // Initialise first time
  }

  bat1.avg_dA = (bat1.avg_dA * 0.6666) + (dA * 0.333);  // moving average

  return bat1.avg_dA;
  }

void Accum_Volts1(uint32_t mVlt) {    //  mV   milli-Volts
  bat1.tot_volts += (mVlt / 1000);    // Volts
  bat1.samples++;
}

void Accum_mAh1(uint32_t dAs) {        //  dA    10 = 1A
  if (bat1.ft) {
    bat1.prv_millis = millis() -1;   // prevent divide zero
    bat1.ft = false;
  }
  uint32_t period = millis() - bat1.prv_millis;
  bat1.prv_millis = millis();
    
  double hrs = (float)(period / 3600000.0f);  // ms to hours

  bat1.mAh = dAs * hrs;     //  Tiny dAh consumed this tiny period di/dt
 // bat1.mAh *= 100;        //  dA to mA  
  bat1.mAh *= 10;           //  dA to mA ?
  bat1.mAh *= 1.0625;       // Emirical adjustment Markus Greinwald 2019/05/21
  bat1.tot_mAh += bat1.mAh;   //   Add them all in
}

float Total_mAh1() {
  return bat1.tot_mAh;
}

float Total_mWh1() {                                     // Total energy consumed bat1
  return bat1.tot_mAh * (bat1.tot_volts / bat1.samples);
}
//***********************************************************
uint32_t Get_Volt_Average2(uint16_t mV)  {
  
  if (bat2.avg_mV == 0) bat2.avg_mV = mV;  // Initialise first time

  bat2.avg_mV = (bat2.avg_mV * 0.666) + (mV * 0.333);  // moving average
  Accum_Volts2(mV);  
  return bat2.avg_mV;
}
  
uint32_t Get_Current_Average2(uint16_t dA)  {

  if (bat2.avg_dA == 0) bat2.avg_dA = dA;  // Initialise first time

  bat2.avg_dA = (bat2.avg_dA * 0.666) + (dA * 0.333);  // moving average

  Accum_mAh2(dA);  
  return bat2.avg_dA;
  }

void Accum_Volts2(uint32_t mVlt) {    //  mV   milli-Volts
  bat2.tot_volts += (mVlt / 1000);    // Volts
  bat2.samples++;
}

void Accum_mAh2(uint32_t dAs) {        //  dA    10 = 1A
  if (bat2.ft) {
    bat2.prv_millis = millis() -1;   // prevent divide zero
    bat2.ft = false;
  }
  uint32_t period = millis() - bat2.prv_millis;
  bat2.prv_millis = millis();
    
 double hrs = (float)(period / 3600000.0f);  // ms to hours

  bat2.mAh = dAs * hrs;   //  Tiny dAh consumed this tiny period di/dt
 // bat2.mAh *= 100;        //  dA to mA  
  bat2.mAh *= 10;        //  dA to mA ?
  bat2.mAh *= 1.0625;       // Emirical adjustment Markus Greinwald 2019/05/21 
  bat2.tot_mAh += bat2.mAh;   //   Add them all in
}

float Total_mAh2() {
  return bat2.tot_mAh;
}

float Total_mWh2() {                                     // Total energy consumed bat1
  return bat2.tot_mAh * (bat2.tot_volts / bat2.samples);
}
//*********************************************************************************
uint32_t GetBaud(uint8_t rxPin) {
  Debug.printf("AutoBaud - Sensing FC_Mav_rxPin %2d \n", FC_Mav_rxPin );
  uint8_t i = 0;
  uint8_t col = 0;
  pinMode(rxPin, INPUT);       
  digitalWrite (rxPin, HIGH); // pull up enabled for noise reduction ?

  uint32_t gb_baud = GetConsistent(rxPin);
  while (gb_baud == 0) {
    if(ftGetBaud) {
      ftGetBaud = false;
    }

    i++;
    if ((i % 5) == 0) {
      Debug.print(".");
      col++; 
    }
    if (col > 60) {
      Debug.println(); 
      Debug.printf("No telemetry found on pin %2d\n", FC_Mav_rxPin); 
      col = 0;
      i = 0;
    }
    gb_baud = GetConsistent(rxPin);
  } 
  if (!ftGetBaud) {
    Debug.println();
  }

  Debug.print("Telem found at "); Debug.print(gb_baud);  Debug.println(" b/s");
  OledPrintln("Telem found at " + String(gb_baud));

  return(gb_baud);
}
//************************************************
uint32_t GetConsistent(uint8_t rxPin) {
  uint32_t t_baud[5];

  while (true) {  
    t_baud[0] = SenseUart(rxPin);
    delay(10);
    t_baud[1] = SenseUart(rxPin);
    delay(10);
    t_baud[2] = SenseUart(rxPin);
    delay(10);
    t_baud[3] = SenseUart(rxPin);
    delay(10);
    t_baud[4] = SenseUart(rxPin);
    #if defined Debug_All || defined Debug_Baud
      Debug.print("  t_baud[0]="); Debug.print(t_baud[0]);
      Debug.print("  t_baud[1]="); Debug.print(t_baud[1]);
      Debug.print("  t_baud[2]="); Debug.print(t_baud[2]);
      Debug.print("  t_baud[3]="); Debug.println(t_baud[3]);
    #endif  
    if (t_baud[0] == t_baud[1]) {
      if (t_baud[1] == t_baud[2]) {
        if (t_baud[2] == t_baud[3]) { 
          if (t_baud[3] == t_baud[4]) {   
            #if defined Debug_All || defined Debug_Baud    
              Debug.print("Consistent baud found="); Debug.println(t_baud[3]); 
            #endif   
            return t_baud[3]; 
          }          
        }
      }
    }
  }
}
//************************************************
uint32_t SenseUart(uint8_t  rxPin) {

uint32_t pw = 999999;  //  Pulse width in uS
uint32_t min_pw = 999999;
uint32_t su_baud = 0;
const uint32_t su_timeout = 5000; // uS !

#if defined Debug_All || defined Debug_Baud
  Debug.print("rxPin ");  Debug.println(rxPin);
#endif  

  while(digitalRead(rxPin) == 1){ }  // wait for low bit to start
  
  for (int i = 1; i <= 10; i++) {            // 1 start bit, 8 data and 1 stop bit
    pw = pulseIn(rxPin,LOW, su_timeout);     // default timeout 1000mS! Returns the length of the pulse in uS
    SenseWiFiPin();
    if (pw !=0) {
      min_pw = (pw < min_pw) ? pw : min_pw;  // Choose the lowest
    } else {
       return 0;  // timeout - no telemetry
    }
  }
 
  #if defined Debug_All || defined Debug_Baud
    Debug.print("pw="); Debug.print(pw); Debug.print("  min_pw="); Debug.println(min_pw);
  #endif

  switch(min_pw) {   
    case 1:     
     su_baud = 921600;
      break;
    case 2:     
     su_baud = 460800;
      break;     
    case 4 ... 11:     
     su_baud = 115200;
      break;
    case 12 ... 19:  
     su_baud = 57600;
      break;
     case 20 ... 28:  
     su_baud = 38400;
      break; 
    case 29 ... 39:  
     su_baud = 28800;
      break;
    case 40 ... 59:  
     su_baud = 19200;
      break;
    case 60 ... 79:  
     su_baud = 14400;
      break;
    case 80 ... 149:  
     su_baud = 9600;
      break;
    case 150 ... 299:  
     su_baud = 4800;
      break;
     case 300 ... 599:  
     su_baud = 2400;
      break;
     case 600 ... 1199:  
     su_baud = 1200;  
      break;                        
    default:  
     su_baud = 0;    // no signal        
 }

 return su_baud;
} 
//*************************************************************
