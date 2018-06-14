  
/*  *****************************************************************************

    BETA
 
    This program is free software. You may redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation. See here <http://www.gnu.org/licenses>

    The application was written in the hope that it will be useful, but it comes
    without any warranty or implied warranty of merchantability or fitness 
    for a particular purpose 
    
    *****************************************************************************

    Inspired by original S.Port firmware by Rolf Blomgren

    Written from scratch by Eric Stockenstrom - April/May 2018

    Acknowledgements and thanks to Craft and Theory (http://www.craftandtheoryllc.com/) for
    the Mavlink / Frsky Passthrough protocol

    Thank you to yaapu for advice on working with his excellent LUA scrip

    Thank you florent for advice on working with my FlightDeck

    *****************************************************************************

    Whereas the Orange (and some other) UHF Long Range RC and telemetry radio systems deliver 
    19.2kb/s two-way Mavlink link, the FrSky Taranis and Horus hand-held RC controllers expect
    to receive FrSky S.Port protocol telemetry for display on their screen.  While excellent 
    firmware is available to convert Mavlink to the native S.Port protocol, the author is 
    unaware of a suitable solution to convert to the Passthrough protocol. 

    Recently some excellent Lua scripts for Taris displays, like this one by yaapu 
    https://github.com/yaapu/FrskyTelemetryScript 
    
    This firmware converts APM or PX4 Mavlink telemetry to FrSky SPort passthrough telemetry, 
    and is designed to run on a Teensy 3.2, or cheap STM32F103 (with some small mods and a signal 
    inverter). The Mavlink procol telemetry can still be fed on to Mission Planner or other GCSs.

    For now the Teensy 3.2 is prefered to the STM32 because of it's small size. It fits snugly 
    into the Orange LRS UHF RX/TX enclosure in the back bay of the Taranis, and requires no 
    external inverter.

   FrSky telemetry is unlike regular telemetry. It evolved from a simple system to poll sensors 
   on a 'plane, and as the number of sensors grew over time so too did the temetry requirements.
   Synchromous sensor polling is central to the telemetry, and timing is critical.

   On the other hand, most flight control computers manage internal and external sensors so 
   that the polling is handled internally. Telemetry is organised into meaningful records or 
   frames and sent asynchronously (whenever you like).

   Originally written for use with ULRS UHF, which delivers Mavlink to the back bay of the 
   Taranis X9D Plus to provide Frsky Passthrough compatible telemetry to yaapu's outstanding 
   LUA script.

   The converter can work in one of three modes: Ground_Mode, Air_Mode or Relay_Mode

   Ground_Mode
   In ground mode, it is located in the back of the Taranis. Since there is no FrSky receiver to 
   provide sensor polling, we create a routine in the firmware to emulate FrSky receiver sensor
   polling. (It pretends to be a receiver for polling purposes). 
   Un-comment this line       #define Ground_Mode      like this.

   Air_Mode
   In air mode, it is located on the aircraft between the FC and a Frsky receiver. It converts 
   Mavlink out of a Pixhawk and feeds passthru telemetry to the frsky receiver, which sends it 
   to Taranis on the ground. In this situation it responds to the FrSky receiver's sensor polling. 
   The APM firmware can deliver passthru telemetry, but the PX4 Pro firmware cannot. 
   Un-comment this line      #define Air_Mode    like this
   
   Relay_Mode
   Consider the situation where an air-side LRS UHF tranceiver (trx) (like the Orange), 
   communicates with a matching ground-side UHF trx located in a "relay" box using Mavlik 
   telemetry. The UHF trx in the relay box feeds Mavlink telemtry into our passthru converter, and 
   the converter feeds FrSky passthru telemtry into the FrSky receiver (like an XSR), also 
   located in the relay box. The XSR receiver (actually a tranceiver - trx) then communicates on 
   the public 2.4GHz band with the Taranis on the ground. In this situation the converter need not 
   emulate sensor polling, as the FrSky receiver will provide it. However, the converter must 
   determine the true rssi of the air link and forward it, as the rssi forwarded by the FrSky 
   receiver in the relay box will incorrectly be that of the short terrestrial link from the relay
   box to the Taranis.  To enable Relay_Mode :
   Un-comment this line      #define Relay_Mode    like this

   Select the target mpu by un-commenting either //#define Target_Teensy3x or //#define Target_STM32

   Connections to Teensy3.2 are:

    1) SPort S     -->TX1 Pin 1    S.Port out to Taranis bay, bottom pin
    2) Mavlink     <--RX2 Pin 9    Mavlink from Taranis to Teensy
    3) Mavlink     -->TX2 Pin 10   Mavlink from Teensy to Taranis
    4) Aux_Mavlink <--RX3 Pin 7    NOT NECESSARY - wire direct from Orange TX to BT RX  
    5) Aux_Mavlink -->TX3 Pin 8    Auxiliary Mavlink From Teensy to WiFi Module or general use
    6) Vcc 3.3V !
    7) GND

   Connections to STM32F103C are:

    1) SPort S     -->TX2 Pin A2   Serial1 to inverter, convert to single wire then to S.Port
    2) SPort S     <--RX2 Pin A3   Serial1 To inverter, convert to single wire then to S.Port
    3) Mavlink     <--RX3 Pin B11  Mavlink from Taranis to STM32 
    4) Mavlink     -->TX3 Pin B10  Mavlink from STM32 to Taranis
    5) Aux_Mavlink    Not available   
    6) Aux_Mavlink    Not available
    7) Vcc 3.3V !
    8) GND
    
Change log:
v0.18   2018-05-09  Establish "home" position when we get 3D+ fix (fixtype 4) rather than after 5 seconds of fixtype 3
v0.19   2018-05-12  Now works with FlightDeck. Changed 0x5007 param from once at start, to 0.2 Hz
v0.20   2018-05-13  Tidy up
v0.21   2018-05-15  Add byte-stuffing on outgoing frsky payload - thanks Alex
v0.22   2018-05-15  Make txsw_pin (6 on Teensy) HIGH after battery parameters safely read shortly after start - for CMOS switch
v0.23   2018-05-16  Include crc in byte-stuffing. Modify data stream request code slightly.
v0.24   2018-05-16  Single source code targets Teensy 3.x or STM32F103C depending on #defines
v0.25   2018-05-17  Make _txsw_pin (5 on Teensy) the inverse (inverted value) of txsw_pin. For 2nd bi-lateral cmos switch.
v0.26   2018-05-23  Pass Mavlink auxiliary telemetry through Serial3 - bi-directional
v0.27   2018-05-26  Rather use mAh from FC than my di/dt accumulation for bat1
v0.28   2018-05-30  Enable receiver (like XSR) SPort polling of SPort, make emulation optional with #define
v0.29   2018-06-01  As per yaapu: Respond only to sensor 0x1B, attitude 10Hz, rssi from FrSky receiver, not other
v0.30   2018-06-05  Improve the technical explanation of ground_mode, air_mode and polling emulation 
v0.31   2018-05-06  Change recommended wiring to eliminate receive latency
v0.32   2018-06-08  Fine tuned polling, much improved latency. Fixed bug in groundspeed, vertical speed.
v0.33   2018-06-08  Cleaned up STM32F103C8 build options and tested STM32 version
v0.34   2018-06-09  Fix bitfield overlap in groundspeed and yaw in 0x5005 
v0.35   2018-06-10  athertop - Use VFR_HUD data for vx and vy. Also fix 05004 fr_home_alt bit field, was one bit out
                    was bit32Pack(fr_home_alt ,13, 12), should be bit32Pack(fr_home_alt ,12, 12).
v0.36   2018-06-11  Fix fr_gps_status and advanced status. Do not send 0x5004 Home alt before homGood is true  
v0.37   2018-06-12  Introduce support for three modes, 1-Ground, 2-Air, 3-Relay     
v0.38   2018-06-13  #define auxDuplex, without which aux is simplex from BT to FC only
                    Determine home position only when motors are armed  
v0.39   2018-06-13  yaapu fix - ignore GCS heartbeat for armed status,  and timer start/stop. 
                    Fix GPS co-ords out to FrSky  
v0.40   2018-06-14  Use baro alt_ag from #33 for home alt (alt above home field)                    
*/

#include <GCS_MAVLink.h>

// Choose one (only) of these two target boards
//#define Target_STM32       // Un-comment this line if you are using an STM32F103C and an inverter+single wire
#define Target_Teensy3x      // OR  Un-comment this line if you are using a Teensy 3.x

#define Use_Serial1_For_SPort   // The default, else use Serial3

// Choose one (only) of these three modes
#define Ground_Mode          // Converter between Taranis and LRS tranceiver (like Orange)
//#define Air_Mode             // Converter between FrSky receiver lke XRS and Flight Controller like Pixhawk
//#define Relay_Mode           // Converter between LRS tranceiver (like Orange) and FrSky receiver (like XRS) in relay box on the ground

#if defined Ground_Mode && defined Target_Teensy3x && defined Use_Serial1_For_SPort
// #define Aux_Port_Enabled    // For BlueTooth or other auxilliary serial passthrough. 
#endif

#define Debug               Serial         // USB 

#define frBaud              57600          // Use 57600
#define mavSerial           Serial2        
#define mavBaud             57600   

#ifdef Use_Serial1_For_SPort     // The default
  #define frSerial            Serial1        // S.Port 
#else
  #define frSerial            Serial3        // S.Port 
#endif
  
#if defined Aux_Port_Enabled 
#define auxSerial           Serial3        // Mavlink telemetry to and from auxilliary adapter
#define auxBaud              57600         // Use 57600
//#define auxDuplex                          // Pass aux <-> FC traffic up and down, else only up to FC
#endif


//#define Frs_Dummy_rssi       // For LRS testing only - force valid rssi. NOTE: If no rssi FlightDeck or other script won't connect!
//#define Data_Streams_Enabled // Rather set SRn in Mission Planner

// Debugging options below
//#define Mav_List_Params
//#define Aux_Debug_All
//#define Aux_Debug_Params
//#define Frs_Debug_All
//#define Aux_Port_Debug
//#define Mav_Debug_Params
//#define Frs_Debug_Params
//#define Frs_Debug_Payload
//#define Mav_Debug_Rssi
//#define Mav_Debug_Heartbeat
//#define Mav_Debug_SysStatus
//#define Frs_Debug_LatLon
//#define Frs_Debug_APStatus
//#define Debug_Batteries
//#define Frs_Debug_Home
//#define Mav_Debug_GPS_Raw     // #24
//#define Mav_Debug_GPS_Int     // #33
//#define Frs_Debug_YelYaw
//#define Frs_Debug_GPS_Status
//#define Mav_Debug_Raw_IMU
//#define Mav_Debug_VFR_HUD
//#define Mav_Debug_Scaled_Pressure
//#define Mav_Debug_Attitude
//#define Frs_Debug_Attitude
#define Mav_Debug_Text
#define Frs_Debug_Text          

uint8_t StatusLed = 13; 
uint8_t ledState = LOW; 

uint8_t   buf[MAVLINK_MAX_PACKET_LEN];
uint16_t  hb_count=0;

bool      ap_bat_paramsReq = false;
bool      ap_bat_paramsRead=false; 
bool      parm_msg_shown = false;
bool      ap_paramsList=false;
bool      fr_paramsSent=false; 
uint8_t   paramsID=0;

bool      homGood=false;      
bool      mavGood=false;
bool      rssiGood=false;
bool      textFlag=false;

uint32_t  hb_millis=0;
uint32_t  rds_millis=0;
uint32_t  acc_millis=0;
uint32_t  em_millis=0;
uint32_t  sp_millis=0;
uint32_t  led_millis=0;

uint32_t  now_millis = 0;
uint32_t  prev_millis = 0;

uint32_t  lat800_millis = 0;
uint32_t  lon800_millis = 0;
uint32_t  AP5001_millis = 0;
uint32_t  GPS5002_millis = 0;
uint32_t  Bat1_5003_millis = 0;
uint32_t  Home_5004_millis = 0;
uint32_t  VelYaw5005_millis = 0;
uint32_t  Atti5006_millis = 0;
uint32_t  Param5007_millis = 0;
uint32_t  Bat2_5008_millis = 0;
uint32_t  rssi_F101_millis=0;

char sensID [10] = { 0x1B, 0x48, 0xBA,  0x98, 0xE9, 0x6A, 0xCB, 0xAC, 0x0D, 0x8E};  // Should be enough to achieve synchronisation
char prevByte;
uint8_t sensPtr=0;

mavlink_message_t msg;

float   lon1,lat1,lon2,lat2,alt1,alt2;  

// 3D Location vectors
struct Location {
  float lat; 
  float lon;
  float alt;
  float hdg;
  };
struct Location hom     = {
  0,0,0,0};   // home location

struct Location cur      = {
  0,0,0,0};   // current location  

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

// ******************************************
uint8_t len;
// Mavlink Messages

// Mavlink Header
uint8_t    system_id;
uint8_t    component_id;
uint8_t    target_component;
uint8_t    mvType;

// Message #0  HEARTHBEAT 
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 0;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

// Message # 1  SYS_STATUS 
uint16_t   ap_voltage_battery1= 0;    // 1000 = 1V
int16_t    ap_current_battery1= 0;    //  10 = 1A
uint8_t   ap_ccell_count1= 0;

// Message #20 PARAM_REQUEST_READ
// target_system  System ID
uint8_t  target_system;     //   System ID
char     req_param_id[16];  //  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
int16_t  req_param_index;  //  Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)

// Message #20 PARAM_REQUEST_READ 
//  Generic Mavlink Header defined above
// use #22 PARAM_VALUE variables below
// ap_param_index . Send -1 to use the param ID field as identifier (else the param id will be ignored)
float ap_bat1_capacity;
float ap_bat2_capacity;

// Message #21 PARAM_REQUEST_LIST 
//  Generic Mavlink Header defined above
  
// Message #22 PARAM_VALUE
char     ap_param_id [16]; 
float    ap_param_value;
uint8_t  ap_param_type;  
uint16_t ap_param_count;              //  Total number of onboard parameters
uint16_t ap_param_index;              //  Index of this onboard parameter

// Message #24  GPS_RAW_INT 
uint8_t    ap_fixtype = 3;            // 0= No GPS, 1=No Fix, 2=2D Fix, 3=3D Fix, 4=DGPS, 5=RTK_Float, 6=RTK_Fixed, 7=Static, 8=PPP
uint8_t    ap_sat_visible = 0;        // numbers of visible satelites
int32_t    ap_latitude = 0;           // 7 assumed decimal places
int32_t    ap_longitude = 0;          // 7 assumed decimal places
int32_t    ap_amsl24 = 0;             // 1000 = 1m
uint16_t   ap_eph;                    // GPS HDOP horizontal dilution of position (unitless)
uint16_t   ap_epv;                    // GPS VDOP vertical dilution of position (unitless)
uint16_t   ap_vel;                    // GPS ground speed (m/s * 100) cm/s
uint16_t   ap_cog;                    // Course over ground in degrees * 100, 0.0..359.99 degrees. 

// Message #27 RAW IMU 
int32_t   ap_accX = 0;
int32_t   ap_accY = 0;
int32_t   ap_accZ = 0;

// Message #29 SCALED_PRESSURE
uint32_t   ap_time_boot_ms;      // Timestamp (milliseconds since system boot)
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
int32_t ap_lat;            // Latitude, expressed as degrees * 1E7
int32_t ap_lon;            // Longitude, expressed as degrees * 1E7
int32_t ap_amsl33;         // Altitude above mean sea level (millimeters)
int32_t ap_alt_ag;         // Altitude above ground (millimeters)
int16_t ap_vx;             // Ground X Speed (Latitude, positive north), expressed as m/s * 100
int16_t ap_vy;             // Ground Y Speed (Longitude, positive east), expressed as m/s * 100
int16_t ap_vz;             // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
uint16_t ap_hdg;           // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees

// Message #65 RC_Channels
uint8_t  ap_chancount;  
uint16_t ap_chan3_raw;           // Throttle - just for reference
uint16_t ap_chan16_raw;          // Used for RSSI uS 1000=0%  2000=100%
uint8_t  rssi;                   // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown

// Message #74 VFR_HUD   // Not used at present
float    ap_airspeed;
float    ap_groundspeed;
int16_t  ap_heading;
uint16_t ap_throttle;
float    ap_bar_altitude;   
float    ap_climb_rate;        

// Message  #125 POWER_STATUS 
uint16_t  ap_Vcc;                 // 5V rail voltage in millivolts
uint16_t  ap_Vservo;              // servo rail voltage in millivolts
uint16_t  ap_flags;               // power supply status flags (see MAV_POWER_STATUS enum)
/*
 * MAV_POWER_STATUS
Power supply status flags (bitmask)
1   MAV_POWER_STATUS_BRICK_VALID  main brick power supply valid
2   MAV_POWER_STATUS_SERVO_VALID  main servo power supply valid for FMU
4   MAV_POWER_STATUS_USB_CONNECTED  USB power is connected
8   MAV_POWER_STATUS_PERIPH_OVERCURRENT peripheral supply is in over-current state
16  MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT hi-power peripheral supply is in over-current state
32  MAV_POWER_STATUS_CHANGED  Power status has changed since boot
 */

// Message  #147 BATTERY_STATUS 
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

// Message #166 RADIO
uint8_t ap_rssi;                // local signal strength
uint8_t ap_remrssi;             // remote signal strength
uint8_t ap_txbuf;               // how full the tx buffer is as a percentage
uint8_t ap_noise;               // background noise level
uint8_t ap_remnoise;            // remote background noise level
uint16_t ap_rxerrors;           // receive errors
uint16_t ap_fixed;              // count of error corrected packets

// Message #181 BATTERY2 
uint16_t   ap_voltage_battery2 = 0;    // 1000 = 1V
int16_t    ap_current_battery2 = 0;    //  10 = 1A
uint8_t    ap_cell_count2 = 0;

// Message #253 STATUSTEXT
uint8_t   ap_severity;
char      ap_text[50];
char      prev_ap_text[50];
uint8_t   p_text = 0;  // pointer
uint8_t   ap_simple=0;



// FrSky Passthrough Variables
uint32_t  fr_payload;

// 0x800 GPS
uint8_t ms2bits;
uint32_t fr_lat = 0;
uint32_t fr_lon = 0;

// 0x5000 Text Msg
uint8_t txtlth;
uint32_t fr_textmsg;
char ct[5];
char p_ct[5];
int ct_dups=0;
uint8_t fr_severity;
char fr_text[30];
boolean eot=false;

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
uint32_t fr_home_angle;
int32_t fr_home_alt;

short fr_pwr;

// 0x5005 Velocity and yaw
uint32_t fr_velyaw;
float fr_vy;
float fr_vx;
float fr_yaw;

// 0x5006 Attitude and range
uint16_t fr_roll;
uint16_t fr_pitch;
uint16_t fr_range;

// 0x5007 Parameters  - Sent 3x each at init
uint8_t fr_param_id ;
uint32_t fr_param_val;
uint32_t fr_frame_type;
//uint32_t fr_fs_bat_volts;
//uint32_t fr_fs_bat_mAh;
uint32_t fr_bat1_capacity;
uint32_t fr_bat2_capacity;

//0x5008 Batt
float fr_bat2_volts;
float fr_bat2_amps;
uint16_t fr_bat2_mAh;

//0xF103
uint32_t fr_rssi;

// ******************************************
void setup()  {
  
  FrSkySPort_Init();
  mavSerial.begin(mavBaud);
  
  #if defined  Aux_Port_Enabled 
  auxSerial.begin(auxBaud);
  #endif
  
  Debug.begin(115200);

  mavGood = false;
  homGood = false;     
  hb_count = 0;
  hb_millis=millis();
  acc_millis=millis();
  rds_millis=millis();
  em_millis=millis();
  
  delay(1000);
  Debug.println("Starting.....");

}

// ******************************************
// ******************************************
void loop()  {
  
#ifdef Data_Streams_Enabled
  if(mavGood) {                      // If we have a link request data streams from MavLink every 5s
    if(millis()-rds_millis > 5000) {
    rds_millis=millis();
    Debug.println("Requesting data streams");    
    RequestDataStreams();   // ensure Teensy Tx connected to Taranis RX  (When SRx not enumerated)
    }
  }
#endif 

  if(mavGood && (millis() - hb_millis) > 3000)  {   // if no heartbeat from APM in 3s then assume mav not connected
    mavGood=false;
    Debug.println("Heartbeat timed out! Mavlink not connected");    
    hb_count = 0;
   } 

  // Request battery capacity params, and when they have safely arrived switch txsw_pin (6) high 
  if (mavGood) {
    if (!ap_bat_paramsReq) {
      Request_Param_Read(356);    // Request Bat1 capacity   do this twice in case of lost frame
      Request_Param_Read(356);    
      Request_Param_Read(364);    // Request Bat2 capacity
      Request_Param_Read(364);    
      Debug.println("Battery capacities requested");
      ap_bat_paramsReq = true;
    } else {
      if (ap_bat_paramsRead &&  (!parm_msg_shown)) {
        parm_msg_shown = true; 
        Debug.println("Battery params successfully read"); 
      }
    } 
  }
  #ifdef Mav_List_Params
    if(mavGood && (!ap_paramsList)) {
      Request_Param_List();
      ap_paramsList = true;
    }
  #endif 

  MavLink_Receive();                      // Get Mavlink Data

  Aux_ReceiveAndForward();                 // Service aux incoming if enabled

  #ifdef Ground_Mode
  if(mavGood && ((millis() - em_millis) > 10)) {   
     Emulate_ReadSPort();                // Emulate the sensor IDs received from XRS receiver on SPort
     em_millis=millis();
    }
  #endif
     
  #if defined Air_Mode_Mode || defined Relay_Mode
  if(mavGood && ((millis() - sp_millis) > 10)) {   
     ReadSPort();                       // Receive round-robin of sensor IDs received from XRS receiver
     sp_millis=millis();
    }
  #endif   
  ServiceTheStatusLed();
}
// ******************************************
//*******************************************

void MavLink_Receive() { 
  mavlink_message_t msg;
  mavlink_status_t status;

  while(mavSerial.available()) 
                { 
    uint8_t c = mavSerial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
       
      //   PrintMavBuffer(&msg);
      
      #if defined  Aux_Port_Enabled && defined auxDuplex
      len = mavlink_msg_to_send_buffer(buf, &msg);
      #ifdef  Aux_Port_Debug
        Debug.println("auxSerial passed down from FC:");
        PrintMavBuffer(&msg);
      #endif
      auxSerial.write(buf,len);
      #endif
     
    // Debug.print(" msgid="); Debug.println(msg.msgid); 

      switch(msg.msgid) {
    
        case MAVLINK_MSG_ID_HEARTBEAT:    // #0   http://mavlink.org/messages/common
          ap_type = mavlink_msg_heartbeat_get_type(&msg);
          ap_autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
          ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
          ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
          ap_system_status = mavlink_msg_heartbeat_get_system_status(&msg);
          ap_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&msg);
          hb_millis=millis(); 

          if ((ap_base_mode >> 7) && (!homGood)) 
            MarkHome();  // If motors armed for the first time, then mark this spot as home

          #if defined Mav_Debug_All || defined Mav_Debug_Heartbeat
            Debug.print("Mavlink in #0 Heartbeat: ");           
            Debug.print("ap_type="); Debug.print(ap_type);   
            Debug.print("  ap_autopilot="); Debug.print(ap_autopilot); 
            Debug.print("  ap_base_mode="); Debug.print(ap_base_mode); 
            Debug.print(" ap_custom_mode="); Debug.print(ap_custom_mode);   
            Debug.print("  ap_system_status="); Debug.print(ap_system_status); 
            Debug.print("  ap_mavlink_version="); Debug.println(ap_mavlink_version);
          #endif

          if(!mavGood) {
            hb_count++; 
            Debug.print(" hb_count=");
            Debug.print(hb_count);
            Debug.println("");

            if(hb_count >= 3) {        // If  3 heartbeats from MavLink then we are connected
              mavGood=true;
              Debug.println("mavgood=true");  
              hb_count=0;
              }
          }
          break;
        case MAVLINK_MSG_ID_SYS_STATUS:   // #1
          if (!mavGood) break;
          ap_voltage_battery1= Get_Volt_Average1(mavlink_msg_sys_status_get_voltage_battery(&msg));        // 1000 = 1V  i.e mV
          ap_current_battery1= Get_Current_Average1(mavlink_msg_sys_status_get_current_battery(&msg));     //  100 = 1A, i.e dA
          if(ap_voltage_battery1> 21000) ap_ccell_count1= 6;
            else if (ap_voltage_battery1> 16800 && ap_ccell_count1!= 6) ap_ccell_count1= 5;
            else if(ap_voltage_battery1> 12600 && ap_ccell_count1!= 5) ap_ccell_count1= 4;
            else if(ap_voltage_battery1> 8400 && ap_ccell_count1!= 4) ap_ccell_count1= 3;
            else if(ap_voltage_battery1> 4200 && ap_ccell_count1!= 3) ap_ccell_count1= 2;
            else ap_ccell_count1= 0;
          #if defined Mav_Debug_All || defined Mav_Debug_SysStatus
            Debug.print("Mavlink in #1 Sys_Status: ");        
            Debug.print(" Bat volts=");
            Debug.print((float)ap_voltage_battery1/ 1000, 3);   // now V
            Debug.print("  Bat amps=");
            Debug.print((float)ap_current_battery1/ 100, 1);   // now A
              
            Debug.print("  mAh="); Debug.print(bat1.mAh, 6);    
            Debug.print("  Total mAh="); Debug.print(bat1.tot_mAh, 3);
         
            Debug.print("  Bat1 cell count= "); 
            Debug.println(ap_ccell_count1);
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
          len=mavlink_msg_param_value_get_param_id(&msg, ap_param_id);
          ap_param_value=mavlink_msg_param_value_get_param_value(&msg);
          ap_param_count=mavlink_msg_param_value_get_param_count(&msg);
          ap_param_index=mavlink_msg_param_value_get_param_index(&msg); 

          switch(ap_param_index) {  
            case 356:         // Bat1 Capacity
              ap_bat1_capacity = ap_param_value;
              #if defined Mav_Debug_All || defined Debug_Batteries
                Debug.print("Mavlink in #22 Param_Value: ");
                Debug.print("bat1 capacity=");
                Debug.println(ap_bat1_capacity);
              #endif
              break;
            case 364:         // Bat2 Capacity
              ap_bat2_capacity = ap_param_value;
              ap_bat_paramsRead = true;
              #if defined Mav_Debug_All || defined Debug_Batteries
                Debug.print("Mavlink in #22 Param_Value: ");
                Debug.print("bat2 capacity=");
                Debug.println(ap_bat2_capacity);
              #endif             
              break;
          } 
             
          #if defined Mav_Debug_All || defined Mav_Debug_Params
            Debug.print("Mavlink in #22 Param_Value: ");
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
          ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                   // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
          ap_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);    // number of visible satellites
          if(ap_fixtype > 2)  {
            ap_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
            ap_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
            ap_amsl24 = mavlink_msg_gps_raw_int_get_alt(&msg);             // 1m =1000 
            ap_eph = mavlink_msg_gps_raw_int_get_eph(&msg);                // GPS HDOP 
            ap_epv = mavlink_msg_gps_raw_int_get_epv(&msg);                // GPS VDOP 
            ap_vel = mavlink_msg_gps_raw_int_get_vel(&msg);                // GPS ground speed (m/s * 100)
            ap_cog = mavlink_msg_gps_raw_int_get_cog(&msg);                // Course over ground (NOT heading) in degrees * 100
          }
          #if defined Mav_Debug_All || defined Mav_Debug_GPS_Raw
            Debug.print("Mavlink in #24 GPS_RAW_INT: ");  
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
            Debug.print("  latitude="); Debug.print((float)(ap_latitude)/1E7, 7);
            Debug.print("  longitude="); Debug.print((float)(ap_longitude)/1E7, 7);
            Debug.print("  gps alt amsl="); Debug.print((float)(ap_amsl24)/1E3, 1);
            Debug.print("  eph (hdop)="); Debug.print(ap_eph, 1);               // HDOP
            Debug.print("  epv (vdop)="); Debug.print(ap_epv, 1);
            Debug.print("  vel="); Debug.print((float)ap_vel / 100, 3);         // GPS ground speed (m/s)
            Debug.print("  cog="); Debug.println((float)ap_cog / 100, 1);       // Course over ground in degrees
          #endif     
          break;
        case MAVLINK_MSG_ID_RAW_IMU:   // #27
          if (!mavGood) break;        
          ap_accX = mavlink_msg_raw_imu_get_xacc(&msg);                 
          ap_accY = mavlink_msg_raw_imu_get_yacc(&msg);
          ap_accZ = mavlink_msg_raw_imu_get_zacc(&msg);
          #if defined Mav_Debug_All || defined Mav_Debug_Raw_IMU
            Debug.print("Mavlink in #27 Raw_IMU: ");
            Debug.print("accX="); Debug.print((float)ap_accX / 1000); 
            Debug.print("  accY="); Debug.print((float)ap_accY / 1000); 
            Debug.print("  accZ="); Debug.println((float)ap_accZ / 1000);
          #endif     
          break;      
        case MAVLINK_MSG_ID_SCALED_PRESSURE:         // #29
          if (!mavGood) break;        
          ap_press_abs = mavlink_msg_scaled_pressure_get_press_abs(&msg);
          ap_temperature = mavlink_msg_scaled_pressure_get_temperature(&msg);
          #if defined Mav_Debug_All || defined Mav_Debug_Scaled_Pressure
            Debug.print("Mavlink in #29 Scaled_Pressure: ");
            Debug.print("  press_abs=");  Debug.print(ap_press_abs,1);
            Debug.print("hPa  press_diff="); Debug.print(ap_press_diff, 3);
            Debug.print("hPa  temperature=");  Debug.print((float)(ap_temperature)/100, 1); 
            Debug.println("C");             
          #endif             
          break;     
        case MAVLINK_MSG_ID_ATTITUDE:                // #30
          if (!mavGood) break;   

          ap_roll = mavlink_msg_attitude_get_roll(&msg);              // Roll angle (rad, -pi..+pi)
          ap_pitch = mavlink_msg_attitude_get_pitch(&msg);            // Pitch angle (rad, -pi..+pi)
          ap_yaw = mavlink_msg_attitude_get_yaw(&msg);                // Yaw angle (rad, -pi..+pi)
          ap_rollspeed = mavlink_msg_attitude_get_rollspeed(&msg);    // Roll angular speed (rad/s)
          ap_pitchspeed = mavlink_msg_attitude_get_pitchspeed(&msg);  // Pitch angular speed (rad/s)
          ap_yawspeed = mavlink_msg_attitude_get_yawspeed(&msg);      // Yaw angular speed (rad/s)           

          ap_roll = RadToDeg(ap_roll);   // Now degrees
          ap_pitch = RadToDeg(ap_pitch);
          ap_yaw = RadToDeg(ap_yaw);

          #if defined Mav_Debug_All || defined Mav_Debug_Attitude   
            Debug.print("Mavlink in #30 Attitude: ");      
            Debug.print(" ap_roll degs=");
            Debug.print(ap_roll, 1);
            Debug.print(" ap_pitch degs=");   
            Debug.print(ap_pitch, 1);
            Debug.print(" ap_yaw degs=");         
            Debug.println(ap_yaw, 1);
          #endif             

          break;  
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:     // #33
          if ((!mavGood) || (ap_fixtype < 3)) break;  
          ap_lat = mavlink_msg_global_position_int_get_lat(&msg);             // Latitude, expressed as degrees * 1E7
          ap_lon = mavlink_msg_global_position_int_get_lon(&msg);             // Pitch angle (rad, -pi..+pi)
          ap_amsl33 = mavlink_msg_global_position_int_get_alt(&msg);          // Altitude above mean sea level (millimeters)
          ap_alt_ag = mavlink_msg_global_position_int_get_relative_alt(&msg); // Altitude above ground (millimeters)
          ap_vx = mavlink_msg_global_position_int_get_vx(&msg);               //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
          ap_vy = mavlink_msg_global_position_int_get_vy(&msg);               //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
          ap_vz = mavlink_msg_global_position_int_get_vz(&msg);               // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
          ap_hdg = mavlink_msg_global_position_int_get_hdg(&msg);             // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees        
 
          cur.lat =  (float)ap_lat / 1E7;
          cur.lon = (float)ap_lon / 1E7;
          cur.alt = ap_amsl33 / 1E3;
          cur.hdg = ap_hdg / 100;

          #if defined Mav_Debug_All || defined Mav_Debug_GPS_Int
            Debug.print("Mavlink in #33 GPS Int: ");
            Debug.print(" ap_lat="); Debug.print((float)ap_lat / 1E7, 6);
            Debug.print(" ap_lon="); Debug.print((float)ap_lon / 1E7, 6);
            Debug.print(" ap_amsl="); Debug.print((float)ap_amsl33 / 1E3, 0);
            Debug.print(" ap_alt_ag="); Debug.print((float)ap_alt_ag / 1E3, 1);           
            Debug.print(" ap_vx="); Debug.print((float)ap_vx / 100, 2);
            Debug.print(" ap_vy="); Debug.print((float)ap_vy / 100, 2);
            Debug.print(" ap_vz="); Debug.print((float)ap_vz / 100, 2);
            Debug.print(" ap_hdg="); Debug.println((float)ap_hdg / 100, 1);
          #endif  
                
          break;  
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:         // #35
          if (!mavGood) break;        
          break; 
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:        // #36
          if (!mavGood) break;        
          break;   
        case MAVLINK_MSG_ID_MISSION_CURRENT:         // #42
          if (!mavGood) break;       
          break; 
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:   // #62
          if (!mavGood) break;       
          break;     
        case MAVLINK_MSG_ID_RC_CHANNELS:             // #65
          if (!mavGood) break; 
          rssiGood=true;               //  We have received at least one rssi packet from air mavlink   
          ap_chancount = mavlink_msg_rc_channels_get_chancount(&msg);
          ap_chan3_raw = mavlink_msg_rc_channels_get_chan3_raw(&msg);   
          ap_chan16_raw = mavlink_msg_rc_channels_get_chan16_raw(&msg);
          ap_rssi = mavlink_msg_rc_channels_get_rssi(&msg);   // Receive RSSI 0: 0%, 254: 100%, 255: invalid/unknown
          
          #if defined Mav_Debug_All || defined Mav_Debug_Rssi
            Debug.print("Mavlink in #65 RC_Channels: ");
            Debug.print("Channel count= "); Debug.print(ap_chancount); 
            Debug.print("  Channel 3= ");  Debug.print(ap_chan3_raw);            
            Debug.print("  Channel 16= ");  Debug.print(ap_chan16_raw);       
            Debug.print("  Receive RSSI=");  Debug.println(ap_rssi/ 2.54);        
          #endif             
          break;      
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:     // #66 - OUTGOING TO UAV
          if (!mavGood) break;       
          break;                             
        case MAVLINK_MSG_ID_VFR_HUD:                 //  #74
          if (!mavGood) break;      
          ap_airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
          ap_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);      //  in m/s
          ap_heading = mavlink_msg_vfr_hud_get_heading(&msg);              //  in degrees
          ap_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);            //  integer percent
          ap_bar_altitude = mavlink_msg_vfr_hud_get_alt(&msg);             //  m
          ap_climb_rate=mavlink_msg_vfr_hud_get_climb(&msg);               //  m/s

          #if defined Mav_Debug_All || defined Mav_Debug_VFR_HUD
            Debug.print("Mavlink in #74 VFR_HUD: ");
            Debug.print("Airspeed= "); Debug.print(ap_airspeed, 2);                    // m/s    
            Debug.print("  Groundspeed= "); Debug.print(ap_groundspeed, 2);            // m/s
            Debug.print("  Heading= ");  Debug.print(ap_heading);                      // deg
            Debug.print("  Throttle= ");  Debug.print(ap_throttle);                    // %
            Debug.print("  Barometric altitude= "); Debug.print(ap_bar_altitude);      // m                  
            Debug.print("  Climb rate= "); Debug.println(ap_climb_rate);               // m/s
          #endif  
          break; 
        case MAVLINK_MSG_ID_SCALED_IMU2:       // #116   http://mavlink.org/messages/common
          if (!mavGood) break;       
          break; 
        case MAVLINK_MSG_ID_POWER_STATUS:      // #125   http://mavlink.org/messages/common
          if (!mavGood) break;  
          ap_Vcc = mavlink_msg_power_status_get_Vcc(&msg);         // 5V rail voltage in millivolts
          ap_Vservo = mavlink_msg_power_status_get_Vservo(&msg);   // servo rail voltage in millivolts
          ap_flags = mavlink_msg_power_status_get_flags(&msg);     // power supply status flags (see MAV_POWER_STATUS enum)
          #ifdef Mav_Debug_All
            Debug.print("Mavlink in #125 Power Status: ");
            Debug.print("Vcc= "); Debug.print(ap_Vcc); 
            Debug.print("  Vservo= ");  Debug.print(ap_Vservo);       
            Debug.print("  flags= ");  Debug.println(ap_flags);       
          #endif  
          break; 
        case MAVLINK_MSG_ID_BATTERY_STATUS:      // #147   http://mavlink.org/messages/common
          if (!mavGood) break;         
          ap_current_battery = mavlink_msg_battery_status_get_current_battery(&msg);      // in 10*milliamperes (1 = 10 milliampere)
          ap_current_consumed = mavlink_msg_battery_status_get_current_consumed(&msg);    // mAh
          ap_battery_remaining = mavlink_msg_battery_status_get_battery_remaining(&msg);  // (0%: 0, 100%: 100)              
          #if defined Mav_Debug_All || defined Debug_Batteries
            Debug.print("Mavlink in #147 Battery Status: ");
            Debug.print(" bat current= "); Debug.print(ap_current_battery); 
            Debug.print(" bat mAh= ");  Debug.print(ap_current_consumed);       
            Debug.print(" bat % remaining= ");  Debug.println(ap_time_remaining);       
          #endif  
          break;    
        case MAVLINK_MSG_ID_SENSOR_OFFSETS:    // #150   http://mavlink.org/messages/ardupilotmega
          if (!mavGood) break;        
          break; 
        case MAVLINK_MSG_ID_MEMINFO:           // #152   http://mavlink.org/messages/ardupilotmega
          if (!mavGood) break;        
          break;   
        case MAVLINK_MSG_ID_RADIO:             // #166   http://mavlink.org/messages/ardupilotmega
          if (!mavGood) break;
          ap_rssi = mavlink_msg_radio_get_rssi(&msg);            // local signal strength
          ap_remrssi = mavlink_msg_radio_get_remrssi(&msg);      // remote signal strength
          ap_txbuf = mavlink_msg_radio_get_txbuf(&msg);          // how full the tx buffer is as a percentage
          ap_noise = mavlink_msg_radio_get_noise(&msg);          // remote background noise level
          ap_remnoise = mavlink_msg_radio_get_remnoise(&msg);    // receive errors
          ap_rxerrors = mavlink_msg_radio_get_rxerrors(&msg);    // count of error corrected packets
          ap_fixed = mavlink_msg_radio_get_fixed(&msg);    
         #ifdef Mav_Debug_All
            Debug.print("Mavlink in #166 Radio: "); 
            Debug.print("rssi="); Debug.print(ap_rssi);
            Debug.print("remrssi="); Debug.print(ap_remrssi);
            Debug.print("txbuf="); Debug.print(ap_txbuf);
            Debug.print("noise="); Debug.print(ap_noise); 
            Debug.print("remnoise="); Debug.print(ap_remnoise);
            Debug.print("rxerrors="); Debug.print(ap_rxerrors);
            Debug.print("fixed="); Debug.println(ap_fixed);                                
         #endif        
          break; 
        case MAVLINK_MSG_ID_AHRS2:             // #178   http://mavlink.org/messages/ardupilotmega
          if (!mavGood) break;       
          break;  
        case MAVLINK_MSG_ID_BATTERY2:          // #181   http://mavlink.org/messages/ardupilotmega
          if (!mavGood) break;
          ap_voltage_battery2 = Get_Volt_Average2(mavlink_msg_battery2_get_voltage(&msg));        // 1000 = 1V
          ap_current_battery2 = Get_Current_Average2(mavlink_msg_battery2_get_current_battery(&msg));     //  100 = 1A
          if(ap_voltage_battery2 > 21000) ap_cell_count2 = 6;
            else if (ap_voltage_battery2 > 16800 && ap_cell_count2 != 6) ap_cell_count2 = 5;
            else if(ap_voltage_battery2 > 12600 && ap_cell_count2 != 5) ap_cell_count2 = 4;
            else if(ap_voltage_battery2 > 8400 && ap_cell_count2 != 4) ap_cell_count2 = 3;
            else if(ap_voltage_battery2 > 4200 && ap_cell_count2 != 3) ap_cell_count2 = 2;
            else ap_cell_count2 = 0;
          #if defined Mav_Debug_All || defined Mav_Debug_Batteriestery2
            Debug.print("Mavlink in #181 Battery2: ");        
            Debug.print(" Bat volts=");
            Debug.print((float)ap_voltage_battery2 / 1000, 3);   // now V
            Debug.print("  Bat amps=");
            Debug.print((float)ap_current_battery2 / 100, 1);   // now A
              
            Debug.print("  mAh="); Debug.print(bat2.mAh, 6);    
            Debug.print("  Total mAh="); Debug.print(bat2.tot_mAh, 3);
         
            Debug.print("  Bat cell count= "); 
            Debug.println(ap_cell_count2);
          #endif
          break;
          
        case MAVLINK_MSG_ID_AHRS3:             // #182   http://mavlink.org/messages/ardupilotmega
          if (!mavGood) break;       
          break;
        case MAVLINK_MSG_ID_STATUSTEXT:        // #253      
          ap_severity = mavlink_msg_statustext_get_severity(&msg);
          len=mavlink_msg_statustext_get_text(&msg, ap_text);

    //      if (*prev_ap_text==*ap_text) break;  // Ignore duplicate text messages
    //      *prev_ap_text=*ap_text;

          for (int i=0; i<=len ; i++) {       // Get real len
            if ((ap_text[i]==32 || ap_text[i]==0) && (ap_text[i+1]==32 || ap_text[i+1]==0)) {      // find first consecutive double-space
              len=i;
              break;
            }
          }
          ap_text[len+1]=0x00;
          textFlag = true;
          p_text = 0;
          
          if (strcmp (ap_text,"SIMPLE mode on") == 0)
            ap_simple = 1;
          else if
              (strcmp (ap_text,"SIMPLE mode off") == 0)
                ap_simple = 0;

          #if defined Mav_Debug_All || defined Mav_Debug_Text
            Debug.print("Mavlink in #253 Statustext: ");
            Debug.print("length="); Debug.print(len);
            Debug.print(" Severity="); Debug.print(ap_severity);
            Debug.print(" "); Debug.print(MavSeverity(ap_severity));
            Debug.print("  Text= "); Debug.print(ap_text);
            Debug.print("  ap_simple "); Debug.println(ap_simple);
          #endif
          break;                                      
        default:
          if (!mavGood) break;
          #ifdef Mav_Debug_All
          //  Debug.print("Mavlink in: ");
          //  Debug.print("Unknown Message ID #");
          //  Debug.print(msg.msgid);
           // Debug.println(" Ignored"); 
          #endif

          break;
      }
    }
  }
}

//***************************************************
void MarkHome()  {
  homGood = true;
  hom.lat = (float)ap_lat / 1E7;
  hom.lon = (float)ap_lon / 1E7;
  hom.alt = (float)ap_amsl24 / 1E3;
  hom.hdg = (float)ap_hdg / 100;

  #if defined Mav_Debug_All || defined Mav_Debug_GPS_Int
    Debug.print("******************************************Mavlink in #33 GPS Int: Home established: ");       
    Debug.print("hom.lat=");  Debug.print(hom.lat, 7);
    Debug.print(" hom.lon=");  Debug.print(hom.lon, 7 );        
    Debug.print(" hom.alt="); Debug.print(hom.alt, 1);
    Debug.print(" hom.hdg="); Debug.println(hom.hdg);                   
 #endif  
}
//***************************************************
 void Request_Param_Read(int16_t param_index) {
  system_id = 20;                          // ID 20 for this aircraft
  component_id = 1;                        //  autopilot1

  mavlink_msg_param_request_read_pack(system_id, component_id, &msg,
                   target_system, target_component, ap_param_id, param_index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);                 
 }

//***************************************************
 void Request_Param_List() {

  system_id = 20;                          // ID 20 for this aircraft
  component_id = 1;                        //  autopilot1
  
  mavlink_msg_param_request_list_pack(system_id,  component_id, &msg,
                    target_system,  target_component);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len); 
                    
 }
//***************************************************
#ifdef Data_Streams_Enabled    
void RequestDataStreams() {    //  REQUEST_DATA_STREAM ( #66 ) DEPRECATED. USE SRx, SET_MESSAGE_INTERVAL INSTEAD
uint16_t len;
const uint8_t mavSysid=0xFF;
const uint8_t mavCompid=0xBE;
const uint8_t mavSys = 1;
const uint8_t mavComp = 1;

const int maxStreams = 7;
const uint8_t mavStreams[] = {
MAV_DATA_STREAM_RAW_SENSORS,
MAV_DATA_STREAM_EXTENDED_STATUS,
MAV_DATA_STREAM_RC_CHANNELS,
MAV_DATA_STREAM_POSITION,
MAV_DATA_STREAM_EXTRA1, 
MAV_DATA_STREAM_EXTRA2,
MAV_DATA_STREAM_EXTRA3
};
//const uint16_t mavRates[] = { 0x02, 0x05, 0x02, 0x05, 0x02, 0x02};
const uint16_t mavRates[] = { 0x04, 0x0a, 0x04, 0x0a, 0x04, 0x04 0x04};
 // req_message_rate The requested interval between two messages of this type

  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(mavSysid, mavCompid, &msg,
        mavSys, mavComp, mavStreams[i], mavRates[i], 1);    // start_stop 1 to start sending, 0 to stop sending   
  
    len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf,len);
    delay(10);
    }
 // Debug.println("Request Data Streams xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
}
#endif

//***************************************************

void Aux_ReceiveAndForward() { 
  #if defined Aux_Port_Enabled 
  mavlink_message_t msg; 
  mavlink_status_t status;

  while(auxSerial.available()) 
                { 
    uint8_t c = auxSerial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      
      #ifdef  Aux_Port_Debug
      Debug.println("auxSerial passed up to FC:");
      PrintMavBuffer(&msg);
      #endif
      
      len = mavlink_msg_to_send_buffer(buf, &msg);
      mavSerial.write(buf,len);
    }
   }
#endif    
}
 
//***************************************************
void ServiceTheStatusLed() {
  if (mavGood) {
      ledState = HIGH;
  }
    else 
      BlinkLed(500);

    digitalWrite(StatusLed, ledState);  
}
//***************************************************
void BlinkLed(uint32_t period) {
  uint32_t cMillis = millis();
     if (cMillis - led_millis >= period) {    // blink period
        led_millis = cMillis;
        if (ledState == LOW) {
          ledState = HIGH; }   
        else {
          ledState = LOW;  } 
      }
}

//***************************************************
void DisplayByte(byte b) {
  if (b<=0xf) Debug.print("0");
  Debug.print(b,HEX);
  Debug.print(" ");
}
//***************************************************

void PrintMavBuffer (const void *object){

  /*
  bytes[0]    // CRC1  
  bytes[1]    // CRC2      
  bytes[2]    // Magic    
  bytes[3]    // Payload length     
  bytes[4]    // Packet sequence  
  bytes[5]    // System ID      
  bytes[6]    // Component ID
  bytes[7]    // Message ID
*/

  byte b; 
  int lth;

  const unsigned char * const bytes = static_cast<const unsigned char *>(object);

  /*
  pSeq=b=bytes[4];
  if (prevSeq==255) prevSeq= -1; // Cater for roll over
  while (pSeq>(prevSeq+1)) {     // This should be next in sequence
    Debug.print("Frame    ");
    Debug.print(prevSeq+1);
    Debug.print("   missing   ");
    Debug.println("---------------------------------------------------");
    prevSeq++;
  }
  prevSeq=pSeq; 
  */
  
  Debug.print("CRC=");
  DisplayByte(bytes[0]);
  DisplayByte(bytes[1]);
  Debug.print("  ");
  
  b= bytes[3];
  lth=b+8;                  // total length  = payload lengthj + crc + header
  Debug.print("Pay_lth=");
  Debug.print(b);
  Debug.print("\t");
  
  b= bytes[4];
  Debug.print("Seq_byte=");
  Debug.print(b);
  Debug.print("\t");
  
  b= bytes[7];
  Debug.print(" Msg_ID=#");
  Debug.print(b);
  Debug.print("\t(0x");
  Debug.print(b,HEX);
  Debug.print(")\t");
  
  for ( int i = 2; i < lth; i++ ) {
    DisplayByte(bytes[i]);
    if(i==7) Debug.print("  ");      // Print space after header
  }
  Debug.println();
}
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
void ShowPeriod() {
  Debug.print("Period ms=");
  now_millis=millis();
  Debug.print(now_millis-prev_millis);
  Debug.print("\t");
  prev_millis=now_millis;
}
//***************************************************


