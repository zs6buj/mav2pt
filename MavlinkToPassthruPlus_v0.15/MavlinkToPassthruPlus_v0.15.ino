   
/*  *****************************************************************************

    MavToPassthruPlus  July 2018
 
    This program is free software. You may redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation. See here <http://www.gnu.org/licenses>

    The application was written in the hope that it will be useful, but it comes
    without any warranty or implied warranty of merchantability or fitness 
    for a particular purpose 
    
    *****************************************************************************

    Inspired by original S.Port firmware by Rolf Blomgren

    Author: Eric Stockenstrom

    Acknowledgements and thanks to Craft and Theory (http://www.craftandtheoryllc.com/) for
    the Mavlink / Frsky Passthrough protocol

    Thank you to yaapu for advice and testing, and his excellent LUA scrip

    Thank you athertop for advice and extensive testing
    
    Thank you florent for advice on working with my FlightDeck

    *****************************************************************************
    PLUS version adds additional sensor IDs to Mavlink Passthrough protocol DIY range

    Whereas the Orange (and some other) UHF Long Range RC and telemetry radio systems deliver 
    19.2kb/s two-way Mavlink link, the FrSky Taranis and Horus hand-held RC controllers expect
    to receive FrSky S.Port protocol telemetry for display on their screen.  While excellent 
    firmware is available to convert Mavlink to the native S.Port protocol, the author is 
    unaware of a suitable solution to convert to the Passthrough protocol. 

    Recently some excellent Lua scripts for Taranis displays, like this one by yaapu 
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

   Select the target mpu by un-commenting either //#define Target_Teensy3x or //#define Target_Blue_Pill or //#define Target_Maple_Mini

   Battery capacities in mAh can be 
   
   1 Requested from the flight controller via Mavlink
   2 Defined within this firmware  or 
   3 Defined within the LUA script on the Taranis/Horus. This is the prefered method as the Orange's
     RX (and Teensy TX) line is freed-up for use for BlueTooth or other Mavlink connections
     
   Please #define the appropriate Battery_mAh_Source below
   
   Connections to Teensy3.2 are:

    1) SPort S     -->TX1 Pin 1    S.Port out to Taranis bay, bottom pin
    2) Mavlink     <--RX2 Pin 9    Mavlink from Taranis to Teensy
    3) Mavlink     -->TX2 Pin 10   Mavlink from Teensy to Taranis
    4) Aux_Mavlink <--RX3 Pin 7    Auxiliary Mavlink From BT Module to Teensy
    5) Aux_Mavlink -->TX3 Pin 8    NOT NECESSARY - wire direct from Orange TX to BT RX  
    6) Vcc 3.3V !
    7) GND

   Connections to Blue Pill STM32F103C  are:

    1) SPort S     -->TX2 Pin A2   Serial1 to inverter, convert to single wire then to S.Port
    2) SPort S     <--RX2 Pin A3   Serial1 To inverter, convert to single wire then to S.Port
    3) Mavlink     -->TX3 Pin B10  Mavlink from STM32 to Taranis 
    4) Mavlink     <--RX3 Pin B11  Mavlink from Taranis to STM32  
    5) Aux_Mavlink    Not available   
    6) Aux_Mavlink    Not available
    7) Vcc 3.3V !
    8) GND

   Connections to Maple Mini STM32F103C are:

    1) SPort S     -->TX1 Pin A10   Serial1 to inverter, convert to single wire then to S.Port
    2) SPort S     <--RX1 Pin A9    Serial1 To inverter, convert to single wire then to S.Port
    3) Mavlink     -->TX2 Pin A2    Serial2 Mavlink from STM32 to Taranis
    4) Mavlink     <--RX2 Pin A3    Serial2 Mavlink from Taranis to STM32 
    5) Aux_Mavlink -->TX3 Pin B10   Serial3 NOT NECESSARY - wire direct from Orange TX to BT RX
    6) Aux_Mavlink <--RX3 Pin B11   Serial3 Auxiliary Mavlink From BT Module to Teensy  
    7) Vcc 3.3V !
    8) GND    
    
Change log:

v0.03 2018-07-11  Add sensor types 0x5009 RX Channels, 0x5010 VFR Hud 2018-07-17 board LED solid when mavGood
v0.04 2018-07-31  Add support for Maple Mini. Change rc channel 0x5009 as per yaapu's proposal  
v0.05 2018-08-02  Add circular buffers for mavlink incoming from FC
v0.06 2018-08-03  Fixed "#if defined Target_Teensy3x" not changed everywhere - Thanks Alex
                  Translate RC values : PWM 1000 to 2000 -> 6 bit 0-63 plus sign bit
v0.07             Do some tests                  
v0.08 2018-08-14  Servo outputs, not RC inputs 
v0.09 2018-09-20  Add support for PX4 flight stack. Specifically flight mode. 2018-09-24 Use di/dt for mAh. 
v0.10 2018-09-26  Fix bug in current consumption. Two options, from FC or accumulated di/dt. They now track each other.
v0.11 2018-08-30  PX4 new flight mode scheme. Clean up battery capacity source logic.
v0.12 2018-09-11  Add support for missions 
v0.13 2018-09-16  COG - Azimuth offset as per yaapu requirements 
v0.14 2018-09-17  Missions 0x5011 after TLog testing. 
v0.15 2018-09-18  prep_number function:   Included number encoded on 7 bits (6 bits + 1 for 10^power) 
                  Fixed 2 errors in 0x5011 bit32Pack() 

*/

#include <CircularBuffer.h>
#include <GCS_MAVLink.h>

//************************************* Please select your options here before compiling **************************
// Choose one (only) of these target boards
//#define Target_Board   0      // Teensy 3.x              Un-comment this line if you are using a Teensy 3.x
//#define Target_Board   1      // Blue Pill STM32F103C    OR un-comment this line if you are using a Blue Pill STM32F103C
#define Target_Board   2      // Maple_Mini STM32F103C   OR un-comment this line if you are using a Maple_Mini STM32F103C

// Choose one (only) of these three modes
#define Ground_Mode          // Converter between Taranis and LRS tranceiver (like Orange)
//#define Air_Mode             // Converter between FrSky receiver (like XRS) and Flight Controller (like Pixhawk)
//#define Relay_Mode           // Converter between LRS tranceiver (like Orange) and FrSky receiver (like XRS) in relay box on the ground

#define Battery_mAh_Source  1  // Get battery mAh from the FC - note both RX and TX lines must be connected      
//#define Battery_mAh_Source  2  // Define bat1_capacity and bat2_capacity below and use those 
//#define Battery_mAh_Source  3  // Define battery mAh in the LUA script on the Taranis/Horus - Recommended

const uint16_t bat1_capacity = 5200;       
const uint16_t bat2_capacity = 0;

#define SPort_Serial   1    // The default is Serial 1, but 3 is possible if we don't want aux port

//#define Aux_Port_Enabled            // For BlueTooth or other auxilliary serial passthrough
//#define Request_Missions_From_FC    // Un-comment if you need mission waypoint from FC - NOT NECESSARY NOW

//*** LEDS ********************************************************************************************************
//uint16_t MavStatusLed = 13; 
uint8_t MavLedState = LOW; 
uint16_t BufStatusLed = 12; 
uint8_t BufLedState = LOW; 

#if (Target_Board == 0) // Teensy3x
  #define MavStatusLed  13
#elif (Target_Board == 1) // Blue Pill
  #define MavStatusLed  PC13
#elif (Target_Board == 2) //  Maple Mini
  #define MavStatusLed  33  // PB1
#endif
//********************************************************* 
#if (Target_Board == 1) // Blue Pill
  #if defined Aux_Port_Enabled       
    #error Blue Pill board does not have enough UARTS for Auxilliary port. Un-comment #define Aux_Port_Enabled.
  #endif
#endif

#define Debug               Serial         // USB 
#define frBaud              57600          // Use 57600
#define mavSerial           Serial2        
#define mavBaud             57600   

#if (SPort_Serial == 1) 
  #define frSerial              Serial1        // S.Port 
  #elif (SPort_Serial == 3)
    #define frSerial            Serial3        // S.Port 
  #else
    #error SPort_Serial can only be 1 or 3. Please correct.
#endif  
   
#if defined Aux_Port_Enabled
  #if (SPort_Serial == 3) 
   #error Aux port and SPort both configured for Serial3. Please correct.
  #else 
    #define auxSerial             Serial3        // Mavlink telemetry to and from auxilliary adapter     
    #define auxBaud               57600          // Use 57600
    #define auxDuplex                            // Pass aux <-> FC traffic up and down, else only down from FC
  #endif
#endif

//#define Frs_Dummy_rssi       // For LRS testing only - force valid rssi. NOTE: If no rssi FlightDeck or other script won't connect!
//#define Data_Streams_Enabled // Rather set SRn in Mission Planner

#define Max_Waypoints  256     // Note. This is a RAM trade-off. If exceeded then Debug message and shut down

// Debugging options below ***************************************************************************************
//#define Mav_Debug_All
//#define Frs_Debug_All
//#define Frs_Debug_Payload
//#define Mav_Debug_RingBuff
//#define Debug_Air_Mode
//#define Mav_List_Params
//#define Aux_Debug_Params
//#define Aux_Port_Debug
//#define Mav_Debug_Params
//#define Frs_Debug_Params
//#define Mav_Debug_Servo
//#define Frs_Debug_Servo
//#define Mav_Debug_Rssi
//#define Mav_Debug_RC
//#define Frs_Debug_RC
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
//#define Mav_Debug_Hud
//#define Frs_Debug_Hud
//#define Mav_Debug_Scaled_Pressure
//#define Mav_Debug_Attitude
//#define Frs_Debug_Attitude
//#define Mav_Debug_Text
//#define Frs_Debug_Text    
//#define Mav_Debug_Mission 
#define Frs_Debug_Mission              
//*****************************************************************************************************************

uint8_t   buf[MAVLINK_MAX_PACKET_LEN];
uint16_t  hb_count=0;

bool      ap_bat_paramsReq = false;
bool      ap_bat_paramsRead=false; 
bool      parm_msg_shown = false;
bool      ap_paramsList=false;
uint8_t   paramsID=0;

bool      homGood=false;      
bool      mavGood=false;
bool      rssiGood=false;

uint32_t  hb_millis=0;
uint32_t  rds_millis=0;
uint32_t  acc_millis=0;
uint32_t  em_millis=0;
uint32_t  sp_millis=0;
uint32_t  mav_led_millis=0;

uint32_t  now_millis = 0;
uint32_t  prev_millis = 0;

uint32_t  lat800_millis = 0;
uint32_t  lon800_millis = 0;
uint32_t  ST5000_millis = 0;
uint32_t  AP5001_millis = 0;
uint32_t  GPS5002_millis = 0;
uint32_t  Bat1_5003_millis = 0;
uint32_t  Home_5004_millis = 0;
uint32_t  VelYaw5005_millis = 0;
uint32_t  Atti5006_millis = 0;
uint32_t  Param5007_millis = 0;
uint32_t  Bat2_5008_millis = 0;
uint32_t  Servo_5009_millis = 0; 
uint32_t  Hud_5010_millis = 0;
uint32_t  Miss_5011_millis = 0; 
uint32_t  rssi_F101_millis=0;

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


// ******************************************

mavlink_message_t msg;

uint16_t len;

// Mavlink Messages

// Mavlink Header
uint8_t    ap_sysid;
uint8_t    ap_compid;
uint8_t    ap_targcomp;
uint8_t    mvType;

// Message #0  HEARTHBEAT 
uint8_t    ap_type_tmp = 0;              // hold the type until we know HB not from GCS or Tracket
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 0;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;
bool       px4_flight_stack = false;
uint8_t    px4_main_mode = 0;
uint8_t    px4_sub_mode = 0;

// Message # 1  SYS_STATUS 
uint16_t   ap_voltage_battery1= 0;    // 1000 = 1V
int16_t    ap_current_battery1= 0;    //  10 = 1A
uint8_t    ap_ccell_count1= 0;


// Message #20 PARAM_REQUEST_READ
// ap_targsys  System ID
uint8_t  ap_targsys;     //   System ID
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
uint16_t ap_gps_hdg;           // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees

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

// Message #40 Mission_Request
//  Generic Mavlink Header defined above

// Message #42 Mission_Current
//  Generic Mavlink Header defined above
bool ap_ms_current_flag = false;

// Message #43 Mission_Request_list
//  Generic Mavlink Header defined above
bool ap_ms_list_req = false;

// Message #44 Mission_Count
//  Generic Mavlink Header defined above
uint8_t   ap_mission_count = 0;
bool      ap_ms_count_ft = true;

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
uint8_t  rssi;                   // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown

// Message #74 VFR_HUD  
float    ap_hud_air_spd;
float    ap_hud_grd_spd;
int16_t  ap_hud_hdg;
uint16_t ap_hud_throt;
float    ap_hud_bar_alt;   
float    ap_hud_climb;        

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
 char      ap_text[60];
 uint8_t   ap_txtlth;
 bool      ap_simple=0;
 

//***************************************************************
// FrSky Passthrough Variables
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
uint8_t  fr_chunk_pntr = 0;  // chunk pointer

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
bool     fr_paramsSent = false;

//0x5008 Batt
float fr_bat2_volts;
float fr_bat2_amps;
uint16_t fr_bat2_mAh;

//0x5009 Servo_raw         // 4 ch per frame
uint8_t  fr_port; 
int8_t   fr_sv[5];       

//0x5010 HUD
float    fr_air_spd;       // dm/s
uint16_t fr_throt;         // 0 to 100%
float    fr_bar_alt;       // metres

//0x5011 Missions       
uint16_t  fr_ms_seq;                // WP number
uint16_t  fr_ms_dist;               // To next WP  
float     fr_ms_xtrack;             // Cross track error in metres
float     fr_ms_target_bearing;     // Direction of next WP
float     fr_ms_cog;                // Course-over-ground in degrees
int8_t    fr_ms_offset;             // Next WP bearing offset from COG

//0xF103
uint32_t fr_rssi;


//****************** Ring Buffers *************************
typedef struct  {
  uint8_t   severity;
  char      text[50];
  uint8_t   txtlth;
  bool      simple;
  } ST_type;

ST_type ST_record;

CircularBuffer<ST_type, 10> MsgRingBuff; 

CircularBuffer<mavlink_message_t, 10> MavRingBuff; 

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
  
  delay(2500);
  Debug.println("Starting .... ");
  
  Debug.print("Target Board is ");
  #if (Target_Board == 0) // Teensy3x
  Debug.println("Teensy 3.x");
  #elif (Target_Board == 1) // Blue Pill
  Debug.println("Blue Pill STM32F103C");
  #elif (Target_Board == 2) //  Maple Mini
  Debug.println("Maple Mini STM32F103C");
  #endif

  #ifdef Ground_Mode
  Debug.println("Ground Mode");
  #endif
  #ifdef Air_Mode
  Debug.println("Air Mode");
  #endif
  #ifdef Relay_Mode
  Debug.println("Relay Mode");
  #endif

  #if defined Aux_Port_Enabled
  Debug.print("Auxilliary Mavlink port enabled");
    #ifdef auxDuplex
      Debug.println(" for two-way telemetry to/from Flight Control computer"); 
    #else 
      Debug.println(" for one-way telemetry down from Flight Control computer"); 
    #endif 
  #endif  
  
  #if (Battery_mAh_Source == 1)  
    Debug.println("Battery_mAh_Source = 1 - Get battery capacities from the FC");
  #elif (Battery_mAh_Source == 2)
    Debug.println("Battery_mAh_Source = 2 - Define battery capacities in this firmware");  
  #elif (Battery_mAh_Source == 3)
    Debug.println("Battery_mAh_Source = 3 - Define battery capacities in the LUA script"); 
  #else
    #error You must define at least one Battery_mAh_Source. Please correct.
  #endif            

  #if (SPort_Serial == 1) 
    Debug.println("Using Serial_1 for S.Port"); 
  #else
    Debug.println("Using Serial_3 for S.Port"); 
  #endif  

   pinMode(MavStatusLed, OUTPUT); 
   pinMode(BufStatusLed, OUTPUT); 
   
}

// ******************************************
// ******************************************
void loop()  {
  
  #ifdef Data_Streams_Enabled
  if(mavGood) {                      // If we have a link, request data streams from MavLink every 5s
    if(millis()-rds_millis > 5000) {
    rds_millis=millis();
    Debug.println("Requesting data streams");    
    RequestDataStreams();   // ensure Teensy Tx connected to Taranis RX  (When SRx not enumerated)
    }
  }
  #endif 

  #ifdef Request_Missions_From_FC
  if (mavGood) {
    if (!ap_ms_list_req) {
      RequestMissionList();  //  #43
      ap_ms_list_req = true;
    }
  }
  #endif

  if(mavGood && (millis() - hb_millis) > 3000)  {   // if no heartbeat from APM in 3s then assume mav not connected
    mavGood=false;
    Debug.println("Heartbeat timed out! Mavlink not connected");    
    hb_count = 0;
   } 

  #if (Battery_mAh_Source == 1)  // Get battery capacity from the FC
  // Request battery capacity params 
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
  #endif 
  
  #ifdef Mav_List_Params
    if(mavGood && (!ap_paramsList)) {
      Request_Param_List();
      ap_paramsList = true;
    }
  #endif 

  if(mavSerial.available()) QueueAvailableMavFrames(); // to the ring buffer

  DecodeOneMavFrame();                     // Decode a Mavlink frame from the ring buffer if there is one

  Aux_ReceiveAndForward();                 // Service aux incoming if enabled

  #ifdef Ground_Mode
  if(mavGood && ((millis() - em_millis) > 10)) {   
     Emulate_ReadSPort();                // Emulate the sensor IDs received from XRS receiver on SPort
     em_millis=millis();
    }
  #endif
     
  #if defined Air_Mode || defined Relay_Mode
  if(mavGood && ((millis() - sp_millis) > 10)) {   
     ReadSPort();                       // Receive round-robin of sensor IDs received from XRS receiver
     sp_millis=millis();
    }
  #endif   
  
  ServiceStatusLeds();
}
// ******************************************
//*******************************************

void QueueAvailableMavFrames() {
  mavlink_message_t ring_msg;
  mavlink_status_t status;
  while(mavSerial.available())             { 
    uint8_t c = mavSerial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &ring_msg, &status)) {
      if (MavRingBuff.isFull()) 
        Debug.println("MavRingBuff is full. Dropping records!");
       else {
        MavRingBuff.push(ring_msg);
        #if defined Mav_Debug_RingBuff
          Debug.print(" Mav queue length after push= "); 
          Debug.println(MavRingBuff.size());
        #endif
       }
    }
  }
}
//*******************************************
void DecodeOneMavFrame() { 

  if (!MavRingBuff.isEmpty()) {
    
    msg = (MavRingBuff.shift());  // Get a mavlink message from front of queue
    
    #if defined Mav_Debug_RingBuff
      Debug.print("Mavlink ring buffer msg: ");  
      PrintMavBuffer(&msg);
      Debug.print(" Mav queue length after shift= "); Debug.println(MavRingBuff.size());
    #endif

      
    #if defined  Aux_Port_Enabled 
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
          ap_type_tmp = mavlink_msg_heartbeat_get_type(&msg);   // Alex - don't contaminate the ap-type variable
          if (ap_type_tmp == 5 || ap_type_tmp == 6) break;      // Ignore heartbeats from GCS (6) or Ant Trackers(5)
          ap_type = ap_type_tmp;
          ap_autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
          ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
          ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
          
          px4_main_mode = bit32Extract(ap_custom_mode,16, 8);
          px4_sub_mode = bit32Extract(ap_custom_mode,24, 8);
          px4_flight_stack = (ap_autopilot == MAV_AUTOPILOT_PX4);
            
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
            Debug.print("  ap_mavlink_version="); Debug.print(ap_mavlink_version);   


            if (px4_flight_stack) {         
              Debug.print(" px4_main_mode="); Debug.print(px4_main_mode); 
              Debug.print(" px4_sub_mode="); Debug.print(px4_sub_mode);  
              Debug.print(" ");Debug.print(PX4FlightModeName(px4_main_mode, px4_sub_mode));  
           }
            
            Debug.println();
          #endif

          if(!mavGood) {
            hb_count++; 
            Debug.print("hb_count=");
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
            Debug.print("  Total mAh="); Debug.print(bat1.tot_mAh, 3);  // Consumed so far, calculated in Average module
         
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

          switch(ap_param_index) {      // if #define Battery_mAh_Source !=1 these will never arrive
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
          ap_gps_hdg = mavlink_msg_global_position_int_get_hdg(&msg);         // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees        
 
          cur.lat = (float)ap_lat / 1E7;
          cur.lon = (float)ap_lon / 1E7;
          cur.alt = ap_amsl33 / 1E3;
          cur.hdg = ap_gps_hdg / 100;

          #if defined Mav_Debug_All || defined Mav_Debug_GPS_Int
            Debug.print("Mavlink in #33 GPS Int: ");
            Debug.print(" ap_lat="); Debug.print((float)ap_lat / 1E7, 6);
            Debug.print(" ap_lon="); Debug.print((float)ap_lon / 1E7, 6);
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
          break; 
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW :          // #36
          if (!mavGood) break; 
      
          ap_port = mavlink_msg_servo_output_raw_get_port(&msg);
          ap_servo_raw[0] = mavlink_msg_servo_output_raw_get_servo1_raw(&msg);   
          ap_servo_raw[1] = mavlink_msg_servo_output_raw_get_servo2_raw(&msg);
          ap_servo_raw[2]= mavlink_msg_servo_output_raw_get_servo3_raw(&msg);   
          ap_servo_raw[3] = mavlink_msg_servo_output_raw_get_servo4_raw(&msg);  
          ap_servo_raw[4] = mavlink_msg_servo_output_raw_get_servo5_raw(&msg);   
          ap_servo_raw[5] = mavlink_msg_servo_output_raw_get_servo6_raw(&msg);
          ap_servo_raw[6] = mavlink_msg_servo_output_raw_get_servo7_raw(&msg);   
          ap_servo_raw[7] = mavlink_msg_servo_output_raw_get_servo8_raw(&msg); 
          /* 
           *  not supported right now
          ap_servo_raw[8] = mavlink_msg_servo_output_raw_get_servo9_raw(&msg);   
          ap_servo_raw[9] = mavlink_msg_servo_output_raw_get_servo10_raw(&msg);
          ap_servo_raw[10] = mavlink_msg_servo_output_raw_get_servo11_raw(&msg);   
          ap_servo_raw[11] = mavlink_msg_servo_output_raw_get_servo12_raw(&msg); 
          ap_servo_raw[12] = mavlink_msg_servo_output_raw_get_servo13_raw(&msg);   
          ap_servo_raw[13] = mavlink_msg_servo_output_raw_get_servo14_raw(&msg);
          ap_servo_raw[14] = mavlink_msg_servo_output_raw_get_servo15_raw(&msg);   
          ap_servo_raw[15] = mavlink_msg_servo_output_raw_get_servo16_raw(&msg);
          */       
          ap_servo_flag = true;                                  // tell fr routine we have a servo record
          #if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_Servo
            Debug.print("Mavlink in #36 servo_output: ");
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
          break;  
        case MAVLINK_MSG_ID_MISSION_ITEM :          // #39
          if (!mavGood) break;
            ap_ms_seq = mavlink_msg_mission_item_get_seq(&msg);
            ap_ms_frame = mavlink_msg_mission_item_get_frame(&msg);                // The coordinate system of the waypoint.
            ap_ms_command = mavlink_msg_mission_item_get_command(&msg);            // The scheduled action for the waypoint.
            ap_ms_current = mavlink_msg_mission_item_get_current(&msg);            // false:0, true:1
            ap_ms_autocontinue = mavlink_msg_mission_item_get_autocontinue(&msg);  //  Autocontinue to next waypoint
            ap_ms_param1 = mavlink_msg_mission_item_get_param1(&msg);              // PARAM1, see MAV_CMD enum
            ap_ms_param2 = mavlink_msg_mission_item_get_param2(&msg);              // PARAM2, see MAV_CMD enum
            ap_ms_param3 = mavlink_msg_mission_item_get_param3(&msg);              // PARAM3, see MAV_CMD enum
            ap_ms_param3 = mavlink_msg_mission_item_get_param4(&msg);              // PARAM4, see MAV_CMD enum
            ap_ms_x = mavlink_msg_mission_item_get_x(&msg);                        // PARAM5 / local: X coordinate, global: latitude
            ap_ms_y = mavlink_msg_mission_item_get_y(&msg);                        // PARAM6 / local: Y coordinate, global: longitude
            ap_ms_z = mavlink_msg_mission_item_get_z(&msg);                        // PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
                     
            #if defined Mav_Debug_All || defined Mav_Debug_Mission
              Debug.print("Mavlink in #39 Mission Item: ");
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
              Debug.println();    
            #endif
            
            if (ap_ms_seq > Max_Waypoints) {
              Debug.println(" Max Waypoints exceeded! Waypoint ignored.");
              break;
            }

             WP[ap_ms_seq-1].lat = ap_ms_x;     //  seq = 1 goes into slot [0]
             WP[ap_ms_seq-1].lon = ap_ms_y;
             
          break;                    
        case MAVLINK_MSG_ID_MISSION_CURRENT:         // #42 should come down regularly as part of EXTENDED_STATUS group
          if (!mavGood) break;   
            ap_ms_seq =  mavlink_msg_mission_current_get_seq(&msg);  
            
            #if defined Mav_Debug_All || defined Mav_Debug_Mission
              Debug.print("Mavlink in #42 Mission Current: ");
              Debug.print("ap_mission_current="); Debug.println(ap_ms_seq);   
            #endif 
              
            if (ap_ms_seq > 0) ap_ms_current_flag = true;     //  Ok to send passthru frames 
  
          break; 
        case MAVLINK_MSG_ID_MISSION_COUNT :          // #44   received back after #43 Mission_Request_List sent
        #ifdef Request_Missions_From_FC
          if (!mavGood) break;  
            ap_mission_count =  mavlink_msg_mission_count_get_count(&msg);  // Reports 1 too many, i.e. the next empty WP
            #if defined Mav_Debug_All || defined Mav_Debug_Mission
              Debug.print("Mavlink in #44 Mission Count: ");
              Debug.print("ap_mission_count="); Debug.println(ap_mission_count);   
            #endif
            if ((ap_mission_count > 0) && (ap_ms_count_ft)) {
              ap_ms_count_ft = false;
              RequestAllWaypoints(ap_mission_count);  // # multiple #40, then wait for them to arrive at #39
            }
          break; 
        #endif
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:   // #62
          if (!mavGood) break;    
            ap_nav_roll =  mavlink_msg_nav_controller_output_get_nav_roll(&msg);             // Current desired roll
            ap_nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);            // Current desired pitch
            ap_nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);        // Current desired heading
            ap_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);  // Bearing to current waypoint/target
            ap_wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);                // Distance to active waypoint
            ap_alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);            // Current altitude error
            ap_aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);          // Current airspeed error
            ap_xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msg);      // Current crosstrack error on x-y plane

            #if defined Mav_Debug_All || defined Mav_Debug_Mission
              Debug.print("Mavlink in #62 Mission Item: ");
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
             
          break;     
        case MAVLINK_MSG_ID_RC_CHANNELS:             // #65
          if (!mavGood) break; 
          rssiGood=true;               //  We have received at least one rssi packet from air mavlink   
          ap_chcnt = mavlink_msg_rc_channels_get_chancount(&msg);
          ap_chan_raw[0] = mavlink_msg_rc_channels_get_chan1_raw(&msg);   
          ap_chan_raw[1] = mavlink_msg_rc_channels_get_chan2_raw(&msg);
          ap_chan_raw[2]= mavlink_msg_rc_channels_get_chan3_raw(&msg);   
          ap_chan_raw[3] = mavlink_msg_rc_channels_get_chan4_raw(&msg);  
          ap_chan_raw[4] = mavlink_msg_rc_channels_get_chan5_raw(&msg);   
          ap_chan_raw[5] = mavlink_msg_rc_channels_get_chan6_raw(&msg);
          ap_chan_raw[6] = mavlink_msg_rc_channels_get_chan7_raw(&msg);   
          ap_chan_raw[7] = mavlink_msg_rc_channels_get_chan8_raw(&msg);  
          ap_chan_raw[8] = mavlink_msg_rc_channels_get_chan9_raw(&msg);   
          ap_chan_raw[9] = mavlink_msg_rc_channels_get_chan10_raw(&msg);
          ap_chan_raw[10] = mavlink_msg_rc_channels_get_chan11_raw(&msg);   
          ap_chan_raw[11] = mavlink_msg_rc_channels_get_chan12_raw(&msg); 
          ap_chan_raw[12] = mavlink_msg_rc_channels_get_chan13_raw(&msg);   
          ap_chan_raw[13] = mavlink_msg_rc_channels_get_chan14_raw(&msg);
          ap_chan_raw[14] = mavlink_msg_rc_channels_get_chan15_raw(&msg);   
          ap_chan_raw[15] = mavlink_msg_rc_channels_get_chan16_raw(&msg);
          ap_chan_raw[16] = mavlink_msg_rc_channels_get_chan17_raw(&msg);   
          ap_chan_raw[17] = mavlink_msg_rc_channels_get_chan18_raw(&msg);
          
          ap_rssi = mavlink_msg_rc_channels_get_rssi(&msg);   // Receive RSSI 0: 0%, 254: 100%, 255: invalid/unknown
          ap_rc_flag = true;                                  // tell fr routine we have an rc records
          #if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_RC
            Debug.print("Mavlink in #65 RC_Channels: ");
            Debug.print("ap_chcnt="); Debug.print(ap_chcnt); 
            Debug.print(" PWM: ");
            for (int i=0 ; i < ap_chcnt ; i++) {
              Debug.print(" "); 
              Debug.print(i+1);
              Debug.print("=");  
              Debug.print(ap_chan_raw[i]);   
            }                         
            Debug.print("  Receive RSSI=");  Debug.println(ap_rssi/ 2.54);        
          #endif             
          break;      
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:     // #66 - OUTGOING TO UAV
          if (!mavGood) break;       
          break;                             
        case MAVLINK_MSG_ID_VFR_HUD:                 //  #74
          if (!mavGood) break;      
          ap_hud_air_spd = mavlink_msg_vfr_hud_get_airspeed(&msg);
          ap_hud_grd_spd = mavlink_msg_vfr_hud_get_groundspeed(&msg);      //  in m/s
          ap_hud_hdg = mavlink_msg_vfr_hud_get_heading(&msg);              //  in degrees
          ap_hud_throt = mavlink_msg_vfr_hud_get_throttle(&msg);           //  integer percent
          ap_hud_bar_alt = mavlink_msg_vfr_hud_get_alt(&msg);              //  m
          ap_hud_climb = mavlink_msg_vfr_hud_get_climb(&msg);              //  m/s

          #if defined Mav_Debug_All || defined Mav_Debug_Hud
            Debug.print("Mavlink in #74 VFR_HUD: ");
            Debug.print("Airspeed= "); Debug.print(ap_hud_air_spd, 2);                    // m/s    
            Debug.print("  Groundspeed= "); Debug.print(ap_hud_grd_spd, 2);            // m/s
            Debug.print("  Heading= ");  Debug.print(ap_hud_hdg);                      // deg
            Debug.print("  Throttle %= ");  Debug.print(ap_hud_throt);                    // %
            Debug.print("  Baro alt= "); Debug.print(ap_hud_bar_alt, 0);   // m                  
            Debug.print("  Climb rate= "); Debug.println(ap_hud_climb);               // m/s
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
          ap_battery_id = mavlink_msg_battery_status_get_id(&msg);  
          ap_current_battery = mavlink_msg_battery_status_get_current_battery(&msg);      // in 10*milliamperes (1 = 10 milliampere)
          ap_current_consumed = mavlink_msg_battery_status_get_current_consumed(&msg);    // mAh
          ap_battery_remaining = mavlink_msg_battery_status_get_battery_remaining(&msg);  // (0%: 0, 100%: 100)  

          if (ap_battery_id == 0) {  // Battery 1
            fr_bat1_mAh = ap_current_consumed;                       
          } else if (ap_battery_id == 1) {  // Battery 2
              fr_bat2_mAh = ap_current_consumed;                              
          } 
             
          #if defined Mav_Debug_All || defined Debug_Batteries
            Debug.print("Mavlink in #147 Battery Status: ");
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

          for (int i=0; i<=len ; i++) {       // Get real len
            if ((ap_text[i]==32 || ap_text[i]==0) && (ap_text[i+1]==32 || ap_text[i+1]==0)) {      // find first consecutive double-space
              len=i;
              break;
            }
          }
          ap_text[len+1]=0x00;
          ap_text[len+2]=0x00;  // mark the end of text chunk +
          ap_text[len+3]=0x00;
          ap_text[len+4]=0x00;
          
          ap_txtlth = len + 1;

       //   fr_chunk_pntr = 0;
          
          if (strcmp (ap_text,"SIMPLE mode on") == 0)
            ap_simple = true;
          else if
              (strcmp (ap_text,"SIMPLE mode off") == 0)
                ap_simple = false;

          if (MsgRingBuff.isFull()) {
            Debug.println("MsgRingBuff is full!");
          } else {
            ST_record.severity = ap_severity;
            memcpy(ST_record.text, ap_text, ap_txtlth+ 4);   // length + rest of last chunk at least
       //     memcpy(&ST_record.text[0], &ap_text[0], ap_txtlth);
            ST_record.txtlth = ap_txtlth;
            ST_record.simple = ap_simple;
            MsgRingBuff.push(ST_record);
          }
          
          #if defined Mav_Debug_All || defined Mav_Debug_Text
            Debug.print("Mavlink in #253 Statustext pushed onto MsgRingBuff: ");
            Debug.print("Queue length= "); Debug.print(MsgRingBuff.size());
            Debug.print(" Msg lth="); Debug.print(len);
            Debug.print(" Severity="); Debug.print(ap_severity);
            Debug.print(" "); Debug.print(MavSeverity(ap_severity));
            Debug.print("  Text= ");  Debug.print(" |"); Debug.print(ap_text); Debug.print("| ");
            Debug.print("  ap_simple "); Debug.println(ap_simple);
          #endif
          break;                                      
        default:
          if (!mavGood) break;
          #ifdef Mav_Debug_All
            Debug.print("Mavlink in: ");
            Debug.print("Unknown Message ID #");
            Debug.print(msg.msgid);
            Debug.println(" Ignored"); 
          #endif

          break;
      }
    }
}

//***************************************************
void MarkHome()  {
  homGood = true;
  hom.lat = (float)ap_lat / 1E7;
  hom.lon = (float)ap_lon / 1E7;
  hom.alt = (float)ap_amsl24 / 1E3;
  hom.hdg = (float)ap_gps_hdg / 100;

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
  ap_sysid = 20;                        // ID 20 for this aircraft
  ap_compid = 1;                        //  autopilot1

  mavlink_msg_param_request_read_pack(ap_sysid, ap_compid, &msg,
                   ap_targsys, ap_targcomp, ap_param_id, param_index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);                 
 }

//***************************************************
 void Request_Param_List() {

  ap_sysid = 20;                        // ID 20 for this aircraft
  ap_compid = 1;                        //  autopilot1
  
  mavlink_msg_param_request_list_pack(ap_sysid,  ap_compid, &msg,
                    ap_targsys,  ap_targcomp);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len); 
                    
 }
//***************************************************
#ifdef Request_Missions_From_FC
void RequestMission(uint16_t ms_seq) {    //  #40
  ap_sysid = 0xFF;
  ap_compid = 0xBE;
  ap_targsys = 1;
  ap_targcomp = 1; 
  
  mavlink_msg_mission_request_pack(ap_sysid, ap_compid, &msg,
                               ap_targsys, ap_targcomp, ms_seq);

  len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf,len);
  #if defined Mav_Debug_All || defined Mav_Debug_Mission
    Debug.print("Mavlink out #40 Request Mission:  ms_seq="); Debug.println(ms_seq);
  #endif  
}
#endif 
 
//***************************************************
#ifdef Request_Missions_From_FC
void RequestMissionList() {   // #43   get back #44 Mission_Count
  ap_sysid = 0xFF;
  ap_compid = 0xBE;
  ap_targsys = 1;
  ap_targcomp = 1; 
 // ap_mission_type = 0;   // 0 = Items are mission commands for main mission
  
  mavlink_msg_mission_request_list_pack(ap_sysid, ap_compid, &msg,
                               ap_targsys, ap_targcomp);

  len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf,len);
  #if defined Mav_Debug_All || defined Mav_Debug_Mission
    Debug.println("Mavlink out #43 Request Mission List:");
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
  MAV_DATA_STREAM_EXTENDED_STATUS,
  MAV_DATA_STREAM_RC_CHANNELS,
  MAV_DATA_STREAM_POSITION,
  MAV_DATA_STREAM_EXTRA1, 
  MAV_DATA_STREAM_EXTRA2,
  MAV_DATA_STREAM_EXTRA3
  };

  const uint16_t mavRates[] = { 0x04, 0x0a, 0x04, 0x0a, 0x04, 0x04, 0x04};
 // req_message_rate The requested interval between two messages of this type

  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(ap_sysid, ap_compid, &msg,
        ap_targsys, ap_targcomp, mavStreams[i], mavRates[i], 1);    // start_stop 1 to start sending, 0 to stop sending   
  
    len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf,len);
    delay(10);
    }
 // Debug.println("Mavlink out #66 Request Data Streams:");
}
#endif

//***************************************************

void Aux_ReceiveAndForward() {   // up to FC, optional
  #if defined Aux_Port_Enabled && defined auxDuplex
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
void ServiceStatusLeds() {
  ServiceMavStatusLed();
  ServiceBufStatusLed();
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
  if (MsgRingBuff.isFull()) {
      BufLedState = HIGH;
  }
    else 
      BufLedState = LOW;
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



