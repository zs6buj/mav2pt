//================================================================================================= 
//================================================================================================= 
//
//                                    C O N F I G U R A T I O N 
// 

#define MAJOR_VERSION      2
#define MINOR_VERSION      65
#define PATCH_LEVEL        3
/*
=================================================================================================== 
                                M o s t    R e c e n t   C h a n g e s
=================================================================================================== 

Complete change log and debugging options are at the bottom of this tab

GitHub Tag
----------                                            
 
V2.64.8    2021-04-07  Fixed AP mode web setting trying STA mode first
V2.64.9    2021-04-07  Fixed pure AP mode UDP object / no port swap
V2.65.0    2021-05-11  PR merged from Alex (yaapu)
                          added RPM frame 0x500A for rpm1 and rpm2
                          added TERRAIN frame 0x500B for terrain enabled/unhealthy
                          added FENCE status bits to frame 0x5001
                          added THROTTLE to frame 0x5001
V2.65.1   2021-05-13   PR merged from Alex (yaapu)
                          fix throttle scale from 0,100 to [-63,63]
V2.65.2   2021-05-17   Beta folder only. Bytestuff enable Write_Crc() 
                       Delay 15ms 0x5000 status_text chunks > 1                           
V2.65.3   2021-05-18   PR merged from Alex
                       SPort loop period from 18mS to 24mS
                       Work around apparent bit32Pack() anomaly.                         
                          
                                                                           
*/
//===========================================================================================
//
//                      PLEASE SELECT YOUR DEFAULT OPTIONS BELOW BEFORE COMPILING
//
//              Most of the settings below are saved to EEPROM the first time mav2pt runs
//              Use the web interface to change them, or Reset_Web_Defaults below
//
//===========================================================================================

                                       // Most of the settings below are saved to EEPROM the first time mav2pt runs
//#define Reset_Web_Defaults           // Reset settings in eeprom. Do this if you suspect eeprom settings are corrupt USE SPARINGLY

// Choose only one setting for FrSky Port Type 
//#define FrSky_Port_Type 0   // No FrSky Port support needed. Now I'm a "Mavlink Switch"
//#define FrSky_Port_Type 1   // F.Port v1
//#define FrSky_Port_Type 2   // F.Port v2 FrSky ISRM/ACCESS capable transmitters and receivers only
#define FrSky_Port_Type 3   // S.Port / legacy
//#define FrSky_Port_Type 4   // Auto detect


#define Device_sysid     251                     // Our Mavlink Identity - APM FC is 1, Mission Planner is 255, QGC default is 0 
#define Device_compid    MAV_COMP_ID_PERIPHERAL  // 158 Generic autopilot peripheral - APM FC is 1, MP is 190, QGC is  https://mavlink.io/en/messages/common.html

#define webSupport                     // ESP only. Enable wifi web support, including OTA firmware updating. Browse to IP.
#define webPassword      "admin"       // Web password 

#define displaySupport                 // Enable if you have a display attached - choose display type where board variant is defined 
                                       // NOTE: Set mvBaud = 57600 for Dragonlink and RFD900X
#define mvBaud            57600        // Mavlink to/from the flight controller - max 921600 - must match FC or long range radio
//#define MavAutoBaud                    // Auto detect Mavlink telemetry speed             
                                       // Default for FrSky is auto detect telemetry speed    
// Do not enable for FlightDeck
#define PlusVersion                    // Added support for 0x5009 Mission WPs, 0x50F1 Servo_Channels, 0x50F2 VFR_Hud

//=================================================================================================
//           D E F A U L T   T R A N S L A T I O N   M O D E   S E T T I N G S   
//=================================================================================================       
// Choose only one of these three translation modes
#define Ground_Mode          // Translator between Taranis et al and LRS transceiver (like Dragonlink, ULRS, RFD900...)
//#define Air_Mode             // Translator between FrSky receiver (like XRS) and Flight Controller (like Pixhawk)
//#define Relay_Mode           // Translator between LRS tranceiver (like Dragonlink) and FrSky receiver (like XRS) in relay box on the ground


//=================================================================================================
//                 D E F A U L T   U P L I N K   ( F C )   S E T T I N G S   
//=================================================================================================
// Choose only one of these default Flight-Controller-side I/O channels 
// How does Mavlink telemetry enter this translator?
#define FC_Mavlink_IO  0    // Serial Port (default)         
//#define FC_Mavlink_IO  1    // BlueTooth Classic - ESP32 only
//#define FC_Mavlink_IO  2    // WiFi - ESP32 or ESP8266 only
//#define FC_Mavlink_IO  3    // SD Card / TF - ESP32 only


//=================================================================================================
//                  D E F A U L T   D O W N L I N K   ( G C S )   S E T T I N G S    
//=================================================================================================
// Choose only one of these default GCS-side I/O channels
// How does Mavlink telemetry leave this translator?
// These are optional, and in addition to the F.Port telemetry output
//#define GCS_Mavlink_IO  0    // Serial Port - simultaneous uplink and downlink serial not supported. Not enough uarts.   
#define GCS_Mavlink_IO  1    // BlueTooth Classic - ESP32 only
//#define GCS_Mavlink_IO  2    // WiFi - ESP32 or ESP8266 only - auto selects on ESP8266
//#define GCS_Mavlink_IO  3    // WiFi AND Bluetooth simultaneously. DON'T DO THIS UNLESS YOU NEED IT. SRAM is scarce! - ESP32 only

//#define GCS_Mavlink_SD       // SD Card - ESP32 only - mutually inclusive with GCS I/O

#ifndef GCS_Mavlink_IO
  #define GCS_Mavlink_IO  9    // NONE (default)
#endif

//=================================================================================================
//                D E F A U L T   F R S K Y   S/F P O R T   I / O    S E T T I N G S    
//=================================================================================================
// Choose only one of these default FrSky S/Port I/O channels
// How does FrSky telemetry leave this translator? 
// Select one option or combination only

//#define FrSky_IO     0     // None  - Mav2PT becomes a Mavlink Switch
#define FrSky_IO     1     // Serial  
//#define FrSky_IO     2     // UDP  
//#define FrSky_IO     3     // Serial & UDP
//#define FrSky_IO     4     // SD Card
//#define FrSky_IO     5     // Serial & SD Card
//#define FrSky_IO     6     // UDP & SD Card
//#define FrSky_IO     7     // Serial & UDP & SD Card
#if (not defined FrSky_IO) 
  #define FrSky_IO  0 
#endif




// NOTE: The Bluetooth class library uses a lot of SRAM application memory. During Compile/Flash
//       you may need to select Tools/Partition Scheme: "Minimal SPIFFS (1.9MB APP ...) or similar


//================================================================================================= 
//=================================================================================================                             
//                          S E L E C T   E S P   B O A R D   V A R I A N T   
//=================================================================================================
//================================================================================================= 
//#define ESP32_Variant     1    //  ESP32 Dev Board - Use Partition Scheme: "Minimal SPIFFS(1.9MB APP...)"
//#define ESP32_Variant     2    //  Wemos® LOLIN ESP32-WROOM-32_OLED_Dual_26p
//#define ESP32_Variant     3    //  Dragonlink V3 slim with internal ESP32 - contributed by Noircogi - Select ESP32 Dev Board in IDE
#define ESP32_Variant     4    //  Heltec Wifi Kit 32 - Use Partition Scheme: "Minimal SPIFFS(Large APPS with OTA)" - contributed by Noircogi select Heltec wifi kit
//#define ESP32_Variant     5    //  LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD (135 x 240) - Select TTGO_T1 in IDE
//#define ESP32_Variant     6    //  LILYGO® TTGO T2 SD SSD1331 TFT Colour 26pin - 16Ch x 8 lines (96 x 64)- Select ESP32 Dev Board in IDE
//#define ESP32_Variant     7    // ESP32 Dev Board with ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  select Dev Board in IDE

//#define ESP8266_Variant   1   // NodeMCU ESP 12F - choose "NodeMCU 1.0(ESP-12E)" board in the IDE
#define ESP8266_Variant   2   // ESP-12E, ESP-F barebones boards. RFD900X TX-MOD, QLRS et al - use Generic ESP8266 on IDE
//#define ESP8266_Variant   3   // ESP-12F - Wemos® LOLIN D1 Mini

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 



//                      D E F A U L T   B L U E T O O T H   S E T T I N G S   
//=================================================================================================
//#define BT_Mode  1           // Master Mode - active, initiate connection with slave (name)
#define BT_Mode  2           // Slave Mode - passive, advertise our hostname & wait for master to connect to us
#define BT_ConnectToSlave     "Crossfire 0277"  // Example

//=================================================================================================
//                            D E F A U L T   W I F I   S E T T I N G S   
//=================================================================================================

#define Start_WiFi                              // Start WiFi at startup, override startWiFi pin

#define HostName             "MavToPass"        // This translator's host name
#define APssid               "MavToPassthru"    // The AP SSID that we advertise         ====>
#define APpw                 "password"         // Change me! Must be >= 8 chars
#define APchannel            9                  // The wifi channel to use for our AP
#define STAssid              "OmegaOffice"      // Target AP to connect to (in STA mode) <====
#define STApw                "password"         // Target AP password (in STA mode). Must be >= 8 chars      

// Choose one default mode for ESP only - AP means advertise as an access point (hotspot). STA means connect to a known host
#define WiFi_Mode   1  //AP
//#define WiFi_Mode   2  // STA
//#define WiFi_Mode   3  // (STA>AP) STA failover to AP

// Choose one default protocol - for ESP32 only
//#define Mav_WiFi_Protocol 1    // TCP/IP
#define Mav_WiFi_Protocol 2    // UDP 

//#define UDP_Broadcast      // Comment out (default) if you want to track and target remote udp client ips
// NOTE; UDP is not a connection based protocol. To communicate with > 1 client at a time, we must broadcast on the subnet  
//=================================================================================================
//                            R  S  S  I    O  P  T  I  O  N  S  
//=================================================================================================

#define Rssi_Pacemaker   200  // mS. RSSI 0xF101 frame sent with this period regardless of rate of RSSI arrival
#define RSSI_Override     70  // Only if (RSSI == 0) force it to value, but only sent when Mavlink good.)                                                                                                            
//#define Rssi_In_Percent     // Un-comment if RSSI is already in %, not relative to (254->100%)
//#define QLRS                // QLRS Longe Range System uses remote rssi field - PR from giocomo892 april 2020)

// RSSI_Source is automatic. Order of precedence: 
//      First:  #109 SiK style RADIO_STATUS
//      Second: #65 RC_CHANNELS
//      Third:  #35 RC_CHANNELS_RAW

//=================================================================================================
//                            O T H E R   U S E R   O P T I O N S  
//=================================================================================================

//defined PitLab                      // Uncomment me to force PitLab OSD stack

//#define Battery_mAh_Source  1       // Get battery mAh from the FC - note both rx and tx lines must be connected      
//#define Battery_mAh_Source  2       // Define bat1_capacity and bat2_capacity below and use those 
const uint16_t bat1_capacity = 5200;       
const uint16_t bat2_capacity = 0;
#define Battery_mAh_Source  3         // Define battery mAh in the LUA script on the Taranis/Horus - Recommended



// Status_Text messages place a huge burden on the meagre 4 byte FrSky telemetry payload bandwidth
// The practice has been to send them 3 times to ensure that they arrive unscathed at the receiver
//  but that makes the bandwidth limitation worse and may crowd out other message types. Try without
//  sending 3 times, but if status_text gets distorted, un-comment the next line
//#define Send_status_Text_3_Times

//#define Send_Sensor_Health_Messages

//#define Request_Missions_From_FC    // Un-comment if you need mission waypoint from FC - NOT NECESSARY RIGHT NOW

//#define Data_Streams_Enabled        // Requests data streams from FC. Requires both rx and tx lines to FC. Rather set SRn in Mission Planner

//================================== Set your time zone here ======================================
// Only for SD / TF Card adapter option
// Date and time determines the TLog file name only
//const float Time_Zone = 10.5;    // Adelaide, Australia
const float Time_Zone = 2.0;    // Jo'burg
bool daylightSaving = false;

//=================================================================================================
//                        E X P E R I M E N T A L    O P T I O N S  
//    Don't change anything here unless you are confident you know the outcome

//#define ESP32_SoftwareSerial            // otherwise HardwareSerial is used 
//#define ESP_Air_Relay_Blind_Inject   // Blind inject instead of interleaving

//#define Support_MavLite

//=================================================================================================   
//                              Auto Determine Target Platform
//================================================================================================= 
//
//                Don't change anything here
//
#if defined (__MK20DX128__) || defined(__MK20DX256__)
  #define TEENSY3X   
      
#elif defined ESP32
  #define Target_Board   3      // Espressif ESP32 Dev Module

#elif defined ESP8266
  #define Target_Board   4      // Espressif ESP8266
  
#else
  #error "Unsupported board type!"
#endif

//=================================================================================================   
//                              CHECK #MACRO OPTIONS LOGIC
//================================================================================================= 

#if defined PlusVersion
  #define Request_Mission_Count_From_FC // Needed for yaapu's mission/waypoint script
#endif

#if (not defined ESP32) && (not defined ESP8266)
  #if defined webSupport
    #undef webSupport
    //    #error webSupport only available on ESP32 or ESP8266
  #endif
#endif
  
#if defined ESP32
  #include <iostream> 
  #include <sstream> 
  #include <driver/uart.h>  // In Arduino ESP32 install repo 
  //C:\Users\<YourName>\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\tools\sdk\include\driver

#endif
  
  #if (defined ESP32) || (defined ESP8266)  // ESP32 or ESP8266 (UART0, UART1, and UART2)
    #if (frPort_Serial  == 3)    
      #error Board does not have Serial3. This configuration is not possible.
    #endif
  #endif

  #if (defined TEENSY3X) 
    #if (FC_Mavlink_IO == 3) || (defined GCS_Mavlink_SD)  
      #error SD card not currently implemented for Teensy
    #endif
  #endif

  #ifndef Battery_mAh_Source
    #error Please choose at least one Battery_mAh_Source
  #endif

  #if (defined TEENSY3X) 
     #if defined webSupport
       #undef  webSupport
     #endif
     #if defined FC_Mavlink
       #undef FC_Mavlink
     #endif  
     #if defined FC_Mavlink_IO
       #undef  FC_Mavlink_IO     
       #define FC_Mavlink_IO  0
     #endif
     #if defined GCS_Mavlink_IO
       #undef GCS_Mavlink_IO
     #endif   
     #define GCS_Mavlink_IO 9
       
     #if (FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3) || (defined webSupport)   // if wifi selected
       #error WiFi and webSupport only work on an ESP32 or ESP8266 board
     #endif  
  #endif

  #if (defined ESP8266)  && ((GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 3))  // Can't do BT on 8266
      #undef GCS_Mavlink_IO
      #define GCS_Mavlink_IO  2    // WiFi Only
  #endif

  #if (FC_Mavlink_IO == 1) || (GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 3)       
    #if (not defined ESP32) 
      #error Bluetooth works only on an ESP32 board      
    #else
      #if (not defined BT_Mode)
        #error Please define BT_Mode  
      #endif
    #endif
  #endif  
  
  #ifndef FC_Mavlink_IO
    #error Please choose at least one Mavlink FC IO channel
  #endif

  #if (defined ESP32)
    #ifndef WiFi_Mode 
      #error Please define WiFi_Mode
    #endif
  #endif  

  #if (defined ESP32)
    #ifndef Mav_WiFi_Protocol
      #error Please define Mav_WiFi_Protocol
    #endif
  #endif

  #if (defined ESP32)         
    #ifndef ESP32_Variant 
      #error Please define an ESP32 board variant
    #endif
  #endif

  #if (defined ESP8266)
    #ifndef ESP8266_Variant
         #error Please define an ESP8266 board variant
    #endif
 
  #endif         

#if ((FrSky_Port_Type == 0) && (defined TEENSY3X) )
   #error Are you sure you want no FrSky port support on this Teensy 3.x?
#endif


  #if (FrSky_Port_Type == 1) || (FrSky_Port_Type == 2) || (FrSky_Port_Type == 3) || (FrSky_Port_Type == 4) 
    #define frBuiltin     //  for S.Port or F.Port we need FrSky_Port support compiled in
  #endif

  #if (defined ESP32 || defined ESP8266) && (FC_Mavlink_IO == 2 || FC_Mavlink_IO == 3 || GCS_Mavlink_IO == 2 || GCS_Mavlink_IO == 3 || defined webSupport)
    #define wifiBuiltin   //  for these features we need wifi support compiled in
  #endif    

  #if (defined ESP32) && (FC_Mavlink_IO == 1 || FC_Mavlink_IO == 3 || GCS_Mavlink_IO == 1 || GCS_Mavlink_IO == 3)
    #define btBuiltin   //  for these features we need bluetooth support compiled in
  #endif

  #if (defined ESP32 || defined ESP8266) &&  ((FC_Mavlink_IO == 3) || (defined GCS_Mavlink_SD) || (defined FrSky_IO_SD) )  // SD Card / TF - ESP32 only 
    #define sdBuiltin  
  #endif

 #if defined ESP_Onewire 
   #if not defined ESP32_SoftwareSerial 
     #error ESP_Onewire is predicated on ESP32_SoftwareSerial
   #endif
 #endif
  
 #if (FC_Mavlink_IO == 0 && GCS_Mavlink_IO == 0)
   #error Similtaneous serial uplink and serial downlink not supported. Not enough uarts.
 #endif
 
#if (not defined FrSky_Port_Type)
  #error define FrSky_Port_Type, None or SPort_Version or FPort_Version 1 or 2 or Auto
#endif    

 
//=================================================================================================   
//                          P L A T F O R M   D E P E N D E N T   S E T U P S
//================================================================================================= 

  
#if defined TEENSY3X               // Teensy3x
  #define Teensy_One_Wire          // default half-duplex
  #define MavStatusLed  13
  #define InvertMavLed false   
  #define BufStatusLed  14        
  #define mav_rxPin      9  
  #define mav_txPin     10
  #define frPort_Serial    1      // Teensy F/SPort port1=pin1, port3=pin8. The default is Serial 1, but 3 is possible 

  #if (frPort_Serial == 1)
    #define fr_rxPin       0      // FPort rx - optional
    #define fr_txPin       1      // FPort tx - Use me in single wire mode 
    #define GC_Mav_rxPin   7    
    #define GC_Mav_txPin   8   
  #elif (frPort_Serial == 3)
    #define fr_txPin       7      // Optional FPort rx  
    #define fr_txPin       8      // Optional FPort tx - use me in single wire mode 
  #endif  
   // #define displaySupport       // un-comment for displaySupport
    #if (defined displaySupport)   // Display type defined with # define displaySupport 
      #define SSD1306_Display      // I2C OLED display type   
      #define SCR_ORIENT   1       // 1 Landscape or 0 Portrait 
      /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        // Digital pin to trigger information display              
      #define Pup           11        // Digital pin to scroll the display up
      #define Pdn           12        // Board Button 2 to scroll the display down   
      // SDA                18        // I2C OLED - default in teensy "pins_arduino.h"
      // SCL                19        // I2C OLED - default in teensy "pins_arduino.h"
      #define i2cAddr      0x3C       // I2C OLED board
    #endif  
 
#elif defined ESP32                 // ESP32 Platform
  
  //========================================================================= 
  //   N O T E:  G P I O 1 2  is a bootstrap pin on the ESP32 - avoid GPIO12 high on bootup

  #if (ESP32_Variant == 1)          // ESP32 Dev Module
    #define MavStatusLed  02        // Onboard LED
    #define InvertMavLed false      
    #define BufStatusLed  27        // untested pin      
    #define mav_rxPin     16        // Mavlink serial rx
    #define mav_txPin     17        // Mavlink serial tx
    #define fr_rxPin      13        // FPort- Not used in 1-wire mode DON'T use 12!
    #define fr_txPin       4        // FPorttx - Use me in single wire mode
    #define startWiFiPin   5        // 5 Trigger WiFi startup  
    //#define displaySupport     // activate me if you have a display
    #if (defined displaySupport)   // Display type defined with board variant
      #define SSD1306_Display         // OLED display type    
      /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        //    Digital pin to toggle information/log page              
      #define Pup           99        // 35 Board Button 1 to scroll the display up
      #define Pdn           99        //  0 Board Button 2 to scroll the display down  
      #define Tinfo         15        // 15 Touch pin to toggle information/log page       
      #define Tup           33        // 33 Touch pin to scroll the display up
      #define Tdn           32        // 32 Touch pin to scroll the display down   
      #define SDA           21        // I2C OLED board
      #define SCL           22        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif   
    /*  
      SPI/CS                       Pin 05   For optional TF/SD Card Adapter
      SPI/MOSI                     Pin 23   For optional TF/SD Card Adapter
      SPI/MISO                     Pin 19   For optional TF/SD Card Adapter
      SPI/SCK                      Pin 18   For optional TF/SD Card Adapter  
    */

  #endif
  
  //========================================================================= 
  #if (ESP32_Variant == 2)          // Wemos® LOLIN ESP32-WROOM-32_OLED_Dual_26p
    #define MavStatusLed  15        // No Onboard LED
    #define InvertMavLed false     
    #define BufStatusLed  99        // None    
    #define mav_rxPin     25        // Mavlink serial rx
    #define mav_txPin     26        // Mavlink serial tx
    #define fr_rxPin      13        // FPort- Not used in single wire mode DON'T use 12!
    #define fr_txPin      14        // FPorttx - Use me in single wire mode
    #define startWiFiPin  18        // Trigger WiFi startup
    
    #if (defined displaySupport)   // Display type defined with # define displaySupport 
      #define SSD1306_Display      // OLED display type   
      #define SCR_ORIENT   1       // 1 Landscape or 0 Portrait 
      /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        //    Digital pin to trigger information display              
      #define Pup           99        // 35 Board Button 1 to scroll the display up
      #define Pdn           99        //  0 Board Button 2 to scroll the display down   
      #define Tinfo         99        // 02 Touch pin to toggle information/log page       
      #define Tup           99        // 33 Touch pin to scroll the display up
      #define Tdn           99        // 32 Touch pin to scroll the display down   
      #define Tinfo         99        //    Touch pin to toggle information/log page
      #define SDA           05        // I2C OLED board
      #define SCL           04        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif  
  #endif
  
  //=========================================================================  
  #if (ESP32_Variant == 3)          // Dragonlink V3 slim with internal ESP32
    #if defined mvBaud
      #undef mvBaud
      #define mvBaud 115200       // Force baud rate to DragonLink rate
    #endif
    #define MavStatusLed  18        // Blue LED
    #define InvertMavLed false    
    #define BufStatusLed  19        // Green LED        
    #define mav_rxPin     16        // Mavlink serial rx
    #define mav_txPin     17        // Mavlink serial tx
    #define fr_rxPin      13        // FPort- Not used in single wire mode DON'T use 12!
    #define fr_txPin      01        // FPorttx - Use me in single wire mode
    #define startWiFiPin  99        // Trigger WiFi startup  
    
    #if (defined displaySupport)   // Display type defined with # define displaySupport   
      #define SSD1306_Display      // OLED display type 
      #define SCR_ORIENT   0       // 1 Landscape or 0 Portrait       
      /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        //    Digital pin to toggle information/log page         
      #define Pup           99        // 35 Board Button 1 to scroll the display up
      #define Pdn           99        //  0 Board Button 2 to scroll the display down 
      #define Tinfo         99        //    Touch pin to toggle information/log page         
      #define Tup           99        // 33 Touch pin to scroll the display up
      #define Tdn           99        // 32 Touch pin to scroll the display down   
      #define SDA           05        // I2C OLED board
      #define SCL           04        // I2C OLED board 
      #define i2cAddr      0x3C       // I2C OLED board
    #endif  
  #endif
  
  //=========================================================================   
  #if (ESP32_Variant == 4)          // Heltec Wifi Kit 32 (NOTE! 8MB) 
    #define MavStatusLed    25        // Onboard LED
    #define InvertMavLed   false     
    #define BufStatusLed    99        // none  
    #define mav_rxPin       27        // Mavlink serial rx
    #define mav_txPin       17        // Mavlink serial tx
    #define fr_rxPin        13        // FPort rx - (NOTE: DON'T use pin 12! boot fails if pulled high)
    #define fr_txPin        14        // FPort tx - Use me in single wire mode
    #define startWiFiPin    18        // Trigger WiFi startup 
    #if !defined displaySupport       // I2C OLED board is built into Heltec WiFi Kit 32
      #define displaySupport
    #endif  
    #define SSD1306_Display         // OLED display type  
    #define SCR_ORIENT   1          // 1 Landscape or 0 Portrait 
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == 99 means the pin-pair is not used
     */ 
    #define Pup           99        // Board Button to scroll the display up
    #define Pdn           99        // Board Button to scroll the display down
    #define Pinfo         99        // 02 Digital pin to toggle information/log page
    #define Tup           33        // 33 Touch pin to scroll the display up
    #define Tdn           32        // 32 Touch pin to scroll the display down 
    #define Tinfo         02        // 02 Touch pin to toggle information/log page
    #define SDA           04        // I2C OLED board 
    #define SCL           15        // I2C OLED board
    #define i2cAddr      0x3C       // I2C OLED board
    #define OLED_RESET    16        // RESET here so no reset lower down    

    /*  
      SPI/CS               05   For optional TF/SD Card Adapter
      SPI/MOSI             23   For optional TF/SD Card Adapter
      SPI/MISO             19   For optional TF/SD Card Adapter
      SPI/SCK              18   For optional TF/SD Card Adapter  
    */

  #endif    
  
  //=========================================================================   
  #if (ESP32_Variant == 5)          // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD(135 x 240), IDE board = "ESP32 Dev Module"
                                    // Remember to select the T_Display board in User_Setup_Select.h in TFT_eSPI library
    #define MavStatusLed  25        // Add your own LED with around 1K series resistor
    #define InvertMavLed false     
    #define BufStatusLed  99        // none
    #define mav_rxPin     27        // Mavlink serial rx
    #define mav_txPin     17        // Mavlink serial tx
    #define fr_rxPin      13        // F/SPort rx - (NOTE: DON'T use pin 12! boot fails if pulled high)
    #define fr_txPin      15        // F/SPort tx - Use me in single wire mode
    #define startWiFiPin  99        // 99=none. No input pin available (non touch!) Could use touch with a bit of messy work.
     
    #if !defined displaySupport    // I2C TFT board is built into TTGO T-Display
      #define displaySupport
    #endif    
    #define ST7789_Display          // TFT display type - if you have a display you must define which type
    #define SCR_ORIENT   1          // 1 Landscape or 0 Portrait    
    
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == 99 means the pin-pair is not used
     */ 
    #define Pinfo          2        //    Digital pin to toggle information/log page     
    #define Pup            0        //  0 Board Button 1 to scroll the display up
    #define Pdn           35        // 35 Board Button 2 to scroll the display down 
    #define Tinfo         99        //    Touch pin to toggle information/log page       
    #define Tup           99        // 33 Touch pin to scroll the display up
    #define Tdn           99        // 32 Touch pin to scroll the display down   
    
    #define SCR_ORIENT     1        // 1 Landscape or 0 Portrait
 
    #define SDA           21        // I2C TFT board 
    #define SCL           22        // I2C TFT board
    #define i2cAddr      0x3C       // I2C TFT board
  #endif
 
   //=========================================================================   
  #if (ESP32_Variant == 6)          // LILYGO® TTGO T2 SD SSD1331 TFT Colour 26pin  IDE board = "ESP32 Dev Module"
    #define MavStatusLed   5        // BoardLED
    #define InvertMavLed true     
    #define BufStatusLed  99        // none
    #define mav_rxPin     17        // Mavlink serial rx
    #define mav_txPin     18        // Mavlink serial tx
    #define fr_rxPin      19        // FPort rx - possible 22 / 23
    #define fr_txPin      21        // FPort tx - Use me in single wire mode
    #define startWiFiPin  99        // 99=none. No input pin available (non touch!) Could use touch with a bit of messy work.

    #if !defined sdBuiltin         // SD reader is built into TTGO T2
      #define sdBuiltin
    #endif
    #if !defined displaySupport      // I2C OLED board is built into TTGO T2
      #define displaySupport
    #endif
    #define SSD1331_Display         // colour TFT display type see here https://github.com/emard/ulx3s/issues/8
                                    // software graphic display clear very slow
    #define SCLK          14          // SPI pins for SSD1331 
    #define MOSI          13
    #define CS            15
    #define DC            16
    #define RST            4
    #define MISO          12          // apparently not used by Adafruit    
    /*    Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *    Pin == 99 means the pin-pair is not used
     */ 
    #define Pinfo         25        // Digital pin to toggle information/log page        
    #define Pup           99        // Board Button 1 to scroll the display up
    #define Pdn           99        // Board Button 2 to scroll the display down    
    #define Tinfo         99        // Touch pin to toggle information/log page    
    #define Tup           32        // Touch pin to scroll the display up
    #define Tdn            2        // Touch pin to scroll the display down      
      
  #endif  

   //=========================================================================  
  #if (ESP32_Variant == 7)          // ESP32 Dev Module with ILI9341 2.8" colour TFT SPI 240x320
    #define MavStatusLed  02        // Onboard LED
    #define InvertMavLed false      
    #define BufStatusLed  27        // untested pin      
    #define mav_rxPin     16        // Mavlink serial rx
    #define mav_txPin     17        // Mavlink serial tx
    #define fr_rxPin      13        // FPort- Not used in 1-wire mode DON'T use 12!
    #define fr_txPin       4        // FPort tx - Use me in single wire mode
    #define startWiFiPin   5        // 5 Trigger WiFi startup  
    
    #if !defined displaySupport      // I2C OLED board is built into TTGO T2
      #define displaySupport
    #endif
    #define ILI9341_Display         // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
    #define SCLK          18        // blue wire on my test setup
    #define MOSI          23        // yellow wire
    #define CS            25        // white wire
    #define DC            22        // green wire
    #define RST           21        // brown wire
    // LED=3.3V,  Vcc=5v,  Gnd 
    // MISO                not used by Adafruit     
    
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == 99 means the pin-pair is not used
     */ 
    #define Pinfo         99        //    Digital pin to toggle information/log page              
    #define Pup           99        // 35 Board Button 1 to scroll the display up
    #define Pdn           99        //  0 Board Button 2 to scroll the display down  
    #define Tinfo         15        // 15 Touch pin to toggle information/log page       
    #define Tup           33        // 33 Touch pin to scroll the display up
    #define Tdn           32        // 32 Touch pin to scroll the display down   

  #endif   
   
#elif defined ESP8266                    // ESP8266 Platform

  
  //========================================================================= 
  #if (ESP8266_Variant == 1)        // NodeMCU 12F board - Dev board with usb etc
  
    #define MavStatusLed  D4        // D4/GPIO2 Board LED - Mav Status LED inverted logic - use 99 while debug
    #define InvertMavLed true      
    #define BufStatusLed  99        // None     
   //                     D4        // TXD1 - Serial1 debug log out SHARED WITH BOARD LED                         
    #define mav_rxPin     D9        // RXD0 default  
    #define mav_txPin     D10       // TXD0 default    
    #define fr_rxPin      D5        // FPort- Not used in single wire mode
    #define fr_txPin      D6        // FPorttx - Use me in single wire mode
    #define startWiFiPin  99        // 99=none or D3/D7 - Trigger WiFi startup 
                                    // NOTE: There are not enough pins for wifi pin and display scrolling

    #if (defined displaySupport)   // Display type defined with # define displaySupport   
      /* Below please choose Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo       99        // Digital pin to toggle information/log page           
      #define Pup         D3        // D3 Board Button 1 to scroll the display up
      #define Pdn         D7        // D7 Board Button 2 to scroll the display down    
      #define SCL         D1        // I2C OLED board   
      #define SDA         D2        // I2C OLED board
      #define i2cAddr    0x3C       // I2C OLED board
    #endif
        
  #endif
  
  //=========================================================================   
  #if (ESP8266_Variant == 2)   // ESP-12E, ESP-F barebones boards. RFD900X TX-MOD, QLRS et al - use Generic ESP8266 on IDE
    //                         GPIO as per node mcu
    static const uint8_t D0   = 16;   // SCL - optional
    static const uint8_t D1   = 5;    // SDA - optional
    static const uint8_t D2   = 4;    // FPorttx - Use me in single wire mode
    static const uint8_t D3   = 0;    // Flash
    static const uint8_t D4   = 2;    // BoardLED & TXD1 optional debug out
    static const uint8_t D5   = 14;   // FPortrx (unused in half-duplex)
    static const uint8_t D6   = 12;   // P2-3 exposed dual row of pins
    static const uint8_t D7   = 13;   // CTS
    static const uint8_t D8   = 15;   // RTS
    static const uint8_t D9   = 3;    // RXD0 mavlink and flashing
    static const uint8_t D10  = 1;    // TXD0 mavlink and flashing
    
    #define MavStatusLed  D4        // D4 Board LED - Mav Status LED inverted logic - use 99 while debug
    #define InvertMavLed true    
    #define BufStatusLed  99        // None
    //                    D4        // TXD1 - Serial1 default debug log out SHARED WITH BOARD LED                           
    #define mav_rxPin     D9        // RXD0 default  
    #define mav_txPin     D10       // TXD0 default    
    #define fr_rxPin      D5        // FPort- Not used in single wire mode
    #define fr_txPin      D2        // FPort(half-duplex) inverted - Use me in single wire mode
    #define startWiFiPin  D6        // Trigger WiFi startup 
                                    // NOTE: There may not be enough pins for wifi pin AND display scrolling  
    #if (defined displaySupport)    // Display type defined with # define displaySupport 
      #define SSD1306_Display  
      /* Below please choose Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        // Digital pin to toggle information/log page          
      #define Pup           99        // D3 Board Button 1 to scroll the display up
      #define Pdn           99        // D7 Board Button 2 to scroll the display down    
      #define SCL           D0        // I2C OLED board   
      #define SDA           D1        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif 
  #endif // end of this ESP8266 variant 
  
  //========================================================================= 
  #if (ESP8266_Variant == 3)   // ESP-12F, Wemos® LOLIN D1 Mini - use Generic ESP8266 on Arduino IDE
    //  Pin Map as per C:\Users\<user>\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.7.1\variants\d1_mini\pins_arduino.h

    #define LED_BUILTIN 2

    static const uint8_t D0   = 16;  // WiFi trigger
    static const uint8_t D1   = 5;   // SCL - optional
    static const uint8_t D2   = 4;   // SDA - optional
    static const uint8_t D3   = 0;   // Flash - reserved
    static const uint8_t D4   = 2;   // LED_BUILTIN & TXD1 optional debug out
    static const uint8_t D5   = 14;  // FPortrx (unused in half-duplex)
    static const uint8_t D6   = 12;  // FPorttx - Use me in single wire mode
    static const uint8_t D7   = 13;  // CTS 
    static const uint8_t D8   = 15;  // RTS
    static const uint8_t D9   = 3;   // RXD0
    static const uint8_t D10  = 1;   // TCD0 
    
    #define MavStatusLed  D4        // D4 Board LED - Mav Status LED inverted logic - use 99 while debug
    #define InvertMavLed true    
    #define BufStatusLed  99        // None
    //                    D4        // TXD1 - Serial1 default debug log out SHARED WITH LED_BUILTIN BOARD LED                           
    #define mav_rxPin     D9        // RXD0 - Serial(0) 
    #define mav_txPin     D10       // TXD0 - Serial(0) 
    #define fr_rxPin      D5        // FPort- Not used in single wire mode
    #define fr_txPin      D6        // FPort- inverted - Use me in single wire mode
    #define startWiFiPin  D16       // Trigger WiFi startup 
      
    #if (defined displaySupport)   // Display type defined with # define displaySupport   
      /* Below please choose Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        // Digital pin to toggle information/log page           
      #define Pup           99        // D3 Board Button 1 to scroll the display up
      #define Pdn           99        // D7 Board Button 2 to scroll the display down    
      #define SCL           D1        // I2C OLED board   
      #define SDA           D2        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif 
 
  #endif // end of this ESP8266 variant 
  
#endif  // end of all ESP8266 variants

  //=================================================================================================   
  //                            E E P R O M    S U P P O R T   -   ESP Only - for now
  //================================================================================================= 

  #if (defined ESP32) || (defined ESP8266)
    #include <EEPROM.h>       // To store AP_Failover_Flag and webSupport Settings
    #define EEPROM_SIZE 166   // 1 + 165 bytes = 166, addr range 0 thru 165 
  #endif
    
  //=================================================================================================   
  //                             S D   C A R D   S U P P O R T   -   ESP Only - for now
  //================================================================================================= 
  #if ((defined ESP32)  || (defined ESP8266)) && (defined sdBuiltin) 

    #include <FS.h>
    #include <SD.h>
    #include <SPI.h>
    #define SD_Libs_Loaded 

    /*
    // Optional SPI interface pins for SD card adapter or SSD1306 OLED display
    #define CS            5        
    #define MOSI          23 
    #define MISO          19 
    #define SCK           18 
    */  

    // Pins generally   CS=5    MOSI=23   MISO=19   SCK=18    3.3V   GND   Dev Board, LilyGO/TTGO
 
 
     
    // Rememeber to change SPI frequency from 4E6 to 25E6, i.e 4MHz to 25MHz in SD.h otherwise MavRingBuff fills up 
    // C:\Users\YourUserName\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.2\libraries\SD\src  
    // bool begin(uint8_t ssPin=SS, SPIClass &spi=SPI, uint32_t frequency=25000000, const char * mountpoint="/sd", uint8_t max_files=5);  

    char              cPath[40];
    std::string       fnPath[30];
    uint8_t           fnCnt;
    uint16_t          sdReadDelay = 10;  // mS   Otherwise the reads run through unnaturally quickly
    
    const uint16_t    sd_buf_sz = 128;
    byte              sd_buf [sd_buf_sz];
    uint16_t          sd_idx = 0;
     
    File     file;  // Create global object from File class for general use

    static  const uint8_t mthdays[]={31,28,31,30,31,30,31,31,30,31,30,31}; 

    typedef struct  { 
      uint16_t yr;   // relative to 1970;  
      uint8_t mth;
      uint8_t day;
      uint8_t dow;   // sunday is day 1 
      uint8_t hh; 
      uint8_t mm; 
      uint8_t ss; 
    }   DateTime_t;

    static DateTime_t dt_tm; 

  #endif  
  //=================================================================================================   
  //                      D I S P L A Y   S U P P O R T    E S P  O N L Y - for now
  //=================================================================================================  

  #if defined displaySupport
    #if (!( (defined SSD1306_Display) || (defined SSD1331_Display) || (defined ST7789_Display) || (defined ILI9341_Display) ))
      #error please define a display type in your board variant configuration, or disable displaySupport
    #endif   

    #if not defined SD_Libs_Loaded    // by SD block
      #include <SPI.h>                // for SD card and/or Display
    #endif  

    typedef enum display_mode_set { logg = 1 , flight_info = 2 } display_mode_t;
    
    display_mode_t      display_mode;     
        
    bool infoPressBusy = false;
    bool infoNewPress = false;         
    bool infoPressed = false;
    bool show_log = true;    
    uint32_t info_debounce_millis = 0; 
    uint32_t  info_millis = 0; 
    uint32_t  last_log_millis = 0;
    const uint16_t db_period = 1000; // debounce period mS

    #if (ESP32_Variant == 6)
      // Generic colour definitions
      #define BLACK           0x0000
      #define BLUE            0x001F
      #define RED             0xF800
      #define GREEN           0x07E0
      #define CYAN            0x07FF
      #define MAGENTA         0xF81F
      #define YELLOW          0xFFE0
      #define WHITE           0xFFFF 
    #endif

  //==========================================================
    
    #if (defined ST7789_Display)      // TTGO T_Display 1.14 TFT display 135 x 240 SPI
      #include <TFT_eSPI.h>           // Remember to select the T_Display board in User_Setup_Select.h in TFT_eSPI library (135 x 240) 
      TFT_eSPI display = TFT_eSPI();
      #define SCR_W_PX      240       // OLED display width, in pixels
      #define SCR_H_PX      135       // OLED display height, in pixels

      #if (SCR_ORIENT == 0)           // portrait
        #define SCR_H_CH     20       // characters not pixels
        #define SCR_W_CH     11       // ?
        #define CHAR_W_PX    12       // pixels    
        #define CHAR_H_PX    12       // pixels 
        #define TEXT_SIZE     1                
      #elif (SCR_ORIENT == 1)         // landscape
        #define SCR_H_CH      8       // characters not pixels 
        #define SCR_W_CH     20  
        #define CHAR_W_PX    12       // 12 x 20 = 240  
        #define CHAR_H_PX    16       // 16 x 8 =  128
        #define TEXT_SIZE     2                       
      #endif 
      
    //==========================================================
    
    #elif (defined SSD1306_Display)    // SSD1306 OLED display     (128 x 64) 
      #include <Adafruit_GFX.h>
      #include <Adafruit_SSD1306.h> 
      #define SCR_W_PX 128     // OLED display width, in pixels
      #define SCR_H_PX 64      // OLED display height, in pixels
      #define SCR_ORIENT  1    // 0 portrait  1 landscape
      #define TEXT_SIZE   1
      #define SCR_H_CH    8    // characters not pixels 
      #define SCR_W_CH   21   
      #define CHAR_W_PX   6    // 6 x 21 = 126  
      #define CHAR_H_PX   8    // 8 x 8 = 64    
      #ifndef OLED_RESET
        #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
      #endif  
      Adafruit_SSD1306 display(SCR_W_PX, SCR_H_PX, &Wire, OLED_RESET); 
          
    //==========================================================  
    #elif (defined SSD1331_Display)    // SSD1331 0.95" TTGO T2 colour TFT display (96 x 64)
      #include <Adafruit_GFX.h>
      #include <Adafruit_SSD1331.h>
      #define SCR_W_PX    96     // OLED display width, in pixels
      #define SCR_H_PX    64     // OLED display height, in pixels
      #define SCR_ORIENT   1     // 0 portrait  1 landscape
      #define TEXT_SIZE    1
      #define SCR_H_CH     8     // characters not pixels 
      #define SCR_W_CH    16        
      #define CHAR_W_PX    6     // 6 x 16 = 96   
      #define CHAR_H_PX    8     // 8 x 8  = 64 
      Adafruit_SSD1331 display = Adafruit_SSD1331(CS, DC, MOSI, SCLK, RST);  
      
    //========================================================== 
    #elif  (defined ILI9341_Display)    // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
      #include "Adafruit_GFX.h"   
      #include "Adafruit_ILI9341.h"
      // Uses hardware SPI
      
      Adafruit_ILI9341 display = Adafruit_ILI9341(CS, DC, RST);    // LED=3.3V,  Vcc=5v,  Gnd 
         
      #define SCR_ORIENT   1            // 0 for portrait or 1 for landscape
      #define TEXT_SIZE    2            // default, may be changed on the fly
        
      #if (SCR_ORIENT == 0)      // portrait
        #define SCR_H_PX  320
        #define SCR_W_PX  240  
      #elif (SCR_ORIENT == 1)    // landscape
        #define SCR_H_PX  240
        #define SCR_W_PX  320
      #endif 

      #if (TEXT_SIZE == 1) 
        #define CHAR_W_PX    6     // 40 / 53 ch wide   portrait / landscape
      #elif  (TEXT_SIZE == 2)       
        #define CHAR_W_PX    12    // 20 / 26 ch wide  
      #elif  (TEXT_SIZE == 3)  
        #define CHAR_W_PX    18    // 13 / 17 ch wide
      #elif  (TEXT_SIZE == 4)  
        #define CHAR_W_PX    24    // 10 / 13 ch wide
      #elif  (TEXT_SIZE == 5)  
        #define CHAR_W_PX    30    // 10 / 8 ch wide
      #endif
      
     #define CHAR_H_PX   (CHAR_W_PX) + ( (CHAR_W_PX) / 3)    // so vertical spacing is h spacing plus 1/3
     #define SCR_W_CH   (SCR_W_PX) / (CHAR_W_PX)     
     #define SCR_H_CH  (SCR_H_PX) / (CHAR_H_PX)  
      
    #endif   
    //==========================================================   

    #if (!(defined SCR_ORIENT) )
      #error please define a desired screen orientation,  0 portrait or 1 landscape for your board variant or display type
    #endif 

    char clear_line[SCR_W_CH+1];
    
    // allocate space for screen buffer   
    #define max_col   SCR_W_CH+1  // +1 for terminating line 0x00        
    #define max_row   64
  
    static const uint16_t threshold = 40;
    volatile bool upButton = false;
    volatile bool dnButton = false;
    volatile bool infoButton = false;
    
    #if (not defined Tup) 
      #define Tup         99
    #endif

    #if (not defined Tdn) 
      #define Tdn         99
    #endif

    #if (not defined Pup) 
      #define Tup         99
    #endif

    #if (not defined Pdn) 
      #define Tdn         99
    #endif

    typedef enum scroll_set { non = 0, up = 1, down = 2 } scroll_t;
    scroll_t up_down = non; 

    typedef enum last_row_set { omit_last_row = 0, show_last_row = 1} last_row_t;
    last_row_t last_row_action;   

    struct row_t {
      char x[max_col];
      };
  
     row_t ScreenRow[max_row]; 
     
    uint8_t   row = 0;
    uint8_t   col = 0;
    uint8_t   scroll_row = 0;
    uint32_t  scroll_millis =0 ;

  #endif   // end of Display_Support

  //=================================================================================================   
  //                     B L U E T O O T H   S U P P O R T -  E S P 3 2  O n l y
  //================================================================================================= 

  #if ((defined ESP32) && (defined btBuiltin))

    #include "BluetoothSerial.h"
    #include "esp_bt.h"
    #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
      #error Bluetooth is not enabled! Please run `idf.py menuconfig` in IDF workspace 
    #endif

    BluetoothSerial SerialBT;

  #endif  

  //=================================================================================================   
  //                       W I F I   S U P P O R T - ESP32 and ES8266 Only
  //================================================================================================= 

    uint16_t  TCP_localPort = 5760;     
    uint16_t  TCP_remotePort = 5760;    
    uint16_t  UDP_localPort = 14555;    // readPort / MP and QGC expect to send to this default FC UDP port
    uint16_t  UDP_remotePort = 14550;   // sendPort / MP and QGC listens & reads on this default port  
    
    uint16_t  udp_read_port;
    uint16_t  udp_send_port;
             
    bool      FtRemIP = true;
    bool      wifiDisconnected = false;
    int16_t   wifi_rssi;
    uint16_t  wifi_status = 0xfe;   // 0xfe = unused yet
    uint8_t   AP_sta_count = 0;
    uint8_t   AP_prev_sta_count = 0;
    
  #if ( (defined ESP32) || (defined ESP8266) )

  #if (defined ESP32)
    // timer for wifi retry interrupt
    hw_timer_t * timer = NULL;
    volatile SemaphoreHandle_t wifiTimerSemaphore;
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  #endif

  #if (defined ESP8266)
    uint32_t esp8266_wifi_retry_millis = 0;
  #endif
  
  volatile bool  wifiButton = false;
    
    // Define link variables
    struct linkStatus {
      uint32_t    packets_received;
      uint32_t    packets_lost;
      uint32_t    packets_sent;
    };
    bool          hb_heard_from = false;
    uint8_t       hb_system_id = 0;
    uint8_t       hb_comp_id = 0;
    uint8_t       hb_seq_expected = 0;
    uint32_t      hb_last_heartbeat = 0;
    linkStatus    link_status;
  
    #if defined ESP32 
      #include <WiFi.h>  // includes UDP class
      #if defined webSupport
        #include <WebServer.h> 
        #include <Update.h> 
        WebServer server(80);
        
      #endif      
      #include <WiFiAP.h>  
    #endif

    #if defined ESP8266
      #include <ESP8266WiFi.h>   // Includes AP class
      #if defined webSupport
        #include <ESP8266WebServer.h>    
        ESP8266WebServer server(80);  
        #include <WiFiUdp.h>       
      #endif      
    #endif
    
   //====================       W i F i   O b j e c t s 
   
    #define max_clients    6
    uint8_t active_client_idx = 0;  // for TCP 
    uint8_t active_object_idx = 0;  // for UDP

    WiFiClient *tcp_client[max_clients] = {NULL}; // pointers to TCP client objects 
    
    WiFiServer TCPserver(TCP_localPort);          // dummy TCP local port(changes on TCPserver.begin() ).

    IPAddress UDP_remoteIP(192, 168, 1, 255);     // default to broadcast unless (not defined UDP_Broadcast)               
    IPAddress udpremoteip[max_clients];           // table of remote UDP client IPs
                   
    WiFiUDP *udp_object[2] = {NULL};              // pointers to UDP objects for STA and AP modes
    WiFiUDP frs_udp_object;  
                 
    IPAddress localIP;                            // tcp and udp
    IPAddress TCP_remoteIP(192,168,4,1);          // when we connect to a server in tcp client mode, put the server IP here 
    
  #endif  // end of ESP32 and ESP8266
  
//=================================================================================================   
//                                 S E R I A L
//=================================================================================================

#if (defined ESP32)  
  #define Log                 Serial         // USB
  #define mvSerial            Serial2        // RXD2 and TXD2

  #if defined ESP32_SoftwareSerial
    #include <SoftwareSerial.h>
    SoftwareSerial frSerial; 
  #else     // default HW Serial
    #define frSerial          Serial1     
  #endif   
    
#elif (defined ESP8266)
  #define Log                 Serial1        //  D4   TXD1 debug out  - no RXD1 !
  #define mvSerial            Serial         //  RXD0 and TXD0
  #include <SoftwareSerial.h>
  SoftwareSerial frSerial;  
#endif 

#if (defined TEENSY3X)      //  Teensy 3.1
  #define Log                 Serial         // USB  
  #define mvSerial            Serial2   
  #if (frPort_Serial == 1) 
    #define frSerial          Serial1        // F.Port 
  #elif (frPort_Serial == 3)
    #define frSerial          Serial3        // F.Port 
 #else
    #error frPort_Serial can only be 1 or 3. Please correct.
  #endif 
#endif 


//=================================================================================================   
//                                 D E B U G G I N G   O P T I O N S
//=================================================================================================

//#define inhibit_SPort     // Use me to send debug messages only, out of GPIO1/TX0 on ESP32_Variant 3, DL V3 internal ESP32
//#define Mav_Debug_All
//#define Frs_Debug_All
//#define Mav_Debug_RingBuff

//#define Debug_Air_Mode
//#define Debug_Relay_Mode

//#define Mav_Debug_Params
//#define Debug_BT    
//#define Debug_FC_Down         // traffic down from FC to Ring Buffer
//#define Debug_FC_Up           // traffic up to FC from GCS
//#define Debug_GCS_Down        // traffic from RB to GCS
//#define Debug_GCS_Up          // traffic up from GCS to FC

//#define Debug_Read_UDP_GCS  

//#define Debug_Send_UDP_GCS

//#define Debug_Read_UDP_FC  
//#define Debug_Send_UDP_FC  

//#define Mav_Debug_Servo
//#define Frs_Debug_Servo
//#define Mav_Debug_Rssi        // #109 > #65 > #35
//#define Frs_Debug_Rssi        // 0xF101
//#define Mav_Debug_RC           
//#define Frs_Debug_RC

//#define Frs_Debug_Params       //0x5007
#define Frs_Debug_APStatus    // 0x5001
//#define Mav_Debug_SysStatus   // #1 && battery
//#define Debug_Batteries       // 0x5003
//#define Frs_Debug_Home        // 0x5004
//#define Mav_Debug_GPS_Raw     // #24
//#define Mav_Debug_GPS_Int     // #33
//#define Frs_Debug_LatLon      // 0x800
//#define Frs_Debug_VelYaw      // 0x5005
//#define Frs_Debug_GPS_status  // 0x5002
//#define Mav_Debug_Scaled_IMU
//#define Mav_Debug_Raw_IMU
//#define Mav_Debug_Hud         // #74
//#define Frs_Debug_Hud         // 0x50F2
//#define Mav_Debug_Scaled_Pressure
//#define Mav_Debug_Attitude    // #30

//#define Mav_Debug_Range       // #173
//#define Frs_Debug_AttiRange   // 0x5006

//#define Mav_Debug_Terrain     // #136
#define Frs_Debug_Terrain     // 0x500B

//#define Mav_Debug_Fence     // #162
#define Frs_Debug_Fence     // 0x500B

//#define Mav_Debug_StatusText  // #253  
//#define Frs_Debug_StatusText  // 0x5000
//#define Mav_Debug_Mission
//#define Frs_Debug_Mission   
//#define Mav_Debug_System_Time   
//#define Decode_Non_Essential_Mav 

//#define Debug_Radio_Status  
//#define Debug_GCS_Unknown
//#define Debug_Param_Request_Read
//#define Mav_Show_Unknown_Msgs
//#define Mav_Print_All_Msgid
//#define Debug_Eeprom
//#define Mav_Debug_RPM
#define Frs_Debug_RPM
//#define Debug_SD   
//#define Debug_WiFi
//#define Debug_Loop_Period
//#define Mav_Debug_Command_Ack
//#define Debug_SRAM

//#define Debug_Web_Settings

//#define Mav_Debug_FC_Heartbeat
//#define Mav_Debug_GCS_Heartbeat
//#define Debug_Our_FC_Heartbeat

//#define Debug_Param_Request_Read  // #20

//#define Debug_Param_Request_List  // #21
//#define Mav_Debug_Params

//#define Frs_Debug_Payload

#define Report_Packetloss   2     // F.Port packet loss every n minutes

//#define CRC_Test_Case
//#define Debug_CRC
//#define Debug_FrPort_Serial_Loop
//#define Debug_FrPort_Switching

//#define Frs_Debug_Period

//#define Support_SBUS_Out 

//#define Debug_Read_TCP
//#define Debug_Read_UDP
//#define Debug_Send_TCP
//#define Debug_Send_UDP
//#define Debug_Inject_Delay

//#define MavLite_Debug_Scheduler
//#define Debug_Mavlite 
//#define Debug_Mavlite_Chunking
//#define Debug_Mavlite_SPort
//#define Mav_List_Params       // Use this to test uplink to Flight Controller 

//#define Debug_FrPort_Stream 
//#define Debug_FrPort_Stream_Out
//#define Debug_FrPort_Safe_Read

//#define Debug_FPort_Buffer 
//#define Frs_Debug_Scheduler // - this debugger affects the performance of the scheduler when activated

//#define Derive_PWM
//#define Debug_PWM_Channels
//#define Debug_Baud 

//#define Debug_FrSPort_Loop_Period

//=================================================================================================   
//                                   C H A N G E   L O G
//=================================================================================================
/*

Change log:
                                    
v2.00 2019-06-07 Plus version firmware ported to ESP32 Dev Module V1 successfully - no improvements  
v2.01 2019-06-09 Added OLED display support
v2.02 2019-05-18 Belatedly include Alex's Rangefinder PR that I missed.
v2.03 2019-05-21 Reduce voltage and current display moving average smoothing
                 Empirical correction of mAh consumed as per Markus Greinwald's measurements 
                 Change mav heartbeat timeout from 3 to 6 seconds
v2.04 2019-05-24 Merge Alex's BT classic PR. Thank you! 
                 Remove Aux port as no longer required
                 Tidy #define options  
v2.05 2019-06-09 Support 3 possible I/O channels on FC side, and 3 on GCS side. UART serial, BT and WiFi.
                 WiFi AP ssid = 'Mav2Passthru', pw = 'password' for now.  
v2.06 2019-06-10 Support added for STA mode and AP mode. Tidied up some lose ends. 
v2.07 2019-06-16 Initiate WiFi session with push button from GPIO15 momentary to ground. 
v2.08 2019-06-16 Add SD/TF card support - wip!!  Added UDP protocol option 
v2.09 2019-06-30 WiFi activation push button momentary but one-time. 
      2019-07-03 Implemented optional TLog input and output via SD card.  
v2.10 2019-07-09 For PX4 flight stack only, send HB to FC every 2 seconds  
      2019-07-10 Radical redesign of F.Port scheduling algorithm. Support SD and WiFi/BT I/O simultaneously 
v2.11 2109-07-11 Auto determine target board. Publish "Universal" version 2.11. One source, four platforms / boards 
v2.12 2019-07-12 Add #define PlusVersion, comment out for FlightDeck   
v2.13 2019-08-13 UDP now working in Access Point mode 
v2.14 2019-07-17 PX4 flight stack only - fixed longitude typo    if (ap24_lat<0) should be if (ap24_lon<0)
v2.15 2019-07-17 Switch to Adafruit_SSD1306 OLED library. 8 lines x 21 chars
v2.16 2019-07-18 Increase time burden for each successive Status Text chunk by 5mS
v2.17 2019-07-19 Auto detect serial telemetry and baud rate    
v2.18 2019-07-21 Tune FrSky packet schduler. Add option. Default is 1x.  //Send_status_Text_3_Times
v2.19 2019-07-22 Implement 2 tier scheduling. Tier1 gets priority, tier2 (0x5000) only when tier1 empty 
v2.20 2019-07-26 Release candidate. Send HB back to FC for APM also, not just PX4. Streamline library #includes.
                 #undef troublesome F function.  
v2.21 2019-07-26 Trap attempt to do GCS I/O on ESP32 or Blue Pill - no Serial3 UART.
      2019-07-29 Implement system health status-text messages as per Alex's request.  
v2.22 2019-08-10 Make sensor health messages optional for now. Fix end-of-sensor message text detection.
v2.23 2019-08-21 Add support for RFD900x long-range telemetry modems, specifically RSSI  
v2.24 2019-08-23 Workaround for Esp32 library "wifi: Set status to INIT" bug
                 Improve responsiveness to baud detect with no telemetry present.    
v2.25 2019-08-28 Version for RFD900x. Bug fixes on rssi. Include #define StartWiFi option to 
                 override startWiFi Pin.  
v2.26 2019-08-31 Improved GCS to FC debugging. Make baud rate sensing optional. 
v2.27 2019-09-13 Small additions to test LILYGO®_TTGO_MINI32_ESP32-WROVER_B
v2.28 2019-09-17 Localise pin definitions in one place to define ESP32 variants more easily
v2.29 2019-09-24 Use #if (TargetBoard == 3) to define soft pins for mvSerial
v2.30 2019-09-26 Don't push #5007 into F.Port table from #147. Push from #1 only.
v2.31 2019-09-30 Configurable declarations moved to config.h file
v2.32 2019-10-08 Source line corrupted in v2.17 affecting Relay Mode, fixed. Thank you burtgree! 
v2.33 2019-10-09 Don't invert ESP32 FPortin Relay Mode. Use commercial inverter/converter. 
                 Tidy up config.h file.
                 Send #5007 3 times at startup and then every 50th heartbeat.
v2.34 2019-10-15 Move typedef struct DateTime_t to global scope in line with VS Code/Platform IO.  
v2.35 2019-10-18 Add pre-defined ESP32 board variants. TargetBoard >> Target_Board typo Paul Atherton. 
v2.36 2019-10-30 Optimise WiFi amd BT read/send as per excellent mavesp8266 bridge by Tridge.
                 Add support for ESP8266. 
v2.41 2019-11-08 Fix STA mode no-connect loop 
      2019-11-08 Make AutoAP optional    
v2.42 2019-11-09 Add support for GCS-side simultaneous WiFi and BT telemetry option 
v2.43 2019-11-10 Tidy up WiFi Setup for auto AP failover. 
                 Support for 2 new ESP32 board variants, complements of Noircogi.     
      2019-11-11 Implement Auto RSSI selection(Order of precedence #109, then #65 then #35) 
      2019-11-11  Support AutoBaud up to 921600. 
v2.44 2019-11-12  Include Target0815 recommended reset after STA fail to connect.     
v2.45 2019-11-12  Augment mission debugging for athertop.  
      2019-11-13  Move #endif outside } in SetupWiFi         
v2.46 2019-11-16  A few cosmetic improvements     
v2.47 2019-12-23  For ESP32 Dev Module, use pin 27 for F.Port tx, 
                   because boot fails if pin 12 pulled high   

v2.48 2019-12-17 Option for SiK #109, if RSSI is already in %, i.e. not relative to 2.54 
                 Added #define Rssi_In_Percent 
      2019-12-31 Changes for PlatformIO compatibility 
      2020-01-02 ESP32 Dev Board change again for stability - F.Port pins rx=13, tx=14 
v2.49 2020-01-07 Move baud, ssid and BT settings to top of config.h for convenience 
v2.50 2020-01-12 AutoAP: Activate udp broadcast on AP dhcp allocated IP subnet.
                 Eliminate annoying periodic "Stabilized Flight Mode" announcements.
                 Further localise options in to logical groups.     
      2020-01-13 Revert max rssi to 254. 255 is invalid/unknown in ardupilot  
v2.51 2020-01-18 Make default rssi 69% for SiK radios, like RFD900x. PR by Hasi123   
v2.52 2020-01-21 Support web OTA. Rehash STA to AP failover using reboot - now stable. 
v2.53 2020-01-25 ESP8266 (Node MFU 12F) debugged, tested. ESP8266 OTA included. 
v2.54 2020-01-27 ESP8266 inverted single-wire enabled, like Teensy. 
      No hardware invert/convert required.                     
v2.54a 2020-01-28 Setup OTA password in config.h   
v2.54b 2020-01-30 Correct irritating warnings  
v2.55  2020-02-04 Add RFD900X TXMOD ESP8266 variant    
v2.56.1  2020-02-26 Add web interface to allow settings/parameter changes 
v2.56.2  2020-02-27 STM32F103C / Blue Pill / Maple Mini deprecated. Tidy up Teensy3.x warnings. 
v2.56.3 2020-03-03 Minor ESP8266 variants logic check 
v2.56.4 2020-03-04 Remove spurious debugging code affecting F.Port Thanks pascale dragos.     
v2.56.5 2020-03-09 Reduce rssi timing cycle to 350mS from 700mS. 
v2.57   2020-03-15 Fix RFD900/TXMOD status LED. SoftwareSerial for ESP32.  
v2.58   2020-03-17 Option to work around apparent bug in Mavlink V2 Library. Tolerate crcout errors.
                   This fixes failure to parse certain mavlink messages, including #226 RPM 
                   Needs more investigation. Use with caution!
v2.58.1 2020-03-18 Improve user options on hw/sw serial 
v2.58.2 2020-03-20 Stable. Lots of nice, small tweaks. Exp. code for inherent 1-wire on ESP
v2.58.3 2020-03-22 Deactivate experimental crcout error tolerance for general use. My bad. 
v2.58.4 2020-03-25 RPM fixed (library path). 
v2.58.5 2020-03-28 Add //#define sdBuiltin to optionally remove all SD support at compile time.
                   This is especially useful for PlatformIO on ESP8266.   
v2.59.1 2020-04-02 Support for QLRS (rssi) by giacomo892. Style and function improvements to web
                   interface.  
v2.59.2 2020-04-21 Some structural tidying up.   
v2.59.3            Main loop minor fix.  
                   wifiBuiltin and btBuiltim macros added.  
        2020-04-28 GetBaud(mav_rxPin) fix. Thanks has1123. 
v2.59.4 2020-05-08 Broadcast on the subnet we attached to. Can't assume 192.168.1/24 :) 
        Patch by Stefan Arbes.   
v2.59.5 2020-05-11 Fixed TCP_LocalPort initialisation 
v2.59.6 2020-05-20 Improve WiFi start button debounce . Add Debug_SRAM.     
v2.60.0 2020-05-23 Deactivate BT to reclaim critical SRAM during web support. Patch by Scott P.
                   Reboot on Cancel web settings page
v2.60.1 2020-05-24 Schedule fr 0x5007 params individually, fixes periodic flight-mode announcement.
                   Schedule fr 0xf101 rssi when mavlink #109, #65 or #35 arrives. Avg < 200 mS. Fixes
                   periodic "telemetry lost".    
v2.60.2 2020-05-25 Added RSSI_Pacemaker option to help prevent "telemetry lost". Default period 200 mS.   
v2.60.3 2020-05-25 Added new variant for ESP8266 - ESP-12F WEMOS D1 Mini    
        2020-05-30 Added some support for PitLab flight stack 
v2.60.4 2020-06-01 Half-duplex MavLite on F.Port - phase 1   
v2.61.0 2020-06-29 Important patch of WiFi TCP client for graceful close/reopen on loss of signal
                    Half-duplex MavLite on F.Port - phase 2 - work in progress!
                    Filter out heartbeats from Onboard_Controllers(18)
                    #define PitLab to force frame_type = 1
                    Monitor GCS heartbeat  
v2.61.1             Change references to pin 12 for all ESP32 variants   
                    Mavlink phase 2 working  
        2020-07-15  FPortClass established   
v2.61.2 2020-07-24  Web settings - value for st/ap failover not picked up. Should be STA/AP not STA_AP.
                    Display WiFi Mode at startup.
                    Emphasise EEPROM settings reset for clarity.
v2.61.3 2020-07-28  Add variant for LILYGO® TTGO T-Display ESP32 1.14 Inch Colour LCD 
v2.61.4 2020-07-30  Added display up/down scrolling on touch pins for ESP32    
v2.61.5 2020-07-30  Added display up/down scrolling pins for ESP8266 
v2.61.6             hasi123 patch for SiK radio rssi restored. It endures! 
                    Fix LogScreenPrint() of number.            
        2020-08-05  Add #define UDP_Broadcast as default to support > 1 UDP concurrent sessions 
v2.61.7 2020-08-10  Support board buttons on TTGO T-Display ESP32 for display up/down scrolling
                    More bi-directional mavlite.
v2.61.8 2020-08-10  F.Port telemetry to SD card option.  
v2.61.9 2020-09-04  Tidy up serial gcs uplink capability. Add outgoing TCP client capability. 
                    Improve web setup data vetting. 
                    Show last line properly & reverse scroll buttons on ST7789.
                    Ignore heartbeats from or Gremsy Gimbal(26).       
v2.62.0 2020-09-08  Add support for multiple incoming tcp clients (GCSs, trackers..)
                    Always broadcast UDP on the /24 subnet. 
v2.62.1 2020-09-10  Restore #defined UDP_broadcast option :)    
v2.62.2 2020-09-15  Add support for multi targeted UDP clients.
                    Flush UDP buffer after send. 
                    Always broadcast heartbeat for UDP.   
v2.62.3 2020-09-17  AP channel change fixed.
                    Display remote IP fixed. 
v2.62.4 2020-09-16  Fix BT slave name truncated by 1 chr
                    Improve when BT disabled to free up SRAM for web support, also #undef btBuiltin     
v2.62.8 2020-10-07  Improve display scrolling
        2020-10-12  Tested WiFi TCP output to AntTracker v2.15   
v2.62.9  2020-10-14 Add support for LILYGO® TTGO T2 SD TFT Colour 
v2.63.0 2020-10-26  S.Port bug introduced here. Add support for FrSky ISRM/ACCESS F.Port receivers (like Archer)  
v2.63.1 2020-10-29  Add option "no FrSky port support". Fw becomes "Mavlink Switch".
                    Restore Bluetooth support in web interface.   
v2.63.2 2020-11-02  Add switchable flight info page on display                         
        2020-10-31  For mark home include GPS 3D Fix                                        
v2.63.3 2020-11-04  Enumerate info page pins for more variants    
        2020-11-11  Info page working for all supported display types 
v2.63.4 2020-11-17  Support simultaneous wifi uplink and downlink (AP_STA mode)   
v2.63.5 2020-12-04  Support added for ILI9341 2.8" 320 x 240 colour display 
                    Basic HUD for flight info display. Needs more work.           
v2.63.6 2020-12-08  Add home arrow and tidy up hud.
                    Use ap_rssi (even in air  mode) on hud      
v2.63.7 2020-12-10  All display types tested, good    
v2.63.8 2020-12-14  SSD1306 I2C OLED display option implemented for Teensy 3x.    
v2.63.9/10 2020-12-18  Mavlite support in ground mode.
                       Add Mav_Debug_Range    
v2.63.11   2020-12-26  Documentation change, uplink is to FC, downlink is to GCS 
           2021-01-09  Tidy up MavLITE, add command_ack 
           2020-01-10  Web page downlink/uplink convention reversed   
v2.63.12   2021-01-22  Debug and test simultaneous udp wifi uplink and downlink  
v2.64.00   2021-02-10  Upgrade to F.Port v2.3.7. Tests good.
v2.64.01   2021-02-12  Add support for FrSky S/Fport udp out
v2.64.02   2021-02-19  Start using GitHub Tags
                       Embed version number constant  
v2.64.03   2021-02-19  Correct char width for ST7789 display                   
                       FrSky, uom was cA, now dA for current
                       Correct HUD current
           2021-02-27  Enable FrSky UDP out            
V2.64.4    2021-03-01  Add serial port polarity detection and auto invert.
                       Add auto FrSky serial port speed detection option. S.Port vs F.Port.
           2021-03-04  Hud rssi blank fix. 
                       Always auto detect FrSky serial speed. Remove option. 
           2021-03-11  fport1 || fport2  
V2.64.5    2021-03-21  Two small PRs by Risto and a small patch to assist disply definition on Dev Kit    
V2.64.6    2021-03-25  Fix screen scroll low limit when actve row < screen height  
V2.64.7    2021-03-30  Update getPolarity() technique. Minor, for very slow baud rates.                                                                                                                                                                                                                                                                                           
*/
