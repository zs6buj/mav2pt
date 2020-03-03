/*
Complete change log and debugging options are at the bottom of this tab
     
v2.56.1  2020-02-26 Add web interface to allow settings/parameter changes 
v2.56.2  2020-02-27 STM32F103C / Blue Pill / Maple Mini deprecated. Tidy up Teensy3.x warnings.             
*/
//===========================================================================================
//
//                   PLEASE SELECT YOUR DEFAULT OPTIONS BELOW BEFORE COMPILING
//
//===========================================================================================

#define webSupport                      // ESP only. Enable wifi web support, including OTA firmware updating. Browse to IP.
#define webPassword      "changeme!"    // Web password 
//#define Reset_Web_Defaults            // Settings in eeprom


//#define AutoBaud                      // Auto detect Mavlink serial-in baud rate
                                        // NOTE: Set Baud = 57600 for Dragonlink and RFD900X
#define mvBaudFC               57600    // Mavlink to/from the flight controller - max 921600 - must match FC or long range radio
#define frBaud                 57600    // S.Port baud setting - default 57600 


// Do not enable for FlightDeck
#define PlusVersion  // Added support for 0x5009 Mission WPs, 0x50F1 Servo_Channels, 0x50F2 VFR_Hud

//=================================================================================================
//           D E F A U L T   T R A N S L A T I O N   M O D E   S E T T I N G S   
//=================================================================================================       
// Choose only one of these three translation modes
#define Ground_Mode          // Translator between Taranis and LRS transceiver (like Dragonlink, ULRS, RFD900...)
//#define Air_Mode             // Translator between FrSky receiver (like XRS) and Flight Controller (like Pixhawk)
//#define Relay_Mode           // Translator between LRS tranceiver (like Dragonlink) and FrSky receiver (like XRS) in relay box on the ground


//=================================================================================================
//           D E F A U L T   F L I G H T   C O M P U T E R    I / O   S E T T I N G S   
//=================================================================================================
// Choose only one of these default Flight-Controller-side I/O channels 
// How does Mavlink telemetry enter this translator?
#define FC_Mavlink_IO  0    // Serial Port (default)         
//#define FC_Mavlink_IO  1    // BlueTooth Classic - ESP32 only
//#define FC_Mavlink_IO  2    // WiFi - ESP32 or ESP8266 only
//#define FC_Mavlink_IO  3    // SD Card / TF - ESP32 only


//=================================================================================================
//           D E F A U L T   G R O U N D S T A T I O N    I / O   S E T T I N G S   
//=================================================================================================
// Choose only one of these default GCS-side I/O channels
// How does Mavlink telemetry leave this translator?
// These are optional, and in addition to the S.Port telemetry output
//#define GCS_Mavlink_IO  0    // Serial Port  - Only Teensy 3.x and Maple Mini have Serial3     
//#define GCS_Mavlink_IO  1    // BlueTooth Classic - ESP32 only
//#define GCS_Mavlink_IO  2    // WiFi - ESP32 or ESP8266 only
//#define GCS_Mavlink_IO  3    // WiFi AND Bluetooth simultaneously - ESP32 only

  #ifndef GCS_Mavlink_IO
    #define GCS_Mavlink_IO  9    // NONE (default)
  #endif

// NOTE2: The Bluetooth class library uses a lot of application memory. During Compile/Flash
//        You may need to select Tools/Partition Scheme: "Minimal SPIFFS (1.9MB APP ...) or similar

//#define GCS_Mavlink_SD       // SD Card - ESP32 only - mutually inclusive with other GCS I/O



//=================================================================================================
//                      D E F A U L T   B L U E T O O T H   S E T T I N G S   
//=================================================================================================

//#define BT_Mode  1           // Master Mode - we advertise the "host" name and slave connects to us
#define BT_Mode  2           // Slave Mode - we connect as slave to a master
#define BT_ConnectToName     "Crossfire 0277"  // Example


//=================================================================================================
//                            D E F A U L T   W  I  F  I   S E T T I N G S   
//=================================================================================================

#define HostName             "MavToPass"        // This translator's host name
#define APssid               "MavToPassthru"    // The AP SSID that we advertise   ====>
#define APpw                 "password"         // Change me!
#define APchannel            9                  // The wifi channel to use for our AP
#define STAssid              "OmegaOffice"      // Target AP to connect to         <====
#define STApw                "changeme!"        // Target AP password        

#define ESP8266_Invert_SPort_To_Onewire  // activated = default
#define Start_WiFi                       // Start WiFi at startup, override startWiFi Pin

// Choose one default mode for ESP only - AP means advertise as an access point (hotspot). STA means connect to a known host
//#define WiFi_Mode   1  //AP            
//#define WiFi_Mode   2  // STA
#define WiFi_Mode   3  // STA failover to AP

// Choose one default protocol - for ESP32 only
//#define WiFi_Protocol 1    // TCP/IP
#define WiFi_Protocol 2    // UDP   

//=================================================================================================
//                              O T H E R   O P T I O N S  
//=================================================================================================

//#define Battery_mAh_Source  1  // Get battery mAh from the FC - note both rx and tx lines must be connected      
//#define Battery_mAh_Source  2  // Define bat1_capacity and bat2_capacity below and use those 
const uint16_t bat1_capacity = 5200;       
const uint16_t bat2_capacity = 0;
#define Battery_mAh_Source  3         // Define battery mAh in the LUA script on the Taranis/Horus - Recommended


#define SPort_Serial        1         // Teensy port1=pin1, port3=pin8. The default is Serial 1, but 3 is possible 



// RSSI_Source is Automatic. Order of precedence: 
//      First:  #109 SiK style RADIO_STATUS
//      Second: #65 RC_CHANNELS
//      Third:  #35 RC_CHANNELS_RAW

#define RSSI_Override                // Dummy RSSI - fixed at 70%                                                                                                                    

//#define Rssi_In_Percent             // Un-comment if RSSI is already %, not relative to (254/100)

// Status_Text messages place a huge burden on the meagre 4 byte FrSky telemetry payload bandwith
// The practice has been to send them 3 times to ensure that they arrive unscathed at the receiver
//  but that makes the bandwidth limitation worse and may crowd out other message types. Try without
//  sending 3 times, but if status_text gets distorted, un-comment the next line
//#define Send_status_Text_3_Times

//#define Send_Sensor_Health_Messages

//#define Request_Missions_From_FC    // Un-comment if you need mission waypoint from FC - NOT NECESSARY RIGHT NOW

//#define Data_Streams_Enabled        // Requests data streams from FC. Requires both rx and tx lines to FC. Rather set SRn in Mission Planner
#define Max_Waypoints  256          // Note. This is a global RAM trade-off. If exceeded then Debug message and shut down

//=================================================================================================                             
//                          S E L E C T   E S P   B O A R D   V A R I A N T   
//=================================================================================================
//#define ESP32_Variant     1    //  ESP32 Dev Module - there are several sub-variants that work
//#define ESP32_Variant     2    //  Wemos® LOLIN ESP32-WROOM-32_OLED_Dual_26p
//#define ESP32_Variant     3    //  Dragonlink V3 slim with internal ESP32 - contributed by Noircogi
#define ESP32_Variant     4    //  Heltec Wifi Kit 32 - contributed by Noircogi

//#define ESP8266_Variant   1   // NodeMCU ESP 12F - choose "NodeMCU 1.0(ESP-12E)" board in the IDE
#define ESP8266_Variant   2   // ESP-F Use me for RFD900X TX-MOD - use generic ESP8266 board on IDE

//================================== Set your time zone here ======================================
// Date and time determines the TLog file name only
//const float Time_Zone = 10.5;    // Adelaide, Australia
const float Time_Zone = 2.0;    // Jo'burg
bool daylightSaving = false;


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
//                              CHECK #define OPTIONS LOGIC
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
    #if (SPort_Serial  == 3)    
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

  #if (not defined ESP32) && (not defined ESP8266) 
     #if (FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3) || (defined webSupport)
       #error WiFi and webSupport only work on an ESP32 or ESP8266 board
     #endif  
  #endif
  
  #if (not defined ESP32) 
     #if (FC_Mavlink_IO == 1) || (GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 3)
       #error Bluetooth works only on an ESP32 board
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
    #ifndef WiFi_Protocol
      #error Please define WiFi_Protocol
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

  #if (ESP8266_Variant == 1)       // NodeMCU ESP 12F 
    #ifndef D3
        #error Please choose the NodeMCU 1.0(ESP-12E Module) board in the IDE
    #endif
  #endif  

    #if (ESP8266_Variant == 2)     // ESP-F for RFD900X TX-MOD
    #ifdef D3
        #error Please choose the Generic ESP8266 Module board in the IDE
    #endif
  #endif  
//=================================================================================================   
//                S E T T I NG S   A N D   O P T I O N S   S T R U C T U R E
//=================================================================================================

    typedef enum trmode_set { ground = 1 , air = 2, relay = 3 } trmode_t;                    // translator operation mode
    typedef enum fc_io_set { fc_ser = 0, fc_bt = 1 , fc_wifi = 2, fc_sd = 3 } fc_io_t;
    typedef enum gs_io_set { gs_ser = 0, gs_bt = 1 , gs_wifi = 2, gs_wifi_bt = 3, gs_none = 9} gs_io_t; 
    typedef enum gs_sd_set { gs_off = 0 , gs_on = 1} gs_sd_t; 
    typedef enum wfmode_set { ap = 1 , sta = 2, sta_ap = 3 } wfmode_t;   
    typedef enum wf_proto_set { tcp = 1 , udp = 2, } wf_proto_t; 
    typedef enum btmode_set { master = 1, slave = 2 } btmode_t;

    typedef struct  {
      byte          validity_check;  // 0xdc 
      trmode_t      trmode;         
      fc_io_t       fc_io;
      gs_io_t       gs_io;   
      gs_sd_t       gs_sd;
      wfmode_t      wfmode;
      wf_proto_t    wfproto;
      uint32_t      baud;
      uint8_t       channel;
      char          apSSID[30];
      char          apPw[20];
      char          staSSID[30];
      char          staPw[20];
      char          host[20];     
      uint16_t      tcp_localPort;
      uint16_t      udp_localPort;
      uint16_t      udp_remotePort;
      btmode_t      btmode;
      char          btSlaveConnectTo[20];
      uint8_t       rssi_override;
      bool          web_support;  // this flag is not saved in eeprom
      char*         trmode1;      //ground
      char*         trmode2;      //air
      char*         trmode3;      //relay      
      char*         fc_io0;       //ser
      char*         fc_io1;       //bt
      char*         fc_io2;       //wifi 
      char*         fc_io3;       //sd       
      char*         gs_io0;       //ser
      char*         gs_io1;       //bt
      char*         gs_io2;       // wifi
      char*         gs_io3;       // wifi_bt 
      char*         gs_io9;       // none     
      char*         gs_sd0;       // off 
      char*         gs_sd1;       // on      
      char*         wfmode1;      // ap 
      char*         wfmode2;      // sta
      char*         wfmode3;      // sta_ap     
      char*         wfproto1;     // tcp
      char*         wfproto2;     // udp       
      char*         btmode1;      // master
      char*         btmode2;      // slave 
      char*         rssioverride; //rssi override  
      
      } settings_struct_t;
      
    settings_struct_t set;  


//=================================================================================================   
//                          P L A T F O R M   D E P E N D E N T   S E T U P S
//================================================================================================= 

  
#if defined TEENSY3X               // Teensy3x
  #define MavStatusLed  13
  #define BufStatusLed  14
  #define FC_Mav_rxPin  9  
  #define FC_Mav_txPin  10
 // Fr_txPin (SPort)    1            Hard wired single wire to Taranis/Horus or XSR receiver
 
#elif defined ESP32                 // ESP32 Platform
  #if (ESP32_Variant == 1)          // ESP32 Dev Module
    #define MavStatusLed  02        // Onboard LED
    #define BufStatusLed  27        // untested pin
    #define FC_Mav_rxPin  16        // Mavlink to FC
    #define FC_Mav_txPin  17        // Mavlink from FC
    #define Fr_rxPin      13        // SPort - Not used in single wire mode
    #define Fr_txPin      4        // SPort - Use me  
    #define SDA           21        // I2C OLED board
    #define SCL           22        // I2C OLED board
    #define i2cAddr      0x3C       // I2C OLED board
    int16_t wifi_rssi;    
    uint8_t startWiFiPin = 15;      // D15
    uint8_t WiFiPinState = 0;
    /*  
      SPI/CS                       Pin 05   For optional TF/SD Card Adapter
      SPI/MOSI                     Pin 23   For optional TF/SD Card Adapter
      SPI/MISO                     Pin 19   For optional TF/SD Card Adapter
      SPI/SCK                      Pin 18   For optional TF/SD Card Adapter  
    */

  #endif

  #if (ESP32_Variant == 2)          // Wemos® LOLIN ESP32-WROOM-32_OLED_Dual_26p
    #define MavStatusLed  15        // No Onboard LED
    #define BufStatusLed  99        // None  
    #define FC_Mav_rxPin  25        // Mavlink to FC
    #define FC_Mav_txPin  26        // Mavlink from FC
    #define Fr_rxPin      12        // SPort - Not used in single wire mode
    #define Fr_txPin      14        // SPort - Use me
    #define SDA           05        // I2C OLED board
    #define SCL           04        // I2C OLED board
    #define i2cAddr      0x3C       // I2C OLED board
    int16_t wifi_rssi;    
    uint8_t startWiFiPin = 13;     
    uint8_t WiFiPinState = 0;
  #endif

  #if (ESP32_Variant == 3)          // Dragonlink V3 slim with internal ESP32
    #if defined mvBaudFC
      #undef mvBaudFC
      #define mvBaudFC 115200       // Force baud rate to DragonLink rate
    #endif
    #define MavStatusLed  18        // Blue LED
    #define BufStatusLed  19        // Green LED
    #define FC_Mav_rxPin  16        // Mavlink to FC
    #define FC_Mav_txPin  17        // Mavlink from FC
    #define Fr_rxPin      12        // SPort - Not used in single wire mode
    #define Fr_txPin      01        // SPort - Use me 
    #define SDA           05        // I2C OLED board
    #define SCL           04        // I2C OLED board
    #define i2cAddr      0x3C       // I2C OLED board
    int16_t wifi_rssi;    
    uint8_t startWiFiPin = 13;     
    uint8_t WiFiPinState = 0;
  #endif
  
  #if (ESP32_Variant == 4)          // Heltec Wifi Kit 32 (NOTE! 8MB) 
    #define MavStatusLed  25        // Onboard LED
    #define BufStatusLed  99          
    #define FC_Mav_rxPin  27        // Mavlink to FC
    #define FC_Mav_txPin  17        // Mavlink from FC
    #define Fr_rxPin      12        // SPort - Not used in single wire mode
    #define Fr_txPin      14        // SPort - Use me 
    #define SDA           04        // I2C OLED board
    #define SCL           15        // I2C OLED board
    #define i2cAddr      0x3C       // I2C OLED board
    #define OLED_RESET    16        // RESET here so no rest lower down
    int16_t wifi_rssi;    
    uint8_t startWiFiPin = 15;      // D15
    uint8_t WiFiPinState = 0;
  #endif
  
#elif defined ESP8266                    // ESP8266 Platform

  #if (ESP8266_Variant == 1)        // Node MFU 12F
    #define MavStatusLed  D0        // Mavlink Status LED
    #define BufStatusLed  99        // None
   //                     D4        // TXD1 - Debug log out                            
    #define FC_Mav_rxPin  D9        // RXD0 default  
    #define FC_Mav_txPin  D10       // TXD0 default    
    #define Fr_rxPin      D5        // GP10 SPort - Not used in single wire mode
    #define Fr_txPin      D6        // GPIO SPort - Use me 
    #define SCL           D1        // I2C OLED board   
    #define SDA           D2        // I2C OLED board

    #define i2cAddr      0x3C       // I2C OLED board
    int16_t wifi_rssi;   
    uint8_t startWiFiPin = D3;      
    uint8_t WiFiPinState = 0;
  #endif
  
  #if (ESP8266_Variant == 2)        // ESP-F RFD900X TX-MOD - use generic esp8266

    //                         GPIO as per node mfu
    static const uint8_t D0   = 16;   // SCL - optional
    static const uint8_t D1   = 5;    // SDA - optional
    static const uint8_t D2   = 4;    // SPort half-duplex
    static const uint8_t D3   = 0;    // Flash
    static const uint8_t D4   = 2;    // TXD1 optional debug out
    static const uint8_t D5   = 14;   // SPort rx (unused in half-duplex)
    static const uint8_t D6   = 12;   // P2-3 exposed dual row of pins
    static const uint8_t D7   = 13;   // CTS
    static const uint8_t D8   = 15;   // RTS
    static const uint8_t D9   = 3;    // RXD0
    static const uint8_t D10  = 1;    // TXD0

    #define MavStatusLed  D7        // Mavlink Status LED
    #define BufStatusLed  99        // None
    //                    D4        // TXD1 - Serial1 default debug log out                            
    #define FC_Mav_rxPin  D9        // RXD0 default  
    #define FC_Mav_txPin  D10       // TXD0 default    
    #define Fr_rxPin      D5        // SPort - Not used in single wire mode
    #define Fr_txPin      D2        // SPort half-duplex inverted - Use me 
    #define SCL           D0        // I2C OLED board   
    #define SDA           D1        // I2C OLED board

    #define i2cAddr      0x3C       // I2C OLED board
    int16_t wifi_rssi;   
    uint8_t startWiFiPin = D8;      
    uint8_t WiFiPinState = 0;
  #endif  
  
#endif

  //=================================================================================================   
  //                            E E P R O M    S U P P O R T   -   ESP Only - for now
  //================================================================================================= 

  #if (defined ESP32) || (defined ESP8266)
    #include <EEPROM.h>       // To store AP_Failover_Flag and webSupport Settings
    #define EEPROM_SIZE 160   // 1 + 159 bytes, addr range 0 thru 160
  #endif
    
  //=================================================================================================   
  //                             S D   C A R D   S U P P O R T   -   ESP Only - for now
  //================================================================================================= 
  #if (defined ESP32)  || (defined ESP8266)

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

    char     cPath[40];
    std::string   fnPath[30];
    uint8_t  fnCnt;
    uint16_t sdReadDelay = 10;  // mS   Otherwise the reads run through unnaturally quickly

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
  //                                 O L E D   S U P P O R T    E S P  O N L Y - for now
  //================================================================================================= 
  #if (defined ESP32 || defined ESP8266)  
  
    #if not defined SD_Libs_Loaded   //  by SD block
      #include <SPI.h>                // for SD card and/or OLED
      #include <Wire.h>
    #endif  

    #include <Adafruit_SSD1306.h> 
    
    #define max_col  22
    #define max_row   8
    // 8 rows of 21 characters

    struct OLED_line {
      char OLx[max_col];
      };
  
     OLED_line OL[max_row]; 

    uint8_t row = 0;
    uint8_t col = 0;
  
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels
    // 8 rows of 21 characters

    // Declaration for an SSD1306 I2C display
    #ifndef OLED_RESET
      #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
    #endif  
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  #endif

  //=================================================================================================   
  //                          B L U E T O O T H   S U P P O R T - ESP32 Only
  //================================================================================================= 

  #if (defined ESP32) 

    #include "BluetoothSerial.h"
    #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
      #error Bluetooth is not enabled! Please run `make menuconfig in ESP32 IDF` 
    #endif

    BluetoothSerial SerialBT;

  #endif  

  //=================================================================================================   
  //                           W I F I   S U P P O R T - ESP32 and ES8266 Only
  //================================================================================================= 

    uint16_t TCP_localPort = 5760;
    uint16_t UDP_localPort = 14555;     
    uint16_t UDP_remotePort = 14550;      
    bool FtRemIP = true;

  #if (defined ESP32 || defined ESP8266) 

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
      #include <WiFi.h>  
      #include <WiFiClient.h>
      #if defined webSupport
        #include <WebServer.h> 
        #include <Update.h> 
        WebServer server(80);          
      #endif      
      #include <WiFiAP.h>  
    #endif

    #if defined ESP8266
      #include <ESP8266WiFi.h>   // Includes AP class
      #include <WiFiClient.h>
      #if defined webSupport
        #include <ESP8266WebServer.h>    
        ESP8266WebServer server(80);
      #endif      
    #endif
    

    #include <WiFiUdp.h>    

    WiFiClient WiFiSTA;   
       
    WiFiServer TCPserver(set.tcp_localPort);

    IPAddress udp_remoteIP(192, 168, 1, 255);    // Declare UDP broadcast on your likely LAN subnet
    WiFiUDP UDP;       // Create UDP object    
         
    IPAddress localIP;   // tcp and udp
    
  #endif  // end of ESP32 and ESP8266
  
//=================================================================================================   
//                                 S E R I A L
//=================================================================================================

#if (defined ESP32)  
  #define Debug               Serial         // USB
  #define mvSerialFC          Serial2        //  RXD0 and TXD0 
#elif (defined ESP8266)
  #define Debug               Serial1        //  D4   TXD1 debug out  - no RXD1 !
  #define mvSerialFC          Serial         //  RXD0 and TXD0
  #include <SoftwareSerial.h>
  SoftwareSerial frSerial;  
#else                      // Non ESP
  #define Debug               Serial         // USB  
  #define mvSerialFC          Serial2   
#endif 

#if (defined TEENSY3X)      //  Teensy 3.1

  #if (SPort_Serial == 1) 
    #define frSerial              Serial1        // S.Port 
  #elif (SPort_Serial == 3)
    #define frSerial              Serial3        // S.Port 
 #else
    #error SPort_Serial can only be 1 or 3. Please correct.
  #endif 
#endif 

#if (not defined TEENSY3X) && (not defined ESP8266)     //  Not Teensy 3.1 and not ESP8266 , i.e. other boards
  #define frSerial              Serial1                 // S.Port 
#endif 


#if (defined TEENSY3X) || (defined MAPLE_MINI)  //  only these have 4 uarts at present
  //#define Enable_GCS_Serial  //  OPTION SET HERE   OPTION SET HERE   OPTION SET HERE 
#endif

#if defined Enable_GCS_Serial  // only Teensy and maple mini have 4 uarts at present

  #if (SPort_Serial == 3) 
   #error Mavlink_GCS and SPort both configured for Serial3. Please correct.
  #endif 
  
  #if (defined TEENSY3X)          
    #define mvSerialGCS             Serial3 
    #define mvBaudGCS               57600        // Use 57600
  #else
    #error Mavlink_GCS Serial not available for ESP32 or ESP8266  - no Serial3. Please correct.
  #endif
 
#endif

//=================================================================================================   
//                                 D E B U G G I N G   O P T I O N S
//=================================================================================================

//#define inhibit_SPort     // Use me to send only debug messages out of GPIO1/TX0 on ESP32_Variant 3, DL V3 internal ESP32

//#define Mav_Debug_All
//#define Frs_Debug_All
//#define Frs_Debug_Period
//#define Frs_Debug_Payload
//#define Mav_Debug_RingBuff
//#define Debug_Air_Mode
//#define Debug_Relay_Mode
//#define Mav_List_Params      // Use this to test uplink to Flight Controller 
//#define Mav_Debug_Params
//#define Frs_Debug_Params
//#define Debug_BT    
//#define Debug_FC_Down        // traffic down from FC to Ring Buffer
//#define Debug_FC_Up          // traffic up to FC from GCS
//#define Debug_GCS_Down       // traffic from RB to GCS
//#define Debug_GCS_Up         // traffic up from GCS to FC
//#define Mav_Debug_Servo
//#define Frs_Debug_Servo
//#define Debug_Rssi
//#define Mav_Debug_RC
//#define Frs_Debug_RC
//#define Mav_Debug_FC_Heartbeat
//#define Mav_Debug_GCS_Heartbeat
//#define Mav_Debug_MavToPass_Heartbeat
//#define Frs_Debug_APStatus
//#define Mav_Debug_SysStatus
//#define Debug_Batteries
//#define Frs_Debug_Home
//#define Mav_Debug_GPS_Raw     // #24
//#define Mav_Debug_GPS_Int     // #33
//#define Frs_Debug_LatLon
//#define Frs_Debug_YelYaw
//#define Frs_Debug_GPS_status
//#define Mav_Debug_Scaled_IMU
//#define Mav_Debug_Raw_IMU
//#define Mav_Debug_Hud
//#define Frs_Debug_Hud
//#define Mav_Debug_Scaled_Pressure
//#define Mav_Debug_Attitude
//#define Frs_Debug_Attitude
//#define Mav_Debug_StatusText
//#define Frs_Debug_StatusText    
//#define Mav_Debug_Mission
//#define Frs_Debug_Mission   
//#define Debug_SD    
//#define Mav_Debug_System_Time   
//#define Frs_Debug_Scheduler // - this debugger affects the performance of the scheduler when activated
//#define Decode_Non_Essential_Mav 
//#define Debug_Baud 
//#define Debug_Radio_Status  
//#define Debug_GCS_Unknown
//#define Debug_Param_Request_Read
//#define Mav_Show_Unknown_Msgs
//#define Mav_Print_All_Msgid
//#define Debug_Eeprom
//#define Debug_Web_Settings

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
v2.08 2019-06-16 Add SD/TF card support - wip!!  Added UTP protocol option 
v2.09 2019-06-30 WiFi activation push button momentary but one-time. 
      2019-07-03 Implemented optional TLog input and output via SD card.  
v2.10 2019-07-09 For PX4 flight stack only, send HB to FC every 2 seconds  
      2019-07-10 Radical redesign of S.Port scheduling algorithm. Support SD and WiFi/BT I/O simultaneously 
v2.11 2109-07-11 Auto determine target board. Publish "Universal" version 2.11. One source, four platforms / boards 
v2.12 2019-07-12 Add #define PlusVersion, comment out for FlightDeck   
v2.13 2019-08-13 UDP now working in Access Point mode 
v2.14 2019-07-17 PX4 flight stack only - fixed longitude typo    if (ap_lat24<0) should be if (ap_lon24<0)
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
v2.29 2019-09-24 Use #if (TargetBoard == 3) to define soft pins for mvSerialFC
v2.30 2019-09-26 Don't push #5007 into sensor table from #147. Push from #1 only.
v2.31 2019-09-30 Configurable declarations moved to config.h file
v2.32 2019-10-08 Source line corrupted in v2.17 affecting Relay Mode, fixed. Thank you burtgree! 
v2.33 2019-10-09 Don't invert ESP32 SPort in Relay Mode. Use commercial inverter/converter. 
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
v2.47 2019-12-23  For ESP32 Dev Module, use pin 27 for S.Port tx, 
                   because boot fails if pin 12 pulled high   

v2.48 2019-12-17 Option for SiK #109, if RSSI is already in %, i.e. not relative to 2.54 
                 Added #define Rssi_In_Percent 
      2019-12-31 Changes for PlatformIO compatibility 
      2020-01-02 ESP32 Dev Board change again for stability - S.Port pins rx=13, tx=14 
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
*/
