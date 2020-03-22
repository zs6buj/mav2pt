
// Debugging options are at the bottom of this tab

// ******************************* Please select your options here before compiling *******************************

// Do not enable for FlightDeck
#define PlusVersion  // Added support for 0x5009 Mission WPs, 0x50F1 Servo_Channels, 0x50F2 VFR_Hud

// Choose one only of these three modes
#define Ground_Mode          // Converter between Taranis and LRS tranceiver (like Dragonlink, ULRS, RFD900...)
//#define Air_Mode             // Converter between FrSky receiver (like XRS) and Flight Controller (like Pixhawk)
//#define Relay_Mode           // Converter between LRS tranceiver (like Orange) and FrSky receiver (like XRS) in relay box on the ground

// Choose one only of these Flight-Controller-side I/O channels 
// How does Mavlink telemetry enter the converter?
#define FC_Mavlink_IO  0    // Serial Port (default)         
//#define FC_Mavlink_IO  1    // BlueTooth Classic - ESP32 only
//#define FC_Mavlink_IO  2    // WiFi - ESP32 only
//#define FC_Mavlink_IO  3    // SD Card / TF - ESP32 only

// Choose one only of these GCS-side I/O channels
// How does Mavlink telemetry leave the converter?
// These are optional, and in addition to the S.Port telemetry output
//#define GCS_Mavlink_IO  9    // NONE (default)
//#define GCS_Mavlink_IO  0    // Serial Port  - Only Teensy 3.x and Maple Mini  have Serial3     
//#define GCS_Mavlink_IO  1    // BlueTooth Classic - ESP32 only
//#define GCS_Mavlink_IO  2    // WiFi - ESP32 only

//#define GCS_Mavlink_SD      // SD Card  - for ESP32 only

//#define Start_WiFi         // Start WiFi at startup, override startWiFi Pin

// Choose one protocol - for ESP32 only
//#define WiFi_Protocol 1    // TCP/IP
#define WiFi_Protocol 2    // UDP     useful for Ez-WiFiBroadcast in STA mode

// Choose one mode for ESP only - AP means advertise as an access point (hotspot). STA means connect to a known host
#define WiFi_Mode   1  //AP            
//#define WiFi_Mode   2  // STA

//#define Battery_mAh_Source  1  // Get battery mAh from the FC - note both rx and tx lines must be connected      
//#define Battery_mAh_Source  2  // Define bat1_capacity and bat2_capacity below and use those 
const uint16_t bat1_capacity = 5200;       
const uint16_t bat2_capacity = 0;
#define Battery_mAh_Source  3  // Define battery mAh in the LUA script on the Taranis/Horus - Recommended

#define SPort_Serial        1         // The default is Serial 1, but 3 is possible 

#define RSSI_Source         0         // default FrSky receiver
//#define RSSI_Source         1         // Designated RC PWM channel - ULRS, QLRS, Dragonlink ....
//#define RSSI_Source         2         // RFD900x - frame #109 injected by SiK radio firmware into Mavlink stream
//#define RSSI_Source         3         // Dummy RSSI - fixed at 70%

// Status_Text messages place a huge burden on the meagre 4 byte FrSky telemetry payload bandwith
// The practice has been to send them 3 times to ensure that they arrive unscathed at the receiver
//  but that makes the bandwidth limitation worse and may crowd out other message types. Try without
//  sending 3 times, but if status_text gets distorted, un-comment the next line
//#define Send_Status_Text_3_Times

//#define Send_Sensor_Health_Messages
//#define AutoBaud                    // Auto detect telemetry baud - takes a few seconds
//#define Request_Missions_From_FC    // Un-comment if you need mission waypoint from FC - NOT NECESSARY RIGHT NOW

//#define Data_Streams_Enabled // Requests data streams from FC. Requires both rx and tx lines to FC. Rather set SRn in Mission Planner
#define Max_Waypoints  256     // Note. This is a global RAM trade-off. If exceeded then Debug message and shut down

// ****************************** Set your time zone here ******************************************
// Date and time determines the TLog file name only
//const float Time_Zone = 10.5;    // Adelaide
const float Time_Zone = 2.0;    // Jo'burg
bool daylightSaving = false;

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// ******************************************* Auto Determine Target Board *****************************************
//
//                Don't change anything here
//
#if defined (__MK20DX128__) || defined(__MK20DX256__)
  #define Target_Board   0      // Teensy 3.1 and 3.2    
      
#elif defined (__BluePill_F103C8__) ||  defined (MCU_STM32F103RB)
  #define Target_Board   1      // Blue Pill STM32F103C  
         
#elif defined (STM32_MEDIUM_DENSITY) 
  #define Target_Board   2      // Maple_Mini STM32F103C  
     
#elif defined (_BOARD_MAPLE_MINI_H_)
  // LeafLabs high density
  #define Target_Board   2      // Maple_Mini 

#elif defined STM32_HIGH_DENSITY
  // LeafLabs high density
  #define Target_Board   2      // Maple_Mini  
#elif defined ESP32

  #define Target_Board   3      // Espressif ESP32 Dev Module
  
#else
  #error "No board type defined!"
#endif

//***************************************************************************************
// Check #defines options logic  

#if defined PlusVersion
  #define Request_Mission_Count_From_FC // Needed for yaapu's mission/waypoint script
#endif

#if (Target_Board == 3) 
  #include <iostream> 
  #include <sstream> 
#endif

  #ifndef Target_Board
    #error Please choose at least one target board
  #endif

    #ifndef RSSI_Source
    #error Please choose a RSSI_Source
  #endif
  
  #if (Target_Board == 1) || (Target_Board == 3) // Blue Pill or ESP32 (UART0, UART1, and UART2)
    #if (SPort_Serial  == 3)    
      #error Board does not have Serial3. This configuration is not possible.
    #endif
  #endif

  #ifndef Battery_mAh_Source
    #error Please choose at least one Battery_mAh_Source
  #endif

  #if (Target_Board != 3) 
     #if (FC_Mavlink_IO == 1) || (GCS_Mavlink_IO == 1) || (FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2)
       #error WiFi or Bluetooth work only on an ESP32 board
     #endif  
  #endif
  
    #ifndef GCS_Mavlink_IO
      #define GCS_Mavlink_IO  9    // NONE (default)
    #endif
  
    #ifndef FC_Mavlink_IO
      #error Please choose at least one Mavlink FC IO channel
    #endif

    #if (Target_Board == 3)
      #ifndef WiFi_Mode 
        #error Please define WiFi_Mode
      #endif
    #endif  

    #if (Target_Board == 3)
      #ifndef WiFi_Protocol
        #error Please define WiFi_Protocol
      #endif
    #endif


// ************************* P L A T F O R M   D E P E N D E N T   S E T U P S **********************************
//********************************************* LEDS, OLED SSD1306, rx pin **************************************

  
#if (Target_Board == 0)           // Teensy3x
  #define MavStatusLed  13
  #define BufStatusLed  14
  #define FC_Mav_rxPin  9  
  #define FC_Mav_txPin  10
 // Fr_txPin (SPort)    1            Hard wired single wire to Taranis/Horus or XSR receiver
 
#elif (Target_Board == 1)         // Blue Pill
  #define MavStatusLed  PC13
  #define BufStatusLed  PC14
  #define FC_Mav_rxPin  PB11  
  #define FC_Mav_txPin  PB10  
 // Fr_txPin (SPort)    PA2          SPort hard wired tx to inverter/converter
 // Fr_txPin (SPort)    PA3          SPort hard wired rx to inverter/converter 
 
#elif (Target_Board == 2)         // Maple Mini
  #define MavStatusLed  33        // PB1
  #define BufStatusLed  34 
  #define FC_Mav_rxPin  8         // PA3  
  #define FC_Mav_txPin  9         // PA2 
 // Fr_txPin (SPort)    PA10         SPort hard wired tx to inverter/converter 
 // Fr_txPin (SPort)    PA9          SPort hard wired rx to inverter/converter   
#elif (Target_Board == 3)         // ESP32 Dev Module V2

  #define MavStatusLed  02        // Dev Board=02, TTGO OLED Battery board = 16, LilyGo Mini32 = No Onboard LED
  #define BufStatusLed  13          
  #define FC_Mav_rxPin  16        // Dev Board = 16, LilyGo Mini32 WROVER_B Dev4 = 26
  #define FC_Mav_txPin  17        // Dev Board = 17, LilyGo Mini32 WROVER_B Dev4 = 27 
  #define Fr_rxPin      12        // SPort - Dev Board = 12 - Use both for Air Mode or Relay Mode to inverter/converter
  #define Fr_txPin      14        // SPort - Dev Board = 14 - Use me for Ground Mode to Taranis/Horus 
  
  #include <SPI.h>                // for SD card or OLED
  #include <Wire.h>
  #include <Adafruit_SSD1306.h> 

  //  Put your SCL / SDA pin numbers for your I2C OLED board here
  #define SDA            21      // Dev board = 21 
  #define SCL            22      // Dev board = 22   
  #define i2cAddr        0x3C

  /*
  // Optional SPI interface pins for SD card adapter or SSD1306 OLED display
  #define CS            5        
  #define MOSI          23 
  #define MISO          19 
  #define SCK           18 
  */  
  
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  // 8 rows of 21 characters

  // Declaration for an SSD1306 I2C display
  #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  
#endif

//************************************************************************** 
//**************************** Bluetooth - ESP32 Only **********************

  #if (FC_Mavlink_IO == 1) || (GCS_Mavlink_IO == 1)  // Bluetooth
    #if (Target_Board == 3) // ESP32
      #include "BluetoothSerial.h"
      #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
        #error Bluetooth is not enabled! Please run `make menuconfig in ESP32 IDF` 
      #endif
    #else
      #error Bluetooth only available on ESP32
    #endif    
#endif

///************************************************************************** 
//***************************** WiFi - ESP32 Only ***************************

  #if ((FC_Mavlink_IO == 2) || (GCS_Mavlink_IO == 2))  // WiFi
  
    #include <WiFi.h>  
    #include <WiFiClient.h>
 
    #if (WiFi_Protocol == 2) 
      #include <WiFiUDP.h>
    #endif   
    
    int16_t wifi_rssi;    
    uint8_t startWiFiPin = 15;    // D15
    uint8_t WiFiPinState = 0;
    
    #if (WiFi_Mode == 1)  // AP
      #include <WiFiAP.h>  
      const char *APssid =    "Mav2Passthru";    // The AP SSID that we advertise  ====>
      const char *APpw =      "password";        // Change me!
    #endif
    
    #if (WiFi_Mode == 2)  //  STA
  //    const char *STAssid =     "TargetAPName";    // Target AP to connect to      <====
  //    const char *STApw =       "targetPw";      // Change me!

  //    const char *STAssid =     "EZ-WifiBroadcast";    // Target AP to connect to      <====
  //    const char *STApw =       "wifibroadcast";         

      const char *STAssid =     "TXMOD-54-DD-FE";    // Target AP to connect to      <====
      const char *STApw =       "txmod123";    

    #endif   

    WiFiClient wifi;   
    
    #if (WiFi_Protocol == 1)
      uint16_t tcp_Port = 5760;  
      WiFiServer server(tcp_Port);
    #endif 
    
    #if (WiFi_Protocol == 2)
   
      #if (FC_Mavlink_IO == 2)   // FC side
        uint16_t udp_localPort = 14550;
        uint16_t udp_remotePort = 14550;
        bool FtRemIP = true;
        IPAddress remoteIP =  (192, 168, 2, 2);   // First guess for EZ-WFB in STA mode. Will adopt IP allocated
        WiFiServer server(udp_localPort);     
      #endif
      
      #if (GCS_Mavlink_IO == 2)   // QGC side   
        uint16_t udp_localPort = 14550;
        uint16_t udp_remotePort = 14550;         
        bool FtRemIP = true;
        IPAddress remoteIP =  (192, 168, 4, 2); // We hand out this IP to the first client via DHCP
        WiFiServer server(udp_localPort);     
      #endif  
      
      WiFiUDP udp;       // Create udp object      
    #endif   
    
    IPAddress localIP;

    
 #endif  
  
//************************************************************************** 
//******************************* SD Card **********************************

  #if ((FC_Mavlink_IO == 3) || defined GCS_Mavlink_SD)  // SD Card

  // Pins generally   CS=5    MOSI=23   MISO=19   SCK=18    3.3V   GND   Dev Board, LilyGO/TTGO
 
  #include "FS.h"
  #include "SD.h"
  #include "SPI.h"
// Rememeber to change SPI frequency from 4E6 to 25E6, i.e 4MHz to 25MHz in SD.h otherwise MavRingBuff fills up 
// C:\Users\YourUserName\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.2\libraries\SD\src  
// bool begin(uint8_t ssPin=SS, SPIClass &spi=SPI, uint32_t frequency=25000000, const char * mountpoint="/sd", uint8_t max_files=5);  

char     cPath[40];
string   fnPath[30];
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
//************************************************************************** 
//********************************** Serial ********************************

#define Debug               Serial         // USB 
#define frBaud              57600          // Use 57600
#define mvSerialFC          Serial2        
uint16_t mvBaudFC     =     57600;         // Default    

#if (Target_Board == 0)      //  Teensy 3.1
 #if (SPort_Serial == 1) 
  #define frSerial              Serial1        // S.Port 
 #elif (SPort_Serial == 3)
  #define frSerial              Serial3        // S.Port 
 #else
  #error SPort_Serial can only be 1 or 3. Please correct.
 #endif 
#endif 

#if (Target_Board != 0)      //  Not Teensy 3.1, i.e. other boards
  #define frSerial              Serial1        // S.Port 
#endif 

#if (GCS_Mavlink_IO == 0) // Mavlink_GCS optional feature available for Teensy 3.1/2 and Maple Mini

  #if (SPort_Serial == 3) 
   #error Mavlink_GCS and SPort both configured for Serial3. Please correct.
  #endif 
  
  #if (Target_Board == 0) || (Target_Board == 2) // Teensy 3.x or Maple Mini
    #define mvSerialGCS             Serial3 
    #define mvBaudGCS               57600        // Use 57600
  #else
    #error Mavlink_GCS Serial not available for ESP32 or Blue Pill - no Serial3. Please correct.
  #endif
#endif


// ******************************** D E B U G G I N G   O P T I O N S ***************************************

//#define Mav_Debug_All
//#define Frs_Debug_All
//#define Frs_Debug_Period
//#define Frs_Debug_Payload
//#define Mav_Debug_RingBuff
//#define Debug_Air_Mode
//#define Debug_Relay_Mode
//#define Mav_List_Params      // Use this to test uplink to Flight Controller 
//#define Debug_BT    
//#define Debug_FC             // traffic down from FC to Ring Buffer
//#define Debug_GCS_Down       // traffic from RB to GCS
//#define Debug_GCS_Up         // traffic up from GCS to FC
//#define Mav_Debug_Params
//#define Mav_Debug_Servo
//#define Frs_Debug_Servo
//#define Debug_Rssi
//#define Mav_Debug_RC
//#define Frs_Debug_RC
//#define Mav_Debug_FC_Heartbeat
//#define Mav_Debug_GCS_Heartbeat
//#define Mav_Debug_Mav2PT_Heartbeat
//#define Frs_Debug_Params
//#define Frs_Debug_APStatus
//#define Mav_Debug_SysStatus
//#define Debug_Batteries
//#define Frs_Debug_Home
//#define Mav_Debug_GPS_Raw     // #24
//#define Mav_Debug_GPS_Int     // #33
//#define Frs_Debug_LatLon
//#define Frs_Debug_YelYaw
//#define Frs_Debug_GPS_Status
//#define Mav_Debug_Raw_IMU
//#define Mav_Debug_Hud
//#define Frs_Debug_Hud
//#define Mav_Debug_Scaled_Pressure
//#define Mav_Debug_Attitude
//#define Frs_Debug_Attitude
//#define Mav_Debug_StatusText
//#define Frs_Debug_Status_Text    
///#define Mav_Debug_Mission 
//#define Frs_Debug_Mission   
//#define Debug_SD    
//#define Mav_Debug_System_Time   
//#define Frs_Debug_Scheduler - this debugger affects the performance of the scheduler when activated
//#define Decode_Non_Essential_Mav 
//#define Debug_Baud 
//#define Debug_Radio_Status  
//#define Debug_Mission_Request_Int 
//#define Debug_GCS_Unknown


// *****************************************************************************************************************
