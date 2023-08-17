/*
=====================================================================================================================
     Mav2PT  (Mavlink To FrSky Passthru) Protocol Translator

 
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

  By downloading this software you are agreeing to the terms specified in this page and the spirit thereof.
    
    =====================================================================================================================

    Author: Eric Stockenstrom
    
    Inspired by original S.Port firmware by Rolf Blomgren
    
    Acknowledgements and thanks to Craft and Theory (http://www.craftandtheoryllc.com/) for
    the Mavlink / Frsky Passthru protocol

    Many thanks to yaapu for PRs, advice and testing, and his excellent LUA script

    Thanks also to athertop for advice and testing, and to florent for advice on working with FlightDeck

    Acknowledgement and thanks to the author of, and contributors to, mavESP8266 serial/Wifi bridge

    Thank you Scott Pritchett for refinements, especially around Dragonlink
    
    ======================================================================================================

    See: https://github.com/zs6buj/MavlinkToPassthru/wiki
    
    While the DragonLink and Orange UHF Long Range RC and telemetry radio systems deliver 
    a two-way Mavlink link, the FrSky Taranis and Horus hand-held RC controllers expect
    to receive FrSky S/FPort protocol telemetry for display on their screen.  Excellent 
    firmware is available to convert Mavlink to the native SPort protocol, however the 
    author is unaware of a suitable solution to convert to the Passthru protocol. 

    This protocol translator has been especially tailored to work with the excellent LUA 
    display interface from yaapu for the FrSky Horus, Taranis and QX7 controllers. 
    https://github.com/yaapu/FrskyTelemetryScript . The translator also works with the 
    popular FlightDeck product.
    
    The firmware translates APM or PX4 Mavlink telemetry to FrSky F/SPort passthru telemetry, 
    and is designed to run on an ESP32, ESP8266 or Teensy 3.2. The ESP32 implementation supports
    WiFI, Bluetooth and SD card I/O, while the ESP8266 supports only WiFi and SD I/O into and 
    out of the translator. So for example, Mavlink telemetry can be fed directly into Mission
    Planner, QGround Control or an antenna tracker.

    The performance of the translator on the ESP32 platform is superior to that of the other boards.
    However, the Teensy 3.x is much smaller and fits neatly into the back bay of a Taranis or Horus
    transmitter. The STM32F103C boards are no longer supported.

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

    The popular LUA telemetry scripts use RSSI to determine that a telemetry connection has been 
    successfully established between the 'craft and the Taranis/Horus. Be sure to set-up RSSI properly 
    before testing the system.

    ======================================================================================================

    Connections to boards depend on the board variant 

    Go to config.h tab and look for "S E L E C T   B O A R D   V A R I A N T" 

    NOTE : STM32 support is deprecated as of 2020-02-27 v2.56.2

    ======================================================================================================    
*/

#include <CircularBuffer.h>
#include <mavlink_types.h>
#include "global_variables.h"
#include "config.h"                      // ESP_IDF libs included here

#if defined ESP8266
  #include<sstream> // for esp8266 (bug in standard library?
#endif  

#if (defined frBuiltin)
  #include "FrSky_Ports.h"
#endif

#if defined TEENSY3X || defined ESP8266  ||  defined ESP32 
  #undef F   // F defined as m->counter[5]  in c_library_v2\mavlink_sha256.h
             // Macro F()defined in Teensy3/WString.h && ESP8266WebServer-impl.h (forces string literal into prog mem)   
#endif

#include <ardupilotmega/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>



//=================================================================================================   
//                     F O R W A R D    D E C L A R A T I O N S
//=================================================================================================

void main_loop();
void serviceWiFiRoutines();
void serviceInboundTCPClients();
void checkStaLinkStatus(); 
void startWiFiTimer();
void restartWiFiSta();
void startAccessPoint();
bool newOutboundTCPClient();
bool readFCtoRingBuffer();
void decodeRBtoGCS();
bool readGCS();
void decodeGCStoFC();
void sendToFC(uint32_t);
void sendFcHeartbeat();
void mavParamRequestRead(int16_t, char *);
void mavRequestMissionList();
void requestDataStreams();
void mavParamSet();
void Send_Mavlink_Command_Long();
void Mavlink_Request_Home_Position();
void ServiceStatusLeds();
void MavToRingBuffer();
void Send_From_RingBuf_To_GCS();
void checkWifiLinkErrors(mavlink_message_t*); 
bool Read_Bluetooth(mavlink_message_t*);
bool Send_Bluetooth(mavlink_message_t*);
bool readTCP(mavlink_message_t*);
bool readUDP(io_side_t, mavlink_message_t*);
bool sendTCP(mavlink_message_t*);
bool sendUDP(io_side_t, mavlink_message_t*);
void DecodeOneMavFrame();
void CheckMavTimeouts();
void MarkHome();
uint32_t Get_Volt_Average1(uint16_t);
uint32_t Get_Volt_Average2(uint16_t);
uint32_t Get_Current_Average1(uint16_t);
uint32_t Get_Current_Average2(uint16_t); 
void Accum_mAh1(uint32_t);
void Accum_mAh2(uint32_t);
void Accum_Volts1(uint32_t); 
void Accum_Volts2(uint32_t); 
uint32_t getConsistent(uint8_t);
uint32_t SenseUart(uint8_t);
void OpenSDForWrite();  
void ServiceMavStatusLed();
void ServiceBufStatusLed();
void BlinkMavLed(uint32_t);
void DisplayRemoteIP();
bool Leap_yr(uint16_t);
void WebServerSetup();
void RecoverSettingsFromFlash();
void PrintPeriod(bool);
void PrintLoopPeriod();
void SetupWiFi(); 
void handleLoginPage();
void handleSettingsPage();
void handleSettingsReturn();
void handleReboot();
void handleOtaPage();
uint8_t EEPROMRead8(uint16_t);
void ReadSettingsFromEEPROM();
uint32_t EEPROMRead32(uint16_t);
uint16_t EEPROMRead16(uint16_t);
void RefreshHTMLButtons();
void RawSettingsToStruct();
void WriteSettingsToEEPROM();
void EEPROMReadString(uint16_t, char*);
void EEPROMWrite16(uint16_t, uint16_t);
void EEPROMWrite8(uint16_t, uint8_t);
void EEPROMWriteString(uint16_t, char*);
void EEPROMWrite32(uint16_t, uint32_t);
uint32_t bit32Extract(uint32_t, uint8_t, uint8_t);
float RadToDeg (float);

void printRemoteIP(io_side_t);
void LogScreenPrint(String);
#if (defined ESP32) || (defined ESP8266)
  void IRAM_ATTR gotButtonDn();
  void IRAM_ATTR gotButtonUp();
  void IRAM_ATTR gotButtonInfo();  
  void IRAM_ATTR gotWifiButton();
  String wifiStatusText(uint16_t);  
#endif
#if (defined ESP32)
  void IRAM_ATTR onTimer();                         // wifi retry timer
  esp_err_t event_handler(system_event_t *event);   // wifi events handler
#endif  

#if defined displaySupport
  void PaintLogScreen(uint8_t, last_row_t);
  void Scroll_Display(scroll_t);
  void SetupLogDisplayStyle(); 
  void SetupInfoDisplayStyle(); 
  uint32_t draw_horizon(float, float, int16_t, int16_t); 
  void draw_hud_arrow(int16_t, int16_t, int16_t, int16_t, int16_t); 
  void SetScreenSizeOrient(uint8_t, uint8_t);
   
  void DisplayFlightInfo(); 
  void HandleDisplayButtons();  
#endif

  #if (defined frBuiltin)
    FrSkyPort FrPort;   // instantiate FrSky Port object   
  #endif

//=================================================================================================
//=================================================================================================   
//                                      S   E   T   U   P 
//=================================================================================================
//=================================================================================================
void setup()  {
 
  log.begin(115200);
  #if defined RP2040
    delay(5000);
  #else
    delay(2500);
  #endif
  log.println();
  pgm_path = __FILE__;  // ESP8266 __FILE__ macro returns pgm_name and no path
  pgm_name = pgm_path.substring(pgm_path.lastIndexOf("\\")+1);  
  pgm_name = pgm_name.substring(0, pgm_name.lastIndexOf('.'));  // remove the extension
  log.print("Starting "); log.print(pgm_name);
  log.printf(" version:%d.%02d.%02d\n", MAJOR_VERSION,  MINOR_VERSION, PATCH_LEVEL);
    
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_WiFi)
   WiFi.onEvent(WiFiEventHandler);   
  #endif  
 
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    log.printf("Free Heap just after startup = %d\n", ESP.getFreeHeap());
  #endif  
//=================================================================================================   
//                                   S E T U P   D I S P L A Y
//=================================================================================================
  #if (defined displaySupport) 

    #if (defined ESP32)
 
      if ( (Tup != 99) && (Tdn != 99) ) {   // enable touch pin-pair
        touchAttachInterrupt(digitalPinToInterrupt(Tup), gotButtonUp, threshold);
        touchAttachInterrupt(digitalPinToInterrupt(Tdn), gotButtonDn, threshold);   
      } else
      if ( (Pup != 99) && (Pdn != 99) ) {   // enable digital pin-pair
        pinMode(Pup, INPUT_PULLUP );        // low = true
        pinMode(Pdn, INPUT_PULLUP);                          
      }
    #endif  
    
    #if defined RP2040
      if ( (Pup != 99) && (Pdn != 99) ) {   // enable digital pin-pair
        pinMode(Pup, INPUT );          // high = true
        pinMode(Pdn, INPUT);                          
      }
    #endif
    
    #if ((defined ESP8266) || (defined TEENSY3X))         
      if ( (Pup != 99) && (Pdn != 99) ) { // enable digital pin pair
        pinMode(Pup, INPUT_PULLUP);       // low = true
        pinMode(Pdn, INPUT_PULLUP);       // low = true
      }

    #endif 

    #if (defined ST7789_Display)               // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD (135 x 240)
      display.init(); 
      #define SCR_BACKGROUND TFT_BLACK
      
    #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display (128 x 64)
    
      //RP2040 - Wire.h
      //Wire.setSDA(0);
      //Wire.setSCL(1);
      //Wire.begin();
      //Wire1.setSDA(2);
      //Wire1.setSCL(3);
      //Wire1.begin(0x30); 
      
      #if (not defined TEENSY3X) && (not defined RP2040) // Teensy & RP2040, SCA and SCL defined in appropriate "pins_arduino.h"
         Wire.begin(SDA, SCL);  
      #endif   
      display.begin(SSD1306_SWITCHCAPVCC, i2cAddr);         
      #define SCR_BACKGROUND BLACK   
      
    #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display (96 x 64)
        //  uses software SPI pins defined in config.h 
        display.begin();
        #define SCR_BACKGROUND BLACK  
        
    #elif (defined ILI9341_Display)    // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2 (320 x 240)
        //  uses hardware SPI pins defined in config.h 
        display.begin();
        #define SCR_BACKGROUND ILI9341_BLUE
    #endif
   
    #if (SCR_ORIENT == 0)              // portrait
        log.print("Display Setup: Portrait ");         
    #elif (SCR_ORIENT == 1)            // landscape   
        log.println("Display support activated: Landscape "); 
    #endif

    int eol= 0;
    for (eol = 0 ; eol < scr_w_ch ; eol++) {
      clear_line[eol] = ' ';
    }
    clear_line[eol] = 0x00;

    SetScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);

    log.printf("%dx%d  text_size=%d  char_w_px=%d  char_h_px=%d  scr_h_ch=%d  scr_w_ch=%d\n", scr_h_px, scr_w_px, TEXT_SIZE, char_w_px, char_h_px, scr_h_ch, scr_w_ch);
    
    LogScreenPrintln("Starting .... ");
  #else
    log.println("No display support selected or built-in");    
  #endif
  /*
  display.setFont(Dialog_plain_8);     //  col=24 x row 8  on 128x64 display
  display.setFont(Dialog_plain_16);    //  col=13 x row=4  on 128x64 display
  */
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    log.printf("==============>Free Heap after OLED setup = %d\n", ESP.getFreeHeap());
  #endif
  
//=================================================================================================   
//                             S E T U P   E E P R O M
//=================================================================================================

  #if defined ESP32    
    if (!EEPROM.begin(EEPROM_SIZE))  {
      log.println("Fatal error!  EEPROM failed to initialise.");
      LogScreenPrintln("EEPROM fatal error!");
      while (true) delay(100);  // wait here forever 
     } else {
      log.println("EEPROM initialised successfully");
      LogScreenPrintln("EEPROM good"); 
     }
  #endif       
    
  #if defined ESP8266
    EEPROM.begin(EEPROM_SIZE);
    log.println("EEPROM initialised successfully");
    LogScreenPrintln("EEPROM good"); 
  #endif

  RawSettingsToStruct();                      // So that we can use them regardless of webSupport
  
  #if (defined webSupport) 
    RecoverSettingsFromFlash(); 
    if (resetEepromPin != 99) {               // Pin to reset eeprom to default settings on the fly
      pinMode(resetEepromPin, INPUT );        // High = true                       
    }  
  #endif

  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    log.printf("==============>Free Heap after EEPROM setup = %d\n", ESP.getFreeHeap());
  #endif

  // =============================================
  
  #if (defined frBuiltin)  // FrSky Port support

    if (set.trmode == ground) {
     log.println("Ground Mode selected");
      LogScreenPrintln("Ground Mode");
   } else  
   if (set.trmode == air) {
      log.println("Air Mode selected");
      LogScreenPrintln("Air Mode");
   } else
   if (set.trmode == relay) {
      log.println("Relay Mode selected");
     LogScreenPrintln("Relay Mode");
    }

    #if (Battery_mAh_Source == 1)  
      log.println("Battery_mAh_Source = 1 - Get battery capacities from the FC");
     LogScreenPrintln("mAh from FC");
    #elif (Battery_mAh_Source == 2)
     log.println("Battery_mAh_Source = 2 - Define battery capacities in this firmware");  
     LogScreenPrintln("mAh defined in fw");
    #elif (Battery_mAh_Source == 3)
     log.println("Battery_mAh_Source = 3 - Define battery capacities in the LUA script");
     LogScreenPrintln("Setup mAh in LUA");     
    #else
     #error You must define at least one Battery_mAh_Source. Please correct.
    #endif            

    #ifndef RSSI_Override
      log.println("RSSI Automatic Select");
      LogScreenPrintln("RSSI Auto Select");     
    #else
     log.printf("RSSI Override for testing = %d%%\n", RSSI_Override);
      snprintf(snprintf_buf, snp_max, "RSSI Override %d%%", RSSI_Override);
      LogScreenPrintln(snprintf_buf);      
    #endif

  #else
      log.println("No FrSky port support selected or built in");
      LogScreenPrintln("No FrSky support");
  #endif
  
  log.print("Target Board is ");
  #if (defined TEENSY3X) // Teensy3x
    log.println("Teensy 3.x");
    LogScreenPrintln("Teensy 3.x");
  #elif (defined ESP32) //  ESP32 Board
    log.print("ESP32 / Variant is ");
    LogScreenPrintln("ESP32 Variant is");
    #if (ESP32_Variant == 1)
      log.println("Dev Module");
      LogScreenPrintln("Dev Module");
    #endif
    #if (ESP32_Variant == 2)
      log.println("Wemos® LOLIN ESP32-WROOM-32");
      LogScreenPrintln("Wemos® LOLIN");
    #endif
    #if (ESP32_Variant == 3)
      log.println("Dragonlink V3 slim with internal ESP32");
      LogScreenPrintln("Dragonlink V3 ESP32");
    #endif
    #if (ESP32_Variant == 4)
      log.println("Heltec Wifi Kit 32");
      LogScreenPrintln("Heltec Wifi Kit 32");
    #endif
    #if (ESP32_Variant == 5)
      log.println("LILYGO® TTGO T-Display ESP32 1.14 inch ST7789 Colour LCD");
      LogScreenPrintln("TTGO T-Display ESP32");
    #endif    
    #if (ESP32_Variant == 6)
      log.println("LILYGO® TTGO T2 ESP32 OLED SD");
      LogScreenPrintln("TTGO T2 ESP32 SD");
    #endif 
    #if (ESP32_Variant == 7)
      log.println("Dev Module with ILI9341 2.8in COLOUR TFT SPI");
      LogScreenPrintln("Dev module + TFT");
    #endif                   
  #elif (defined ESP8266) 
    log.print("ESP8266 / Variant is ");
    LogScreenPrintln("ESP8266 / Variant is");  
    #if (ESP8266_Variant == 1)
      log.println("Lonlin Node MCU 12F");
      LogScreenPrintln("Node MCU 12");
    #endif 
    #if (ESP8266_Variant == 2)
      log.println("ESP-F - RFD900X TX-MOD");
      LogScreenPrintln("RFD900X TX-MOD");
    #endif   
    #if (ESP8266_Variant == 3)
      log.println("Wemos® ESP-12F D1 Mini");
      LogScreenPrintln("Wemos ESP-12F D1 Mini");
    #endif   
  #elif (defined RP2040)
    #if (RP2040_Variant == 1)
      log.println("Raspberry Pi Pico RP2040");
      LogScreenPrintln("Pi Pico RP2040");
    #endif        
  #endif

  if (set.fc_io == fc_ser)   {
    log.println("Mavlink Serial  In");
    LogScreenPrintln("Mav Serial In");
  }

  if (set.fc_io == fc_bt)  {
    log.println("Mavlink Bluetooth In");
    LogScreenPrintln("Mav BT In");
  } 

  if (set.fc_io == fc_wifi)  {
    log.println("Mavlink WiFi In");
    LogScreenPrintln("Mav WiFi In");
  } 
  #if defined Slowdown_SITL_Input
    log.println("** SLOWING DOWN INPUT (FOR SITL) **");
    LogScreenPrintln("**SITL SLOWDOWN ");
  #endif
  if (set.fr_io == fr_ser)     {         // 1
    log.println("FrSky Serial Out");
    LogScreenPrintln("Frs Serial Out");
  } 
  #if (defined wifiBuiltin)     
    else
    if (set.fr_io == fr_udp)  {           // 2
      log.println("FrSky UDP Out");
      LogScreenPrintln("Frs UDP Out");
    } else   
    if (set.fr_io == fr_ser_udp)    {     // 3
      log.println("FrSky Serial and UDP Out");
      LogScreenPrintln("Frs Serial+UDP Out");
    } 
  #endif
  #if (defined wifiBuiltin) && (defined sdBuiltin)    
    else   
    if (set.fr_io == fr_udp_sd)    {      // 4
      log.println("FrSky UDP and SD Out");
      LogScreenPrintln("Frs UDP+SD Out");
    } else    
    if (set.fr_io == fr_ser_sd)    {      // 5
      log.println("FrSky Serial and SD Out");
      LogScreenPrintln("Frs Serial+SD Out");
    } else  
    if (set.fr_io == fr_udp_sd)    {      // 6
      log.println("FrSky UDP and SD Out");
      LogScreenPrintln("Frs UDP+SD Out");
    } else    
    if (set.fr_io == fr_ser_udp_sd)     { // 7
      log.println("FrSky Serial and UDP and SD Out");
      LogScreenPrintln("Frs Ser+UDP+SD Out");
    }
  #endif
  if (set.gs_io == gs_ser)  {
    log.println("Mavlink Serial Out");
    LogScreenPrintln("Mav Serial Out");
  }
  #if (defined btBuiltin)   
    if (set.gs_io == gs_bt)  {
      log.println("Mavlink Bluetooth Out");
      LogScreenPrintln("Mav BT Out");
    }
    #endif
  #if (defined wifiBuiltin)  
    if (set.gs_io == gs_wifi)  {
      log.print("Mavlink WiFi Out - ");
      LogScreenPrintln("Mav WiFi Out");
    }
  #endif
  #if (defined wifiBuiltin) && (defined btBuiltin)    
    if (set.gs_io == gs_wifi_bt)  {
      log.print("Mavlink WiFi+BT Out - ");
      LogScreenPrintln("Mav WiFi+BT Out");
    }
  #endif
  #if defined wifiBuiltin  
    if ((set.fc_io == fc_wifi) || (set.gs_io == gs_wifi) ||  (set.gs_io == gs_wifi_bt) || (set.web_support)) {
     if (set.wfmode == ap)  {
       log.println("WiFi mode is AP");
       LogScreenPrintln("WiFi-mode is AP");
     } else
     if (set.wfmode == sta)  {
       log.println("WiFi mode is STA");
       LogScreenPrintln("WiFi-mode is STA");
     } else
     if (set.wfmode == sta_ap)  {
       log.println("WiFi mode is STA>AP");
       LogScreenPrintln("WiFi-mode=STA>AP");
     } else
     if (set.wfmode == ap_sta)  {
       log.println("WiFi mode is AP_STA");
       LogScreenPrintln("WiFi-mode=AP_STA");
     }
    
     if (set.mav_wfproto == tcp)  {
       log.println("Protocol is TCP/IP");
       LogScreenPrintln("Protocol=TCP/IP");
     }
     else if  (set.mav_wfproto == udp) {
       log.print("Protocol is UDP");
       LogScreenPrintln("Protocol = UDP");
       #if defined UDP_Broadcast
         log.println(" Broadcast");
         LogScreenPrintln(" Broadcast");
       #else
         log.println(" IP-Targeted");     
         LogScreenPrintln("IP-Targeted");
       #endif
     }
    }
  #endif
    
  #if defined sdBuiltin                    
    if (set.fc_io == fc_sd) {
      log.println("Mavlink SD In");
      LogScreenPrintln("Mavlink SD In");
   }

    if (set.gs_sd == gs_on) {
      log.println("Mavlink SD Out");
      LogScreenPrintln("Mavlink SD Out");
    }
    
    if (set.sport_sd == spsd_on) {
      log.println("SPort To SD Out");
      LogScreenPrintln("SPort To SD Out");
    }
    
  #endif

  #if (defined Support_MavLite)
    log.println("MavLite supported");
    LogScreenPrintln("MavLite supported");
    set.Support_MavLite = true;
  #else
    set.Support_MavLite = false;
  #endif
  

//=================================================================================================   
//                                S E T U P   W I F I  --  E S P only
//=================================================================================================

/*
 * Interrupt Modes
#define DISABLED  0x00
#define RISING    0x01
#define FALLING   0x02
#define CHANGE    0x03
#define ONLOW     0x04
#define ONHIGH    0x05
#define ONLOW_WE  0x0C
#define ONHIGH_WE 0x0D
 * 
*/
  #if (defined wifiBuiltin)
    if ((set.fc_io == fc_wifi) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.web_support)) {     
      #if (not defined Start_WiFi)
        if (startWiFiPin != 99) { 
          pinMode(startWiFiPin, INPUT_PULLUP);
          attachInterrupt(digitalPinToInterrupt(startWiFiPin),gotWifiButton, FALLING);   // For optional Start_WiFi button       
        }
      #endif       
    }

  #else
    log.println("No WiFi options selected, WiFi support not compiled in");
  #endif

//=================================================================================================   
//                                   S E T U P   B L U E T O O T H
//=================================================================================================

// Slave advertises hostname for pairing, master connects to slavename

  #if (defined btBuiltin)
    if ((set.fc_io == fc_bt) || (set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt)) { 
      if (set.btmode == 1)   {               // master
        log.printf("Bluetooth master mode host %s is trying to connect to slave %s. This can take up to 30s\n", set.host, set.btConnectToSlave); 
        LogScreenPrintln("BT connecting ......");   
        SerialBT.begin(set.host, true);      
        // Connect(address) is relatively fast (10 secs max), connect(name) is slow (30 secs max) as it 
        // needs to resolve the name to an address first.
        // Set CoreDebugLevel to Info to view devices bluetooth addresses and device names
        bool bt_connected;
        bt_connected = SerialBT.connect(set.btConnectToSlave);
        if(bt_connected) {
          log.println("Bluetooth done");  
          LogScreenPrintln("BT done");
        }          
      } else {                                  // slave, passive                           
        SerialBT.begin(set.btConnectToSlave);   // advertise this name      
        log.printf("Bluetooth slave mode, host name for pairing is %s\n", set.btConnectToSlave);               
      } 
      btActive = true;    
    }
  #else
    log.println("No Bluetooth options selected, BT support not compiled in");
    btActive = false;
  #endif
  
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    log.printf("==============>Free Heap after Bluetooth setup = %d\n", ESP.getFreeHeap());
  #endif

    
 //=================================================================================================   
 //                                  S E T U P   S D   C A R D  -  E S P 3 2  O N L Y  for now
 //=================================================================================================
  #if ((defined ESP32) || (defined ESP8266)) && (defined sdBuiltin)
  
    void listDir(fs::FS &fs, const char *, uint8_t);  // Fwd declare
    
    if(!SD.begin()){   
        log.println("No SD card reader found. Ignoring SD!"); 
        LogScreenPrintln("No SD reader");
        LogScreenPrintln("Ignoring!");
        sdStatus = 0; // 0=no reader, 1=reader found, 2=SD found, 3=open for append 
                      // 4=open for read, 5=eof detected, 9=failed
    } else {
      log.println("SD card reader mount OK");
      LogScreenPrintln("SD drv mount OK");
      uint8_t cardType = SD.cardType();
      sdStatus = 1;
      if(cardType == CARD_NONE){
          log.println("No SD card found");
          LogScreenPrintln("No SD card");
          LogScreenPrintln("Ignoring!");      
      } else {
        log.println("SD card found");
        LogScreenPrintln("SD card found");
        sdStatus = 2;

   //     log.printf("Total space: %lluMB\n", SD.totalbytes() / (1024 * 1024));
   //     log.printf("Used space: %lluMB\n", SD.usedbytes() / (1024 * 1024));

        listDir(SD, "/", 2);

        if (set.fc_io == fc_sd)  {   //  FC side SD in only   
          std::string S = "";  
          char c = 0x00;
           log.println("Enter the number of the SD file to read, and press Send");
           while (c != 0xA) { // line feed
            if (log.available())  {
              c = log.read();
              S+=c;
             }
             delay(50);
           }
           
           int i;
           // object from the class stringstream 
           std::istringstream myInt(S); 
           myInt >> i;
           log.print(i); log.print(" ");
           /*
           for (int j= 0 ; fnCnt > j ; j++)  {
           //   cout << i << fnPath[j] << "\n";
             log.print(j); log.print(" "); log.println(fnPath[j].c_str());
            }
           */     

           sprintf(cPath, "%s", fnPath[i].c_str());  // Select the path
           log.print(cPath); log.println(" selected "); 
           log.println("Reading SD card");
           LogScreenPrintln("Reading SD card");
           file = SD.open(cPath);
           if(!file){
             log.printf("Can't open file: %s\n", cPath);
             log.println(" for reading");
             sdStatus = 9;  // error
           } else {
             sdStatus = 4;
           }
        }
               
        // The SD is initialised/opened for write in Main Loop after timeGood
        // because the path/file name includes the date-time
     }  
   }
   
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    log.printf("==============>Free Heap after SD Card setup = %d\n", ESP.getFreeHeap());
  #endif

#endif

//=================================================================================================   
//                             S E T U P   S E R I A  L   -  Serial (uart0) setup in config.h
//=================================================================================================  

  if ((set.fc_io == fc_ser) || (set.gs_io == gs_ser))  {  //  Serial
    
    #if defined MavAutoBaud
      set.baud = FrPort.getBaud(fc_rxPin, idle_high); // mavlink port is a regular non-inverted port
      log.printf("Mavlink baud detected at %d b/s on rx:%d\n", set.baud, fc_rxPin);  
      String s_baud=String(set.baud);   // integer to string. "String" overloaded
      LogScreenPrintln("Mav baud:"+ s_baud);        
    #endif   
    
    #if (defined ESP32) 
      delay(100);
      #if (defined ESP32_Mav_SoftwareSerial) 
        fcSerial.begin(set.baud, SWSERIAL_8N1, fc_rxPin, fc_txPin);   // SoftwareSerial           
        log.println("Using SoftwareSerial for fcSerial");
      #else  // ESP32 HardwareSerial
        // system can wait here for a few seconds (timeout) if there is no telemetry in
         fcSerial.begin(set.baud, SERIAL_8N1, fc_rxPin, fc_txPin);   //  rx,tx, cts, rts  
      #endif
      delay(100);

      #if defined Support_SBUS_Out 
        uint8_t sbusInvert = true;
        delay(100);
        sbusSerial.begin(100000, SERIAL_8E1, sbus_rxPin, sbus_txPin, sbusInvert);     // HardwareSerial  
        delay(100);     
        log.printf("SBUS out on UART1 pin tx:%d\n", sbus_txPin);
      #endif 
    #endif
   
    #if (defined ESP8266)           // Always HardwareSerial
      fcSerial.begin(set.baud);  
      delay(60);  // for esp8266 debug on txd1     
    #endif
        
    #if (defined TEENSY3X)          // Always HardwareSerial
      fcSerial.begin(set.baud);   
      if (set.gs_io == gs_ser) {
        gsSerial.begin(set.baud);           
      }      
      #if defined Support_SBUS_Out 
        sbusSerial.begin(100000, SERIAL_8E1_TXINV);        // SBUS out = tx pin 
        log.printf("SBUS out on UART2 pin tx:%d\n", sbus_txPin );
      #endif      
    #endif

    #if (defined RP2040) 
      fcSerial.begin(set.baud);      // Always HardwareSerial  
    #endif
     
    log.printf("Mavlink serial on pins rx:%d and tx:%d  baud:%d\n", fc_rxPin, fc_txPin, set.baud); 
  }
  
  #if (defined frBuiltin)  
    if (set.fr_io & 0x01) {  // Serial bit flag set
      FrPort.initialise();
    }
  #endif
  
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    log.printf("==============>Free Heap after Serial UART setup = %d\n", ESP.getFreeHeap());
  #endif

//=================================================================================================   
//                                    S E T U P   O T H E R
//=================================================================================================   
  mavGood = false;
  homGood = false;     
  hb_count = 0;
  hb_millis=millis();
  downlink_millis = millis();       // mavlink uplink to gcs timimg
  fchb_millis = millis();
  gshb_millis = millis();
  acc_millis = millis();
  rds_millis = millis();
  blind_inject_millis = millis();
  health_millis = millis();
  bool esp8266_variant2 = false;
  
  #if (defined ESP8266) && (ESP8266_Variant == 2) 
    esp8266_variant2 = true;
  #endif

  if (MavStatusLed != 99) {
    if (esp8266_variant2) {
      log.println("MavStatusLed = D4. Set MavStatusLed = 99 if you need more TXD1 debugging");
      delay(200);
    }
    pinMode(MavStatusLed, OUTPUT); 
  } else {
    if (esp8266_variant2) {
      log.println("MavStatusLed = 99. TXD1 debugging possible if enabled in IDE");
    }
  }
 
}

  
//================================================================================================= 
//================================================================================================= 
//                                   M  A  I  N    L  O  O  P
//================================================================================================= 
//================================================================================================= 

void loop() {
                             
  //====================       C h e c k   F r e e   H e a p

  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    if (millis() - free_heap_millis > 5000) {
      free_heap_millis = millis();
      log.printf("Free Heap now = %d\n", ESP.getFreeHeap());
    }
  #endif

  //====================
 
  #if ((defined ESP32) || (defined ESP8266)) 
    serviceWiFiRoutines();
  #endif
 
  //====================     C h e c k   F o r   E E P R O M   R e s e t   T r i g g e r

  #if (defined webSupport) 
    bool pinval = 0;
    if (resetEepromPin != 99) {   // if digital pin enumerated
    //  #if (defined ESP32) || (defined RP20400)
        pinval = digitalRead(resetEepromPin);       // pin to 3.3v = high = pressed  
        resetEepromPress = pinval;
    //  #else // esp8266 or teensy 3.x
    //    pinval = digitalRead(resetEepromPin);       // pin to gnd = low = pressed  
    //    resetEepromPress = !pinval;
    //  #endif
    }
    //log.printf("Pin %u:%u  eepromStart:%u eepromElapsed:%u \n", resetEepromPin, resetEepromPress, eepromStart, eepromElapsed);  
    if (resetEepromPress) {
      if (eepromStart == 0) {
        eepromStart = millis();
      } else {
        eepromElapsed = millis() - eepromStart;
        if (eepromElapsed >= eepromTrigger) {
          log.printf("Pin %u:%u  interrupt ** ALL SETTINGS IN EEPROM SET TO COMPILE_TIME DEFAULTS! **\n", resetEepromPin, pinval ); 
          LogScreenPrintln("*EEPROM RESET*");  
          RawSettingsToStruct();                                         
          WriteSettingsToEEPROM();
          eepromStart =  0;  
          log.println("Rebooting ....");
          LogScreenPrintln("Rebooting");    
          delay(1000);
          ESP.restart();                 // esp32 and esp8266         
        }
      }
    } else {
      eepromStart =  0; 
    }
             
  #endif

  //====================

  #if defined Debug_Loop_Period
    PrintLoopPeriod();
  #endif

  //==================== F l i g h t   C o m p u t e r   T o   R i n g   B u f f e r
  
  if (!readFCtoRingBuffer()) {  //  check for SD eof
    if (sdStatus == 5) {
      log.println("End of SD file");
      LogScreenPrintln("End of SD file");  
      sdStatus = 0;  // closed after reading   
    }
  }

  //==================== 
  
  bool rssiOverride = false;
  #ifdef RSSI_Override
    rssiOverride = true;     // for debugging
  #endif

  //====================    R i n g   B u f f e r   D e c o d e  &  S e n d   T o   G C S
   
 // if (millis() - downlink_millis > 1) {   // main timing loop for mavlink decode and to GCS
    decodeRBtoGCS();
 // }
 
 //====================     H a n d l e   F r S k y   P o r t   T r a f f i c
 
  #if (defined frBuiltin)
    FrPort.HandleTraffic(); 
  #endif  

  //====================   R e a d   F r o m   G C S
  
  if (readGCS()) {
    if (GCS_available) {
      decodeGCStoFC();
      sendToFC(G2Fmsg.msgid);  
      GCS_available = false;                      
    }   
  }



  //==================== Check For Heartbeat from GCS Timeout
  
  if(gshbGood && (millis() - gshb_millis) > 10000){ // if no heartbeat from GCS in n seconds then assume GCS not connected
    gshbGood = false;
  //  log.println("Heartbeat from GCS timed out! GCS not connected"); 
  //  LogScreenPrintln("GCS Heartbeat lost!"); 
    
  }

  //==================== Send Our Own Heartbeat to FC Only When No GCS Heartbeat (this to trigger tardy telemetry)

  if (!gshbGood) {
    if(millis()- fchb_millis > 2000) {  // MavToPass heartbeat to FC every 2 seconds
      fchb_millis=millis();       
      sendFcHeartbeat();              // must have MavToPass tx pin connected to Telem radio rx pin  
    }
  }

  //====================            Check For FC Heartbeat Timeout
  
  if(mavGood && (millis() - hb_millis) > 15000)  {   // if no heartbeat from APM in 15s then assume FC mav not connected
    mavGood=false;
    log.println("Heartbeat from FC timed out! FC not connected"); 
    LogScreenPrintln("FC heartB lost!");       
    hb_count = 0;
   } 
  //====================              RSSI pacemaker for OpenTX 

  #if (defined frBuiltin)
    #if defined Rssi_Pacemaker
      if ((set.trmode == ground) || (set.trmode == relay))  {       // In Air_Mode the FrSky receiver provides rssi
        if ( ( (rssiGood && mavGood)  || rssiOverride ) && (millis() - rssi_millis >= Rssi_Pacemaker) ) {
          FrPort.PushMessage(0xF101, 0);   // 0xF101 RSSI 
          rssi_millis = millis(); 
         }   
      }
    #endif   
  #endif  

  //===============   Check For Display Button Touch / Press
  
  #if defined displaySupport
    HandleDisplayButtons();
  #endif
     
  //==================== Data Streaming Option
  
  #ifdef Data_Streams_Enabled 
  if(mavGood) {                      // If we have a link, request data streams from MavLink every 30s
    if(millis()-rds_millis > 30000) {
    rds_millis=millis();
    //log.println("Requesting data streams"); 
    //LogScreenPrintln("Reqstg datastreams");    
    requestDataStreams();   // must have Teensy Tx connected to Taranis/FC rx  (When SRx not enumerated)
    }
  }
  #endif 
  
  //==================== Download Missions Option
  
  #if defined Request_Missions_From_FC || defined Request_Mission_Count_From_FC
  if (mavGood) {
    if (!ap_ms_list_req) {
      mavRequestMissionList();  //  #43
      ap_ms_list_req = true;
    }
  }
  #endif
  
  //==================== Get battery capacity from the FC Option
  
  #if (Battery_mAh_Source == 1)  
  // Request battery capacity params 
  if (mavGood) {
    if (!ap_bat_paramsReq) {
      mavParamRequestRead(356, (char*)"BATT_CAPACITY");    // Request Bat1 capacity   do this twice in case of lost frame
      mavParamRequestRead(356, (char*)"BATT_CAPACITY");    
      mavParamRequestRead(364, (char*)"BATT2_CAPACITY");    // Request Bat2 capacity
      mavParamRequestRead(364, (char*)"BATT2_CAPACITY");     
      log.println("Battery capacities requested");
      LogScreenPrintln("Bat mAh from FC");    
      ap_bat_paramsReq = true;
    } else {
      if (ap_bat_paramsRead &&  (!parm_msg_shown)) {
        parm_msg_shown = true; 
        log.println("Battery params successfully read"); 
        LogScreenPrintln("Bat params read ok"); 
      }
    } 
  }
  #endif 
  
  //====================   List All Parameters From the FC Option
  
  #ifdef Mav_List_Params
    if(mavGood && (!ap_paramsList)) {
      Request_Param_List();
      ap_paramsList = true;
    }
  #endif 
  
  //====================   #410 Request Home Position from the FC Option
  
  #ifdef REQUEST_HOME_FROM_FC 
    if(mavGood && motArmed && (!ap242_homGood)) {
      static bool rhft = true;
      if (rhft) {
        home_request_millis = millis();
        homeRequested = false;
      } else {
        if ( (!homeRequested) && (millis() - home_request_millis > 5000) ) {
          home_request_millis = millis();
          Mavlink_Request_Home_Position(); // #410 do this every 5 seconds after arming until ap242 received
          homeRequested = true;  
             
          #if defined Mav_Debug_Home_Position
            log.println("Mavlink_Request_Home_Position command sent to FC");
          #endif   
        }    
      }
    }
  #endif 

  //====================   Open an SD File Here Because We Waited For The Time From FC
  
  #if defined sdBuiltin
    if ((timeGood) && (sdStatus == 2)) OpenSDForWrite();
  #endif
  
  //====================   Service The Leds

  ServiceStatusLeds();
  
  //====================   Service The Web Server
  #if defined webSupport     //  esp32 and esp8266 only
    if (wifiSuGood) {
      server.handleClient();
    }
  #endif
 
}
//================================================================================================= 
//                               E N D   O F    M  A  I  N    L  O  O  P
//================================================================================================= 

bool readFCtoRingBuffer() {

  if (set.fc_io == fc_ser)  {  // Serial
    
    mavlink_status_t status;
    static bool got_one = false; 
    while(fcSerial.available()) { 
      byte c = fcSerial.read();
     // printbyte(c, 1, '<');
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &F2Rmsg, &status)) {  // Read a frame
         #ifdef  Debug_FC_Down
           log.println("Serial passed to RB from FC side :");
           printMavBuffer(&F2Rmsg);
        #endif              
        MavToRingBuffer();    
        return true;   
      } 
    }
    return false;  
  } 

  #if (defined btBuiltin) 
    if (set.fc_io == fc_bt)  {  // Bluetooth

     bool msgReceived = Read_Bluetooth(&F2Rmsg);

     if (msgReceived) {

        MavToRingBuffer();      

        #ifdef  Debug_FC_Down   
          log.print("Read from FC Bluetooth to Ringbuffer: msgReceived=" ); log.println(msgReceived);
          printMavBuffer(&F2Rmsg);
        #endif      
      }
    return true;  
    }   
  #endif

  #if (defined wifiBuiltin)
    if (set.fc_io == fc_wifi)  {  //  WiFi
      
      if ((set.mav_wfproto == tcp) && (outbound_clientGood))  { // TCP  from FC side
 
        bool msgReceived = readTCP(&F2Rmsg);
        if (msgReceived) {
        
          MavToRingBuffer();  
          
          #ifdef  Debug_FC_Down    
            log.print("Read from FC WiFi TCP to Ringbuffer: msgReceived=" ); log.println(msgReceived);
            printMavBuffer(&F2Rmsg);
          #endif      
        }
       return true;  
      }
      
      if (set.mav_wfproto == udp)  {    // UDP from FC

        if ((set.wfmode == ap_sta) || (set.wfmode == sta)) {// if AP_STA or STA mode                 
          active_udp_obj_idx = 0;                            // Use STA UDP object for FC read        
          udp_read_port = set.udp_remotePort;               // used by printRemoteIP() only. read port set by UDP.begin().
          udp_send_port = set.udp_localPort;                   
        } else {
          if (set.wfmode == ap) {
            active_udp_obj_idx = 1;                          // Use AP UDP object for FC read       
            udp_read_port = set.udp_localPort;  
            udp_send_port = set.udp_remotePort;                     
          }                 
        }     
                  
        bool msgReceived = readUDP(fc_side, &F2Rmsg);              // FC side
        if (msgReceived) {

          #if defined  Debug_readUDP || defined Debug_readUDP_FC  
            log.print("Read WiFi UDP from FC to G2Fmsg: msgReceived=" ); log.println(msgReceived);
          #endif
          
          MavToRingBuffer();   
         
          #if defined Debug_FC_Down || defined Debug_readUDP_FC 
            log.print("Read from FC WiFi UDP to Ringbuffer: msgReceived=" ); log.println(msgReceived);
            printMavBuffer(&F2Rmsg);
          #endif      
        }
       return true;     
      }
   
    }
  #endif 
  
 #if ((defined ESP32)  || (defined ESP8266)) && (defined sdBuiltin)
    if (set.fc_io == fc_sd)  {   //  SD
      mavlink_status_t status;
      if (sdStatus == 4) {      //  if open for read
        while (file.available()) {
          uint8_t c = file.read();
          if(mavlink_parse_char(MAVLINK_COMM_0, c, &F2Rmsg, &status)) {  // Parse a frame
            #ifdef  Debug_FC_Down
              log.println("Read from FC SD to Ringbuffer:");
              printMavBuffer(&F2Rmsg);
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
    }
  #endif 
  return false; 
}
//================================================================================================= 

void decodeRBtoGCS() {

  if (!MavRingBuff.isEmpty()) {
    R2Gmsg = (MavRingBuff.shift());  // Get a mavlink message from front of queue
    #if defined Mav_Debug_RingBuff
      //log.print("Mavlink ring buffer R2Gmsg: ");  
      //printMavBuffer(&R2Gmsg);
      log.print("Ring queue = "); log.println(MavRingBuff.size());
    #endif
    
    Send_From_RingBuf_To_GCS();
    
    CheckMavTimeouts();   // Check if we need to reset some variables based on specific mavlink message timeouts
    DecodeOneMavFrame();  // Decode a Mavlink frame and buffer passthru

  }
}  

//================================================================================================= 
bool readGCS() {

  #if (defined TEENSY3X)
    if (set.gs_io == gs_ser)  {  // Serial 
      mavlink_status_t status;
      static bool got_one = false;      
      while(gsSerial.available()) { 
        uint8_t c = gsSerial.read();
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &G2Fmsg, &status)) {  // Read a frame from GCS  
          GCS_available = true;  // Record waiting
          #ifdef  Debug_GCS_Up
            log.println("Passed up from GCS Serial to G2Fmsg:");
            printMavBuffer(&G2Fmsg);
          #endif     
          return = true;   
        }
      }
      return false;  
    } 
  #endif 

  #if (defined btBuiltin) 
    if ((set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt)) {  // Bluetooth
 
       bool msgReceived = Read_Bluetooth(&G2Fmsg);

       if (msgReceived) {
          GCS_available = true;  // Record waiting to go to FC 
          #ifdef  Debug_GCS_Up    
            log.print("Passed up from GCS BT to G2Fmsg: msgReceived=" ); log.println(msgReceived);
            if (msgReceived) printMavBuffer(&G2Fmsg);
          #endif      
        }
        return true;
    }  
  #endif

  #if (defined wifiBuiltin)
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) ) {   //  WiFi
    
      if ((set.mav_wfproto == tcp) && (inbound_clientGood))  { // TCP from GCS side    
    
        bool msgReceived = readTCP(&G2Fmsg);

        if (msgReceived) {
          GCS_available = true;  // Record waiting to go to FC 

          #ifdef  Debug_readTCP 
            log.print("Read WiFi TCP to G2Fmsg: msgReceived=" ); log.println(msgReceived);
            if (msgReceived) printMavBuffer(&G2Fmsg);
          #endif      
        }
        return true; 
      }
      
      if (set.mav_wfproto == udp)  { // UDP from GCS
  
        if ((set.wfmode == ap_sta) || (set.wfmode == ap)) { // if AP_STA mode                 
          active_udp_obj_idx = 1;                            // Use AP UDP object for GCS read        
          udp_read_port = set.udp_localPort;               // used by printRemoteIP() only. read port set by UDP.begin().
          udp_send_port = set.udp_remotePort;                   
        } else {
          if (set.wfmode == sta) {
            active_udp_obj_idx = 0;                          // Use STA UDP object for GCS read  
            udp_read_port = set.udp_localPort;  
            udp_send_port = set.udp_remotePort;                          
          }               
        }  
            
        bool msgReceived = readUDP(gcs_side, &G2Fmsg);               // GCS side

        if (msgReceived) {
          GCS_available = true;  // Record waiting to go to FC 

          #if defined  Debug_readUDP || defined Debug_GCS_Up || defined Debug_readUDP_GCS  
            log.printf("Read WiFi UDP from GCS to G2Fmsg: msgReceived=%d ==============================\n", msgReceived); 
            printMavBuffer(&G2Fmsg);
          #endif 
          return true;     
        }   
      } 
    }
    return false;
  #endif  
} 

//================================================================================================= 
  #if (defined btBuiltin) 
  bool Read_Bluetooth(mavlink_message_t* msgptr)  {
    
    bool msgRcvd = false;
    mavlink_status_t _status;
    
    len = SerialBT.available();
    uint16_t bt_count = len;
    if(bt_count > 0) {

        while(bt_count--)  {
            int result = SerialBT.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {

                    if(!hb_heard_from) {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from     = true;
                            hb_system_id      = msgptr->sysid;
                            hb_comp_id        = msgptr->compid;
                            hb_seq_expected   = msgptr->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkWifiLinkErrors(msgptr);
                    }
                 
                    break;
                }
            }
        }
    }
    
    return msgRcvd;
}
#endif
//================================================================================================= 
#if (defined wifiBuiltin)
  bool readTCP(mavlink_message_t* msgptr)  {
    if ( (!wifiSuGood) || ((!inbound_clientGood) && (!outbound_clientGood)) ) return false; 

    bool msgRcvd = false;
    mavlink_status_t _status;

    for (int i = 0 ; i < max_clients ; ++i) {  // check each connected client for waiting data  
      if (NULL != tcp_client[i]) {             // if null pointer go around, tcp_client[0] is reserved for inbound
        // active client
        active_client_idx = i;
        len = tcp_client[active_client_idx]->available();     // is there some data to read
        uint16_t tcp_count = len;

        if(tcp_count > 0) {           // if so, read until no more
          while(tcp_count--)  {
            int result = tcp_client[active_client_idx]->read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {
                    if(!hb_heard_from) {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from     = true;
                            hb_system_id      = msgptr->sysid;
                            hb_comp_id        = msgptr->compid;
                            hb_seq_expected   = msgptr->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkWifiLinkErrors(msgptr);
                    }
                 
                    break;    // after complete message received, remembering active_client_idx
                }
            }
          }
        }
      }  // end of active tcp_client   
    }    // end of tcp_client for() loop

    return msgRcvd;
  }
#endif

//================================================================================================= 
#if (defined wifiBuiltin)
  bool readUDP(io_side_t io_side,  mavlink_message_t* msgptr)  {
    if (!wifiSuGood) return false;  
    bool msgRcvd = false;
    mavlink_status_t _status;

    // 2 possible udp objects, STA [0]    and    AP [1]   
        
        len = udp_object[active_udp_obj_idx]->parsePacket();
        // esp sometimes reboots here: WiFiUDP.cpp line 213 char * buf = new char[1460]; 
   
        int udp_count = len;
        if(udp_count > 0) {  
          //log.printf("Read UDP io_side=%d object=%d port:%d  len:%d\n", io_side, active_udp_obj_idx, udp_read_port, len);
          //if (io_side == gcs_side) log.printf("Read UDP io_side=%d object=%d port:%d\n", io_side, active_udp_obj_idx, udp_read_port);
          while(udp_count--)  {

            int result = udp_object[active_udp_obj_idx]->read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {

                    #if (not defined UDP_Broadcast)
                      UDP_remoteIP = udp_object[active_udp_obj_idx]->remoteIP();                     
                      bool in_table = false;
                      
                      if (io_side == fc_side) {
                        if (udpremoteip[0] != UDP_remoteIP) {
                          udpremoteip[0] = UDP_remoteIP;   // IP [0] reserved for FC side. can only ever have one FC client at a time
                          log.printf("%s FC client inserted in table\n", UDP_remoteIP.toString().c_str() ); 
                        }                
                      } else {     // gcs side
                      
                        for (int i = 1 ; i < max_clients ; i++) {
                          if (udpremoteip[i] == UDP_remoteIP) {  // IP already in the table
                        //    log.printf("%s already in table\n", UDP_remoteIP.toString().c_str() );
                            in_table = true;
                            break;
                          }
                        }
                        if (!in_table) {  // if not in table, add it into empty slot, but not [0] reserved for otgoing (FC side)
                          for (int i = 1 ; i < max_clients ; i++) {
                            if ((udpremoteip[i][0] == 0) || (udpremoteip[i][3] == 255)) {    // overwrite empty or broadcast ips
                              udpremoteip[i] = UDP_remoteIP;    // remember unique IP of remote udp client so we can target it  
                              log.printf("%s GCS client inserted in UDP client table\n", UDP_remoteIP.toString().c_str() );
                              FtRemIP = true;   
                              break;      
                            }
                          }
                        }
                      }
                    #endif  
                    
                    printRemoteIP(io_side);  
                                   
                    if(!hb_heard_from) {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from      = true;
                            hb_system_id       = msgptr->sysid;
                            hb_comp_id         = msgptr->compid;
                            hb_seq_expected   = msgptr->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkWifiLinkErrors(msgptr);
                    }
                    
                    break;
                }  // if(msgRcvd) {
            }      // if (result >= 0) 
          }        // while(udp_count--)
        }          // if(udp_count > 0)
        return msgRcvd;


  }
#endif

//================================================================================================= 
#if (defined wifiBuiltin)
  void checkWifiLinkErrors(mavlink_message_t* msgptr)   {

    //-- Don't bother if we have not heard from the link (and it's the proper sys/comp ids)
    if(!hb_heard_from || msgptr->sysid != hb_system_id || msgptr->compid != hb_comp_id) {
        return;
    }
    uint16_t seq_received = (uint16_t)msgptr->seq;
    uint16_t packet_lost_count = 0;
    //-- Account for overflow during packet loss
    if(seq_received < hb_seq_expected) {
        packet_lost_count = (seq_received + 255) - hb_seq_expected;
    } else {
        packet_lost_count = seq_received - hb_seq_expected;
    }
    hb_seq_expected = msgptr->seq + 1;
    link_status.packets_lost += packet_lost_count;
  }
#endif
//================================================================================================= 

void decodeGCStoFC() {
  if ((set.gs_io == gs_ser) || (set.gs_io == gs_bt) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt)) { // if any GCS I/O requested
    #if (defined Mav_Print_All_Msgid) || (defined Debug_All)
      log.printf("GCS to FC - msgid = %3d \n",  G2Fmsg.msgid);
    #endif

   apo_sysid = G2Fmsg.sysid;
   apo_compid = G2Fmsg.compid;

    switch(G2Fmsg.msgid) {
       case MAVLINK_MSG_ID_HEARTBEAT:    // #0
       
          gshb_millis = millis();
          gshbGood = true;
          
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Mav_Debug_GCS_Heartbeat                     
            apo_type = mavlink_msg_heartbeat_get_type(&G2Fmsg); 
            apo_autopilot = mavlink_msg_heartbeat_get_autopilot(&G2Fmsg);
            apo_base_mode = mavlink_msg_heartbeat_get_base_mode(&G2Fmsg);
            apo_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&G2Fmsg);
            apo_system_status = mavlink_msg_heartbeat_get_system_status(&G2Fmsg);
            apo_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&G2Fmsg);
  

            log.print("=====>GCS to FC: #0 Heartbeat: ");  
            log.print("apo_sysid="); log.print(ap_sysid);   
            log.print("  apo_compid="); log.print(ap_compid);                     
            log.print("  apo_type="); log.print(ap_type);   
            log.print("  apo_autopilot="); log.print(ap_autopilot); 
            log.print("  apo_base_mode="); log.print(ap_base_mode); 
            log.print(" apo_custom_mode="); log.print(ap_custom_mode);
            log.print("  apo_system_status="); log.print(ap_system_status); 
            log.print("  apo_mavlink_version="); log.print(ap_mavlink_version);      
            log.println();
          #endif   
          break;
          
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:  // #20  from GCS
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Debug_Param_Request_Read 
            apo_targsys = mavlink_msg_param_request_read_get_target_system(&G2Fmsg);        
            mavlink_msg_param_request_read_get_param_id(&G2Fmsg, gcs_req_param_id);
            gcs_req_param_index = mavlink_msg_param_request_read_get_param_index(&G2Fmsg);                  

            log.print("=====>GCS to FC: #20 Param_Request_Read: ");           
            log.print("apo_targsys="); log.print(apo_targsys);   
            log.print("  gcs_req_param_id="); log.print(gcs_req_param_id);          
            log.print("  gcs_req_param_index="); log.print(gcs_req_param_index);    
            log.println();  
              
            mavParamRequestRead(gcs_req_param_index, gcs_req_param_id); 
            
          #endif
          break;
          
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:  // #21  To FC from GCS
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Debug_Param_Request_List 
            apo_targsys = mavlink_msg_param_request_list_get_target_system(&G2Fmsg);  
            apo_targcomp = mavlink_msg_param_request_list_get_target_component(&G2Fmsg);                                  

            log.print("=====>GCS to FC: #21 Param_Request_List: ");           
            log.print("apo_targsys="); log.print(apo_targsys); 
            log.print(" apo_targcomp="); log.print(apo_targcomp);                    
            log.println();         
          #endif
          break;
          
         case MAVLINK_MSG_ID_MISSION_REQUEST_INT:  // #51 To FC from GCS
            apo_targsys = mavlink_msg_mission_request_int_get_target_system(&G2Fmsg);
            apo_targcomp = mavlink_msg_mission_request_int_get_target_component(&G2Fmsg);
            apo_seq = mavlink_msg_mission_request_int_get_seq(&G2Fmsg); 
            apo_mission_type = mavlink_msg_mission_request_int_get_seq(&G2Fmsg);       
                          
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Mav_Debug_Missions
            log.print("=====>GCS to FC: #51 Mission_Request_Int: ");           
            log.print("apo_targsys="); log.print(apo_targsys);   
            log.print("  apo_targcomp="); log.print(apo_targcomp);          
            log.print("  apo_seq="); log.print(apo_seq);    
            log.print("  apo_mission_type="); log.print(apo_mission_type);    // Mav2
            log.println();
          #endif
          break;  

        case MAVLINK_MSG_ID_COMMAND_LONG:        // #76 To FC from GCS
            apo_targsys = mavlink_msg_command_long_get_target_system(&G2Fmsg);
            apo_targcomp = mavlink_msg_command_long_get_target_component(&G2Fmsg);
            apo76_command = mavlink_msg_command_long_get_command(&G2Fmsg);      
            apo76_confirmation = mavlink_msg_command_long_get_confirmation(&G2Fmsg);  
            apo76_param[0] = mavlink_msg_command_long_get_param1(&G2Fmsg);  
            apo76_param[1] = mavlink_msg_command_long_get_param2(&G2Fmsg); 
            apo76_param[2] = mavlink_msg_command_long_get_param3(&G2Fmsg); 
            apo76_param[3] = mavlink_msg_command_long_get_param4(&G2Fmsg); 
            apo76_param[4] = mavlink_msg_command_long_get_param5(&G2Fmsg); 
            apo76_param[5] = mavlink_msg_command_long_get_param6(&G2Fmsg); 
            apo76_param[6] = mavlink_msg_command_long_get_param7(&G2Fmsg);      

            apo_sysid = Device_sysid;                    // Reply to this device. From config.h MP is 255, QGC default is 0
            apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
            apo_targsys = 1;                             // FC
            apo_targcomp = 1;                            // FC
            
            mavlink_msg_command_long_pack(apo_sysid, apo_compid, &G2Fmsg,
                    apo_targsys, apo_targcomp, apo76_command, apo76_confirmation, apo76_param[0], apo76_param[1], 
                    apo76_param[2], apo76_param[3], apo76_param[4], apo76_param[5], apo76_param[6]); 
               
            sendToFC(76);  // #76    
           
            #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Debug_Commands
              log.print("=====>GCS to FC: #76 Command_Long: ");           
              log.print("apo_targsys="); log.print(apo_targsys);   
              log.print("  apo_targcomp="); log.print(apo_targcomp);          
              log.print("  apo76_command="); log.print(apo76_command);    
              log.print("  apo76_confirmation="); log.print(apo76_confirmation);    
              for (int i = 0 ; i < 7 ; i++) {
                 log.printf("  apo76_param%u=%5.3f  ", i+1, apo76_param[i]); 
              }
              log.println();
            #endif 
            break;       
        default:
          if (!mavGood) break;
          #if defined Debug_All || defined Debug_GCS_Up || defined Debug_GCS_Unknown
            log.print("=====>GCS to FC: ");
            log.print("Unknown Message ID #");
            log.print(G2Fmsg.msgid);
            log.println(" Ignored"); 
          #endif

          break;
    }
  }
}
//================================================================================================= 
void sendToFC(uint32_t msg_id) {
  
  if (set.fc_io == fc_ser)  {   // Serial to FC
    len = mavlink_msg_to_send_buffer(FCbuf, &G2Fmsg);
    fcSerial.write(FCbuf,len);  
         
    #if defined  Debug_FC_Up || defined Debug_GCS_Up
      if (msg_id) {    //  dont print heartbeat - too much info
        log.printf("Sent to FC Serial from G2Fmsg: len=%d\n", len);
        printMavBuffer(&G2Fmsg);
      }  
    #endif    
  }

  #if (defined btBuiltin) 
    if (set.fc_io == fc_bt)  {   // BT to FC
        bool msgSent = Send_Bluetooth(&G2Fmsg);  
        msgSent = msgSent;  // silence irritating compiler warning 
        #ifdef  Debug_FC_Up
          log.print("Sent to FC Bluetooth from G2Fmsg: msgSent="); log.println(msgSent);
          if (msgSent) printMavBuffer(&G2Fmsg);
        #endif     
    }
  #endif

  #if (defined wifiBuiltin)
    if (set.fc_io == fc_wifi) {  // WiFi to FC
      if (wifiSuGood) { 
        if (set.mav_wfproto == tcp)  { // TCP  
           active_client_idx = 0;             // tcp_client[0] is reserved for inbound client (FC side)
           bool msgSent = sendTCP(&G2Fmsg);  // to FC  
           msgSent = msgSent;  // silence irritating compiler warning 
           #ifdef  Debug_GCS_Up
             log.print("Sent to FC WiFi TCP from G2Fmsg: msgSent="); log.println(msgSent);
             printMavBuffer(&G2Fmsg);
           #endif    
         }    
         
         if (set.mav_wfproto == udp)  { // UDP 

          if ((set.wfmode == ap_sta) || (set.wfmode == sta)) {// if AP_STA or STA mode                 
            active_udp_obj_idx = 0;                            // Use STA UDP object for FC send     
            udp_read_port = set.udp_remotePort;           
            udp_send_port = set.udp_localPort;                   
          } else {
            if (set.wfmode == ap) {
              active_udp_obj_idx = 1;                          // Use AP UDP object for FC send 
              udp_read_port = set.udp_localPort;  
              udp_send_port = set.udp_remotePort;                            
            }               
          }   
           UDP_remoteIP = udpremoteip[0];                     // Fc side can only ever have 1 client
           //log.printf("Send to FC udpremoteip[0]=%s UDP_remoteIP= %s\n", udpremoteip[0].toString().c_str(), UDP_remoteIP.toString().c_str());                   
           bool msgSent = sendUDP(fc_side, &G2Fmsg);  // to FC    
           msgSent = msgSent;  // silence irritating compiler warning
           #if ( (defined  Debug_GCS_Up) || (defined Debug_sendUDP_FC) ) 
             log.print("Sent to FC WiFi UDP from G2Fmsg: msgSent="); log.println(msgSent);
             if (msgSent) printMavBuffer(&G2Fmsg);
           #endif           
          }                                                             
      }
   }
  #endif       
}  
//================================================================================================= 

void MavToRingBuffer() {
  
    #if defined Slowdown_SITL_Input // then drop every second message
      static bool toggle = false;
      if (toggle == false) {
        toggle = true;
       return;
      } else {
        toggle = false;
      }
    #endif
  
      // MAIN Queue
      if (MavRingBuff.isFull()) {
        BufLedState = HIGH;
        log.println("MavRingBuff full. Ignoring new records");
     //   LogScreenPrintln("Mav buff full"); 
      }
       else {
        BufLedState = LOW;
        MavRingBuff.push(F2Rmsg);
        #if defined Mav_Debug_RingBuff
          log.print("Ring queue = "); 
          log.println(MavRingBuff.size());
        #endif
      }
  }
  
//================================================================================================= 

void Send_From_RingBuf_To_GCS() {   // Down to GCS (or other) from Ring Buffer
  
  if ((set.gs_io == gs_ser) || (set.gs_io == gs_bt) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.gs_sd == gs_on)) {

    #if (defined TEENSY3X)
      if (set.gs_io == gs_ser) {  // Serial
        len = mavlink_msg_to_send_buffer(GCSbuf, &R2Gmsg);
        gsSerial.write(GCSbuf,len);  

        #ifdef  Debug_GCS_Down
          log.printf("Sent from ring buffer to GCS Serial: len=%d\n", len);
          printMavBuffer(&R2Gmsg);
        #endif
      }
    #endif  
 
  #if (defined btBuiltin)
    if ((set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt))  {  // Bluetooth     
      bool msgSent = Send_Bluetooth(&R2Gmsg);  
      msgSent = msgSent;  // silence irritating compiler warning 
      #ifdef  Debug_GCS_Down
        log.print("Sent to GCS Bluetooth from R2Gmsg: msgSent="); log.println(msgSent);
        if (msgSent) printMavBuffer(&G2Fmsg);
      #endif   
    }
  #endif

  #if (defined wifiBuiltin)
    static bool msgSent = false;
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt)) { //  WiFi
      if (wifiSuGood) {
        if (set.mav_wfproto == tcp)  { // TCP      
          for (int i = 1 ; i < max_clients ; ++i) {       // send to each active client. Not to inbound client [0] (FC side)
            if (NULL != tcp_client[i]) {        // if active client
              active_client_idx = i;
                           
              prev_millis = millis();
                           
              msgSent = sendTCP(&R2Gmsg);  // to GCS side
              if ((millis() -  prev_millis) > 4) {
                PrintPeriod(false);  log.println(" TCP Write timeout =====================================>");  
              }
              
              #ifdef  Debug_GCS_Down
                if (msgSent) {
                  log.printf("Sent to GCS client %d by WiFi TCP: msgid=%d\n", active_client_idx+1, R2Gmsg.msgid);  
                }
              //  log.printf("Sent to GCS client %d by WiFi TCP: msgid=%d  msgSent=%d\n", active_client_idx+1, R2Gmsg.msgid, msgSent);
              //  printMavBuffer(&R2Gmsg);
              #endif
            }
          }    
        }
        
        if (set.mav_wfproto == udp)  { // UDP     
          if ((set.wfmode == ap_sta) || (set.wfmode == ap)) { // if AP_STA or AP mode                 
            active_udp_obj_idx = 1;                            // Use AP UDP object for FC send     
            udp_read_port = set.udp_localPort;           
            udp_send_port = set.udp_remotePort;                   
          } else {
            if (set.wfmode == sta) {
              active_udp_obj_idx = 0;                          // Use STA UDP object for FC send 
              //log.printf("active_udp_obj_idx set to 0\n");  
              udp_read_port = set.udp_localPort;  
              udp_send_port = set.udp_remotePort;                       
            }               
          }   
          
          #if defined UDP_Broadcast                     // broadcast to remote udp clients 
            UDP_remoteIP[3] = 255; 
            msgSent = sendUDP(gcs_side, &R2Gmsg);  // to GCS
            msgSent = msgSent; // stop stupid compiler warnings                       
          #else         
            for (int i = 1 ; i < max_clients ; i++) {   // send to each individual remote udp ip. Not to FC side client ip [0] 
              if (udpremoteip[i][0] != 0) {
                //log.printf("Non-zero ip i=%d  ip=%s\n", i, udpremoteip[i].toString().c_str());
                UDP_remoteIP =  udpremoteip[i];         // target the remote IP
                msgSent = sendUDP(gcs_side, &R2Gmsg);            // to GCS
                msgSent = msgSent; // stop stupid compiler warnings    
                #if (defined Debug_GCS_Down) || (defined Debug_sendUDP_GCS)
                  log.print(" Sent to GCS by WiFi UDP remote IP="); log.println(UDP_remoteIP);
                  printMavBuffer(&R2Gmsg);
                #endif                          
              }
           }  
         #endif  
  
        }                                                                     
      }  
    }
  #endif

  #if ((defined ESP32) || (defined ESP8266)) && (defined sdBuiltin) 
    if  (set.gs_sd == gs_on) {   //  GCS downlink telem to SD Card
      if (sdStatus == 3) {     //  if open for write
          File file = SD.open(cPath, FILE_APPEND);
          if(!file){
             log.println("Failed to open file for appending");
             sdStatus = 9;
             return;
            }

         memcpy(GCSbuf, (void*)&ap_time_unix_usec, sizeof(uint64_t));
         len=mavlink_msg_to_send_buffer(GCSbuf+sizeof(uint64_t), &R2Gmsg);

         if(file.write(GCSbuf, len+18)){   // 8 bytes plus some head room   
            } else {
            log.println("Append failed");
           }
         
          file.close();
        
          #ifdef  Debug_SD
            log.println("Passed down from Ring buffer to SD:");
            printMavBuffer(&R2Gmsg);
          #endif        
        }  
    }   
  #endif 
  }
}

//================================================================================================= 
#if (defined btBuiltin)
  bool Send_Bluetooth(mavlink_message_t* msgptr) {

    bool msgSent = false;
     
    uint16_t len = mavlink_msg_to_send_buffer(sendbuf, msgptr);
  
    size_t sent = SerialBT.write(sendbuf,len);

    if (sent == len) {
      msgSent = true;
      link_status.BT_packets_sent++;
    } else {
      link_status.BT_packets_lost++;
    }

    return msgSent;
  }
#endif
//================================================================================================= 
#if (defined wifiBuiltin)
  bool sendTCP(mavlink_message_t* msgptr) {
  if ( (!wifiSuGood) || ((!inbound_clientGood) && (!outbound_clientGood)) ) return false; 
    bool msgSent = false;
    uint16_t len = mavlink_msg_to_send_buffer(sendbuf, msgptr);
  
    size_t sent =  tcp_client[active_client_idx]->write(sendbuf,len);  

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    return msgSent;
  }
#endif
//================================================================================================= 
#if (defined wifiBuiltin)
  bool sendUDP(io_side_t io_side, mavlink_message_t* msgptr) {
    if (!wifiSuGood) return false;  

    // 2 possible udp objects, STA [0]  and    AP [1] 
        
    bool msgSent = false;

    if (msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      UDP_remoteIP[3] = 255;       // always broadcast a heartbeat, either from the GCS or from the FC                 
   //   log.print("Broadcast heartbeat UDP_remoteIP="); log.println(UDP_remoteIP.toString());     
    } 
    
    //if (io_side == fc_side) log.printf("Send UDP io_side=%d object=%d remote ip = %s:%d\n", io_side, active_udp_obj_idx, UDP_remoteIP.toString().c_str(), udp_send_port);

    udp_object[active_udp_obj_idx]->beginPacket(UDP_remoteIP, udp_send_port);  //  udp ports gets flipped for sta_ap mode
    
    uint16_t len = mavlink_msg_to_send_buffer(sendbuf, msgptr);
  
    size_t sent = udp_object[active_udp_obj_idx]->write(sendbuf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
      udp_object[active_udp_obj_idx]->flush();
    }

    bool endOK = udp_object[active_udp_obj_idx]->endPacket();
    endOK = endOK;  // silence irritating compiler warning  
 //   if (!endOK) log.printf("msgSent=%d   endOK=%d\n", msgSent, endOK);
    return msgSent;
  }
#endif
//================================================================================================= 

void CheckMavTimeouts() {
  // fence status update
  if (ap_fence_enabled && (millis()-ap_fence_last_update) > 2000) {
    ap_fence_enabled = false;
    ap_breach_status = 0;
    #if defined Mav_Debug_All || defined Mav_Debug_Fence
      log.print("#162 Fence Status timeout expired: ");
      log.print("  fence_enabled= ");  log.print(ap_fence_enabled?"true":"false");
      log.print("  breach_status= ");  log.println(ap_breach_status);
    #endif
  }
}
//=================================================================================================
void DecodeOneMavFrame() {
  
   #if defined Mav_Print_All_Msgid
     uint16_t sz = sizeof(R2Gmsg);
     log.printf("FC to GGS - msgid = %3d Msg size =%3d\n",  R2Gmsg.msgid, sz);
   #endif

   ap_sysid = R2Gmsg.sysid;
   ap_compid = R2Gmsg.compid;  

   switch(R2Gmsg.msgid) {
    
        case MAVLINK_MSG_ID_HEARTBEAT:    // #0   https://mavlink.io/en/messages/common.html
          ap_type_tmp = mavlink_msg_heartbeat_get_type(&R2Gmsg);   // Alex - don't contaminate the ap_type variable
          if (ap_type_tmp == 5 || ap_type_tmp == 6 || ap_type_tmp == 18 || ap_type_tmp == 26 || ap_type_tmp == 27) break;      
          // Ignore heartbeats from GCS (6) or Ant Trackers(5) or Onboard_Controllers(18) or Gremsy Gimbal(26) or ADSB (27))
          ap_type = ap_type_tmp;
          ap_autopilot = mavlink_msg_heartbeat_get_autopilot(&R2Gmsg);
          ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&R2Gmsg);
          ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&R2Gmsg);
          
          px4_main_mode = bit32Extract(ap_custom_mode,16, 8);
          px4_sub_mode = bit32Extract(ap_custom_mode,24, 8);
          px4_flight_stack = (ap_autopilot == MAV_AUTOPILOT_PX4);

          pitlab_flight_stack = (ap_autopilot == MAV_AUTOPILOT_INVALID);
          #if (defined PitLab)
            pitlab_flight_stack = true;
            ap_type = 1; // Force Plane (fixed wing)
          #endif
               
          ap_system_status = mavlink_msg_heartbeat_get_system_status(&R2Gmsg);
          ap_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&R2Gmsg);
          hb_millis=millis();  

          motArmed = ap_base_mode >> 7;  // motors armed!
          
          #if defined SET_HOME_AT_ARM_TIME  
            if (motArmed && (ap24_fixtype >= 6) && (!homGood)) {  // AND 3D Fix
              MarkHome();  // If motors armed for the first time, then mark this spot as home
            }
          #endif
          
          hb_count++; 
          
          if(!mavGood) {
            log.printf("hb_count=%d\n", hb_count);
            if(hb_count >= 3) {        // If  3 heartbeats from MavLink then we are connected
              mavGood=true;
              log.println("Mavlink good!");
              LogScreenPrintln("Mavlink good!");      
              }
            }
          #if (defined frBuiltin)       
            if (mavGood) {
              FrPort.PushMessage(0x5001, 0);    // Flight mode (from ap_base_mode)
              FrPort.PushMessage(0x5007, 1);    // Frame type (from ap_type 
            }
          #endif
           
          #if defined Mav_Debug_All || defined Mav_Debug_FC_Heartbeat
            log.print("Mavlink from FC #0 Heartbeat: ");  
            log.print("ap_sysid="); log.print(ap_sysid);   
            log.print("  ap_compid="); log.print(ap_compid);                      
            log.print("  ap_type(frame)="); log.print(ap_type);   
            log.print("  ap_autopilot="); log.print(ap_autopilot); 
            log.print("  ap_base_mode="); log.print(ap_base_mode); 
            log.print(" ap_custom_mode="); log.print(ap_custom_mode);
            log.print("  ap_system_status="); log.print(ap_system_status); 
            log.print("  ap_mavlink_version="); log.print(ap_mavlink_version);   

            if (px4_flight_stack) {         
              log.print(" px4_main_mode="); log.print(px4_main_mode); 
              log.print(" px4_sub_mode="); log.print(px4_sub_mode);  
              log.print(" ");log.print(PX4FlightModeName(px4_main_mode, px4_sub_mode));  
           }

           if (pitlab_flight_stack) {         
              log.print(" Pitlab flight stack detected");
           }           
            
            log.println();
          #endif

          
          break;
        case MAVLINK_MSG_ID_SYS_STATUS:   // #1
          if (!mavGood) break;

          ap_onboard_control_sensors_health = mavlink_msg_sys_status_get_onboard_control_sensors_health(&R2Gmsg);
          ap_voltage_battery1 = Get_Volt_Average1(mavlink_msg_sys_status_get_voltage_battery(&R2Gmsg));        // V  from Get_Volt_Average1()
          ap_current_battery1 = Get_Current_Average1(mavlink_msg_sys_status_get_current_battery(&R2Gmsg));     // dA,  100 = 1A
          if(ap_voltage_battery1> 21000) ap_ccell_count1= 6;
            else if (ap_voltage_battery1> 16800 && ap_ccell_count1!= 6) ap_ccell_count1= 5;
            else if(ap_voltage_battery1> 12600 && ap_ccell_count1!= 5) ap_ccell_count1= 4;
            else if(ap_voltage_battery1> 8400 && ap_ccell_count1!= 4) ap_ccell_count1= 3;
            else if(ap_voltage_battery1> 4200 && ap_ccell_count1!= 3) ap_ccell_count1= 2;
            else ap_ccell_count1= 0;
            
          pt_bat1_volts = ap_voltage_battery1 * 0.01F; // mV -> dV
          pt_bat1_amps = ap_current_battery1 * 0.1F;   // cA -> dA  Mavlink cA (100cA = A), 10dA = A for Frsky
                     
          #if defined Mav_Debug_All || defined Mav_Debug_SysStatus || defined Debug_Batteries
            log.print("Mavlink from FC #1 Sys_status: ");     
            log.print(" Sensor health=");
            log.print(ap_onboard_control_sensors_health);   // 32b bitwise 0: error, 1: healthy.
            log.print(" Bat volts=");
            log.print(ap_voltage_battery1 * 0.001F, 3);   // mV -> V
            log.print("  Bat amps=");
            log.print(ap_current_battery1 * 0.01F, 3);    // dA -> A
            log.print("  mAh="); log.print(bat1.mAh, 6);    
            log.print("  Total mAh="); log.print(bat1.tot_mAh, 3);  // Consumed so far, calculated in Average module
         
            log.print("  Bat1 cell count= "); 
            log.println(ap_ccell_count1);
          #endif

          #if defined Send_Sensor_Health_Messages
          if ((millis() - health_millis) > 5000) {
            health_millis = millis();
            if ( bit32Extract(ap_onboard_control_sensors_health, 5, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  // severity = 2
              strcpy(ap_text, "Bad GPS Health");
              Push_Text_Chunks_5000(0x5000);
            } else
          
            if ( bit32Extract(ap_onboard_control_sensors_health, 0, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad Gyro Health + 0x00 +0x00");
              Push_Text_Chunks_5000(0x5000);
            } else 
                 
            if ( bit32Extract(ap_onboard_control_sensors_health, 1, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad Accel Health");
              Push_Text_Chunks_5000(0x5000);
            } else
          
            if ( bit32Extract(ap_onboard_control_sensors_health, 2, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad Compass Health");
              Push_Text_Chunks_5000(0x5000);
            } else
            
            if ( bit32Extract(ap_onboard_control_sensors_health, 3, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  // severity = 2
              strcpy(ap_text, "Bad Baro Health");
              Push_Text_Chunks_5000(0x5000);
            } else
          
            if ( bit32Extract(ap_onboard_control_sensors_health, 8, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad LiDAR Health");
              Push_Text_Chunks_5000(0x5000);
            } else 
                 
            if ( bit32Extract(ap_onboard_control_sensors_health, 6, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad OptFlow Health + 0x00 +0x00");
              Push_Text_Chunks_5000(0x5000);
            } else
          
            if ( bit32Extract(ap_onboard_control_sensors_health, 22, 1) ) {
             ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad or No Terrain Data");
              Push_Text_Chunks_5000(0x5000);
            } else

            if ( bit32Extract(ap_onboard_control_sensors_health, 20, 1) ) {
             ap_severity = MAV_SEVERITY_CRITICAL;  
             strcpy(ap_text, "Geofence Breach");
             Push_Text_Chunks_5000(0x5000);
           } else  
                 
            if ( bit32Extract(ap_onboard_control_sensors_health, 21, 1) ) {
              ap_severity = MAV_SEVERITY_CRITICAL;  
              strcpy(ap_text, "Bad AHRS");
              Push_Text_Chunks_5000(0x5000);
            } else
          
           if ( bit32Extract(ap_onboard_control_sensors_health, 16, 1) ) {
             ap_severity = MAV_SEVERITY_CRITICAL;  
             strcpy(ap_text, "No RC Receiver");
             Push_Text_Chunks_5000(0x5000);
           } else  

           if ( bit32Extract(ap_onboard_control_sensors_health, 24, 1) ) {
             ap_severity = MAV_SEVERITY_CRITICAL;  
             strcpy(ap_text, "Bad Logging");
             Push_Text_Chunks_5000(0x5000);
           } 
         }                  
         #endif     

         #if (defined frBuiltin)
           FrPort.PushMessage(0x5003, 0);
         #endif             
     
         break;
        case MAVLINK_MSG_ID_SYSTEM_TIME:   // #2
          if (!mavGood) break;
          ap_time_unix_usec= (mavlink_msg_system_time_get_time_unix_usec(&R2Gmsg));    // us
          ap_time_boot_ms= (mavlink_msg_system_time_get_time_boot_ms(&R2Gmsg));        //  ms
          if ( ap_time_unix_usec != 0 ) {
            timeGood = true;
          }
          #if defined Mav_Debug_All || defined Mav_Debug_System_Time
            log.print("Mavlink from FC #2 System_Time: ");        
            log.print(" Unix secs="); log.print((float)(ap_time_unix_usec/1E6), 6);  
            log.print("  Boot secs="); log.println((float)(ap_time_boot_ms/1E3), 0);   
          #endif
          break;                   
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:   // #20 - UPLINK TO UAV
          if (!mavGood) break;
          break;     
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:   // #21 - UPLINK  TO UAV
          if (!mavGood) break;
          break;  
        case MAVLINK_MSG_ID_PARAM_VALUE:          // #22
          if (!mavGood) break;        
          len=mavlink_msg_param_value_get_param_id(&R2Gmsg, ap22_param_id);
          ap22_param_value=mavlink_msg_param_value_get_param_value(&R2Gmsg);
          ap22_param_count=mavlink_msg_param_value_get_param_count(&R2Gmsg);
          ap22_param_index=mavlink_msg_param_value_get_param_index(&R2Gmsg);

          #if (defined frBuiltin)
            mt20_row = FrPort.MatchWaitingParamRequests(ap22_param_id);
            // log.printf("matched row mt20_row=%d\n", mt20_row);
            if (mt20_row != 0xffff) {       // if matched

              #if defined Mav_Debug_All || defined Debug_Mavlite
                  log.print("Mavlink return from FC #22 Param_Value for MavLITE: ");
                  log.print("ap22_param_id="); log.print(ap22_param_id);
                  log.print(" ap22_param_value="); log.println(ap22_param_value, 3);             
              #endif 

              mt20[mt20_row].inuse = 0;
              FrPort.PushMessage(0x16, 0);    // MavLite PARAM_VALUE ( #22 ) use ap-param-id and ap22_param_value                       
              break;            
            }
          #endif
          
          switch(ap22_param_index) {      // if #define Battery_mAh_Source !=1 these will never arrive
            case 356:         // Bat1 Capacity
              ap_bat1_capacity = ap22_param_value;
              #if (defined frBuiltin)
                FrPort.PushMessage(0x5007, 4);    // Bat1 capacity
              #endif  
                    
              #if defined Mav_Debug_All || defined Debug_Batteries
                log.print("Mavlink from FC #22 Param_Value: ");
                log.print("bat1 capacity=");
                log.println(ap_bat1_capacity);
              #endif
              break;
            case 364:         // Bat2 Capacity
              ap_bat2_capacity = ap22_param_value;
              #if (defined frBuiltin)
                FrPort.PushMessage(0x5007, 5);    // Bat2 capacity
              #endif  
              
              ap_bat_paramsRead = true;
              #if defined Mav_Debug_All || defined Debug_Batteries
                log.print("Mavlink from FC #22 Param_Value: ");
                log.print("bat2 capacity=");
                log.println(ap_bat2_capacity);
              #endif             
              break;
          } 
             
          #if defined Mav_Debug_All || defined Mav_Debug_Params || defined Mav_List_Params || defined Debug_Mavlite
            log.print("Mavlink from FC #22 Param_Value: ");
            log.print("param_id=");
            log.print(ap22_param_id);
            log.print("  \tparam_value=");
            log.print(ap22_param_value, 4);
            log.print("  param_count=");
            log.print(ap22_param_count);
            log.print("  param_index=");
            log.println(ap22_param_index);
          #endif       
          break;   

        case MAVLINK_MSG_ID_PARAM_SET:   // #23 - UPLINK TO UAV
          if (!mavGood) break;
          break;  
                     
        case MAVLINK_MSG_ID_GPS_RAW_INT:          // #24
          if (!mavGood) break;        
          ap24_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&R2Gmsg);                   // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
          ap24_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&R2Gmsg);    // number of visible satellites
          if(ap24_fixtype > 2)  {
            ap24_lat = mavlink_msg_gps_raw_int_get_lat(&R2Gmsg);
            ap24_lon = mavlink_msg_gps_raw_int_get_lon(&R2Gmsg);
            ap24_amsl = mavlink_msg_gps_raw_int_get_alt(&R2Gmsg);                    // 1m =1000 
            ap24_eph = mavlink_msg_gps_raw_int_get_eph(&R2Gmsg);                       // GPS HDOP 
            ap24_epv = mavlink_msg_gps_raw_int_get_epv(&R2Gmsg);                       // GPS VDOP 
            ap24_vel = mavlink_msg_gps_raw_int_get_vel(&R2Gmsg);                       // GPS ground speed (m/s * 100)
            ap24_cog = mavlink_msg_gps_raw_int_get_cog(&R2Gmsg);                       // Course over ground (NOT heading) in degrees * 100
     // mav2
           ap24_alt_ellipsoid = mavlink_msg_gps_raw_int_get_alt_ellipsoid(&R2Gmsg);    // mm    Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
           ap24_h_acc = mavlink_msg_gps_raw_int_get_h_acc(&R2Gmsg);                    // mm    Position uncertainty. Positive for up.
           ap24_v_acc = mavlink_msg_gps_raw_int_get_v_acc(&R2Gmsg);                    // mm    Altitude uncertainty. Positive for up.
           ap24_vel_acc = mavlink_msg_gps_raw_int_get_vel_acc(&R2Gmsg);                // mm    Speed uncertainty. Positive for up.
           ap24_hdg_acc = mavlink_msg_gps_raw_int_get_hdg_acc(&R2Gmsg);                // degE5   Heading / track uncertainty       

           cur.lat =  (float)ap24_lat / 1E7;
           cur.lon = (float)ap24_lon / 1E7;
           cur.alt = ap24_amsl / 1E3;
           
          }
          #if defined Mav_Debug_All || defined Mav_Debug_GPS_Raw
            log.print("Mavlink from FC #24 GPS_RAW_INT: ");  
            log.print("ap24_fixtype="); log.print(ap24_fixtype);
            if (ap24_fixtype==0) log.print(" No GPS");
              else if (ap24_fixtype==1) log.print(" No Fix");
              else if (ap24_fixtype==2) log.print(" 2D Fix");
              else if (ap24_fixtype==3) log.print(" 3D Fix");
              else if (ap24_fixtype==4) log.print(" DGPS/SBAS aided");
              else if (ap24_fixtype==5) log.print(" RTK Float");
              else if (ap24_fixtype==6) log.print(" RTK Fixed");
              else if (ap24_fixtype==7) log.print(" Static fixed");
              else if (ap24_fixtype==8) log.print(" PPP");
              else log.print(" Unknown");

            log.print("  sats visible="); log.print(ap24_sat_visible);
            log.print("  latitude="); log.print((float)(ap24_lat)/1E7, 7);
            log.print("  longitude="); log.print((float)(ap24_lon)/1E7, 7);
            log.print("  gps alt amsl="); log.print((float)(ap24_amsl)/1E3, 1);
            log.print("  eph (hdop)="); log.print((float)ap24_eph);                // HDOP
            log.print("  epv (vdop)="); log.print((float)ap24_epv);
            log.print("  vel="); log.print((float)ap24_vel / 100, 3);              // GPS ground speed (m/s)
            log.print("  cog="); log.print((float)ap24_cog / 100, 1);              // Course over ground in degrees
            //  mav2
            log.print("  alt_ellipsoid)="); log.print(ap24_alt_ellipsoid / 1000, 2);      // alt_ellipsoid in mm
            log.print("  h_acc="); log.print(ap24_h_acc);                          // Position uncertainty in mm. Positive for up.
            log.print("  v_acc="); log.print(ap24_v_acc);                          // Altitude uncertainty in mm. Positive for up.
            log.print("  ap24_vel_acc="); log.print(ap24_vel_acc);                 // Speed uncertainty. Positive for up.
            log.print("  ap24_hdg_acc="); log.print(ap24_hdg_acc);                 // degE5   Heading / track uncertainty 
            log.println();
          #endif 
          
          #if (defined frBuiltin)
             FrPort.PushMessage(0x800, 0);   // 0x800 Lat
             FrPort.PushMessage(0x800, 1);   // 0x800 Lon
             FrPort.PushMessage(0x5002, 0);  // 0x5002 GPS Status
             FrPort.PushMessage(0x5004, 0);  // 0x5004 Home     
          #endif  
    
              
          break;
        case MAVLINK_MSG_ID_SCALED_IMU:   // #26

          if (!mavGood) break;        
          ap26_xacc = mavlink_msg_scaled_imu_get_xacc(&R2Gmsg);                 
          ap26_yacc = mavlink_msg_scaled_imu_get_yacc(&R2Gmsg);
          ap26_zacc = mavlink_msg_scaled_imu_get_zacc(&R2Gmsg);
          ap26_xgyro = mavlink_msg_scaled_imu_get_xgyro(&R2Gmsg);                 
          ap26_ygyro = mavlink_msg_scaled_imu_get_ygyro(&R2Gmsg);
          ap26_zgyro = mavlink_msg_scaled_imu_get_zgyro(&R2Gmsg);
          ap26_xmag = mavlink_msg_scaled_imu_get_xmag(&R2Gmsg);                 
          ap26_ymag = mavlink_msg_scaled_imu_get_ymag(&R2Gmsg);
          ap26_zmag = mavlink_msg_scaled_imu_get_zmag(&R2Gmsg);
          //  mav2
          ap26_temp = mavlink_msg_scaled_imu_get_temperature(&R2Gmsg);         
          
          #if defined Mav_Debug_All || defined Mav_Debug_Scaled_IMU
            log.print("Mavlink from FC #26 Scaled_IMU: ");
            log.print("xacc="); log.print((float)ap26_xacc / 1000, 3); 
            log.print("  yacc="); log.print((float)ap26_yacc / 1000, 3); 
            log.print("  zacc="); log.print((float)ap26_zacc / 1000, 3);
            log.print("  xgyro="); log.print((float)ap26_xgyro / 1000, 3); 
            log.print("  ygyro="); log.print((float)ap26_ygyro / 1000, 3); 
            log.print("  zgyro="); log.print((float)ap26_zgyro / 1000, 3);
            log.print("  xmag="); log.print((float)ap26_xmag / 1000, 3); 
            log.print("  ymag="); log.print((float)ap26_ymag / 1000, 3); 
            log.print("  zmag="); log.print((float)ap26_zmag / 1000, 3);  
            log.print("  temp="); log.println((float)ap26_temp / 100, 2);    // cdegC                              
          #endif 

          break; 
          
        case MAVLINK_MSG_ID_RAW_IMU:   // #27
        #if defined Decode_Non_Essential_Mav
          if (!mavGood) break;        
          ap27_xacc = mavlink_msg_raw_imu_get_xacc(&R2Gmsg);                 
          ap27_yacc = mavlink_msg_raw_imu_get_yacc(&R2Gmsg);
          ap27_zacc = mavlink_msg_raw_imu_get_zacc(&R2Gmsg);
          ap27_xgyro = mavlink_msg_raw_imu_get_xgyro(&R2Gmsg);                 
          ap27_ygyro = mavlink_msg_raw_imu_get_ygyro(&R2Gmsg);
          ap27_zgyro = mavlink_msg_raw_imu_get_zgyro(&R2Gmsg);
          ap27_xmag = mavlink_msg_raw_imu_get_xmag(&R2Gmsg);                 
          ap27_ymag = mavlink_msg_raw_imu_get_ymag(&R2Gmsg);
          ap27_zmag = mavlink_msg_raw_imu_get_zmag(&R2Gmsg);
          ap27_id = mavlink_msg_raw_imu_get_id(&R2Gmsg);         
          //  mav2
          ap27_temp = mavlink_msg_raw_imu_get_temperature(&R2Gmsg);           
          #if defined Mav_Debug_All || defined Mav_Debug_Raw_IMU
            log.print("Mavlink from FC #27 Raw_IMU: ");
            log.print("accX="); log.print((float)ap27_xacc / 1000); 
            log.print("  accY="); log.print((float)ap27_yacc / 1000); 
            log.print("  accZ="); log.println((float)ap27_zacc / 1000);
            log.print("  xgyro="); log.print((float)ap27_xgyro / 1000, 3); 
            log.print("  ygyro="); log.print((float)ap27_ygyro / 1000, 3); 
            log.print("  zgyro="); log.print((float)ap27_zgyro / 1000, 3);
            log.print("  xmag="); log.print((float)ap27_xmag / 1000, 3); 
            log.print("  ymag="); log.print((float)ap27_ymag / 1000, 3); 
            log.print("  zmag="); log.print((float)ap27_zmag / 1000, 3);
            log.print("  id="); log.print((float)ap27_id);             
            log.print("  temp="); log.println((float)ap27_temp / 100, 2);    // cdegC               
          #endif 
        #endif             
          break; 
    
        case MAVLINK_MSG_ID_SCALED_PRESSURE:         // #29
        #if defined Decode_Non_Essential_Mav
          if (!mavGood) break;        
          ap_press_abs = mavlink_msg_scaled_pressure_get_press_abs(&R2Gmsg);
          ap_temperature = mavlink_msg_scaled_pressure_get_temperature(&R2Gmsg);
          #if defined Mav_Debug_All || defined Mav_Debug_Scaled_Pressure
            log.print("Mavlink from FC #29 Scaled_Pressure: ");
            log.print("  press_abs=");  log.print(ap_press_abs,1);
            log.print("hPa  press_diff="); log.print(ap_press_diff, 3);
            log.print("hPa  temperature=");  log.print((float)(ap_temperature)/100, 1); 
            log.println("C");             
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
            log.print("Mavlink from FC #30 Attitude: ");      
            log.print(" ap_roll degs=");
            log.print(ap_roll, 1);
            log.print(" ap_pitch degs=");   
            log.print(ap_pitch, 1);
            log.print(" ap_yaw degs=");         
            log.println(ap_yaw, 1);
          #endif  
                     
          #if (defined frBuiltin)
            FrPort.PushMessage(0x5006, 0 );  // 0x5006 Attitude   
          #endif        

          break;  
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:     // #33
          if ((!mavGood) || (ap24_fixtype < 3)) break;  
          ap33_lat = mavlink_msg_global_position_int_get_lat(&R2Gmsg);             // Latitude, expressed as degrees * 1E7
          ap33_lon = mavlink_msg_global_position_int_get_lon(&R2Gmsg);             // Pitch angle (rad, -pi..+pi)
          ap33_amsl = mavlink_msg_global_position_int_get_alt(&R2Gmsg);            // Altitude above mean sea level (millimeters)
          ap33_alt_ag = mavlink_msg_global_position_int_get_relative_alt(&R2Gmsg); // Altitude above ground (millimeters)
          ap33_vx = mavlink_msg_global_position_int_get_vx(&R2Gmsg);               //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
          ap33_vy = mavlink_msg_global_position_int_get_vy(&R2Gmsg);               //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
          ap33_vz = mavlink_msg_global_position_int_get_vz(&R2Gmsg);               // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
          ap33_gps_hdg = mavlink_msg_global_position_int_get_hdg(&R2Gmsg);         // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees        
 
          cur.lat = (float)ap33_lat / 1E7;
          cur.lon = (float)ap33_lon / 1E7;
          cur.alt = ap33_amsl / 1E3;
          cur.alt_ag = ap33_alt_ag / 1E3;  // used by info disply
          cur.hdg = ap33_gps_hdg / 100;

          #if defined Mav_Debug_All || defined Mav_Debug_GPS_Int
            log.print("Mavlink from FC #33 GPS Int: ");
            log.print(" ap_lat="); log.print((float)ap33_lat / 1E7, 6);
            log.print(" ap_lon="); log.print((float)ap33_lon / 1E7, 6);
            log.print(" ap_amsl="); log.print((float)ap33_amsl / 1E3, 0);
            log.print(" ap33_alt_ag="); log.print((float)ap33_alt_ag / 1E3, 1);           
            log.print(" ap33_vx="); log.print((float)ap33_vx / 100, 2);
            log.print(" ap33_vy="); log.print((float)ap33_vy / 100, 2);
            log.print(" ap33_vz="); log.print((float)ap33_vz / 100, 2);
            log.print(" ap33_gps_hdg="); log.println((float)ap33_gps_hdg / 100, 1);
          #endif  
                
          break;  
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:         // #35
          if (!mavGood) break; 
          ap_rssi35 = mavlink_msg_rc_channels_raw_get_rssi(&R2Gmsg);
          rssi35 = true;  
               
          if ((!rssi65) && (!rssi109)) { // If no #65 and no #109 received, then use #35
            rssiGood=true;   
            #if defined Rssi_In_Percent
              ap_rssi = ap_rssi35;          //  Percent
            #else           
              ap_rssi = ap_rssi35 / 2.54;  // 254 -> 100%    
            #endif 
            #if (defined frBuiltin)
              #if (not defined Rssi_Pacemaker)
                FrPort.PushMessage(0xF101, 0);   // 0xF101 RSSI 
                rssi_millis = millis();
              #endif  
            #endif               
                          
            #if defined Mav_Debug_All || defined Debug_Rssi || defined Mav_Debug_RC
              #ifndef RSSI_Override
                log.print("Auto RSSI_Source===>  ");
              #endif
            #endif     
          }

          #if defined Mav_Debug_All || defined Debug_Rssi || defined Mav_Debug_RC
            log.print("Mavlink from FC #35 RC_Channels_Raw: ");                        
            log.print("  ap_rssi35=");  log.print(ap_rssi35);   // 0xff -> 100%
            log.print("  rssiGood=");  log.println(rssiGood); 
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
            log.print("Mavlink from FC #36 servo_output: ");
            log.print("ap_port="); log.print(ap_port); 
            log.print(" PWM: ");
            for (int i=0 ; i < 8; i++) {
              log.print(" "); 
              log.print(i+1);
              log.print("=");  
              log.print(ap_servo_raw[i]);   
            }                         
            log.println();     
          #endif  
          #if (defined frBuiltin)
            #if defined PlusVersion
              FrPort.PushMessage(0x50F1, 0);   // 0x50F1  SERVO_OUTPUT_RAW
            #endif
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
                     
            #if defined Mav_Debug_All || defined Mav_Debug_Missions
              log.print("Mavlink from FC #39 Mission Item: ");
              log.print("ap_ms_seq="); log.print(ap_ms_seq);  
              log.print(" ap_ms_frame="); log.print(ap_ms_frame);   
              log.print(" ap_ms_command="); log.print(ap_ms_command);   
              log.print(" ap_ms_current="); log.print(ap_ms_current);   
              log.print(" ap_ms_autocontinue="); log.print(ap_ms_autocontinue);  
              log.print(" ap_ms_param1="); log.print(ap_ms_param1, 7);   
              log.print(" ap_ms_param2="); log.print(ap_ms_param2, 7);   
              log.print(" ap_ms_param3="); log.print(ap_ms_param3, 7);  
              log.print(" ap_ms_param4="); log.print(ap_ms_param4, 7); 
              log.print(" ap_ms_x="); log.print(ap_ms_x, 7);   
              log.print(" ap_ms_y="); log.print(ap_ms_y, 7);   
              log.print(" ap_ms_z="); log.print(ap_ms_z,0); 
              log.print(" ap_mission_type="); log.print(ap_mission_type); 
              log.println();    
            #endif
            
            if (ap_ms_seq > Max_Waypoints) {
              log.println(" Max Waypoints exceeded! Waypoint ignored.");
              break;
            }

             WP[ap_ms_seq-1].lat = ap_ms_x;     //  seq = 1 goes into slot [0]
             WP[ap_ms_seq-1].lon = ap_ms_y;
             
          break;                    
        case MAVLINK_MSG_ID_MISSION_CURRENT:         // #42 should come down regularly as part of EXTENDED_status group
          if (!mavGood) break;   
            ap_ms_seq =  mavlink_msg_mission_current_get_seq(&R2Gmsg);  
            
            #if defined Mav_Debug_All || defined Mav_Debug_Missions
            if (ap_ms_seq) {
              log.print("Mavlink from FC #42 Mission Current: ");
              log.print("ap_mission_current="); log.println(ap_ms_seq);   
            }
            #endif 
              
            if (ap_ms_seq > 0) ap_ms_current_flag = true;     //  Ok to send passthru frames 
  
          break; 
        case MAVLINK_MSG_ID_MISSION_COUNT :          // #44   received back after #43 Mission_Request_List sent
        #if defined Request_Missions_From_FC || defined Request_Mission_Count_From_FC
          if (!mavGood) break;  
            ap_mission_count =  mavlink_msg_mission_count_get_count(&R2Gmsg); 

            #if (defined frBuiltin)
              FrPort.PushMessage(0x5007, 6);    // fr parameters - mission count
            #endif  
       
            #if defined Mav_Debug_All || defined Mav_Debug_Missions
              log.print("Mavlink from FC #44 Mission Count: ");
              log.print("ap_mission_count="); log.println(ap_mission_count);   
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
              log.print("Mavlink from FC #62 Nav_Controller_Output - (+Waypoint): ");
              log.print("ap_nav_roll="); log.print(ap_nav_roll, 3);  
              log.print(" ap_nav_pitch="); log.print(ap_nav_pitch, 3);   
              log.print(" ap_nav_bearing="); log.print(ap_nav_bearing);   
              log.print(" ap_target_bearing="); log.print(ap_target_bearing);   
              log.print(" ap_wp_dist="); log.print(ap_wp_dist);  
              log.print(" ap_alt_error="); log.print(ap_alt_error, 2);   
              log.print(" ap_aspd_error="); log.print(ap_aspd_error, 2);   
              log.print(" ap_xtrack_error="); log.print(ap_xtrack_error, 2);  
              log.println();    
            #endif
            #if (defined frBuiltin)
              #if defined PlusVersion  
                FrPort.PushMessage(0x5009, 0);  // 0x5009 Waypoints  
              #endif  
            #endif              

          break;     
        case MAVLINK_MSG_ID_RC_CHANNELS:             // #65
          if (!mavGood) break; 

            ap65_chcnt = mavlink_msg_rc_channels_get_chancount(&R2Gmsg);
            ap65_chan_raw[0] = mavlink_msg_rc_channels_get_chan1_raw(&R2Gmsg);   
            ap65_chan_raw[1] = mavlink_msg_rc_channels_get_chan2_raw(&R2Gmsg);
            ap65_chan_raw[2] = mavlink_msg_rc_channels_get_chan3_raw(&R2Gmsg);   
            ap65_chan_raw[3] = mavlink_msg_rc_channels_get_chan4_raw(&R2Gmsg);  
            ap65_chan_raw[4] = mavlink_msg_rc_channels_get_chan5_raw(&R2Gmsg);   
            ap65_chan_raw[5] = mavlink_msg_rc_channels_get_chan6_raw(&R2Gmsg);
            ap65_chan_raw[6] = mavlink_msg_rc_channels_get_chan7_raw(&R2Gmsg);   
            ap65_chan_raw[7] = mavlink_msg_rc_channels_get_chan8_raw(&R2Gmsg);  
            ap65_chan_raw[8] = mavlink_msg_rc_channels_get_chan9_raw(&R2Gmsg);   
            ap65_chan_raw[9] = mavlink_msg_rc_channels_get_chan10_raw(&R2Gmsg);
            ap65_chan_raw[10] = mavlink_msg_rc_channels_get_chan11_raw(&R2Gmsg);   
            ap65_chan_raw[11] = mavlink_msg_rc_channels_get_chan12_raw(&R2Gmsg); 
            ap65_chan_raw[12] = mavlink_msg_rc_channels_get_chan13_raw(&R2Gmsg);   
            ap65_chan_raw[13] = mavlink_msg_rc_channels_get_chan14_raw(&R2Gmsg);
            ap65_chan_raw[14] = mavlink_msg_rc_channels_get_chan15_raw(&R2Gmsg);   
            ap65_chan_raw[15] = mavlink_msg_rc_channels_get_chan16_raw(&R2Gmsg);
            ap65_chan_raw[16] = mavlink_msg_rc_channels_get_chan17_raw(&R2Gmsg);   
            ap65_chan_raw[17] = mavlink_msg_rc_channels_get_chan18_raw(&R2Gmsg);
            ap65_rssi = mavlink_msg_rc_channels_get_rssi(&R2Gmsg);   // Receive RSSI 0: 0%, 254: 100%, 255: invalid/unknown     
   
            rssi65 = true;  
             
            if (!rssi109) { // If no #109 received, then use #65
              rssiGood=true; 
              #if defined Rssi_In_Percent
                ap_rssi = ap65_rssi;          //  Percent
              #else           
                ap_rssi = ap65_rssi / 2.54;  // 254 -> 100%
              #endif   
              #if (defined frBuiltin)
                #if (not defined Rssi_Pacemaker)
                  FrPort.PushMessage(0xF101, 0);   // 0xF101 RSSI 
                  rssi_millis = millis();    
                #endif     
              #endif  
                                   
              #if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_RC
                #ifndef RSSI_Override
                  log.print("Auto RSSI_Source===>  ");
                #endif
              #endif     
              }
             
            #if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_RC
              log.print("Mavlink from FC #65 RC_Channels: ");
              log.print("ap65_chcnt="); log.print(ap65_chcnt); 
              log.print(" PWM: ");
              for (int i=0 ; i < ap65_chcnt ; i++) {
                log.print(" "); 
                log.print(i+1);
                log.print("=");  
                log.print(ap65_chan_raw[i]);   
              }                         
              log.print("  ap65_rssi=");  log.print(ap65_rssi); 
              log.print("  rssiGood=");  log.println(rssiGood);         
            #endif             
          break;   
             
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:     // #66 - UPLINK  TO UAV
          if (!mavGood) break;       
          break; 
          
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:       // #73   received back after #51 Mission_Request_Int sent
        #if defined Mav_Debug_All || defined Mav_Debug_Missions
          if (!mavGood) break; 
          ap_targsys =  mavlink_msg_mission_item_int_get_target_system(&R2Gmsg);   
          ap_targcomp =  mavlink_msg_mission_item_int_get_target_component(&R2Gmsg);   
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
 
          log.print("Mavlink from FC #73 Mission_Item_Int: ");
          log.print("target_system ="); log.print(ap_targsys);   
          log.print("  target_component ="); log.print(ap_targcomp);   
          log.print(" _seq ="); log.print(ap73_seq);   
          log.print("  frame ="); log.print(ap73_frame);     
          log.print("  command ="); log.print(ap73_command);   
          log.print("  current ="); log.print(ap73_current);   
          log.print("  autocontinue ="); log.print(ap73_autocontinue);   
          log.print("  param1 ="); log.print(ap73_param1, 2); 
          log.print("  param2 ="); log.print(ap73_param2, 2); 
          log.print("  param3 ="); log.print(ap73_param3, 2); 
          log.print("  param4 ="); log.print(ap73_param4, 2);                                          
          log.print("  x ="); log.print(ap73_x);   
          log.print("  y ="); log.print(ap73_y);   
          log.print("  z ="); log.print(ap73_z, 4);    
          log.print("  mission_type ="); log.println(ap73_mission_type);                                                    

          break; 
        #endif
                               
        case MAVLINK_MSG_ID_VFR_HUD:                 //  #74
          if (!mavGood) break;      
          ap74_air_spd = mavlink_msg_vfr_hud_get_airspeed(&R2Gmsg);
          ap74_grd_spd = mavlink_msg_vfr_hud_get_groundspeed(&R2Gmsg);      //  in m/s
          ap74_hdg = mavlink_msg_vfr_hud_get_heading(&R2Gmsg);              //  in degrees
          ap74_throt = mavlink_msg_vfr_hud_get_throttle(&R2Gmsg);           //  integer percent
          ap74_amsl = mavlink_msg_vfr_hud_get_alt(&R2Gmsg);                 //  m
          ap74_climb = mavlink_msg_vfr_hud_get_climb(&R2Gmsg);              //  m/s

          cur.hdg = ap74_hdg;
          
         #if defined Mav_Debug_All || defined Mav_Debug_Hud
            log.print("Mavlink from FC #74 VFR_HUD: ");
            log.print("Airspeed= "); log.print(ap74_air_spd, 2);                 // m/s    
            log.print("  Groundspeed= "); log.print(ap74_grd_spd, 2);            // m/s
            log.print("  Heading= ");  log.print(ap74_hdg);                      // deg
            log.print("  Throttle %= ");  log.print(ap74_throt);                 // %
            log.print("  Baro alt= "); log.print(ap74_amsl, 0);                  // m                  
            log.print("  Climb rate= "); log.println(ap74_climb);                // m/s
          #endif  
          
          #if (defined frBuiltin)
            FrPort.PushMessage(0x5005, 0);  // 0x5005 VelYaw
          #endif  

          #if (defined frBuiltin)
            #if defined PlusVersion
              FrPort.PushMessage(0x50F2, 0);  // 0x50F2 VFR HUD
            #endif
          #endif 
            
          break; 
          
        case MAVLINK_MSG_ID_COMMAND_ACK:       // #77   received back after #76 Command_Long

          if (!mavGood) break; 
          ap77_command =  mavlink_msg_command_ack_get_command(&R2Gmsg);   
          ap77_result =  mavlink_msg_command_ack_get_result(&R2Gmsg);    
          ap77_progress =  mavlink_msg_command_ack_get_progress(&R2Gmsg);   
          ap77_result_param2 =  mavlink_msg_command_ack_get_result_param2(&R2Gmsg); 
          ap77_target_system =  mavlink_msg_command_ack_get_target_system(&R2Gmsg);
          ap77_target_component =  mavlink_msg_command_ack_get_target_component(&R2Gmsg);  

          #if (defined frBuiltin)
            #if (defined Support_MavLite)
              FrPort.PushMessage(0x4d, 0);    // MavLite COMMAND_ACK ( #77 ) 
            #endif
          #endif
          
          #if defined Mav_Debug_All || defined Mav_Debug_Commands || defined Debug_Mavlite                
            log.print("Mavlink from FC #77 Command_Ack: ");
            log.print("command="); log.print(ap77_command);   
            log.print(" result="); log.print(ap77_result);   
            log.print(" progres="); log.print(ap77_progress);   
            log.print(" result_parm2="); log.print(ap77_result_param2);     
            log.print(" target_system="); log.print(ap77_target_system);   
            log.print(" target_component="); log.println(ap77_target_component);                                                    
            break; 
          #endif
          
          break;
          
        case MAVLINK_MSG_ID_RADIO_STATUS:         // #109
          if (!mavGood) break;

            ap109_rssi = mavlink_msg_radio_status_get_rssi(&R2Gmsg);            // air signal strength
            ap109_remrssi = mavlink_msg_radio_status_get_remrssi(&R2Gmsg);      // remote signal strength
            ap109_txbuf = mavlink_msg_radio_status_get_txbuf(&R2Gmsg);          // how full the tx buffer is as a percentage
            ap109_noise = mavlink_msg_radio_status_get_noise(&R2Gmsg);          // remote background noise level
            ap109_remnoise = mavlink_msg_radio_status_get_remnoise(&R2Gmsg);    // receive errors
            ap109_rxerrors = mavlink_msg_radio_status_get_rxerrors(&R2Gmsg);    // count of error corrected packets
            ap109_fixed = mavlink_msg_radio_status_get_fixed(&R2Gmsg);
            rssi109 = true;  
              
            // If we get #109 then it must be a SiK fw radio, so use this record for rssi
            rssiGood=true;   
            #ifdef QLRS 
              ap_rssi = ap109_remrssi;        // QRLS uses remote rssi - patch from giocomo892
            #else
              ap_rssi = ap109_rssi;        //  254 -> 100% (default) or percent (option)
            #endif          
            
            #if not defined Rssi_In_Percent
              ap_rssi /=  2.54;   //  254 -> 100%    // Patch from hasi123        
            #endif

            #if (defined frBuiltin)
              #if (not defined Rssi_Pacemaker)
                FrPort.PushMessage(0xF101, 0);   // 0xF101 RSSI 
                rssi_millis = millis();    
              #endif  
            #endif        
           
            #if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_RC
              #ifndef RSSI_Override
                log.print("Auto RSSI_Source===>  ");
              #endif
            #endif     

            #if defined Mav_Debug_All || defined Debug_Radio_Status || defined Mav_Debug_Rssi
              log.print("Mavlink from FC #109 Radio: "); 
              log.print("ap109_rssi="); log.print(ap109_rssi);
              log.print("  remrssi="); log.print(ap109_remrssi);
              log.print("  txbuf="); log.print(ap109_txbuf);
              log.print("  noise="); log.print(ap109_noise); 
              log.print("  remnoise="); log.print(ap109_remnoise);
              log.print("  rxerrors="); log.print(ap109_rxerrors);
              log.print("  fixed="); log.print(ap109_fixed);  
              log.print("  rssiGood=");  log.println(rssiGood);                                
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
              log.print("Mavlink from FC #125 Power Status: ");
              log.print("Vcc= "); log.print(ap_Vcc); 
              log.print("  Vservo= ");  log.print(ap_Vservo);       
              log.print("  flags= ");  log.println(ap_flags);       
            #endif  
          #endif              
            break; 

        case MAVLINK_MSG_ID_TERRAIN_REPORT:      // #136   https://mavlink.io/en/messages/common.html
          if (!mavGood) break;
          ap_terrain_spacing = mavlink_msg_terrain_report_get_spacing(&R2Gmsg);           // terrain grid spacing
          ap136_current_height = mavlink_msg_terrain_report_get_current_height(&R2Gmsg);  // height above terrain
          #if defined Mav_Debug_All || defined Mav_Debug_Terrain
            log.print("Mavlink from FC #136 Terrain Report: ");
            log.print("  spacing= "); log.print(ap_terrain_spacing);
            log.print("  current_height= ");  log.println(ap136_current_height);
          #endif

          #if (defined frBuiltin)
            FrPort.PushMessage(0x500B, 0);   // 0x500B Terrain status
          #endif
            break;

         case MAVLINK_MSG_ID_BATTERY_STATUS:      // #147   https://mavlink.io/en/messages/common.html
          if (!mavGood) break;       
          ap_battery_id = mavlink_msg_battery_status_get_id(&R2Gmsg);  
          ap_current_battery = mavlink_msg_battery_status_get_current_battery(&R2Gmsg);      // cA (10*milliamperes) (1 = 10 milliampere)
          ap_current_consumed = mavlink_msg_battery_status_get_current_consumed(&R2Gmsg);    // mAh
          ap147_battery_remaining = mavlink_msg_battery_status_get_battery_remaining(&R2Gmsg);  // (0%: 0, 100%: 100)  

          if (ap_battery_id == 0) {  // Battery 1
            pt_bat1_mAh = ap_current_consumed;                       
          } else if (ap_battery_id == 1) {  // Battery 2
              pt_bat2_mAh = ap_current_consumed;                              
          } 
             
          #if defined Mav_Debug_All || defined Debug_Batteries
            log.print("Mavlink from FC #147 Battery Status: ");
            log.print(" bat id= "); log.print(ap_battery_id); 
            log.print(" bat current mA= "); log.print(ap_current_battery*10); // now shows mA
            log.print(" ap_current_consumed mAh= ");  log.print(ap_current_consumed);   
            if (ap_battery_id == 0) {
              log.print(" my di/dt mAh= ");  
              log.println(Total_mAh1(), 0);  
            }
            else {
              log.print(" my di/dt mAh= ");  
              log.println(Total_mAh2(), 0);   
            }    
        //  log.print(" bat % remaining= ");  log.println(ap_time_remaining);       
          #endif                        
          
          break;    
        case MAVLINK_MSG_ID_SENSOR_OFFSETS:    // #150   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;        
          break; 
        case MAVLINK_MSG_ID_MEMINFO:           // #152   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;        
          break;   

        case MAVLINK_MSG_ID_FENCE_STATUS:      // #162   https://mavlink.io/en/messages/common.html
            if (!mavGood) break;
            ap_breach_status = mavlink_msg_fence_status_get_breach_status(&R2Gmsg);       // 0 inside, 1 outside
            ap_fence_last_update = millis();
            ap_fence_enabled = true;
            #if defined Mav_Debug_All || defined Mav_Debug_Fence
              log.print("Mavlink from FC #162 Fence Status: ");
              log.print("  fence_enabled= ");  log.print(ap_fence_enabled?"true":"false");
              log.print("  breach_status= ");  log.println(ap_breach_status);
            #endif
            break;

        case MAVLINK_MSG_ID_RADIO:             // #166   See #109 RADIO_status
        
          break; 
        case MAVLINK_MSG_ID_RANGEFINDER:       // #173   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;       
          ap_range = mavlink_msg_rangefinder_get_distance(&R2Gmsg);  // distance in meters

          #if defined Mav_Debug_All || defined Mav_Debug_Range
            log.print("Mavlink from FC #173 rangefinder: ");        
            log.print(" distance=");
            log.println(ap_range);   // now V
          #endif  

          #if (defined frBuiltin)
            FrPort.PushMessage(0x5006, 0);  // 0x5006 Rangefinder
          #endif  
   
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
            log.print("Mavlink from FC #181 Battery2: ");        
            log.print(" Bat volts=");
            log.print((float)ap_voltage_battery2 / 1000, 3);   // now V
            log.print("  Bat amps=");
            log.print((float)ap_current_battery2 / 100, 1);   // now A
              
            log.print("  mAh="); log.print(bat2.mAh, 6);    
            log.print("  Total mAh="); log.print(bat2.tot_mAh, 3);
         
            log.print("  Bat cell count= "); 
            log.println(ap_cell_count2);
          #endif
          
          #if (defined frBuiltin)
            FrPort.PushMessage(0x5008, 0);   // 0x5008 Bat2     
          #endif  
                    
          break;
          
        case MAVLINK_MSG_ID_AHRS3:            // #182   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;       
          break;

        case MAVLINK_MSG_ID_RPM:              // #226   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break; 
          ap_rpm1 = mavlink_msg_rpm_get_rpm1(&R2Gmsg); 
          ap_rpm2 = mavlink_msg_rpm_get_rpm2(&R2Gmsg);                    
          #if (defined Mav_Debug_RPM) || (defined Mav_Debug_All) 
            log.print("Mavlink from FC #226 RPM: ");
            log.print("RPM1= "); log.print(ap_rpm1, 0); 
            log.print("  RPM2= ");  log.println(ap_rpm2, 0);          
          #endif

          #if (defined frBuiltin)
            FrPort.PushMessage(0x500A, 0);   // 0x500A RPM
          #endif

          break;

        case MAVLINK_MSG_ID_HOME_POSITION:        // #242     
          ap242_latitude = mavlink_msg_home_position_get_latitude(&R2Gmsg);
          ap242_longitude = mavlink_msg_home_position_get_longitude(&R2Gmsg);
          ap242_alt_msl = mavlink_msg_home_position_get_altitude(&R2Gmsg);    
          ap242_homGood = true;                  
          hom.lat = ap242_latitude / 1E7;
          hom.lon = ap242_longitude / 1E7;
          hom.alt = ap242_alt_msl / 1E3;
          hom.hdg = cur.hdg;
          
          #if defined Mav_Debug_All || defined Mav_Debug_Home_Position
            log.print("Mavlink from FC #242 Home_Position:");
            log.print(" lat="); log.print((float)(ap242_latitude/1E7), 7); 
            log.print(" lon="); log.print((float)(ap242_longitude/1E7), 7);           
            log.print(" alt_msl="); log.print(ap242_alt_msl/1E3, 0);
            log.print(" cur.hdg="); log.println(cur.hdg, 0);              
          #endif

          break;                                      
          
        case MAVLINK_MSG_ID_STATUSTEXT:        // #253      
          ap_severity = mavlink_msg_statustext_get_severity(&R2Gmsg);
          len=mavlink_msg_statustext_get_text(&R2Gmsg, ap_text);

          #if defined Mav_Debug_All || defined Mav_Debug_StatusText
            log.print("Mavlink from FC #253 Statustext pushed onto MsgRingBuff: ");
            log.print(" Severity="); log.print(ap_severity);
            log.print(" "); log.print(MavSeverity(ap_severity));
            log.print("  Text= ");  log.print(" |"); log.print(ap_text); log.println("| ");
          #endif
          
          #if (defined frBuiltin)
            FrPort.PushMessage(0x5000, 0);         // 0x5000 StatusText Message
          #endif  
          
          break;                                      
        default:
          if (!mavGood) break;
          #if defined Mav_Debug_All || defined Mav_Show_Unknown_Msgs
            log.print("Mavlink from FC: ");
            log.print("Unknown Message ID #");
            log.print(R2Gmsg.msgid);
            log.println(" Ignored"); 
          #endif
          break;
      }
}

//================================================================================================= 
void MarkHome()  {
  
  homGood = true;
  hom.lat = cur.lat;
  hom.lon = cur.lon;
  hom.alt = cur.alt;
  hom.hdg = cur.hdg;
  
  log.println("First time motors armed, home established:");       
  log.print("hom.lat=");  log.print(hom.lat, 7);
  log.print(" hom.lon=");  log.print(hom.lon, 7 );        
  log.print(" hom.alt="); log.print(hom.alt, 0);
  log.print(" hom.hdg="); log.println(hom.hdg, 0);                   
}
//================================================================================================= 
void sendFcHeartbeat() {
  
  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;                             // FC
  apo_targcomp = 1;                            // FC                   

  apo_type = MAV_TYPE_GCS;                       // 6 Pretend to be a GCS
  apo_autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;   // 3 AP Mega
  apo_base_mode = 0;
  apo_system_status = MAV_STATE_ACTIVE;         // 4
   
  mavlink_msg_heartbeat_pack(apo_sysid, apo_compid, &G2Fmsg, apo_type, apo_autopilot, apo_base_mode, apo_system_status, 0); 
  sendToFC(0); 
  #if defined Debug_Our_FC_Heartbeat
     log.print("Our own heartbeat to FC: #0 Heartbeat: ");  
     log.print("apo_sysid="); log.print(apo_sysid);   
     log.print("  apo_compid="); log.print(apo_compid);  
     log.print("  apo_targsys="); log.print(apo_targsys);   
     log.print("  apo_targcomp="); log.print(apo_targcomp);                         
     log.print("  apo_type="); log.print(apo_type);   
     log.print("  apo_autopilot="); log.print(apo_autopilot); 
     log.print("  apo_base_mode="); log.print(apo_base_mode); 
     log.print("  apo_custom_mode="); log.print(apo_custom_mode);
     log.print("  apo_system_status="); log.print(apo_system_status);    
     log.println();
  #endif   
}
//================================================================================================= 
void mavParamRequestRead(int16_t param_index, char * param_id) {  // #20
  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;                             // APM FC
  apo_targcomp = 1;                            // APM FC
  
  mavlink_msg_param_request_read_pack(apo_sysid, apo_compid, &G2Fmsg,
                   apo_targsys, apo_targcomp, param_id, param_index);              
  sendToFC(20);             
 }

//================================================================================================= 
 void Request_Param_List() {

  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;                           // APM FC
  apo_targcomp = 1;                            // APM FC
  
  mavlink_msg_param_request_list_pack(apo_sysid,  apo_compid, &G2Fmsg,
                    apo_targsys,  apo_targcomp);
              
  sendToFC(21);
                    
 }
 //================================================================================================= 
 void mavParamSet() {    // #23

  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190 
  apo_targsys = 1;                             // APM FC
  apo_targcomp = 1;                            // APM FC
                                          
  mavlink_msg_param_set_pack(apo_sysid, apo_compid, &G2Fmsg,
                        apo_targsys, apo_targcomp, ap23_param_id, ap23_param_value, 10);  // 10=MAV_PARAM_TYPE_REAL64 https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE            
  sendToFC(23);  // #23  

  #if defined Debug_Mavlite
     log.printf("UPLINK: Mavlink sent #23 Param_Set. Parameter-id= %s  Value = %.5f\n", ap23_param_id, ap23_param_value);  
   #endif                
 }
//================================================================================================= 
#ifdef Request_Missions_From_FC
void RequestMission(uint16_t ms_seq) {         //  #40
  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;
  apo_targcomp = 1; 
  ap_mission_type = 0;   // Mav2  0 = Items are mission commands for main mission
  
  mavlink_msg_mission_request_pack(apo_sysid, apo_compid, &G2Fmsg,
                               apo_targsys, apo_targcomp, ms_seq, ap_mission_type);

  sendToFC(40);
  #if defined Mav_Debug_All || defined Mav_Debug_Missions
    log.print("Mavlink to FC #40 Request Mission:  ms_seq="); log.println(ms_seq);
  #endif  
}
#endif 
 
//================================================================================================= 
#if defined Request_Missions_From_FC || defined Request_Mission_Count_From_FC
void mavRequestMissionList() {   // #43   get back #44 Mission_Count
  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  ap_targsys = 1;                              // APM FC
  ap_targcomp = 1; 
  ap_mission_type = 0;   // Mav2  0 = Items are mission commands for main mission
  
  mavlink_msg_mission_request_list_pack(apo_sysid, apo_compid, &G2Fmsg,
                               apo_targsys, apo_targcomp, ap_mission_type);
                             
  sendToFC(43);
  #if defined Mav_Debug_All || defined Mav_Debug_Missions
    log.println("Mavlink to FC #43 Request Mission List (count)");
  #endif  
}
#endif
//================================================================================================= 
#ifdef Request_Missions_From_FC
void RequestAllWaypoints(uint16_t ms_count) {
  for (int i = 0; i < ms_count; i++) {  //  Mission count = next empty WP, i.e. one too high
    RequestMission(i); 
  }
}
#endif
//================================================================================================= 
#ifdef Data_Streams_Enabled    
void requestDataStreams() {    //  REQUEST_DATA_STREAM ( #66 ) DEPRECATED. USE SRx, SET_MESSAGE_INTERVAL INSTEAD

  apo_sysid = 255;              // Reply to APM FC
  apo_compid = 190;             // Reply to APM FC
  apo_targsys = 1;              // FC
  apo_targcomp = 1;             // FC

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

  const uint16_t mavRates[] = { 0x02, 0x03, 0x02, 0x03, 0x06, 0x02, 0x02};
 // req_message_rate The requested interval between two messages of this type

  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(apo_sysid, apo_compid, &G2Fmsg,
        apo_targsys, apo_targcomp, mavStreams[i], mavRates[i], 1);    // start_stop 1 to start sending, 0 to stop sending   
                          
  sendToFC(66);
    }
 // log.println("Mavlink to FC #66 Request Data Streams:");
}
#endif
//================================================================================================= 
void Send_Mavlink_Command_Long() {  // #76

  apo_sysid = Device_sysid;                    // Reply to this device. From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;                             // FC
  apo_targcomp = 1;                            // FC
  
  mavlink_msg_command_long_pack(apo_sysid, apo_compid, &G2Fmsg,
          apo_targsys, apo_targcomp, apo76_command, apo76_confirmation, apo76_param[0], apo76_param[1], 
          apo76_param[2], apo76_param[3], apo76_param[4], apo76_param[5], apo76_param[6]); 
     
  sendToFC(76);  // #76                
 }
//================================================================================================= 
void Mavlink_Request_Home_Position() {  // #410  https://mavlink.io/en/messages/common.html#MAV_CMD
  apo76_command = 410;
  apo76_param[0] = 0;  
  apo76_param[1] = 0;  
  apo76_param[2] = 0;  
  apo76_param[3] = 0;  
  apo76_param[4] = 0;  
  apo76_param[5] = 0;  
  apo76_param[6] = 0;  
  Send_Mavlink_Command_Long();    
 } 
 //================================================================================================= 
 
