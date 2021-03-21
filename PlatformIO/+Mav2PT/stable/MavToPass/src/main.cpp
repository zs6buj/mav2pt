#include <Arduino.h>

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

    Many thanks to yaapu for advice and testing, and his excellent LUA script

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

    The popular LUA telemetry scripts use RSSI to determine that a telemetry connection has been successfully established 
    between the 'craft and the Taranis/Horus. Be sure to set-up RSSI properly before testing the system.


    =====================================================================================================================


   Connections to ESP32 or ESP8266 boards depend on the board variant

    Go to config.h tab and look for "S E L E C T   E S P   B O A R D   V A R I A N T" 

    
   Connections to Teensy3.2 are:
    0) USB                         Flashing and serial monitor for debug
    1) F/SPort     -->tx1 Pin 1    F/SPort out to XSR  or Taranis bay, bottom pin
    2) Mavlink_In  <--rx2 Pin 9    Mavlink source to Teensy - mav_rxPin
    3) Mavlink_In  -->tx2 Pin 10   Mavlink source to Taranis
    4) Mavlink_Out <--rx3 Pin 7    Optional feature - see #defined
    5) Mavlink_Out -->tx3 Pin 8    Optional feature - see #defined
    6) MavStatusLed       Pin 13   BoardLed
    7) BufStatusLed  1
    8) Vcc 3.3V !
    9) GND

    NOTE : STM32 support is deprecated as of 2020-02-27 v2.56.2
*/
//    =====================================================================================================================
  
#include <CircularBuffer.h>

#include <mavlink_types.h>
#include "global_variables.h"
#include "config.h"                      // ESP_IDF libs included here

#if (defined frBuiltin)
  #include "FrSky_Ports.h"
#endif

#if defined TEENSY3X || defined ESP8266  // Teensy 3.x && ESP8266 
  #undef F   // F defined as m->counter[5]  in c_library_v2\mavlink_sha256.h
             // Macro F()defined in Teensy3/WString.h && ESP8266WebServer-impl.h (forces string literal into prog mem)   
#endif

#include <ardupilotmega/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>



//=================================================================================================   
//                     F O R W A R D    D E C L A R A T I O N S
//=================================================================================================

void main_loop();
void ServiceWiFiRoutines();
void ServiceInboundTCPClients();
void CheckStaLinkStatus(); 
void StartWiFiTimer();
void RestartWiFiSta();
void Start_Access_Point();
bool NewOutboundTCPClient();
bool Read_FC_To_RingBuffer();
void RB_To_Decode_and_GCS();
void Read_From_GCS();
void Decode_GCS_To_FC();
void Send_To_FC(uint32_t);
void Send_FC_Heartbeat();
void Mavlink_Param_Request_Read(int16_t, char *);
void Mavlink_Request_Mission_List();
void Mavlink_Param_Set();
void Mavlink_Command_Long();
void ServiceStatusLeds();
void MavToRingBuffer();
void Send_From_RingBuf_To_GCS();
void checkLinkErrors(mavlink_message_t*); 
bool Read_Bluetooth(mavlink_message_t*);
bool Send_Bluetooth(mavlink_message_t*);
bool Read_TCP(mavlink_message_t*);
bool Read_UDP(io_side_t, mavlink_message_t*);
bool Send_TCP(mavlink_message_t*);
bool Send_UDP(io_side_t, mavlink_message_t*);
void DecodeOneMavFrame();
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

void PrintRemoteIP();
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
  void draw_home_arrow(int16_t, int16_t, int16_t, int16_t, int16_t);  
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
 
  Log.begin(115200);
  delay(2500);
  Log.println();
  pgm_path = __FILE__;  // ESP8266 __FILE__ macro returns pgm_name and no path
  pgm_name = pgm_path.substring(pgm_path.lastIndexOf("\\")+1);  
  pgm_name = pgm_name.substring(0, pgm_name.lastIndexOf('.'));  // remove the extension
  Log.print("Starting "); Log.print(pgm_name);
  Log.printf(" version:%d.%02d.%02d\n", MAJOR_VERSION,  MINOR_VERSION, PATCH_LEVEL);
    
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_WiFi)
   WiFi.onEvent(WiFiEventHandler);   
  #endif  
 
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("Free Heap just after startup = %d\n", ESP.getFreeHeap());
  #endif  
//=================================================================================================   
//                                   S E T U P   D I S P L A Y
//=================================================================================================
  #if (defined displaySupport) 

    #if (defined ESP32)
    
      if (Tinfo != 99)  {   // enable info touch pin-pair
        touchAttachInterrupt(digitalPinToInterrupt(Tinfo), gotButtonInfo, threshold); 
       } else
      if (Pinfo != 99)  {   // enable info digital pin
        pinMode(Pinfo, INPUT_PULLUP);   
      }  
       
      if ( (Tup != 99) && (Tdn != 99) ) {   // enable touch pin-pair
        touchAttachInterrupt(digitalPinToInterrupt(Tup), gotButtonUp, threshold);
        touchAttachInterrupt(digitalPinToInterrupt(Tdn), gotButtonDn, threshold);   
      } else
      if ( (Pup != 99) && (Pdn != 99) ) {   // enable digital pin-pair
        pinMode(Pup, INPUT_PULLUP);
        pinMode(Pdn, INPUT_PULLUP);          
      }

    #endif  

    #if ((defined ESP8266) || (defined TEENSY3X))         
      if ( (Pup != 99) && (Pdn != 99) ) { // enable digital pin pair
        pinMode(Pup, INPUT_PULLUP);
        pinMode(Pdn, INPUT_PULLUP);
      }

    #endif 

    #if (defined ST7789_Display)               // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD (135 x 240)
      display.init(); 
      #define SCR_BACKGROUND TFT_BLACK
      
    #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display (128 x 64)
      #if not defined TEENSY3X                 // Teensy uses default SCA and SCL in teensy "pins_arduino.h"
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
        Log.print("Display Setup: Portrait ");         
    #elif (SCR_ORIENT == 1)            // landscape   
        Log.println("Display support activated: Landscape "); 
    #endif

    int eol= 0;
    for (eol = 0 ; eol < SCR_W_CH ; eol++) {
      clear_line[eol] = ' ';
    }
    clear_line[eol] = 0x00;

    Log.printf("%dx%d  TEXT_SIZE=%d  CHAR_W_PX=%d  CHAR_H_PX=%d  SCR_H_CH=%d  SCR_W_CH=%d\n", SCR_H_PX, SCR_W_PX, TEXT_SIZE, CHAR_W_PX, CHAR_H_PX, SCR_H_CH, SCR_W_CH);
    
    LogScreenPrintln("Starting .... ");
  #endif
  /*
  display.setFont(Dialog_plain_8);     //  col=24 x row 8  on 128x64 display
  display.setFont(Dialog_plain_16);    //  col=13 x row=4  on 128x64 display
  */
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap after OLED setup = %d\n", ESP.getFreeHeap());
  #endif
  
//=================================================================================================   
//                             S E T U P   E E P R O M
//=================================================================================================

  #if defined ESP32    
    if (!EEPROM.begin(EEPROM_SIZE))  {
      Log.println("Fatal error!  EEPROM failed to initialise.");
      LogScreenPrintln("EEPROM fatal error!");
      while (true) delay(100);  // wait here forever 
     } else {
      Log.println("EEPROM initialised successfully");
      LogScreenPrintln("EEPROM good"); 
     }
  #endif       
    
  #if defined ESP8266
    EEPROM.begin(EEPROM_SIZE);
    Log.println("EEPROM initialised successfully");
    LogScreenPrintln("EEPROM good"); 
  #endif

  RawSettingsToStruct();      // So that we can use them regardless of webSupport
  
  #if (defined webSupport) 
    RecoverSettingsFromFlash(); 
  #endif

  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap after EEPROM setup = %d\n", ESP.getFreeHeap());
  #endif

  // =============================================
  
  #if (defined frBuiltin)  // FrSky Port support

    if (set.trmode == ground) {
     Log.println("Ground Mode selected");
      LogScreenPrintln("Ground Mode");
   } else  
   if (set.trmode == air) {
      Log.println("Air Mode selected");
      LogScreenPrintln("Air Mode");
   } else
   if (set.trmode == relay) {
      Log.println("Relay Mode selected");
     LogScreenPrintln("Relay Mode");
    }

    #if (Battery_mAh_Source == 1)  
      Log.println("Battery_mAh_Source = 1 - Get battery capacities from the FC");
     LogScreenPrintln("mAh from FC");
    #elif (Battery_mAh_Source == 2)
     Log.println("Battery_mAh_Source = 2 - Define battery capacities in this firmware");  
     LogScreenPrintln("mAh defined in fw");
    #elif (Battery_mAh_Source == 3)
     Log.println("Battery_mAh_Source = 3 - Define battery capacities in the LUA script");
     LogScreenPrintln("Setup mAh in LUA");     
    #else
     #error You must define at least one Battery_mAh_Source. Please correct.
    #endif            

    #ifndef RSSI_Override
      Log.println("RSSI Automatic Select");
      LogScreenPrintln("RSSI Auto Select");     
    #else
     Log.printf("RSSI Override for testing = %d%%\n", RSSI_Override);
      snprintf(snprintf_buf, snp_max, "RSSI Override %d%%", RSSI_Override);
      LogScreenPrintln(snprintf_buf);      
    #endif

  #else
      Log.println("No FrSky port support selected or built in");
      LogScreenPrintln("No FrSky support");
  #endif
  
  Log.print("Target Board is ");
  #if (defined TEENSY3X) // Teensy3x
    Log.println("Teensy 3.x");
    LogScreenPrintln("Teensy 3.x");
  #elif (defined ESP32) //  ESP32 Board
    Log.print("ESP32 / Variant is ");
    LogScreenPrintln("ESP32 Variant is");
    #if (ESP32_Variant == 1)
      Log.println("Dev Module");
      LogScreenPrintln("Dev Module");
    #endif
    #if (ESP32_Variant == 2)
      Log.println("Wemos® LOLIN ESP32-WROOM-32");
      LogScreenPrintln("Wemos® LOLIN");
    #endif
    #if (ESP32_Variant == 3)
      Log.println("Dragonlink V3 slim with internal ESP32");
      LogScreenPrintln("Dragonlink V3 ESP32");
    #endif
    #if (ESP32_Variant == 4)
      Log.println("Heltec Wifi Kit 32");
      LogScreenPrintln("Heltec Wifi Kit 32");
    #endif
    #if (ESP32_Variant == 5)
      Log.println("LILYGO® TTGO T-Display ESP32 1.14 inch ST7789 Colour LCD");
      LogScreenPrintln("TTGO T-Display ESP32");
    #endif    
    #if (ESP32_Variant == 6)
      Log.println("LILYGO® TTGO T2 ESP32 OLED SD");
      LogScreenPrintln("TTGO T2 ESP32 SD");
    #endif 
    #if (ESP32_Variant == 7)
      Log.println("Dev Module with ILI9341 2.8in COLOUR TFT SPI");
      LogScreenPrintln("Dev module + TFT");
    #endif                   
  #elif (defined ESP8266) 
    Log.print("ESP8266 / Variant is ");
    LogScreenPrintln("ESP8266 / Variant is");  
    #if (ESP8266_Variant == 1)
      Log.println("Lonlin Node MCU 12F");
      LogScreenPrintln("Node MCU 12");
    #endif 
    #if (ESP8266_Variant == 2)
      Log.println("ESP-F - RFD900X TX-MOD");
      LogScreenPrintln("RFD900X TX-MOD");
    #endif   
    #if (ESP8266_Variant == 3)
      Log.println("Wemos® ESP-12F D1 Mini");
      LogScreenPrintln("Wemos ESP-12F D1 Mini");
    #endif       
  #endif

  if (set.fc_io == fc_ser)   {
    Log.println("Mavlink Serial In");
    LogScreenPrintln("Mav Serial In");
  }

  if (set.fc_io == fc_bt)  {
    Log.println("Mavlink Bluetooth In");
    LogScreenPrintln("Mav BT In");
  } 

  if (set.fc_io == fc_wifi)  {
    Log.println("Mavlink WiFi In");
    LogScreenPrintln("Mav WiFi In");
  } 

  if (set.fr_io == fr_ser)     {         // 1
    Log.println("FrSky Serial Out");
    LogScreenPrintln("Frs Serial Out");
  } else
  if (set.fr_io == fr_udp)  {           // 2
    Log.println("FrSky UDP Out");
    LogScreenPrintln("Frs UDP Out");
  } else   
  if (set.fr_io == fr_ser_udp)    {     // 3
    Log.println("FrSky Serial and UDP Out");
    LogScreenPrintln("Frs Serial+UDP Out");
  } else   
  if (set.fr_io == fr_udp_sd)    {      // 4
    Log.println("FrSky UDP and SD Out");
    LogScreenPrintln("Frs UDP+SD Out");
  } else    
  if (set.fr_io == fr_ser_sd)    {      // 5
    Log.println("FrSky Serial and SD Out");
    LogScreenPrintln("Frs Serial+SD Out");
  } else  
  if (set.fr_io == fr_udp_sd)    {      // 6
    Log.println("FrSky UDP and SD Out");
    LogScreenPrintln("Frs UDP+SD Out");
  } else    
  if (set.fr_io == fr_ser_udp_sd)     { // 7
    Log.println("FrSky Serial and UDP and SD Out");
    LogScreenPrintln("Frs Ser+UDP+SD Out");
  }
 
  if (set.gs_io == gs_ser)  {
    Log.println("Mavlink Serial Out");
    LogScreenPrintln("Mav Serial Out");
  }

  if (set.gs_io == gs_bt)  {
    Log.println("Mavlink Bluetooth Out");
    LogScreenPrintln("Mav BT Out");
  }

  if (set.gs_io == gs_wifi)  {
    Log.print("Mavlink WiFi Out - ");
    LogScreenPrintln("Mav WiFi Out");
  }

  if (set.gs_io == gs_wifi_bt)  {
    Log.print("Mavlink WiFi+BT Out - ");
    LogScreenPrintln("Mav WiFi+BT Out");
  }

  if ((set.fc_io == fc_wifi) || (set.gs_io == gs_wifi) ||  (set.gs_io == gs_wifi_bt) || (set.web_support)) {
   if (set.wfmode == ap)  {
     Log.println("WiFi mode is AP");
     LogScreenPrintln("WiFi-mode is AP");
   } else
   if (set.wfmode == sta)  {
     Log.println("WiFi mode is STA");
     LogScreenPrintln("WiFi-mode is STA");
   } else
   if (set.wfmode == sta_ap)  {
     Log.println("WiFi mode is STA>AP");
     LogScreenPrintln("WiFi-mode=STA>AP");
   } else
   if (set.wfmode == ap_sta)  {
     Log.println("WiFi mode is AP_STA");
     LogScreenPrintln("WiFi-mode=AP_STA");
   }
    
   if (set.mav_wfproto == tcp)  {
     Log.println("Protocol is TCP/IP");
     LogScreenPrintln("Protocol=TCP/IP");
   }
   else if  (set.mav_wfproto == udp) {
     Log.print("Protocol is UDP");
     LogScreenPrintln("Protocol = UDP");
     #if defined UDP_Broadcast
       Log.println(" Broadcast");
       LogScreenPrintln(" Broadcast");
     #else
       Log.println(" IP-Targeted");     
       LogScreenPrintln("IP-Targeted");
     #endif
   }
  }
  #if defined sdBuiltin                    
    if (set.fc_io == fc_sd) {
      Log.println("Mavlink SD In");
      LogScreenPrintln("Mavlink SD In");
   }

    if (set.gs_sd == gs_on) {
      Log.println("Mavlink SD Out");
      LogScreenPrintln("Mavlink SD Out");
    }
    
    if (set.sport_sd == spsd_on) {
      Log.println("SPort To SD Out");
      LogScreenPrintln("SPort To SD Out");
    }
    
  #endif

  #if (defined Support_MavLite)
    Log.println("MavLite supported");
    LogScreenPrintln("MavLite supported");
    set.Support_MavLite = true;
  #else
    set.Support_MavLite = false;
  #endif
  
 
//=================================================================================================   
//                                S E T U P   W I F I  --  E S P only
//=================================================================================================

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
    Log.println("No WiFi options selected, WiFi support not compiled in");
  #endif

//=================================================================================================   
//                                   S E T U P   B L U E T O O T H
//=================================================================================================

// Slave advertises hostname for pairing, master connects to slavename

  #if (defined btBuiltin)
    if ((set.fc_io == fc_bt) || (set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt)) { 
      if (set.btmode == 1)   {               // master
        Log.printf("Bluetooth master mode host %s is trying to connect to slave %s. This can take up to 30s\n", set.host, set.btConnectToSlave); 
        LogScreenPrintln("BT connecting ......");   
        SerialBT.begin(set.host, true);      
        // Connect(address) is relatively fast (10 secs max), connect(name) is slow (30 secs max) as it 
        // needs to resolve the name to an address first.
        // Set CoreDebugLevel to Info to view devices bluetooth addresses and device names
        bool bt_connected;
        bt_connected = SerialBT.connect(set.btConnectToSlave);
        if(bt_connected) {
          Log.println("Bluetooth done");  
          LogScreenPrintln("BT done");
        }          
      } else {                               // slave, passive                           
        SerialBT.begin(set.host);        
        Log.printf("Bluetooth slave mode, host name for pairing is %s\n", set.host);               
      } 
      btActive = true;    
    }
  #else
    Log.println("No Bluetooth options selected, BT support not compiled in");
    btActive = false;
  #endif
  
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap after Bluetooth setup = %d\n", ESP.getFreeHeap());
  #endif
 //=================================================================================================   
 //                                  S E T U P   S D   C A R D  -  E S P 3 2  O N L Y  for now
 //=================================================================================================
  #if ((defined ESP32) || (defined ESP8266)) && (defined sdBuiltin)
  
    void listDir(fs::FS &fs, const char *, uint8_t);  // Fwd declare
    
    if(!SD.begin()){   
        Log.println("No SD card reader found. Ignoring SD!"); 
        LogScreenPrintln("No SD reader");
        LogScreenPrintln("Ignoring!");
        sdStatus = 0; // 0=no reader, 1=reader found, 2=SD found, 3=open for append 
                      // 4=open for read, 5=eof detected, 9=failed
    } else {
      Log.println("SD card reader mount OK");
      LogScreenPrintln("SD drv mount OK");
      uint8_t cardType = SD.cardType();
      sdStatus = 1;
      if(cardType == CARD_NONE){
          Log.println("No SD card found");
          LogScreenPrintln("No SD card");
          LogScreenPrintln("Ignoring!");      
      } else {
        Log.println("SD card found");
        LogScreenPrintln("SD card found");
        sdStatus = 2;

   //     Log.printf("Total space: %lluMB\n", SD.totalbytes() / (1024 * 1024));
   //     Log.printf("Used space: %lluMB\n", SD.usedbytes() / (1024 * 1024));

        listDir(SD, "/", 2);

        if (set.fc_io == fc_sd)  {   //  FC side SD in only   
          std::string S = "";  
          char c = 0x00;
           Log.println("Enter the number of the SD file to read, and press Send");
           while (c != 0xA) { // line feed
            if (Log.available())  {
              c = Log.read();
              S+=c;
             }
             delay(50);
           }
           
           int i;
           // object from the class stringstream 
           std::istringstream myInt(S); 
           myInt >> i;
           Log.print(i); Log.print(" ");
           /*
           for (int j= 0 ; fnCnt > j ; j++)  {
           //   cout << i << fnPath[j] << "\n";
             Log.print(j); Log.print(" "); Log.println(fnPath[j].c_str());
            }
           */     

           sprintf(cPath, "%s", fnPath[i].c_str());  // Select the path
           Log.print(cPath); Log.println(" selected "); 
           Log.println("Reading SD card");
           LogScreenPrintln("Reading SD card");
           file = SD.open(cPath);
           if(!file){
             Log.printf("Can't open file: %s\n", cPath);
             Log.println(" for reading");
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
    Log.printf("==============>Free Heap after SD Card setup = %d\n", ESP.getFreeHeap());
  #endif
   
#endif
//=================================================================================================   
//                                    S E T U P   S E R I A L
//=================================================================================================  

  if ((set.fc_io == fc_ser) || (set.gs_io == gs_ser))  {  //  Serial
    #if defined MavAutoBaud
      set.baud = FrPort.getBaud(mav_rxPin, idle_high); // mavlink port is a regular non-inverted port
      Log.printf("Mavlink baud detected at %d b/s on rx:%d\n", set.baud, mav_rxPin);  
      String s_baud=String(set.baud);   // integer to string. "String" overloaded
      LogScreenPrintln("Mav baud:"+ s_baud);        
    #endif   
    #if (defined ESP32)   
      delay(100);
      // system can wait here for a few seconds (timeout) if there is no telemetry in
      mvSerial.begin(set.baud, SERIAL_8N1, mav_rxPin, mav_txPin);   //  rx,tx, cts, rts  
      delay(10);
    #else
      mvSerial.begin(set.baud);    
    #endif 
    Log.printf("Mavlink serial on pins rx:%d and tx:%d  baud:%d\n", mav_rxPin, mav_txPin, set.baud); 
  }
  #if (defined frBuiltin)       
    FrPort.initialise();
  #endif
  
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap after Serial UART setup = %d\n", ESP.getFreeHeap());
  #endif

//=================================================================================================   
//                                    S E T U P   O T H E R
//=================================================================================================   
  mavGood = false;
  homGood = false;     
  hb_count = 0;
  hb_millis=millis();
  uplink_millis = millis();       // mavlink uplink to gcs timimg
  fchb_millis = millis();
  gshb_millis = millis();
  acc_millis = millis();
  rds_millis = millis();
  blind_inject_millis = millis();
  health_millis = millis();

  pinMode(MavStatusLed, OUTPUT); 
  if (BufStatusLed != 99) {
    pinMode(BufStatusLed, OUTPUT); 
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
      Log.printf("Free Heap now = %d\n", ESP.getFreeHeap());
    }
  #endif

 //====================
 
 #if ((defined ESP32) || (defined ESP8266)) 
   ServiceWiFiRoutines();
 #endif

 //====================
  
  #if defined Debug_Loop_Period
    PrintLoopPeriod();
  #endif

  //==================== F l i g h t   C o m p u t e r   T o   R i n g   B u f f e r
  
  if (!Read_FC_To_RingBuffer()) {  //  check for SD eof
    if (sdStatus == 5) {
      Log.println("End of SD file");
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
   
 // if (millis() - uplink_millis > 1) {   // main timing loop for mavlink decode and to GCS
    RB_To_Decode_and_GCS();
 // }
  
 //====================     H a n d l e   F r S k y   P o r t   T r a f f i c
 
  #if (defined frBuiltin)
    FrPort.HandleTraffic(); 
  #endif  

  //====================   R e a d   F r o m   G C S
  
  Read_From_GCS();  

  if (GCS_available) {
    Decode_GCS_To_FC();
    Send_To_FC(G2Fmsg.msgid);  
    GCS_available = false;                      
   }

  //==================== Check For Heartbeat from GCS Timeout
  
  if(gshbGood && (millis() - gshb_millis) > 10000){ // if no heartbeat from GCS in n seconds then assume GCS not connected
    gshbGood = false;
  //  Log.println("Heartbeat from GCS timed out! GCS not connected"); 
  //  LogScreenPrintln("GCS Heartbeat lost!"); 
    
  }

  //==================== Send Our Own Heartbeat to FC Only When No GCS Heartbeat (this to trigger tardy telemetry)

  if (!gshbGood) {
    if(millis()- fchb_millis > 2000) {  // MavToPass heartbeat to FC every 2 seconds
      fchb_millis=millis();       
      Send_FC_Heartbeat();              // must have MavToPass tx pin connected to Telem radio rx pin  
    }
  }

  //====================            Check For FC Heartbeat Timeout
  
  if(mavGood && (millis() - hb_millis) > 15000)  {   // if no heartbeat from APM in 15s then assume FC mav not connected
    mavGood=false;
    Log.println("Heartbeat from FC timed out! FC not connected"); 
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
  if(mavGood) {                      // If we have a link, request data streams from MavLink every 5s
    if(millis()-rds_millis > 5000) {
    rds_millis=millis();
    Log.println("Requesting data streams"); 
    LogScreenPrintln("Reqstg datastreams");    
    RequestDataStreams();   // must have Teensy Tx connected to Taranis/FC rx  (When SRx not enumerated)
    }
  }
  #endif 
  
  //==================== Download Missions Option
  
  #if defined Request_Missions_From_FC || defined Request_Mission_Count_From_FC
  if (mavGood) {
    if (!ap_ms_list_req) {
      Mavlink_Request_Mission_List();  //  #43
      ap_ms_list_req = true;
    }
  }
  #endif
  
  //==================== Get battery capacity from the FC Option
  
  #if (Battery_mAh_Source == 1)  
  // Request battery capacity params 
  if (mavGood) {
    if (!ap_bat_paramsReq) {
      Mavlink_Param_Request_Read(356);    // Request Bat1 capacity   do this twice in case of lost frame
      Mavlink_Param_Request_Read(356);    
      Mavlink_Param_Request_Read(364);    // Request Bat2 capacity
      Mavlink_Param_Request_Read(364);    
      Log.println("Battery capacities requested");
      LogScreenPrintln("Bat mAh from FC");    
      ap_bat_paramsReq = true;
    } else {
      if (ap_bat_paramsRead &&  (!parm_msg_shown)) {
        parm_msg_shown = true; 
        Log.println("Battery params successfully read"); 
        LogScreenPrintln("Bat params read ok"); 
      }
    } 
  }
  #endif 
  
  //====================   List All Parameters From The FC Option
  
  #ifdef Mav_List_Params
    if(mavGood && (!ap_paramsList)) {
      Request_Param_List();
      ap_paramsList = true;
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

bool Read_FC_To_RingBuffer() {

  if (set.fc_io == fc_ser)  {  // Serial
    mavlink_status_t status;

    while(mvSerial.available()) { 
      byte c = mvSerial.read();
     // Printbyte(c, 1, '<');
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &F2Rmsg, &status)) {  // Read a frame
         #ifdef  Debug_FC_Down
           Log.println("Serial passed to RB from FC side :");
           PrintMavBuffer(&F2Rmsg);
        #endif              
        MavToRingBuffer();    
      }
    }
    return true;  
  } 

  #if (defined btBuiltin) 
    if (set.fc_io == fc_bt)  {  // Bluetooth

     bool msgReceived = Read_Bluetooth(&F2Rmsg);

     if (msgReceived) {

        MavToRingBuffer();      

        #ifdef  Debug_FC_Down   
          Log.print("Read from FC Bluetooth to Ringbuffer: msgReceived=" ); Log.println(msgReceived);
          PrintMavBuffer(&F2Rmsg);
        #endif      
      }
    return true;  
    }   
  #endif

  #if (defined wifiBuiltin)
    if (set.fc_io == fc_wifi)  {  //  WiFi
      
      if ((set.mav_wfproto == tcp) && (outbound_clientGood))  { // TCP  from FC side
 
        bool msgReceived = Read_TCP(&F2Rmsg);
        if (msgReceived) {
        
          MavToRingBuffer();  
          
          #ifdef  Debug_FC_Down    
            Log.print("Read from FC WiFi TCP to Ringbuffer: msgReceived=" ); Log.println(msgReceived);
            PrintMavBuffer(&F2Rmsg);
          #endif      
        }
       return true;  
      }
      
      if (set.mav_wfproto == udp)  {    // UDP from FC

        if ((set.wfmode == ap_sta) || (set.wfmode == sta)) {// if AP_STA or STA mode                 
          active_object_idx = 0;                            // Use STA UDP object for FC read        
          udp_read_port = set.udp_remotePort;               // used by PrintRemoteIP() only. read port set by UDP.begin().
          udp_send_port = set.udp_localPort;                   
        } else {
          if (set.wfmode == ap) {
            active_object_idx = 1;                          // Use AP UDP object for FC read       
            udp_read_port = set.udp_localPort;  
            udp_send_port = set.udp_remotePort;                     
          }                 
        }     
                  
        bool msgReceived = Read_UDP(fc_side, &F2Rmsg);              // FC side
        if (msgReceived) {

          #if defined  Debug_Read_UDP || defined Debug_Read_UDP_FC  
            Log.print("Read WiFi UDP from FC to G2Fmsg: msgReceived=" ); Log.println(msgReceived);
          #endif
          
          MavToRingBuffer();   
         
          #if defined Debug_FC_Down || defined Debug_Read_UDP_FC 
            Log.print("Read from FC WiFi UDP to Ringbuffer: msgReceived=" ); Log.println(msgReceived);
            PrintMavBuffer(&F2Rmsg);
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
              Log.println("Read from FC SD to Ringbuffer:");
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
    }
  #endif 
  return false; 
}
//================================================================================================= 

void RB_To_Decode_and_GCS() {

  if (!MavRingBuff.isEmpty()) {
    R2Gmsg = (MavRingBuff.shift());  // Get a mavlink message from front of queue
    #if defined Mav_Debug_RingBuff
      //Log.print("Mavlink ring buffer R2Gmsg: ");  
      //PrintMavBuffer(&R2Gmsg);
      Log.print("Ring queue = "); Log.println(MavRingBuff.size());
    #endif
    
    Send_From_RingBuf_To_GCS();
    
    DecodeOneMavFrame();  // Decode a Mavlink frame and buffer passthru

  }
}  

//================================================================================================= 
void Read_From_GCS() {

    if (set.gs_io == gs_ser)  {  // Serial 
      mavlink_status_t status;
      while(mvSerial.available()) { 
        uint8_t c = mvSerial.read();
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &G2Fmsg, &status)) {  // Read a frame from GCS  
          GCS_available = true;  // Record waiting
          #ifdef  Debug_GCS_Up
            Log.println("Passed up from GCS Serial to G2Fmsg:");
            PrintMavBuffer(&G2Fmsg);
          #endif     
        }
      } 
     } 

  #if (defined btBuiltin) 
    if ((set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt)) {  // Bluetooth
 
       bool msgReceived = Read_Bluetooth(&G2Fmsg);

       if (msgReceived) {
          GCS_available = true;  // Record waiting to go to FC 
          #ifdef  Debug_GCS_Up    
            Log.print("Passed up from GCS BT to G2Fmsg: msgReceived=" ); Log.println(msgReceived);
            if (msgReceived) PrintMavBuffer(&G2Fmsg);
          #endif      
        }
    }  
  #endif

  #if (defined wifiBuiltin)
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) ) {   //  WiFi
    
      if ((set.mav_wfproto == tcp) && (inbound_clientGood))  { // TCP from GCS side    
    
        bool msgReceived = Read_TCP(&G2Fmsg);

        if (msgReceived) {
          GCS_available = true;  // Record waiting to go to FC 

          #ifdef  Debug_Read_TCP 
            Log.print("Read WiFi TCP to G2Fmsg: msgReceived=" ); Log.println(msgReceived);
            if (msgReceived) PrintMavBuffer(&G2Fmsg);
          #endif      
        }
      }
      
      if (set.mav_wfproto == udp)  { // UDP from GCS
  
        if ((set.wfmode == ap_sta) || (set.wfmode == ap)) { // if AP_STA mode                 
          active_object_idx = 1;                            // Use AP UDP object for GCS read        
          udp_read_port = set.udp_localPort;               // used by PrintRemoteIP() only. read port set by UDP.begin().
          udp_send_port = set.udp_remotePort;                   
        } else {
          if (set.wfmode == sta) {
            active_object_idx = 0;                          // Use STA UDP object for GCS read  
            udp_read_port = set.udp_localPort;  
            udp_send_port = set.udp_remotePort;                          
          }               
        }  
            
        bool msgReceived = Read_UDP(gcs_side, &G2Fmsg);               // GCS side

        if (msgReceived) {
          GCS_available = true;  // Record waiting to go to FC 

          #if defined  Debug_Read_UDP || defined Debug_Read_UDP_GCS  
            Log.printf("Read WiFi UDP from GCS to G2Fmsg: msgReceived=%d ==============================\n", msgReceived); 
            PrintMavBuffer(&G2Fmsg);
          #endif      
        }   
      } 
    }
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
                          checkLinkErrors(msgptr);
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
  bool Read_TCP(mavlink_message_t* msgptr)  {
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
                          checkLinkErrors(msgptr);
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
  bool Read_UDP(io_side_t io_side,  mavlink_message_t* msgptr)  {
    if (!wifiSuGood) return false;  
    bool msgRcvd = false;
    mavlink_status_t _status;

    // 2 possible udp objects, STA [0] and    AP [1]       

        len = udp_object[active_object_idx]->parsePacket();
        // esp sometimes reboots here: WiFiUDP.cpp line 213 char * buf = new char[1460]; 

        int udp_count = len;
        if(udp_count > 0) {  
          //if (io_side == gcs_side) Log.printf("Read UDP io_side=%d object=%d port:%d\n", io_side, active_object_idx, udp_read_port);
          while(udp_count--)  {

            int result = udp_object[active_object_idx]->read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {

                    #if (not defined UDP_Broadcast)
                      UDP_remoteIP = udp_object[active_object_idx]->remoteIP();                     
                      bool in_table = false;
                      
                      if (io_side == fc_side) {
                        if (udpremoteip[0] != UDP_remoteIP) {
                          udpremoteip[0] = UDP_remoteIP;   // IP [0] reserved for FC side. can only ever have one FC client at a time
                          Log.printf("%s inserted in table\n", UDP_remoteIP.toString().c_str() ); 
                        }                
                      } else {     // gcs side
                      
                        for (int i = 1 ; i < max_clients ; i++) {
                          if (udpremoteip[i] == UDP_remoteIP) {  // IP already in the table
                        //    Log.printf("%s already in table\n", UDP_remoteIP.toString().c_str() );
                            in_table = true;
                            break;
                          }
                        }
                        if (!in_table) {  // if not in table, add it into empty slot, but not [0] reserved for otgoing (FC side)
                          for (int i = 1 ; i < max_clients ; i++) {
                            if ((udpremoteip[i][0] == 0) || (udpremoteip[i][3] == 255)) {    // overwrite empty or broadcast ips
                              udpremoteip[i] = UDP_remoteIP;    // remember unique IP of remote udp client so we can target it  
                              Log.printf("%s inserted in UDP client table\n", UDP_remoteIP.toString().c_str() );
                              FtRemIP = true;   
                              break;      
                            }
                          }
                        }
                      }
                    #endif  
                    
                    PrintRemoteIP();  
                                   
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
                          checkLinkErrors(msgptr);
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
  void checkLinkErrors(mavlink_message_t* msgptr)   {

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

void Decode_GCS_To_FC() {
  if ((set.gs_io == gs_ser) || (set.gs_io == gs_bt) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt)) { // if any GCS I/O requested
    #if (defined Mav_Print_All_Msgid) || (defined Debug_All)
      Log.printf("GCS to FC - msgid = %3d \n",  G2Fmsg.msgid);
    #endif

   gcs_sysid = G2Fmsg.sysid;
   gcs_compid = G2Fmsg.compid;

    switch(G2Fmsg.msgid) {
       case MAVLINK_MSG_ID_HEARTBEAT:    // #0
       
          gshb_millis = millis();
          gshbGood = true;
          
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Mav_Debug_GCS_Heartbeat                     
            gcs_type = mavlink_msg_heartbeat_get_type(&G2Fmsg); 
            gcs_autopilot = mavlink_msg_heartbeat_get_autopilot(&G2Fmsg);
            gcs_base_mode = mavlink_msg_heartbeat_get_base_mode(&G2Fmsg);
            gcs_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&G2Fmsg);
            gcs_system_status = mavlink_msg_heartbeat_get_system_status(&G2Fmsg);
            gcs_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&G2Fmsg);
  

            Log.print("=====>GCS to FC: #0 Heartbeat: ");  
            Log.print("gcs_sysid="); Log.print(ap_sysid);   
            Log.print("  gcs_compid="); Log.print(ap_compid);                     
            Log.print("  gcs_type="); Log.print(ap_type);   
            Log.print("  gcs_autopilot="); Log.print(ap_autopilot); 
            Log.print("  gcs_base_mode="); Log.print(ap_base_mode); 
            Log.print(" gcs_custom_mode="); Log.print(ap_custom_mode);
            Log.print("  gcs_system_status="); Log.print(ap_system_status); 
            Log.print("  gcs_mavlink_version="); Log.print(ap_mavlink_version);      
            Log.println();
          #endif   
          break;
          
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:  // #20  from GCS
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Debug_Param_Request_Read 
            gcs_target_system = mavlink_msg_param_request_read_get_target_system(&G2Fmsg);        
            mavlink_msg_param_request_read_get_param_id(&G2Fmsg, gcs_req_param_id);
            gcs_req_param_index = mavlink_msg_param_request_read_get_param_index(&G2Fmsg);                  

            Log.print("=====>GCS to FC: #20 Param_Request_Read: ");           
            Log.print("gcs_target_system="); Log.print(gcs_target_system);   
            Log.print("  gcs_req_param_id="); Log.print(gcs_req_param_id);          
            Log.print("  gcs_req_param_index="); Log.print(gcs_req_param_index);    
            Log.println();  
              
            Mavlink_Param_Request_Read(gcs_req_param_index, gcs_req_param_id); 
            
          #endif
          break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:  // #21  To FC from GCS
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Debug_Param_Request_List 
            gcs_target_system = mavlink_msg_param_request_list_get_target_system(&G2Fmsg);  
            gcs_target_component = mavlink_msg_param_request_list_get_target_component(&G2Fmsg);                                  

            Log.print("=====>GCS to FC: #21 Param_Request_List: ");           
            Log.print("gcs_target_system="); Log.print(gcs_target_system); 
            Log.print(" gcs_target_component="); Log.print(gcs_target_component);                    
            Log.println();         
          #endif
          break;

          
          
         case MAVLINK_MSG_ID_MISSION_REQUEST_INT:  // #51 To FC from GCS
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Mav_Debug_Mission
            gcs_target_system = mavlink_msg_mission_request_int_get_target_system(&G2Fmsg);
            gcs_target_component = mavlink_msg_mission_request_int_get_target_component(&G2Fmsg);
            gcs_seq = mavlink_msg_mission_request_int_get_seq(&G2Fmsg); 
            gcs_mission_type = mavlink_msg_mission_request_int_get_seq(&G2Fmsg);                     

            Log.print("=====>GCS to FC: #51 Mission_Request_Int: ");           
            Log.print("gcs_target_system="); Log.print(gcs_target_system);   
            Log.print("  gcs_target_component="); Log.print(gcs_target_component);          
            Log.print("  gcs_seq="); Log.print(gcs_seq);    
            Log.print("  gcs_mission_type="); Log.print(gcs_mission_type);    // Mav2
            Log.println();
          #endif
          break;        
        default:
          if (!mavGood) break;
          #if defined Debug_All || defined Debug_GCS_Up || defined Debug_GCS_Unknown
            Log.print("=====>GCS to FC: ");
            Log.print("Unknown Message ID #");
            Log.print(G2Fmsg.msgid);
            Log.println(" Ignored"); 
          #endif

          break;
    }
  }
}
//================================================================================================= 
void Send_To_FC(uint32_t msg_id) {
  
  if (set.fc_io == fc_ser)  {   // Serial to FC
    len = mavlink_msg_to_send_buffer(FCbuf, &G2Fmsg);
    mvSerial.write(FCbuf,len);  
         
    #if defined  Debug_FC_Up || defined Debug_GCS_Up
      if (msg_id) {    //  dont print heartbeat - too much info
        Log.printf("Sent to FC Serial from G2Fmsg: len=%d\n", len);
        PrintMavBuffer(&G2Fmsg);
      }  
    #endif    
  }

  #if (defined btBuiltin) 
    if (set.fc_io == fc_bt)  {   // BT to FC
        bool msgSent = Send_Bluetooth(&G2Fmsg);      
        #ifdef  Debug_FC_Up
          Log.print("Sent to FC Bluetooth from G2Fmsg: msgSent="); Log.println(msgSent);
          if (msgSent) PrintMavBuffer(&G2Fmsg);
        #endif     
    }
  #endif

  #if (defined wifiBuiltin)
    if (set.fc_io == fc_wifi) {  // WiFi to FC
      if (wifiSuGood) { 
        if (set.mav_wfproto == tcp)  { // TCP  
           active_client_idx = 0;             // tcp_client[0] is reserved for inbound client (FC side)
           bool msgSent = Send_TCP(&G2Fmsg);  // to FC   
           #ifdef  Debug_GCS_Up
             Log.print("Sent to FC WiFi TCP from G2Fmsg: msgSent="); Log.println(msgSent);
             PrintMavBuffer(&G2Fmsg);
           #endif    
         }    
         
         if (set.mav_wfproto == udp)  { // UDP 

          if ((set.wfmode == ap_sta) || (set.wfmode == sta)) {// if AP_STA or STA mode                 
            active_object_idx = 0;                            // Use STA UDP object for FC send     
            udp_read_port = set.udp_remotePort;           
            udp_send_port = set.udp_localPort;                   
          } else {
            if (set.wfmode == ap) {
              active_object_idx = 1;                          // Use AP UDP object for FC send 
              udp_read_port = set.udp_localPort;  
              udp_send_port = set.udp_remotePort;                            
            }               
          }   
           UDP_remoteIP = udpremoteip[0];                     // Fc side can only ever have 1 client
           //Log.printf("Send to FC udpremoteip[0]=%s UDP_remoteIP= %s\n", udpremoteip[0].toString().c_str(), UDP_remoteIP.toString().c_str());                   
           bool msgSent = Send_UDP(fc_side, &G2Fmsg);  // to FC    
           #if ( (defined  Debug_GCS_Up) || (defined Debug_Send_UDP_FC) ) 
             Log.print("Sent to FC WiFi UDP from G2Fmsg: msgSent="); Log.println(msgSent);
             if (msgSent) PrintMavBuffer(&G2Fmsg);
           #endif           
          }                                                             
      }
   }
  #endif       
}  
//================================================================================================= 

void MavToRingBuffer() {

      // MAIN Queue
      if (MavRingBuff.isFull()) {
        BufLedState = HIGH;
        Log.println("MavRingBuff full. Ignoring new records");
     //   LogScreenPrintln("Mav buff full"); 
      }
       else {
        BufLedState = LOW;
        MavRingBuff.push(F2Rmsg);
        #if defined Mav_Debug_RingBuff
          Log.print("Ring queue = "); 
          Log.println(MavRingBuff.size());
        #endif
      }
  }
  
//================================================================================================= 

void Send_From_RingBuf_To_GCS() {   // Down to GCS (or other) from Ring Buffer
  
  if ((set.gs_io == gs_ser) || (set.gs_io == gs_bt) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.gs_sd == gs_on)) {

      if (set.gs_io == gs_ser) {  // Serial
        len = mavlink_msg_to_send_buffer(GCSbuf, &R2Gmsg);
        mvSerial.write(GCSbuf,len);  

        #ifdef  Debug_GCS_Down
          Log.printf("Sent from ring buffer to GCS Serial: len=%d\n", len);
          PrintMavBuffer(&R2Gmsg);
        #endif
      }
 

  #if (defined btBuiltin)
    if ((set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt))  {  // Bluetooth
      len = mavlink_msg_to_send_buffer(GCSbuf, &R2Gmsg);     
      #ifdef  Debug_GCS_Down
        Log.println("Passed down from Ring buffer to GCS by Bluetooth:");
        PrintMavBuffer(&R2Gmsg);
      #endif
      if (SerialBT.hasClient()) {
        SerialBT.write(GCSbuf,len);
      }
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
                           
              msgSent = Send_TCP(&R2Gmsg);  // to GCS side
              if ((millis() -  prev_millis) > 4) {
                PrintPeriod(false);  Log.println(" TCP Write timeout =====================================>");  
              }
              
              #ifdef  Debug_GCS_Down
                if (msgSent) {
                  Log.printf("Sent to GCS client %d by WiFi TCP: msgid=%d\n", active_client_idx+1, R2Gmsg.msgid);  
                }
              //  Log.printf("Sent to GCS client %d by WiFi TCP: msgid=%d  msgSent=%d\n", active_client_idx+1, R2Gmsg.msgid, msgSent);
              //  PrintMavBuffer(&R2Gmsg);
              #endif
            }
          }    
        }
        
        if (set.mav_wfproto == udp)  { // UDP     

          if ((set.wfmode == ap_sta) || (set.wfmode == ap)) { // if AP_STA or AP mode                 
            active_object_idx = 1;                            // Use AP UDP object for FC send     
            udp_read_port = set.udp_localPort;           
            udp_send_port = set.udp_remotePort;                   
          } else {
            if (set.wfmode == sta) {
              active_object_idx = 0;                          // Use STA UDP object for FC send 
              //Log.printf("active_object_idx set to 0\n");  
              udp_read_port = set.udp_localPort;  
              udp_send_port = set.udp_remotePort;                       
            }               
          }   
          
          #if defined UDP_Broadcast                     // broadcast to remote udp clients 
            UDP_remoteIP[3] = 255; 
            msgSent = Send_UDP(gcs_side, &R2Gmsg);  // to GCS
            msgSent = msgSent; // stop stupid compiler warnings                       
          #else
                   
            for (int i = 1 ; i < max_clients ; i++) {   // send to each individual remote udp ip. Not to FC side client ip [0] 
              if (udpremoteip[i][0] != 0) {
                //Log.printf("Non-zero ip i=%d  ip=%s\n", i, udpremoteip[i].toString().c_str());
                UDP_remoteIP =  udpremoteip[i];         // target the remote IP
                msgSent = Send_UDP(gcs_side, &R2Gmsg);            // to GCS
                msgSent = msgSent; // stop stupid compiler warnings    

                #if (defined Debug_GCS_Down) || (defined Debug_Send_UDP_GCS)
                  Log.print("Sent to GCS by WiFi UDP: msgSent="); Log.println(msgSent);
                  PrintMavBuffer(&R2Gmsg);
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
             Log.println("Failed to open file for appending");
             sdStatus = 9;
             return;
            }

         memcpy(GCSbuf, (void*)&ap_time_unix_usec, sizeof(uint64_t));
         len=mavlink_msg_to_send_buffer(GCSbuf+sizeof(uint64_t), &R2Gmsg);

         if(file.write(GCSbuf, len+18)){   // 8 bytes plus some head room   
            } else {
            Log.println("Append failed");
           }
         
          file.close();
        
          #ifdef  Debug_SD
            Log.println("Passed down from Ring buffer to SD:");
            PrintMavBuffer(&R2Gmsg);
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
      link_status.packets_sent++;
    }

    return msgSent;
  }
#endif
//================================================================================================= 
#if (defined wifiBuiltin)
  bool Send_TCP(mavlink_message_t* msgptr) {
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
  bool Send_UDP(io_side_t io_side, mavlink_message_t* msgptr) {
    if (!wifiSuGood) return false;  

    // 2 possible udp objects, STA [0]  and    AP [1] 
        
    bool msgSent = false;

    if (msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      UDP_remoteIP[3] = 255;       // always broadcast a heartbeat, either from the GCS or from the FC                 
   //   Log.print("Broadcast heartbeat UDP_remoteIP="); Log.println(UDP_remoteIP.toString());     
    } 
    
    //if (io_side == fc_side) Log.printf("Send UDP io_side=%d object=%d remote ip = %s:%d\n", io_side, active_object_idx, UDP_remoteIP.toString().c_str(), udp_send_port);

    udp_object[active_object_idx]->beginPacket(UDP_remoteIP, udp_send_port);  //  udp ports gets flipped for sta_ap mode
    
    uint16_t len = mavlink_msg_to_send_buffer(sendbuf, msgptr);
  
    size_t sent = udp_object[active_object_idx]->write(sendbuf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
      udp_object[active_object_idx]->flush();
    }

    bool endOK = udp_object[active_object_idx]->endPacket();
 //   if (!endOK) Log.printf("msgSent=%d   endOK=%d\n", msgSent, endOK);
    return msgSent;
  }
#endif
//================================================================================================= 

void DecodeOneMavFrame() {
  
   #if defined Mav_Print_All_Msgid
     uint16_t sz = sizeof(R2Gmsg);
     Log.printf("FC to GGS - msgid = %3d Msg size =%3d\n",  R2Gmsg.msgid, sz);
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

          if ((ap_base_mode >> 7) && (pt_gps_status >= 3) && (!homGood)) // AND 3D Fix
            MarkHome();  // If motors armed for the first time, then mark this spot as home

          hb_count++; 
          
          if(!mavGood) {
            Log.printf("hb_count=%d\n", hb_count);
            if(hb_count >= 3) {        // If  3 heartbeats from MavLink then we are connected
              mavGood=true;
              Log.println("Mavlink good!");
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
            Log.print("Mavlink from FC #0 Heartbeat: ");  
            Log.print("ap_sysid="); Log.print(ap_sysid);   
            Log.print("  ap_compid="); Log.print(ap_compid);                      
            Log.print("  ap_type(frame)="); Log.print(ap_type);   
            Log.print("  ap_autopilot="); Log.print(ap_autopilot); 
            Log.print("  ap_base_mode="); Log.print(ap_base_mode); 
            Log.print(" ap_custom_mode="); Log.print(ap_custom_mode);
            Log.print("  ap_system_status="); Log.print(ap_system_status); 
            Log.print("  ap_mavlink_version="); Log.print(ap_mavlink_version);   

            if (px4_flight_stack) {         
              Log.print(" px4_main_mode="); Log.print(px4_main_mode); 
              Log.print(" px4_sub_mode="); Log.print(px4_sub_mode);  
              Log.print(" ");Log.print(PX4FlightModeName(px4_main_mode, px4_sub_mode));  
           }

           if (pitlab_flight_stack) {         
              Log.print(" Pitlab flight stack detected");
           }           
            
            Log.println();
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
            
          pt_bat1_volts = ap_voltage_battery1 * 0.01F;         // mV -> dV
          pt_bat1_amps = ap_current_battery1 * 0.1F;           // cA -> dA 
                     
          #if defined Mav_Debug_All || defined Mav_Debug_SysStatus || defined Debug_Batteries
            Log.print("Mavlink from FC #1 Sys_status: ");     
            Log.print(" Sensor health=");
            Log.print(ap_onboard_control_sensors_health);   // 32b bitwise 0: error, 1: healthy.
            Log.print(" Bat volts=");
            Log.print(ap_voltage_battery1 * 0.001F, 3);   // mV -> V
            Log.print("  Bat amps=");
            Log.print(ap_current_battery1 * 0.01F, 3);    // dA -> A
            Log.print("  mAh="); Log.print(bat1.mAh, 6);    
            Log.print("  Total mAh="); Log.print(bat1.tot_mAh, 3);  // Consumed so far, calculated in Average module
         
            Log.print("  Bat1 cell count= "); 
            Log.println(ap_ccell_count1);
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
            Log.print("Mavlink from FC #2 System_Time: ");        
            Log.print(" Unix secs="); Log.print((float)(ap_time_unix_usec/1E6), 6);  
            Log.print("  Boot secs="); Log.println((float)(ap_time_boot_ms/1E3), 0);   
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
            // Log.printf("matched row mt20_row=%d\n", mt20_row);
            if (mt20_row != 0xffff) {       // if matched

              #if defined Mav_Debug_All || defined Debug_Mavlite
                  Log.print("Mavlink return from FC #22 Param_Value for MavLITE: ");
                  Log.print("ap22_param_id="); Log.print(ap22_param_id);
                  Log.print(" ap22_param_value="); Log.println(ap22_param_value, 3);             
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
                Log.print("Mavlink from FC #22 Param_Value: ");
                Log.print("bat1 capacity=");
                Log.println(ap_bat1_capacity);
              #endif
              break;
            case 364:         // Bat2 Capacity
              ap_bat2_capacity = ap22_param_value;
              #if (defined frBuiltin)
                FrPort.PushMessage(0x5007, 5);    // Bat2 capacity
              #endif  
              
              ap_bat_paramsRead = true;
              #if defined Mav_Debug_All || defined Debug_Batteries
                Log.print("Mavlink from FC #22 Param_Value: ");
                Log.print("bat2 capacity=");
                Log.println(ap_bat2_capacity);
              #endif             
              break;
          } 
             
          #if defined Mav_Debug_All || defined Mav_Debug_Params || defined Mav_List_Params || defined Debug_Mavlite
            Log.print("Mavlink from FC #22 Param_Value: ");
            Log.print("param_id=");
            Log.print(ap22_param_id);
            Log.print("  param_value=");
            Log.print(ap22_param_value, 4);
            Log.print("  param_count=");
            Log.print(ap22_param_count);
            Log.print("  param_index=");
            Log.println(ap22_param_index);
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
            Log.print("Mavlink from FC #24 GPS_RAW_INT: ");  
            Log.print("ap24_fixtype="); Log.print(ap24_fixtype);
            if (ap24_fixtype==0) Log.print(" No GPS");
              else if (ap24_fixtype==1) Log.print(" No Fix");
              else if (ap24_fixtype==2) Log.print(" 2D Fix");
              else if (ap24_fixtype==3) Log.print(" 3D Fix");
              else if (ap24_fixtype==4) Log.print(" DGPS/SBAS aided");
              else if (ap24_fixtype==5) Log.print(" RTK Float");
              else if (ap24_fixtype==6) Log.print(" RTK Fixed");
              else if (ap24_fixtype==7) Log.print(" Static fixed");
              else if (ap24_fixtype==8) Log.print(" PPP");
              else Log.print(" Unknown");

            Log.print("  sats visible="); Log.print(ap24_sat_visible);
            Log.print("  latitude="); Log.print((float)(ap24_lat)/1E7, 7);
            Log.print("  longitude="); Log.print((float)(ap24_lon)/1E7, 7);
            Log.print("  gps alt amsl="); Log.print((float)(ap24_amsl)/1E3, 1);
            Log.print("  eph (hdop)="); Log.print((float)ap24_eph);                // HDOP
            Log.print("  epv (vdop)="); Log.print((float)ap24_epv);
            Log.print("  vel="); Log.print((float)ap24_vel / 100, 3);              // GPS ground speed (m/s)
            Log.print("  cog="); Log.print((float)ap24_cog / 100, 1);              // Course over ground in degrees
            //  mav2
            Log.print("  alt_ellipsoid)="); Log.print(ap24_alt_ellipsoid / 1000, 2);      // alt_ellipsoid in mm
            Log.print("  h_acc="); Log.print(ap24_h_acc);                          // Position uncertainty in mm. Positive for up.
            Log.print("  v_acc="); Log.print(ap24_v_acc);                          // Altitude uncertainty in mm. Positive for up.
            Log.print("  ap24_vel_acc="); Log.print(ap24_vel_acc);                 // Speed uncertainty. Positive for up.
            Log.print("  ap24_hdg_acc="); Log.print(ap24_hdg_acc);                 // degE5   Heading / track uncertainty 
            Log.println();
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
            Log.print("Mavlink from FC #26 Scaled_IMU: ");
            Log.print("xacc="); Log.print((float)ap26_xacc / 1000, 3); 
            Log.print("  yacc="); Log.print((float)ap26_yacc / 1000, 3); 
            Log.print("  zacc="); Log.print((float)ap26_zacc / 1000, 3);
            Log.print("  xgyro="); Log.print((float)ap26_xgyro / 1000, 3); 
            Log.print("  ygyro="); Log.print((float)ap26_ygyro / 1000, 3); 
            Log.print("  zgyro="); Log.print((float)ap26_zgyro / 1000, 3);
            Log.print("  xmag="); Log.print((float)ap26_xmag / 1000, 3); 
            Log.print("  ymag="); Log.print((float)ap26_ymag / 1000, 3); 
            Log.print("  zmag="); Log.print((float)ap26_zmag / 1000, 3);  
            Log.print("  temp="); Log.println((float)ap26_temp / 100, 2);    // cdegC                              
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
            Log.print("Mavlink from FC #27 Raw_IMU: ");
            Log.print("accX="); Log.print((float)ap27_xacc / 1000); 
            Log.print("  accY="); Log.print((float)ap27_yacc / 1000); 
            Log.print("  accZ="); Log.println((float)ap27_zacc / 1000);
            Log.print("  xgyro="); Log.print((float)ap27_xgyro / 1000, 3); 
            Log.print("  ygyro="); Log.print((float)ap27_ygyro / 1000, 3); 
            Log.print("  zgyro="); Log.print((float)ap27_zgyro / 1000, 3);
            Log.print("  xmag="); Log.print((float)ap27_xmag / 1000, 3); 
            Log.print("  ymag="); Log.print((float)ap27_ymag / 1000, 3); 
            Log.print("  zmag="); Log.print((float)ap27_zmag / 1000, 3);
            Log.print("  id="); Log.print((float)ap27_id);             
            Log.print("  temp="); Log.println((float)ap27_temp / 100, 2);    // cdegC               
          #endif 
        #endif             
          break; 
    
        case MAVLINK_MSG_ID_SCALED_PRESSURE:         // #29
        #if defined Decode_Non_Essential_Mav
          if (!mavGood) break;        
          ap_press_abs = mavlink_msg_scaled_pressure_get_press_abs(&R2Gmsg);
          ap_temperature = mavlink_msg_scaled_pressure_get_temperature(&R2Gmsg);
          #if defined Mav_Debug_All || defined Mav_Debug_Scaled_Pressure
            Log.print("Mavlink from FC #29 Scaled_Pressure: ");
            Log.print("  press_abs=");  Log.print(ap_press_abs,1);
            Log.print("hPa  press_diff="); Log.print(ap_press_diff, 3);
            Log.print("hPa  temperature=");  Log.print((float)(ap_temperature)/100, 1); 
            Log.println("C");             
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
            Log.print("Mavlink from FC #30 Attitude: ");      
            Log.print(" ap_roll degs=");
            Log.print(ap_roll, 1);
            Log.print(" ap_pitch degs=");   
            Log.print(ap_pitch, 1);
            Log.print(" ap_yaw degs=");         
            Log.println(ap_yaw, 1);
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
          cur.hdg = ap33_gps_hdg / 100;

          #if defined Mav_Debug_All || defined Mav_Debug_GPS_Int
            Log.print("Mavlink from FC #33 GPS Int: ");
            Log.print(" ap_lat="); Log.print((float)ap33_lat / 1E7, 6);
            Log.print(" ap_lon="); Log.print((float)ap33_lon / 1E7, 6);
            Log.print(" ap_amsl="); Log.print((float)ap33_amsl / 1E3, 0);
            Log.print(" ap33_alt_ag="); Log.print((float)ap33_alt_ag / 1E3, 1);           
            Log.print(" ap33_vx="); Log.print((float)ap33_vx / 100, 2);
            Log.print(" ap33_vy="); Log.print((float)ap33_vy / 100, 2);
            Log.print(" ap33_vz="); Log.print((float)ap33_vz / 100, 2);
            Log.print(" ap33_gps_hdg="); Log.println((float)ap33_gps_hdg / 100, 1);
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
                Log.print("Auto RSSI_Source===>  ");
              #endif
            #endif     
          }

          #if defined Mav_Debug_All || defined Debug_Rssi || defined Mav_Debug_RC
            Log.print("Mavlink from FC #35 RC_Channels_Raw: ");                        
            Log.print("  ap_rssi35=");  Log.print(ap_rssi35);   // 0xff -> 100%
            Log.print("  rssiGood=");  Log.println(rssiGood); 
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
            Log.print("Mavlink from FC #36 servo_output: ");
            Log.print("ap_port="); Log.print(ap_port); 
            Log.print(" PWM: ");
            for (int i=0 ; i < 8; i++) {
              Log.print(" "); 
              Log.print(i+1);
              Log.print("=");  
              Log.print(ap_servo_raw[i]);   
            }                         
            Log.println();     
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
                     
            #if defined Mav_Debug_All || defined Mav_Debug_Mission
              Log.print("Mavlink from FC #39 Mission Item: ");
              Log.print("ap_ms_seq="); Log.print(ap_ms_seq);  
              Log.print(" ap_ms_frame="); Log.print(ap_ms_frame);   
              Log.print(" ap_ms_command="); Log.print(ap_ms_command);   
              Log.print(" ap_ms_current="); Log.print(ap_ms_current);   
              Log.print(" ap_ms_autocontinue="); Log.print(ap_ms_autocontinue);  
              Log.print(" ap_ms_param1="); Log.print(ap_ms_param1, 7);   
              Log.print(" ap_ms_param2="); Log.print(ap_ms_param2, 7);   
              Log.print(" ap_ms_param3="); Log.print(ap_ms_param3, 7);  
              Log.print(" ap_ms_param4="); Log.print(ap_ms_param4, 7); 
              Log.print(" ap_ms_x="); Log.print(ap_ms_x, 7);   
              Log.print(" ap_ms_y="); Log.print(ap_ms_y, 7);   
              Log.print(" ap_ms_z="); Log.print(ap_ms_z,0); 
              Log.print(" ap_mission_type="); Log.print(ap_mission_type); 
              Log.println();    
            #endif
            
            if (ap_ms_seq > Max_Waypoints) {
              Log.println(" Max Waypoints exceeded! Waypoint ignored.");
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
              Log.print("Mavlink from FC #42 Mission Current: ");
              Log.print("ap_mission_current="); Log.println(ap_ms_seq);   
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
       
            #if defined Mav_Debug_All || defined Mav_Debug_Mission
              Log.print("Mavlink from FC #44 Mission Count: ");
              Log.print("ap_mission_count="); Log.println(ap_mission_count);   
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
              Log.print("Mavlink from FC #62 Nav_Controller_Output - (+Waypoint): ");
              Log.print("ap_nav_roll="); Log.print(ap_nav_roll, 3);  
              Log.print(" ap_nav_pitch="); Log.print(ap_nav_pitch, 3);   
              Log.print(" ap_nav_bearing="); Log.print(ap_nav_bearing);   
              Log.print(" ap_target_bearing="); Log.print(ap_target_bearing);   
              Log.print(" ap_wp_dist="); Log.print(ap_wp_dist);  
              Log.print(" ap_alt_error="); Log.print(ap_alt_error, 2);   
              Log.print(" ap_aspd_error="); Log.print(ap_aspd_error, 2);   
              Log.print(" ap_xtrack_error="); Log.print(ap_xtrack_error, 2);  
              Log.println();    
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
                  Log.print("Auto RSSI_Source===>  ");
                #endif
              #endif     
              }
             
            #if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_RC
              Log.print("Mavlink from FC #65 RC_Channels: ");
              Log.print("ap65_chcnt="); Log.print(ap65_chcnt); 
              Log.print(" PWM: ");
              for (int i=0 ; i < ap65_chcnt ; i++) {
                Log.print(" "); 
                Log.print(i+1);
                Log.print("=");  
                Log.print(ap65_chan_raw[i]);   
              }                         
              Log.print("  ap65_rssi=");  Log.print(ap65_rssi); 
              Log.print("  rssiGood=");  Log.println(rssiGood);         
            #endif             
          break;   
             
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:     // #66 - UPLINK  TO UAV
          if (!mavGood) break;       
          break; 
          
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:       // #73   received back after #51 Mission_Request_Int sent
        #if defined Mav_Debug_All || defined Mav_Debug_Mission
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
 
          Log.print("Mavlink from FC #73 Mission_Item_Int: ");
          Log.print("target_system ="); Log.print(ap_targsys);   
          Log.print("  target_component ="); Log.print(ap_targcomp);   
          Log.print(" _seq ="); Log.print(ap73_seq);   
          Log.print("  frame ="); Log.print(ap73_frame);     
          Log.print("  command ="); Log.print(ap73_command);   
          Log.print("  current ="); Log.print(ap73_current);   
          Log.print("  autocontinue ="); Log.print(ap73_autocontinue);   
          Log.print("  param1 ="); Log.print(ap73_param1, 2); 
          Log.print("  param2 ="); Log.print(ap73_param2, 2); 
          Log.print("  param3 ="); Log.print(ap73_param3, 2); 
          Log.print("  param4 ="); Log.print(ap73_param4, 2);                                          
          Log.print("  x ="); Log.print(ap73_x);   
          Log.print("  y ="); Log.print(ap73_y);   
          Log.print("  z ="); Log.print(ap73_z, 4);    
          Log.print("  mission_type ="); Log.println(ap73_mission_type);                                                    

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
            Log.print("Mavlink from FC #74 VFR_HUD: ");
            Log.print("Airspeed= "); Log.print(ap74_air_spd, 2);                 // m/s    
            Log.print("  Groundspeed= "); Log.print(ap74_grd_spd, 2);            // m/s
            Log.print("  Heading= ");  Log.print(ap74_hdg);                      // deg
            Log.print("  Throttle %= ");  Log.print(ap74_throt);                 // %
            Log.print("  Baro alt= "); Log.print(ap74_amsl, 0);                  // m                  
            Log.print("  Climb rate= "); Log.println(ap74_climb);                // m/s
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
          
          #if defined Mav_Debug_All || defined Mav_Debug_Command_Ack || defined Debug_Mavlite                
            Log.print("Mavlink from FC #77 Command_Ack: ");
            Log.print("command="); Log.print(ap77_command);   
            Log.print(" result="); Log.print(ap77_result);   
            Log.print(" progres="); Log.print(ap77_progress);   
            Log.print(" result_parm2="); Log.print(ap77_result_param2);     
            Log.print(" target_system="); Log.print(ap77_target_system);   
            Log.print(" target_component="); Log.println(ap77_target_component);                                                    
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
                Log.print("Auto RSSI_Source===>  ");
              #endif
            #endif     

            #if defined Mav_Debug_All || defined Debug_Radio_Status || defined Mav_Debug_Rssi
              Log.print("Mavlink from FC #109 Radio: "); 
              Log.print("ap109_rssi="); Log.print(ap109_rssi);
              Log.print("  remrssi="); Log.print(ap109_remrssi);
              Log.print("  txbuf="); Log.print(ap109_txbuf);
              Log.print("  noise="); Log.print(ap109_noise); 
              Log.print("  remnoise="); Log.print(ap109_remnoise);
              Log.print("  rxerrors="); Log.print(ap109_rxerrors);
              Log.print("  fixed="); Log.print(ap109_fixed);  
              Log.print("  rssiGood=");  Log.println(rssiGood);                                
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
              Log.print("Mavlink from FC #125 Power Status: ");
              Log.print("Vcc= "); Log.print(ap_Vcc); 
              Log.print("  Vservo= ");  Log.print(ap_Vservo);       
              Log.print("  flags= ");  Log.println(ap_flags);       
            #endif  
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
            Log.print("Mavlink from FC #147 Battery Status: ");
            Log.print(" bat id= "); Log.print(ap_battery_id); 
            Log.print(" bat current mA= "); Log.print(ap_current_battery*10); // now shows mA
            Log.print(" ap_current_consumed mAh= ");  Log.print(ap_current_consumed);   
            if (ap_battery_id == 0) {
              Log.print(" my di/dt mAh= ");  
              Log.println(Total_mAh1(), 0);  
            }
            else {
              Log.print(" my di/dt mAh= ");  
              Log.println(Total_mAh2(), 0);   
            }    
        //  Log.print(" bat % remaining= ");  Log.println(ap_time_remaining);       
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
            Log.print("Mavlink from FC #173 rangefinder: ");        
            Log.print(" distance=");
            Log.println(ap_range);   // now V
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
            Log.print("Mavlink from FC #181 Battery2: ");        
            Log.print(" Bat volts=");
            Log.print((float)ap_voltage_battery2 / 1000, 3);   // now V
            Log.print("  Bat amps=");
            Log.print((float)ap_current_battery2 / 100, 1);   // now A
              
            Log.print("  mAh="); Log.print(bat2.mAh, 6);    
            Log.print("  Total mAh="); Log.print(bat2.tot_mAh, 3);
         
            Log.print("  Bat cell count= "); 
            Log.println(ap_cell_count2);
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
            Log.print("Mavlink from FC #226 RPM: ");
            Log.print("RPM1= "); Log.print(ap_rpm1, 0); 
            Log.print("  RPM2= ");  Log.println(ap_rpm2, 0);          
          #endif       
          break;

        case MAVLINK_MSG_ID_STATUSTEXT:        // #253      
          ap_severity = mavlink_msg_statustext_get_severity(&R2Gmsg);
          len=mavlink_msg_statustext_get_text(&R2Gmsg, ap_text);

          #if defined Mav_Debug_All || defined Mav_Debug_StatusText
            Log.print("Mavlink from FC #253 Statustext pushed onto MsgRingBuff: ");
            Log.print(" Severity="); Log.print(ap_severity);
            Log.print(" "); Log.print(MavSeverity(ap_severity));
            Log.print("  Text= ");  Log.print(" |"); Log.print(ap_text); Log.println("| ");
          #endif
          
          #if (defined frBuiltin)
            FrPort.PushMessage(0x5000, 0);         // 0x5000 StatusText Message
          #endif  
          
          break;                                      
        default:
          if (!mavGood) break;
          #if defined Mav_Debug_All || defined Mav_Show_Unknown_Msgs
            Log.print("Mavlink from FC: ");
            Log.print("Unknown Message ID #");
            Log.print(R2Gmsg.msgid);
            Log.println(" Ignored"); 
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

  #if defined Mav_Debug_All || defined Mav_Debug_GPS_Int
    Log.print("******************************************Mavlink in #33 GPS Int: Home established: ");       
    Log.print("hom.lat=");  Log.print(hom.lat, 7);
    Log.print(" hom.lon=");  Log.print(hom.lon, 7 );        
    Log.print(" hom.alt="); Log.print(hom.alt, 1);
    Log.print(" hom.hdg="); Log.println(hom.hdg);                   
 #endif  
}
//================================================================================================= 
void Send_FC_Heartbeat() {
  
  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;                             // FC
  apo_targcomp = 1;                            // FC                   

  apo_type = MAV_TYPE_GCS;                       // 6 Pretend to be a GCS
  apo_autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;   // 3 AP Mega
  apo_base_mode = 0;
  apo_system_status = MAV_STATE_ACTIVE;         // 4
   
  mavlink_msg_heartbeat_pack(apo_sysid, apo_compid, &G2Fmsg, apo_type, apo_autopilot, apo_base_mode, apo_system_status, 0); 
  Send_To_FC(0); 
  #if defined Debug_Our_FC_Heartbeat
     Log.print("Our own heartbeat to FC: #0 Heartbeat: ");  
     Log.print("apo_sysid="); Log.print(apo_sysid);   
     Log.print("  apo_compid="); Log.print(apo_compid);  
     Log.print("  apo_targsys="); Log.print(apo_targsys);   
     Log.print("  apo_targcomp="); Log.print(apo_targcomp);                         
     Log.print("  apo_type="); Log.print(apo_type);   
     Log.print("  apo_autopilot="); Log.print(apo_autopilot); 
     Log.print("  apo_base_mode="); Log.print(apo_base_mode); 
     Log.print("  apo_custom_mode="); Log.print(apo_custom_mode);
     Log.print("  apo_system_status="); Log.print(apo_system_status);    
     Log.println();
  #endif   
}
//================================================================================================= 
void Mavlink_Param_Request_Read(int16_t param_index, char * param_id) {  // #20
  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;                             // APM FC
  apo_targcomp = 1;                            // APM FC
  
  mavlink_msg_param_request_read_pack(apo_sysid, apo_compid, &G2Fmsg,
                   apo_targsys, apo_targcomp, param_id, param_index);              
  Send_To_FC(20);             
 }

//================================================================================================= 
 void Request_Param_List() {

  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;                           // APM FC
  apo_targcomp = 1;                            // APM FC
  
  mavlink_msg_param_request_list_pack(apo_sysid,  apo_compid, &G2Fmsg,
                    apo_targsys,  apo_targcomp);
              
  Send_To_FC(21);
                    
 }
 //================================================================================================= 
 void Mavlink_Param_Set() {    // #23

  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190 
  apo_targsys = 1;                             // APM FC
  apo_targcomp = 1;                            // APM FC
                                          
  mavlink_msg_param_set_pack(apo_sysid, apo_compid, &G2Fmsg,
                        apo_targsys, apo_targcomp, ap23_param_id, ap23_param_value, 10);  // 10=MAV_PARAM_TYPE_REAL64 https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE            
  Send_To_FC(23);  // #23  

  #if defined Debug_Mavlite
     Log.printf("UPLINK: Mavlink sent #23 Param_Set. Parameter-id= %s  Value = %.5f\n", ap23_param_id, ap23_param_value);  
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
  
  mavlink_msg_mission_request_pack(apo_sysid, aop_compid, &G2Fmsg,
                               apo_targsys, apo_targcomp, ms_seq, ap_mission_type);

  Send_To_FC(40);
  #if defined Mav_Debug_All || defined Mav_Debug_Mission
    Log.print("Mavlink to FC #40 Request Mission:  ms_seq="); Log.println(ms_seq);
  #endif  
}
#endif 
 
//================================================================================================= 
#if defined Request_Missions_From_FC || defined Request_Mission_Count_From_FC
void Mavlink_Request_Mission_List() {   // #43   get back #44 Mission_Count
  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  ap_targsys = 1;                              // APM FC
  ap_targcomp = 1; 
  ap_mission_type = 0;   // Mav2  0 = Items are mission commands for main mission
  
  mavlink_msg_mission_request_list_pack(apo_sysid, apo_compid, &G2Fmsg,
                               apo_targsys, apo_targcomp, ap_mission_type);
                             
  Send_To_FC(43);
  #if defined Mav_Debug_All || defined Mav_Debug_Mission
    Log.println("Mavlink to FC #43 Request Mission List (count)");
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
void RequestDataStreams() {    //  REQUEST_DATA_STREAM ( #66 ) DEPRECATED. USE SRx, SET_MESSAGE_INTERVAL INSTEAD

  apo_sysid = 255;              // Reply to APM FC
  apo_compid = 190;             // Reply to AMP FC
  apo_targsys = 1;              // FC
  apo_targcomp = 1;             // FC

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
    mavlink_msg_request_data_stream_pack(apo_sysid, apo_compid, &G2Fmsg,
        apo_targsys, apo_targcomp, mavStreams[i], mavRates[i], 1);    // start_stop 1 to start sending, 0 to stop sending   
                          
  Send_To_FC(66);
    }
 // Log.println("Mavlink to FC #66 Request Data Streams:");
}
#endif
//================================================================================================= 
void Mavlink_Command_Long() {  // #76

  apo_sysid = Device_sysid;                    // Reply to this device. From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;                             // FC
  apo_targcomp = 1;                            // FC
  
  mavlink_msg_command_long_pack(apo_sysid, apo_compid, &G2Fmsg,
          apo_targsys, apo_targcomp, ap76_command, ap76_confirm, ap76_param[0], ap76_param[1], 
          ap76_param[2], ap76_param[3], ap76_param[4], ap76_param[5], ap76_param[6]); 
     
  Send_To_FC(76);  // #76                
 }
 //================================================================================================= 
 

 //=================================================================================================  
//================================================================================================= 
//
//                                       U T I L I T I E S
//
//================================================================================================= 
//=================================================================================================
 

//=================================================================================================   
//                             W I F I   S U P P O R T   -   ESP Only 
//=================================================================================================  
#if (defined wifiBuiltin)

  void IRAM_ATTR gotWifiButton(){        // "Start WiFi" Button
    if (millis() - debnceTimr < delaytm) return;
    debnceTimr = millis();  
    wifiButton = true;
    
  }
  //==========================================
  void SenseWiFiPin() {
    #if defined Start_WiFi
      if (!wifiSuDone) {
        SetupWiFi();
        return;
      }
      return;
    #else
      if ((wifiButton) && (!wifiSuDone)) {
        wifiButton = false;
        SetupWiFi();
        } 
    #endif      
  }
  
  //===============================       H a n d l e   W i F i   E v e n t s 
  
    void CheckStaLinkStatus();  // Forward declaration 
    
    void ServiceWiFiRoutines() {
      
      SenseWiFiPin();   // check optional start-wifi pin
      
      if ((set.wfmode == sta) && wifiSuGood && wifiSuDone)  { // in sta mode check for disconnect from AP
        CheckStaLinkStatus();    // Handle disconnect/reconnect from AP
      }
      
      ServiceInboundTCPClients();  // from GCS side

      // Report stations connected to/from our AP
      AP_sta_count = WiFi.softAPgetStationNum();
      
      if (AP_sta_count > AP_prev_sta_count) {  
        AP_prev_sta_count = AP_sta_count;
        Log.printf("Remote STA %d connected to our AP\n", AP_sta_count);  
        snprintf(snprintf_buf, snp_max, "STA %d connected", AP_sta_count);                
        LogScreenPrintln(snprintf_buf); 
        
        if ((set.fc_io == fc_wifi) && (set.mav_wfproto == tcp))  {  // if we expect wifi from the fc, we are a client
          if (!outbound_clientGood) // and we don't have an active tcp session, start a new session
          outbound_clientGood = NewOutboundTCPClient();
        }                         
      } else 
      if (AP_sta_count < AP_prev_sta_count) {      // a device has disconnected from the AP
        AP_prev_sta_count = AP_sta_count;
        Log.println("A STA disconnected from our AP");     // back in listening mode
        LogScreenPrintln("A STA disconnected"); 
      }
   }  

   //===============================     H a n d l e   I n B o u n d   T C P   C l i e n t s ( G C S   s i d e )

   void ServiceInboundTCPClients() {
    
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt)) {  // Only when we expect incoming remote tcp_client
      
      if (set.mav_wfproto == tcp)  {  // TCP  
        if (wifiSuGood) {

          WiFiClient newClient = TCPserver.available();  // check if a new client wants to connect
          if (newClient) {
            for (int i=0 ; i < max_clients ; ++i) {      // if connected add it into our connected tcp_client table
              if (NULL == tcp_client[i]) {                  // find first empty slot in table
                tcp_client[i] = new WiFiClient(newClient);  // create new client object to use
                inbound_clientGood = true;
                active_client_idx = i;
                Log.printf("Remote tcp client %d connected\n", i+1);
                snprintf(snprintf_buf, snp_max, "Client %d connected", i+1);        
                LogScreenPrintln(snprintf_buf);               
                break;
              }
            }
          }

          for (int i=0 ; i < max_clients ; ++i) {      // check to see if any clients have disconnected
            if (NULL != tcp_client[i]) {      
                if (!tcp_client[i]) {
                  nbdelay(100);  
                  tcp_client[i]->stop();
                  tcp_client[i] = NULL;
                  nbdelay(100);  
                  inbound_clientGood = false; 
                  Log.println("TCP client disconnected");
                  LogScreenPrintln("TCP client discnct");           
                }
                break;
              }
          }
        }
      }
    } 
   }      

   //===================     H a n d l e   S T A   D i s c o n n e c t  /  R e c o n n e c t
  
   void CheckStaLinkStatus() {

      if ( !WiFi.isConnected() )   { 
        if (!wifiDisconnected) StartWiFiTimer();   // start wifi retry interrupt timer 
        wifiDisconnected = true;
 
        #if (defined ESP32)
          if (xSemaphoreTake(wifiTimerSemaphore, 0) == pdTRUE)  { 
            uint32_t isrCount = 0, isrTime = 0;   
            portENTER_CRITICAL(&timerMux);
           //  do something here if necessary   
            portEXIT_CRITICAL(&timerMux);
            RestartWiFiSta();
          } 
        #endif  

        #if (defined ESP8266)
          if ( (esp8266_wifi_retry_millis > 0) && ( (millis() - esp8266_wifi_retry_millis) > 5000) ) {
            RestartWiFiSta();
            esp8266_wifi_retry_millis = millis();  // restart timer
          }
        #endif           

      } else {
        if (wifiDisconnected) {
          Log.println("Wifi link restored");
          LogScreenPrintln("Wifi link restored"); 
          wifiDisconnected = false;
          
          #if (defined ESP32) 
            if (timer) {
              timerEnd(timer);
            }  
          #endif
          #if (defined ESP8266)        
            esp8266_wifi_retry_millis = 0;  // stop timer
          #endif    
        }
       }        
   }
   //==================================================  
  
   void StartWiFiTimer() {
    #if (defined ESP32)
      timerAlarmEnable(timer);
    #endif

    #if (defined ESP8266)
      esp8266_wifi_retry_millis = millis();
    #endif
    
   }
   //==================================================
   void RestartWiFiSta() { 
     Log.println("WiFi link lost - retrying");
     LogScreenPrintln("Wifilink lost- retry"); 
     WiFi.disconnect(false); 
     if (set.wfmode = ap_sta) {
       WiFi.mode(WIFI_AP_STA);
     } else {
       WiFi.mode(WIFI_STA);
     }

     WiFi.begin(set.staSSID, set.staPw);
     nbdelay(250);
   }
   //=================================================================================================
   #if (defined ESP32)     
   void IRAM_ATTR onTimer(){         // interrupt for periodic wifi retry 

     portENTER_CRITICAL_ISR(&timerMux);
     // do something here if needed 
     portEXIT_CRITICAL_ISR(&timerMux);
     // Give a semaphore that we can check in the loop
     xSemaphoreGiveFromISR(wifiTimerSemaphore, NULL);  

   }
   #endif
 
   //==================================================
   void SetupWiFi() { 
    
    for (int i = 0 ; i < max_clients ; i++) {   // initialise udp remote ip table
         udpremoteip[i][0] = 0;   
         udpremoteip[i][1] = 0; 
         udpremoteip[i][2] = 0;   
         udpremoteip[i][3] = 0;         
    }

    udpremoteip[0][3] = 255;              // initialise the only outbound udp remote client ip for broadcast
    udpremoteip[1][3] = 255;              // initialise the first inbound udp remote client ip for broadcast
   
    bool apFailover = false;            // used when STA fails to connect
    
    #if (defined ESP32)
      // set up wifi retry interrupt 
      wifiTimerSemaphore = xSemaphoreCreateBinary();  // Create wifi retry semaphore
      timer = timerBegin(3, 80, true);                // use timer 0 (0 thru 3) set 80 divider for prescaler
      timerAttachInterrupt(timer, &onTimer, true);
      timerAlarmWrite(timer, 5 * 1E6, true);          // uS, repeat  (semaphore every 5 seconds when alarm enabled)
    #endif

    #if (defined ESP8266)             // ESP8266 - not using interrup times, just use millis()
      esp8266_wifi_retry_millis = 0;
    #endif
    
    apFailover = byte(EEPROM.read(0));              //  Read first eeprom byte
    #if defined Debug_Eeprom
      Log.print("Read EEPROM apFailover = "); Log.println(apFailover); 
    #endif

    //=====================================  S T A T I O N ========================================== 

   if ( (set.fc_io == fc_wifi) && ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt)) ) {  // if user selects wifi fc uplink AND wifi gcs downlink simultaneously, then wifi mode must be ap_sta
     if (set.wfmode != ap_sta) {
       Log.println("User selected wifi fc uplink AND wifi gcs downlink simultaneously. Switching to AP_STA mode");  
       set.wfmode = ap_sta;    
     }
   }
   
   if ((set.wfmode == sta) || (set.wfmode == ap_sta) || (set.wfmode = sta_ap) )  {  // STA mode or AP_STA mode or STA failover to AP mode
     if (!apFailover) {   
     
      uint8_t retry = 0;
      WiFi.disconnect(true);   // To circumvent "wifi: Set status to INIT" error bug
      nbdelay(500);
      
      if (set.wfmode == ap_sta) {
        if (WiFi.mode(WIFI_AP_STA)) {
           Log.println("WiFi mode set to AP_STA sucessfully"); 
     //      Start_Access_Point(); 
        } else {
          Log.println("WiFi mode set to AP_STA failed!");  
        }
      } else {
        if (WiFi.mode(WIFI_STA)) {
           Log.println("WiFi mode set to STA sucessfully");  
        } else {
          Log.println("WiFi mode set to STA failed!");  
        }
      }
      
      Log.print("Trying to connect to ");  
      Log.print(set.staSSID); 
      LogScreenPrintln("WiFi trying ..");    
      nbdelay(500);
      
      WiFi.begin(set.staSSID, set.staPw);
      while (WiFi.status() != WL_CONNECTED){
        retry++;
        if (retry > 20) {
          Log.println();
          Log.println("Failed to connect in STA mode");
          LogScreenPrintln("No connect STA Mode");
          if (set.wfmode == sta_ap) {  // STA failover to AP mode
            apFailover = true;         
            Log.println("Failover to AP. Rebooting ....");
            LogScreenPrintln("Failover to AP");  
            apFailover = 1;                // set STA failover to AP flag
            EEPROM.write(0, apFailover);   // (addr, val)  
            EEPROM.commit();
            #if defined Debug_Eeprom
              Log.print("Write EEPROM apFailover = "); Log.println(apFailover); 
            #endif          
            delay(1000);
            ESP.restart();                 // esp32 and esp8266
          }  
          
          break;
        }
        nbdelay(500);
        Log.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {

        if (set.wfmode == sta_ap) {   // in sta failover to ap mode we had successful sta connect
          set.wfmode = sta;           // so set correct mode      
        }
        
        localIP = WiFi.localIP();  // TCP and UDP
   
        UDP_remoteIP = localIP;    // Initially broadcast on the subnet we are attached to. patch by Stefan Arbes. 
        UDP_remoteIP[3] = 255;     // patch by Stefan Arbes  
                               
        Log.println();
        Log.println("WiFi connected!");
        Log.print("Local IP address: ");
        Log.print(localIP);
        if (set.mav_wfproto == tcp)  {   // TCP
          Log.print("  port: ");
          Log.println(set.tcp_localPort);    //  UDP port is printed lower down
        } else {
          Log.println();
        }
 
        wifi_rssi = WiFi.RSSI();
        Log.print("WiFi RSSI:");
        Log.print(wifi_rssi);
        Log.println(" dBm");

        LogScreenPrintln("Connected!");
        LogScreenPrintln(localIP.toString());

        if (set.mav_wfproto == tcp)  {   // TCP     
          if (set.fc_io == fc_wifi) {  // if we expect wifi from the fc, we are a client, so need a new session
             outbound_clientGood = NewOutboundTCPClient();
          }
   
          TCPserver.begin(set.tcp_localPort);                     //  tcp server socket started
          Log.println("TCP server started");  
          LogScreenPrintln("TCP server started");
        }

        if ( (set.mav_wfproto == udp) || (set.fr_io & 0x02) ) {  // UDP

          if (set.wfmode == ap_sta) {                // in WIFI_AP_STA mode, we need to discriminate between sta and ap read ports 
            udp_read_port = set.udp_remotePort;      // so we flip read and send ports as a device
            udp_send_port = set.udp_localPort;           
          } else {    
            udp_read_port = set.udp_localPort; 
            udp_send_port = set.udp_remotePort;                          
          }
          if (set.mav_wfproto == udp) {
            WiFiUDP UDP_STA_Object;        
            udp_object[0] = new WiFiUDP(UDP_STA_Object);   
            Log.printf("Begin UDP using STA UDP object  read port:%d  send port:%d\n", udp_read_port, udp_send_port);                                                             
            udp_object[0]->begin(udp_read_port);  
          }
          if (set.fr_io & 0x02)  {  // UDP
            Log.printf("Begin UDP using Frs UDP object  read port:%d  send port:%d\n", set.udp_localPort+1, set.udp_remotePort+1);                       
            frs_udp_object.begin(set.udp_localPort+1);          // use local port + 1 for Frs out                   
          }
          UDP_remoteIP = localIP;        
          UDP_remoteIP[3] = 255;           // broadcast until we know which ip to target     

          if ((set.wfmode == ap_sta) || (set.fc_io == fc_wifi)) {      // if wifi both sides OR fc wifi                        
            udpremoteip[0] = UDP_remoteIP;                             // [0] IPs reserved for FC side              
          } else 
          if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) ) {  // if gcs wifi            
            udpremoteip[1] = UDP_remoteIP;                             // [1] IPs reserved for GCS side              
          }    
 
          Log.printf("UDP for STA started, local %s   remote %s\n", localIP.toString().c_str(), 
              UDP_remoteIP.toString().c_str());
          snprintf(snprintf_buf, snp_max, "Local port=%d", set.udp_localPort);     
          LogScreenPrintln(snprintf_buf);
       
        }
        if (set.wfmode == ap_sta) {
          Start_Access_Point(); 
        }
        wifiSuGood = true;
        
      } 
    }  else {   // if apFailover clear apFailover flag
        apFailover = 0;
        EEPROM.write(0, apFailover);   // (addr, val)  
        EEPROM.commit();
        #if defined Debug_Eeprom
          Log.print("Clear EEPROM apFailover = "); Log.println(apFailover); 
        #endif  
      }

   }
 
   //===============================  A C C E S S   P O I N T ===============================
     
   if ((set.wfmode == ap) || (set.wfmode == sta_ap)) { // AP mode or STA failover to AP mode
    if (!wifiSuGood) {  // not already setup in STA above  
    
      Log.printf("WiFi mode set to WIFI_AP %s\n", WiFi.mode(WIFI_AP) ? "" : "Failed!"); 
      
      Start_Access_Point();
      
      if (set.wfmode == sta_ap) {  // in sta_ap mode we had successful failover to ap
        set.wfmode = ap;           // so set correct mode      
      }
      
    }
      
    wifiSuGood = true;  
    
   }        // end of AP ===========================================================================         

   #if defined Debug_SRAM
     Log.printf("==============>Free Heap after WiFi setup = %d\n", ESP.getFreeHeap());
   #endif

   #if defined webSupport
     if (wifiSuGood) {
       WebServerSetup();  
       Log.print("Web support active on http://"); 
       Log.println(localIP.toString().c_str());
       LogScreenPrintln("webSupprt active");  
     }  else {
       Log.println("No web support possible"); 
       LogScreenPrintln("No web support!");  
     }
   #endif

   wifiSuDone = true;
    
  } 
  //=================================================================================================  
  void Start_Access_Point() {
    
      WiFi.softAP(set.apSSID, set.apPw, set.channel);
      
      localIP = WiFi.softAPIP();   // tcp and udp

      Log.print("AP IP address: ");
      Log.print (localIP); 
      snprintf(snprintf_buf, snp_max, "AP IP = %s", localIP.toString().c_str());        
      LogScreenPrintln(snprintf_buf);  
      
      Log.print("  SSID: ");
      Log.println(String(set.apSSID));
      LogScreenPrintln("WiFi AP SSID =");
      snprintf(snprintf_buf, snp_max, "%s", set.apSSID);        
      LogScreenPrintln(snprintf_buf);  
      
      if (set.mav_wfproto == tcp)  {         // TCP
          TCPserver.begin(set.tcp_localPort);   //  Server for TCP/IP traffic     
          Log.printf("TCP/IP started, local IP:port %s:%d\n", localIP.toString().c_str(), set.tcp_localPort);
          snprintf(snprintf_buf, snp_max, "TCP port = %d", set.tcp_localPort);        
          LogScreenPrintln(snprintf_buf);        
        }

        if ( (set.mav_wfproto == udp) || (set.fr_io & 0x02) ) {  // UDP
          WiFiUDP UDP_AP_Object;        
          udp_object[1] = new WiFiUDP(UDP_AP_Object); 
           
          if (set.wfmode == ap_sta) {               // NB in WIFI_AP_STA mode, we need to discriminate between sta and ap read ports 
            udp_read_port = set.udp_localPort;      // so we flip read and send ports as a device
            udp_send_port = set.udp_remotePort;              
          } else {    
            udp_read_port = set.udp_remotePort;  
            udp_send_port = set.udp_localPort;                    
          }
          
          if (set.mav_wfproto == udp) {
            WiFiUDP UDP_STA_Object;        
            udp_object[0] = new WiFiUDP(UDP_STA_Object);         
            Log.printf("Begin UDP using STA UDP object  read port:%d  send port:%d\n", udp_read_port, udp_send_port);                 
            udp_object[0]->begin(udp_read_port);  
          }
        if (set.fr_io & 0x02) {  // UDP  
            Log.printf("Begin UDP using Frs UDP object  read port:%d  send port:%d\n", set.udp_localPort+1, set.udp_remotePort+1);                    
            frs_udp_object.begin(set.udp_localPort+1);          // use local port + 1 for Frs out                   
          }
                  
          UDP_remoteIP = WiFi.softAPIP();
          UDP_remoteIP[3] = 255;           // broadcast until we know which ip to target       

          if (set.wfmode != ap_sta) {      // NOT wifi both sides                   
            if (set.fc_io == fc_wifi)  {   // AND fc wifi   [0] reserved for FC side 
          
              udpremoteip[0] = UDP_remoteIP;
            }                
          }  else {
            Log.printf("Using UDP object 1\n");             
            udp_object[1]->begin(udp_read_port);           
            udpremoteip[1] = UDP_remoteIP;                
          }  
        
          Log.printf("UDP for AP started, local %s   remote %s\n", WiFi.softAPIP().toString().c_str(), 
              UDP_remoteIP.toString().c_str());         
          snprintf(snprintf_buf, snp_max, "UDP port = %d", set.udp_localPort);        
          LogScreenPrintln(snprintf_buf);            
      }      
  }
  
  //=================================================================================================  
  bool NewOutboundTCPClient() {
  static uint8_t retry = 3;

    WiFiClient newClient;        
    while (!newClient.connect(TCP_remoteIP, TCP_remotePort)) {
      Log.printf("Local outbound tcp client connect failed, retrying %d\n", retry);
      retry--;
      if (retry == 0) {
         Log.println("Tcp client connect aborted!");
         return false;
      }
      nbdelay(4000);
    }
    active_client_idx = 0;     // reserve the first tcp client object for our single outbound session   
    tcp_client[0] = new WiFiClient(newClient); 
    Log.print("Local tcp client connected to remote server IP:"); Log.print(TCP_remoteIP);
    Log.print(" remote Port:"); Log.println(TCP_remotePort);

    LogScreenPrintln("Client connected");
    LogScreenPrintln("to remote TCP IP =");
    LogScreenPrintln(TCP_remoteIP.toString()); 
    return true;
  }

  //=================================================================================================  
   void PrintRemoteIP() {
    if (FtRemIP)  {
      FtRemIP = false;
      Log.print("UDP client identified, remote IP: "); Log.print(UDP_remoteIP);
      Log.print(", remote port: "); Log.println(udp_send_port);
      LogScreenPrintln("UDP client connected");
      LogScreenPrintln("Remote IP =");
      snprintf(snprintf_buf, snp_max, "%s", UDP_remoteIP.toString().c_str());        
      LogScreenPrintln(snprintf_buf);        
      snprintf(snprintf_buf, snp_max, "Remote port = %d", set.udp_remotePort);        
      LogScreenPrintln(snprintf_buf);    
     }
  }
  
#endif //end of  wifiBuiltin
//=================================================================================================   
//                             E N D   O F   W I F I   S U P P O R T   -   ESP Only - for now
//=================================================================================================

void ServiceStatusLeds() {
  if (MavStatusLed != 99) {
    ServiceMavStatusLed();
  }
  if (BufStatusLed != 99) {
    ServiceBufStatusLed();
  }
}
void ServiceMavStatusLed() {
  if (mavGood) {

      if (InvertMavLed) {
       MavLedState = LOW;
      } else {
       MavLedState = HIGH;
      }
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

//=================================================================================================  

void Printbyte(byte b, bool LF, char delimiter) {
  if ((b == 0x7E) && (LF)) {
    Log.println();
  }
  
  if (b == 0x7E)  {//             || (b == 0x10)  || (b == 0x32)) {
    Log.println();
  } 
  if (b<=0xf) Log.print("0");
  Log.print(b,HEX);
  Log.write(delimiter);
}

//=================================================================================================  
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
uint16_t mav_checksum;          ///< X.25 crcout
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
    Log.print("mav1: /");

    if (j == 0) {
      Printbyte(bytes[0], 0, '|');   // crcout1
      Printbyte(bytes[1], 0, '|');   // crcout2
      Log.print("/");
      }
    mav_magic = bytes[j+2];   
    mav_len = bytes[j+3];
 //   mav_incompat_flags = bytes[j+4];;
 //   mav_compat_flags = bytes[j+5];;
    mav_seq = bytes[j+6];
 //   mav_sysid = bytes[j+7];
 //   mav_compid = bytes[j+8];
    mav_msgid = bytes[j+9];

    //Log.print(TimeString(millis()/1000)); Log.print(": ");
  
    Log.print("seq="); Log.print(mav_seq); Log.print("\t"); 
    Log.print("len="); Log.print(mav_len); Log.print("\t"); 
    Log.print("/");
    for (int i = (j+2); i < (j+10); i++) {  // Print the header
      Printbyte(bytes[i], 0, '|'); 
    }
    
    Log.print("  ");
    Log.print("#");
    Log.print(mav_msgid);
    if (mav_msgid < 100) Log.print(" ");
    if (mav_msgid < 10)  Log.print(" ");
    Log.print("\t");
    
    tl = (mav_len+10);                // Total length: 8 bytes header + Payload + 2 bytes crcout
 //   for (int i = (j+10); i < (j+tl); i++) {  
    for (int i = (j+10); i <= (tl); i++) {    
     Printbyte(bytes[i], 0, '|');     
    }
    if (j == -2) {
      Log.print("//");
      Printbyte(bytes[mav_len + 8], 0, '|'); 
      Printbyte(bytes[mav_len + 9], 0, '|'); 
      }
    Log.println("//");  
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
    
    Log.print("mav2:  /");
    if (j == 0) {
      Printbyte(bytes[0], 0, '|');   // crcout1
      Printbyte(bytes[1], 0, '|');   // crcout2 
      Log.print("/");
    }
    mav_magic = bytes[2]; 
    mav_len = bytes[3];
//    mav_incompat_flags = bytes[4]; 
  //  mav_compat_flags = bytes[5];
    mav_seq = bytes[6];
//    mav_sysid = bytes[7];
   // mav_compid = bytes[8]; 
    mav_msgid = (bytes[11] << 16) | (bytes[10] << 8) | bytes[9]; 

    //Log.print(TimeString(millis()/1000)); Log.print(": ");

    Log.print("seq="); Log.print(mav_seq); Log.print("\t"); 
    Log.print("len="); Log.print(mav_len); Log.print("\t"); 
    Log.print("/");
    for (int i = (j+2); i < (j+12); i++) {  // Print the header
     Printbyte(bytes[i], 0, '|'); 
    }

    Log.print("  ");
    Log.print("#");
    Log.print(mav_msgid);
    if (mav_msgid < 100) Log.print(" ");
    if (mav_msgid < 10)  Log.print(" ");
    Log.print("\t");

 //   tl = (mav_len+27);                // Total length: 10 bytes header + Payload + 2B crcout +15 bytes signature
    tl = (mav_len+22);                  // This works, the above does not!
    for (int i = (j+12); i < (tl+j); i++) {   
       if (i == (mav_len + 12)) {
        Log.print("/");
      }
      if (i == (mav_len + 12 + 2+j)) {
        Log.print("/");
      }
      Printbyte(bytes[i], 0, '|'); 
    }
    Log.println();
  }

   Log.print("Raw: ");
   for (int i = 0; i < 40; i++) {  //  unformatted
      Printbyte(bytes[i], 0, '|'); 
    }
   Log.println();
  
}
//=================================================================================================  
float RadToDeg(float _Rad) {
  return _Rad * 180 / PI;  
}
//=================================================================================================  
float DegToRad(float _Deg) {
  return _Deg * PI / 180;  
}

//=================================================================================================  
uint32_t TenToPwr(uint8_t pwr) {
  uint32_t ttp = 1;
  for (int i = 1 ; i<=pwr ; i++) {
    ttp*=10;
  }
  return ttp;
} 
//================================================================================================= 
 
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
//=================================================================================================  
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

//=================================================================================================  
void PrintPeriod(bool LF) {
  now_millis=millis();
  now_micros=micros();

  uint32_t period = now_millis - prev_millis;
  if (period < 10) {
    period = now_micros - prev_micros;
    Log.printf(" Period uS=%d", period);
  } else {
    Log.printf(" Period mS=%d", period);
  }

  if (LF) {
    Log.print("\t\n");
  } else {
   Log.print("\t");
  }
    
  prev_millis=now_millis;
  prev_micros=now_micros;
}

//=================================================================================================  
void PrintLoopPeriod() {
  now_millis=millis();
  now_micros=micros();

  uint32_t period = now_millis - prev_lp_millis;
  if (period < 10) {
    period = now_micros - prev_lp_micros;
    Log.printf("Loop Period uS=%d\n", period);
  } else {
    Log.printf("Loop Period mS=%d\n", period);
    if (period > 5000) Log.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  }
    
  prev_lp_millis=now_millis;
  prev_lp_micros=now_micros;
}

//=================================================================================================   
//                           D I S P L A Y   S U P P O R T   -   ESP Only - for now
//================================================================================================= 

  #if defined displaySupport  
    void HandleDisplayButtons() {

      if (Pinfo != 99)  {   // if digital pin for info display enumerated
        infoButton = !digitalRead(Pinfo);  // low == pressed
      }
        
      if ((infoButton) && (!infoPressBusy)) { 
        infoPressBusy = true; 
        infoNewPress = true;          
        info_debounce_millis = millis();   

        info_millis = millis();                       
        if (show_log) {
          show_log = false; 
          info_millis = millis() + db_period;  
        } else {
          show_log = true;    
        }
      }
        
      if(millis() - info_debounce_millis > db_period) { 
        infoPressBusy = false; 
        infoButton = false; // for slow decay touch buttons
      }

      if (millis() - last_log_millis > 15000) { // after 15 seconds default to flight info screen
        last_log_millis = millis();             // and enable toggle button again
        show_log = false;
      }

      
      if (show_log) {
        if (infoNewPress) {     
          PaintLogScreen(row, show_last_row);  // one time     
          infoNewPress = false; 
          last_log_millis = millis();
        }
      } else {            // else show flight info
        DisplayFlightInfo();             
      }
      
      //Log.printf("busy=%d  new=%d log=%d  bounce=%d  info=%d\n", infoPressBusy, infoNewPress, show_log, info_debounce_millis, info_millis); 
      
     #if ((defined ESP32) || (defined ESP8266))   // Teensy does not have touch pins          
      if ( (Tup != 99) && (Tdn != 99) ) {         // if ESP touch pin-pair enumerated
        if (upButton) {
          Scroll_Display(up);
        }
        if (dnButton) {
          Scroll_Display(down);
        }     
      } else
      #endif
      
      if ( (Pup != 99) && (Pdn != 99) ) {   // if digital pin-pair enumerated
        upButton = !digitalRead(Pup);       // low == pressed
        if (upButton) {                 
          Scroll_Display(up);
        }
        dnButton = !digitalRead(Pdn);
        if (dnButton) {
          Scroll_Display(down);
        }        
      }        
    }

    //===================================
    #if ((defined ESP32) || (defined ESP8266)) 
    void IRAM_ATTR gotButtonUp(){
      upButton = true;
    }

    void IRAM_ATTR gotButtonDn(){
      dnButton = true;  
    }
    
    void IRAM_ATTR gotButtonInfo(){
      infoButton = true;
    }
    #endif 
    //===================================
   
    void Scroll_Display(scroll_t up_dn) {
      
      if (millis() - scroll_millis < 300) return;
      show_log = true;    
      scroll_millis = millis(); 
      
      if (up_dn == up) {
         scroll_row--;
         scroll_row = constrain(scroll_row, SCR_H_CH, row);
         upButton = false; 
         PaintLogScreen(scroll_row, show_last_row);   // paint down to scroll_row
      }
      if (up_dn == down) {
          scroll_row++; 
          scroll_row = constrain(scroll_row, SCR_H_CH, row);       
          dnButton = false; 
          PaintLogScreen(scroll_row, show_last_row);  // paint down to scroll_row      
      }   
    }

    //===================================
    void PaintLogScreen(uint8_t new_row, last_row_t last_row_action) { 
      if (display_mode != logg) { 
          SetupLogDisplayStyle();
          display_mode = logg; 
      }  
     
        #if (defined ST7789_Display) || (defined SSD1331_Display) ||  (defined ILI9341_Display)   
        //  hardware SPI pins defined in config.h 
          display.fillScreen(SCR_BACKGROUND);                 
        #elif (defined SSD1306_Display) 
          display.clearDisplay();
        #endif  
        display.setCursor(0,0);  
        int8_t first_row = (last_row_action==omit_last_row) ? (new_row - SCR_H_CH +1) : (new_row - SCR_H_CH); 
        int8_t last_row = (last_row_action==omit_last_row) ? new_row : (new_row );        
        for (int i = first_row ; i < last_row; i++) { // drop first line, display rest of old lines & leave space for new line          
          display.println(ScreenRow[i].x);
        }
   
        #if (defined SSD1306_Display)
          display.display();
        #endif 
    }
    
  #endif  // end of defined displaySupport
    
    //===================================
    void LogScreenPrintln(String S) {
    #if defined displaySupport   
    
      if (display_mode != logg) {
          SetupLogDisplayStyle();
          display_mode = logg; 
      }   
      if (row >= SCR_H_CH) {                 // if the new line exceeds the page lth, re-display existing lines
        PaintLogScreen(row, omit_last_row);
      }
      uint16_t lth = strlen(S.c_str());           // store the new line a char at a time
      if (lth > max_col-1) {
        Log.printf("Display width of %d exceeded for |%s|\n", SCR_W_CH, S.c_str());  // SCR_W_CH = max_col-1
        lth = max_col-1;  // prevent array overflow
      }

      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
      } 

      for (col=col ; col < max_col; col++) {    //  padd out the new line to eol
        ScreenRow[row].x[col] = '\0';
      } 

      display.println(ScreenRow[row].x);        // display the new line, which is always the last line
      #if (defined SSD1306_Display)
        display.display();
      #endif  

      col = 0;
      row++;
      if (row > max_row-1) {
        Log.println("Display rows exceeded!");
        row = max_row-1;  // prevent array overflow
      }
      last_log_millis = millis();             // and enable toggle button again
      show_log = true;          
    #endif       
    } // ready for next line

    //===================================
   
    void LogScreenPrint(String S) {
    #if defined displaySupport  

      if (display_mode != logg) {
          SetupLogDisplayStyle();
          display_mode = logg; 
      }   

     // scroll_row = row; 
      if (row >= SCR_H_CH) {              // if the new line exceeds the page lth, re-display existing lines
        PaintLogScreen(row, omit_last_row);
      }
      display.print(S);                         // the new line
      #if (defined SSD1306_Display)
        display.display();
      #endif 
       
      uint8_t lth = strlen(S.c_str());          // store the line a char at a time
      if (lth > SCR_W_CH) {
        Log.printf("Display width of %d exceeded for |%s|\n", SCR_W_CH, S.c_str());  // SCR_W_CH = max_col-1
        lth = max_col-1;  // prevent array overflow
      }  

      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
      } 
      for (col=col ; col < max_col; col++) {  //  padd out to eol
        ScreenRow[row].x[col] = '\0';
      }
      
      if (col > max_col-1) {   // only if columns exceeded, increment row
        col = 0;
        row++;
      }
      last_log_millis = millis();             // and enable toggle button again
      show_log = true;
    #endif    
    } // ready for next line
    
    //===================================
    #if defined displaySupport  
    
    void DisplayFlightInfo() {
      uint16_t xx, yy; 
      if (display_mode != flight_info) {
          SetupInfoDisplayStyle();
          display_mode = flight_info; 
      }
      

      #if  (defined ILI9341_Display)


        if (millis() - info_millis > 200) {    // refresh rate
          info_millis = millis();  

          // artificial horizon
          draw_horizon(ap_roll, ap_pitch, SCR_W_PX, SCR_H_PX);

          display.setTextSize(2);    // 26 ch wide x 15 ch deep
          
          // sats visible
          xx = 0;
          yy = 0 ;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "Sats:%d", ap24_sat_visible); 
          display.fillRect(xx +(5*CHAR_W_PX), yy, 2 * CHAR_W_PX, CHAR_H_PX, ILI9341_BLUE); // clear the previous line               
          display.println(snprintf_buf);  

          // heading (yaw)
          xx = 9 * CHAR_W_PX;
          yy = 0 ;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "Hdg:%.0f%", cur.hdg);
          display.fillRect(xx+(4*CHAR_W_PX), yy, 4 * CHAR_W_PX, CHAR_H_PX, ILI9341_BLUE); // clear the previous line                                
          display.println(snprintf_buf);

          // Radio RSSI
          xx = 17 * CHAR_W_PX;
          yy = 0 ;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "RSSI:%ld%%", ap_rssi); 
          display.fillRect(xx+(4*CHAR_W_PX), yy, 4 * CHAR_W_PX, CHAR_H_PX, ILI9341_BLUE); // clear the previous line               
          display.println(snprintf_buf);
              
          // distance to home
          xx = 0;
          yy = 13.5 * CHAR_H_PX;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Home:%d", pt_home_dist);    // m 
          display.fillRect(xx+(5*CHAR_W_PX), yy, (4*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 

          // arrow to home
          xx = 14 * CHAR_W_PX;
          yy = 13.5 * CHAR_H_PX;   
          draw_home_arrow(xx, yy, pt_home_angle, SCR_W_PX, SCR_H_PX);
   
          // altitude above home
          xx = 18 * CHAR_W_PX;
          yy = 14 * CHAR_W_PX;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Alt:%d", cur.alt / 1000);    // mm => m 
          display.fillRect(xx+(4*CHAR_W_PX), yy, (4*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 

          // voltage
          xx = 0;
          yy = 16 * CHAR_W_PX;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "V:%.1fV", ap_voltage_battery1 * 0.1F);     
          display.fillRect(xx+(2*CHAR_W_PX), yy, (6*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 
          
          // current
          xx = 9 * CHAR_W_PX;
          yy = 16 * CHAR_W_PX;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "A:%.0f", ap_current_battery1* 0.1F);     
          display.fillRect(xx+(2*CHAR_W_PX), yy, (6*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 
          
          // Ah consumed
          xx = 18 * CHAR_W_PX;
          yy = 16 * CHAR_W_PX;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Ah:%.1f", )pt_bat1_mAh 0.001F);     
          display.fillRect(xx+(3*CHAR_W_PX), yy, (5*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf);           
          
          // latitude and logitude
          display.setTextSize(1);          
          xx = 0;
          yy = 18 * CHAR_H_PX;
          display.setCursor(xx,yy);       
          snprintf(snprintf_buf, snp_max, "Lat:%.7f", cur.lat);
          display.fillRect(xx, yy, (15*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line        
          display.println(snprintf_buf);  
          xx = 18 * CHAR_W_PX;   
          yy = 18 * CHAR_H_PX;  
          display.setCursor(xx, yy);    
          snprintf(snprintf_buf, snp_max, "Lon:%.7f", cur.lon);
          display.fillRect(xx, yy, 21 * CHAR_W_PX, CHAR_H_PX, ILI9341_BLUE); // clear the previous line            
          display.println(snprintf_buf);  
          display.setTextSize(2);    // 26 ch wide x 15 ch deep
          display_mode = flight_info;
        }
      #else
        if (millis() - info_millis > 2000) {    // refresh rate
          info_millis = millis();  
          // Latitude
          xx = 0;
          yy = 0;
          display.setCursor(xx,yy);       
          snprintf(snprintf_buf, snp_max, "Lat %.7f", cur.lat);
          display.fillRect(xx+(4*CHAR_W_PX), yy, 11 * CHAR_W_PX, CHAR_H_PX, SCR_BACKGROUND); // clear the previous data           
          display.println(snprintf_buf);  

          // Longitude
          xx = 0;
          yy = 1.8 * CHAR_H_PX;    
          display.setCursor(xx, yy);                 
          snprintf(snprintf_buf, snp_max, "Lon %.7f", cur.lon);
          display.fillRect(xx+(4*CHAR_W_PX), yy, 11 * CHAR_W_PX, CHAR_H_PX, SCR_BACKGROUND);        
          display.println(snprintf_buf); 

          // Volts, Amps and Ah 
          xx = 0;
          yy = 3.6 * CHAR_H_PX;      
          display.setCursor(xx, yy);               
          snprintf(snprintf_buf, snp_max, "%.1fV %.0fA %.1fAh", pt_bat1_volts * 0.1F, pt_bat1_amps * 0.1F, pt_bat1_mAh * 0.001F);     
          display.fillRect(xx, yy, SCR_W_PX, CHAR_H_PX, SCR_BACKGROUND); // clear the whole line  
          display.println(snprintf_buf); 

          // Number of Sats and RSSI
          xx = 0;
          yy = 5.4 * CHAR_H_PX;      
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Sats %d RSSI %ld%%", ap24_sat_visible, pt_rssi); 
          display.fillRect(xx+(5*CHAR_W_PX), yy, (3 * CHAR_W_PX), CHAR_H_PX, SCR_BACKGROUND);  
          display.fillRect(xx+(12*CHAR_W_PX), yy, (4 * CHAR_W_PX), CHAR_H_PX, SCR_BACKGROUND);   // blank rssi  
          display.println(snprintf_buf);       

           
          #if (defined SSD1306_Display)
            display.display();
          #endif 
  
        }
      #endif    
    } 
    #endif    
    
    //===================================
    #if defined displaySupport  
    
    void SetupLogDisplayStyle() {
       
      #if (defined ST7789_Display)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
        #if (SCR_ORIENT == 0)           // portrait
          display.setRotation(0);       // or 4 
          display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6 
        #elif (SCR_ORIENT == 1)         // landscape
          display.setRotation(3);       // or 1 
          display.setTextFont(1);        
        #endif   
         
      display.setTextSize(TEXT_SIZE);
      display.fillScreen(SCR_BACKGROUND);
      display.setTextColor(TFT_SKYBLUE);    
            
      //display.setTextColor(TFT_WHITE);
      //display.setTextColor(TFT_BLUE);  
      //display.setTextColor(TFT_GREEN, TFT_BLACK);
    
      #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display
        display.clearDisplay(); 
        display.setTextColor(WHITE);  
        display.setTextSize(TEXT_SIZE);  
 
      #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display
        //  software SPI pins defined in config.h 
        display.fillScreen(BLACK);
        display.setCursor(0,0);
        display.setTextSize(TEXT_SIZE);
        #define SCR_BACKGROUND BLACK  
        
      #elif (defined ILI9341_Display)           // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
        //  hardware SPI pins defined in config.h 
        display.fillScreen(ILI9341_BLUE);    
        display.setCursor(0,0);
        display.setTextSize(TEXT_SIZE);    // setup in config.h  
        #if (SCR_ORIENT == 0)              // portrait
          display.setRotation(2);          // portrait pins at the top rotation      
        #elif (SCR_ORIENT == 1)            // landscape
          display.setRotation(3);          // landscape pins on the left    
        #endif 
        #define SCR_BACKGROUND ILI9341_BLUE 
      #endif

    }
   #endif
    //===================================
    #if defined displaySupport  
    
    void SetupInfoDisplayStyle() {
    
      #if (defined ST7789_Display)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
        #if (SCR_ORIENT == 0)           // portrait
          display.setRotation(0);       // or 4 
          display.setTextSize(1);
          display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6 
        #elif (SCR_ORIENT == 1)         // landscape
          display.setRotation(3);       // or 1
          display.setTextSize(2);  
          display.setTextFont(1);        
        #endif    
      
      display.fillScreen(SCR_BACKGROUND);
      display.setTextColor(TFT_SKYBLUE);    
            
      //display.setTextColor(TFT_WHITE);
      //display.setTextColor(TFT_BLUE);  
      //display.setTextColor(TFT_GREEN, TFT_BLACK);
    
    #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display
      display.clearDisplay(); 
      display.setTextColor(WHITE);  
      display.setTextSize(1);  
 
    #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display
      //  SPI pins defined in config.h 
      display.fillScreen(BLACK);
      display.setTextColor(WHITE);  
      display.setTextSize(1);
      #define SCR_BACKGROUND BLACK  
      
    #elif (defined ILI9341_Display)            // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
      //  SPI pins defined in config.h 
      display.fillScreen(ILI9341_BLUE);
      display.setRotation(3);          // landscape pins on the left  
      display.setTextSize(2);     
      display.setCursor(0,0);
      #define SCR_BACKGROUND ILI9341_BLUE      
    #endif
   }
   #endif    

//=================================================================================================   
//                             S D   C A R D   S U P P O R T   -   ESP32 Only - for now
//================================================================================================= 

#if ((defined ESP32) || (defined ESP8266)) && (defined sdBuiltin)

  void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Log.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Log.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Log.println("Not a directory");
        return;
    }
    
    File file = root.openNextFile();
    
    int i = 0;  
    while(file){
      if(file.isDirectory()){    
        Log.print("  DIR : ");
        Log.println(file.name());
        if(levels){
          listDir(fs, file.name(), levels -1);   //  Recursive :)
          }
      } else { 
        std::string myStr (file.name());  
        if (myStr.compare(0,14,"/System Volume") != 0)  {
      
          fnPath[i] = myStr;
        
          Log.print("  FILE: "); Log.print(i); Log.print(" ");
          Log.print(file.name());
          Log.print("  SIZE: ");
          Log.println(file.size());
          i++;    
        }
      }
      file = root.openNextFile();
    }
    fnCnt = i-1;
  }

  void writeFile(fs::FS &fs, const char * path, const char * message){
    Log.printf("Initialising file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Log.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Log.println("File initialised");
    } else {
        Log.println("Write failed");
    }
    file.close();
  }

  void deleteFile(fs::FS &fs, const char * path){
    Log.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Log.println("File deleted");
    } else {
        Log.println("Delete failed");
    }
  }

  //=================================================================================================  

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
  //=================================================================================================  
  bool Leap_yr(uint16_t y) {
  return ((1970+y)>0) && !((1970+y)%4) && ( ((1970+y)%100) || !((1970+y)%400) );  
  }
  //=================================================================================================  

  String DateTimeString (DateTime_t &ep){
  String S = "";
  ep.yr += 1970;
  S += String(ep.yr);
  S += "-";
  if (ep.mth<10) S += "0"; 
  S += String(ep.mth);
  S += "-";
  if (ep.day<10) S += "0"; 
  S += String(ep.day);
  S += " ";
  if (ep.hh<10) S += "0";
  S += String(ep.hh);
  S += ".";
  if (ep.mm<10) S += "0";
  S += String(ep.mm);
  S += ".";  
  if (ep.ss<10) S += "0";
  S += String(ep.ss);
  return S;
  }

   //=================================================================================================  

   void OpenSDForWrite() {
  
  //  deleteFile(SD, "/mav2passthu.tlog");
  
    uint32_t time_unix_sec = (ap_time_unix_usec/1E6) + (Time_Zone * 3600);   // add time zone adjustment decs
    if (daylightSaving) ap_time_unix_usec -= 3600;   // deduct an hour
    decomposeEpoch(time_unix_sec, dt_tm);
    String sPath;
    if  (set.gs_sd == gs_on){   
      sPath = "/Mav2PT "  + DateTimeString(dt_tm) + ".tlog";
      LogScreenPrintln("Writing Tlog");
    } else
    if  (set.sport_sd == spsd_on) {
      sPath = "/Mav2PT "  + DateTimeString(dt_tm) + ".splog"; 
      LogScreenPrintln("Writing SPlog");  
    }
  //  Log.print("Path: "); Log.println(sPath); 

    strcpy(cPath, sPath.c_str());
    writeFile(SD, cPath , "Mavlink to FrSky Passthru by zs6buj");
    sdStatus = 3;      
   }

#endif      //  ESP Only - for now   
   //================================================================================================= 
   
   void SP_Byte_To_SD(byte SD_Byte) {  // FPort to SD
    
    #if ((defined ESP32) || (defined ESP8266)) && (defined sdBuiltin) 
    
    //if (sp_msg_id > 0xff) return;   // lets just write MavLite for testing
    if  (set.sport_sd == spsd_on) {   // Sport to SD Card
      
    if (sd_idx < sd_buf_sz) {  // buffering
      sd_buf[sd_idx] = SD_Byte;
      sd_idx++; 
      return; 
    }
      
      if (sdStatus == 3) {     //  if open for write
          File file = SD.open(cPath, FILE_APPEND);
          if(!file){
             Log.println("Failed to open file for appending");
             sdStatus = 9;
             return;
            }
            
          if(file.write(sd_buf, sd_buf_sz)){   // write contents of buffer to SD
            } else {
            Log.println("Append failed");
           }          
          if(file.print(SD_Byte)){
          } else {
            Log.println("SD Write failed");
          }
          
          sd_idx = 0;
          file.close();
        
          #ifdef  Debug_SD
            for (int i = 0 ; i < sd_buf_sz ; i++) {
              Printbyte(sd_buf[i], false, '>');
              if ((i>0) && (i%40 == 0)) Log.println();
            }
           Log.println("|");
          #endif  
          return;      
        }
        return;  
      }   
    #endif
    return;   
    }

//=================================================================================================   
//                 E N D   O F   S D   C A R D   S U P P O R T   -   ESP Only - for now
//================================================================================================= 

//=================================================================================================
uint32_t Get_Volt_Average1(uint16_t mV)  {

  if (bat1.avg_mV < 1) bat1.avg_mV = mV;  // Initialise first time

 // bat1.avg_mV = (bat1.avg_mV * 0.9) + (mV * 0.1);  // moving average
  bat1.avg_mV = (bat1.avg_mV * 0.6666) + (mV * 0.3333);  // moving average
  Accum_Volts1(mV);  
  return bat1.avg_mV;
}
//=================================================================================================  
uint32_t Get_Current_Average1(uint16_t cA)  {   // in 100*milliamperes (1 = 100 milliampere)
  
  Accum_mAh1(cA);  
  
  if (bat1.avg_cA < 1){
    bat1.avg_cA = cA;  // Initialise first time
  }

  bat1.avg_cA = (bat1.avg_cA * 0.6666F) + (cA * 0.333F);  // moving average

  return bat1.avg_cA;
  }

void Accum_Volts1(uint32_t mVlt) {    //  mV   milli-Volts
  bat1.tot_volts += (mVlt / 1000);    // Volts
  bat1.samples++;
}

void Accum_mAh1(uint32_t cAs) {        //  cA    100 = 1A
  if (bat1.ft) {
    bat1.prv_millis = millis() -1;   // prevent divide zero
    bat1.ft = false;
  }
  uint32_t period = millis() - bat1.prv_millis;
  bat1.prv_millis = millis();
    
  double hrs = (float)(period / 3600000.0f);  // ms to hours

  bat1.mAh = cAs * hrs;     //  Tiny cAh consumed this tiny period di/dt
 // bat1.mAh *= 100;        //  dA to mA  
  bat1.mAh *= 10;           //  cA to mA 
  bat1.mAh *= 1.0625;       // Emirical adjustment Markus Greinwald 2019/05/21
  bat1.tot_mAh += bat1.mAh;   //   Add them all in
}

float Total_mAh1() {
  return bat1.tot_mAh;
}

float Total_mWh1() {                                     // Total energy consumed bat1
  return bat1.tot_mAh * (bat1.tot_volts / bat1.samples);
}
//=================================================================================================  
uint32_t Get_Volt_Average2(uint16_t mV)  {
  
  if (bat2.avg_mV == 0) bat2.avg_mV = mV;  // Initialise first time

  bat2.avg_mV = (bat2.avg_mV * 0.666) + (mV * 0.333);  // moving average
  Accum_Volts2(mV);  
  return bat2.avg_mV;
}
  
uint32_t Get_Current_Average2(uint16_t cA)  {

  if (bat2.avg_cA == 0) bat2.avg_cA = cA;  // Initialise first time

  bat2.avg_cA = (bat2.avg_cA * 0.666) + (cA * 0.333);  // moving average

  Accum_mAh2(cA);  
  return bat2.avg_cA;
  }

void Accum_Volts2(uint32_t mVlt) {      //  mV   milli-Volts
  bat2.tot_volts += (mVlt * 0.001F);    // Volts
  bat2.samples++;
}

void Accum_mAh2(uint32_t cAs) {        //  cA    100 = 1A
  if (bat2.ft) {
    bat2.prv_millis = millis() -1;   // prevent divide zero
    bat2.ft = false;
  }
  uint32_t period = millis() - bat2.prv_millis;
  bat2.prv_millis = millis();
    
 double hrs = (float)(period / 3600000.0f);  // ms to hours

  bat2.mAh = cAs * hrs;   //  Tiny cAh consumed this tiny period di/dt
 // bat2.mAh *= 100;        //  cA to mA  
  bat2.mAh *= 10;        //  cA to mA ?
  bat2.mAh *= 1.0625;       // Emirical adjustment Markus Greinwald 2019/05/21 
  bat2.tot_mAh += bat2.mAh;   //   Add them all in
}

float Total_mAh2() {
  return bat2.tot_mAh;
}

float Total_mWh2() {                                     // Total energy consumed bat1
  return bat2.tot_mAh * (bat2.tot_volts / bat2.samples);
}

//=================================================================================================  
int8_t PWM_To_63(uint16_t PWM) {       // PWM 1000 to 2000   ->    nominal -63 to 63
int8_t myint;
  myint = round((PWM - 1500) * 0.126); 
  myint = myint < -63 ? -63 : myint;            
  myint = myint > 63 ? 63 : myint;  
  return myint; 
}

//=================================================================================================  
uint32_t Abs(int32_t num) {
  if (num<0) 
    return (num ^ 0xffffffff) + 1;
  else
    return num;  
}
//=================================================================================================  
float Distance(Loc2D loc1, Loc2D loc2) {
float a, c, d, dLat, dLon;  

  loc1.lat=loc1.lat/180*PI;  // degrees to radians
  loc1.lon=loc1.lon/180*PI;
  loc2.lat=loc2.lat/180*PI;
  loc2.lon=loc2.lon/180*PI;
    
  dLat = (loc1.lat-loc2.lat);
  dLon = (loc1.lon-loc2.lon);
  a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(loc2.lat) * cos(loc1.lat); 
  c = 2* asin(sqrt(a));  
  d = 6371000 * c;    
  return d;
}

//=================================================================================================  
//Add two bearing in degrees and correct for 360 boundary
int16_t Add360(int16_t arg1, int16_t arg2) {  
  int16_t ret = arg1 + arg2;
  if (ret < 0) ret += 360;
  if (ret > 359) ret -= 360;
  return ret; 
}
//=================================================================================================  
// Correct for 360 boundary - yaapu
float wrap_360(int16_t angle)
{
    const float ang_360 = 360.f;
    float res = fmodf(static_cast<float>(angle), ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return res;
}
//=================================================================================================  
// From Arducopter 3.5.5 code
uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power)
{
    uint16_t res = 0;
    uint32_t abs_number = abs(number);

   if ((digits == 1) && (power == 1)) { // number encoded on 5 bits: 4 bits for digits + 1 for 10^power
        if (abs_number < 10) {
            res = abs_number<<1;
        } else if (abs_number < 150) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x0F x 10^1 = 150)
            res = 0x1F;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<5;
        }
    } else if ((digits == 2) && (power == 1)) { // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
        if (abs_number < 100) {
            res = abs_number<<1;
        } else if (abs_number < 1270) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x7F x 10^1 = 1270)
            res = 0xFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<8;
        }
    } else if ((digits == 2) && (power == 2)) { // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
        if (abs_number < 100) {
            res = abs_number<<2;
         //   Log.print("abs_number<100  ="); Log.print(abs_number); Log.print(" res="); Log.print(res);
        } else if (abs_number < 1000) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<2)|0x1;
         //   Log.print("abs_number<1000  ="); Log.print(abs_number); Log.print(" res="); Log.print(res);
        } else if (abs_number < 10000) {
            res = ((uint8_t)roundf(abs_number * 0.01f)<<2)|0x2;
          //  Log.print("abs_number<10000  ="); Log.print(abs_number); Log.print(" res="); Log.print(res);
        } else if (abs_number < 127000) {
            res = ((uint8_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x7F x 10^3 = 127000)
            res = 0x1FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<9;
        }
    } else if ((digits == 3) && (power == 1)) { // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<1;
        } else if (abs_number < 10240) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x3FF x 10^1 = 10240)
            res = 0x7FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<11;
        }
    } else if ((digits == 3) && (power == 2)) { // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<2;
        } else if (abs_number < 10000) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<2)|0x1;
        } else if (abs_number < 100000) {
            res = ((uint16_t)roundf(abs_number * 0.01f)<<2)|0x2;
        } else if (abs_number < 1024000) {
            res = ((uint16_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x3FF x 10^3 = 127000)
            res = 0xFFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<12;
        }
    }
    return res;
}  
//=================================================================================================  
  uint32_t bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth) {
  uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
  return r;
}
//=================================================================================================  
// Mask then AND the shifted bits, then OR them to the payload
  void bit32Pack(uint32_t dword ,uint8_t displ, uint8_t lth) {   
  uint32_t dw_and_mask =  (dword<<displ) & (createMask(displ, displ+lth-1)); 
  pt_payload |= dw_and_mask; 
}
//=================================================================================================  
uint32_t createMask(uint8_t lo, uint8_t hi) {
  uint32_t r = 0;
  for (unsigned i=lo; i<=hi; i++)
       r |= 1 << i;  
  return r;
}

//================================================================================================= 
void nbdelay(uint32_t delaymS) { // non-blocking delay
uint32_t start;
  start = millis();
  
  while (millis() - start < delaymS) {     
    yield();
  }
}  
//================================================================================================= 
#if (defined ESP32)
void WiFiEventHandler(WiFiEvent_t event)  {
    Log.printf("[WiFi-event] event: %d ", event);

    switch (event) {
        case SYSTEM_EVENT_WIFI_READY: 
            Log.println("WiFi interface ready");
            break;
        case SYSTEM_EVENT_SCAN_DONE:
            Log.println("Completed scan for access points");
            break;
        case SYSTEM_EVENT_STA_START:
            Log.println("WiFi client started");
            break;
        case SYSTEM_EVENT_STA_STOP:
            Log.println("WiFi client stopped");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            Log.println("Connected to access point");
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Log.println("Disconnected from WiFi access point");
            break;
        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
            Log.println("Authentication mode of access point has changed");
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            Log.print("Obtained IP address: ");
            Log.println(WiFi.localIP());
            break;
        case SYSTEM_EVENT_STA_LOST_IP:
            Log.println("Lost IP address and IP address is reset to 0");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
            Log.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_FAILED:
            Log.println("WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
            Log.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_PIN:
            Log.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case SYSTEM_EVENT_AP_START:
            Log.println("WiFi access point started");
            break;
        case SYSTEM_EVENT_AP_STOP:
            Log.println("WiFi access point  stopped");
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            Log.println("Client connected");
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            Log.println("Client disconnected");
            break;
        case SYSTEM_EVENT_AP_STAIPASSIGNED:
            Log.println("Assigned IP address to client");
            break;
        case SYSTEM_EVENT_AP_PROBEREQRECVED:
            Log.println("Received probe request");
            break;
        case SYSTEM_EVENT_GOT_IP6:
            Log.println("IPv6 is preferred");
            break;
        case SYSTEM_EVENT_ETH_START:
            Log.println("Ethernet started");
            break;
        case SYSTEM_EVENT_ETH_STOP:
            Log.println("Ethernet stopped");
            break;
        case SYSTEM_EVENT_ETH_CONNECTED:
            Log.println("Ethernet connected");
            break;
        case SYSTEM_EVENT_ETH_DISCONNECTED:
            Log.println("Ethernet disconnected");
            break;
        case SYSTEM_EVENT_ETH_GOT_IP:
            Log.println("Obtained IP address");
            break;
        default: break;
    }
}
#endif
/* AVAILABLE EVENTS:
0  SYSTEM_EVENT_WIFI_READY               < ESP32 WiFi ready
1  SYSTEM_EVENT_SCAN_DONE                < ESP32 finish scanning AP
2  SYSTEM_EVENT_STA_START                < ESP32 station start
3  SYSTEM_EVENT_STA_STOP                 < ESP32 station stop
4  SYSTEM_EVENT_STA_CONNECTED            < ESP32 station connected to AP
5  SYSTEM_EVENT_STA_DISCONNECTED         < ESP32 station disconnected from AP
6  SYSTEM_EVENT_STA_AUTHMODE_CHANGE      < the auth mode of AP connected by ESP32 station changed
7  SYSTEM_EVENT_STA_GOT_IP               < ESP32 station got IP from connected AP
8  SYSTEM_EVENT_STA_LOST_IP              < ESP32 station lost IP and the IP is reset to 0
9  SYSTEM_EVENT_STA_WPS_ER_SUCCESS       < ESP32 station wps succeeds in enrollee mode
10 SYSTEM_EVENT_STA_WPS_ER_FAILED        < ESP32 station wps fails in enrollee mode
11 SYSTEM_EVENT_STA_WPS_ER_TIMEOUT       < ESP32 station wps timeout in enrollee mode
12 SYSTEM_EVENT_STA_WPS_ER_PIN           < ESP32 station wps pin code in enrollee mode
13 SYSTEM_EVENT_AP_START                 < ESP32 soft-AP start
14 SYSTEM_EVENT_AP_STOP                  < ESP32 soft-AP stop
15 SYSTEM_EVENT_AP_STACONNECTED          < a station connected to ESP32 soft-AP
16 SYSTEM_EVENT_AP_STADISCONNECTED       < a station disconnected from ESP32 soft-AP
17 SYSTEM_EVENT_AP_STAIPASSIGNED         < ESP32 soft-AP assign an IP to a connected station
18 SYSTEM_EVENT_AP_PROBEREQRECVED        < Receive probe request packet in soft-AP interface
19 SYSTEM_EVENT_GOT_IP6                  < ESP32 station or ap or ethernet interface v6IP addr is preferred
20 SYSTEM_EVENT_ETH_START                < ESP32 ethernet start
21 SYSTEM_EVENT_ETH_STOP                 < ESP32 ethernet stop
22 SYSTEM_EVENT_ETH_CONNECTED            < ESP32 ethernet phy link up
23 SYSTEM_EVENT_ETH_DISCONNECTED         < ESP32 ethernet phy link down
24 SYSTEM_EVENT_ETH_GOT_IP               < ESP32 ethernet got IP from connected AP
25 SYSTEM_EVENT_MAX
*/
//=================================================================================================

    void PrintFrPeriod(bool LF) {
      now_millis=millis();
      now_micros=micros();

      uint32_t period = now_millis - prev_pt_millis;
      if (period < 10) {
        period = now_micros - prev_pt_micros;
        Log.printf(" FrPeriod uS=%d", period);
      } else {
        Log.printf(" FrPeriod mS=%d", period);
      }

      if (LF) {
        Log.print("\t\n");
      } else {
       Log.print("\t");
      }
    
      prev_pt_millis=now_millis;
      prev_pt_micros=now_micros;
    }
    //===================================================================  
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
    //===================================================================
    void Free_Bluetooth_RAM() {
    #if (defined btBuiltin)  
      if (!btDisabled) {  // if not already disabled           
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        esp_bt_mem_release(ESP_BT_MODE_BTDM);
        btActive = false;
        btDisabled = true;
        Log.println("Bluetooth disabled to free up SRAM for web support. PROCEED TO REBOOT for BT to be restored"); 
        LogScreenPrintln("Bluetooth disabled");               
        #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
          Log.printf("==============>Free Heap after bluetooth disabled = %d\n", ESP.getFreeHeap());
        #endif
      }
    #endif
    }
    //=================================================================== 
        String wifiStatusText(uint16_t wifi_status) {
       switch(wifi_status) {  
        case 0:
          return "WL_IDLE_STATUS";  
        case 1:
          return "WL_NO_SSID_AVAIL"; 
        case 2:
          return "WL_SCAN_COMPLETED";                
        case 3:
          return "WL_CONNECTED";                       
        case 4:
          return "WL_CONNECT_FAILED";            
        case 5:
          return "WL_CONNECTION_LOST";       
        case 6:
          return "WL_DISCONNECTED";  
        case 255:
          return "WL_NO_SHIELD";               
        default:
          return "UNKNOWN";     
       }
    }
    //=================================================================== 
    #if  (defined ILI9341_Display)
    uint32_t draw_horizon(float roll, float pitch, int16_t width, int16_t height) {
      int16_t x0, y0, x1, y1, xc, yc, ycp, lth, tick_lean;
      static int16_t px0, py0, px1, py1, pycp, ptick_lean; 
      uint8_t tick_height = 5;  
      float roll_slope = 0;      // [-180,180]
      float pitch_offset = 0;     //  [-90,90]
      const float AngleToRad = PI / 180;
      
      xc = width / 2;    // centre point / pivot / origin
      yc = height / 2;   // 

      display.drawLine(0 + (width * 0.2), yc, width - (width * 0.2), yc, ILI9341_WHITE);   // static reference horizon
      display.drawLine(xc, yc+10, xc, yc-10, ILI9341_WHITE);
      
      roll_slope = tan(roll * AngleToRad);             // roll slope 
      pitch_offset = (sin(pitch * AngleToRad)) * yc;   // pitch offset  
      tick_lean = roll_slope * tick_height;

      lth = (xc * 0.8) * cos(roll * AngleToRad);
      x0 = (xc - lth);
      x1 = (xc + lth);

      y0 = yc - ((xc - x0) * roll_slope);
      y1 = yc + ((x1 - xc) * roll_slope);   
  
      y0 += pitch_offset;
      y1 += pitch_offset;
      ycp = yc + pitch_offset;
     
      static bool ft = true;
      if (!ft) {
        display.drawLine(px0, py0, px1, py1, ILI9341_BLUE);   // Erase old horizon line 
        display.drawLine(xc-ptick_lean, pycp+tick_height, xc+ptick_lean, pycp-tick_height, ILI9341_BLUE); 
        display.drawLine(xc-ptick_lean+1, pycp+tick_height, xc+ptick_lean+1, pycp-tick_height, ILI9341_BLUE);
        display.drawLine(px0-ptick_lean+1, py0+tick_height, px0+ptick_lean+1, py0-tick_height, ILI9341_BLUE);
        display.drawLine(px1-ptick_lean+1, py1+tick_height, px1+ptick_lean+1, py1-tick_height, ILI9341_BLUE);        
        display.drawLine(px0+1, py0, px1+1, py1,ILI9341_BLUE);
      }  
      ft = false;
      display.drawLine(x0, y0, x1, y1, ILI9341_WHITE);      // Horizon line over the top
      display.drawLine(xc-tick_lean, ycp+tick_height, xc+tick_lean, ycp-tick_height, ILI9341_WHITE);
      display.drawLine(xc-tick_lean+1, ycp+tick_height, xc+tick_lean+1, ycp-tick_height, ILI9341_WHITE);
      display.drawLine(x0-tick_lean+1, y0+tick_height, x0+tick_lean+1, y0-tick_height, ILI9341_WHITE);
      display.drawLine(x1-tick_lean+1, y1+tick_height, x1+tick_lean+1, y1-tick_height, ILI9341_WHITE); 
      display.drawLine(x0+1, y0, x1+1, y1, ILI9341_WHITE);
      px0 = x0;
      py0 = y0;
      px1 = x1;
      py1 = y1;    
      pycp = ycp;
      ptick_lean = tick_lean;
    
      return micros();   
    }
    #endif
    //=================================================================== 
    #if  (defined ILI9341_Display)
    void draw_home_arrow(int16_t x, int16_t y, int16_t arrow_angle, int16_t width, int16_t height) {
      int16_t x0, y0, x1, y1, x2, y2, x3, y3;
      static int16_t px0, py0, px1, py1, px2, py2, px3, py3;
      int16_t opp, adj, hyp, opp90, adj90, hyp90;
      const int16_t al = 40;  // arrow length  
      static bool ft = true;
      const float AngleToRad = PI / 180;
      
      int16_t home_angle = 0 - arrow_angle - 25;    // direction of rotation
    
      home_angle = (home_angle < 0) ? home_angle + 360 : home_angle;
      home_angle = (home_angle > 360) ? home_angle - 360 : home_angle;   
      //Log.printf("home_angle=%d \n", pt_home_angle);         
      hyp = al / 2;        
      opp = hyp * sin(home_angle * AngleToRad);
      adj = hyp * cos(home_angle * AngleToRad);
      
      hyp90 = al / 5; 
      opp90 = hyp90 * sin((home_angle + 90) * AngleToRad);
      adj90 = hyp90 * cos((home_angle + 90) * AngleToRad); 
      
      x0 = x + adj; 
      y0 = y - opp; 
      
      x1 = x - adj;
      y1 = y + opp;
 
      x2 = x1 + adj90;
      y2 = y1 - opp90;

      x3 = x1 - adj90;
      y3 = y1 + opp90;
           
      if (!ft) {
       //display.drawTriangle(px0, py0, px2, py2, px3, py3, ILI9341_BLUE);   
       display.fillTriangle(px0, py0, px2, py2, px3, py3, ILI9341_BLUE);                 
      }
      ft = false;     

      //display.drawTriangle(x0, y0, x2, y2, x3, y3, ILI9341_RED); 
      display.fillTriangle(x0, y0, x2, y2, x3, y3, ILI9341_RED);         
             
      px0 = x0; py0 = y0; px1 = x1; py1 = y1; px2 = x2; py2 = y2; px3 = x3; py3 = y3;

    }
    #endif
    //===================================================================     

    //================================================================================================= 
//================================================================================================= 
//
//                                      W E B   S U P P O R T  
// 
//================================================================================================= 
//================================================================================================= 
#if defined webSupport

/*
In the ESP32, constant data is automatically stored in FLASH memory and can be accessed directly 
from FLASH memory without first copying it to RAM. So, there is no need to use the PROGMEM keyword
*/

 static const String styleLogin =  // Stored in FLASH not SRAM Heap - see above
    "<style>h1{background:#3498db;color:#fff;border-radius:5px;height:34px;font-family:sans-serif;}"
    "#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
    "input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
    "form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
    ".btn{background:#3498db;color:#fff;cursor:pointer} .big{ width: 1em; height: 1em;}"
    "::placeholder {color: white; opacity: 1; /* Firefox */}"
    "</style>";
   
 static const String styleSettings =
    "<style>"
    "h{color:#fff;font-family:sans-serif;}"
    "h3{background:#3498db;color:#fff;border-radius:5px;height:22px;font-family:sans-serif;}"
    "input{background:#f1f1f1;border:1;margin:8px auto;font-size:14px}"
    "body{background:#3498db;font-family:arial;font-size:10px;color:black}"
    "#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
    "form{background:#fff;max-width:440px;margin:30px auto;padding:30px;border-radius:10px;text-align:left;font-size:16px}"
    ".big{ width: 1em; height: 1em;} .bold {font-weight: bold;}"
    "</style>";

 static const String styleOTA =
    "<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
    "input{background:#f1f1f1;border:0;padding:0}"
    "body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
    "#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
    "#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
    "form{background:#fff;margin:75px auto;padding:30px;text-align:center;max-width:450px;border-radius:10px;}"       
    ".btn{background:#3498db;color:#fff;cursor:pointer; width: 80px;} .big{ width: 1em; height: 1em;}</style>"  
    "<script>function backtoLogin() {window.close(); window.open('/');} </script>";

   
 static const String otaIndex = styleOTA +  
    "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.4.1/jquery.min.js'></script>"
    "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
    "<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
    "<label id='file-input' for='file' class=btn > Choose file...</label><br><br>"
    "<center><input type='submit' onclick='backtoLogin()' class=btn value='Cancel'> &nbsp &nbsp &nbsp &nbsp "  
    "<input type='submit' class=btn value='Update'></center>"
    "<br><br>"
    "<div id='prg' align='left'></div>"
    "<br><left><div id='prgbar'><div id='bar'></div></div><br><br>"
    "<p id='rebootmsg'></p><br><br>"
    "<center><input type='submit' onclick='window.close()' class=btn value='Close'></center></form>"
    "<script>"
    "function sub(obj){"
    "var fileName = obj.value.split('\\\\');"
    "document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
    "};"
    "$('form').submit(function(e){"
    "e.preventDefault();"
    "var form = $('#upload_form')[0];"
    "var data = new FormData(form);"
    "$.ajax({"
    "url: '/update',"
    "type: 'POST',"
    "data: data,"
    "contentType: false,"
    "processData:false,"
    "xhr: function() {"
    "var xhr = new window.XMLHttpRequest();"
    "xhr.upload.addEventListener('progress', function(evt) {"
    "if (evt.lengthComputable) {"
    "var per = evt.loaded / evt.total;"
    "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
    "$('#bar').css('width',Math.round(per*100) + '%');"
    "if (per == 1.0) {document.getElementById('rebootmsg').innerHTML = 'Rebooting .....'}"
    "}"
    "}, false);"
    "return xhr;"
    "},"
    "success:function(d, s) {console.log('success!')},"
    "error: function (a, b, c) {}"
    "});"
    "});"
    "</script>";

  String settingsPage;
  String loginPage;
  char   temp[128]; 
    
  //===========================================================================================
  void WebServerSetup() {
      
            //===========================================================
            //                 HANDLE INCOMING HTML 
            //===========================================================


  server.on("/", handleLoginPage);                          // root
  server.on("/settingsIndex", handleSettingsPage);             
  server.on("/settingsReturnIndex", handleSettingsReturn);  // save settings and reboot
  server.on("/rebootIndex", handleReboot);                  // reboot only
  server.on("/otaIndex", handleOtaPage); 
  
  server.begin();
   
  #if defined Debug_SRAM
    Log.printf("==============>Free Heap after WebServer setup = %d\n", ESP.getFreeHeap());
  #endif
  
  }
  
//=================================================================================================   
//                            Recover WiFi Settings From Flash
//=================================================================================================

void RecoverSettingsFromFlash() {
  
  set.validity_check = EEPROMRead8(1);         // apFailover is in 0

  #if defined Reset_Web_Defaults
    set.validity_check = 0;                    // reset
  #endif

  if (set.validity_check != 0xdc) {            // eeprom does not contain previously stored setings
                                               // so write default settings to eeprom
    Log.println("NOTE! ALL SETTINGS IN EEPROM SET TO COMPILE_TIME DEFAULTS!");                                           
    WriteSettingsToEEPROM();
    }  
      
  ReadSettingsFromEEPROM();                           

}

//=================================================================================
int32_t String_long(String S) {
  const char* c;
  c = S.c_str();
  return strtol(c, NULL, 0);
}   

//=================================================================================
  void ComposeLoginPage() {

    loginPage  =  styleLogin;
    loginPage += "<form name=loginForm>";
    #if (defined frBuiltin) 
      sprintf(temp,  "<h1>%s Login</h1>", set.host);
      loginPage += temp;
    #else
      loginPage += "<h1>Mavlink Switch Login</h1>";  
    #endif
    loginPage += "<br><input type='radio' class='big' name='_nextFn' value='set' checked> Settings &nbsp &nbsp &nbsp";          
    loginPage += "<input type='radio' class='big' name='_nextFn' value='ota' > Update Firmware<br> <br>";
    loginPage += "<input name=userid class=btn placeholder='User ID' size='10' color:#fff;> ";
    loginPage += "<input name=pwd class=btn placeholder=Password type=Password> <br> <br>";
    loginPage += "<input type=submit onclick=check(this.form) class=btn value=Login></form>";
    loginPage += "<script>";
    loginPage += "function check(form) {";
    sprintf(temp, "if(form.userid.value=='admin' && form.pwd.value=='%s')", webPassword);
    loginPage += temp;
    loginPage += "{{if(form._nextFn.value=='ota'){window.close(); window.open('/otaIndex')}}";
    loginPage += "{if(form._nextFn.value=='set'){window.close(); window.open('/settingsIndex')}}}";
    loginPage += "else";
    loginPage += "{alert('Error Password or Username')}";
    loginPage += "}";
    loginPage += "</script>";
  }
  //=================================================================================
  void ComposeSettingsPage() {
   
  settingsPage  = styleSettings;
  #if (defined frBuiltin) 
    settingsPage += "<!DOCTYPE html><html><body><h>Mavlink To Passthru</h><form action='' ";   
    settingsPage += "autocomplete='on'> <center> <b><h3>MavToPassthru Translator Setup</h3> </b></center> <style>text-align:left</style>";
     
    settingsPage += "Translator Mode: &nbsp &nbsp";
    sprintf(temp, "<input type='radio' class='big' name='_trmode' value='Ground' %s> Ground &nbsp &nbsp", set.trmode1);
    settingsPage += temp;
    sprintf(temp, "<input type='radio' class='big' name='_trmode' value='Air' %s> Air &nbsp &nbsp", set.trmode2);
    settingsPage += temp;
    sprintf(temp, "<input type='radio' class='big' name='_trmode' value='Relay' %s> Relay", set.trmode3);
    settingsPage += temp;
    
    settingsPage += "<br>FrSky Port Type: &nbsp ";    
    sprintf(temp, "<input type='radio' class='big' name='_frport' value='FPort1' %s> FPort1 &nbsp ", set.frport1);
    settingsPage += temp;
    sprintf(temp, "<input type='radio' class='big' name='_frport' value='FPort2' %s> FPort2 &nbsp ", set.frport2);
    settingsPage += temp;  
    sprintf(temp, "<input type='radio' class='big' name='_frport' value='SPort' %s> SPort &nbsp ", set.frport3);
    settingsPage += temp;    
    sprintf(temp, "<input type='radio' class='big' name='_frport' value='Auto' %s> Auto <br>", set.frport4);
    settingsPage += temp;    
           
    settingsPage += "Frs Downlink &nbsp &nbsp";  
    sprintf(temp, "<input type='checkbox' class='big' name='_set.fr_io_ser' value='Serial' %s> Serial &nbsp &nbsp", set.fr_io1);
    settingsPage += temp; 
    #if (defined wifiBuiltin)  
      sprintf(temp, "<input type='checkbox' class='big' name='_set.fr_io_udp' value='UDP' %s> UDP &nbsp &nbsp", set.fr_io2);
      settingsPage += temp; 
    #endif  
    #if (defined sdBuiltin)     
      sprintf(temp, "<input type='checkbox' class='big' name='_set.fr_io_sd' value='SD' %s> SD", set.fr_io3);
      settingsPage += temp; 
    #endif  
  #else
    settingsPage += "<!DOCTYPE html><html><body><h>Mavlink Switch</h><form action='' ";   
    settingsPage += "autocomplete='on'> <center> <b><h3>Mavlink Switch Setup</h3> </b></center> <style>text-align:left</style>";  
  #endif    
  settingsPage += "<br>Mav Uplink &nbsp &nbsp";
  sprintf(temp, "<input type='radio' class='big' name='_fc_io' value='Serial' %s> Serial &nbsp &nbsp", set.fc_io0);
  settingsPage += temp;
  #if (defined ESP32) && (defined btBuiltin) 
    sprintf(temp, "<input type='radio' class='big' name='_fc_io' value='BT' %s> BT &nbsp &nbsp", set.fc_io1);
    settingsPage += temp;
  #endif  
  #if (defined ESP32) || (defined ESP8266)    
    sprintf(temp, "<input type='radio' class='big' name='_fc_io' value='WiFi' %s> WiFi &nbsp &nbsp", set.fc_io2);
    settingsPage += temp; 
  #endif  
  #if defined sdBuiltin   
    sprintf(temp, "<input type='radio' class='big' name='_fc_io' value='SD' %s> SD ", set.fc_io3);
    settingsPage += temp;  
  #endif    
  settingsPage += "<br>Mav Downlink &nbsp";
  sprintf(temp, "<input type='radio' class='big' name='_gs_io' value='None' %s> None &nbsp", set.gs_io9);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' onclick=checkserial(this.form) class='big' name='_gs_io' value='Serial' %s> Serial &nbsp", set.gs_io0);
  settingsPage += temp;
  #if (defined ESP32) && (defined btBuiltin) 
    sprintf(temp, "<input type='radio' class='big' name='_gs_io' value='BT' %s> BT &nbsp", set.gs_io1);
    settingsPage += temp;
  #endif  
  #if (defined ESP32) || (defined ESP8266)  
    sprintf(temp, "<input type='radio' class='big' name='_gs_io' value='WiFi' %s> WiFi &nbsp ", set.gs_io2);
    settingsPage += temp; 
  #endif  
  #if (defined ESP32) && (defined btBuiltin) 
    sprintf(temp, "<input type='radio' class='big' name='_gs_io' value='WiFi+BT' %s> WiFi+BT ", set.gs_io3);
    settingsPage += temp; 
  #endif  
  #if defined sdBuiltin   
    settingsPage += " <br> UPLINK SD: &nbsp";
    sprintf(temp, "<input type='radio' class='big' name='_gs_sd' value='OFF' %s> OFF  &nbsp", set.gs_sd0);
    settingsPage += temp; 
    sprintf(temp, "<input type='radio' class='big' name='_gs_sd' value='ON' %s> ON ", set.gs_sd1);
    settingsPage += temp; 
    #if (defined frBuiltin) 
      settingsPage += " &nbsp &nbsp SPortSD: &nbsp";
      sprintf(temp, "<input type='radio' class='big' name='_sport_sd' value='OFF' %s> OFF &nbsp", set.sport_sd0);
      settingsPage += temp; 
      sprintf(temp, "<input type='radio' class='big' name='_sport_sd' value='ON' %s> ON ", set.sport_sd1);
      settingsPage += temp;  
    #endif     
  #endif       
  settingsPage += "<br>WiFi Mode: &nbsp &nbsp ";
  sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='AP' %s> AP &nbsp &nbsp", set.wfmode1);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='STA' %s> STA &nbsp &nbsp", set.wfmode2);
  settingsPage += temp;    
  sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='STA>AP' %s> STA>AP &nbsp <br>", set.wfmode3);
  settingsPage += temp;
  //sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='AP_STA' %s> AP_STA &nbsp <br>", set.wfmode4);
  //settingsPage += temp;
  settingsPage += "WiFi Protocol: &nbsp &nbsp ";
  sprintf(temp, "<input type='radio' class='big' name='_mav_wfproto' value='TCP' %s> TCP &nbsp &nbsp", set.mav_wfproto1);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_mav_wfproto' value='UDP' %s> UDP &nbsp <br>", set.mav_wfproto2);
  settingsPage += temp; 
  sprintf(temp, "Mavlink Baud: <input type='text' name='_baud' value='%d' size='3' minlength='4' required maxlength='6'> <br>", set.baud);
  settingsPage += temp;
  sprintf(temp, "WiFi Channel: <input type='text' name='_channel' value='%d' size='1' maxlength='2'> <br>", set.channel);
  settingsPage += temp;
  sprintf(temp, "AP SSID: <input type='text' name='_apSSID' value='%s' size='30' maxlength='30'> <br>", set.apSSID);
  settingsPage+= temp;
  sprintf(temp, "AP Password: <input type='password' name='_apPw' value='%s' size='20' minlength='8' required> <br>", set.apPw);
  settingsPage += temp;
  sprintf(temp, "STA SSID: <input type='text' name='_staSSID' value='%s' size='30'> <br>", set.staSSID);
  settingsPage += temp;
  sprintf(temp, "STA Password: <input type='password' name='_staPw' value='%s' size='20' minlength='8' required> <br>", set.staPw);
  settingsPage += temp;
  sprintf(temp, "Host Name: <input type='text' name='_host' value='%s' size='20'> <br>", set.host);
  settingsPage += temp;
  sprintf(temp, "TCP Ports: local  <input type='text' name='_tcp_localPort' value='%d' size='2' minlength='2' required maxlength='5'>", set.tcp_localPort);
  settingsPage += temp;
  sprintf(temp, " &nbsp &nbsp remote <input type='text' name='_tcp_remotePort' value='%d' size='2' minlength='2' required maxlength='5'> <br>", set.tcp_remotePort);
  settingsPage += temp;
  sprintf(temp, "UDP Ports: local  <input type='text' name='_udp_localPort' value='%d' size='2' minlength='2' required maxlength='5'> ", set.udp_localPort);
  settingsPage += temp;
  sprintf(temp, " &nbsp &nbsp remote <input type='text' name='_udp_remotePort' value='%d' size='2' minlength='2' required maxlength='5'> <br>", set.udp_remotePort);
  settingsPage += temp;
  settingsPage += "Bluetooth Mode: &nbsp &nbsp ";
  sprintf(temp, "<input type='radio' class='big' name='_btmode' value='Master' %s> Master &nbsp &nbsp &nbsp &nbsp ", set.btmode1);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_btmode' value='Slave' %s> Slave &nbsp &nbsp <br>", set.btmode2);
  settingsPage += temp;
  sprintf(temp, "Slave Name: <input type='text' name='_btConnectToSlave' value='%s' size='22' maxlength='22'>  <br><br><center>", set.btConnectToSlave);
  settingsPage += temp;  
  settingsPage += "<b><input type='submit' onclick='closeWin()' formaction='/rebootIndex' class=btn value='Cancel'> </b>&nbsp &nbsp &nbsp &nbsp";
  settingsPage += "&nbsp &nbsp &nbsp &nbsp<b><input type='submit' formaction='/settingsReturnIndex' class=btn value='Save & Reboot'> </b><br><br>";
  settingsPage += "<p><font size='1' color='black'><strong>";
  settingsPage += pgm_name + ".  Compiled for "; 
  #if defined ESP32 
    settingsPage += "ESP32";
  #elif defined ESP8266
    settingsPage += "ESP8266";
  #endif   
  settingsPage += "</strong></p></center> </form> </body>";
  
  settingsPage += "<script>";
  settingsPage += "function checkserial(form) {";
  settingsPage += "if((form._fc_io.value=='Serial') && (form._fc_io.value=='Serial'))";
  settingsPage += "{alert('You may not select Serial for both FC and GCS. Please de-select one.')}";
  settingsPage += "}";
  settingsPage += "</script>";

  settingsPage += "<script>";
  settingsPage += "var myWindow";
  settingsPage += "function closeWin() {";
  settingsPage += "myWindow.close() }";
  settingsPage += "</script>";
}
//=================================================================================

void ReadSettingsFromEEPROM() {
byte b;
    b = EEPROMRead8(2);     // translator mode
    if (b == 1) {
      set.trmode = ground;
    } else   
    if (b == 2) {
      set.trmode = air;
    } else 
    if (b == 3) {
      set.trmode = relay;
    }
    
    b = EEPROMRead8(164);                         // fr-port type 164
    if (b == 0) {
      set.frport = f_none;
    } else if (b == 1) {
      set.frport = f_port1;
    } else if (b == 2) {
      set.frport = f_port2;      
    }  else if (b == 3) {
      set.frport = s_port;
    }  else if (b == 4) {
      set.frport = f_auto;
    }     
        
    b = EEPROMRead8(165);                         // set.fr_io 165
    if (b == 0) {
      set.fr_io = fr_none;
    } else
    if (b == 1) {
      set.fr_io = fr_ser;
    }  else 
    if (b == 2) {
      set.fr_io = fr_udp;
    }  else 
     if (b == 3) {
      set.fr_io = fr_ser_udp;
    }  else     
     if (b == 4) {
      set.fr_io = fr_sd;
    }  else          
    if (b == 5) {
      set.fr_io = fr_ser_sd;
    }  else   
     if (b == 6) {
      set.fr_io = fr_udp_sd;
    }  else      
    if (b == 7) {
      set.fr_io = fr_ser_udp_sd;
    } 
        
    b = EEPROMRead8(3);     // fc io
    if (b == 0) {
      set.fc_io = fc_ser;
    } else   
    if (b == 1) {
      set.fc_io = fc_bt;
    } else 
    if (b == 2) {
      set.fc_io = fc_wifi;
    } else 
    if (b == 3) {
      set.fc_io = fc_sd;
    }  

    b = EEPROMRead8(4);     // gcs io
    if (b == 0) {
      set.gs_io = gs_ser;
    } else
    if (b == 1) {
      set.gs_io = gs_bt;
    } else 
    if (b == 2) {
      set.gs_io = gs_wifi;
    } else 
    if (b == 3) {
      set.gs_io = gs_wifi_bt;
    }  else 
    if (b == 9) {
      set.gs_io = gs_none;
    } 
        
    b = EEPROMRead8(5);     // gcs sd
    if (b == 0) {
      set.gs_sd = gs_off;
    } else if (b == 1) {
      set.gs_sd = gs_on;
    } 
     b = EEPROMRead8(161);     // sport sd
    if (b == 0) {
      set.sport_sd = spsd_off;
    } else if (b == 1) {
      set.sport_sd = spsd_on;
    }    

    b = EEPROMRead8(6);     // wifi mode
    if (b == 1) {
      set.wfmode = ap;
    } else if (b == 2) {
      set.wfmode = sta;
    } else if (b == 3) {
      set.wfmode = sta_ap;
    } else if (b == 4) {
      set.wfmode = ap_sta;
    } 
    b = EEPROMRead8(7);     // wifi protocol
    if (b == 1) {
      set.mav_wfproto = tcp;
    } else if (b == 2) {
      set.mav_wfproto = udp;
    }  
    set.baud = EEPROMRead32(8);                  //  8 thru  11
    set.channel = EEPROMRead8(12);               //  12
    EEPROMReadString(13, set.apSSID);            //  13 thru 42
    EEPROMReadString(43, set.apPw);               //  4 thru 62

    if(strlen(set.apPw) < 8) {
        // esp fail passphrase too short
      Log.println("AP Password < 8 chars long! Will fail to authenticate");
      LogScreenPrintln("AP PW < 8 chars long");   
    }
    
    EEPROMReadString(63, set.staSSID);           // 63 thru 92 
    EEPROMReadString(93, set.staPw);             // 93 thru 112  

    if(strlen(set.staPw) < 8) {
        // esp fail passphrase too short
      Log.println("STA Password < 8 chars long! Will fail to authenticate");
      LogScreenPrintln("STA PW < 8 chars long");   
    }
    
    EEPROMReadString(113, set.host);             // 113 thru 132   
    set.tcp_localPort = EEPROMRead16(133);       // 133 thru 134 
    set.tcp_remotePort = EEPROMRead16(162);      // 162 thru 163     
    set.udp_localPort = EEPROMRead16(135);       // 135 thru 136 
    set.udp_remotePort = EEPROMRead16(137);      // 137 thru 138 
    b = EEPROMRead8(139);                        // 139
    if (b == 1) {
      set.btmode = master;
    } else if (b == 2) {
      set.btmode = slave;
    } 
    EEPROMReadString(140, set.btConnectToSlave);  // 140 thru 159 - 160 unused for now, 161 sport sd above 

    RefreshHTMLButtons();     
   
    #if defined Debug_Web_Settings
      Log.println();
      Log.println("Debug Read WiFi Settings from EEPROM: ");
      Log.print("validity_check = "); Log.println(set.validity_check, HEX);
      Log.print("translator mode = "); Log.println(set.trmode);  
      Log.print("fr_port_type = "); Log.println(set.frport);      
      Log.print("set.fr_io = "); Log.println(set.fr_io);             
      Log.print("fc_io = "); Log.println(set.fc_io);                
      Log.print("gcs_io = "); Log.println(set.gs_io);            
      Log.print("gcs_sd = "); Log.println(set.gs_sd);   
      Log.print("sport_sd = "); Log.println(set.sport_sd);            
      Log.print("wifi mode = "); Log.println(set.wfmode);
      Log.print("wifi protocol = "); Log.println(set.mav_wfproto);     
      Log.print("baud = "); Log.println(set.baud);
      Log.print("wifi channel = "); Log.println(set.channel);  
      Log.print("apSSID = "); Log.println(set.apSSID);
      Log.print("apPw = "); Log.println(set.apPw);
      Log.print("staSSID = "); Log.println(set.staSSID);
      Log.print("staPw = "); Log.println(set.staPw); 
      Log.print("Host = "); Log.println(set.host);           
      Log.print("tcp_localPort = "); Log.println(set.tcp_localPort);
      Log.print("tcp_remotePort = "); Log.println(set.tcp_remotePort);      
      Log.print("udp_localPort = "); Log.println(set.udp_localPort);
      Log.print("udp_remotePort = "); Log.println(set.udp_remotePort); 
      Log.print("bt mode = "); Log.println(set.btmode); 
      Log.print("SlaveConnect To Name = "); Log.println(set.btConnectToSlave);
      Log.println();          
    #endif  

    Log.println("EEPROM settings read and adopted");  
                
  }
//=================================================================================
void WriteSettingsToEEPROM() {
      set.validity_check = 0xdc;                                     
      EEPROMWrite8(1, set.validity_check);           // apFailover is in 0 
      EEPROMWrite8(2, set.trmode);                   //  2   
      EEPROMWrite8(3, set.fc_io);                    //  3
      EEPROMWrite8(4, set.gs_io);                    //  4  
      EEPROMWrite8(164, set.frport);                 //  164  came late to the party        
      EEPROMWrite8(165, set.fr_io);                  //  165  came late to the party   
      EEPROMWrite8(5, set.gs_sd);                    //  5
      EEPROMWrite8(161, set.sport_sd);               //  161 came late to the party      
      EEPROMWrite8(6, set.wfmode);                   //  6
      EEPROMWrite8(7, set.mav_wfproto);                  //  7     
      EEPROMWrite32(8,set.baud);                     //  8 thru 11
      EEPROMWrite8(12, set.channel);                 // 12
      EEPROMWriteString(13, set.apSSID);             // 13 thru 42 
      EEPROMWriteString(43, set.apPw);               // 43 thru 62      
      EEPROMWriteString(63, set.staSSID);            // 63 thru 92 
      EEPROMWriteString(93, set.staPw);              // 93 thru 112 
      EEPROMWriteString(113, set.host);              // 113 thru 132 
      EEPROMWrite16(133, set.tcp_localPort);         // 133 thru 134       
      EEPROMWrite16(162, set.tcp_remotePort);        // 162 thru 163       
      EEPROMWrite16(135, set.udp_localPort);         // 135 thru 136
      EEPROMWrite16(137, set.udp_remotePort);        // 137 thru 138
      EEPROMWrite8(139, set.btmode);                 // 139     
      EEPROMWriteString(140, set.btConnectToSlave);  // 140 thru 159 - 160 unused for now      
      EEPROM.commit();  
      RefreshHTMLButtons();
                                                                  
      #if defined Debug_Web_Settings
        Log.println();
        Log.println("Debug Write WiFi Settings to EEPROM: ");
        Log.print("validity_check = "); Log.println(set.validity_check, HEX);
        Log.print("translator mode = "); Log.println(set.trmode); 
        Log.print("fr_port_type = "); Log.println(set.frport);        
        Log.print("set.fr_io = "); Log.println(set.fr_io);                
        Log.print("fc_io = "); Log.println(set.fc_io);                
        Log.print("gcs_io = "); Log.println(set.gs_io);              
        Log.print("gcs_sd = "); Log.println(set.gs_sd);   
        Log.print("sport_sd = "); Log.println(set.sport_sd);               
        Log.print("wifi mode = "); Log.println(set.wfmode);
        Log.print("wifi protocol = "); Log.println(set.mav_wfproto);     
        Log.print("baud = "); Log.println(set.baud);
        Log.print("wifi channel = "); Log.println(set.channel);  
        Log.print("apSSID = "); Log.println(set.apSSID);
        Log.print("apPw = "); Log.println(set.apPw);
        Log.print("staSSID = "); Log.println(set.staSSID);
        Log.print("staPw = "); Log.println(set.staPw); 
        Log.print("Host = "); Log.println(set.host);           
        Log.print("tcp_localPort = "); Log.println(set.tcp_localPort);
        Log.print("tcp_remotePort = "); Log.println(set.tcp_remotePort);        
        Log.print("udp_localPort = "); Log.println(set.udp_localPort);
        Log.print("udp_remotePort = "); Log.println(set.udp_remotePort); 
        Log.print("bt mode = "); Log.println(set.btmode); 
        Log.print("Master to Slave Name = "); Log.println(set.btConnectToSlave);       
        Log.println();             
      #endif 
}  
//=================================================================================
void ReadSettingsFromForm() {
  String S;

  #if (defined frBuiltin)
    S = server.arg("_frport");
    if (S == "None") {
      set.frport = f_none;
   } else 
   if (S == "FPort1") {
      set.frport = f_port1;
   }  else 
   if (S == "FPort2") {
      set.frport = f_port2;
   }  else 
   if (S == "SPort") {
     set.frport = s_port; 
   } else 
   if (S == "Auto") {
     set.frport = f_auto; 
   }
  #endif 

  #if (defined frBuiltin)
    S = server.arg("_trmode");
    if (S == "Ground") {
      set.trmode = ground;
   } else 
   if (S == "Air") {
     set.trmode = air;
   } else 
   if (S == "Relay") {
     set.trmode = relay;
   }
  #endif
  
  uint8_t frio = 0;
  S = server.arg("_set.fr_io_ser");      
  if (S == "Serial") {
    frio = frio + (uint8_t)fr_ser;  // bit 1     
  } 
  S = server.arg("_set.fr_io_udp");   
  if (S == "UDP") {
    frio = frio + (uint8_t)fr_udp;  // bit 2
  } 
  S = server.arg("_set.fr_io_sd");   
  if (S == "SD") {
    frio = frio + (uint8_t)fr_sd;   // bit 4  
  }  
  set.fr_io = (fr_io_t)frio;
 
  S = server.arg("_fc_io");
  if (S == "Serial") {
    set.fc_io = fc_ser;
  } else  
  if (S == "BT") {
    set.fc_io = fc_bt;
  } else 
  if (S == "WiFi") {
    set.fc_io = fc_wifi;
  } else 
  if (S == "SD") {
    set.fc_io = fc_sd;
  }   

  S = server.arg("_gs_io");    
  if (S == "Serial") {
    if (set.fc_io == fc_ser) {
      Log.println("Selection of simultaneous serial uplink and serial downlink ignored!");
      LogScreenPrintln("Bad serial selection!");     
    } else {
      set.gs_io = gs_ser;
    }
  } else
  if (S == "BT") {
    set.gs_io = gs_bt;
  } else 
  if (S == "WiFi") {
    set.gs_io = gs_wifi;
  } else 
  if (S == "WiFi+BT") {
    set.gs_io = gs_wifi_bt;
  } else {
    set.gs_io = gs_none;
  }
     
  S = server.arg("_gs_sd");
  if (S == "OFF") {
    set.gs_sd = gs_off;
  } else 
  if (S == "ON") {
    set.gs_sd = gs_on;
  }

  S = server.arg("_sport_sd");
  if (S == "OFF") {
    set.sport_sd = spsd_off;
  } else 
  if (S == "ON") {
    set.sport_sd = spsd_on;
  }
  
  S = server.arg("_wfmode");
  if (S == "AP") {
    set.wfmode = ap;
  } else 
  if (S == "STA") {
    set.wfmode = sta;
  } else 
  if (S == "STA>AP") {
    set.wfmode = sta_ap;
  } else 
  if (S == "AP_STA") {
    set.wfmode = ap_sta;
  } 
   
  S = server.arg("_mav_wfproto");
  if (S == "TCP") {
    set.mav_wfproto = tcp;
  } else 
  if (S == "UDP") {
    set.mav_wfproto = udp;
  }   
  set.baud = String_long(server.arg("_baud"));
  set.channel = String_long(server.arg("_channel")); 
  S = server.arg("_apSSID");
  strcpy(set.apSSID, S.c_str());
  S = server.arg("_apPw");
  strcpy(set.apPw, S.c_str());

  if(strlen(set.apPw) < 8) {
        // esp fail passphrase too short
      Log.println("AP Password < 8 chars long! Will fail to authenticate");
      LogScreenPrintln("AP PW < 8 chars long");   
    }

  S = server.arg("_staSSID");
  strcpy(set.staSSID, S.c_str());
  S = server.arg("_staPw");
  strcpy(set.staPw, S.c_str());  

  if(strlen(set.staPw) < 8) {
        // esp fail passphrase too short
      Log.println("STA Password < 8 chars long! Will fail to authenticate");
      LogScreenPrintln("STA PW < 8 chars long");   
    }

  S = server.arg("_host");
  strcpy(set.host, S.c_str());    
  set.tcp_localPort = String_long(server.arg("_tcp_localPort"));
  set.tcp_remotePort = String_long(server.arg("_tcp_remotePort"));  
  set.udp_localPort = String_long(server.arg("_udp_localPort"));
  set.udp_remotePort = String_long(server.arg("_udp_remotePort")); 
  S = server.arg("_btmode");
  if (S == "Master") {
    set.btmode = master;
  } else 
  if (S == "Slave") {
    set.btmode = slave;
  } 
  
  S = server.arg("_btConnectToSlave");
  strcpy(set.btConnectToSlave, S.c_str());    // strcpy() copies the C string including the terminating null character
  
      #if defined Debug_Web_Settings
        Log.println();
        Log.println("Debug Read WiFi Settings from Form: ");
        Log.print("validity_check = "); Log.println(set.validity_check, HEX);
        Log.print("translator mode = "); Log.println(set.trmode);   
        Log.print("fr_port_type = "); Log.println(set.frport);       
        Log.print("set.fr_io = "); Log.println(set.fr_io);               
        Log.print("fc_io = "); Log.println(set.fc_io);                
        Log.print("gcs_io = "); Log.println(set.gs_io);           
        Log.print("gcs_sd = "); Log.println(set.gs_sd); 
        Log.print("sport_sd = "); Log.println(set.sport_sd);                     
        Log.print("wifi mode = "); Log.println(set.wfmode);
        Log.print("wifi protocol = "); Log.println(set.mav_wfproto);     
        Log.print("baud = "); Log.println(set.baud);
        Log.print("wifi channel = "); Log.println(set.channel);  
        Log.print("apSSID = "); Log.println(set.apSSID);
        Log.print("apPw = "); Log.println(set.apPw);
        Log.print("staSSID = "); Log.println(set.staSSID);
        Log.print("staPw = "); Log.println(set.staPw); 
        Log.print("Host = "); Log.println(set.host);           
        Log.print("tcp_localPort = "); Log.println(set.tcp_localPort);
        Log.print("tcp_remotePort = "); Log.println(set.tcp_remotePort);        
        Log.print("udp_localPort = "); Log.println(set.udp_localPort);
        Log.print("udp_remotePort = "); Log.println(set.udp_remotePort); 
        Log.print("bt mode = "); Log.println(set.btmode); 
        Log.print("Master to Slave Name = "); Log.println(set.btConnectToSlave);      
        Log.println();      
      #endif 
 }
//=================================================================================
void RefreshHTMLButtons() {
  if (set.trmode == ground) {
    set.trmode1 = "checked";
    set.trmode2 = "";
    set.trmode3 = "";
  } else 
  if (set.trmode == air) {
    set.trmode1 = "";
    set.trmode2 = "checked";
    set.trmode3 = "";  
  } else 
  if (set.trmode == relay) {
    set.trmode1 = "";
    set.trmode2 = "";
    set.trmode3 = "checked";
  } 

  if (set.frport == f_port1) {
    set.frport1 = "checked";
    set.frport2 = "";
    set.frport3 = ""; 
    set.frport4 = "";          
  }  else 
  if (set.frport == f_port2) {
    set.frport1 = "";
    set.frport2 = "checked";
    set.frport3 = "";   
    set.frport4 = "";         
  }  else
   if (set.frport == s_port) {
    set.frport1 = "";
    set.frport2 = "";
    set.frport3 = "checked"; 
    set.frport4 = "";          
  } else
   if (set.frport == f_auto) {
    set.frport1 = "";
    set.frport2 = "";
    set.frport3 = ""; 
    set.frport4 = "checked";          
  } 
  
  set.fr_io1 = "";
  set.fr_io2 = "";
  set.fr_io3 = ""; 
  if (set.fr_io & 0x01) {         // Serial
    set.fr_io1 = "checked";
  }  
  if (set.fr_io & 0x02) {         // UDP
    set.fr_io2 = "checked";
  }  
  if (set.fr_io & 0x04) {         // SD Card
    set.fr_io3 = "checked";
  }   

  if (set.fc_io == fc_ser) {
    set.fc_io0 = "checked";
    set.fc_io1 = "";
    set.fc_io2 = "";
    set.fc_io3 = "";   
  } else 
  if (set.fc_io == fc_bt) {
    set.fc_io0 = "";
    set.fc_io1 = "checked";
    set.fc_io2 = "";
    set.fc_io3 = "";     
  } else 
  if (set.fc_io == fc_wifi) {
    set.fc_io0 = "";
    set.fc_io1 = "";
    set.fc_io2 = "checked";
    set.fc_io3 = "";      
  } else
  if (set.fc_io == fc_sd) {
    set.fc_io0 = "";
    set.fc_io1 = "";
    set.fc_io2 = "";
    set.fc_io3 = "checked";  
  }

  if (set.gs_io == gs_ser) {
    set.gs_io0 = "checked";
    set.gs_io1 = "";
    set.gs_io2 = "";
    set.gs_io3 = ""; 
    set.gs_io9 = "";           
  } else 
  if (set.gs_io == gs_bt) {
    set.gs_io0 = "";
    set.gs_io1 = "checked";
    set.gs_io2 = "";
    set.gs_io3 = ""; 
    set.gs_io9 = "";           
  } else 
  if (set.gs_io == gs_wifi) {
    set.gs_io0 = "";
    set.gs_io1 = "";
    set.gs_io2 = "checked";
    set.gs_io3 = "";   
    set.gs_io9 = "";           
  } else
  if (set.gs_io == gs_wifi_bt) {
    set.gs_io0 = "";
    set.gs_io1 = "";
    set.gs_io2 = "";
    set.gs_io3 = "checked";
    set.gs_io9 = "";           
  } else
  if (set.gs_io == gs_none) {
    set.gs_io0 = "";
    set.gs_io1 = "";
    set.gs_io2 = "";
    set.gs_io3 = "";
    set.gs_io9 = "checked";            
  } 
   
  if (set.gs_sd == gs_off) {
    set.gs_sd0 = "checked";
    set.gs_sd1 = "";
  }  else 
  if (set.gs_sd == gs_on) {
    set.gs_sd0 = "";
    set.gs_sd1 = "checked";
  }

  if (set.sport_sd == spsd_off) {
    set.sport_sd0 = "checked";
    set.sport_sd1 = "";
  }  else 
  if (set.sport_sd == spsd_on) {
    set.sport_sd0 = "";
    set.sport_sd1 = "checked";
  }
    
  if (set.wfmode == ap) {
    set.wfmode1 = "checked";
    set.wfmode2 = "";
    set.wfmode3 = "";
    set.wfmode4 = "";    
  } else 
  if (set.wfmode == sta) {
    set.wfmode1 = "";
    set.wfmode2 = "checked";
    set.wfmode3 = "";
    set.wfmode4 = "";     
  } else
  if (set.wfmode == sta_ap) {
    set.wfmode1 = "";
    set.wfmode2 = "";
    set.wfmode3 = "checked";
    set.wfmode4 = "";     
  } else
  if (set.wfmode == ap_sta) {
    set.wfmode1 = "";
    set.wfmode2 = "";
    set.wfmode3 = "";
    set.wfmode4 = "checked";     
  }

  if (set.mav_wfproto == tcp) {
    set.mav_wfproto1 = "checked";
    set.mav_wfproto2 = "";
  } else 
  if (set.mav_wfproto == udp) {
    set.mav_wfproto1 = "";
    set.mav_wfproto2 = "checked";
  }

  if (set.btmode == master) {
    set.btmode1 = "checked";
    set.btmode2 = "";
  } else 
  if (set.btmode == slave) {
    set.btmode1 = "";
    set.btmode2 = "checked";
  }  
 
}
 //===========================================================================================
 void handleLoginPage() {

  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap before handleLoginPage = %d\n", ESP.getFreeHeap());
  #endif  

  ComposeLoginPage();
  server.send(200, "text/html", loginPage); 
 }
 //===========================================================================================
 void handleSettingsPage() {

  Free_Bluetooth_RAM();   // Disables BT and required a reboot to reinstate BT

  ComposeSettingsPage();
  server.send(200, "text/html", settingsPage); 
 }
 //===========================================================================================
 void handleSettingsReturn() {
  ReadSettingsFromForm();

  WriteSettingsToEEPROM();
  
  String s = "<a href='/'> Rebooting........  Back to login screen</a>";
  server.send(200, "text/html", styleLogin+s);
  Log.println("Rebooting to adopt new settings ....\n");  
  delay(3000);
  ESP.restart();                 // esp32 and esp8266   
 }
 
 //===========================================================================================
 void handleReboot() {
  String s = "<a href='/'> Rebooting........  Back to login screen</a>";
  server.send(200, "text/html", styleLogin+s);
  Log.println("Rebooting ......");  
  delay(3000);
  ESP.restart();                 // esp32 and esp8266   
 }
 
 //===========================================================================================
 void handleOtaPage() {

  //Free_Bluetooth_RAM();   // Disables BT and required a reboot to reinstate BT
  
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", otaIndex);
  /*handle upload of firmware binary file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      uint32_t uploadSize;
 //   Log.setDebugOutput(true);
      #if (defined ESP32) 
        uploadSize = UPDATE_SIZE_UNKNOWN;
      #elif (defined ESP8266) 
        WiFiUDP::stopAll();
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        uploadSize = maxSketchSpace;
      #endif

      Log.printf("Update: %s\n", upload.filename.c_str());    
      if (!Update.begin(uploadSize)) { //start with max available size
        Update.printError(Log);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP OTA space */
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Log);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to show the percentage progress bar
        Log.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        delay(2000);
      } else {
        Update.printError(Log);
      }
   // Log.setDebugOutput(false);
    }
    #if (defined ESP8266) // ESP8266 
      yield();
    #endif  
  });
 }
  //===========================================================================================
  //                           E E P R O M  Routines - ESP Only
  //===========================================================================================


  //This function will write a 4 byte (32bit) uint32_t to the eeprom at
  //the specified address to address + 3.
  void EEPROMWrite32(uint16_t address, uint32_t value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }

  //===========================================================================================
  //This function will read a 4 byte (32bits) uint32_t from the eeprom at
  //the specified address to address + 3.         
  uint32_t EEPROMRead32(uint16_t address)
      {
      //Read the 4 bytes from the eeprom memory.
      uint32_t four = EEPROM.read(address);
      uint32_t three = EEPROM.read(address + 1);
      uint32_t two = EEPROM.read(address + 2);
      uint32_t one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }
  //===========================================================================================
  //This function will read a 2 byte (16bits) uint16_t from the eeprom at
  //the specified address to address + 1.     
  uint16_t EEPROMRead16(uint16_t address)
      {
      //Read the 2 bytes from the eeprom memory.
      uint32_t two = EEPROM.read(address);
      uint32_t one = EEPROM.read(address + 1);

      //Return the recomposed uint by using bitshift.
      uint16_t n = ((two << 0) & 0xFF) + ((one << 8)& 0xFFFF);
      return n;
      }
  //===========================================================================================
  //This function will read a byte (8bits) from the eeprom at
  //the specified address.    
  uint8_t EEPROMRead8(uint16_t address)   {
      uint8_t one = EEPROM.read(address);
      return one;
  }
  //===========================================================================================
  // This function will read a char array (string) from EEPROM at 
  //the specified address. 

  /*
  void EEPROMReadString(uint16_t address, char strptr[]) {
    String s;
    s = EEPROM.readString(address);  
    strcpy(strptr, s.c_str()); 
  }
  */

  void EEPROMReadString(uint16_t address, char *strptr) {
  char s[30];
        for (int i = 0 ; i < 30 ; i++) {  // safety limit
          s[i] = EEPROM.read(address+i);
          if (s[i] == 0x00) {             // eo string
            strcpy(strptr, s);  
            break;    
          }
        }
   } 

  //===========================================================================================
  //This function will write a 2 byte (16bits) uint16_t to the eeprom at
  //the specified address to address + 1.
  void EEPROMWrite16(uint16_t address, uint16_t value)
      {
      //Decomposition from an int to 2 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte two = (value & 0xFF);
      byte one = ((value >> 8) & 0xFF);

      //Write the 2 bytes into the eeprom memory.
      EEPROM.write(address, two);
      EEPROM.write(address + 1, one);
      } 
  //===========================================================================================
  //This function will write 1 byte (8bits) uint16_t to the eeprom at
  //the specified address.
  void EEPROMWrite8(uint16_t address, uint8_t value)
      {
      //Write the byte into the eeprom memory.
      EEPROM.write(address, value);
      }   
  //===========================================================================================
  // This function will write a char array (string) to EEPROM at 
  //the specified address.    
  void EEPROMWriteString(uint16_t address, char* strptr) {
        for (int i = 0 ; i <= 30 ; i++) {    // constraint
          EEPROM.write(address+i, strptr[i]);
          if (strptr[i] == 0x00) {
            break;       // eo string
          }
        }   
   }

  /*
  Also available in class library
  EEPROMwriteString (int address, const char* value)
  */

  //=================================================================================
#endif               // End of webSupport - ESP only
 //=================================================================================
 
void RawSettingsToStruct() {
  
  #if defined Ground_Mode 
    set.trmode = ground;
  #elif  defined Air_Mode 
    set.trmode = air;
  #elif  defined Relay_Mode 
    set.trmode = relay;
  #endif
  
  #if (FrSky_Port_Type == 0)
    set.frport = f_none;
  #elif (FrSky_Port_Type == 1)
    set.frport = f_port1;
  #elif (FrSky_Port_Type == 2)
    set.frport = f_port2;   
   #elif (FrSky_Port_Type == 3)
    set.frport = s_port;   
  #elif (FrSky_Port_Type == 4)
    set.frport = f_auto;    
  #endif
  
  if (FC_Mavlink_IO == 0) {
    set.fc_io = fc_ser;
  } else  
  if (FC_Mavlink_IO == 1) {
    set.fc_io = fc_bt;
  } else 
  if (FC_Mavlink_IO == 2) {
    set.fc_io = fc_wifi;
  } else 
  if (FC_Mavlink_IO == 3) {
    set.fc_io = fc_sd;
  }  

  if (GCS_Mavlink_IO == 0) {
    set.gs_io = gs_ser;
  } else 
  if (GCS_Mavlink_IO == 1) {
    set.gs_io = gs_bt;
  } else 
  if (GCS_Mavlink_IO == 2) {
    set.gs_io = gs_wifi;
  } else 
  if (GCS_Mavlink_IO == 3) {
    set.gs_io = gs_wifi_bt;
  }  else
  if (GCS_Mavlink_IO == 9) {
    set.gs_io = gs_none;
  }  

  if (FrSky_IO == 1) {
    set.fr_io = fr_ser;
  } else 
  if (FrSky_IO == 2) {
    set.fr_io = fr_udp;
  }  else
  if (FrSky_IO == 3) {
    set.fr_io = fr_ser_udp;
  }  else  
  if (FrSky_IO == 4) {
    set.fr_io = fr_sd;
  }  else   
  if (FrSky_IO == 5) {
    set.fr_io = fr_ser_sd;
  }  else  
  if (FrSky_IO == 6) {
    set.fr_io = fr_udp_sd;
  }  else    
  if (FrSky_IO == 7) {
    set.fr_io = fr_ser_udp_sd;
  }  

  #if defined GCS_Mavlink_SD 
    set.gs_sd = gs_on;
  #else
    set.gs_sd = gs_off;
  #endif

  #if defined FPort_To_SD 
    set.sport_sd = spsd_on;
  #else
    set.sport_sd = spsd_off;
  #endif
      
  if (WiFi_Mode == 1) {
    set.wfmode = ap;
  } else 
  if (WiFi_Mode == 2) {
    set.wfmode = sta;
  } else 
  if (WiFi_Mode == 3) {
    set.wfmode = sta_ap;
  } else 
  if (WiFi_Mode == 4) {
    set.wfmode = ap_sta;
  } 
    
  if (Mav_WiFi_Protocol == 1) {
    set.mav_wfproto = tcp;
  } else 
  if (Mav_WiFi_Protocol == 2) {
    set.mav_wfproto = udp;
  } 
         
  set.baud = mvBaud;          
  set.channel = APchannel;
  strcpy(set.apSSID, APssid);  
  strcpy(set.apPw, APpw);                          
  strcpy(set.staSSID, STAssid);           
  strcpy(set.staPw, STApw);   
  strcpy(set.host, HostName);        
  set.tcp_localPort = TCP_localPort;
  set.tcp_remotePort = TCP_remotePort;  
  set.udp_localPort = UDP_remotePort;
  set.udp_remotePort = UDP_localPort;  

  if ( BT_Mode == 1 ) {
    set.btmode = master;
  } else if ( BT_Mode == 2 ) {
    set.btmode = slave; 
  }  
  strcpy(set.btConnectToSlave, BT_ConnectToSlave); 
  
  #if (defined webSupport)   
    set.web_support =  true;   // this flag is not saved in eeprom
  #else
    set.web_support =  false;
  #endif

  #if defined webSupport
     RefreshHTMLButtons();
  #endif
     
  #if defined Debug_Web_Settings
      Log.println();
      Log.println("Debug Raw WiFi Settings : ");
      Log.print("web_support = "); Log.println(set.web_support);      
      Log.print("validity_check = "); Log.println(set.validity_check, HEX);   
      Log.print("translator mode = "); Log.println(set.trmode);    
      Log.print("frport = "); Log.println(set.frport);           
      Log.print("set.fr_io = "); Log.println(set.fr_io);         
      Log.print("fc_io = "); Log.println(set.fc_io);                
      Log.print("gcs_io = "); Log.println(set.gs_io);     
      Log.print("gcs_sd = "); Log.println(set.gs_sd);   
      Log.print("sport_sd = "); Log.println(set.sport_sd);              
      Log.print("wifi mode = "); Log.println(set.wfmode);
      Log.print("wifi protocol = "); Log.println(set.mav_wfproto);     
      Log.print("baud = "); Log.println(set.baud);
      Log.print("wifi channel = "); Log.println(set.channel);  
      Log.print("apSSID = "); Log.println(set.apSSID);
      Log.print("apPw = "); Log.println(set.apPw);
      Log.print("staSSID = "); Log.println(set.staSSID);
      Log.print("staPw = "); Log.println(set.staPw); 
      Log.print("Host = "); Log.println(set.host);           
      Log.print("tcp_localPort = "); Log.println(set.tcp_localPort);
      Log.print("tcp_remotePort = "); Log.println(set.tcp_remotePort);      
      Log.print("udp_localPort = "); Log.println(set.udp_localPort);
      Log.print("udp_remotePort = "); Log.println(set.udp_remotePort); 
      Log.print("bt mode = "); Log.println(set.btmode); 
      Log.print("Master to Slave Name = "); Log.println(set.btConnectToSlave); 
   
      Log.println(); 
  #endif    
}

//=================================================================================

