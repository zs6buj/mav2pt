/*
=====================================================================================================================
     MavToPass  (Mavlink To FrSky Passthru) Protocol Translator

 
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

  By downloading this software you are agreeing to the terms specified in this page and the spirit of thereof.
    
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
    to receive FrSky S.Port protocol telemetry for display on their screen.  Excellent 
    firmware is available to convert Mavlink to the native S.Port protocol, however the 
    author is unaware of a suitable solution to convert to the Passthru protocol. 

    This protocol translator has been especially tailored to work with the excellent LUA 
    display interface from yaapu for the FrSky Horus, Taranis and QX7 controllers. 
    https://github.com/yaapu/FrskyTelemetryScript . The translator also works with the 
    popular FlightDeck product.
    
    The firmware translates APM or PX4 Mavlink telemetry to FrSky S.Port passthru telemetry, 
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
    1) SPort S     -->tx1 Pin 1    S.Port out to XSR  or Taranis bay, bottom pin
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
#include "SPort.h"

#if defined TEENSY3X || defined ESP8266  // Teensy 3.x && ESP8266 
  #undef F   // F defined as m->counter[5]  in c_library_v2\mavlink_sha256.h
             // Macro F()defined in Teensy3/WString.h && ESP8266WebServer-impl.h (forces string literal into prog mem)   
#endif

#include <ardupilotmega/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>


//=================================================================================================   
//                     F O R W A R D    D E C L A R A T I O N S
//=================================================================================================

void DisplayPrintln(String);
uint32_t GetBaud(uint8_t);
void main_loop();
void SenseWiFiStatus(); 
void StartWiFiTimer();
void RestartWiFiSta();
bool Read_FC_To_RingBuffer();
void RB_To_Decode_and_GCS();
void Read_From_GCS();
void Decode_GCS_To_FC();
void Write_To_FC(uint32_t);
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
bool Read_UDP(mavlink_message_t*);
bool Send_TCP(mavlink_message_t*);
bool Send_UDP(mavlink_message_t*);
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
uint32_t GetConsistent(uint8_t);
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
void PrintRemoteIP();
void DisplayPrint(String);
#if (defined ESP32) || (defined ESP8266)
  void PaintDisplay(uint8_t, last_row_t);
  void Scroll_Display(scroll_t);
  void IRAM_ATTR gotButtonDn();
  void IRAM_ATTR gotButtonUp();
  void IRAM_ATTR gotWifiButton();
  #endif
#if (defined ESP32)
  void IRAM_ATTR onTimer();                         // wifi retry timer
  esp_err_t event_handler(system_event_t *event);   // wifi events handler
#endif  


  SPort mySPort;   // instantiate S.Port object 
   

//=================================================================================================
//=================================================================================================   
//                                      S   E   T   U  P 
//=================================================================================================
//=================================================================================================
void setup()  {
 
  Debug.begin(115200);
  delay(2500);
  Debug.println();
  pgm_path = __FILE__;  // ESP8266 __FILE__ macro returns pgm_name and no path
  pgm_name = pgm_path.substring(pgm_path.lastIndexOf("\\")+1);  
  pgm_name = pgm_name.substring(0, pgm_name.lastIndexOf('.'));  // remove the extension
  Debug.print("Starting "); Debug.print(pgm_name); Debug.println(" .....");
    
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_WiFi)
   WiFi.onEvent(WiFiEventHandler);   
  #endif  
 
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Debug.printf("Free Heap just after startup = %d\n", ESP.getFreeHeap());
  #endif  
//=================================================================================================   
//                                 S E T U P   D I S P L A Y
//=================================================================================================
  #if (defined Display_Support) 

    #if (defined ESP32)
      if ( (Tup != 99) && (Tdn != 99) ) {   // enable touch pin-pair
        touchAttachInterrupt(digitalPinToInterrupt(Tup), gotButtonUp, threshold);
        touchAttachInterrupt(digitalPinToInterrupt(Tdn), gotButtonDn, threshold);
      } else
      if ( (Pup != 99) && (Pdn != 99) ) {   // enable digital pin-pair
        pinMode(Pup, INPUT_PULLUP);
        pinMode(Pdn, INPUT_PULLUP);          
      }

    #endif  

    #if (defined ESP8266)           // interrupt on ESP8266 is momentary, not what we need
      if ( (Pup != 99) && (Pdn != 99) ) { // enable digital pin pair
        pinMode(Pup, INPUT_PULLUP);
        pinMode(Pdn, INPUT_PULLUP);
      }

    #endif 

    #if (defined ST7789_Display)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
      display.init();
      if (screenOrientation == 0) {   // portrait
        display.setRotation(0);       // or 4 
        display.setTextSize(1);
        display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6
        #define screen_height  20     // characters not pixels
      } else 
      if (screenOrientation == 1) {   // landscape
        display.setRotation(3);       // or 1
        display.setTextSize(2);  
        display.setTextFont(1); 
        #define screen_height   8     // characters not pixels         
      }      

      #define screenBackground TFT_BLACK
      
      display.fillScreen(screenBackground);
      display.setTextColor(TFT_SKYBLUE);    
            
  //    display.setTextColor(TFT_WHITE);
  //    display.setTextColor(TFT_BLUE);  
  //    display.setTextColor(TFT_GREEN, TFT_BLACK);
    
    #elif (defined SSD1306_Display)            // all other boards with SSD1306 OLED display
      Wire.begin(SDA, SCL);
      display.begin(SSD1306_SWITCHCAPVCC, i2cAddr); 
      display.clearDisplay(); 
      display.setTextColor(WHITE);  
      display.setTextSize(1);  
      #define screenBackground BLACK  
      #define screen_height  8      // characters not pixels
    #endif
    
    display.setCursor(0,0);
  
    Debug.println("Display support activated");
    DisplayPrintln("Starting .... ");
  #endif
  /*
  display.setFont(Dialog_plain_8);     //  col=24 x row 8  on 128x64 display
  display.setFont(Dialog_plain_16);    //  col=13 x row=4  on 128x64 display
  */
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Debug.printf("==============>Free Heap after OLED setup = %d\n", ESP.getFreeHeap());
  #endif
  
//=================================================================================================   
//                             S E T U P   E E P R O M
//=================================================================================================

  #if defined ESP32    
    if (!EEPROM.begin(EEPROM_SIZE))  {
      Debug.println("Fatal error!  EEPROM failed to initialise.");
      DisplayPrintln("EEPROM fatal error!");
      while (true) delay(100);  // wait here forever 
     } else {
      Debug.println("EEPROM initialised successfully");
      DisplayPrintln("EEPROM good"); 
     }
  #endif       
    
  #if defined ESP8266
    EEPROM.begin(EEPROM_SIZE);
    Debug.println("EEPROM initialised successfully");
    DisplayPrintln("EEPROM good"); 
  #endif

  RawSettingsToStruct();      // So that we can use them regardless of webSupport
  
  #if (defined webSupport) 
    RecoverSettingsFromFlash(); 
  #endif

  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Debug.printf("==============>Free Heap after EEPROM setup = %d\n", ESP.getFreeHeap());
  #endif

  // =============================================

  Debug.print("Target Board is ");
  #if (defined TEENSY3X) // Teensy3x
    Debug.println("Teensy 3.x");
    DisplayPrintln("Teensy 3.x");
  #elif (defined ESP32) //  ESP32 Board
    Debug.print("ESP32 / Variant is ");
    DisplayPrintln("ESP32 / Variant is");
    #if (ESP32_Variant == 1)
      Debug.println("Dev Module");
      DisplayPrintln("Dev Module");
    #endif
    #if (ESP32_Variant == 2)
      Debug.println("Wemos® LOLIN ESP32-WROOM-32");
      DisplayPrintln("Wemos® LOLIN");
    #endif
    #if (ESP32_Variant == 3)
      Debug.println("Dragonlink V3 slim with internal ESP32");
      DisplayPrintln("Dragonlink V3 ESP32");
    #endif
    #if (ESP32_Variant == 4)
      Debug.println("Heltec Wifi Kit 32");
      DisplayPrintln("Heltec Wifi Kit 32");
    #endif
    #if (ESP32_Variant == 5)
      Debug.println("LILYGO® TTGO T-Display ESP32 1.14 inch ST7789 Colour LCD");
      DisplayPrintln("TTGO T-Display ESP32");
    #endif    
    
  #elif (defined ESP8266) 
    Debug.print("ESP8266 / Variant is ");
    DisplayPrintln("ESP8266 / Variant is");  
    #if (ESP8266_Variant == 1)
      Debug.println("Lonlin Node MCU 12F");
      DisplayPrintln("Node MCU 12");
    #endif 
    #if (ESP8266_Variant == 2)
      Debug.println("ESP-F - RFD900X TX-MOD");
      DisplayPrintln("RFD900X TX-MOD");
    #endif   
    #if (ESP8266_Variant == 3)
      Debug.println("Wemos® ESP-12F D1 Mini");
      DisplayPrintln("Wemos ESP-12F D1 Mini");
    #endif       
  #endif

  if (set.trmode == ground) {
    Debug.println("Ground Mode");
    DisplayPrintln("Ground Mode");
  } else  
  if (set.trmode == air) {
    Debug.println("Air Mode");
    DisplayPrintln("Air Mode");
  } else
  if (set.trmode == relay) {
    Debug.println("Relay Mode");
    DisplayPrintln("Relay Mode");
  }

  #if (Battery_mAh_Source == 1)  
    Debug.println("Battery_mAh_Source = 1 - Get battery capacities from the FC");
    DisplayPrintln("mAh from FC");
  #elif (Battery_mAh_Source == 2)
    Debug.println("Battery_mAh_Source = 2 - Define battery capacities in this firmware");  
    DisplayPrintln("mAh defined in fw");
  #elif (Battery_mAh_Source == 3)
    Debug.println("Battery_mAh_Source = 3 - Define battery capacities in the LUA script");
    DisplayPrintln("Define mAh in LUA");     
  #else
    #error You must define at least one Battery_mAh_Source. Please correct.
  #endif            

  #ifndef RSSI_Override
    Debug.println("RSSI Automatic Select");
    DisplayPrintln("RSSI Auto Select");     
  #else
    Debug.printf("RSSI Override for testing = %d%%\n", RSSI_Override);
    snprintf(snprintf_buf, max_col, "RSSI Override %d%%", RSSI_Override);
    DisplayPrint(snprintf_buf);      
  #endif

  if (set.fc_io == fc_ser)   {
    Debug.println("Mavlink Serial In");
    DisplayPrintln("Mavlink Serial In");
  }


  if (set.fc_io == fc_bt)  {
    Debug.println("Mavlink Bluetooth In");
    DisplayPrintln("Mavlink BT In");
  } 

  if (set.fc_io == fc_wifi)  {
    Debug.println("Mavlink WiFi In");
    DisplayPrintln("Mavlink WiFi In");
  } 

  if (set.gs_io == gs_ser)  {
    Debug.println("Mavlink Serial Out");
    DisplayPrintln("Mavlink Serial Out");
  }

  if (set.gs_io == gs_bt)  {
    Debug.println("Mavlink Bluetooth Out");
    DisplayPrintln("Mavlink BT Out");
  }

  if (set.gs_io == gs_wifi)  {
    Debug.print("Mavlink WiFi Out - ");
    DisplayPrintln("Mavlink WiFi Out");
  }

  if (set.gs_io == gs_wifi_bt)  {
    Debug.print("Mavlink WiFi+BT Out - ");
    DisplayPrintln("Mavlink WiFi+BT Out");
  }

  if ((set.fc_io == fc_wifi) || (set.gs_io == gs_wifi) ||  (set.gs_io == gs_wifi_bt) || (set.web_support)) {
   if (set.wfmode == ap)  {
     Debug.println("WiFi mode is AP");
     DisplayPrintln("WiFi mode is AP");
   }  
   else if (set.wfmode == sta)  {
     Debug.println("WiFi mode is STA");
     DisplayPrintln("WiFi mode is STA");
   }
   else if (set.wfmode == sta_ap)  {
     Debug.println("WiFi mode is STA/AP");
     DisplayPrintln("WiFi mode is STA/AP");
   }
    
   if (set.wfproto == tcp)  {
     Debug.println("Protocol is TCP/IP");
     DisplayPrintln("Protocol is TCP/IP");
   }
   else if  (set.wfproto == udp) {
     Debug.print("Protocol is UDP");
     DisplayPrintln("Protocol is UDP");
     #if defined UDP_Broadcast
       Debug.println(" Broadcast");
       DisplayPrintln(" Broadcast");
     #else
       Debug.println(" IP-Targeted");     
       DisplayPrintln("IP-Targeted");
     #endif
   }
  }
  #if defined SD_Support                    
    if (set.fc_io == fc_sd) {
      Debug.println("Mavlink SD In");
      DisplayPrintln("Mavlink SD In");
   }

    if (set.gs_sd == gs_on) {
      Debug.println("Mavlink SD Out");
      DisplayPrintln("Mavlink SD Out");
    }
    
    if (set.sport_sd == spsd_on) {
      Debug.println("SPort To SD Out");
      DisplayPrintln("SPort To SD Out");
    }
    
  #endif
 
//=================================================================================================   
//                                S E T U P   W I F I  --  E S P only
//=================================================================================================

  #if (defined wifiBuiltin)
    if ((set.fc_io == fc_wifi) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.web_support)) {     
      #if (not defined Start_WiFi)
        if (startWiFiPin != 99) { 
          pinMode(startWiFiPin, INPUT_PULLUP);
          attachInterrupt(digitalPinToInterrupt(startWiFiPin),gotWifiButton, FALLING);         
        }
      #endif       
    }

  #else
    Debug.println("No WiFi options selected, WiFi support not compiled in");
  #endif

//=================================================================================================   
//                                   S E T U P   B L U E T O O T H
//=================================================================================================

// Slave advertises hostname for pairing, master connects to slavename

  #if (defined btBuiltin)
    if ((set.fc_io == fc_bt) || (set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt)) { 
      if (set.btmode == 1)   {               // master
        Debug.printf("Bluetooth master mode host %s is trying to connect to slave %s. This can take up to 30s\n", set.host, set.btConnectToSlave); 
        DisplayPrintln("BT connecting ......");   
        SerialBT.begin(set.host, true);      
        // Connect(address) is relatively fast (10 secs max), connect(name) is slow (30 secs max) as it 
        // needs to resolve the name to an address first.
        // Set CoreDebugLevel to Info to view devices bluetooth addresses and device names
        bool bt_connected;
        bt_connected = SerialBT.connect(set.btConnectToSlave);
        if(bt_connected) {
          Debug.println("Bluetooth done");  
          DisplayPrintln("BT done");
        }          
      } else {                               // slave, passive                           
        SerialBT.begin(set.host);        
        Debug.printf("Bluetooth slave mode, host name for pairing is %s\n", set.host);               
      } 
      btActive = true;    
    }
  #else
    Debug.println("No Bluetooth options selected, BT support not compiled in");
    btActive = false;
  #endif
  
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Debug.printf("==============>Free Heap after Bluetooth setup = %d\n", ESP.getFreeHeap());
  #endif
 //=================================================================================================   
 //                                  S E T U P   S D   C A R D  -  E S P 3 2  O N L Y  for now
 //=================================================================================================
  #if ((defined ESP32) || (defined ESP8266)) && (defined SD_Support)
  
    void listDir(fs::FS &fs, const char *, uint8_t);  // Fwd declare
    
    if(!SD.begin()){   
        Debug.println("No SD card reader found. Ignoring SD!"); 
        DisplayPrintln("No SD reader");
        DisplayPrintln("Ignoring!");
        sdStatus = 0; // 0=no reader, 1=reader found, 2=SD found, 3=open for append 
                      // 4=open for read, 5=eof detected, 9=failed
    } else {
      Debug.println("SD card reader mount OK");
      DisplayPrintln("SD drv mount OK");
      uint8_t cardType = SD.cardType();
      sdStatus = 1;
      if(cardType == CARD_NONE){
          Debug.println("No SD card found");
          DisplayPrintln("No SD card");
          DisplayPrintln("Ignoring!");      
      } else {
        Debug.println("SD card found");
        DisplayPrintln("SD card found");
        sdStatus = 2;

        Debug.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
        Debug.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

        listDir(SD, "/", 2);

        if (set.fc_io == fc_sd)  {   //  FC side SD in only   
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
           //   cout << i << fnPath[j] << "\n";
             Debug.print(j); Debug.print(" "); Debug.println(fnPath[j].c_str());
            }
           */     

           sprintf(cPath, "%s", fnPath[i].c_str());  // Select the path
           Debug.print(cPath); Debug.println(" selected "); 
           Debug.println("Reading SD card");
           DisplayPrintln("Reading SD card");
           file = SD.open(cPath);
           if(!file){
             Debug.printf("Can't open file: %s\n", cPath);
             Debug.println(" for reading");
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
    Debug.printf("==============>Free Heap after SD Card setup = %d\n", ESP.getFreeHeap());
  #endif
   
#endif
//=================================================================================================   
//                                    S E T U P   S E R I A L
//=================================================================================================  

  if ((set.fc_io == fc_ser) || (set.gs_io == gs_ser))  {  //  Serial
    #if defined AutoBaud
      set.baud = GetBaud(mav_rxPin);
    #endif   
    #if (defined ESP32)   
      // system can wait here for a few seconds (timeout) if there is no telemetry in
      mvSerial.begin(set.baud, SERIAL_8N1, mav_rxPin, mav_txPin);   //  rx,tx, cts, rts  
    #else
      mvSerial.begin(set.baud);    
    #endif 
    Debug.printf("Mavlink serial on pins rx = %d and tx = %d\n", mav_rxPin, mav_txPin); 
  }
   
  mySPort.initialise();

  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Debug.printf("==============>Free Heap after Serial UART setup = %d\n", ESP.getFreeHeap());
  #endif

//=================================================================================================   
//                                    S E T U P   O T H E R
//=================================================================================================   
  mavGood = false;
  homGood = false;     
  hb_count = 0;
  hb_millis=millis();
  sp_timeout_millis = millis();     // most recent s.port read
  downlink_millis = millis();       // mavlink downlink timimg
  sport_millis = millis();          // sport timimg
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
  
  //===============================       C h e c k   F r e e   H e a p

  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    if (millis() - free_heap_millis > 5000) {
      free_heap_millis = millis();
      Debug.printf("Free Heap now = %d\n", ESP.getFreeHeap());
    }
  #endif
  
  //===============================       H a n d l e   i n c o m i n g   t c p  c l i e n t s

  #if (defined wifiBuiltin)
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt)) {  // Only when we expect incoming remote clients
      
      if (set.wfproto == tcp)  {  // TCP  
        if (wifiSuGood) {

          WiFiClient newClient = TCPserver.available();  // check if a new client wants to connect
          if (newClient) {
            for (int i=0 ; i < max_clients ; ++i) {      // if connected add it into our connected clients table
              if (NULL == clients[i]) {                  // find first empty slot in table
                clients[i] = new WiFiClient(newClient);  // create new client object to use
                inbound_clientGood = true;
                active_client_idx = i;
                Debug.printf("Remote tcp client %d connected\n", i+1);
                snprintf(snprintf_buf, max_col, "Client %d connected", i+1);        
                DisplayPrintln(snprintf_buf);               
                break;
              }
            }
          }

          for (int i=0 ; i < max_clients ; ++i) {      // check to see if any clients have disconnected
            if (NULL != clients[i]) {      
                if (!clients[i]) {
                  nbdelay(100);  
                  clients[i]->stop();
                  clients[i] = NULL;
                  nbdelay(100);  
                  inbound_clientGood = false; 
                  Debug.println("Client disconnected");
                  DisplayPrintln("Client disconnected");           
                }
                break;
              }
          }
        }
      }
    }     
  #endif

  
 //====================

  #if defined Debug_Loop_Period
    PrintLoopPeriod();
  #endif
  
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_WiFi)
    if (inbound_clientGood != clientPrev) {
      Debug.printf("inbound_clientGood=%d\n", inbound_clientGood);
      clientPrev = inbound_clientGood;
    }
  #endif

  #if (defined wifiBuiltin)
    SenseWiFiStatus();  // Optional start-wifi pin
  #endif

  //==================== F l i g h t   C o m p u t e r   T o   R i n g   B u f f e r
  
  if (!Read_FC_To_RingBuffer()) {  //  check for SD eof
    if (sdStatus == 5) {
      Debug.println("End of SD file");
      DisplayPrintln("End of SD file");  
      sdStatus = 0;  // closed after reading   
    }
  }

  //==================== 
  
  bool rssiOverride = false;
  #ifdef RSSI_Override
    rssiOverride = true;     // for debugging
  #endif

  //====================    R i n g   B u f f e r   D e c o d e  &  S e n d   T o   G C S
   
  if (millis() - downlink_millis > 1) {   // main timing loop for mavlink decode and GCS
    RB_To_Decode_and_GCS();
  }
  
 //====================     S P o r t   S e n d   a n d   R e c e i v e

  if( ((millis() - sport_millis) > 2) ) {   // zero does not work for Teensy 3.2
    mySPort.SendAndReceive();       
    sport_millis = millis();
  }

  //====================   R e a d   F r o m   G C S
  
  Read_From_GCS();  

  if (GCS_available) {
    Decode_GCS_To_FC();
    Write_To_FC(G2Fmsg.msgid);  
    GCS_available = false;                      
   }

  //==================== Check For Heartbeat from GCS Timeout
  
  if(gshbGood && (millis() - gshb_millis) > 10000){ // if no heartbeat from GCS in n seconds then assume GCS not connected
    gshbGood = false;
  //  Debug.println("Heartbeat from GCS timed out! GCS not connected"); 
  //  DisplayPrintln("GCS Heartbeat lost!"); 
    
    if (wifiSuGood && wifiSuDone && set.wfmode == sta)  {        
      // do something
    }
  }
 
  //====================   RSSI pacemaker for OpenTX 

  #if defined Rssi_Pacemaker
    if ((set.trmode == ground) || (set.trmode == relay))  {       // In Air_Mode the FrSky receiver provides rssi
      if ( ( (rssiGood && mavGood)  || rssiOverride ) && (millis() - rssi_millis >= Rssi_Pacemaker) ) {
        mySPort.PushMessage(0xF101, 0);   // 0xF101 RSSI 
        rssi_millis = millis(); 
       }   
    }
  #endif   

  //==================== Check For FC Heartbeat Timeout
  
  if(mavGood && (millis() - hb_millis) > 15000)  {   // if no heartbeat from APM in 15s then assume FC mav not connected
    mavGood=false;
    Debug.println("Heartbeat from FC timed out! FC not connected"); 
    DisplayPrintln("FC heartbeat lost!");       
    hb_count = 0;
   } 

  //===============   Check For Display Scroll Button Touch / Press
  
  #if defined Display_Support
  
    #if (defined ESP32)             
      if ( (Tup != 99) && (Tdn != 99) ) {   // if touch pin-pair enabled
        if (upButton) {
          Scroll_Display(up);
        }
        if (dnButton) {
          Scroll_Display(down);
        } 
      } else
      if ( (Pup != 99) && (Pdn != 99) ) {   // if digital pin-pair enabled
        upButton = digitalRead(Pup);
        if (!upButton) {                 // low == pressed
          Scroll_Display(up);
        }
        dnButton = digitalRead(Pdn);
        if (!dnButton) {
          Scroll_Display(down);
        }        
      }
    #endif
    
    #if (defined ESP8266)                 // digital pin interrupt is momentary, not what we need
      if ( (Pup != 99) && (Pdn != 99) ) { // 99 means pins not avaliable, maybe not enough pins
        upButton = digitalRead(Pup);
        if (!upButton) {                 // low == pressed
          Scroll_Display(up);
        }
        dnButton = digitalRead(Pdn);
        if (!dnButton) {
          Scroll_Display(down);
        }    
      }
    #endif
    
  #endif
     
  //==================== Check SPort Timeout
  
  if ((set.trmode == air) || (set.trmode == relay)) {
    if((millis() - sp_timeout_millis) > 5000) {     
      spGood = false;
    } 
    mySPort.ReportSPortOnlineStatus();  
  }
  
  //==================== Data Streaming Option
  
  #ifdef Data_Streams_Enabled 
  if(mavGood) {                      // If we have a link, request data streams from MavLink every 5s
    if(millis()-rds_millis > 5000) {
    rds_millis=millis();
    Debug.println("Requesting data streams"); 
    DisplayPrintln("Reqstg datastreams");    
    RequestDataStreams();   // must have Teensy Tx connected to Taranis/FC rx  (When SRx not enumerated)
    }
  }
  #endif 
  
  //==================== Send Our Own Heartbeat to FC
  
  if(millis()- fchb_millis > 2000) {  // MavToPass heartbeat to FC every 2 seconds
    fchb_millis=millis();
    #if defined Mav_Debug_MavToPass_Heartbeat
      Debug.println("Sending MavToPass hb to FC");  
    #endif    
   Send_FC_Heartbeat();   // must have MavToPass tx pin connected to Telem radio rx pin  
  }
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
      Debug.println("Battery capacities requested");
      DisplayPrintln("Bat mAh from FC");    
      ap_bat_paramsReq = true;
    } else {
      if (ap_bat_paramsRead &&  (!parm_msg_shown)) {
        parm_msg_shown = true; 
        Debug.println("Battery params successfully read"); 
        DisplayPrintln("Bat params read ok"); 
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
  
  #if defined SD_Support
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
     // PrintByte(c, 1);
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &F2Rmsg, &status)) {  // Read a frame
         #ifdef  Debug_FC_Down
           Debug.println("Serial passed to RB from FC side :");
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
          Debug.print("Read from FC Bluetooth to Ringbuffer: msgReceived=" ); Debug.println(msgReceived);
          PrintMavBuffer(&F2Rmsg);
        #endif      
      }
    return true;  
    }   
  #endif

  #if (defined wifiBuiltin)
    if (set.fc_io == fc_wifi)  {  //  WiFi
      if (set.wfproto == tcp)  { // TCP from FC
 
        bool msgReceived = Read_TCP(&F2Rmsg);
        if (msgReceived) {
        
          MavToRingBuffer();  
          
          #ifdef  Debug_FC_Down    
            Debug.print("Read from FC WiFi TCP to Ringbuffer: msgReceived=" ); Debug.println(msgReceived);
            PrintMavBuffer(&F2Rmsg);
          #endif      
        }
       return true;  
      }
      
      if (set.wfproto == udp)  {// UDP from FC
        bool msgReceived = Read_UDP(&F2Rmsg);
       if (msgReceived) {
          
          MavToRingBuffer();   
         
          #ifdef  Debug_FC_Down   
            Debug.print("Read from FC WiFi UDP to Ringbuffer: msgReceived=" ); Debug.println(msgReceived);
            PrintMavBuffer(&F2Rmsg);
          #endif      
        }
       return true;     
      }
   
    }
  #endif 
  
 #if ((defined ESP32)  || (defined ESP8266)) && (defined SD_Support)
    if (set.fc_io == fc_sd)  {   //  SD
      mavlink_status_t status;
      if (sdStatus == 4) {      //  if open for read
        while (file.available()) {
          uint8_t c = file.read();
          if(mavlink_parse_char(MAVLINK_COMM_0, c, &F2Rmsg, &status)) {  // Parse a frame
            #ifdef  Debug_FC_Down
              Debug.println("Read from FC SD to Ringbuffer:");
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
  //   Debug.print("Mavlink ring buffer R2Gmsg: ");  
  //    PrintMavBuffer(&R2Gmsg);
      Debug.print("Ring queue = "); Debug.println(MavRingBuff.size());
    #endif
    
    Send_From_RingBuf_To_GCS();
    
    DecodeOneMavFrame();  // Decode a Mavlink frame from the ring buffer 

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
            Debug.println("Passed up from GCS Serial to G2Fmsg:");
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
            Debug.print("Passed up from GCS BT to G2Fmsg: msgReceived=" ); Debug.println(msgReceived);
            if (msgReceived) PrintMavBuffer(&G2Fmsg);
          #endif      
        }
    }  
  #endif

  #if (defined wifiBuiltin)
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) ) {   //  WiFi
    
      if (set.wfproto == tcp)  { // TCP 
    
        bool msgReceived = Read_TCP(&G2Fmsg);

        if (msgReceived) {
          GCS_available = true;  // Record waiting to go to FC 

          #ifdef  Debug_GCS_Up    
            Debug.print("Passed up from GCS WiFi TCP to G2Fmsg: msgReceived=" ); Debug.println(msgReceived);
            if (msgReceived) PrintMavBuffer(&G2Fmsg);
          #endif      
        }
      }
      
      if (set.wfproto == udp)  { // UDP from GCS
   
        bool msgReceived = Read_UDP(&G2Fmsg);

        if (msgReceived) {
          GCS_available = true;  // Record waiting to go to FC 

          #ifdef  Debug_GCS_Up    
            Debug.print("Passed up from GCS WiFi UDP to G2Fmsg: msgReceived=" ); Debug.println(msgReceived);
            if (msgReceived) PrintMavBuffer(&G2Fmsg);
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

    for (int i = 0 ; i < max_clients ; ++i) {                    // check each connected client for waiting data  
      if (NULL != clients[i]) {            // if null pointer go around
        // active client
        active_client_idx = i;
        len = clients[active_client_idx]->available();     // is there some data to read
        uint16_t tcp_count = len;

        if(tcp_count > 0) {           // if so, read until no more
          while(tcp_count--)  {
            int result = clients[active_client_idx]->read();
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
      }
      
    //  clients[active_client_idx]->flush();
    }  // end of for()

    return msgRcvd;
  }
#endif

//================================================================================================= 
#if (defined wifiBuiltin)
  bool Read_UDP(mavlink_message_t* msgptr)  {
    if (!wifiSuGood) return false;  
    bool msgRcvd = false;
    mavlink_status_t _status;
    
    len = UDP.parsePacket();

    int udp_count = len;
    if(udp_count > 0) {    
        while(udp_count--)  {

            int result = UDP.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {

                    #if (not defined UDP_Broadcast)
                      UDP_remoteIP = UDP.remoteIP();                     
                      bool in_table = false;
                      for (int i = 0 ; i < max_clients ; i++) {
                        if (UDP_remoteIP_B3[i] == UDP_remoteIP[3]) {  // already in to table
                          in_table = true;
                          break;
                        }
                      }
                      if (!in_table) {  // if not in table, add it into empty slot
                        for (int i = 0 ; i < max_clients ; i++) {
                          if (UDP_remoteIP_B3[i] == 0)  {   
                            UDP_remoteIP_B3[i] = UDP_remoteIP[3];    // remember unique last byte of remote udp client so we can target it  
                            FtRemIP = true;   
                            break;      
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
                }
            }
        }
    }
    
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
      Debug.printf("GCS to FC - msgid = %3d \n",  G2Fmsg.msgid);
    #endif
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
              
            Mavlink_Param_Request_Read(gcs_req_param_index, gcs_req_param_id); 
            
          #endif
          break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:  // #21  from GCS
          #if defined Mav_Debug_All || defined Debug_GCS_Up || defined Debug_Param_Request_List 
            gcs_target_system = mavlink_msg_param_request_list_get_target_system(&G2Fmsg);  
            gcs_target_component = mavlink_msg_param_request_list_get_target_component(&G2Fmsg);                                  

            Debug.print("Mavlink to FC: #21 Param_Request_List: ");           
            Debug.print("gcs_target_system="); Debug.print(gcs_target_system); 
            Debug.print(" gcs_target_component="); Debug.print(gcs_target_component);                    
            Debug.println();         
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
  }
}
//================================================================================================= 
void Write_To_FC(uint32_t msg_id) {
  
  if (set.fc_io == fc_ser)  {   // Serial to FC
    len = mavlink_msg_to_send_buffer(FCbuf, &G2Fmsg);
    mvSerial.write(FCbuf,len);  
         
    #if defined  Debug_FC_Up || defined Debug_GCS_Up
      if (msg_id) {    //  dont print heartbeat - too much info
        Debug.printf("Sent to FC Serial from G2Fmsg: len=%d\n", len);
        PrintMavBuffer(&G2Fmsg);
      }  
    #endif    
  }

  #if (defined btBuiltin) 
    if (set.fc_io == fc_bt)  {   // BT to FC
        bool msgSent = Send_Bluetooth(&G2Fmsg);      
        #ifdef  Debug_FC_Up
          Debug.print("Sent to FC Bluetooth from G2Fmsg: msgSent="); Debug.println(msgSent);
          if (msgSent) PrintMavBuffer(&G2Fmsg);
        #endif     
    }
  #endif

  #if (defined wifiBuiltin)
    if (set.fc_io == fc_wifi) {  // WiFi to FC
      if (wifiSuGood) { 
        if (set.wfproto == tcp)  { // TCP  
           active_client_idx = 0;             // we only ever need 1
           bool msgSent = Send_TCP(&G2Fmsg);  // to FC   
           #ifdef  Debug_GCS_Up
             Debug.print("Sent to FC WiFi TCP from G2Fmsg: msgSent="); Debug.println(msgSent);
             PrintMavBuffer(&G2Fmsg);
           #endif    
         }    
         if (set.wfproto == udp)  { // UDP 
           active_client_idx = 0;             // we only ever need 1 here   
           bool msgSent = Send_UDP(&G2Fmsg);  // to FC    
           #ifdef  Debug_GCS_Up
             Debug.print("Sent to FC WiFi UDP from G2Fmsg: msgSent="); Debug.println(msgSent);
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
        Debug.println("MavRingBuff full. Dropping records!");
     //   DisplayPrintln("Mav buffer full!"); 
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
  
//================================================================================================= 

void Send_From_RingBuf_To_GCS() {   // Down to GCS (or other) from Ring Buffer

  if ((set.gs_io == gs_ser) || (set.gs_io == gs_bt) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.gs_sd == gs_on)) {

      if (set.gs_io == gs_ser) {  // Serial
        len = mavlink_msg_to_send_buffer(GCSbuf, &R2Gmsg);
        mvSerial.write(GCSbuf,len);  

        #ifdef  Debug_GCS_Down
          Debug.printf("Sent from ring buffer to GCS Serial: len=%d\n", len);
          PrintMavBuffer(&R2Gmsg);
        #endif
      }
 

  #if (defined btBuiltin)
    if ((set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt))  {  // Bluetooth
      len = mavlink_msg_to_send_buffer(GCSbuf, &R2Gmsg);     
      #ifdef  Debug_GCS_Down
        Debug.println("Passed down from Ring buffer to GCS by Bluetooth:");
        PrintMavBuffer(&R2Gmsg);
      #endif
      if (SerialBT.hasClient()) {
        SerialBT.write(GCSbuf,len);
      }
    }
  #endif

  #if (defined wifiBuiltin)
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt)) { //  WiFi
    
      if (wifiSuGood) {

        if (set.wfproto == tcp)  { // TCP      
          for (int i = 0 ; i < max_clients ; ++i) {       // send to each active client
            if (NULL != clients[i]) {        
              // active client
              active_client_idx = i;

              bool msgSent = Send_TCP(&R2Gmsg);  // to GCS
              #ifdef  Debug_GCS_Down
                Debug.printf("From Ring buffer to GCS client %d by WiFi TCP: msgSent=%d\n", active_client_idx+1, msgSent);
                PrintMavBuffer(&R2Gmsg);
              #endif
            }
          }    
        }
        
        if (set.wfproto == udp)  { // UDP               
          
          #if defined UDP_Broadcast                     // broadcast to remote udp clients
              bool msgSent = Send_UDP(&R2Gmsg);  // to GCS
              msgSent = msgSent; // stop stupid compiler warnings                       
          #else
                   
            for (int i = 0 ; i < max_clients ; i++) {   // send to each individual remote client udp ip
              if (UDP_remoteIP_B3[i] != 0) {
                UDP_remoteIP[3] =  UDP_remoteIP_B3[i];    // target the remote IP
                bool msgSent = Send_UDP(&R2Gmsg);  // to GCS
                msgSent = msgSent; // stop stupid compiler warnings                
              }
           }  
         #endif  
         #ifdef  Debug_GCS_Down
           Debug.print("Passed down from Ring buffer to GCS by WiFi UDP: msgSent="); Debug.println(msgSent);
           PrintMavBuffer(&R2Gmsg);
         #endif         
        }                                                                     
      }  
    }
  #endif

  #if ((defined ESP32) || (defined ESP8266)) && (defined SD_Support) 
    if  (set.gs_sd == gs_on) {   //  GCS downlink telem to SD Card
      if (sdStatus == 3) {     //  if open for write
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
  
    size_t sent =  clients[active_client_idx]->write(sendbuf,len);  

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    return msgSent;
  }
#endif
//================================================================================================= 
#if (defined wifiBuiltin)
  bool Send_UDP(mavlink_message_t* msgptr) {
    if (!wifiSuGood) return false;  
    bool msgSent = false;

    if (msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      UDP_remoteIP[3] = 255;       // always broadcast a heartbeat, either from the GCS or from the FC                 
    //  Debug.print("Broadcast heartbeat UDP_remoteIP="); Debug.println(UDP_remoteIP.toString());     
    } 

    UDP.beginPacket(UDP_remoteIP, set.udp_remotePort);

    uint16_t len = mavlink_msg_to_send_buffer(sendbuf, msgptr);
  
    size_t sent = UDP.write(sendbuf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
      UDP.flush();
    }

    bool endOK = UDP.endPacket();
 //   if (!endOK) Debug.printf("msgSent=%d   endOK=%d\n", msgSent, endOK);
    return msgSent;
  }
#endif
//================================================================================================= 

void DecodeOneMavFrame() {
  
   #if defined Mav_Print_All_Msgid
     uint16_t sz = sizeof(R2Gmsg);
     Debug.printf("FC to GGS - msgid = %3d Msg size =%3d\n",  R2Gmsg.msgid, sz);
   #endif

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

          if ((ap_base_mode >> 7) && (!homGood)) 
            MarkHome();  // If motors armed for the first time, then mark this spot as home

          hb_count++; 
          
          if(!mavGood) {
            Debug.print("hb_count=");
            Debug.print(hb_count);
            Debug.println("");

            if(hb_count >= 3) {        // If  3 heartbeats from MavLink then we are connected
              mavGood=true;
              Debug.println("Mavlink good!");
              DisplayPrintln("Mavlink good!");      
              }
            }

          mySPort.PushMessage(0x5001, 0);    // Flight mode (from ap_base_mode)
          mySPort.PushMessage(0x5007, 1);    // Frame type (from ap_type)
          
          #if defined Mav_Debug_All || defined Mav_Debug_FC_Heartbeat
            Debug.print("Mavlink from FC #0 Heartbeat: ");           
            Debug.print("ap_type(frame)="); Debug.print(ap_type);   
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

           if (pitlab_flight_stack) {         
              Debug.print(" Pitlab flight stack detected");
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
             
         mySPort.PushMessage(0x5003, 0);
         
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

          mt20_row = mySPort.MatchWaitingParamRequests(ap22_param_id);
    //      Debug.printf("matched row mt20_row=%d\n", mt20_row);
          if (mt20_row != 0xffff) {       // if matched

            #if defined Mav_Debug_All || defined Debug_Mavlite
                Debug.print("Mavlink return from FC #22 Param_Value for MavLITE: ");
                Debug.print("ap22_param_id="); Debug.print(ap22_param_id);
                Debug.print(" ap22_param_value="); Debug.println(ap22_param_value, 3);             
            #endif 

            mt20[mt20_row].inuse = 0;
            mySPort.PushMessage(0x16, 0);    // MavLite PARAM_VALUE ( #22 ) use ap-param-id and ap22_param_value

            break;            
          }
          
          switch(ap22_param_index) {      // if #define Battery_mAh_Source !=1 these will never arrive
            case 356:         // Bat1 Capacity
              ap_bat1_capacity = ap22_param_value;

              mySPort.PushMessage(0x5007, 4);    // Bat1 capacity
              
              #if defined Mav_Debug_All || defined Debug_Batteries
                Debug.print("Mavlink from FC #22 Param_Value: ");
                Debug.print("bat1 capacity=");
                Debug.println(ap_bat1_capacity);
              #endif
              break;
            case 364:         // Bat2 Capacity
              ap_bat2_capacity = ap22_param_value;

              mySPort.PushMessage(0x5007, 5);    // Bat2 capacity
              
              ap_bat_paramsRead = true;
              #if defined Mav_Debug_All || defined Debug_Batteries
                Debug.print("Mavlink from FC #22 Param_Value: ");
                Debug.print("bat2 capacity=");
                Debug.println(ap_bat2_capacity);
              #endif             
              break;
          } 
             
          #if defined Mav_Debug_All || defined Mav_Debug_Params || defined Mav_List_Params || defined Debug_Mavlite
            Debug.print("Mavlink from FC #22 Param_Value: ");
            Debug.print("param_id=");
            Debug.print(ap22_param_id);
            Debug.print("  param_value=");
            Debug.print(ap22_param_value, 4);
            Debug.print("  param_count=");
            Debug.print(ap22_param_count);
            Debug.print("  param_index=");
            Debug.println(ap22_param_index);
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
            Debug.print("Mavlink from FC #24 GPS_RAW_INT: ");  
            Debug.print("ap24_fixtype="); Debug.print(ap24_fixtype);
            if (ap24_fixtype==0) Debug.print(" No GPS");
              else if (ap24_fixtype==1) Debug.print(" No Fix");
              else if (ap24_fixtype==2) Debug.print(" 2D Fix");
              else if (ap24_fixtype==3) Debug.print(" 3D Fix");
              else if (ap24_fixtype==4) Debug.print(" DGPS/SBAS aided");
              else if (ap24_fixtype==5) Debug.print(" RTK Float");
              else if (ap24_fixtype==6) Debug.print(" RTK Fixed");
              else if (ap24_fixtype==7) Debug.print(" Static fixed");
              else if (ap24_fixtype==8) Debug.print(" PPP");
              else Debug.print(" Unknown");

            Debug.print("  sats visible="); Debug.print(ap24_sat_visible);
            Debug.print("  latitude="); Debug.print((float)(ap24_lat)/1E7, 7);
            Debug.print("  longitude="); Debug.print((float)(ap24_lon)/1E7, 7);
            Debug.print("  gps alt amsl="); Debug.print((float)(ap24_amsl)/1E3, 1);
            Debug.print("  eph (hdop)="); Debug.print((float)ap24_eph);                 // HDOP
            Debug.print("  epv (vdop)="); Debug.print((float)ap24_epv);
            Debug.print("  vel="); Debug.print((float)ap24_vel / 100, 3);           // GPS ground speed (m/s)
            Debug.print("  cog="); Debug.print((float)ap24_cog / 100, 1);           // Course over ground in degrees
            //  mav2
            Debug.print("  alt_ellipsoid)="); Debug.print(ap24_alt_ellipsoid / 1000, 2);      // alt_ellipsoid in mm
            Debug.print("  h_acc="); Debug.print(ap24_h_acc);                       // Position uncertainty in mm. Positive for up.
            Debug.print("  v_acc="); Debug.print(ap24_v_acc);                       // Altitude uncertainty in mm. Positive for up.
            Debug.print("  ap24_vel_acc="); Debug.print(ap24_vel_acc);                // Speed uncertainty. Positive for up.
            Debug.print("  ap24_hdg_acc="); Debug.print(ap24_hdg_acc);                // degE5   Heading / track uncertainty 
            Debug.println();
          #endif 

           mySPort.PushMessage(0x800, 0);   // 0x800 Lat
           mySPort.PushMessage(0x800, 1);   // 0x800 Lon
           mySPort.PushMessage(0x5002, 0);  // 0x5002 GPS Status
           mySPort.PushMessage(0x5004, 0);  // 0x5004 Home         
              
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
            Debug.print("Mavlink from FC #26 Scaled_IMU: ");
            Debug.print("xacc="); Debug.print((float)ap26_xacc / 1000, 3); 
            Debug.print("  yacc="); Debug.print((float)ap26_yacc / 1000, 3); 
            Debug.print("  zacc="); Debug.print((float)ap26_zacc / 1000, 3);
            Debug.print("  xgyro="); Debug.print((float)ap26_xgyro / 1000, 3); 
            Debug.print("  ygyro="); Debug.print((float)ap26_ygyro / 1000, 3); 
            Debug.print("  zgyro="); Debug.print((float)ap26_zgyro / 1000, 3);
            Debug.print("  xmag="); Debug.print((float)ap26_xmag / 1000, 3); 
            Debug.print("  ymag="); Debug.print((float)ap26_ymag / 1000, 3); 
            Debug.print("  zmag="); Debug.print((float)ap26_zmag / 1000, 3);  
            Debug.print("  temp="); Debug.println((float)ap26_temp / 100, 2);    // cdegC                              
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
          ap26_temp = mavlink_msg_scaled_imu_get_temperature(&R2Gmsg);           
          #if defined Mav_Debug_All || defined Mav_Debug_Raw_IMU
            Debug.print("Mavlink from FC #27 Raw_IMU: ");
            Debug.print("accX="); Debug.print((float)ap27_xacc / 1000); 
            Debug.print("  accY="); Debug.print((float)ap27_yacc / 1000); 
            Debug.print("  accZ="); Debug.println((float)ap27_zacc / 1000);
            Debug.print("  xgyro="); Debug.print((float)ap27_xgyro / 1000, 3); 
            Debug.print("  ygyro="); Debug.print((float)ap27_ygyro / 1000, 3); 
            Debug.print("  zgyro="); Debug.print((float)ap27_zgyro / 1000, 3);
            Debug.print("  xmag="); Debug.print((float)ap27_xmag / 1000, 3); 
            Debug.print("  ymag="); Debug.print((float)ap27_ymag / 1000, 3); 
            Debug.print("  zmag="); Debug.print((float)ap27_zmag / 1000, 3);
            Debug.print("  id="); Debug.print((float)ap27_id);             
            Debug.print("  temp="); Debug.println((float)ap27_temp / 100, 2);    // cdegC               
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
      
          mySPort.PushMessage(0x5006, 0 );  // 0x5006 Attitude      

          break;  
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:     // #33
          if ((!mavGood) || (ap24_fixtype < 3)) break;  
          ap33_lat = mavlink_msg_global_position_int_get_lat(&R2Gmsg);             // Latitude, expressed as degrees * 1E7
          ap33_lon = mavlink_msg_global_position_int_get_lon(&R2Gmsg);             // Pitch angle (rad, -pi..+pi)
          ap33_amsl = mavlink_msg_global_position_int_get_alt(&R2Gmsg);          // Altitude above mean sea level (millimeters)
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
            Debug.print("Mavlink from FC #33 GPS Int: ");
            Debug.print(" ap_lat="); Debug.print((float)ap33_lat / 1E7, 6);
            Debug.print(" ap_lon="); Debug.print((float)ap33_lon / 1E7, 6);
            Debug.print(" ap_amsl="); Debug.print((float)ap33_amsl / 1E3, 0);
            Debug.print(" ap33_alt_ag="); Debug.print((float)ap33_alt_ag / 1E3, 1);           
            Debug.print(" ap33_vx="); Debug.print((float)ap33_vx / 100, 2);
            Debug.print(" ap33_vy="); Debug.print((float)ap33_vy / 100, 2);
            Debug.print(" ap33_vz="); Debug.print((float)ap33_vz / 100, 2);
            Debug.print(" ap33_gps_hdg="); Debug.println((float)ap33_gps_hdg / 100, 1);
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
            #if (not defined Rssi_Pacemaker)
              mySPort.PushMessage(0xF101, 0);   // 0xF101 RSSI 
              rssi_millis = millis();
            #endif  
                          
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
            mySPort.PushMessage(0x50F1, 0);   // 0x50F1  SERVO_OUTPUT_RAW
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

            mySPort.PushMessage(0x5007, 6);    // fr parameters - mission count
            
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
              mySPort.PushMessage(0x5009, 0);  // 0x5009 Waypoints  
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

              #if (not defined Rssi_Pacemaker)
                mySPort.PushMessage(0xF101, 0);   // 0xF101 RSSI 
                rssi_millis = millis();    
              #endif           
                           
              #if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_RC
                #ifndef RSSI_Override
                  Debug.print("Auto RSSI_Source===>  ");
                #endif
              #endif     
              }
             
            #if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_RC
              Debug.print("Mavlink from FC #65 RC_Channels: ");
              Debug.print("ap65_chcnt="); Debug.print(ap65_chcnt); 
              Debug.print(" PWM: ");
              for (int i=0 ; i < ap65_chcnt ; i++) {
                Debug.print(" "); 
                Debug.print(i+1);
                Debug.print("=");  
                Debug.print(ap65_chan_raw[i]);   
              }                         
              Debug.print("  ap65_rssi=");  Debug.print(ap65_rssi); 
              Debug.print("  rssiGood=");  Debug.println(rssiGood);         
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
 
          Debug.print("Mavlink from FC #73 Mission_Item_Int: ");
          Debug.print("target_system ="); Debug.print(ap_targsys);   
          Debug.print("  target_component ="); Debug.print(ap_targcomp);   
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

        case MAVLINK_MSG_ID_COMMAND_ACK:       // #77   received back after #76 Command_Long

          if (!mavGood) break; 
          ap77_command =  mavlink_msg_command_ack_get_command(&R2Gmsg);   
          ap77_result =  mavlink_msg_command_ack_get_result(&R2Gmsg);    
          ap77_progress =  mavlink_msg_command_ack_get_progress(&R2Gmsg);   
          ap77_result_param2 =  mavlink_msg_command_ack_get_result_param2(&R2Gmsg); 
          ap77_target_system =  mavlink_msg_command_ack_get_target_system(&R2Gmsg);
          ap77_target_component =  mavlink_msg_command_ack_get_target_component(&R2Gmsg);  
          
         #if defined Mav_Debug_All || defined Mav_Debug_Command_Ack                  
          Debug.print("Mavlink from FC #77 Command_Ack: ");
          Debug.print("ap77_command ="); Debug.print(ap77_command);   
          Debug.print(" ap77_result ="); Debug.print(ap77_result);   
          Debug.print(" ap77_progress ="); Debug.print(ap77_progress);   
          Debug.print(" ap77_result_param2 ="); Debug.print(ap77_result_param2);     
          Debug.print(" ap77_target_system ="); Debug.print(ap77_target_system);   
          Debug.print(" ap77_target_component ="); Debug.println(ap77_target_component);                                                    
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
            Debug.print("Mavlink from FC #74 VFR_HUD: ");
            Debug.print("Airspeed= "); Debug.print(ap74_air_spd, 2);                 // m/s    
            Debug.print("  Groundspeed= "); Debug.print(ap74_grd_spd, 2);            // m/s
            Debug.print("  Heading= ");  Debug.print(ap74_hdg);                      // deg
            Debug.print("  Throttle %= ");  Debug.print(ap74_throt);                 // %
            Debug.print("  Baro alt= "); Debug.print(ap74_amsl, 0);                  // m                  
            Debug.print("  Climb rate= "); Debug.println(ap74_climb);                // m/s
          #endif  

          mySPort.PushMessage(0x5005, 0);  // 0x5005 VelYaw

          #if defined PlusVersion
            mySPort.PushMessage(0x50F2, 0);  // 0x50F2 VFR HUD
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

            #if (not defined Rssi_Pacemaker)
              mySPort.PushMessage(0xF101, 0);   // 0xF101 RSSI 
              rssi_millis = millis();    
            #endif         
           
            #if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_RC
              #ifndef RSSI_Override
                Debug.print("Auto RSSI_Source===>  ");
              #endif
            #endif     

            #if defined Mav_Debug_All || defined Debug_Radio_Status || defined Mav_Debug_Rssi
              Debug.print("Mavlink from FC #109 Radio: "); 
              Debug.print("ap109_rssi="); Debug.print(ap109_rssi);
              Debug.print("  remrssi="); Debug.print(ap109_remrssi);
              Debug.print("  txbuf="); Debug.print(ap109_txbuf);
              Debug.print("  noise="); Debug.print(ap109_noise); 
              Debug.print("  remnoise="); Debug.print(ap109_remnoise);
              Debug.print("  rxerrors="); Debug.print(ap109_rxerrors);
              Debug.print("  fixed="); Debug.print(ap109_fixed);  
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
          ap147_battery_remaining = mavlink_msg_battery_status_get_battery_remaining(&R2Gmsg);  // (0%: 0, 100%: 100)  

          if (ap_battery_id == 0) {  // Battery 1
            pt_bat1_mAh = ap_current_consumed;                       
          } else if (ap_battery_id == 1) {  // Battery 2
              pt_bat2_mAh = ap_current_consumed;                              
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

          mySPort.PushMessage(0x5006, 0);  // 0x5006 Rangefinder
             
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

          mySPort.PushMessage(0x5008, 0);   // 0x5008 Bat2       
                   
          break;
          
        case MAVLINK_MSG_ID_AHRS3:            // #182   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break;       
          break;
        case MAVLINK_MSG_ID_RPM:              // #226   https://mavlink.io/en/messages/ardupilotmega.html
          if (!mavGood) break; 
          ap_rpm1 = mavlink_msg_rpm_get_rpm1(&R2Gmsg); 
          ap_rpm2 = mavlink_msg_rpm_get_rpm2(&R2Gmsg);                    
          #if (defined Mav_Debug_RPM) || (defined Mav_Debug_All) 
            Debug.print("Mavlink from FC #226 RPM: ");
            Debug.print("RPM1= "); Debug.print(ap_rpm1, 0); 
            Debug.print("  RPM2= ");  Debug.println(ap_rpm2, 0);          
          #endif       
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

          mySPort.PushMessage(0x5000, 0);         // 0x5000 StatusText Message
          
          break;                                      
        default:
          if (!mavGood) break;
          #if defined Mav_Debug_All || defined Mav_Show_Unknown_Msgs
            Debug.print("Mavlink from FC: ");
            Debug.print("Unknown Message ID #");
            Debug.print(R2Gmsg.msgid);
            Debug.println(" Ignored"); 
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
    Debug.print("******************************************Mavlink in #33 GPS Int: Home established: ");       
    Debug.print("hom.lat=");  Debug.print(hom.lat, 7);
    Debug.print(" hom.lon=");  Debug.print(hom.lon, 7 );        
    Debug.print(" hom.alt="); Debug.print(hom.alt, 1);
    Debug.print(" hom.hdg="); Debug.println(hom.hdg);                   
 #endif  
}
//================================================================================================= 
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
//================================================================================================= 
void Mavlink_Param_Request_Read(int16_t param_index, char * param_id) {  // #20
  ap_sysid = 20;                        // ID 20 for this aircraft
  ap_compid = 1;                        //  autopilot = 1

  mavlink_msg_param_request_read_pack(ap_sysid, ap_compid, &G2Fmsg,
                   ap_targsys, ap_targcomp, param_id, param_index);              
  Write_To_FC(20);             
 }

//================================================================================================= 
 void Request_Param_List() {

  ap_sysid = 20;                        // ID 20 for this aircraft
  ap_compid = 1;                        //  autopilot1
  
  mavlink_msg_param_request_list_pack(ap_sysid,  ap_compid, &G2Fmsg,
                    ap_targsys,  ap_targcomp);
              
  Write_To_FC(21);
                    
 }
 //================================================================================================= 
 void Mavlink_Param_Set() {    // #23

  ap_sysid = 20;                        // ID 20 for this aircraft
  ap_compid = 1;                        //  autopilot = 1
                                          
  mavlink_msg_param_set_pack(ap_sysid, ap_compid, &G2Fmsg,
                        ap_targsys, ap_targcomp, ap23_param_id, ap23_param_value, 10);  // 10=MAV_PARAM_TYPE_REAL64 https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE            
  Write_To_FC(23);  // #23                
 }
//================================================================================================= 
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
 
//================================================================================================= 
#if defined Request_Missions_From_FC || defined Request_Mission_Count_From_FC
void Mavlink_Request_Mission_List() {   // #43   get back #44 Mission_Count
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
//================================================================================================= 
void Mavlink_Command_Long() {  // #76

  ap_sysid = 20;                        // ID 20 for this aircraft
  ap_compid = 1;                        //  autopilot = 1    

  mavlink_msg_command_long_pack(ap_sysid, ap_compid, &G2Fmsg,
          ap_targsys, ap_targcomp, ap76_command, ap76_confirm, ap76_param[0], ap76_param[1], 
          ap76_param[2], ap76_param[3], ap76_param[4], ap76_param[5], ap76_param[6]); 
     
  Write_To_FC(76);  // #76                
 }
 //================================================================================================= 
 
