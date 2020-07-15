#include <Arduino.h>

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
  4. This notice may not be removed or altered from any distribution.  

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
    2) Mavlink_In  <--rx2 Pin 9    Mavlink source to Teensy - FC_Mav_rxPin
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

void OledPrintln(String);
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
void Param_Request_Read(int16_t, char *);
void RequestMissionList();
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

void ServiceMavStatusLed();
void ServiceBufStatusLed();
void BlinkMavLed(uint32_t);
void DisplayRemoteIP();
bool Leap_yr(uint16_t);
void WebServerSetup();
void RecoverSettingsFromFlash();

void PrintPeriod(bool);
void PrintFrPeriod(bool);
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
//                                 S E T U P   O L E D
//=================================================================================================
  #if ((defined ESP32) || (defined ESP8266)) && (defined OLED_Support) 
    Wire.begin(SDA, SCL);
    display.begin(SSD1306_SWITCHCAPVCC, i2cAddr);  
    display.clearDisplay();
  
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
  
    Debug.println("OLED Support activated");
    OledPrintln("Starting .... ");
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
      OledPrintln("EEPROM fatal error!");
      while (true) delay(100);  // wait here forever 
     } else {
      Debug.println("EEPROM initialised successfully");
      OledPrintln("EEPROM good"); 
     }
  #endif       
    
  #if defined ESP8266
    EEPROM.begin(EEPROM_SIZE);
    Debug.println("EEPROM initialised successfully");
    OledPrintln("EEPROM good"); 
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
    OledPrintln("Teensy 3.x");
  #elif (defined ESP32) //  ESP32 Board
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
    
  #elif (defined ESP8266) 
    Debug.print("ESP8266 / Variant is ");
    OledPrintln("ESP8266 / Variant is");  
    #if (ESP8266_Variant == 1)
      Debug.println("Lonlin Node MCU 12F");
      OledPrintln("Node MCU 12");
    #endif 
    #if (ESP8266_Variant == 2)
      Debug.println("ESP-F - RFD900X TX-MOD");
      OledPrintln("RFD900X TX-MOD");
    #endif   
    #if (ESP8266_Variant == 3)
      Debug.println("Wemos® ESP-12F D1 Mini");
      OledPrintln("Wemos ESP-12F D1 Mini");
    #endif       
  #endif

  if (set.trmode == ground) {
    Debug.println("Ground Mode");
    OledPrintln("Ground Mode");
  } else  
  if (set.trmode == air) {
    Debug.println("Air Mode");
    OledPrintln("Air Mode");
  } else
  if (set.trmode == relay) {
    Debug.println("Relay Mode");
    OledPrintln("Relay Mode");
  }

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

  #ifndef RSSI_Override
    Debug.println("RSSI Automatic Select");
    OledPrintln("RSSI Auto Select");     
  #else
    Debug.println("RSSI Override for testing = 70%");
    OledPrintln("RSSI Override = 70%");              // for debugging          
  #endif

  if (set.fc_io == fc_ser)   {
    Debug.println("Mavlink Serial In");
    OledPrintln("Mavlink Serial In");
  }

  if (set.gs_io == gs_ser)  {
    Debug.println("Mavlink Serial Out");
    OledPrintln("Mavlink Serial Out");
  }

  if (set.fc_io == fc_bt)  {
    Debug.println("Mavlink Bluetooth In");
    OledPrintln("Mavlink BT In");
  } 

  if (set.gs_io == gs_bt)  {
    Debug.println("Mavlink Bluetooth Out");
    OledPrintln("Mavlink BT Out");
  }

  if (set.fc_io == fc_wifi)  {
    Debug.println("Mavlink WiFi In");
    OledPrintln("Mavlink WiFi In");
  } 

  if (set.gs_io == gs_wifi)  {
    Debug.print("Mavlink WiFi Out - ");
    OledPrintln("Mavlink WiFi Out");
  }

  if (set.gs_io == gs_wifi_bt)  {
    Debug.print("Mavlink WiFi+BT Out - ");
    OledPrintln("Mavlink WiFi+BT Out");
  }

  if ((set.fc_io == fc_wifi) || (set.gs_io == gs_wifi) ||  (set.gs_io == gs_wifi_bt) || (set.web_support)) {
   if (set.wfproto == tcp)  {
     Debug.println("Protocol is TCP/IP");
     OledPrintln("Protocol is TCP/IP");
   }
   else if  (set.wfproto == udp) {
     Debug.println("Protocol is UDP");
     OledPrintln("Protocol is UDP");
   }
  }
  #if defined SD_Support                    
    if (set.fc_io == fc_sd) {
      Debug.println("Mavlink SD In");
      OledPrintln("Mavlink SD In");
   }

    if (set.gs_sd == gs_on) {
      Debug.println("Mavlink SD Out");
      OledPrintln("Mavlink SD Out");
    }
  #endif
 
//=================================================================================================   
//                                S E T U P   W I F I  --  E S P only
//=================================================================================================

  #if (defined wifiBuiltin)
    if ((set.fc_io == fc_wifi) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.web_support)) {     
      #if (not defined Start_WiFi)
      pinMode(startWiFiPin, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(startWiFiPin), [] {if (wifiButnPres+= (millis() - debnceTimr) >= (delaytm)) debnceTimr = millis();}, RISING);
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
        OledPrintln("BT connecting ......");   
        SerialBT.begin(set.host, true);      
        // Connect(address) is relatively fast (10 secs max), connect(name) is slow (30 secs max) as it 
        // needs to resolve the name to an address first.
        // Set CoreDebugLevel to Info to view devices bluetooth addresses and device names
        bool bt_connected;
        bt_connected = SerialBT.connect(set.btConnectToSlave);
        if(bt_connected) {
          Debug.println("Bluetooth done");  
          OledPrintln("BT done");
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
  
    Debug.println("SD Support activated");
    OledPrintln("SD support activated");

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
          Debug.println("No SD card found");
          OledPrintln("No SD card");
          OledPrintln("Ignoring!");      
      } else {
        Debug.println("SD card found");
        OledPrintln("SD card found");
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
           OledPrintln("Reading SD card");
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

  if (set.fc_io == fc_ser)  {  //  Serial
    #if defined AutoBaud
      set.baud = GetBaud(FC_Mav_rxPin);
    #endif   
    #if (defined ESP32)   
      // system can wait here for a few seconds (timeout) if there is no telemetry in
      mvSerialFC.begin(set.baud, SERIAL_8N1, FC_Mav_rxPin, FC_Mav_txPin);   //  rx,tx, cts, rts
    #else
      mvSerialFC.begin(set.baud);    
    #endif 
    Debug.printf("Mavlink serial input on pins rx = %d and tx = %d\n", FC_Mav_rxPin, FC_Mav_txPin); 
  } 

  mySPort.initialise();
  
  
   #if defined Enable_GCS_Serial         // Only Teensy 3.x or Maple Mini have 4 uarts , NOT ESP
    if (set.gs_io == gs_ser)  {          //  GCS Serial
      mvSerialGCS.begin(mvBaudGCS);
      Debug.printf("Mavlink serial output on pins rx = %d and tx = %d\n", GC_Mav_rxPin, GC_Mav_txPin);
    }
  #endif

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
//                                        L  O  O  P
//================================================================================================= 
void loop() {            // For WiFi STA only

  #if (defined wifiBuiltin)
    if ((set.fc_io == fc_wifi) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.web_support)) {  
      
      SenseWiFiStatus();

      if (set.wfproto == tcp)  {  // TCP  
        if (wifiSuGood) {
          TCPclient = TCPserver.available();              // listen for incoming clients 
          if(TCPclient) {
            Debug.println("New client connected"); 
            OledPrintln("New client ok!");  



                
            while ((TCPclient) && (TCPclient.connected())) {            // loop while the client's connected
       //       if (!clientGood) TCPclient.setTimeout(4);  
              clientGood = true;  
              main_loop(); 
            }
          nbdelay(100);  
          TCPclient.stop();
          nbdelay(100);  
          Debug.println("Client disconnected XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
          OledPrintln("Client disconnected!");  
          } else {
             clientGood = false;
             main_loop();
         } 
        }  else { 
           main_loop();
        }  
      }
  
     if (set.wfproto == udp)  {  // UDP  
        main_loop();       
       } 
  
    else 
      main_loop();
    } else {
      main_loop();
    }
  #else 
    main_loop();
  #endif
}
//================================================================================================= 
//================================================================================================= 
//                                   M  A  I  N    L  O  O  P
//================================================================================================= 
//================================================================================================= 

void main_loop() {

  #if defined Debug_Loop_Period
    PrintLoopPeriod();
  #endif
  
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_WiFi)
    if (clientGood != clientPrev) {
      Debug.printf("clientGood=%d\n", clientGood);
      clientPrev = clientGood;
    }
  #endif

  #if (defined wifiBuiltin)
    SenseWiFiStatus();  // Optional start-wifi pin
  #endif

  //==================== F l i g h t   C o m p u t e r   T o   R i n g   B u f f e r
  
  if (!Read_FC_To_RingBuffer()) {  //  check for SD eof
    if (sdStatus == 5) {
      Debug.println("End of SD file");
      OledPrintln("End of SD file");  
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
    Debug.println("Heartbeat from GCS timed out! GCS not connected"); 
    OledPrintln("GCS Heartbeat lost!"); 
    
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
    OledPrintln("FC heartbeat lost!");       
    hb_count = 0;
   } 
   
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
    OledPrintln("Reqstg datastreams");    
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
      RequestMissionList();  //  #43
      ap_ms_list_req = true;
    }
  }
  #endif
  
  //==================== Get battery capacity from the FC Option
  
  #if (Battery_mAh_Source == 1)  
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
  
  //====================   List All Parameters From The FC Option
  
  #ifdef Mav_List_Params
    if(mavGood && (!ap_paramsList)) {
      Request_Param_List();
      ap_paramsList = true;
    }
  #endif 
  
  //====================   Open an SD File Here Because We Waited For The Time From FC
  
  #if defined GCS_Mavlink_SD
    void OpenSDForWrite();  // fwd define
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

    while(mvSerialFC.available()) { 
      byte c = mvSerialFC.read();
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

     bool msgRcvdBT = Read_Bluetooth(&F2Rmsg);

     if (msgRcvdBT) {

        MavToRingBuffer();      

        #ifdef  Debug_FC_Down   
          Debug.print("BT passed to RB from FC side: msgRcvdBT=" ); Debug.println(msgRcvdBT);
          PrintMavBuffer(&F2Rmsg);
        #endif      
      }
    return true;  
    }   
  #endif

  #if (defined wifiBuiltin)
    if (set.fc_io == fc_wifi)  {  //  WiFi
    
      if (set.wfproto == tcp)  { // TCP from FC
    
        bool msgRcvdWF = Read_TCP(&F2Rmsg);

        if (msgRcvdWF) {
        
          MavToRingBuffer();  
          
          #ifdef  Debug_FC_Down    
            Debug.print("Passed down from FC WiFi TCP to F2Rmsg: msgRcvdWF=" ); Debug.println(msgRcvdWF);
            PrintMavBuffer(&G2Fmsg);
          #endif      
        }
       return true;  
      }
      
      if (set.wfproto == udp)  {// UDP from FC
    
        bool msgRcvdWF = Read_UDP(&F2Rmsg);

      if (msgRcvdWF) {
          
          MavToRingBuffer();   
         
          #ifdef  Debug_FC_Down   
            Debug.print("Passed down from FC WiFi UDP to F2Rmsg: msgRcvdWF=" ); Debug.println(msgRcvdWF);
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
  
  #if defined Enable_GCS_Serial  // only these have a 4th uart
    if (set.gs_io == gs_ser)  {  // Serial 
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
     } 
  #endif

  #if (defined btBuiltin) 
    if ((set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt)) {  // Bluetooth
 
       bool msgRcvdBT = Read_Bluetooth(&G2Fmsg);

       if (msgRcvdBT) {
          GCS_available = true;  // Record waiting to go to FC 
          #ifdef  Debug_GCS_Up    
            Debug.print("Passed up from GCS BT to G2Fmsg: msgRcvdBT=" ); Debug.println(msgRcvdBT);
            PrintMavBuffer(&G2Fmsg);
          #endif      
        }
    }  
  #endif

  #if (defined wifiBuiltin)
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.web_support)) {   //  WiFi
    
      if (set.wfproto == tcp)  { // TCP 
    
        bool msgRcvdWF = Read_TCP(&G2Fmsg);

        if (msgRcvdWF) {
          GCS_available = true;  // Record waiting to go to FC 

          #ifdef  Debug_GCS_Up    
            Debug.print("Passed up from GCS WiFi TCP to G2Fmsg: msgRcvdWF=" ); Debug.println(msgRcvdWF);
            PrintMavBuffer(&G2Fmsg);
          #endif      
        }
      }
      
      if (set.wfproto == udp)  { // UDP from GCS
        bool msgRcvdWF = Read_UDP(&G2Fmsg);

        if (msgRcvdWF) {
          GCS_available = true;  // Record waiting to go to FC 

          #ifdef  Debug_GCS_Up    
            Debug.print("Passed up from GCS WiFi UDP to G2Fmsg: msgRcvdWF=" ); Debug.println(msgRcvdWF);
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
    if ( (!wifiSuGood) || (!clientGood) ) return false;  
    bool msgRcvd = false;
    mavlink_status_t _status;
    
    len = TCPclient.available();
    uint16_t tcp_count = len;
    if(tcp_count > 0) {

        while(tcp_count--)  {
            int result = TCPclient.read();
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
                  
                    udp_remoteIP = UDP.remoteIP();  // remember which remote client sent this packet so we can target it
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
              
            Param_Request_Read(gcs_req_param_index, gcs_req_param_id); 
            
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
    mvSerialFC.write(FCbuf,len);  
         
    #if defined  Debug_FC_Up || defined Debug_GCS_Up
      if (msg_id) {    //  dont print heartbeat - too much info
        Debug.println("Written to FC Serial from G2Fmsg:");
        PrintMavBuffer(&G2Fmsg);
      }  
    #endif    
  }

  #if (defined btBuiltin) 
    if (set.fc_io == fc_bt)  {   // BT to FC
        bool msgSent = Send_Bluetooth(&G2Fmsg);      
        #ifdef  Debug_FC_Up
          Debug.print("Sent to FC Bluetooth from G2Fmsg: msgSent="); Debug.println(msgSent);
          PrintMavBuffer(&R2Gmsg);
        #endif     
    }
  #endif

  #if (defined wifiBuiltin)
    if (set.fc_io == fc_wifi) {  // WiFi to FC
      if (wifiSuGood) { 
        if (set.wfproto == tcp)  { // TCP  
           bool msgSent = Send_TCP(&G2Fmsg);  // to FC   
           #ifdef  Debug_GCS_Up
             Debug.print("Sent to FC WiFi TCP from G2Fmsg: msgSent="); Debug.println(msgSent);
             PrintMavBuffer(&R2Gmsg);
           #endif    
         }    
         if (set.wfproto == udp)  { // UDP 
           bool msgRead = Send_UDP(&G2Fmsg);  // to FC    
           #ifdef  Debug_GCS_Up
             Debug.print("Sent to FC WiFi UDP from G2Fmsg: magRead="); Debug.println(msgRead);
             PrintMavBuffer(&G2Fmsg);
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
  
//================================================================================================= 

void Send_From_RingBuf_To_GCS() {   // Down to GCS (or other) from Ring Buffer

  if ((set.gs_io == gs_ser) || (set.gs_io == gs_bt) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.gs_sd == gs_on)) {

    #if ((defined TEENSY3X) || (defined MAPLE_MINI)) && (defined Enable_GCS_Serial)   // only these have a 4th uart  - consider using softwareserial
      if (set.gs_io == gs_ser) {  // Serial
        len = mavlink_msg_to_send_buffer(GCSbuf, &R2Gmsg);
        #ifdef  Debug_GCS_Down
          Debug.println("Passed down from Ring buffer to GCS by Serial:");
          PrintMavBuffer(&R2Gmsg);
        #endif
         mvSerialGCS.write(GCSbuf,len);  
      }
    #endif  

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
          bool sentOK = Send_TCP(&R2Gmsg);  // to GCS
          #ifdef  Debug_GCS_Down
            Debug.print("Passed down from Ring buffer to GCS by WiFi TCP: sentOk="); Debug.println(sentOk);
            PrintMavBuffer(&R2Gmsg);
          #endif
        }
        
        if (set.wfproto == udp)  { // UDP 
          bool msgSent = Send_UDP(&R2Gmsg);  // to GCS
          msgSent = msgSent; // stop stupid compiler warnings
          #ifdef  Debug_GCS_Down
            Debug.print("Passed down from Ring buffer to GCS by WiFi UDP: msgSent="); Debug.println(msgSent);
            PrintMavBuffer(&R2Gmsg);
          #endif                 
        }                                                                     
      }  
    }
  #endif

  #if ((defined ESP32) || (defined ESP8266)) && (defined SD_Support) 
    if  (set.gs_sd == gs_on) {   //  SD Card
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
    uint8_t buf[300];
     
    uint16_t len = mavlink_msg_to_send_buffer(buf, msgptr);
  
    size_t sent = SerialBT.write(buf,len);

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
    if ( (!wifiSuGood) || (!clientGood) ) return false;   
    bool msgSent = false;
    uint8_t buf[300];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msgptr);
  
    size_t sent =  TCPclient.write(buf,len);  

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
    uint8_t buf[300];

    UDP.beginPacket(udp_remoteIP, set.udp_remotePort);

    uint16_t len = mavlink_msg_to_send_buffer(buf, msgptr);
  
    size_t sent = UDP.write(buf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    UDP.endPacket();
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
    
        case MAVLINK_MSG_ID_HEARTBEAT:    // #0   http://mavlink.org/messages/common
          ap_type_tmp = mavlink_msg_heartbeat_get_type(&R2Gmsg);   // Alex - don't contaminate the ap-type variable
          if (ap_type_tmp == 5 || ap_type_tmp == 6 || ap_type_tmp == 18 || ap_type_tmp == 27) break;      
          // Ignore heartbeats from GCS (6) or Ant Trackers(5) or Onboard_Controllers(18) or ADSB (27))
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
              OledPrintln("Mavlink good!");      
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

          mt20_row = mySPort.MatchWaitingParamRequests(ap_param_id);
    //      Debug.printf("matched row mt20_row=%d\n", mt20_row);
          if (mt20_row != 0xffff) {       // if matched

            #if defined Mav_Debug_All || defined Debug_MavLite
                Debug.print("Mavlink return from FC #22 Param_Value for MavLITE: ");
                Debug.print("ap_param_id="); Debug.print(ap_param_id);
                Debug.print(" ap_param_value="); Debug.println(ap_param_value, 3);             
            #endif 

            mt20[mt20_row].inuse = 0;
            mySPort.PushMessage(0x16, 0);    // MavLite PARAM_VALUE ( #22 ) use ap-param-id and ap_param_value

            break;            
          }
          
          switch(ap_param_index) {      // if #define Battery_mAh_Source !=1 these will never arrive
            case 356:         // Bat1 Capacity
              ap_bat1_capacity = ap_param_value;

              mySPort.PushMessage(0x5007, 4);    // Bat1 capacity
              
              #if defined Mav_Debug_All || defined Debug_Batteries
                Debug.print("Mavlink from FC #22 Param_Value: ");
                Debug.print("bat1 capacity=");
                Debug.println(ap_bat1_capacity);
              #endif
              break;
            case 364:         // Bat2 Capacity
              ap_bat2_capacity = ap_param_value;

              mySPort.PushMessage(0x5007, 5);    // Bat2 capacity
              
              ap_bat_paramsRead = true;
              #if defined Mav_Debug_All || defined Debug_Batteries
                Debug.print("Mavlink from FC #22 Param_Value: ");
                Debug.print("bat2 capacity=");
                Debug.println(ap_bat2_capacity);
              #endif             
              break;
          } 
             
          #if defined Mav_Debug_All || defined Mav_Debug_Params || defined Mav_List_Params || defined Debug_MavLite
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
              #if defined Rssi_In_Percent
                ap_rssi = ap_rssi65;          //  Percent
              #else           
                ap_rssi = ap_rssi65 / 2.54;  // 254 -> 100%
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
              Debug.print("ap_chcnt="); Debug.print(ap_chcnt); 
              Debug.print(" PWM: ");
              for (int i=0 ; i < ap_chcnt ; i++) {
                Debug.print(" "); 
                Debug.print(i+1);
                Debug.print("=");  
                Debug.print(ap_chan_raw[i]);   
              }                         
              Debug.print("  ap_rssi65=");  Debug.print(ap_rssi65); 
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
          ap_hud_amsl = mavlink_msg_vfr_hud_get_alt(&R2Gmsg);              //  m
          ap_hud_climb = mavlink_msg_vfr_hud_get_climb(&R2Gmsg);              //  m/s

          cur.hdg = ap_hud_hdg;
          
         #if defined Mav_Debug_All || defined Mav_Debug_Hud
            Debug.print("Mavlink from FC #74 VFR_HUD: ");
            Debug.print("Airspeed= "); Debug.print(ap_hud_air_spd, 2);                 // m/s    
            Debug.print("  Groundspeed= "); Debug.print(ap_hud_grd_spd, 2);            // m/s
            Debug.print("  Heading= ");  Debug.print(ap_hud_hdg);                      // deg
            Debug.print("  Throttle %= ");  Debug.print(ap_hud_throt);                 // %
            Debug.print("  Baro alt= "); Debug.print(ap_hud_amsl, 0);               // m                  
            Debug.print("  Climb rate= "); Debug.println(ap_hud_climb);                // m/s
          #endif  

          mySPort.PushMessage(0x5005, 0);  // 0x5005 VelYaw

          #if defined PlusVersion
            mySPort.PushMessage(0x50F2, 0);  // 0x50F2 VFR HUD
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
            #ifdef QLRS 
              ap_rssi = ap_remrssi;        // QRLS uses remote rssi - patch from giocomo892
            #else
              ap_rssi = ap_rssi109;        //  254 -> 100% (default) or percent (option)
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
          ap147_battery_remaining = mavlink_msg_battery_status_get_battery_remaining(&R2Gmsg);  // (0%: 0, 100%: 100)  

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
void Param_Request_Read(int16_t param_index, char * param_id) {
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



//=================================================================================================  
//================================================================================================= 
//
//                                       U T I L I T I E S
//
//================================================================================================= 
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
void PrintByte(byte b, bool LF) {  // line feed
  if ((b == 0x7E) && (LF)) {
    Debug.println();
  }
  if (b<=0xf) Debug.print("0");
  Debug.print(b,HEX);
  if (pb_rx) {
    Debug.print("<");
  } else {
    Debug.print(">");
  }
}
void PrintByteNon(byte b) {
  if (b<=0xf) Debug.print("0");
  Debug.print(b,HEX);
  Debug.print(" ");
}
void PrintByteOut(byte b) {
  if ((b == 0x7E) || (b == 0x10)  || (b == 0x32)) {
    Debug.println();
  } 
  if (b<=0xf) Debug.print("0");
  Debug.print(b,HEX);
  Debug.print(">");
}
void PrintByteIn(byte b) {
  if ((b == 0x7E) || (b == 0x10)  || (b == 0x32)) {
    Debug.println();
  } 
  if (b<=0xf) Debug.print("0");
  Debug.print(b,HEX);
  Debug.print("<");
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
      PrintByte(bytes[0], 0);   // CRC1
      PrintByte(bytes[1], 0);   // CRC2
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
      PrintByte(bytes[i], 0); 
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
     PrintByte(bytes[i], 0);     
    }
    if (j == -2) {
      Debug.print("//");
      PrintByte(bytes[mav_len + 8], 0); 
      PrintByte(bytes[mav_len + 9], 0); 
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
      PrintByte(bytes[0], 0);   // CRC1
      PrintByte(bytes[1], 0);   // CRC2 
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
     PrintByte(bytes[i], 0); 
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
      PrintByte(bytes[i], 0); 
    }
    Debug.println();
  }

   Debug.print("Raw: ");
   for (int i = 0; i < 40; i++) {  //  unformatted
      PrintByte(bytes[i], 0); 
    }
   Debug.println();
  
}
//=================================================================================================  
float RadToDeg (float _Rad) {
  return _Rad * 180 / PI;  
}
//=================================================================================================  
float DegToRad (float _Deg) {
  return _Deg * PI / 180;  
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

//=================================================================================================  
void PrintPeriod(bool LF) {
  now_millis=millis();
  now_micros=micros();

  uint32_t period = now_millis - prev_millis;
  if (period < 10) {
    period = now_micros - prev_micros;
    Debug.printf(" Period uS=%d", period);
  } else {
    Debug.printf(" Period mS=%d", period);
  }

  if (LF) {
    Debug.print("\t\n");
  } else {
   Debug.print("\t");
  }
    
  prev_millis=now_millis;
  prev_micros=now_micros;
}
//=================================================================================================  
void PrintFrPeriod(bool LF) {
  now_millis=millis();
  now_micros=micros();

  uint32_t period = now_millis - prev_fr_millis;
  if (period < 10) {
    period = now_micros - prev_fr_micros;
    Debug.printf(" FrPeriod uS=%d", period);
  } else {
    Debug.printf(" FrPeriod mS=%d", period);
  }

  if (LF) {
    Debug.print("\t\n");
  } else {
   Debug.print("\t");
  }
    
  prev_fr_millis=now_millis;
  prev_fr_micros=now_micros;
}
//=================================================================================================  
void PrintLoopPeriod() {
  now_millis=millis();
  now_micros=micros();

  uint32_t period = now_millis - prev_lp_millis;
  if (period < 10) {
    period = now_micros - prev_lp_micros;
    Debug.printf("Loop Period uS=%d\n", period);
  } else {
    Debug.printf("Loop Period mS=%d\n", period);
    if (period > 5000) Debug.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  }
    
  prev_lp_millis=now_millis;
  prev_lp_micros=now_micros;
}
//=================================================================================================   
//                             O L E D   S U P P O R T   -   ESP Only - for now
//================================================================================================= 
void OledPrintln(String S) {
#if (defined OLED_Support) && ((defined ESP32) || (defined ESP8266))

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

//===================================
void OledPrint(String S) {
#if (defined OLED_Support) && ((defined ESP32) || (defined ESP8266))     
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
//=================================================================================================   
//                   E N D   O F   O L E D   S U P P O R T   -   ESP Only - for now
//================================================================================================= 

//=================================================================================================   
//                             W I F I   S U P P O R T   -   ESP Only - for now
//=================================================================================================  
#if (defined wifiBuiltin)

  void SenseWiFiStatus() {

  // If external STA disconnects from our AP, then it is the resposibility of that STA to reconnect

  if (wifiSuGood && wifiSuDone && (set.wfmode == sta) )  { 
        
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
          Debug.println("Wifi link restored");
          OledPrintln("Wifi link restored"); 
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

   #if defined Start_WiFi
    if (!wifiSuDone) {
      SetupWiFi();
      return;
    }
    return;
  #else
    if ((wifiButnPres > 0) && (!wifiSuDone)) {
      wifiButnPres = 0;
      SetupWiFi();
      } 
  #endif    
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
    Debug.println("WiFi link lost - retrying XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    OledPrintln("Wifilink lost - retry"); 
    WiFi.disconnect(false); 
    WiFi.mode(WIFI_STA);
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

    bool apFailover = false;            // used when STA fails to connect
    #if (defined ESP32)
      // set up wifi retry interrupt 
      wifiTimerSemaphore = xSemaphoreCreateBinary();  // Create wifi retry semaphore
      timer = timerBegin(3, 80, true);                // use timer 0 (0 thru 3) set 80 divider for prescaler
      timerAttachInterrupt(timer, &onTimer, true);
      timerAlarmWrite(timer, 5 * 1E6, true);         // uS, repeat  (semaphore every 5 seconds when alarm enabled)
    #endif

    #if (defined ESP8266)
      esp8266_wifi_retry_millis = 0;
    #endif
    
    apFailover = byte(EEPROM.read(0));              //  Read first eeprom byte
    #if defined Debug_Eeprom
      Debug.print("Read EEPROM apFailover = "); Debug.println(apFailover); 
    #endif

    //=====================================  S T A T I O N ========================================== 

  if ((set.wfmode == sta) || (set.wfmode == sta_ap))  {  // STA mode or STA failover to AP mode
    if (!apFailover) {   

      
      uint8_t retry = 0;
      WiFi.disconnect(true);   // To circumvent "wifi: Set status to INIT" error bug
      nbdelay(500);
      if (WiFi.mode(WIFI_STA)) {
         Debug.println("Wi-Fi mode set to STA sucessfully");  
      } else {
        Debug.println("Wi-Fi mode set to STA failed!");  
      }
      Debug.print("Trying to connect to ");  
      Debug.print(set.staSSID); 
      OledPrintln("WiFi trying ..");
      nbdelay(500);
      
      WiFi.begin(set.staSSID, set.staPw);
      while (WiFi.status() != WL_CONNECTED){
        retry++;
        if (retry > 10) {
          Debug.println();
          Debug.println("Failed to connect in STA mode");
          OledPrintln("No connect STA Mode");
          #if (WiFi_Mode == 3)         // STA failover to AP mode
            apFailover = true;         
            Debug.println("Failover to AP");
            OledPrintln("Failover to AP");  
            apFailover = 1;                // set STA failover to AP flag
            EEPROM.write(0, apFailover);   // (addr, val)  
            EEPROM.commit();
            #if defined Debug_Eeprom
              Debug.print("Write EEPROM apFailover = "); Debug.println(apFailover); 
            #endif          
            delay(1000);
            ESP.restart();                 // esp32 and esp8266
          #endif  
          
          break;
        }
        nbdelay(500);
        Debug.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        localIP = WiFi.localIP();  // TCP and UDP
        udp_remoteIP = localIP;    // Broadcast on the subnet we are attached to. patch by Stefan Arbes. 
        udp_remoteIP[3] = 255;     // patch by Stefan Arbes   
        Debug.println();
        Debug.println("WiFi connected!");
        Debug.print("Local IP address: ");
        Debug.print(localIP);
        if (set.wfproto == tcp)  {   // TCP
          Debug.print("  port: ");
          Debug.println(set.tcp_localPort);    //  UDP port is printed lower down
        } else {
          Debug.println();
        }
 
        wifi_rssi = WiFi.RSSI();
        Debug.print("WiFi RSSI:");
        Debug.print(wifi_rssi);
        Debug.println(" dBm");

        OledPrintln("Connected!");
        OledPrintln(localIP.toString());
        
        if (set.wfproto == tcp)  {   // TCP
          TCPserver.begin(set.tcp_localPort);                     //  tcp socket started
          Debug.println("TCP server started");  
          OledPrintln("TCP server started");
        }

        if (set.wfproto == udp)  {  // UDP
          UDP.begin(set.udp_localPort);
          Debug.printf("UDP started, listening on IP %s, UDP port %d \n", localIP.toString().c_str(), set.udp_localPort);
          OledPrint("UDP port = ");  OledPrintln(String(set.udp_localPort));
        }
        
        wifiSuGood = true;
        
      } 
    }  else {   // if apFailover clear apFailover flag
        apFailover = 0;
        EEPROM.write(0, apFailover);   // (addr, val)  
        EEPROM.commit();
        #if defined Debug_Eeprom
          Debug.print("Clear EEPROM apFailover = "); Debug.println(apFailover); 
        #endif  
      }

  }
 
  
     //===============================  A C C E S S   P O I N T ===============================
     
  if ((set.wfmode == ap) || (set.wfmode == sta_ap)) { // AP mode or STA failover to AP mode
    if (!wifiSuGood) {  // not already setup in STA above  
    
      Debug.printf("Wi-Fi mode set to WIFI_AP %s\n", WiFi.mode(WIFI_AP) ? "" : "Failed!");
      WiFi.softAP(set.apSSID, set.apPw, set.channel);
      localIP = WiFi.softAPIP();   // tcp and udp
      Debug.print("AP IP address: ");
      Debug.print (localIP); 
      Debug.print("  SSID: ");
      Debug.println(String(set.apSSID));
      OledPrintln("WiFi AP SSID =");
      OledPrintln(String(set.apSSID));
      
      if (set.wfproto == tcp)  {         // TCP
          TCPserver.begin(set.tcp_localPort);   //  Server for TCP/IP traffic
          Debug.printf("TCP/IP started, listening on IP %s, TCP port %d\n", localIP.toString().c_str(), set.tcp_localPort);
          OledPrint("TCP port = ");  OledPrintln(String(set.tcp_localPort));
        }

      if (set.wfproto == udp)  {      // UDP
          UDP.begin(set.udp_localPort);
          Debug.printf("UDP started, listening on IP %s, UDP port %d \n", WiFi.softAPIP().toString().c_str(), set.udp_localPort);
          OledPrint("UDP port = ");  OledPrintln(String(set.udp_localPort));
          udp_remoteIP[2] = 4;     
          udp_remoteIP[3] = 255;    // UDP broadcast on the AP 192.168.4/ subnet
      }
    }
      
    wifiSuGood = true;  
    
  }        // end of AP ===========================================================================         

  #if defined Debug_SRAM
    Debug.printf("==============>Free Heap after WiFi setup = %d\n", ESP.getFreeHeap());
  #endif

  #if defined webSupport
    if (wifiSuGood) {
      WebServerSetup();  
      Debug.print("Web support active on http://"); 
      Debug.println(localIP.toString().c_str());
      OledPrintln("webSupport active");  
    }  else {
      Debug.println("No web support possible"); 
      OledPrintln("No web support!");  
    }
  #endif

  wifiSuDone = true;
    
 } 

  //=================================================================================================  
   void PrintRemoteIP() {
    if (FtRemIP)  {
      FtRemIP = false;
      Debug.print("Client connected: Remote UDP IP: "); Debug.print(udp_remoteIP);
      Debug.print("  Remote  UDP port: "); Debug.println(set.udp_remotePort);
      OledPrintln("Client connected");
      OledPrintln("Remote UDP IP =");
      OledPrintln(udp_remoteIP.toString());
      OledPrintln("Remote UDP port =");
      OledPrintln(String(set.udp_remotePort));
     }
  }
#endif 
//=================================================================================================   
//                             E N D   O F   W I F I   S U P P O R T   -   ESP Only - for now
//=================================================================================================
//=================================================================================================   
//                             S D   C A R D   S U P P O R T   -   ESP32 Only - for now
//================================================================================================= 

#if ((defined ESP32) || (defined ESP8266)) && (defined SD_Support)

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
    Debug.printf("Initialising file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Debug.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Debug.println("File initialised");
    } else {
        Debug.println("Write failed");
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

//=================================================================================================  

void OpenSDForWrite() {
  
  //  deleteFile(SD, "/mav2passthu.tlog");
  
    uint32_t time_unix_sec = (ap_time_unix_usec/1E6) + (Time_Zone * 3600);   // add time zone adjustment decs
    if (daylightSaving) ap_time_unix_usec -= 3600;   // deduct an hour
    decomposeEpoch(time_unix_sec, dt_tm);

    String sPath = "/MavToPass"  + DateTimeString(dt_tm) + ".tlog";
    Debug.print("  Path: "); Debug.println(sPath); 

    strcpy(cPath, sPath.c_str());
    writeFile(SD, cPath , "Mavlink to FrSky Passthru by zs6buj");
    OledPrintln("Writing Tlog");
    sdStatus = 3;      
}

#endif      //  ESP Only - for now

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
//=================================================================================================  
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
//=================================================================================================  
uint32_t GetBaud(uint8_t rxPin) {
  Debug.printf("AutoBaud - Sensing FC_Mav_rxPin %2d \n", rxPin );
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
      Debug.printf("No telemetry found on pin %2d\n", rxPin); 
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
//=================================================================================================  
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
//=================================================================================================  
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
    #if (defined wifiBuiltin)
      SenseWiFiStatus();
    #endif  
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



//=================================================================================================  
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
         //   Debug.print("abs_number<100  ="); Debug.print(abs_number); Debug.print(" res="); Debug.print(res);
        } else if (abs_number < 1000) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<2)|0x1;
         //   Debug.print("abs_number<1000  ="); Debug.print(abs_number); Debug.print(" res="); Debug.print(res);
        } else if (abs_number < 10000) {
            res = ((uint8_t)roundf(abs_number * 0.01f)<<2)|0x2;
          //  Debug.print("abs_number<10000  ="); Debug.print(abs_number); Debug.print(" res="); Debug.print(res);
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
  fr_payload |= dw_and_mask; 
}
//=================================================================================================  
  uint32_t bit32Unpack(uint32_t dword,uint8_t displ, uint8_t lth) {
  uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
  return r;
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
    Debug.printf("[WiFi-event] event: %d ", event);

    switch (event) {
        case SYSTEM_EVENT_WIFI_READY: 
            Debug.println("WiFi interface ready");
            break;
        case SYSTEM_EVENT_SCAN_DONE:
            Debug.println("Completed scan for access points");
            break;
        case SYSTEM_EVENT_STA_START:
            Debug.println("WiFi client started");
            break;
        case SYSTEM_EVENT_STA_STOP:
            Debug.println("WiFi clients stopped");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            Debug.println("Connected to access point");
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Debug.println("Disconnected from WiFi access point");
            break;
        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
            Debug.println("Authentication mode of access point has changed");
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            Debug.print("Obtained IP address: ");
            Debug.println(WiFi.localIP());
            break;
        case SYSTEM_EVENT_STA_LOST_IP:
            Debug.println("Lost IP address and IP address is reset to 0");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
            Debug.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_FAILED:
            Debug.println("WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
            Debug.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_PIN:
            Debug.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case SYSTEM_EVENT_AP_START:
            Debug.println("WiFi access point started");
            break;
        case SYSTEM_EVENT_AP_STOP:
            Debug.println("WiFi access point  stopped");
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            Debug.println("Client connected");
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            Debug.println("Client disconnected");
            break;
        case SYSTEM_EVENT_AP_STAIPASSIGNED:
            Debug.println("Assigned IP address to client");
            break;
        case SYSTEM_EVENT_AP_PROBEREQRECVED:
            Debug.println("Received probe request");
            break;
        case SYSTEM_EVENT_GOT_IP6:
            Debug.println("IPv6 is preferred");
            break;
        case SYSTEM_EVENT_ETH_START:
            Debug.println("Ethernet started");
            break;
        case SYSTEM_EVENT_ETH_STOP:
            Debug.println("Ethernet stopped");
            break;
        case SYSTEM_EVENT_ETH_CONNECTED:
            Debug.println("Ethernet connected");
            break;
        case SYSTEM_EVENT_ETH_DISCONNECTED:
            Debug.println("Ethernet disconnected");
            break;
        case SYSTEM_EVENT_ETH_GOT_IP:
            Debug.println("Obtained IP address");
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



//================================================================================================= 
//================================================================================================= 
//
//                                      W E B   S U P P O R T  
// 
//================================================================================================= 
//================================================================================================= 
#if defined webSupport

 static const String styleLogin =  // Store in FLASH not SRAM Heap
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
    "form{background:#fff;max-width:400px;margin:30px auto;padding:30px;border-radius:10px;text-align:left;font-size:16px}"
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


  server.on("/", handleLoginPage);                  // root
  server.on("/settingsIndex", handleSettingsPage);             
  server.on("/settingsReturnIndex", handleSettingsReturn);  // save settings and reboot
  server.on("/rebootIndex", handleReboot);                  // reboot only
  server.on("/otaIndex", handleOtaPage); 
  
  server.begin();
   
  #if defined Debug_SRAM
    Debug.printf("==============>Free Heap after WebServer setup = %d\n", ESP.getFreeHeap());
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
    WriteSettingsToEEPROM();
    }  
      
  ReadSettingsFromEEPROM();                           

}

//===========================================================================================

void String_char(char* ch, String S) {
  int i;
  int lth = sizeof(S);
  for ( i = 0 ; i <= lth ; i++) {
    ch[i] = S[i];
  }
  ch[i] = 0;
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
    sprintf(temp,  "<h1>%s Login</h1>", set.host);
    loginPage += temp;
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
  settingsPage += "<!DOCTYPE html><html><body><h>Mavlink To Passthru</h><form action='' ";  
  settingsPage += "autocomplete='on'> <center> <b><h3>MavToPassthru Translator Setup</h3> </b></center> <style>text-align:left</style>";
  settingsPage += "Translator Mode: &nbsp &nbsp";
  sprintf(temp, "<input type='radio' class='big' name='_trmode' value='Ground' %s> Ground &nbsp &nbsp", set.trmode1);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_trmode' value='Air' %s> Air &nbsp &nbsp", set.trmode2);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_trmode' value='Relay' %s> Relay <br>", set.trmode3);
  settingsPage += temp;
  settingsPage += "FC  IO: &nbsp &nbsp";
  sprintf(temp, "<input type='radio' class='big' name='_fc_io' value='Serial' %s> Serial &nbsp &nbsp", set.fc_io0);
  settingsPage += temp;
  #if defined ESP32 
    sprintf(temp, "<input type='radio' class='big' name='_fc_io' value='BT' %s> BT &nbsp &nbsp", set.fc_io1);
    settingsPage += temp;
  #endif  
  sprintf(temp, "<input type='radio' class='big' name='_fc_io' value='WiFi' %s> WiFi &nbsp &nbsp", set.fc_io2);
  settingsPage += temp; 
  #if defined SD_Support   
    sprintf(temp, "<input type='radio' class='big' name='_fc_io' value='SD' %s> SD ", set.fc_io3);
    settingsPage += temp;  
  #endif    
  settingsPage += "<br>GCS IO: &nbsp";
  sprintf(temp, "<input type='radio' class='big' name='_gs_io' value='None' %s> None &nbsp &nbsp", set.gs_io9);
  settingsPage += temp;
  #if defined Enable_GCS_Serial
    sprintf(temp, "<input type='radio' class='big' name='_gs_io' value='Serial' %s> Serial &nbsp &nbsp", set.gs_io0);
    settingsPage += temp;
  #endif  
  #if defined ESP32
    sprintf(temp, "<input type='radio' class='big' name='_gs_io' value='BT' %s> BT &nbsp &nbsp", set.gs_io1);
    settingsPage += temp;
  #endif  
  sprintf(temp, "<input type='radio' class='big' name='_gs_io' value='WiFi' %s> WiFi &nbsp &nbsp ", set.gs_io2);
  settingsPage += temp; 
  #if defined ESP32 
    sprintf(temp, "<input type='radio' class='big' name='_gs_io' value='WiFi+BT' %s> WiFi+BT ", set.gs_io3);
    settingsPage += temp;
  #endif  
  #if defined SD_Support   
    settingsPage += " <br> GCS SD: &nbsp";
    sprintf(temp, "<input type='radio' class='big' name='_gs_sd' value='OFF' %s> OFF  &nbsp &nbsp ", set.gs_sd0);
    settingsPage += temp; 
    sprintf(temp, "<input type='radio' class='big' name='_gs_sd' value='ON' %s> ON ", set.gs_sd1);
    settingsPage += temp; 
  #endif       
  settingsPage += "<br>WiFi Mode: &nbsp &nbsp ";
  sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='AP' %s> AP &nbsp &nbsp", set.wfmode1);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='STA' %s> STA &nbsp &nbsp", set.wfmode2);
  settingsPage += temp;    
  sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='STA_AP' %s> STA/AP &nbsp <br>", set.wfmode3);
  settingsPage += temp;
  settingsPage += "WiFi Protocol: &nbsp &nbsp ";
  sprintf(temp, "<input type='radio' class='big' name='_wfproto' value='TCP' %s> TCP &nbsp &nbsp", set.wfproto1);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_wfproto' value='UDP' %s> UDP &nbsp <br>", set.wfproto2);
  settingsPage += temp; 
  sprintf(temp, "Mavlink Baud: <input type='text' name='_baud' value='%d' size='3' maxlength='6'> <br>", set.baud);
  settingsPage += temp;
  sprintf(temp, "WiFi Channel: <input type='text' name='_channel' value='%d' size='1' maxlength='2'> <br>", set.channel);
  settingsPage += temp;
  sprintf(temp, "AP SSID: <input type='text' name='_apSSID' value='%s' size='30' maxlength='30'> <br>", set.apSSID);
  settingsPage+= temp;
  sprintf(temp, "AP Password: <input type='text' name='_apPw' value='%s' size='20'> <br>", set.apPw);
  settingsPage += temp;
  sprintf(temp, "STA SSID: <input type='text' name='_staSSID' value='%s' size='30'> <br>", set.staSSID);
  settingsPage += temp;
  sprintf(temp, "STA Password: <input type='text' name='_staPw' value='%s' size='20'> <br>", set.staPw);
  settingsPage += temp;
  sprintf(temp, "Host Name: <input type='text' name='_host' value='%s' size='20'> <br>", set.host);
  settingsPage += temp;
  sprintf(temp, "TCP Local Port: <input type='text' name='_tcp_localPort' value='%d' size='2' maxlength='5'> <br>", set.tcp_localPort);
  settingsPage += temp;
  sprintf(temp, "UDP Local Port: <input type='text' name='_udp_localPort' value='%d' size='2' maxlength='5'> <br>", set.udp_localPort);
  settingsPage += temp;
  sprintf(temp, "UDP Remote Port: <input type='text' name='_udp_remotePort' value='%d' size='2' maxlength='5'> <br>", set.udp_remotePort);
  settingsPage += temp;
  settingsPage += "Bluetooth Mode: &nbsp &nbsp ";
  sprintf(temp, "<input type='radio' class='big' name='_btmode' value='Master' %s> Master &nbsp &nbsp &nbsp &nbsp ", set.btmode1);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_btmode' value='Slave' %s> Slave &nbsp &nbsp <br>", set.btmode2);
  settingsPage += temp;
  sprintf(temp, "Master to Slave: <input type='text' name='_btConnectToSlave' value='%s' size='20' maxlength='20'>  <br><br><center>", set.btConnectToSlave);
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
    #if defined Enable_GCS_Serial 
      if (b == 0) {
        set.gs_io = gs_ser;
      } else
    #endif  
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
    } else if (b == 2) {
      set.gs_sd = gs_on;
    } 

    b = EEPROMRead8(6);     // wifi mode
    if (b == 1) {
      set.wfmode = ap;
    } else if (b == 2) {
      set.wfmode = sta;
    } else if (b == 3) {
      set.wfmode = sta_ap;
    } 
    b = EEPROMRead8(7);     // wifi protocol
    if (b == 1) {
      set.wfproto = tcp;
    } else if (b == 2) {
      set.wfproto = udp;
    }  
    set.baud = EEPROMRead32(8);                  //  8 thru  11
    set.channel = EEPROMRead8(12);               //  12
    EEPROMReadString(13, set.apSSID);            //  13 thru 42
    EEPROMReadString(43, set.apPw);               //  4 thru 62
    EEPROMReadString(63, set.staSSID);           // 63 thru 92 
    EEPROMReadString(93, set.staPw);             // 93 thru 112  
    EEPROMReadString(113, set.host);             // 113 thru 132   
    set.tcp_localPort = EEPROMRead16(133);       // 133 thru 134 
    set.udp_localPort = EEPROMRead16(135);       // 135 thru 136 
    set.udp_remotePort = EEPROMRead16(137);      // 137 thru 138 
    b = EEPROMRead8(139);                        // 139
    if (b == 1) {
      set.btmode = master;
    } else if (b == 2) {
      set.btmode = slave;
    } 
    EEPROMReadString(140, set.btConnectToSlave);  // 140 thru 160 

    RefreshHTMLButtons();
   
    #if defined Debug_Web_Settings
      Debug.println();
      Debug.println("Debug Read WiFi Settings from EEPROM: ");
      Debug.print("validity_check = "); Debug.println(set.validity_check, HEX);
      Debug.print("translator mode = "); Debug.println(set.trmode);       
      Debug.print("fc_io = "); Debug.println(set.fc_io);                
      Debug.print("gcs_io = "); Debug.println(set.gs_io);     
      Debug.print("gcs_sd = "); Debug.println(set.gs_sd);       
      Debug.print("wifi mode = "); Debug.println(set.wfmode);
      Debug.print("wifi protocol = "); Debug.println(set.wfproto);     
      Debug.print("baud = "); Debug.println(set.baud);
      Debug.print("wifi channel = "); Debug.println(set.channel);  
      Debug.print("apSSID = "); Debug.println(set.apSSID);
      Debug.print("apPw = "); Debug.println(set.apPw);
      Debug.print("staSSID = "); Debug.println(set.staSSID);
      Debug.print("staPw = "); Debug.println(set.staPw); 
      Debug.print("Host = "); Debug.println(set.host);           
      Debug.print("tcp_localPort = "); Debug.println(set.tcp_localPort);
      Debug.print("udp_localPort = "); Debug.println(set.udp_localPort);
      Debug.print("udp_remotePort = "); Debug.println(set.udp_remotePort); 
      Debug.print("bt mode = "); Debug.println(set.btmode); 
      Debug.print("SlaveConnect To Name = "); Debug.println(set.btConnectToSlave);
      Debug.println();          
    #endif              
  }
//=================================================================================
void WriteSettingsToEEPROM() {
      set.validity_check = 0xdc;                                     
      EEPROMWrite8(1, set.validity_check);           // apFailover is in 0 
      EEPROMWrite8(2, set.trmode);                   //  2   
      EEPROMWrite8(3, set.fc_io);                    //  3
      EEPROMWrite8(4, set.gs_io);                    //  4    
      EEPROMWrite8(5, set.gs_sd);                    //  5
      EEPROMWrite8(6, set.wfmode);                   //  6
      EEPROMWrite8(7, set.wfproto);                  //  7     
      EEPROMWrite32(8,set.baud);                     //  8 thru 11
      EEPROMWrite8(12, set.channel);                 // 12
      EEPROMWriteString(13, set.apSSID);             // 13 thru 42 
      EEPROMWriteString(43, set.apPw);               // 43 thru 62      
      EEPROMWriteString(63, set.staSSID);            // 63 thru 92 
      EEPROMWriteString(93, set.staPw);              // 93 thru 112 
      EEPROMWriteString(113, set.host);              // 113 thru 132 
      EEPROMWrite16(133, set.tcp_localPort);         // 133 thru 134
      EEPROMWrite16(135, set.udp_localPort);         // 135 thru 136
      EEPROMWrite16(137, set.udp_remotePort);        // 137 thru 138
      EEPROMWrite8(139, set.btmode);                 // 139     
      EEPROMWriteString(140, set.btConnectToSlave);   // 140 thru 160   
      EEPROM.commit();  
      RefreshHTMLButtons();
                                                                  
      #if defined Debug_Web_Settings
        Debug.println();
        Debug.println("Debug Write WiFi Settings to EEPROM: ");
        Debug.print("validity_check = "); Debug.println(set.validity_check, HEX);
        Debug.print("translator mode = "); Debug.println(set.trmode);       
        Debug.print("fc_io = "); Debug.println(set.fc_io);                
        Debug.print("gcs_io = "); Debug.println(set.gs_io);     
        Debug.print("gcs_sd = "); Debug.println(set.gs_sd);       
        Debug.print("wifi mode = "); Debug.println(set.wfmode);
        Debug.print("wifi protocol = "); Debug.println(set.wfproto);     
        Debug.print("baud = "); Debug.println(set.baud);
        Debug.print("wifi channel = "); Debug.println(set.channel);  
        Debug.print("apSSID = "); Debug.println(set.apSSID);
        Debug.print("apPw = "); Debug.println(set.apPw);
        Debug.print("staSSID = "); Debug.println(set.staSSID);
        Debug.print("staPw = "); Debug.println(set.staPw); 
        Debug.print("Host = "); Debug.println(set.host);           
        Debug.print("tcp_localPort = "); Debug.println(set.tcp_localPort);
        Debug.print("udp_localPort = "); Debug.println(set.udp_localPort);
        Debug.print("udp_remotePort = "); Debug.println(set.udp_remotePort); 
        Debug.print("bt mode = "); Debug.println(set.btmode); 
        Debug.print("Master to Slave Name = "); Debug.println(set.btConnectToSlave); 
        Debug.println();             
      #endif 
}  
//=================================================================================
void ReadSettingsFromForm() {
  String S;

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
  #if defined Enable_GCS_Serial
    if (S == "Serial") {
      set.gs_io = gs_ser;
    } else
  #endif  
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
  
  S = server.arg("_wfmode");
  if (S == "AP") {
    set.wfmode = ap;
  } else 
  if (S == "STA") {
    set.wfmode = sta;
  } else 
  if (S == "STA/AP") {
    set.wfmode = sta_ap;
  } 
   
  S = server.arg("_wfproto");
  if (S == "TCP") {
    set.wfproto = tcp;
  } else 
  if (S == "UDP") {
    set.wfproto = udp;
  }   
  set.baud = String_long(server.arg("_baud"));
  set.channel = String_long(server.arg("_channel")); 
  String_char(set.apSSID, server.arg("_apSSID"));
  String_char(set.apPw, server.arg("_apPw"));
  String_char(set.staSSID, server.arg("_staSSID"));
  String_char(set.staPw, server.arg("_staPw"));
  String_char(set.host, server.arg("_host"));
  set.tcp_localPort = String_long(server.arg("_tcp_localPort"));
  set.udp_localPort = String_long(server.arg("_udp_localPort"));
  set.udp_remotePort = String_long(server.arg("_udp_remotePort")); 
  S = server.arg("_btmode");
  if (S == "Master") {
    set.btmode = master;
  } else 
  if (S == "Slave") {
    set.btmode = slave;
  } 
  String_char(set.btConnectToSlave, server.arg("_btConnectToSlave"));

      #if defined Debug_Web_Settings
        Debug.println();
        Debug.println("Debug Read WiFi Settings from Form: ");
        Debug.print("validity_check = "); Debug.println(set.validity_check, HEX);
        Debug.print("translator mode = "); Debug.println(set.trmode);       
        Debug.print("fc_io = "); Debug.println(set.fc_io);                
        Debug.print("gcs_io = "); Debug.println(set.gs_io);  
        Debug.print("gcs_sd = "); Debug.println(set.gs_sd);          
        Debug.print("wifi mode = "); Debug.println(set.wfmode);
        Debug.print("wifi protocol = "); Debug.println(set.wfproto);     
        Debug.print("baud = "); Debug.println(set.baud);
        Debug.print("wifi channel = "); Debug.println(set.channel);  
        Debug.print("apSSID = "); Debug.println(set.apSSID);
        Debug.print("apPw = "); Debug.println(set.apPw);
        Debug.print("staSSID = "); Debug.println(set.staSSID);
        Debug.print("staPw = "); Debug.println(set.staPw); 
        Debug.print("Host = "); Debug.println(set.host);           
        Debug.print("tcp_localPort = "); Debug.println(set.tcp_localPort);
        Debug.print("udp_localPort = "); Debug.println(set.udp_localPort);
        Debug.print("udp_remotePort = "); Debug.println(set.udp_remotePort); 
        Debug.print("bt mode = "); Debug.println(set.btmode); 
        Debug.print("Master to Slave Name = "); Debug.println(set.btConnectToSlave);  
        Debug.println();      
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

  #if defined Enable_GCS_Serial
    if (set.gs_io == gs_ser) {
      set.gs_io0 = "checked";
      set.gs_io1 = "";
      set.gs_io2 = "";
      set.gs_io3 = ""; 
      set.gs_io9 = "";           
    } else 
  #endif  
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
  
  if (set.wfmode == ap) {
    set.wfmode1 = "checked";
    set.wfmode2 = "";
    set.wfmode3 = "";
  } else 
  if (set.wfmode == sta) {
    set.wfmode1 = "";
    set.wfmode2 = "checked";
    set.wfmode3 = "";
  } else
  if (set.wfmode == sta_ap) {
    set.wfmode1 = "";
    set.wfmode2 = "";
    set.wfmode3 = "checked";
  }

  if (set.wfproto == tcp) {
    set.wfproto1 = "checked";
    set.wfproto2 = "";
  } else 
  if (set.wfproto == udp) {
    set.wfproto1 = "";
    set.wfproto2 = "checked";
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

  #if (defined btBuiltin)
  //  if (btActive) {
    
      #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
        Debug.printf("==============>Free Heap before handleLoginPage = %d\n", ESP.getFreeHeap());
      #endif  
        
      esp_bt_controller_disable();
      esp_bt_controller_deinit();
      esp_bt_mem_release(ESP_BT_MODE_BTDM);
      btActive = false;
      Debug.println("Bluetooth disabled to free up SRAM for web support"); 
      OledPrintln("Bluetooth disabled");         
      #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
        Debug.printf("==============>Free Heap after bluetooth disabled = %d\n", ESP.getFreeHeap());
      #endif
  //  }
  #endif
  
  ComposeLoginPage();
  server.send(200, "text/html", loginPage); 
 }
 //===========================================================================================
 void handleSettingsPage() {
  ComposeSettingsPage();
  server.send(200, "text/html", settingsPage); 
 }
 //===========================================================================================
 void handleSettingsReturn() {
  ReadSettingsFromForm();

  WriteSettingsToEEPROM();
  
  String s = "<a href='/'> Rebooting........  Back to login screen</a>";
  server.send(200, "text/html", styleLogin+s);
  Debug.println("Rebooting ......");  
  delay(3000);
  ESP.restart();                 // esp32 and esp8266   
 }
 
 //===========================================================================================
 void handleReboot() {
  String s = "<a href='/'> Rebooting........  Back to login screen</a>";
  server.send(200, "text/html", styleLogin+s);
  Debug.println("Rebooting ......");  
  delay(3000);
  ESP.restart();                 // esp32 and esp8266   
 }
 
 //===========================================================================================
 void handleOtaPage() {
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
 //   Debug.setDebugOutput(true);
      #if (defined ESP32) 
        uploadSize = UPDATE_SIZE_UNKNOWN;
      #elif (defined ESP8266) 
        WiFiUDP::stopAll();
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        uploadSize = maxSketchSpace;
      #endif

      Debug.printf("Update: %s\n", upload.filename.c_str());    
      if (!Update.begin(uploadSize)) { //start with max available size
        Update.printError(Debug);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP OTA space */
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Debug);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to show the percentage progress bar
        Debug.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        delay(2000);
      } else {
        Update.printError(Debug);
      }
   // Debug.setDebugOutput(false);
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
          if (s[i] == 0x00) {                  // eo string
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

  #if defined Enable_GCS_Serial
    if (GCS_Mavlink_IO == 0) {
      set.gs_io = gs_ser;
    } else 
  #endif  
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

  #if defined GCS_Mavlink_SD 
    set.gs_sd = gs_on;
  #else
    set.gs_sd = gs_off;
  #endif
      
  if (WiFi_Mode == 1) {
    set.wfmode = ap;
  } else 
  if (WiFi_Mode == 2) {
    set.wfmode = sta;
  } else 
  if (WiFi_Mode == 3) {
    set.wfmode = sta_ap;
  } 
    
  if (WiFi_Protocol == 1) {
    set.wfproto = tcp;
  } else 
  if (WiFi_Protocol == 2) {
    set.wfproto = udp;
  } 
         
  set.baud = mvBaudFC;          
  set.channel = APchannel;
  strcpy(set.apSSID, APssid);  
  strcpy(set.apPw, APpw);                          
  strcpy(set.staSSID, STAssid);           
  strcpy(set.staPw, STApw);   
  strcpy(set.host, HostName);        
  set.tcp_localPort = TCP_localPort;
  set.udp_localPort = UDP_localPort;
  set.udp_remotePort = UDP_remotePort;  

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
      Debug.println();
      Debug.println("Debug Raw WiFi Settings : ");
      Debug.print("web_support = "); Debug.println(set.web_support);      
      Debug.print("validity_check = "); Debug.println(set.validity_check, HEX);   
      Debug.print("translator mode = "); Debug.println(set.trmode);       
      Debug.print("fc_io = "); Debug.println(set.fc_io);                
      Debug.print("gcs_io = "); Debug.println(set.gs_io);     
      Debug.print("gcs_sd = "); Debug.println(set.gs_sd);         
      Debug.print("wifi mode = "); Debug.println(set.wfmode);
      Debug.print("wifi protocol = "); Debug.println(set.wfproto);     
      Debug.print("baud = "); Debug.println(set.baud);
      Debug.print("wifi channel = "); Debug.println(set.channel);  
      Debug.print("apSSID = "); Debug.println(set.apSSID);
      Debug.print("apPw = "); Debug.println(set.apPw);
      Debug.print("staSSID = "); Debug.println(set.staSSID);
      Debug.print("staPw = "); Debug.println(set.staPw); 
      Debug.print("Host = "); Debug.println(set.host);           
      Debug.print("tcp_localPort = "); Debug.println(set.tcp_localPort);
      Debug.print("udp_localPort = "); Debug.println(set.udp_localPort);
      Debug.print("udp_remotePort = "); Debug.println(set.udp_remotePort); 
      Debug.print("bt mode = "); Debug.println(set.btmode); 
      Debug.print("Master to Slave Name = "); Debug.println(set.btConnectToSlave); 
      Debug.println(); 
  #endif    
}

//=================================================================================



