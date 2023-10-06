/*
================================================================================================   
=================================================================================================

GitHub Tag
---------- 
                                    
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
v2.29 2019-09-24 Use #if (TargetBoard == 3) to define soft pins for fcSerial
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
        2020-04-28 GetBaud(fc_rxPin) fix. Thanks has1123. 
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
V2.64.8    2021-04-07  Fixed AP mode web setting trying STA mode first
V2.64.9    2021-04-07  Fixed pure AP mode UDP object / no port swap
V2.65.0    2021-05-11  PR merged from Alex (yaapu)
                          added RPM frame 0x500A for rpm1 and rpm2
                          added TERRAIN frame 0x500B for terrain enabled/unhealthy
                          added FENCE status bits to frame 0x5001
                          added THROTTLE to frame 0x5001  
v2.65.11   2021-06-12  Fix scr_w_ch length check.       
V2.65.1   2021-05-13   PR merged from Alex (yaapu)
                          fix throttle scale from 0,100 to [-63,63]
V2.65.2   2021-05-17   Beta folder only. Bytestuff enable Write_Crc() 
                       Delay 15ms 0x5000 status_text chunks > 1                           
V2.65.3   2021-05-18   PR merged from Alex
                       SPort loop period from 18mS to 24mS
                       Work around apparent bit32Pack() anomaly.                         
V2.65.4   2021-05-19   500a and 500b, clear payload before bit32Pack() 

V2.65.5   2021-05-21   Add ability to change default AP IP from 192.168.4.1 
                       If FrSky i/o is UDP, start both FrSky and Mavlink UDP objects
                       
V2.65.6   2021-05-25   Show fw version on web setup screen  
V2.65.7   2021-06-01   Initialise FrSky serial only if it is selected 
                       Helps with EDP8266 debug out on txd1
v2.65.8   2021-06-02   For ESP8266 variant 2, report TXD1 Log vs LED setting
                       Fix pinMode() for MavStatusLed                                                                
v2.65.9   2021-06-03   Fix AP non-standard IP assignment timing anomaly  
v2.65.10  2021-06-10   Standardise display approach. Upgrade info display.
                       Improve home location fix.   
v2.65.11  2021-06-16   Fix crc of FPort2 RC control frame(unused right now). 
                       Slow down text messages some more.      
v2.65.12  2021-06-19   Fix crc of FPort2 RC control frame(unused right now). 
                       Change FPort type (1 or 2) on the fly option.
                       Revert status text speedup from v2.65.11. Problematic.
v2.65.13  2021-06-22   Minor display change, speed and climb.   
                       Fix auto detect S.Port, damn typo in v2.65.12.                                         
v2.66.00               Workaround to slow SITL telem under Ubuntu 20.04
v2.66.01               Alt & hdg display fix     
v2.66.02               Sats & rssi display fix 
          2021-07-04   Add support for Pi Pico board RP2040 
v2.67.00  2021-07-27   Fix Mavlink UDP out via AP, broken when S.Port UDP added. :(
v2.67.01  2021-07-29   PIO - add support for JTAG / ESP-PROG adapter  
v2.67.02  2021-08-18   Add pin for on-the-fly reset of EEPROM/NVM to default settings in config.h
v2.67.03  2021-08-19   NVM reset pins for more variants.
v2.67.04  2021-08-19   Add NVM reset pins for ESP8266
                       Tinfo and Pinfo pins obsolete, removed code
v2.67.05  2021-09-11   Tidy up Mavlink BT to GCS    
V2.67.06  2021-11-25   Reset NVM settings to config settings if fw version change detected
                       On NVM reset call RawSettingsToStruct() and reboot  
V2.67.07  2021-11-30   Special configuration, wifi from FC, serial to GCS 
V2.67.08  2021-12-03   Fix OTA in AP mode with embedded jquery (acknowledgement M.Mastenbroek)
v2.67.09  2021-12-10   No web input fields for BT if BT not compiled in. (PR by Vabe7) 
v2.67.10  2021-12-13   Retro fix APM bat capacity request. 
v2.67.11  2022-01-13   Add F.Port to SBUS functionality 
                       Add SoftwareSerial option for mavSerial on ESP32 
        D 2022-01-14   Fix converting a string constant to ‘char* with (char*) cast                                     
                       First guess at SBUS pins on ESP32 variants 
v2.67.12  2022-01-31   Clean up ESP8266 compile (sbus options) 
v2.67.13  2022-02-01   Enable FC serial passthrough (on Teensy 3.x only)  
V2.67.14  2022-04-06   define RESET_NVS resurected     
v2.67.15  2022-05-05   Fixed nasty transposition of udp local and remote port numbers                                                                                                                                                                                                                                                                                                                                                 
                                           
v2.67.16  2022-05-12   Clear sb[idx].inuse at end of popNexFrame()      
          2022-05-24   Improve FC and GCS uart read 
          2022-05-25   Refresh web interface.  
v2.67.17  2022-05-31   Revert battery current UOM fro cA back to dA (v2.62.8) - ninja-zx11 
       bc  2022-06-02  Also OLED disply, pt_bat1_amps * 0.1F -> 
       d   2022-06-02  Also ILI9341_Display, and mAh!
v2.67.18  2022-06-04   Fix OLED display amps display, 0x5003 uom is dA, but divided again   
v2.68.00  2022-06-07   Add support for command_long from GCS 
v2.68.01  2022-06-10   Correct 5009 waypoints, 500A rpm, 500B terrain, 50f1 servo_raw, 50f2 hud, 50f3 wind
          2022-06-28   Update printMavBuffer().
v2.68.02  2022-07 25   Improve udp remote client fc/gcs reporting 
v2.68.03  2022-10-22   Revise current calculation from Mav to pt yet again  
v2.68.04
v2.68.05
v2.68.06  2023-08-11   Add support for LilyGo T-Display-S3  
                       FC WiFi and GCS serial special variant for marc dornan     
                       Fix wifi failover bug introduced in v2.68.02
                       Ignore bug lines 536 and 644 of TFT_eSPI.cpp in TFT-eSPI library TFT_CS not initialised, 
                         digitalWrite(TFT_CS, HIGH) throws an error message:E (2521) gpio: gpio_set_level(226): 
                         GPIO output gpio_num error
v2.68.07 2023-08-17    Minor fix for debug report of "Passthru out Bat1 0x5003" 
v2.68.08 2023-10-06    PIO WIP version only. Heltec V3 board no pins for fc + GS serial. Macro out.                        
*/
