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
            isrCount = isrCount;  // silence irritating compiler warning 
            isrTime = isrTime;    // silence irritating compiler warning              
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
     if (set.wfmode == ap_sta) {
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
 
   if ((set.wfmode == sta) || (set.wfmode == ap_sta) || (set.wfmode == sta_ap) )  {  // STA mode or AP_STA mode or STA failover to AP mode
     if (!apFailover) {   
     
      uint8_t retry = 0;
      WiFi.disconnect(true);   // To circumvent "wifi: Set status to INIT" error bug
      nbdelay(500);
      
      if (set.wfmode == ap_sta) {
        if (WiFi.mode(WIFI_AP_STA)) {
           Log.println("WiFi mode set to AP_STA sucessfully"); 
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
        Log.println();
        Log.println("WiFi connected!");
                
        /*use mdns for host name resolution*/
        if (!MDNS.begin(HostName)) { //http://<HostName>.local
          Log.println("Error setting up MDNS responder!");
          while (1) {
           delay(1000);
         }
        }
        Log.println("mDNS responder started"); 

        localIP = WiFi.localIP();  // TCP and UDP
   
        UDP_remoteIP = localIP;    // Initially broadcast on the subnet we are attached to. patch by Stefan Arbes. 
        UDP_remoteIP[3] = 255;     // patch by Stefan Arbes  
                               
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
          if (set.fc_io == fc_wifi) {    // if we expect wifi from the fc, we are a client, so need a new session
             outbound_clientGood = NewOutboundTCPClient();
          }
   
          TCPserver.begin(set.tcp_localPort);                     //  tcp server socket started
          Log.println("TCP server started");  
          LogScreenPrintln("TCP server started");
        }

        if ( (set.mav_wfproto == udp) || (set.fr_io & 0x02) ) {  // UDP

          if (set.wfmode == ap_sta) {           // in WIFI_AP_STA mode, we need to discriminate between sta and ap read ports 
            udp_read_port = set.udp_remotePort; // so we swap read and send ports as a device
            udp_send_port = set.udp_localPort;           
          } else {                              // simple sta mode
            udp_read_port = set.udp_localPort; 
            udp_send_port = set.udp_remotePort;                          
          }
          if (set.mav_wfproto == udp) {
            WiFiUDP UDP_STA_Object;        
            udp_object[0] = new WiFiUDP(UDP_STA_Object);   
            Log.printf("Begin UDP using STA UDP object  read port:%d  send port:%d\n", udp_read_port, udp_send_port);                                                             
            udp_object[0]->begin(udp_read_port);      // there are 2 possible udp objects, STA [0]    and    AP [1]  
          }
          
          if (set.fr_io & 0x02)  {  // UDP
            Log.printf("Begin UDP using Frs UDP object  read port:%d  send port:%d\n", set.udp_localPort+1, set.udp_remotePort+1);                       
            frs_udp_object.begin(set.udp_localPort+1);          // use local port + 1 for Frs out                   
          }
          
          UDP_remoteIP = localIP;        
          UDP_remoteIP[3] = 255;           // broadcast until we know which ip to target     

          if ((set.wfmode == ap_sta) || (set.fc_io == fc_wifi)) {      // if wifi both sides OR fc wifi                        
            udpremoteip[0] = UDP_remoteIP;                             // [0] IP reserved for FC side              
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
       
       std::string s = set.host;
       std::transform(s.begin(), s.end(), s.begin(),
           [](unsigned char c) -> unsigned char { return std::tolower(c); });
       Log.printf("Web support active on http://%s.local\n", s.c_str()); 
       
       //Log.println(localIP.toString().c_str());
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
      delay(100);
      Log.print("AP_default_IP:"); Log.print(AP_default_IP); // these print statement give the module time to complete the above setting
      Log.print("  AP_gateway:"); Log.print(AP_gateway);  
      Log.print("  AP_mask:"); Log.println(AP_mask);    
      WiFi.softAPConfig(AP_default_IP, AP_gateway, AP_mask);
      
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

      /*use mdns for host name resolution*/
      if (!MDNS.begin(HostName)) { //http://<HostName>.local
        Log.println("Error setting up MDNS responder!");
        while (1) {
          delay(1000);
        }
      }
      Log.println("mDNS responder started"); 
      
      if (set.mav_wfproto == tcp)  {         // TCP
          TCPserver.begin(set.tcp_localPort);   //  Server for TCP/IP traffic     
          Log.printf("TCP/IP started, local IP:port %s:%d\n", localIP.toString().c_str(), set.tcp_localPort);
          snprintf(snprintf_buf, snp_max, "TCP port = %d", set.tcp_localPort);        
          LogScreenPrintln(snprintf_buf);        
        }

        if ( (set.mav_wfproto == udp) || (set.fr_io & 0x02) ) {  // UDP
           
          udp_read_port = set.udp_localPort;                   // we don't ever swap send and receive ports here
          udp_send_port = set.udp_remotePort;  
                      
          // Start FrSky UDP Object 
          if (set.fr_io & 0x02) {  
            Log.printf("Begin UDP using Frs UDP object  read port:%d  send port:%d\n", set.udp_localPort+1, set.udp_remotePort+1);                    
            frs_udp_object.begin(set.udp_localPort+1);          // use local port + 1 for Frs out                   
          } 
          
          // Start Mavlink UDP Object 
          WiFiUDP UDP_STA_Object;    
          udp_object[1] = new WiFiUDP(UDP_STA_Object);         
          Log.printf("Begin UDP using AP UDP object  read port:%d  send port:%d\n", udp_read_port, udp_send_port);                      
          udp_object[1]->begin(udp_read_port);    // there are 2 possible udp objects, STA [0]    and    AP [1]            
               
          UDP_remoteIP = WiFi.softAPIP();
          UDP_remoteIP[3] = 255;           // broadcast until we know which ip to target       

          // Now initialise the first entry of the udp targeted ip table 
          // FC side uses udpremoteip[0] and GCS side uses udpremoteip[1]
          // S.Port out and Mavlink out share GCS side

          udpremoteip[0] = UDP_remoteIP;
          udpremoteip[1] = UDP_remoteIP; 
          
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
    if (period > 5000) Log.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  }
    
  prev_lp_millis=now_millis;
  prev_lp_micros=now_micros;
}



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
  
uint32_t Get_Current_Average2(uint16_t dA)  {

  if (bat2.avg_dA == 0) bat2.avg_dA = dA;  // Initialise first time

  bat2.avg_dA = (bat2.avg_dA * 0.666) + (dA * 0.333);  // moving average

  Accum_mAh2(dA);  
  return bat2.avg_dA;
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
int8_t THR_To_63(uint16_t value) {       // THR -100 to 100   ->    nominal -63 to 63
int8_t myint;
  myint = round(value * 0.63); 
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
/* Not used right now, keep for possible future use
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
*/
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
//=================================================================================================   
//                           D I S P L A Y   S U P P O R T   -   ESP Only - for now
//================================================================================================= 

  #if defined displaySupport  
    void HandleDisplayButtons() {

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
      
     #if ((defined ESP32) || (defined ESP8266))         
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

        #if (defined RP2040)
          upButton = digitalRead(Pup);       // high = pressed  
          dnButton = digitalRead(Pdn);            
        #else  // esp32, esp8266 or teensy
          upButton = !digitalRead(Pup);       // low = pressed 
          dnButton = !digitalRead(Pdn);               
        #endif       

        //Log.printf("upButton:%u  dnButton:%u\n", upButton, dnButton);
        if (upButton) {                 
          Scroll_Display(up);
        }      
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
    
    #endif 
    //===================================
   
    void Scroll_Display(scroll_t up_dn) {
      
      if (millis() - scroll_millis < 300) return;
      show_log = true;    
      scroll_millis = millis(); 
      
      if (up_dn == up) {  // towards last line painted, so lines move up
         scroll_row--;
         scroll_row = constrain(scroll_row, scr_h_ch, row);
         upButton = false; 
         PaintLogScreen(scroll_row, show_last_row);   // paint down to scroll_row
      }
      if (up_dn == down) {  // towards first line painted, so lines move down
          scroll_row++; 
          scroll_row = constrain(scroll_row, scr_h_ch, row);       
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
        int8_t first_row;
        int8_t last_row;
        if (row < scr_h_ch) {
          first_row = (last_row_action==omit_last_row) ? 1 : 0; 
          last_row = (last_row_action==omit_last_row) ? new_row : new_row ;           
        } else {
          first_row = (last_row_action==omit_last_row) ? (new_row - scr_h_ch +1) : (new_row - scr_h_ch); 
          last_row = (last_row_action==omit_last_row) ? new_row : (new_row );            
        }     
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
          PaintLogScreen(row, omit_last_row);
      } else {   
        if (row >= scr_h_ch) {                      // if the new line exceeds the page lth, re-display existing lines
          PaintLogScreen(row, omit_last_row);
        }
      }
      uint16_t lth = strlen(S.c_str());           // store the new line a char at a time
      if (lth > scr_w_ch) {    
        Log.printf("Display width of %d exceeded for |%s|\n", scr_w_ch, S.c_str());  // scr_w_ch = max_col-1
        lth = scr_w_ch-1;  // prevent array overflow
      }

      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
      } 

      for (col=col ; col  < scr_w_ch; col++) {    //  padd out the new line to eol
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
          PaintLogScreen(row, omit_last_row);
      } else {   
        if (row >= scr_h_ch) {                      // if the new line exceeds the page lth, re-display existing lines
          PaintLogScreen(row, omit_last_row);
        }
      }
      display.print(S);                         // the new line
      #if (defined SSD1306_Display)
        display.display();
      #endif 
       
      uint8_t lth = strlen(S.c_str());          // store the line a char at a time
      if (lth > scr_w_ch) {
        Log.printf("Display width of %d exceeded for |%s|\n", scr_w_ch, S.c_str());  // scr_w_ch = max_col-1
        lth = scr_w_ch-1;  // prevent array overflow
      }  

      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
      } 
      for (col=col ; col < scr_w_ch; col++) {  //  padd out to eol
        ScreenRow[row].x[col] = '\0';
      }
      
      if (col > scr_w_ch-1) {   // only if columns exceeded, increment row
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
          draw_horizon(ap_roll, ap_pitch, scr_w_px, scr_h_px);
          
          SetScreenSizeOrient(2, SCR_ORIENT);   // text size, screen orientation, 26 ch wide x 15 ch deep
          
          // sats visible
          xx = 0;
          yy = 1 * char_h_px;;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "Sats:%d", ap24_sat_visible); 
          display.fillRect(xx +(5*char_w_px), yy, 2 * char_w_px, char_h_px, ILI9341_BLUE); // clear the previous line               
          display.println(snprintf_buf);  

          // heading (yaw)
          xx = 9 * char_w_px;
          yy = 1 * char_h_px;;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "Hdg:%.0f%", cur.hdg);
          display.fillRect(xx+(4*char_w_px), yy, 4 * char_w_px, char_h_px, ILI9341_BLUE); // clear the previous line                                
          display.println(snprintf_buf);

          // Radio RSSI
          xx = 17 * char_w_px;
          yy = 1 * char_h_px; ;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "RSSI:%d%%", pt_rssi); 
          display.fillRect(xx+(5*char_w_px), yy, 4 * char_w_px, char_h_px, ILI9341_BLUE); // clear the previous line               
          display.println(snprintf_buf);

          // Motors Armed
          xx = 0;
          yy = 3 * char_h_px;
          display.setCursor(xx, yy);         
          display.println("Arm:");
          if (motArmed) {
            display.fillRect(xx+(4*char_w_px), yy, char_w_px, char_h_px, ILI9341_GREEN);                        
          } else {
            display.fillRect(xx+(4*char_w_px), yy, char_w_px, char_h_px, ILI9341_RED);             
          }
          // Ground Speed
          xx = 7 * char_w_px;
          yy = 3 * char_h_px;                 
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Spd %2.1f", ap74_grd_spd); 
          display.fillRect(xx+(4*char_w_px), yy, (5 * char_w_px), char_h_px, ILI9341_BLUE);    // blank speed 
          display.println(snprintf_buf);  

          //  Climb m/s         
          xx = 16 * char_w_px;
          yy = 3 * char_h_px;                 
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Climb %2.1f", ap74_climb); 
          display.fillRect(xx+(6*char_w_px), yy, (5 * char_w_px), char_h_px, ILI9341_BLUE);    // blank climb 
          display.println(snprintf_buf);   
           
          // distance from home
          xx = 0;
          yy = 10 * char_h_px;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Dist:%dm", pt_home_dist);    // m 
          display.fillRect(xx+(5*char_w_px), yy, (5*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 

          // arrow from home to uav, or uav to home
          xx = 14 * char_w_px;
          yy = 10 * char_h_px;   
          draw_hud_arrow(xx, yy, pt_home_angle, scr_w_px, scr_h_px);
   
          // altitude above home
          xx = 17 * char_w_px;
          yy = 10 * char_h_px;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Alt:%3.0f", cur.alt_ag);    // m 
          display.fillRect(xx+(4*char_w_px), yy, (4*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 

          // voltage
          xx = 0;
          yy = 16 * char_w_px;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "V:%.1fV", pt_bat1_volts * 0.1F);     
          display.fillRect(xx+(2*char_w_px), yy, (6*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 
          
          // current
          xx = 9 * char_w_px;
          yy = 16 * char_w_px;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "A:%.0f", pt_bat1_amps * 0.1F);     
          display.fillRect(xx+(2*char_w_px), yy, (6*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 
          
          // Ah consumed
          xx = 18 * char_w_px;
          yy = 16 * char_w_px;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Ah:%.1f", pt_bat1_mAh * 0.001F);     
          display.fillRect(xx+(3*char_w_px), yy, (5*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf);           
          
          // latitude and logitude
          SetScreenSizeOrient(1, SCR_ORIENT);   // text size, screen orientation             
          xx = 0;
          yy = 28 * char_h_px;                  // chars now small (6px wide, 6px high, screen 30ch h x 53 ch w)
          display.setCursor(xx,yy);       
          snprintf(snprintf_buf, snp_max, "Lat:%.7f", cur.lat);
          display.fillRect(xx, yy, (15*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line        
          display.println(snprintf_buf);  
          xx = 35 * char_w_px;   
          yy = 28 * char_h_px;  
          display.setCursor(xx, yy);    
          snprintf(snprintf_buf, snp_max, "Lon:%.7f", cur.lon);
          display.fillRect(xx, yy, 21 * char_w_px, char_h_px, ILI9341_BLUE); // clear the previous line            
          display.println(snprintf_buf);  
          SetScreenSizeOrient(2, SCR_ORIENT);   // text size, screen orientation, 26 ch wide x 15 ch deep
          display_mode = flight_info;
        }
        
      #else  // other display types
      
        if (millis() - info_millis > 2000) {    // refresh rate
          info_millis = millis();  

           // Number of Sats 
          xx = 0;
          yy = 0 * char_h_px;      
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Sats %d", ap24_sat_visible); 
          display.fillRect(xx+(5*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);  // blank sats
          display.println(snprintf_buf);         
          #if (defined SSD1306_Display)
            display.display();
          #endif   
          
          // RSSI
          xx = 9 * char_w_px;
          yy = 0 * char_h_px;      
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "RSSI %u%%", pt_rssi); 
          display.fillRect(xx+(5*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);   // blank rssi  
          display.println(snprintf_buf);         
          #if (defined SSD1306_Display)
            display.display();
          #endif  
                            
          // Altitude 
          xx = 0;
          #if (defined SSD1306_Display) 
            yy = 1.2 * char_h_px;  
          #elif (defined SSD1331_Display)
            yy = 1.1 * char_h_px;            
          #elif (defined ST7789_Display)   
            yy = 1.8 * char_h_px;  
          #endif  
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Alt %.0f", (cur.alt_ag)); 
          display.fillRect(xx+(4*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);  // blank Alt
          display.println(snprintf_buf);  
          
          // Heading
          xx = 9 * char_w_px;
          #if (defined SSD1306_Display) 
            yy = 1.2 * char_h_px;  
          #elif (defined SSD1331_Display)
            yy = 1.1 * char_h_px;            
          #elif (defined ST7789_Display)   
            yy = 1.8 * char_h_px;  
          #endif  
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Hdg %.0f", cur.hdg); 
          display.fillRect(xx+(4*char_w_px), yy, (5 * char_w_px), char_h_px, SCR_BACKGROUND); // blank Hdg
          display.println(snprintf_buf);  
              
          // Ground Speed m/s
          xx = 0;
          #if (defined SSD1306_Display) 
            yy = 2.4 * char_h_px;  
          #elif (defined SSD1331_Display)
            yy = 2.2 * char_h_px;               
          #elif (defined ST7789_Display)   
            yy = 3.6 * char_h_px;  
          #endif               
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Spd %2.1f", ap74_grd_spd); 
          display.fillRect(xx+(4*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);    // blank speed 
          display.println(snprintf_buf);  
                 
          // Climb m/s
          xx = 9 * char_w_px;
          #if (defined SSD1306_Display) 
            yy = 2.4 * char_h_px;  
          #elif (defined SSD1331_Display)
            yy = 2.2 * char_h_px;               
          #elif (defined ST7789_Display)   
            yy = 3.6 * char_h_px;  
          #endif               
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Clm %2.1f", ap74_climb); 
          display.fillRect(xx+(4*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);   // blank climb 
          display.println(snprintf_buf);        
        
          // Volts, Amps and Ah 
          xx = 0;
          #if (defined SSD1306_Display) 
            yy = 3.6 * char_h_px;  
          #elif (defined SSD1331_Display)
            yy = 3.3 * char_h_px;                
          #elif (defined ST7789_Display)   
            yy = 5.4 * char_h_px;  
          #endif            
          display.setCursor(xx, yy);               
          snprintf(snprintf_buf, snp_max, "%2.1fV %2.0fA %2.1fAh", pt_bat1_volts * 0.1F, pt_bat1_amps * 0.1F, pt_bat1_mAh * 0.001F);     
          display.fillRect(xx, yy, scr_w_px, char_h_px, SCR_BACKGROUND); // clear the whole line  
          display.println(snprintf_buf); 

          // Latitude and Longitude
          xx = 0;
          
          #if (defined SSD1306_Display) 
            yy = 4.8 * char_h_px; 
            display.setCursor(xx,yy);       
            snprintf(snprintf_buf, snp_max, "Lat %3.7f", cur.lat);               
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear lat
            display.println(snprintf_buf);
            yy = 6.0 * char_h_px; 
            snprintf(snprintf_buf, snp_max, "Lon %3.7f", cur.lon);  
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear lon
            display.println(snprintf_buf);  
            display.display();  // NB NB dont forget me, SSD1306 only
            
          #elif (defined SSD1331_Display)       
            yy = 4.4 * char_h_px;
            display.setCursor(xx,yy);       
            snprintf(snprintf_buf, snp_max, "Lat %3.7f", cur.lat);               
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear lat
            display.println(snprintf_buf);
            yy = 5.5 * char_h_px; 
            snprintf(snprintf_buf, snp_max, "Lon %3.7f", cur.lon);  
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear lon
            display.println(snprintf_buf); 
                                                                  
          #elif (defined ST7789_Display)         
            yy = 7.2 * char_h_px;  
            display.setCursor(xx,yy);       
            snprintf(snprintf_buf, snp_max, "Lat %3.7f Lon %3.7f", cur.lat, cur.lon);        
            SetScreenSizeOrient(TEXT_SIZE -1, SCR_ORIENT);  // small text size       
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear the previous data 
            display.fillRect(xx+(16*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); 
            display.println(snprintf_buf);  
            SetScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);  // restore text size           
          #endif             
        }
      #endif    
    } 
    #endif    
    
    //===================================
    #if defined displaySupport  
    
    void SetupLogDisplayStyle() {

      SetScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);
       
      #if (defined ST7789_Display)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
        #if (SCR_ORIENT == 0)           // portrait
          display.setRotation(0);       // or 4 
          display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6 
        #elif (SCR_ORIENT == 1)         // landscape
          display.setRotation(3);       // or 1 
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
             
      #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display
        //  software SPI pins defined in config.h 
        display.fillScreen(BLACK);
        display.setCursor(0,0);
        #define SCR_BACKGROUND BLACK  
        
      #elif (defined ILI9341_Display)           // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
        //  hardware SPI pins defined in config.h 
        display.fillScreen(ILI9341_BLUE);    
        display.setCursor(0,0);
       //#if (SCR_ORIENT == 0)              // portrait
       //   display.setRotation(2);          // portrait pins at the top rotation      
       // #elif (SCR_ORIENT == 1)            // landscape
       //   display.setRotation(3);          // landscape pins on the left    
        //#endif 
        #define SCR_BACKGROUND ILI9341_BLUE 
      #endif

    }
   #endif
   //===================================
   #if defined displaySupport  
    
    void SetupInfoDisplayStyle() {

      SetScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);

      #if (defined ST7789_Display)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
        #if (SCR_ORIENT == 0)           // portrait
          SetScreenSizeOrient(1, 0);    // change text size
          display.setTextSize(1);       // and font
          display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6 
        #elif (SCR_ORIENT == 1)         // landscape
          SetScreenSizeOrient(2, 3);    // change text size
          display.setTextFont(1);       // and font 
        #endif    

      display.fillScreen(SCR_BACKGROUND);
      display.setTextColor(TFT_SKYBLUE);    
            
      //display.setTextColor(TFT_WHITE);
      //display.setTextColor(TFT_BLUE);  
      //display.setTextColor(TFT_GREEN, TFT_BLACK);
    
      #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display
        display.clearDisplay(); 
        display.setTextColor(WHITE);  
 
      #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display
        //  SPI pins defined in config.h 
        display.fillScreen(BLACK);
        display.setTextColor(WHITE);  
        #define SCR_BACKGROUND BLACK  
      
      #elif (defined ILI9341_Display)            // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
        //  SPI pins defined in config.h 
       display.fillScreen(ILI9341_BLUE);
        display.setRotation(3);          // landscape pins on the left   
        display.setCursor(0,0);
        #define SCR_BACKGROUND ILI9341_BLUE      
      #endif
     }
    #endif        
    //=================================================================== 
    #if (defined displaySupport) && (defined ILI9341_Display)
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
    void draw_hud_arrow(int16_t x, int16_t y, int16_t home_angle, int16_t width, int16_t height) {

      int16_t x0, y0, x1, y1, x2, y2, x3, y3;
      static int16_t px0, py0, px1, py1, px2, py2, px3, py3;
      int16_t opp, adj, hyp, opp90, adj90, hyp90;
      const int16_t al = 40;  // arrow length  
      static bool ft = true;
      const float AngleToRad = PI / 180;
      int16_t arr_angle = 270 - home_angle; // change direction of rotation and correct angle
      
      if (HUD_ARROW_OFFSET == 999) {
        //hom.hdg = 360.0 - 90.0;  // fix hom.hdg to west for testing
        arr_angle = arr_angle - hom.hdg + 180;   // hdg is from uav to home, so home to uav +180          
      } else {
        arr_angle += HUD_ARROW_OFFSET;
      }
         
      arr_angle = (arr_angle < 0) ? arr_angle + 360 : arr_angle;
      arr_angle = (arr_angle > 360) ? arr_angle - 360 : arr_angle;   
       
      hyp = al / 2;        
      opp = hyp * sin(arr_angle * AngleToRad);
      adj = hyp * cos(arr_angle * AngleToRad);
      
      hyp90 = al / 5; 
      opp90 = hyp90 * sin((arr_angle + 90) * AngleToRad);
      adj90 = hyp90 * cos((arr_angle + 90) * AngleToRad); 
      
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
    #if defined displaySupport       
    void SetScreenSizeOrient(uint8_t txtsz, uint8_t scr_orient) {

      #if (defined SSD1306_Display) || (defined SSD1331_Display)   // rotation arguement depends on display type
        if (scr_orient == 0) {          // portrait
          display.setRotation(1);              
          scr_h_px = SCR_H_PX;
          scr_w_px = SCR_W_PX;
        } else                       
        if (scr_orient == 1) {          // landscape
          display.setRotation(0);             
          scr_h_px = SCR_W_PX;
          scr_w_px = SCR_H_PX;
        }
      #else                             // ST7789 (T-Display) and ILI9341_Display
        if (scr_orient == 0) {          // portrait
          display.setRotation(0);       // or 4            
          scr_h_px = SCR_H_PX;
          scr_w_px = SCR_W_PX;
        } else
        if (scr_orient == 1) {          // landscape
          display.setRotation(3);       // or 1         
          scr_h_px = SCR_W_PX;
          scr_w_px = SCR_H_PX;
        }
      #endif
        
      display.setTextSize(txtsz);   
      
      if (txtsz == 1) {
        char_w_px = 6;    
      } else 
      if(txtsz == 2) {       
        char_w_px = 12;   
      } else       
      if  (txtsz == 3) { 
        char_w_px = 18;   
      } else         
      if  (txtsz == 4) {
        char_w_px = 24;    
      } else           
      if  (txtsz == 5)  {
        char_w_px = 30;    
      }     
   
      char_h_px = (uint8_t)(char_w_px * 1.4);   // vertical spacing

      scr_w_ch = scr_w_px / char_w_px;
      scr_h_ch = scr_h_px / char_h_px;       
    }
    #endif
    //===================================================================  
     
