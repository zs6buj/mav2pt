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
//                           D I S P L A Y   S U P P O R T   -   ESP Only - for now
//================================================================================================= 
    #if defined Display_Support  

    void IRAM_ATTR gotButtonUp(){
      upButton = true;
    }

    void IRAM_ATTR gotButtonDn(){
      dnButton = true;  
    }

    //===================================
    void Scroll_Display(scroll_t up_dn) {
      
      if (millis() - scroll_millis < 300) return;
      scroll_millis = millis(); 
      
      if (up_dn == up) {
         scroll_row--;
         scroll_row = constrain(scroll_row, screen_height, row);
         upButton = false; 
         PaintDisplay(scroll_row, show_last_row);   // paint down to scroll_row
      }
      if (up_dn == down) {
          scroll_row++; 
          scroll_row = constrain(scroll_row, screen_height, row);       
          dnButton = false; 
          PaintDisplay(scroll_row, show_last_row);  // paint down to scroll_row      
      }   
    }
    
    //===================================
    void PaintDisplay(uint8_t to_row, last_row_t last_row_action) {   // paint down to scroll_row

        #if (defined ST7789_Display)
          display.fillScreen(screenBackground);
        #elif (defined SSD1306_Display)
          display.clearDisplay();
        #endif  
        display.setCursor(0,0);  
        uint8_t last_row = (last_row_action==omit_last_row) ? (to_row - 1) : to_row ;
        for (int i = (to_row - screen_height); i < last_row; i++) {  // display all old lines & leave space for new line at the bottom 
          display.println(ScreenRow[i].x);
        }
   
        #if (defined SSD1306_Display)
          display.display();
        #endif  
    }
    #endif    
    //===================================
    void DisplayPrintln(String S) {
    #if defined Display_Support      
      scroll_row = row; 
      if (row > (screen_height - 1)) {           // if the new line exceeds the page lth, re-display existing lines
        PaintDisplay(row, omit_last_row);
      }
      
      display.println(S);                        // display the new line, which is now the last line
      #if (defined SSD1306_Display)
        display.display();
      #endif  
      
      uint8_t lth = strlen(S.c_str());           // store the new line a char at a time
      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
        if (col > max_col-1) break;
      } 
      for (col=col ; col < max_col-1; col++) {   //  padd out the new line to eol
        ScreenRow[row].x[col] = '\0';
      }
      
      col = 0;
      row++;
    #endif       
    } // ready for next line

    //===================================
    void DisplayPrint(String S) {
      #if defined Display_Support  
      scroll_row = row; 
      if (row > (screen_height - 1)) {          // if the new line exceeds the page lth, re-display existing lines
        PaintDisplay(row, omit_last_row);
      }
      display.print(S);                         // the new line
      #if (defined SSD1306_Display)
        display.display();
      #endif 
       
      uint8_t lth = strlen(S.c_str());          // store the line a char at a time
      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
        if (col > max_col-1) break;
      } 
      for (col=col ; col < max_col-1; col++) {  //  padd out to eol
        ScreenRow[row].x[col] = '\0';
      }
      
      if (col > max_col-1) {   // only if columns exceeded, increment row
        col = 0;
        row++;
       }
    #endif    
    } // ready for next line

 /*
    try these
    display.drawCentreString("Font size 4", 120, 30, 4); // Draw text centre at position 120, 30 using font 4
    display.drawString(" is pi", xpos, ypos, font); 

 */
 

//=================================================================================================   
//                             W I F I   S U P P O R T   -   ESP Only - for now
//=================================================================================================  
#if (defined wifiBuiltin)

  void IRAM_ATTR gotWifiButton(){
    if (millis() - debnceTimr < delaytm) return;
    debnceTimr = millis();  
    wifiButton = true;
    
  }
  //==========================================
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
          DisplayPrintln("Wifi link restored"); 
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
    if ((wifiButton) && (!wifiSuDone)) {
      wifiButton = false;
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
    Debug.println("WiFi link lost - retrying");
    DisplayPrintln("Wifilink lost - retry"); 
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

    UDP_remoteIP_B3[0] = 255;                         // initialise the first udp remote client ip for broadcast
    for (int i = 1 ; i < max_clients ; i++) {         // and zero the others
      UDP_remoteIP_B3[i] = 0; 
    }

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
      DisplayPrintln("WiFi trying ..");    
      nbdelay(500);
      
      WiFi.begin(set.staSSID, set.staPw);
      while (WiFi.status() != WL_CONNECTED){
        retry++;
        if (retry > 10) {
          Debug.println();
          Debug.println("Failed to connect in STA mode");
          DisplayPrintln("No connect STA Mode");
          if (set.wfmode == sta_ap) {  // STA failover to AP mode
            apFailover = true;         
            Debug.println("Failover to AP. Rebooting ....");
            DisplayPrintln("Failover to AP");  
            apFailover = 1;                // set STA failover to AP flag
            EEPROM.write(0, apFailover);   // (addr, val)  
            EEPROM.commit();
            #if defined Debug_Eeprom
              Debug.print("Write EEPROM apFailover = "); Debug.println(apFailover); 
            #endif          
            delay(1000);
            ESP.restart();                 // esp32 and esp8266
          }  
          
          break;
        }
        nbdelay(500);
        Debug.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        localIP = WiFi.localIP();  // TCP and UDP
   
        UDP_remoteIP = localIP;    // Initially broadcast on the subnet we are attached to. patch by Stefan Arbes. 
        UDP_remoteIP[3] = 255;     // patch by Stefan Arbes  
                                   
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

        DisplayPrintln("Connected!");
        DisplayPrintln(localIP.toString());

        if (set.wfproto == tcp)  {   // TCP
          
          if (set.fc_io == fc_wifi) {  // if we expect wifi from the fc, we are a client, so connect ..
            WiFiClient newClient;        
            while (!newClient.connect(TCP_remoteIP, TCP_remotePort)) {
              Debug.println("Local outgoing tcp client connect failed, retrying");
              outbound_clientGood = false;
              nbdelay(5000);
            }
          active_client_idx = 0;     // use the first client object for  our single outgoing session   
          clients[0] = new WiFiClient(newClient); 
          Debug.print("Local tcp client connected to remote server IP:"); Debug.print(TCP_remoteIP);
          Debug.print(" remote Port:"); Debug.println(TCP_remotePort);

          DisplayPrintln("Client connected");
          DisplayPrintln("to remote TCP IP =");
          DisplayPrintln(TCP_remoteIP.toString()); 
          outbound_clientGood = true;
        }

         
          TCPserver.begin(set.tcp_localPort);                     //  tcp server socket started
          Debug.println("TCP server started");  
          DisplayPrintln("TCP server started");
        }

        if (set.wfproto == udp)  {  // UDP
          UDP.begin(set.udp_localPort);
          Debug.printf("UDP started, listening on IP %s, port %d \n", localIP.toString().c_str(), set.udp_localPort);
          snprintf(snprintf_buf, max_col, "Local port = %d", set.udp_localPort);     
          DisplayPrintln(snprintf_buf);
          #if defined UDP_Broadcast      
            UDP_remoteIP[3] = 255;    // in this case UDP always broadcasts on the AP 192.168.4/24 subnet
          #endif          
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
      //bool WiFiAPClass::softAP(const char* ssid, const char* passphrase, int channel, int ssid_hidden, int max_connection)
      WiFi.softAP(set.apSSID, set.apPw, set.channel);
      
      localIP = WiFi.softAPIP();   // tcp and udp

      Debug.print("AP IP address: ");
      Debug.print (localIP); 
      snprintf(snprintf_buf, max_col, "AP IP = %s", localIP.toString().c_str());        
      DisplayPrintln(snprintf_buf);  
      
      Debug.print("  SSID: ");
      Debug.println(String(set.apSSID));
      DisplayPrintln("WiFi AP SSID =");
      snprintf(snprintf_buf, max_col, "%s", set.apSSID);        
      DisplayPrintln(snprintf_buf);  
      
      if (set.wfproto == tcp)  {         // TCP
          TCPserver.begin(set.tcp_localPort);   //  Server for TCP/IP traffic     
          Debug.printf("TCP/IP started, listening on IP %s, TCP port %d\n", localIP.toString().c_str(), set.tcp_localPort);
          snprintf(snprintf_buf, max_col, "TCP port = %d", set.tcp_localPort);        
          DisplayPrintln(snprintf_buf);        
        }

      if (set.wfproto == udp)  {      // UDP
          UDP.begin(set.udp_localPort);
          Debug.printf("UDP started, listening on IP %s, port %d \n", WiFi.softAPIP().toString().c_str(), set.udp_localPort);
          snprintf(snprintf_buf, max_col, "UDP port = %d", set.udp_localPort);        
          DisplayPrintln(snprintf_buf);            
          #if defined UDP_Broadcast   
            UDP_remoteIP[2] = 4;     
            UDP_remoteIP[3] = 255;    // in this case UDP always broadcasts on the AP 192.168.4/24 subnet
          #endif  
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
      DisplayPrintln("webSupport active");  
    }  else {
      Debug.println("No web support possible"); 
      DisplayPrintln("No web support!");  
    }
  #endif

  wifiSuDone = true;
    
 } 

  //=================================================================================================  
   void PrintRemoteIP() {
    if (FtRemIP)  {
      FtRemIP = false;
      Debug.print("UDP client identified, remote IP: "); Debug.print(UDP_remoteIP);
      Debug.print(", remote port: "); Debug.println(set.udp_remotePort);
      DisplayPrintln("UDP client connected");
      DisplayPrintln("Remote IP =");
      snprintf(snprintf_buf, max_col, "%s", UDP_remoteIP.toString().c_str());        
      DisplayPrintln(snprintf_buf);        
      snprintf(snprintf_buf, max_col, "Remote port = %d", set.udp_remotePort);        
      DisplayPrintln(snprintf_buf);    
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
      DisplayPrintln("Writing Tlog");
    } else
    if  (set.sport_sd == spsd_on) {
      sPath = "/Mav2PT "  + DateTimeString(dt_tm) + ".splog"; 
      DisplayPrintln("Writing SPlog");  
    }
  //  Debug.print("Path: "); Debug.println(sPath); 

    strcpy(cPath, sPath.c_str());
    writeFile(SD, cPath , "Mavlink to FrSky Passthru by zs6buj");
    sdStatus = 3;      
   }

#endif      //  ESP Only - for now   
   //================================================================================================= 
   
   void SP_Byte_To_SD(byte SD_Byte) {  // SPort to SD
    
    #if ((defined ESP32) || (defined ESP8266)) && (defined SD_Support) 
    
    if (sp_msg_id > 0xff) return;  // lets just write MavLite right now
    if  (set.sport_sd == spsd_on) {   //  Sport to SD Card
      
    if (sd_idx < sd_buf_sz) {  // buffering
      sd_buf[sd_idx] = SD_Byte;
      sd_idx++; 
      return; 
    }
      
      if (sdStatus == 3) {     //  if open for write
          File file = SD.open(cPath, FILE_APPEND);
          if(!file){
             Debug.println("Failed to open file for appending");
             sdStatus = 9;
             return;
            }
            
          if(file.write(sd_buf, sd_buf_sz)){   // write contents of buffer to SD
            } else {
            Debug.println("Append failed");
           }          
          if(file.print(SD_Byte)){
          } else {
            Debug.println("SD Write failed");
          }
          
          sd_idx = 0;
          file.close();
        
          #ifdef  Debug_SD
            for (int i = 0 ; i < sd_buf_sz ; i++) {
              PrintByteNon(sd_buf[i]);
              if ((i>0) && (i%40 == 0)) Debug.println();
            }
           Debug.println("|");
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
  Debug.printf("AutoBaud - Sensing mav_rxPin %2d \n", rxPin );
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
  DisplayPrintln("Telem found at " + String(gb_baud));

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
  pt_payload |= dw_and_mask; 
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

    void PrintFrPeriod(bool LF) {
      now_millis=millis();
      now_micros=micros();

      uint32_t period = now_millis - prev_pt_millis;
      if (period < 10) {
        period = now_micros - prev_pt_micros;
        Debug.printf(" FrPeriod uS=%d", period);
      } else {
        Debug.printf(" FrPeriod mS=%d", period);
      }

      if (LF) {
        Debug.print("\t\n");
      } else {
       Debug.print("\t");
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
        #if defined btBuiltin
          #undef btBuiltin
        #endif
        Debug.println("Bluetooth disabled to free up SRAM for web support. YOU MUST PROCEED TO REBOOT for BT to be restored!"); 
        DisplayPrintln("Bluetooth disabled");         
        #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
          Debug.printf("==============>Free Heap after bluetooth disabled = %d\n", ESP.getFreeHeap());
        #endif
      }
    #endif
    }
    //=================================================================== 
