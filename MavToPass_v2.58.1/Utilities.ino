//=================================================================================================  
//================================================================================================= 
//
//                                       U T I L I T I E S
//
//================================================================================================= 
//=================================================================================================
struct Battery {
  float    mAh;
  float    tot_mAh;
  float    avg_dA;
  float    avg_mV;
  uint32_t prv_millis;
  uint32_t tot_volts;      // sum of all samples
  uint32_t tot_mW;
  uint32_t samples;
  bool ft;
  };
  
struct Battery bat1     = {
  0, 0, 0, 0, 0, 0, 0, true};   

struct Battery bat2     = {
  0, 0, 0, 0, 0, 0, 0, true};   

//=================================================================================================  
void ServiceStatusLeds() {
  ServiceMavStatusLed();
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
void PrintByte(byte b) {
  if (b == 0x7E) {
    Debug.println();
    clm = 0;
  }
  if (b<=0xf) Debug.print("0");
  Debug.print(b,HEX);
  Debug.print(" ");
  clm++;
  if (clm > 30) {
    Debug.println();
    clm=0;
  }
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
      PrintByte(bytes[0]);   // CRC1
      PrintByte(bytes[1]);   // CRC2
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
      PrintByte(bytes[i]); 
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
     PrintByte(bytes[i]);     
    }
    if (j == -2) {
      Debug.print("//");
      PrintByte(bytes[mav_len + 8]); 
      PrintByte(bytes[mav_len + 9]); 
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
      PrintByte(bytes[0]);   // CRC1
      PrintByte(bytes[1]);   // CRC2 
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
     PrintByte(bytes[i]); 
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
      PrintByte(bytes[i]); 
    }
    Debug.println();
  }

   Debug.print("Raw: ");
   for (int i = 0; i < 40; i++) {  //  unformatted
      PrintByte(bytes[i]); 
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
void ShowPeriod(bool LF) {
  Debug.print("Period mS=");
  now_millis=millis();
  Debug.print(now_millis-prev_millis);
  if (LF) {
    Debug.print("\t\n");
  } else {
   Debug.print("\t");
  }
    
  prev_millis=now_millis;
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
#if defined ESP32 || defined ESP8266

  
  void SenseWiFiPin() {
 
   #if defined Start_WiFi
    if (!wifiSuDone) {
      SetupWiFi();
    }
    return;
  #endif
  
  WiFiPinState = digitalRead(startWiFiPin);
  if ((WiFiPinState == 0) && (!wifiSuDone)) {
    SetupWiFi();
    }  
}

  //==================================================
  void SetupWiFi() { 
    bool apFailover = false;            // used when STA fails to connect
    
  
    apFailover = byte(EEPROM.read(0));  //  Read first eeprom byte
    #if defined Debug_Eeprom
      Debug.print("Read EEPROM apFailover = "); Debug.println(apFailover); 
    #endif

    //=====================================  S T A T I O N ========================================== 

  if ((set.wfmode == sta) || (set.wfmode == sta_ap))  {  // STA mode or STA failover to AP mode
    if (!apFailover) {   
      uint8_t retry = 0;
      WiFi.disconnect(true);   // To circumvent "wifi: Set status to INIT" error bug
      delay(500);
      if (WiFi.mode(WIFI_STA)) {
         Debug.println("Wi-Fi mode set to STA sucessfully");  
      } else {
        Debug.println("Wi-Fi mode set to STA failed!");  
      }
      Debug.print("Trying to connect to ");  
      Debug.print(set.staSSID); 
      OledPrintln("WiFi trying ..");
      delay(500);
      
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
        delay(500);
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        localIP = WiFi.localIP();  // TCP and UDP
        Debug.println();
        Debug.println("WiFi connected!");
        Debug.print("Local IP address: ");
        Debug.print(localIP);
        #if   (WiFi_Protocol == 1)   // TCP
          Debug.print("  port: ");
          Debug.println(tcp_localPort);    //  UDP port is printed lower down
        #else 
          Debug.println();
        #endif 
 
        wifi_rssi = WiFi.RSSI();
        Debug.print("WiFi RSSI:");
        Debug.print(wifi_rssi);
        Debug.println(" dBm");

        OledPrintln("Connected!");
        OledPrintln(localIP.toString());
        
        if (set.wfproto == tcp)  {   // TCP
          TCPserver.begin();                     //  tcp socket started
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
      
      if (set.wfproto == tcp)  {      // TCP
          TCPserver.begin();             //  Server for TCP/IP traffic
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

  #if defined webSupport
    WebServerSetup();  
    Debug.print("Web support active on http://"); 
    Debug.println(localIP.toString().c_str());
    OledPrintln("webSupport active");      
  #endif
      
  #ifndef Start_WiFi  // if not button override
    delay(2000);      // debounce button press
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

#if defined ESP32 

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
    Serial.printf("Initialising file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File initialised");
    } else {
        Serial.println("Write failed");
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
    Debug.print("  Path: "); Serial.println(sPath); 

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
    SenseWiFiPin();
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
