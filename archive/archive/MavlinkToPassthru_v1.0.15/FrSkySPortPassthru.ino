     
// Frsky variables     
short    crc;                         // of frsky-packet
uint8_t  time_slot_max = 12;              
uint32_t time_slot = 1;
float a, az, c, dis, dLat, dLon;
uint8_t rc_count = 0;

#if (Target_Board == 0) // Teensy3x
volatile uint8_t *uartC3;
enum SPortMode { rx , tx };
SPortMode mode, modeNow;

void setSPortMode(SPortMode mode);

void setSPortMode(SPortMode mode) {   // To share single wire on tx pin

  if(mode == tx && modeNow !=tx) {
    *uartC3 |= 0x20;                 // Switch S.Port into send mode
    modeNow=mode;
    #ifdef Frs_Debug_All
    Debug.println("tx");
    #endif
  }
  else if(mode == rx && modeNow != rx) {   
    *uartC3 ^= 0x20;                 // Switch S.Port into receive mode
    modeNow=mode;
    #ifdef Frs_Debug_All
    Debug.println("rx");
    #endif
  }
}
#endif

// ***********************************************************************
void FrSkySPort_Init(void)  {

 frSerial.begin(frBaud); 

#if (Target_Board == 0) // Teensy3x
 #if (SPort_Serial == 1)
  // Manipulate UART registers for S.Port working
   uartC3   = &UART0_C3;  // UART0 is Serial1
   UART0_C3 = 0x10;       // Invert Serial1 tx levels
   UART0_C1 = 0xA0;       // Switch Serial1 into single wire mode
   UART0_S2 = 0x10;       // Invert Serial1 rx levels;
   
 //   UART0_C3 |= 0x20;    // Switch S.Port into send mode
 //   UART0_C3 ^= 0x20;    // Switch S.Port into receive modearmed
 #else
   uartC3   = &UART2_C3;  // UART2 is Serial3
   UART2_C3 = 0x10;       // Invert Serial1 tx levels
   UART2_C1 = 0xA0;       // Switch Serial1 into single wire mode
   UART2_S2 = 0x10;       // Invert Serial1 rx levels;
 #endif
#endif   
} 
// ***********************************************************************

#if defined Air_Mode || defined Relay_Mode
void ReadSPort(void) {
  uint8_t prevByt=0;
  #if (Target_Board == 0) // Teensy3x
    setSPortMode(rx);
  #endif  
  uint8_t Byt = 0;
  while ( frSerial.available())   {  
    Byt =  frSerial.read();
    #ifdef Frs_Debug_All
    DisplayByte(Byt);
    #endif

    if ((prevByt == 0x7E) && (Byt == 0x1B)) { 
      #if defined Debug_Air_Mode
        Debug.print("S/S "); 
      #endif
      FrSkySPort_Process(); 

    }     
  prevByt=Byt;
  }
  // and back to main loop
}  
#endif
// ***********************************************************************

#if defined Ground_Mode
void Emulate_ReadSPort() {
#if (Target_Board == 0)   // Teensy3x
  setSPortMode(tx);
  #endif
 
  FrSkySPort_Process();  


  // and back to main loop
}
#endif
// ***********************************************************************

void FrSkySPort_Process() {

  #ifdef Frs_Debug_All
    ShowPeriod();   
  #endif  
       
  fr_payload = 0; // Clear the payload field
    
  // ******** Priority processes ***********************

  // NONE !
  
  if (!mavGood) return;  // Wait for good Mavlink data
  
  // ********************************************************** 
  //  One time-slot per "sensor" ID, which ID type can use or donate to next in line 
    #ifdef Frs_Debug_All
      Debug.print(" time_slot=");
      Debug.println(time_slot);
    #endif
    switch(time_slot) {
        case 1:
         #if defined Ground_Mode || defined Relay_Mode      // In Air_Mode the FrSky receiver provides rssi
         if (millis() - rssi_F101_millis > 500) {           // 2 Hz
           rssi_F101_millis = millis();
           SendRssiF101();                                 // Regularly tell LUA script in Taranis we are connected
           break;
           }
         else
            time_slot++;   // Donate the slot to next
         #endif  
        
        case 2:                  // data id 0x800 Latitude 
          if (millis() - lat800_millis > 333)  {  // 3 Hz
            SendLat800();
            lat800_millis = millis();
            break; 
            }
          else
            time_slot++;   // Donate the slot to next
                    
        case 3:                  // data id 0x800 Longitude
          if (millis() - lon800_millis > 333)  {  // 3 Hz
            SendLon800();
            lon800_millis = millis();
            break; 
            }
          else
            time_slot++;   // Donate the slot to next 
                         
        case 4:                 // data id 0x5000 Status Text
          if (((!MsgRingBuff.isEmpty()) || (fr_chunk_pntr > 0)) && (millis() - ST5000_millis > 200)) { // 5 Hz
            SendStatusTextChunk5000();
            ST5000_millis = millis();
            break;
           }
          else
            time_slot++;   // Donate the slot to next 

        case 5:                 // data id 0x5001 AP Status
          if (millis() - AP5001_millis > 333)  {  // 3 Hz
            SendAP_Status5001();
            AP5001_millis = millis();
            break; 
             }
          else
            time_slot++;   // Donate the slot to next 
        case 6:                 // data id 0x5002 GPS Status
          if (millis() - GPS5002_millis > 333)  {  // 3 Hz
            Send_GPS_Status5002();
            GPS5002_millis = millis();
            break; 
            }
          else
            time_slot++;   // Donate the slot to next 
          
        case 7:                  //data id 0x5003 Batt 1
          if (millis() - Bat1_5003_millis > 1000)  {  // 1 Hz
            Send_Bat1_5003();
            Bat1_5003_millis = millis();
            break; 
            }
          else
            time_slot++;   // Donate the slot to next 
                   
        case 8:                  // data id 0x5004 Home
          if (millis() - Home_5004_millis > 500)  {  // 2 Hz
            Send_Home_5004();
            Home_5004_millis = millis();
            break; 
            }
          else
            time_slot++;   // Donate the slot to next 
   
        case 9:        // data id 0x5005 Velocity and yaw
          if (millis() - VelYaw5005_millis > 500)  {  // 2 Hz
            Send_VelYaw_5005();
            VelYaw5005_millis = millis();
            break; 
            }
          else
            time_slot++;   // Donate the slot to next 
   
        case 10:        // data id 0x5006 Attitude and range
          if (millis() - Atti5006_millis > 200)  {  // 5 Hz 
            Send_Atti_5006();
            Atti5006_millis = millis();
            break; 
            }
          else
            time_slot++;   // Donate the slot to next 
            
        case 11:       // data id 0x5007 Parameters 
          if (millis() - Param5007_millis > 5000) {        // 0.2 Hz send the 5007 parameters to keep FlightDeck happy
            SendParameters5007();
            Param5007_millis = millis();
            break; 
            }
          else
            time_slot++;   // Donate the slot to next
              
        case 12:          // data id 0x5008 Batt 2
          if ((fr_bat2_mAh > 0) && (millis() - Bat2_5008_millis > 1000))  {  // 1 Hz
            Send_Bat2_5008();
            Bat2_5008_millis = millis();
            break; 
            }
          else
            time_slot++;   // Donate the slot to next                             
        default:
         // ERROR
         break;
      }
     time_slot++;
     if(time_slot > time_slot_max) time_slot = 1;

 }     
// ***********************************************************************
void FrSkySPort_SendByte(uint8_t byte, bool addCrc) {
#if (Target_Board == 0)   // Teensy3x
   setSPortMode(tx); 
 #endif  
 if (!addCrc) { 
   frSerial.write(byte);  
   return;       
 }

 CheckByteStuffAndSend(byte);
 
  // update CRC
	crc += byte;       //0-1FF
	crc += crc >> 8;   //0-100
	crc &= 0x00ff;
	crc += crc >> 8;   //0-0FF
	crc &= 0x00ff;
}
// ***********************************************************************
void CheckByteStuffAndSend(uint8_t byte) {
 if (byte == 0x7E) {
   frSerial.write(0x7D);
   frSerial.write(0x5E);
 } else if (byte == 0x7D) {
   frSerial.write(0x7D);
   frSerial.write(0x5D);  
 } else {
   frSerial.write(byte);
   }
}
// ***********************************************************************
void FrSkySPort_SendCrc() {
  uint8_t byte;
  byte = 0xFF-crc;

 CheckByteStuffAndSend(byte);
 
 // DisplayByte(byte);
 // Debug.println("");
  crc = 0;          // CRC reset
}
//***************************************************
void FrSkySPort_SendDataFrame(uint8_t Instance, uint16_t Id, uint32_t value) {

#if (Target_Board == 0)   // Teensy3x
  setSPortMode(tx); 
  #endif
  
  #ifdef Ground_Mode   // Only if ground mode send these bytes, else XSR sends them
    FrSkySPort_SendByte(0x7E, false);       //  START/STOP don't add into crc
    FrSkySPort_SendByte(Instance, false);   //  don't add into crc  
  #endif 
  
  FrSkySPort_SendByte(0x10, true );   //  Data framing byte
 
	uint8_t *bytes = (uint8_t*)&Id;
  #if defined Frs_Debug_Payload
    Debug.print("DataFrame. ID "); 
    DisplayByte(bytes[0]);
    Debug.print(" "); 
    DisplayByte(bytes[1]);
  #endif
	FrSkySPort_SendByte(bytes[0], true);
	FrSkySPort_SendByte(bytes[1], true);
	bytes = (uint8_t*)&value;
	FrSkySPort_SendByte(bytes[0], true);
	FrSkySPort_SendByte(bytes[1], true);
	FrSkySPort_SendByte(bytes[2], true);
	FrSkySPort_SendByte(bytes[3], true);
  
  #if defined Frs_Debug_Payload
    Debug.print("Payload (send order) "); 
    DisplayByte(bytes[0]);
    Debug.print(" "); 
    DisplayByte(bytes[1]);
    Debug.print(" "); 
    DisplayByte(bytes[2]);
    Debug.print(" "); 
    DisplayByte(bytes[3]);  
    Debug.print("Crc= "); 
    DisplayByte(0xFF-crc);
    Debug.println("/");  
  #endif
  
	FrSkySPort_SendCrc();
   
}
//***************************************************
  uint32_t bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth) {
  uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
  return r;
}
//***************************************************
// Mask then AND the shifted bits, then OR them to the payload
  void bit32Pack(uint32_t dword ,uint8_t displ, uint8_t lth) {   
  uint32_t dw_and_mask =  (dword<<displ) & (createMask(displ, displ+lth-1)); 
  fr_payload |= dw_and_mask; 
}
//***************************************************
uint32_t createMask(uint8_t lo, uint8_t hi) {
  uint32_t r = 0;
  for (unsigned i=lo; i<=hi; i++)
       r |= 1 << i;  
  return r;
}
// *****************************************************************
// *****************************************************************

void SendLat800() {
  fr_gps_status = ap_fixtype < 3 ? ap_fixtype : 3;                   //  0 - 3
  if (fr_gps_status < 3) return;
  if (px4_flight_stack) {
    fr_lat = Abs(ap_lat24) / 100 * 6;  // ap_lat * 60 / 1000
    if (ap_lat24<0) 
      ms2bits = 1;
    else ms2bits = 0;    
  } else {
    fr_lat = Abs(ap_lat33) / 100 * 6;  // ap_lat * 60 / 1000
    if (ap_lat33<0) 
      ms2bits = 1;
    else ms2bits = 0;
  }
  
  bit32Pack(fr_lat, 0, 30);
  bit32Pack(ms2bits, 30, 2);
 // fr_payload = (ms2bits <<30) | fr_lat;
          
  #if defined Frs_Debug_All || defined Frs_Debug_LatLon
    Debug.print("Frsky out LatLon 0x800: ");   
    Debug.print(" fr_lat="); Debug.print(fr_lat);  
    Debug.print(" fr_payload="); Debug.println(fr_payload);
  #endif
          
  FrSkySPort_SendDataFrame(0x1B, 0x800, fr_payload);  
}
// *****************************************************************
void SendLon800() {
  fr_gps_status = ap_fixtype < 3 ? ap_fixtype : 3;                   //  0 - 3
  if (fr_gps_status < 3) return;
  if (px4_flight_stack) {
    fr_lon = Abs(ap_lon24) / 100 * 6;  // ap_lat * 60 / 1000
    if (ap_lat24<0) 
      ms2bits = 3;
    else ms2bits = 2;    
  } else {
    fr_lon = Abs(ap_lon33) / 100 * 6;  // ap_lat * 60 / 1000
    if (ap_lon33<0) 
      ms2bits = 3;
    else ms2bits = 2;
  }
  
  bit32Pack(fr_lon, 0, 30);
  bit32Pack(ms2bits, 30, 2);
          
  #if defined Frs_Debug_All || defined Frs_Debug_LatLon
    Debug.print("Frsky out LatLon 0x800: ");  
    Debug.print(" ap_lon="); Debug.print(ap_lon); 
    Debug.print(" fr_lon="); Debug.print(fr_lon); 
    Debug.print(" fr_payload="); Debug.println(fr_payload);
  #endif
          
  FrSkySPort_SendDataFrame(0x1B, 0x800, fr_payload); 
}
// *****************************************************************
void SendStatusTextChunk5000() {

  if (fr_chunk_pntr == 0) { 
    ST_record = (MsgRingBuff.shift());  // Get a status text message from front of queue
    fr_severity = ST_record.severity;
    fr_txtlth = ST_record.txtlth;
    memcpy(fr_text, ST_record.text, fr_txtlth+4);   // plus rest of last chunk at least
    fr_simple = ST_record.simple;

    #if defined Frs_Debug_All || defined Frs_Debug_Text
      Debug.print("Frsky out AP_Status 0x5000: ");  
      Debug.print(" fr_severity="); Debug.print(fr_severity);
      Debug.print(" "); Debug.print(MavSeverity(fr_severity)); 
      Debug.print(" Text= ");  Debug.print(" |"); Debug.print(fr_text); Debug.print("| ");
      Debug.print(" Msg queue length after shift= "); Debug.println(MsgRingBuff.size());
    #endif
  }

  while (fr_chunk_pntr+1 <= (fr_txtlth)) {                 // send multiple 4 byte (32b) chunks
  
    fr_chunk[0] = fr_text[fr_chunk_pntr];
    fr_chunk[1] = fr_text[fr_chunk_pntr+1];
    fr_chunk[2] = fr_text[fr_chunk_pntr+2];
    fr_chunk[3] = fr_text[fr_chunk_pntr+3];
    
    #if defined Frs_Debug_All || defined Frs_Debug_Text
      Debug.print(" fr_txtlth="); Debug.print(fr_txtlth); 
      Debug.print(" fr_chunk_pntr="); Debug.print(fr_chunk_pntr); 
      Debug.print(" "); 
      Debug.print(" |"); Debug.print(fr_chunk); Debug.println("| ");
    #endif  
    
    bit32Pack(fr_chunk[0], 24, 7);
    bit32Pack(fr_chunk[1], 16, 7);
    bit32Pack(fr_chunk[2], 8, 7);    
    bit32Pack(fr_chunk[3], 0, 7);  

    if ((fr_chunk[0] == 0x00) | (fr_chunk[1] == 0x00) | (fr_chunk[2] == 0x00) | (fr_chunk[3] == 0x00)) {

      bit32Pack((fr_severity & 0x1), 7, 1);            // ls bit of severity
      bit32Pack(((fr_severity & 0x2) >> 1), 15, 1);    // mid bit of severity
      bit32Pack(((fr_severity & 0x4) >> 2) , 23, 1);   // ms bit of severity                
      bit32Pack(0, 31, 1);     // filler

   // debug    fr_severity = (bit32Extract(fr_payload,23,1) * 4) + (bit32Extract(fr_payload,15,1) * 2) + (bit32Extract(fr_payload,7,1) * 1);
      
      #if defined Frs_Debug_All || defined Frs_Debug_Text  
        Debug.print(" fr_severity ="); Debug.print(fr_severity);
        Debug.print(" "); Debug.print(MavSeverity(fr_severity)); 
        bool lsb = (fr_severity & 0x1);
        bool sb = (fr_severity & 0x2) >> 1;
        bool msb = (fr_severity & 0x4) >> 2;
        Debug.print(" ls bit="); Debug.print(lsb); 
        Debug.print(" mid bit="); Debug.print(sb); 
        Debug.print(" ms bit ="); Debug.print(msb); 
        Debug.print(" Payload ="); Debug.print(fr_payload); 
        Debug.print(" (send order bytes) "); 
        uint8_t *bytes;
        bytes = (uint8_t*)&fr_payload;
        DisplayByte(bytes[3]);
        Debug.print(" "); 
        DisplayByte(bytes[2]);
        Debug.print(" "); 
        DisplayByte(bytes[1]);
        Debug.print(" "); 
        DisplayByte(bytes[0]);   
        Debug.println(); Debug.println();
     #endif 
     }

    
    FrSkySPort_SendDataFrame(0x1B, 0x5000, fr_payload); 
    fr_chunk_pntr +=4;
    return;     // Go around again
 }
  
  fr_chunk_pntr = 0;
   
}

// *****************************************************************
void SendAP_Status5001() {
  if (ap_type == 6) return;      // If GCS heartbeat ignore it  -  yaapu  - ejs also handled at #0 read
  
  fr_simple = ap_simple;         // Derived from "ALR SIMPLE mode on/off" text messages
  fr_armed = ap_base_mode >> 7;  
  fr_land_complete = fr_armed;
  
  if (px4_flight_stack) 
    fr_flight_mode = PX4FlightModeNum(px4_main_mode, px4_sub_mode);
  else   //  APM Flight Stack
    fr_flight_mode = ap_custom_mode + 1; // AP_CONTROL_MODE_LIMIT - ls 5 bits
  
  bit32Pack(fr_flight_mode, 0, 5);      // Flight mode   0-32 - 5 bits
  bit32Pack(fr_simple ,5, 2);           // Simple/super simple mode flags
  bit32Pack(fr_land_complete ,7, 1);    // Landed flag
  bit32Pack(fr_armed ,8, 1);            // Armed
  bit32Pack(fr_bat_fs ,9, 1);           // Battery failsafe flag
  bit32Pack(fr_ekf_fs ,10, 2);          // EKF failsafe flag
  bit32Pack(px4_flight_stack ,12, 1);   // px4_flight_stack flag

  #if defined Frs_Debug_All || defined Frs_Debug_APStatus
    Debug.print("Frsky out AP_Status 0x5001: ");   
    Debug.print(" fr_flight_mode="); Debug.print(fr_flight_mode);
    Debug.print(" fr_simple="); Debug.print(fr_simple);
    Debug.print(" fr_land_complete="); Debug.print(fr_land_complete);
    Debug.print(" fr_armed="); Debug.print(fr_armed);
    Debug.print(" fr_bat_fs="); Debug.print(fr_bat_fs);
    Debug.print(" fr_ekf_fs="); Debug.print(fr_ekf_fs);
    Debug.print(" px4_flight_stack="); Debug.println(px4_flight_stack);
  #endif
           
  FrSkySPort_SendDataFrame(0x1B, 0x5001, fr_payload);  
}
// *****************************************************************
void Send_GPS_Status5002() {

  if (ap_sat_visible > 15)
    fr_numsats = 15;
  else
    fr_numsats = ap_sat_visible;
  
  bit32Pack(fr_numsats ,0, 4); 
          
  fr_gps_status = ap_fixtype < 3 ? ap_fixtype : 3;                   //  0 - 3
  fr_gps_adv_status = ap_fixtype > 3 ? ap_fixtype - 3 : 0;           //  4 - 8 -> 0 - 3   

  fr_amsl = ap_amsl24 / 100;  // dm
  fr_hdop = ap_eph /10;
          
  bit32Pack(fr_gps_status ,4, 2);       // part a, 3 bits
  bit32Pack(fr_gps_adv_status ,14, 2);  // part b, 3 bits
          
  #if defined Frs_Debug_All || defined Frs_Debug_GPS_Status
    Debug.print("Frsky out GPS Status 0x5002: ");   
    Debug.print(" fr_numsats="); Debug.print(fr_numsats);
    Debug.print(" fr_gps_status="); Debug.print(fr_gps_status);
    Debug.print(" fr_gps_adv_status="); Debug.print(fr_gps_adv_status);
    Debug.print(" fr_amsl="); Debug.print(fr_amsl);
    Debug.print(" fr_hdop="); Debug.print(fr_hdop);
  #endif
          
  fr_amsl = prep_number(fr_amsl,2,2);                       // Must include exponent and mantissa    
  fr_hdop = prep_number(fr_hdop,2,1);
          
  #if defined Frs_Debug_All || defined Frs_Debug_GPS_Status
    Debug.print(" After prep: fr_amsl="); Debug.print(fr_amsl);
    Debug.print(" fr_hdop="); Debug.println(fr_hdop);  
  #endif     
              
  bit32Pack(fr_hdop ,6, 8);
  bit32Pack(fr_amsl ,22, 9);
  bit32Pack(0, 31,0);  // 1=negative 
          
  FrSkySPort_SendDataFrame(0x1B, 0x5002,fr_payload); 

}
// *****************************************************************  
void Send_Bat1_5003() {
  
  fr_bat1_volts = ap_voltage_battery1 / 100;         // Were mV, now dV  - V * 10
  fr_bat1_amps = ap_current_battery1 ;               // Remain       dA  - A * 10   
  
  // fr_bat1_mAh is populated at #147 depending on battery id
  //fr_bat1_mAh = Total_mAh1();  // If record type #147 is not sent by FC
  
  #if defined Frs_Debug_All || defined Debug_Batteries
    Debug.print("Frsky out Bat1 0x5003: ");   
    Debug.print(" fr_bat1_volts="); Debug.print(fr_bat1_volts);
    Debug.print(" fr_bat1_amps="); Debug.print(fr_bat1_amps);
    Debug.print(" fr_bat1_mAh="); Debug.println(fr_bat1_mAh);            
  #endif
          
  bit32Pack(fr_bat1_volts ,0, 9);
  fr_bat1_amps = prep_number(roundf(fr_bat1_amps * 0.1F),2,1);          
  bit32Pack(fr_bat1_amps,9, 8);
  bit32Pack(fr_bat1_mAh,17, 15);
            
  FrSkySPort_SendDataFrame(0x1B, 0x5003, fr_payload);             
}
// ***************************************************************** 
void Send_Home_5004() {
  if (fr_gps_status < 3) return;

    lon1=hom.lon/180*PI;  // degrees to radians
    lat1=hom.lat/180*PI;
    lon2=cur.lon/180*PI;
    lat2=cur.lat/180*PI;

    //Calculate azimuth bearing of craft from home
    a=atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1));
    az=a*180/PI;  // radians to degrees
    if (az<0) az=360+az;

    fr_home_angle = Add360(az, -180);                            // now is the angle from the craft to home in degrees
 // fr_home_arrow = Add360(fr_home_angle, -ap_gps_hdg/100);        //  NO, this is done in OSD in Taranis
  
    fr_home_arrow = fr_home_angle * 0.3333;                     // units of 3 degrees

    // Calculate the distance from home to craft
    dLat = (lat2-lat1);
    dLon = (lon2-lon1);
    a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
    c = 2* asin(sqrt(a));    // proportion of Earth's radius
    dis = 6371000 * c;       // mean radius of the Earth is 6371km

    if (homGood)
      fr_home_dist = (int)dis;
    else
      fr_home_dist = 0;

      fr_home_alt = ap_alt_ag / 100;    // mm->dm
        
   #if defined Frs_Debug_All || defined Frs_Debug_Home
     Debug.print("Frsky out Home 0x5004: ");         
     Debug.print("fr_home_dist=");  Debug.print(fr_home_dist);
     Debug.print(" fr_home_alt=");  Debug.print(fr_home_alt);
     Debug.print(" az=");  Debug.print(az);
     Debug.print(" fr_home_angle="); Debug.print(fr_home_angle);  
     Debug.print(" fr_home_arrow="); Debug.println(fr_home_arrow);         // units of 3 deg   
   #endif
   fr_home_dist = prep_number(roundf(fr_home_dist), 3, 2);
   bit32Pack(fr_home_dist ,0, 12);
   fr_home_alt = prep_number(roundf(fr_home_alt), 3, 2);
   bit32Pack(fr_home_alt ,12, 12);
   if (fr_home_alt < 0)
     bit32Pack(1,24, 1);
   else  
     bit32Pack(0,24, 1);
   bit32Pack(fr_home_arrow,25, 7);
   FrSkySPort_SendDataFrame(0x1B, 0x5004,fr_payload);

}

// *****************************************************************
void Send_VelYaw_5005() {
  
  fr_vy = ap_hud_climb * 10;   // from #74   m/s to dm/s;
  fr_vx = ap_hud_grd_spd * 10;  // from #74  m/s to dm/s

  //fr_yaw = (float)ap_gps_hdg / 10;  // (degrees*100) -> (degrees*10)
  fr_yaw = ap_hud_hdg * 10;              // degrees -> (degrees*10)
  
  #if defined Frs_Debug_All || defined Frs_Debug_YelYaw
    Debug.print("Frsky out VelYaw 0x5005:");  
    Debug.print(" fr_vy=");  Debug.print(fr_vy);       
    Debug.print(" fr_vx=");  Debug.print(fr_vx);
    Debug.print(" fr_yaw="); Debug.print(fr_yaw); 
  #endif
  if (fr_vy<0)
    bit32Pack(1, 8, 1);
  else
    bit32Pack(0, 8, 1);
  fr_vy = prep_number(roundf(fr_vy), 2, 1);  // Vertical velocity
  bit32Pack(fr_vy, 0, 8);   

  fr_vx = prep_number(roundf(fr_vx), 2, 1);  // Horizontal velocity
  bit32Pack(fr_vx, 9, 8);    
  fr_yaw = fr_yaw * 0.5f;                   // Unit = 0.2 deg
  bit32Pack(fr_yaw ,17, 11);  

 #if defined Frs_Debug_All || defined Frs_Debug_YelYaw
   Debug.print(" After prep:"); \
   Debug.print(" fr_vy=");  Debug.print((int)fr_vy);          
   Debug.print(" fr_vx=");  Debug.print((int)fr_vx);  
   Debug.print(" fr_yaw="); Debug.println((int)fr_yaw);                
 #endif

  FrSkySPort_SendDataFrame(0x1B, 0x5005,fr_payload);
}
// *****************************************************************  
void Send_Atti_5006() {
  fr_roll = (ap_roll * 5) + 900;             //  -- fr_roll units = [0,1800] ==> [-180,180]
  fr_pitch = (ap_pitch * 5) + 450;           //  -- fr_pitch units = [0,900] ==> [-90,90]
  fr_range = roundf(ap_range*100);   
  bit32Pack(fr_roll, 0, 11);
  bit32Pack(fr_pitch, 11, 10); 
  bit32Pack(prep_number(fr_range,3,1), 21, 11);
  #if defined Frs_Debug_All || defined Frs_Debug_Attitude
    Debug.print("Frsky out Attitude 0x5006: ");         
    Debug.print("fr_roll=");  Debug.print(fr_roll);
    Debug.print(" fr_pitch=");  Debug.print(fr_pitch);
    Debug.print(" fr_range="); Debug.print(fr_range);
    Debug.print(" Frs_Attitude Payload="); Debug.println(fr_payload);  
  #endif
  
  FrSkySPort_SendDataFrame(0x1B, 0x5006,fr_payload);      
}
//***************************************************
void SendParameters5007() {

  if (paramsID >= 5) {
    fr_paramsSent = true;          // get this done early on and then regularly thereafter
    paramsID = 0;
    return;
  }
  paramsID++;
    
  switch(paramsID) {
    case 1:                                    // Frame type
      fr_param_id = paramsID;
      fr_frame_type = ap_type;
        
      bit32Pack(fr_frame_type, 0, 24);
      bit32Pack(fr_param_id, 24, 4);

      FrSkySPort_SendDataFrame(0x1B, 0x5007,fr_payload); 

      #if defined Frs_Debug_All || defined Frs_Debug_Params
        Debug.print("Frsky out Params 0x5007: ");   
        Debug.print(" fr_param_id="); Debug.print(fr_param_id);
        Debug.print(" fr_frame_type="); Debug.println(fr_frame_type);           
      #endif
      
      break;
    case 2:                                   // Previously used to send the battery failsafe voltage
      break;
    case 3:                                   // Previously used to send the battery failsafe capacity in mAh
      break;
    case 4:                                   // Battery pack 1 capacity
      fr_param_id = paramsID;
      #if (Battery_mAh_Source == 2)    // Local
        fr_bat1_capacity = bat1_capacity;
      #elif  (Battery_mAh_Source == 1) //  FC
        fr_bat1_capacity = ap_bat1_capacity;
      #endif   
      bit32Pack(fr_bat1_capacity, 0, 24);
      bit32Pack(fr_param_id, 24, 4);
      FrSkySPort_SendDataFrame(0x1B, 0x5007,fr_payload); 

      #if defined Frs_Debug_All || defined Frs_Debug_Params || defined Debug_Batteries
        Debug.print("Frsky out Params 0x5007: ");   
        Debug.print(" fr_param_id="); Debug.print(fr_param_id);
        Debug.print(" fr_bat1_capacity="); Debug.println(fr_bat1_capacity);           
      #endif
      break;
    case 5:                                   // Battery pack 2 capacity
      fr_param_id = paramsID;
      #if (Battery_mAh_Source == 2)    // Local
        fr_bat2_capacity = bat2_capacity;
      #elif  (Battery_mAh_Source == 1) //  FC
        fr_bat2_capacity = ap_bat2_capacity;
      #endif   
      bit32Pack(fr_bat2_capacity, 0, 24);
      bit32Pack(fr_param_id, 24, 4);
      FrSkySPort_SendDataFrame(0x1B, 0x5007,fr_payload); 
      
      #if defined Frs_Debug_All || defined Frs_Debug_Params || defined Debug_Batteries
        Debug.print("Frsky out Params 0x5007: ");   
        Debug.print(" fr_param_id="); Debug.print(fr_param_id);
        Debug.print(" fr_bat2_capacity="); Debug.println(fr_bat2_capacity);           
      #endif
      
      break;
    }  
}
// ***************************************************************** 
void Send_Bat2_5008() {
  
   fr_bat2_volts = ap_voltage_battery1 / 100;         // Were mV, now dV  - V * 10
   fr_bat2_amps = ap_current_battery1 ;               // Remain       dA  - A * 10   
   
  // fr_bat2_mAh is populated at #147 depending on battery id
  //fr_bat2_mAh = Total_mAh2();  // If record type #147 is not sent and good
  
  #if defined Frs_Debug_All || defined Debug_Batteries
    Debug.print("Frsky out Bat1 0x5003: ");   
    Debug.print(" fr_bat2_volts="); Debug.print(fr_bat2_volts);
    Debug.print(" fr_bat2_amps="); Debug.print(fr_bat2_amps);
    Debug.print(" fr_bat2_mAh="); Debug.println(fr_bat2_mAh);            
  #endif        
          
  bit32Pack(fr_bat2_volts ,0, 9);
  fr_bat2_amps = prep_number(roundf(fr_bat2_amps * 0.1F),2,1);          
  bit32Pack(fr_bat2_amps,9, 8);
  bit32Pack(fr_bat2_mAh,17, 15);      

  FrSkySPort_SendDataFrame(0x1B, 0x5008,fr_payload);          
}

// *****************************************************************  
void SendRssiF101() {          // data id 0xF101 RSSI tell LUA script in Taranis we are connected

  if (rssiGood)
    //fr_rssi = (ap_chan16_raw - 1000)  / 10;  //  RSSI uS 1000=0%  2000=100%
    fr_rssi = (ap_rssi / 2.54);                // %
  else
    fr_rssi = 255;     // We may have a connection but don't yet know how strong. Prevents spurious "Telemetry lost" announcement
  #ifdef Frs_Dummy_rssi
    fr_rssi = 87;
    Debug.print(" Dummy rssi="); Debug.println(fr_rssi); 
  #endif
  bit32Pack(fr_rssi ,0, 32);
  
  #ifdef Relay_Mode
    FrSkySPort_SendByte(0x7E, false);   
    FrSkySPort_SendByte(0x1B, false);  
    FrSkySPort_SendDataFrame(0x1B, 0xF101,fr_payload); 
  #else
    FrSkySPort_SendDataFrame(0x1B, 0xF101,fr_payload); 
  #endif
     
  #if defined Frs_Debug_All || defined Mav_Debug_Rssi
    Debug.print("Frsky out: ");  
    Debug.print(" rssiGood="); Debug.print(rssiGood);       
    Debug.print(" rssi="); Debug.println(fr_rssi);                
  #endif
}
//***************************************************  
uint32_t Abs(int32_t num) {
  if (num<0) 
    return (num ^ 0xffffffff) + 1;
  else
    return num;  
}
//***************************************************
int16_t Add360(int16_t arg1, int16_t arg2) {
  int16_t ret = arg1 + arg2;
  if (ret < 0) ret += 360;
  if (ret > 359) ret -= 360;
  return ret;
}
//***************************************************
// From Arducopter 3.5.5 code
uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power)
{
    uint16_t res = 0;
    uint32_t abs_number = abs(number);

    if ((digits == 2) && (power == 1)) { // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
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
