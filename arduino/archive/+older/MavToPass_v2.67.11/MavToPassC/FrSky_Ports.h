//=================================================================================================  
//================================================================================================= 
//
//                                    F . P O R T   C L A S S
//
//================================================================================================= 
//=================================================================================================


#if defined frBuiltin   // FrSky port support

   #define fpStatusLed    99        // not used in this application
   #define InvertfpLed    false

  //    forward declaration of main and utility functions

  uint32_t  Abs(int32_t);
  float     RadToDeg(float );
  uint32_t  TenToPwr(uint8_t pwr);  
  uint16_t  prep_number(int32_t, uint8_t, uint8_t);
  uint32_t  bit32Extract(uint32_t,uint8_t, uint8_t); 
  void      bit32Pack(uint32_t, uint8_t, uint8_t);
  uint32_t  createMask(uint8_t, uint8_t);
  int16_t   Add360(int16_t, int16_t);
  float     wrap_360(int16_t);
  int8_t    PWM_To_63(uint16_t);
  int8_t    THR_To_63(uint16_t);
  void      nbdelay(uint32_t);
  void      LogScreenPrintln(String);
  void      Printbyte(byte, bool, char);
  void      PrintMavLiteUplink();
  void      PrintFrPeriod(bool);
  void      PrintLoopPeriod();
  uint8_t   PX4FlightModeNum(uint8_t, uint8_t);
  void      Mavlink_Param_Request_Read(int16_t, char *);
  void      Mavlink_Param_Set();
  void      Send_Mavlink_Command_Long();
  void      SP_Byte_To_SD(byte);
  String    MavSeverity(uint8_t);

  class     FrSkyPort 
  { 
    // Data members 
  public: 

  private:
  
    frport_t  frport_id = f_none;  
    
    static const uint8_t timeout_secs = 5;
    
    byte      chr = 0;
    byte      prev_chr = 0;    
    bool      fpInit = false; 
    bool      serGood = false;
    bool      frGood = false;
    bool      frPrev = false; 
    bool      pwmGood = false; 
    bool      pwmPrev = false;
    uint32_t  frGood_millis = 0;   
    uint32_t  serGood_millis = 0;  
    uint32_t  pwmGood_millis = 0;       

    bool      parseGood = false;
    uint32_t  goodFrames = 0;
    uint32_t  badFrames = 0;
    uint32_t  packetloss_millis = 0;
    uint8_t   fpLedState = LOW; 
    uint32_t  fpLed_millis = 0;
             
    #define   frameSize  38                   // len, type, 35(24ch control) + CRC = 38
    byte      fpbuf[frameSize];

    static const uint8_t max_ch = 26;         // 24 + 2 digi ch
    int16_t   pwm_ch[max_ch];                 // PWM Channels
    uint8_t   pwm_rssi = 0;
  
    int16_t   crcin = 0;                      // CRC of inbound frsky frame   
    int16_t   crcout = 0;   
    uint16_t  lth = 0;

    uint8_t   fr_lth = 0;
    uint8_t   fr_type = 0;
    uint8_t   fr_prime = 0;
    
    volatile uint8_t *uartC3;
    
    enum PortMode { rx , tx };
    PortMode mode, modeNow;

    bool      printcrcout = false;
    byte      nb, lb;                      // Nextbyte, Lastbyt of ByteStuff pair     
  
    uint8_t   time_slot_max = 16;              
    uint32_t  time_slot = 1;
    float     a, az, c, dis, dLat, dLon;
    uint8_t   sv_count = 0;

    uint8_t   prevbyt=0;

    bool      frInvert = false;
    uint32_t  frBaud = 0; 
    uint16_t  fr_idx = 0;
    uint8_t   p_fr = 0;   
    
    static const uint16_t  fr_max = 48;           // frport frame buffer  35 + 2 + headroom 
    byte      frbuf[fr_max];

    bool ftgetBaud = true;
    
    //=================================================================================
    //                      S  C  H  E  D  U  L  E  R

    // Give the F.Port table more space when status_text messages sent three times
    #if defined Send_status_Text_3_Times
      static const uint16_t sb_rows = 256;  // possible unsent sensor ids at any moment 
    #else 
     static const uint16_t sb_rows = 128;  
    #endif 
    uint16_t        sb_row = 0;
  
    typedef union {
    struct {
       uint32_t       passthru;       // 4B  passthru payload
       uint16_t       packing;        // 2B   
         };
       mtf_payload_t  mavlite;        // 6B  mavlite payload
      }
      sb_payload_t;                          

    // Scheduler identity for both Mavlink and MavLite
    typedef struct  {
      uint16_t      msg_id;          // 2B - note Mavlink needs 2 byte msg_id
      uint8_t       sub_id;          // 1B
      uint32_t      millis;          // 4B - mS since boot
      uint8_t       mt_idx;          // 1B - index if it is a MavLite frame class
      uint8_t       inuse;           // 1B
      sb_payload_t  payload;         // 6B
    } sb_t;
    
    sb_t          sb[sb_rows];                    // Scheduler buffer table


    uint32_t        sb_buf_full_gear = 0;        // downlink
    uint32_t        mt_buf_full_gear = 0;        // downlink
    uint32_t        mt20_buf_full_gear = 0;      // uplink

    uint32_t        sr_pt_payload;                // 4B
    uint16_t        sb_unsent;                    // how many sb rows in use  
    uint16_t        mt_unsent;                    // how many mt rows in use  

    // Member function prototypes
  public:  
    void          initialise();   
    void          HandleTraffic();
    void          PushMessage(uint16_t msg_id, uint8_t sub_id); 
    void          ReportFrPortOnlineStatus();      
    uint16_t      MatchWaitingParamRequests(char * paramid); 
    uint32_t      getBaud(uint8_t rxpin, pol_t pl);       
             
  private:  
    void          set_gpio_in_inv( uint8_t pin_num );  // set_gpio polarity for RP2040 only
    void          set_gpio_in_ninv( uint8_t pin_num );  
    void          set_gpio_out_inv( uint8_t pin_num );
    void          set_gpio_out_ninv( uint8_t pin_num );
    void          set_gpio_inout_pol( uint8_t pin_num, uint8_t in_out, uint8_t pol);
  
    pol_t         getPolarity(uint8_t pin);
    uint32_t      getConsistent(uint8_t rxpin, pol_t pl);
    uint32_t      SenseUart(uint8_t  rxpin, pol_t pl);          
    frport_t      Read_And_GetID_FrPort(); 
    frport_t      getID_FrPort(uint8_t);   
    void          FPort_Read_And_Write(uint8_t *buf, frport_t  frport_id);  
    void          SPort_Read_And_Write(uint8_t *buf);           
    void          setMode(PortMode mode);
    byte          ReadByte();    
    byte          SafeRead();
    void          WriteByte(byte b);   
    void          SafeWrite(byte b, bool isPayload);
    void          ReportOnlineStatus(); 
    bool          ParseFrame(uint8_t *buf, uint16_t lth);
    uint8_t       crcGet(uint8_t *buf, uint8_t lth);
    bool          crcGood(uint8_t *buf, uint8_t lth);                
    bool          BytesToPWM(uint8_t *buf, int16_t *ch, uint8_t lth); 
    void          WriteSBUS(uint8_t *buf);
    void          crcStepIn(uint8_t b);     
    void          crcStepOut(uint8_t b);  
    void          crcStep(int16_t *mycrc, uint8_t b); 
    void          crcEnd(int16_t *mycrc);          
    void          WriteCrc(); 
    void          Print_PWM_Channels(int16_t *ch, uint16_t max_ch); 
    void          PrintBuffer(uint8_t *buf, uint8_t lth);   
    void          CheckForTimeouts();   
    void          ServiceStatusLed();
    void          BlinkFpLed(uint32_t period);
    void          InjectUplinkFrame(uint8_t _prime);  
    uint16_t      PopNextFrame();  
    msg_class_t   msg_class_now(uint8_t msg_id);
    void          Send_Frsky_Frame(uint16_t idx, uint8_t _prime);    
    bool          DecodeAndUplinkMavLite();  
    void          ChunkMavLitePayload(uint8_t msg_id, uint8_t tlth);        // downlink 
    bool          UnchunkMavLitePayload();                                  // uplink 
    uint16_t      FirstEmptyMt20Row();
    void          PushToEmptyRow(uint16_t msg_id, uint8_t sub_id); 
    void          Push_Param_Val_016(uint16_t msg_id);  
    void          Push_Command_Ack_04d(uint16_t msg_id);   
    void          Push_Lat_800(uint16_t msg_id);  
    void          Push_Lon_800(uint16_t msg_id);
    void          Push_Text_Chunks_5000(uint16_t msg_id);
    void          Push_AP_status_5001(uint16_t msg_id);  
    void          Push_GPS_status_5002(uint16_t msg_id);
    void          Push_Bat1_5003(uint16_t msg_id);
    void          Push_Home_5004(uint16_t msg_id);
    void          Push_VelYaw_5005(uint16_t msg_id);
    void          Push_Atti_5006(uint16_t msg_id);   
    void          Push_Parameters_5007(uint16_t msg_id, uint8_t sub_id);
    void          Push_Bat2_5008(uint16_t msg_id);  
    void          Push_WayPoint_5009(uint16_t msg_id);
    void          Push_RPM_500A(uint16_t msg_id);
    void          Push_Terrain_500B(uint16_t msg_id);
    void          Push_Servo_Raw_50F1(uint16_t msg_id);    
    void          Push_VFR_Hud_50F2(uint16_t msg_id);    
    void          Push_Wind_Estimate_50F3(uint16_t msg_id);
    void          Push_Rssi_F101(uint16_t msg_id);   
    void          PrintPayload(msg_class_t msg_class, telem_direction_t telem_direction);                                                         
    void          PrintMavLiteUplink(); 
    String        frportName(frport_t);
        
}; // end of class

    // External member functions
    //===================================================================
    
  #if (defined RP2040)
   /*
    * IO_BANK0_GPIO0_CTRL
    * 
    * rp2040_datasheet.pdf
    * IO_BANK0_BASE 0x40014000
    * Offset 0x004 * GPIO number  GPIO0_CTRL  GPIO0 control, including function select and overrides pg 264
    * IO_BANK0 bits 17:16 INOVER 0x0/0x1(inverse)  pg 268
    * IO_BANK0 bits 9:8 OUTOVER 0x0/0x1(inverse)  pg 268
    * 
    * acknowledgement python code urbite https://talk.dallasmakerspace.org/t/raspberry-pi-pico-sbus-code-help/79375/14
    */
    
    void FrSkyPort::set_gpio_in_inv( uint8_t pin_num ) {
      set_gpio_inout_pol( pin_num, 1, 1);
    }

    void FrSkyPort::set_gpio_in_ninv( uint8_t pin_num ) {
      set_gpio_inout_pol( pin_num, 1, 0);     
    }

    void FrSkyPort::set_gpio_out_inv( uint8_t pin_num ) {
      set_gpio_inout_pol( pin_num, 0, 1);      // pin, direction, polarity
    }

    // set pin output to non-inverting
    void FrSkyPort::set_gpio_out_ninv( uint8_t pin_num ) {
      set_gpio_inout_pol( pin_num, 0, 0);
    } 
       
    void FrSkyPort::set_gpio_inout_pol( uint8_t pin_num, uint8_t dir, uint8_t  pol ) {  
       volatile int* var = (int*)(0x40014000 + (8*pin_num) + 4);  // get pointer to address
       uint32_t val = *var;  // get the value at addr ...             
       uint32_t pol_value = pol << (8 + (8*dir));
       uint32_t pol_mask = ~(3 << (8 + (8*dir)));  // ones compliment
       *var  = (val & pol_mask) | pol_value;  // changes the memory located at ....  
    }
  #endif // end of RP2040
  // ==================================================================================     
    void FrSkyPort::initialise()  {  

    for (int i=0 ; i < sb_rows ; i++) {  // initialise F.Port table
        sb[i].msg_id = 0;
        sb[i].sub_id = 0;
        sb[i].millis = 0;
        sb[i].inuse = 0;
    }
    
    for (int i = 0 ; i < mt20_rows ; i++) {  // initialise MavLite uplink table
      mt20[i].inuse = 0;
    }   

    typedef enum wires_set { one_wire = 1, two_wire = 2 } wire_t;  
    wire_t wire_mode;
    uint8_t sense_pin = 0;
    #if (defined TEENSY3X) 
      wire_mode = one_wire;
    #else
      wire_mode = two_wire;
    #endif

    pol_t pol = no_traffic;
    if (wire_mode == one_wire) {
      sense_pin = fr_txPin;
    } else {
      sense_pin = fr_rxPin;  
    }
    
    if (set.frport == f_auto) {
        pol = (pol_t)getPolarity(sense_pin); 
        bool ftp = true;
        static int8_t cdown = 5;     
        while ( (pol == no_traffic) && (cdown) ){
          if (ftp) {
            Log.printf("FrSky Port: No telem on pin:%d. Retrying ", sense_pin);
            String sPin=String(sense_pin);   // integer to string
            LogScreenPrintln("No telem on rxpin:"+ sPin);            
            ftp = false;
          }
          Log.print(cdown); Log.print(" ");
          pol = (pol_t)getPolarity(sense_pin);         
          delay(50);      
          if (cdown-- == 1 ) {
            Log.println();
            Log.println("Auto sensing abandoned. Defaulting to S.Port");
            LogScreenPrintln("Default to SPort!");
            pol = idle_low;  
            set.frport = s_port;  
            ftp = true;     
          }
        }  // end of while()
        
        if (!ftp) Log.println();
      
        if (pol == idle_low) {
          frInvert = true;
          Log.printf("Sensed serial port pin %d is IDLE_LOW, inverting polarity\n", sense_pin);
        } else {
          frInvert = false;
          Log.printf("Sensed serial port pin %d is IDLE_HIGH, regular rx polarity\n", sense_pin);        
        }
      
        if (set.frport == f_auto) {
          frBaud = getBaud(sense_pin, pol);
        }
    }    // end of f_auto   
    
    if (set.frport == s_port) {
        frInvert = true;
        frBaud = 57600;  
        Log.printf("S.Port at 57.6kb/s selected on rxpin:%d is IDLE_LOW, inverting rx polarity\n", fr_rxPin);           
    } 
       
    if (set.frport == f_port1) {  
        frInvert = false;
        frBaud = 115200; 
        Log.printf("F.Port1 at 115.2kb/s selected on rxpin:%d is IDLE_HIGH, regular rx polarity\n", fr_rxPin);                 
    }  
    
    if (set.frport == f_port2) {  
        frInvert = false;
        frBaud = 115200; 
        Log.printf("F.Port2 at 115.2kb/s selected on rxpin:%d is IDLE_HIGH, regular rx polarity\n", fr_rxPin);                
    }      
    
    // end of polarity and baud setup            


    #if (defined ESP32) || (defined ESP8266) 
      int8_t frRx;
      int8_t frTx;

      frRx = fr_rxPin;
      frTx = fr_txPin;

      #if ( (defined ESP8266) || ( (defined ESP32) && (defined ESP32_Frs_SoftwareSerial)) )
          if (set.trmode == ground) {
            Log.printf("FrSky is 1-wire simplex on tx pin:%d\n", frTx);
          } else { 
            Log.printf("FrSky on pins rx:%d and tx:%d\n", frRx, frTx);
          }  

          delay(100);
          frSerial.begin(frBaud, SWSERIAL_8N1, frRx, frTx, frInvert);     // SoftwareSerial
          delay(100);
          Log.println("Using SoftwareSerial for F.Port");

      #else  // ESP HardwareSerial
          delay(100);
          frSerial.begin(frBaud, SERIAL_8N1, frRx, frTx, frInvert); 
          delay(100);         
          if ((set.trmode == air) || (set.trmode == relay) || (set.Support_MavLite)) {
            Log.printf("FrSky half-duplex on pins rx:%d and tx:%d\n", frRx, frTx);  
            Log.println("For 1-wire applications a 2-wire to 1-wire converter or diode is required");
          } else          
          if (set.trmode == ground)  {                        
            Log.printf("FrSky simplex on tx pin = %d\n", frTx);
          }    
      #endif    
       
    #endif  // end of ESP block =======================================================

    #if (defined RP2040)
      int8_t frRx;
      int8_t frTx;

      frRx = fr_rxPin;
      frTx = fr_txPin;

      delay(100);
      frSerial.begin(frBaud, SERIAL_8N1); 
      delay(100);         
      if ((set.trmode == air) || (set.trmode == relay) || (set.Support_MavLite)) {
        Log.printf("FrSky half-duplex on pins rx:%d and tx:%d\n", frRx, frTx);  
        Log.println("For 1-wire applications a 2-wire to 1-wire converter or diode is required");
      } else          
      if (set.trmode == ground)  {                        
        Log.printf("FrSky simplex on tx pin = %d\n", frTx);
      } 
     if (frInvert) {
       FrSkyPort::set_gpio_in_inv(fr_rxPin); 
       FrSkyPort::set_gpio_out_inv(fr_txPin);    
     }
   
     
    #endif  // end of RP2040 block  =======================================================

    
    #if (defined TEENSY3X) 
      frSerial.begin(frBaud); // Teensy 3.x    tx pin hard wired
      #if (frPort_Serial == 1)
        // Manipulate UART registers for F.Port working
         uartC3   = &UART0_C3;  // UART0 is Serial1
         if (frInvert) {        // For S.Port not F.Port
           UART0_C3 = 0x10;       // Invert Serial1 Tx levels
           UART0_S2 = 0x10;       // Invert Serial1 Rx levels;       
         }
       #if (defined Teensy_One_Wire)  // default
         UART0_C1 = 0xA0;       // Switch Serial1 to single wire (half-duplex) mode  
         Log.printf("FrSky 1-wire half-duplex on Teensy3.x pin %d \n", fr_txPin);          
       #else
         Log.printf(" and 2-wire full-duplex on Teensy3.x pins rx=%d  tx=%x\n", fr_rxPin, fr_txPin);  
       #endif
        /*
        UART0_C3 |= 0x20;    // Switch F.Port into send mode
        UART0_C3 ^= 0x20;    // Switch F.Port into receive mode
        */
        
     #else
       uartC3   = &UART2_C3;  // UART2 is Serial3
       if (frInvert) {      
         UART2_C3 = 0x10;       // Invert Serial1 Tx levels
         UART2_S2 = 0x10;       // Invert Serial1 Rx levels;  
       }
       #if (defined Teensy_One_Wire)
         UART2_C1 = 0xA0;       // Switch Serial1 to single wire mode
         Log.printf(" and 1-wire half-duplex on Teensy3.x pin %d \n", fr_txPin);         
       #else
         Log.printf(" and 2-wire full-duplex on Teensy3.x pins rx=%d  tx=%x\n", fr_rxPin, fr_txPin);  
       #endif
          

     #endif
      
    #endif 
      
    } // end of member function

    //===================================================================
    void FrSkyPort::HandleTraffic() {    
      if (frport_id == f_none) {
        if (set.frport== f_auto) {     
          frport_id = Read_And_GetID_FrPort();   // loop thru until not f_none, avoids wait-on-read block    
          if (frport_id == s_port) {
            Log.println("SPort detected");         
          } else
          if (frport_id == f_port1) {        
            Log.println("FPort1 detected");   
          } else
          if (frport_id == f_port2) {        
            Log.println("FPort2 detected");   
          }
        } else {
          frport_id = set.frport;
        } 
      }

      if (set.trmode == ground) {   
          if(mavGood  && ((millis() - blind_inject_millis) > 24)) {
            fr_prime = 0x10;
            FrSkyPort::InjectUplinkFrame(fr_prime);        // Blind inject frame into Taranis et al 
            blind_inject_millis=millis();
          }
          if (!set.Support_MavLite) {
            return; // no need to read fr_port if ground mode and no mavlite support needed
          }
      }  

      FrSkyPort::CheckForTimeouts();

      if ( (frport_id == f_port1) || (frport_id == f_port2) ) { 

          #if defined TEENSY3X 
            if(mavGood && ((millis() - sp_millis) > 1)) {   // very necessary for Teensy 3.x
              sp_millis=millis();
              FPort_Read_And_Write(&FrSkyPort::frbuf[0], frport_id);
            }
          
          #else   // ESP32 and ESP8266
            FPort_Read_And_Write(&FrSkyPort::frbuf[0], frport_id);
          #endif   

      } else  
        if (frport_id == s_port) {  
          if(mavGood && ((millis() - sp_millis) > 2)) {   // timing very necessary for S.Port !! especially for Teensy 3.x
            sp_millis=millis();
            SPort_Read_And_Write(&FrSkyPort::frbuf[0]);
          }
      }      
    }
    
    //===================================================================  
    frport_t FrSkyPort::Read_And_GetID_FrPort() {
      uint8_t          b = 0;  
      
      FrSkyPort::setMode(rx);
      lth=frSerial.available();  
      if (lth < 10) {
        if (lth > 0) {
          //Log.printf("lth=%d\n", lth); 
        }
        return f_none;       // prevent 'wait-on-read' blocking
      }
      b = FrSkyPort::ReadByte();
      return getID_FrPort(b);     // read until not f_none avoids wait-on-read block 
    }  
    //===================================================================  
    frport_t FrSkyPort::getID_FrPort(uint8_t b) {      
      static int8_t   sp = 0;   
      static int8_t   fp1 = 0; 
      static int8_t   fp2 = 0;  
      static uint8_t   prev_b = 0; 
      
      //Log.printf("b:%X  prevb:%X  sp:%u  fp1:%u  fp2:%u\n", b, prev_b, sp, fp1, fp2);
      if ((prev_b == 0x7E) && ( (b == 0x08) || (b == 0x19))) { 
        fp1++;     
        sp = 0;
        if (fp1 > 10) {
          sp = 0;
          fp1 = 0;
          fp2 = 0;
          prev_b = 0;
          return f_port1;  // fport1
        }
      } else
      
      if ((prev_b == 0x7E) && (b == 0x1B)) {
        sp++;   
       //Log.printf("b:%X  prevb:%X  sp:%u xxxxxxxxxxxxxxxxxxxxxxxxx\n", b, prev_b, sp);   
        if (sp > 10) {
          sp = 0;
          fp1 = 0;
          fp2 = 0;
          prev_b = 0;
          return s_port;  // sport 
        }
      } else
      
      if ( ( (prev_b == 0x0D) || (prev_b == 0x18)|| (prev_b == 0x23) ) && (b == 0xff) )  {       // fp2 control   
        fp2++;                
        if (fp2 > 10) {
          sp = 0;
          fp1 = 0;
          fp2 = 0;         
          return f_port2;  // fport2
        }
      } else
      
      if ( ( (prev_b == 0x0D) || (prev_b == 0x18)|| (prev_b == 0x23) ) && ( (b == 0xf1) || (b == 0xf1) ) )  {  // fp2 OTA     
        fp2++;                  
        if (fp2 > 10) {
          sp = 0;
          fp1 = 0;
          fp2 = 0; 
          prev_b = 0;        
          return f_port2;
        }
      } else
      
      if ( (prev_b == 0x08) && (b < 0x1b) )   {     // fp2 downlink  
        fp2++;                
        if (fp2 > 10) {
          sp = 0;
          fp1 = 0;
          fp2 = 0; 
          prev_b = 0;         
          return f_port2;
        }
      } 
      
      prev_b = b;
      return f_none;  //  prevent read blocking
    }  

    //===================================================================
    
    void FrSkyPort::SPort_Read_And_Write(uint8_t *buf) {

    #if defined Debug_FrSPort_Loop_Period
      PrintLoopPeriod();
    #endif
      
      FrSkyPort::setMode(rx);  

      uint8_t byt = 0;

      while (frSerial.available())   {  // Receive sensor IDs from X receiver

        byt =  FrSkyPort::ReadByte();        // no bytestuff
        if (fr_idx > fr_max -1) {
         // Log.println("S.Port from X Receiver read buff overflow ignored");
          fr_idx--;
        }

        if ((prevbyt == 0x7E) && (byt == 0x1B)) {   // Real-Time DIY downlink slot found, slot ours in, and send right now 
           FrSkyPort::frGood = true;            
           FrSkyPort::frGood_millis = millis();         
           inject_delay = micros();       
           FrSkyPort::InjectUplinkFrame(0x10);      // Interleave a packet with prime = 0x10 right now!    
           return;  
        }

        if (byt == 0x7E) {
          if ((*(buf+1) == 0x0D) && (*(buf+2) == 0x30)) { // MavLite uplink (to FC) match found
            DecodeAndUplinkMavLite();        
          }      
          fr_idx = 0;
        }   
        *(buf + fr_idx) = byt;
        fr_idx++;
        prevbyt=byt;
      } 
      
    } 
      
    //===================================================================
    
    void FrSkyPort::FPort_Read_And_Write(uint8_t *buf, frport_t  frport_id) {
      /*
       Master arranges timing of transaction, slave responds
       Master sends downlink frame just after channel (control) frame
       We are a slave, and default to receiving status      
       Slave responds with uplink frame immediately if matching ID received
      */
      
      FrSkyPort::setMode(rx);
      lth=frSerial.available();  
      if (lth < 10) {
        if (lth > 0) {
          //Log.printf("lth=%d\n", lth); 
        }
        return;       // prevent 'wait-on-read' blocking
      }
          
      #if defined Report_Packetloss
        uint32_t period = (Report_Packetloss * 60000);
        if (millis() - FrSkyPort::packetloss_millis > period) {
          FrSkyPort::packetloss_millis = millis();
          float packetloss = float(float(FrSkyPort::badFrames * 100) / float(FrSkyPort::goodFrames + FrSkyPort::badFrames));
          Log.printf("goodFrames:%d   badFrames:%d   frame loss:%.3f%%\n", FrSkyPort::goodFrames, FrSkyPort::badFrames, packetloss);
        }
      #endif
  
      #if defined Debug_FrPort_Serial_Loop
        while(true) {            // loop here forever - debugging only
          FrSkyPort::chr = FrSkyPort::ReadByte();  
        }
      #endif  

      if (frport_id == f_port1) {           // find start of frame 
        
        while (!(FrSkyPort::chr==0x7E))   { // find first 0x7E, should be start, but could be previous stop
          FrSkyPort::chr = FrSkyPort::ReadByte();
          if (fpChange) {
            fpChange = false;
            return;
          }
        }
        FrSkyPort::fpbuf[0] = FrSkyPort::chr;
        FrSkyPort::chr = FrSkyPort::ReadByte();    // could be start 0x7E or len 0x08, 0x0D, 0x18, 0x20, 0x23  
        while ( FrSkyPort::chr == 0x7E)  {     // if start 0x7E, then the first one was a stop so ignore it
          if (fpChange) {
            fpChange = false;
            return;
          }            
          *buf = FrSkyPort::chr;
          FrSkyPort::chr = FrSkyPort::ReadByte(); 
        }
        fr_lth = *(buf+1) = FrSkyPort::chr;                  // lth
        fr_type = *(buf+2) = FrSkyPort::ReadByte();          // frame_type  
        if (fpChange) {
          fpChange = false;
          return;
        }        
        if ((fr_lth == 0x08) || (fr_lth == 0x19) ) {  // downlink or control frame
          FrSkyPort::frGood = true;            
          FrSkyPort::frGood_millis = millis(); 
          inject_delay = micros();   
  
          switch(fr_type){
            case 0x00:      // F.Port v1.0 Control Frame (RC Channels)
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth);
              if (FrSkyPort::parseGood) {
                #if defined Derive_PWM           
                  FrSkyPort::pwmGood = FrSkyPort::BytesToPWM(buf+3, &FrSkyPort::pwm_ch[0], fr_lth);
                  if (FrSkyPort::pwmGood) {
                    FrSkyPort::pwmGood_millis = millis();
                  }
                #endif  
                #if defined Support_SBUS_Out 
                  WriteSBUS(buf+3);
                #endif    
              }  
              break;
  
            case 0x01:      // F.Port v1.0 downlink frame from master  -  match on our sensor byte ( range 0x0~0x1B or 0x1E (FC) )              
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth); 
              if (FrSkyPort::parseGood) {    
                fr_prime = buf[3];                         // if prime == 0x00, reply prime = 0x00
                                                           // if prime == 0x10, reply prime = 0x10 (slave ready)                                                       
                FrSkyPort::InjectUplinkFrame(fr_prime);    // Inject our uplink slave packet right now! 
              }
              break;
              
            case 0x0D:   // mavlite downlink frame from master, match on sensor id 0x0D            
              inject_delay = micros();                     // start FP2 inject delay timer                 
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth); 
              if (FrSkyPort::parseGood) {
                fr_prime = buf[3];                         // should be 0x30                                                                                                              
                FrSkyPort::InjectUplinkFrame(fr_prime);    // Inject our uplink slave packet right now! (<=3mS) 
              }
              break;  
                    
            default: 
            //   Log.printf("Unknown frame type = %X\n", fr_type);  
              break;       
          }       // end of switch   
        } else {  // end of length ok
          //Log.printf("Bad FPort1 frame length = %X\n", fr_lth);  
        }     
      }          // end of FPort1
      
      if (frport_id == f_port2) {                // find start of frame
        bool ctl = false;
        bool ota = false; 
        bool dlink = false;
        while ( (!(ctl)) && (!(ota)) && (!(dlink)) ) {  // find valid lth + type combo
          prev_chr = chr;
          FrSkyPort::chr = FrSkyPort::ReadByte();
          if (fpChange) {
            fpChange = false;
            return;
          }                             
          //Printbyte(chr, false, '<'); 
          ctl = (((prev_chr == 0x0D) || (prev_chr == 0x18)|| (prev_chr == 0x23)) && (chr == 0xFF)); 
          ota = (((prev_chr == 0x0D) || (prev_chr == 0x18)|| (prev_chr == 0x23)) && (chr == 0xF1)); 
          dlink = ((prev_chr == 0x08)  && (chr == 0x1B));     // note: we ignore other PHYIDs    
        }           
        *(buf) = 0;                     // not used for fp2
        fr_lth = *(buf+1) = prev_chr;   // lth
        fr_type = *(buf+2) = chr;       // frame_type 
          
        if ((fr_lth == 0x08) || (fr_lth == 0x0d) || (fr_lth == 0x18) || (fr_lth == 0x20) ) {  // 
          FrSkyPort::frGood = true;            
          FrSkyPort::frGood_millis = millis();  
          switch(fr_type){           
            case 0x0D:   // mavlite downlink frame from master, match on sensor id 0x0D            
              inject_delay = micros();                     // start FP2 inject delay timer                 
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth); 
              if (FrSkyPort::parseGood) {
                fr_prime = buf[3];                         // should be 0x30                                                                                                              
                FrSkyPort::InjectUplinkFrame(fr_prime);    // Inject our uplink slave packet right now! (<=3mS) 
              }
              break;
                    
            case 0x1B:   // F.Port v2.3.7 downlink frame from master, match on sensor id 0x0~0x1B or 0x1E (FC)              
              inject_delay = micros();                     // start FP2 inject delay timer                             
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth); // contrary to docs, includes PHYID in lth!!        
              if (FrSkyPort::parseGood) {
                fr_prime = buf[3];                         // if prime == 0x00, reply prime = 0x00
                                                           // if prime == 0x10, reply prime = 0x10 (slave ready)                                                                                                                 
                FrSkyPort::InjectUplinkFrame(fr_prime);    // Inject our uplink slave packet right now! (<=3mS) 
              }
              break;

            case 0xff:      // F.Port v2.3.7 Control Frame (RC Channels)         
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth+1);                        
              if (FrSkyPort::parseGood) {
                #if defined Derive_PWM           
                  FrSkyPort::pwmGood = FrSkyPort::BytesToPWM(buf+3, &FrSkyPort::pwm_ch[0], fr_lth);
                  if (FrSkyPort::pwmGood) {
                    FrSkyPort::pwmGood_millis = millis();
                  }
                #endif  
                #if defined Support_SBUS_Out 
                  WriteSBUS(buf+3);
                #endif        
              }
 
              break;

            case 0xf0:      // OTA start frame
              break;
            case 0xf1:      // OTA data frame
              break;   
            case 0xf2:      // OTA end frame
              break;       
            default: 
              Log.printf("Unknown frame type = %X\n", fr_type);  
              break;           
          }       // end of switch   
        } else {  // end of length ok
          badFrames++; // due to failed length test
          Log.printf("Bad FPort2 frame length = %X\n", fr_lth);  
        }          
      }          // end of FPort2

      // No start/stop 

    }

    
    //===================================================================   
 
    bool FrSkyPort::ParseFrame(uint8_t *buf, uint16_t lth) {
      //Log.printf("buf[0]:%X  buf[1]:%X  buf[2]:%X\n", buf[0], buf[1], buf[2]);
      uint8_t i = 0;    
      for (i = 3 ; i < lth+2 ; i++) {         // payload after start byte[0], lth[1] and type[2], 
        chr = FrSkyPort::SafeRead();          // f_port2 ignores start byte[0]
      // Log.printf("i:%d  chr:%X\n", i, chr);
        *(buf+i) = chr;
      }
      
       chr = FrSkyPort::SafeRead();           // this is the crc byte
       *(buf+i) = chr;

       // FrSkyPort::crcEnd(&crcin);        
       
      #if (defined Debug_FPort_Buffer) 
        FrSkyPort::PrintBuffer(buf, lth+3);  // include the crc
      #endif 

      bool mycrcGood =  0; 
      if (frport_id == f_port1) {
        mycrcGood =  FrSkyPort::crcGood(buf+1, lth+1); 
      }
      if (frport_id == f_port2) {
        mycrcGood =  FrSkyPort::crcGood(buf+2, lth);        
      }
      
      if (mycrcGood) {
        FrSkyPort::goodFrames++;
        //Log.print("Good CRC: ");  FrSkyPort::PrintBuffer(buf, lth+3);  // include the crc        
      } else {
        badFrames++; // due to crc
        //Log.print("Bad CRC: ");  FrSkyPort::PrintBuffer(buf, lth+3);  // include the crc
      }
      #if defined Debug_CRC
        Log.printf("mycrcGood=%d\n\n", mycrcGood);  
      #endif  
      return mycrcGood;   
   
    }     
 
    //===================================================================
      
    void FrSkyPort::setMode(PortMode mode) {   
    
    #if (defined TEENSY3X) 
      if(mode == tx && modeNow !=tx) {
        *uartC3 |= 0x20;                 // Switch FrPort into send mode
        modeNow=mode;
        #if defined Debug_FrPort_Switching
          Log.print("tx");
        #endif
      }
      else if(mode == rx && modeNow != rx) {   
        *uartC3 ^= 0x20;                 // Switch FrPort into receive mode
        modeNow=mode;
        #if defined Debug_FrPort_Switching
          Log.print("rx");
        #endif
      }
    #endif

/*
    #if (defined ESP8266) || (defined ESP32) 
        if(mode == tx && modeNow !=tx) { 
          modeNow=mode;
          #if (defined ESP_Onewire) && (defined ESP32_Frs_SoftwareSerial)        
          frSerial.enableTx(true);  // Switch F.Port into send mode
          #endif
          #if defined Debug_FrPort_Switching
            Log.print("tx");
          #endif
        }   else 
        if(mode == rx && modeNow != rx) {   
          modeNow=mode; 
          #if (defined ESP_Onewire) && (defined ESP32_Frs_SoftwareSerial)                  
          frSerial.enableTx(false);  // disable interrupts on tx pin     
          #endif
          #if defined Debug_FrPort_Switching
            Log.print("rx");
          #endif
        } 
    #endif
*/
    
    }  // end of member function    
    
    //===================================================================

    byte FrSkyPort::ReadByte() {
    byte b;
      FrSkyPort::setMode(rx);
      if (lth == 0) {
        while (lth==0) {
          FrSkyPort::CheckForTimeouts();
          lth=frSerial.available();
        }
     //    Log.printf("\nlen=%3d\n",len); 
      } 
      
      // Data is available
      FrSkyPort::serGood = true;            // We have a good serial connection!
      FrSkyPort::serGood_millis = millis();
      
      b = frSerial.read();
      lth--;
      
      #if (defined Debug_FrPort_Stream)  
        Printbyte(b, false, '<');
      #endif 

      #if defined OnTheFly_FrPort_Change_Allowed  // like change from Fport1 to Fport 2
        if ( (set.frport == f_auto) && (frport_id != f_none) ){   
          frport_t frport_id_now = getID_FrPort(b); 
          if ( (frport_id_now != f_none) && (frport_id != f_none) && (frport_id != frport_id_now) ){
            String s1 = frportName(frport_id);
            String s2 = frportName(frport_id_now);        
            Log.printf("FrPort type changed from %s to %s\n", s1, s2);
            snprintf(snprintf_buf, snp_max, "Changed to %s", s2);
            LogScreenPrintln(snprintf_buf);
            
            //frport_id = frport_id_now;                                  
            if (frport_id_now == f_port1) {   //  workaround
              frport_id = (frport_t)(1);           
            } else {
              frport_id = (frport_t)(2);               
            }
            fpChange = true;       
          }
        }
      #endif  
      
      yield();
      //delay(0); // yield to rtos for wifi & bt to get a sniff      
      return b;
    }

    //===================================================================

    byte FrSkyPort::SafeRead() {
      byte b;  

      FrSkyPort::setMode(rx);

      b = FrSkyPort::ReadByte();     
      
      //  if 0x7D is received it should be omitted, and the next byte should 
      //  be XOR or ADD with 0x20
      //  0x5D => 0x7D, 0x5E => 0x7E
    
      if (b == 0x7D) {
        b = FrSkyPort::ReadByte();
        b ^= 0x20;
      }
      #if (defined Debug_FrPort_Safe_Read)  
        Printbyte(b, false, '<');
      #endif 
      delay(0); // yield to rtos for wifi & bt to get a sniff 
      return b;
    } // end of member function
    //===================================================================

    void FrSkyPort::SafeWrite(byte b, bool isPayload) {
    #if (not defined inhibit_FPort)
      FrSkyPort::setMode(tx);
        
      //  B Y T E   S T U F F   S.Port and F.Port1, not F.Port2
      //  Byte in frame has value 0x7E is changed into 2 bytes: 0x7D 0x5E
      //  Byte in frame has value 0x7D is changed into 2 bytes: 0x7D 0x5D
     
      if ( (isPayload) && ( (frport_id == s_port) || (frport_id == f_port1) ) )  {
        if (b == 0x7E) {
          FrSkyPort::WriteByte(0x7D);           
          FrSkyPort::WriteByte(0x5E);    
        } else if (b == 0x7D) {
          FrSkyPort::WriteByte(0x7D);                   
          FrSkyPort::WriteByte(0x5D);          
        } else {   
        FrSkyPort::WriteByte(b);       
        }
      } else {
        FrSkyPort::WriteByte(b);         
      }
     if (isPayload) {  // Add crcout
       FrSkyPort::crcStepOut(b);
     }
      delay(0); // yield to rtos for wifi & bt to get a sniff    
    #endif      
    }
    //===========================
    void FrSkyPort::WriteByte(byte b) {

      if (set.fr_io & 0x01) {          // Serial
        frSerial.write(b); 
      }

      #if (defined wifiBuiltin)
        if (wifiSuGood) { 
          if (set.fr_io & 0x02) {       // UDP
            frs_udp_object.write(b);                                 
          }
        }
      #endif 
      
      #if (defined sdBuiltin)   
        if (set.fr_io & 0x04) {          // SD Card
          SP_Byte_To_SD(b);         
        }
      #endif
        
      #if (defined Debug_FrPort_Stream) || (defined Debug_FrPort_Stream_Out) 
      if (b == 0x7E) {
        Log.println();
      }
        Printbyte(b, false, '>');
      #endif         
    }
    //===================================================================

    void FrSkyPort::crcStepIn(uint8_t b) {
       crcin += b;          // add in new byte
       crcin += crcin >> 8;   // add in high byte overflow if any
       crcin &= 0xff;  // mask all but low byte, constrain to 8 bits
       #if defined Debug_CRC       
         Log.printf("AddIn %3d %2X\tcrcin_now=%3d %2X\n", b, b, crcin, crcin);
       #endif  
    }  
    //===================================================================

    void FrSkyPort::crcStepOut(uint8_t b) {
       crcout += b;          // add in new byte
       crcout += crcout >> 8;   // add in high byte overflow if any
       crcout &= 0xff;  // mask all but low byte, constrain to 8 bits
       #if defined Debug_CRC       
         Log.printf("AddIn %3d %2X\tcrcout_now=%3d %2X\n", b, b, crcout, crcout);
       #endif  
    }   
    
    //=======================================================================  
    
    void FrSkyPort::crcStep(int16_t *mycrc, uint8_t b) {
       *mycrc += b;             // add in new byte
       *mycrc += *mycrc >> 8;   // add in high byte carry if any
       *mycrc &= 0xff;          // mask all but low byte, constrain to 8 bits

      #if defined Debug_CRC
         Log.printf("CRC Step: b=%3X %3d\  crc=%3X %3d\n", b, b, *mycrc, *mycrc);
       #endif
    }
    //=========================================================================
    
    void FrSkyPort::crcEnd(int16_t *mycrc)  {
      *mycrc = 0xFF - *mycrc;                  // final 2s complement
      #if defined Debug_CRC
        Log.printf("crcEnd=%3X %3d\n", *mycrc, *mycrc );
      #endif  
    } 
    //=======================================================================   
       
    uint8_t FrSkyPort::crcGet(uint8_t *buf, uint8_t lth)  {

      int16_t mycrc = 0;
      for (int i = 0; i < lth; i++) {
        crcStep(&mycrc, *buf++);
      }
      FrSkyPort::crcEnd(&mycrc);
      return mycrc;
    }
    //=======================================================================  
    bool FrSkyPort::crcGood(uint8_t *buf, uint8_t lth)  {
      
      uint8_t mycrc = FrSkyPort::crcGet(buf, lth);   
      uint8_t fpcrc = *(buf+lth);
      #if defined Debug_CRC    
        Log.printf("mycrc=%3X %3d  fpcrc=%3X\ %3d\n", mycrc, mycrc, fpcrc, fpcrc );
      #endif
    return (mycrc == fpcrc);

   }  
     //=======================================================================    
         
    bool FrSkyPort::BytesToPWM(uint8_t *buf, int16_t *ch, uint8_t lth) {  // PWM Channels

      //uint8_t fpcrc = *(buf+lth);  // may be useful
     uint8_t num_of_channels = 0;
     if (lth == 0x0D) {
       num_of_channels = 8;
     } else
     if ((lth == 0x18) || (lth == 0x19)){
       num_of_channels = 16;
     } else     
     if (lth == 0x23) {
       num_of_channels = 24;
     } 
     Log.printf("lth=%d  num_of_channels=%d\n", lth, num_of_channels);
     *ch  = ((*buf|*(buf+1)<< 8) & 0x07FF);
     *(ch+1)  = ((*(buf+1)>>3|*(buf+2)<<5) & 0x07FF);
     *(ch+2)  = ((*(buf+2)>>6|*(buf+3)<<2|*(buf+4)<<10) & 0x07FF);
     *(ch+3)  = ((*(buf+4)>>1|*(buf+5)<<7) & 0x07FF);
     *(ch+4)  = ((*(buf+5)>>4|*(buf+6)<<4) & 0x07FF);
     *(ch+5)  = ((*(buf+6)>>7|*(buf+7)<<1|*(buf+8)<<9) & 0x07FF);
     *(ch+6)  = ((*(buf+8)>>2|*(buf+9)<<6) & 0x07FF);
     *(ch+7)  = ((*(buf+9)>>5|*(buf+10)<<3) & 0x07FF); 
     
     if ((num_of_channels == 16) || (num_of_channels == 24))  {
       *(ch+8)  = ((*(buf+11)|*(buf+12)<< 8) & 0x07FF);
       *(ch+9)  = ((*(buf+12)>>3|*(buf+13)<<5) & 0x07FF);
       *(ch+10) = ((*(buf+13)>>6|*(buf+14)<<2|*(buf+15)<<10) & 0x07FF);
       *(ch+11) = ((*(buf+15)>>1|*(buf+16)<<7) & 0x07FF);
       *(ch+12) = ((*(buf+16)>>4|*(buf+17)<<4) & 0x07FF);
       *(ch+13) = ((*(buf+17)>>7|*(buf+18)<<1|*(buf+19)<<9) & 0x07FF);
       *(ch+14) = ((*(buf+19)>>2|*(buf+20)<<6) & 0x07FF);
       *(ch+15) = ((*(buf+20)>>5|*(buf+21)<<3) & 0x07FF);
     }  

     if (num_of_channels == 24)  {
       *(ch+16)  = ((*(buf+22)|*(buf+23)<< 8) & 0x07FF);
       *(ch+17)  = ((*(buf+23)>>3|*(buf+24)<<5) & 0x07FF);
       *(ch+18) = ((*(buf+24)>>6|*(buf+25)<<2|*(buf+26)<<10) & 0x07FF);
       *(ch+19) = ((*(buf+26)>>1|*(buf+27)<<7) & 0x07FF);
       *(ch+20) = ((*(buf+27)>>4|*(buf+28)<<4) & 0x07FF);
       *(ch+21) = ((*(buf+28)>>7|*(buf+29)<<1|*(buf+30)<<9) & 0x07FF);
       *(ch+22) = ((*(buf+30)>>2|*(buf+31)<<6) & 0x07FF);
       *(ch+23) = ((*(buf+31)>>5|*(buf+32)<<3) & 0x07FF);
     } 

     // remap values to regular uS range
     for (int i = 0 ; i < num_of_channels ; i++) {
       if (*(ch+i) > 0) {
         *(ch+i) = map(*(ch+i), 172, 1811, 885, 2115);      // new F.Port uS limits
       }
     }

      // flags digi ch 1
      if (*(buf+22) & (1<<0)) {
       ch[16] = 1;
      }
      else{
       ch[16] = 0;
      }
      // flags digi ch 2
      if (*(buf+22) & (1<<1)) {
       ch[17] = 1;
      }
      else{
       ch[17] = 0;
     }

     FrSkyPort::pwm_rssi = *(buf+23);

     #if defined Debug_PWM_Channels 
        FrSkyPort::Print_PWM_Channels(&FrSkyPort::pwm_ch[0], num_of_channels);
     #endif   

     if (*ch > 0) {
      return true;
     } else {
      return false;
     }
    }  
     
    //=================================================================================================
    //====================================   U P L I N K      =========================================  


    void FrSkyPort::InjectUplinkFrame(uint8_t _prime) {  
      
      uint16_t nxt = PopNextFrame();
      
      if (nxt != 0xffff) {   // not empty
        FrSkyPort::Send_Frsky_Frame(nxt, _prime);  
      }
     }
    
    //===================================================================  
 
    uint16_t FrSkyPort::PopNextFrame() {

      uint32_t sb_now = millis();    
      int16_t sb_age;
      int16_t sb_tier_age;
      int16_t sb_oldest_tier1 = 0; 
      int16_t sb_oldest_tier2 = 0; 
      int16_t sb_oldest       = 0;     
      uint16_t idx_tier1      = 0;                 // row with oldest sensor data
      uint16_t idx_tier2      = 0; 
      uint16_t idx            = 0; 

      // 2 tier scheduling. Tier 1 gets priority, tier2 (0x5000) only sent when tier 1 empty 
  
      // find the row with oldest sensor data = idx 
      sb_unsent = 0;  // how many rows in-use

      uint16_t i = 0;
      while (i < sb_rows) {  
    
        if (sb[i].inuse) {
          sb_unsent++;   
      
          sb_age = (sb_now - sb[i].millis); 
          sb_tier_age = sb_age - sb[i].sub_id;  

          if (sb[i].msg_id == 0x5000) {
            if (sb_tier_age >= sb_oldest_tier2) {
      //        if ( (sb_tier_age) && ((millis() - prev_5000_millis) < 24) ) return 0xffff;  // don't send me yet. leave enough time between chunks               
              sb_oldest_tier2 = sb_tier_age;
              idx_tier2 = i;
            }
          } else {
          if (sb_tier_age >= sb_oldest_tier1) {
            sb_oldest_tier1 = sb_tier_age;
            idx_tier1 = i;
            }   
          }
        } 
      i++;    
      } 
    
      if (sb_oldest_tier1 == 0) {            // if there are no tier 1 sensor entries
        if (sb_oldest_tier2 > 0) {           // but there are tier 2 entries
          idx = idx_tier2;                   // send tier 2 instead
          sb_oldest = sb_oldest_tier2;
        }
      } else {
        idx = idx_tier1;                    // if there are tier1 entries send them
       sb_oldest = sb_oldest_tier1;
      }
  
      //Log.println(sb_unsent);           // limited detriment :)  

      if (sb_oldest == 0)  return 0xffff;  // flag the scheduler table as empty
      
      //prev_5000_millis = millis();         // when the most recent 0x5000 frame was sent
      
      if ((FrSkyPort::msg_class_now(sb[idx].msg_id)) == passthru) {
          pt_payload = sb[idx].payload.passthru; 
      } else
      if ((FrSkyPort::msg_class_now(sb[idx].msg_id)) == mavlite) {
          mtf_payload = sb[idx].payload.mavlite; 
      }

      #if (defined Frs_Debug_Scheduler) || (defined MavLite_Debug_Scheduler)
        uint16_t msgid_filter;
        #if (defined MavLite_Debug_Scheduler)
          msgid_filter = 0x100;
        #else
          msgid_filter = 0xffff; // just high value
        #endif

        if (sb[idx].msg_id < msgid_filter) {
          Log.print(sb_unsent); 
          Log.printf("\tPop  row= %3d", idx );
          Log.print("  msg_id=0x");  Log.print(sb[idx].msg_id, HEX);
          if (sb[idx].msg_id < 0x1000) Log.print(" ");
          Log.printf("  sub_id= %2d", sb[idx].sub_id); 
      
          pb_rx=false;
          FrSkyPort::PrintPayload(FrSkyPort::msg_class_now(sb[idx].msg_id), downlink);
          Log.printf("  age=%3d mS \n" , sb_oldest_tier1 );        
        }
      #endif

      return idx;  // return the index of the oldest frame
     }
    //===================================================================   

    msg_class_t  FrSkyPort::msg_class_now(uint8_t msg_id) {
      if ((msg_id >= 20) && (msg_id <=77)) {
        return mavlite; 
      } else {
        return passthru;
      }
     }
    //===================================================================    
    void FrSkyPort::Send_Frsky_Frame(uint16_t idx, uint8_t _prime) {
      FrSkyPort::setMode(tx); 
      #if defined Frs_Debug_Period
        PrintFrPeriod(0);   
      #endif    

      #if (defined wifiBuiltin)
        if  (set.fr_io & 0x02) {   // FrSky UDP out - start packet
          if (wifiSuGood) {                  
            //UDP_remoteIP should already be set for broadcast
            frs_udp_object.beginPacket(UDP_remoteIP, set.udp_remotePort+1);  // use remote port + 1 for Frs out
          }
        }
      #endif   

      sp_msg_id = sb[idx].msg_id;   // global msg_id for debug of sending
      msg_class = FrSkyPort::msg_class_now(sp_msg_id);
      
      #ifdef Debug_SITL_Input      
        if (sp_msg_id == 0x5004) {
          Log.print("\nXXXXXXXXXXXXXXXXXXXXXX ");
        }
        Log.printf("POP AND SEND 0x%X idx=%u\n", sp_msg_id, idx);
      #endif  
      
      //========================    RSSI special treatment
      if (sp_msg_id == 0xF101) {                   
        #if (defined Frs_Debug_Rssi)
          PrintFrPeriod(0);    
          Log.println(" 0xF101 sent");
        #endif   
        if (set.trmode != relay) { 
          if ( (frport_id == s_port) || (frport_id == f_port1) )  {
            FrSkyPort::SafeWrite(0x7E, false);      // don't stuff or add to crcout         
          }
   
          if (frport_id == f_port1) {   
            FrSkyPort::SafeWrite(0x08, true);     // lth - add crc                 
            FrSkyPort::SafeWrite(0x81, true);     // FP1 frame type    
          } 
          if (frport_id == f_port2) {
            FrSkyPort::SafeWrite(0x08, false);    // lth - no crc add            
            FrSkyPort::SafeWrite(0x1B, true);     // FP2 echo the sensor id we matched with                    
          }                                   
          if (frport_id == s_port) {
            FrSkyPort::SafeWrite(0x1B, false);    // our sport diy prime sensor byte       
          }   
        }
      }
      //^^^^^^^^^^^^^^^^^^^^^^   RSSI special treatment
      
      #if defined Frs_Debug_Scheduler
        Log.printf("Injecting frame idx=%d msg_id=0x%X  ", idx, sp_msg_id);
        FrSkyPort::PrintPayload(FrSkyPort::msg_class_now(sp_msg_id), downlink); 
        Log.println();
      #elif defined Debug_Mavlite_Scheduler
      if (sp_msg_id < 0x100) {  
        Log.printf("Injecting frame idx=%d msg_id=0x%X  ", idx, sp_msg_id);
        FrSkyPort::PrintPayload(FrSkyPort::msg_class_now(sp_msg_id), downlink); 
        Log.println();
      }  
      #endif

      uint8_t *bytes;
      
      #if defined Debug_Inject_Delay
        Log.printf("Inject_delay %duS\n", (micros() - inject_delay));
      #endif

      if (set.trmode == ground) {             // In ground mode we are master and send a diy poll
        if (frport_id == s_port) {   
            FrSkyPort::SafeWrite(0x7E, false);  // START/STOP
            FrSkyPort::SafeWrite(0x1B, false);  // our diy sensor id               
        }  
        if (frport_id == f_port1) {   // FP1 and 2 in ground mode are conjecture right now until I can get a upgraded transmitter to test with
          FrSkyPort::SafeWrite(0x7E, false);    // START 
          FrSkyPort::SafeWrite(0x08, true);     // lth - add crc                 
          FrSkyPort::SafeWrite(0x01, true);     // download frame type    
        } 
        if (frport_id == f_port2) {
          FrSkyPort::SafeWrite(0x08, false);    // lth - no crc add            
          FrSkyPort::SafeWrite(0x1B, true);     // FP2 echo the sensor id we matched with                    
        }                    
      }  else
      if ( (set.trmode == air) || (set.trmode == relay) ) {  // In relay mode we are a slave, and only respond   
        if (frport_id == f_port1) {   
          FrSkyPort::SafeWrite(0x08, true);     // lth - add crc                 
          FrSkyPort::SafeWrite(0x81, true);     // FP1 frame type    
        } 
        if (frport_id == f_port2) {
          FrSkyPort::SafeWrite(0x08, false);    // lth - no crc add            
          FrSkyPort::SafeWrite(0x1B, true);     // FP2 echo the sensor id we matched with                    
        }                                   
      }   
      
      if (msg_class == passthru) {    
                                          
        FrSkyPort::SafeWrite(_prime, true );    // echo prime received.  should be 0x00 or 0x10            
        bytes = (uint8_t*)&sp_msg_id;           // cast msg_id to bytes and send it
        FrSkyPort::SafeWrite(bytes[0], true);
        FrSkyPort::SafeWrite(bytes[1], true);

        bytes = (uint8_t*)&pt_payload;          //  cast payload to bytes      
      } 
      
      if (msg_class == mavlite) {                                                 
        FrSkyPort::SafeWrite(_prime, true );   // echo prime received.  should be 0x32.  
        bytes = (uint8_t*)&mtf_payload;        // cast payload to bytes                  
      }

    
      FrSkyPort::SafeWrite(bytes[0], true);
      FrSkyPort::SafeWrite(bytes[1], true);
      FrSkyPort::SafeWrite(bytes[2], true);
      FrSkyPort::SafeWrite(bytes[3], true);
      
      if (msg_class == mavlite) {
        FrSkyPort::SafeWrite(bytes[4], true); 
        FrSkyPort::SafeWrite(bytes[5], true);
        
        #if defined Debug_Mavlite
          Log.print("POP: Send Mavlite Frame : ");
          FrSkyPort::PrintPayload(msg_class, downlink);  
          Log.printf("  CRC=%2X\n", (0xFF-crcout));       
        #endif     
      } 

      FrSkyPort::WriteCrc();      // clears crcout

      if (frport_id == f_port1) {    
        FrSkyPort::SafeWrite(0x7E, false);        // STOP_byte              
      }

      // clear the appropriate payload field 
      if (msg_class == passthru) {
        pt_payload = 0;                                             
      } else
      if (msg_class == mavlite) {
        mtf_payload = {};                                     // clear mavlite payload struct
      }
  
      sb[idx].inuse = 0;                                      // 0=free to use, 1=occupied - for passthru and mavlite

      #if (defined wifiBuiltin)
        if (set.fr_io & 0x02)  {       // FrSky send UDP packet
          if (wifiSuGood) { 
            bool endOK = frs_udp_object.endPacket();
            endOK = endOK;  // silence irritating compiler warning
            // if (!endOK) Log.printf("FrSky UDP msgSent=%d   endOK=%d\n", msgSent, endOK);
          }
        }
      #endif  

    }

    //=========================  U P L I N K   T O   F C  ========================
 
    bool FrSkyPort::DecodeAndUplinkMavLite() {
    #if defined Support_MavLite  
      mt_seq = frbuf[3];
      if (mt_seq == 0) {
        mt_paylth = frbuf[4];
        mt_msg_id = frbuf[5];
        mt_idx = 0;
        }

      if (FrSkyPort::UnchunkMavLitePayload()) {  // when fully expanded
 
        switch (mt_msg_id) {                // Decode uplink mavlite messages according to msg-id
          
          case 20:                          //  #20 or 0x14   Mavlink_Param_Request_Read - 
            {
              for (int i = 0; i <16; i++) {
                ap22_param_id[i] = mt_payload[i];
              }

              #if (defined Debug_Mavlite_SPort)
                PrintMavLiteUplink();
              #endif  

              #if defined Debug_Mavlite
                Log.printf("UPLINK: MavLite #20 Param_Request_Read :%s: ", ap22_param_id);  
                PrintMavLiteUplink();
              #endif  

              Mavlink_Param_Request_Read(-1, ap22_param_id); // Request Param Read using param_id, not index  

              // store in table of param requests awaiting reply
              mt20_row = FrSkyPort::FirstEmptyMt20Row();
              //   Log.printf("mt20_row=%d\n", mt20_row);
              if (mt20_row == 0xffff) return false;  // mt20 table overflowed, ignore this message :(
              strncpy(mt20[mt20_row].param_id, ap22_param_id, 16);
              mt20[mt20_row].millis = millis(); 
              mt20[mt20_row].inuse = 1;
              
              return true;           
            }
          case 22:                          // Echo #22 dowwmliink message - Uplink   
            {
             #if defined Debug_Mavlite
              Log.print("UPLINK: Echo of Mavlite #22 : ");
              PrintMavLiteUplink();
            #endif                      
            }
          case 23:                          // #23 or 0x17  PARAM_SET - Uplink
            {
              // expanded payload to value 
              byte *b = (byte *)&ap23_param_value;  // cast value to bytes
              b[0] = mt_payload[0];
              b[1] = mt_payload[1];
              b[2] = mt_payload[2];
              b[3] = mt_payload[3];

              for (int i = 0; i <16; i++) {
                ap23_param_id[i] = mt_payload[i+4];
              }

              Mavlink_Param_Set();
              
              #if defined Debug_Mavlite
                 Log.printf("UPLINK: MavLite #23 Param_Set. Parameter-id= %s  Value = %.5f\n", ap23_param_id, ap23_param_value);  
              #endif  
              return true; 
            }            

          case 76:                          // #76 or 0x4c  COMMAND_LONG
            {
              ap76_command = (mt_payload[1] << 8) + mt_payload[0];          
              ap76_confirm = mt_payload[2];
              uint8_t par_cnt = (mt_payload[3] & 0xE0) >> 5;
              uint8_t conf_cnt = mt_payload[3] & 7;   // unused
              float val = 0;
              byte *b = (byte *)&val;  // cast value to bytes
              for ( int i = 1; i <= par_cnt ; i++) {
                uint8_t j = i * 4;
                b[0] = mt_payload[j+0];
                b[1] = mt_payload[j+1];
                b[2] = mt_payload[j+2];
                b[3] = mt_payload[j+3];
                ap76_param[i-1] = val;
              }

              Send_Mavlink_Command_Long();  // #76

              #if defined Debug_Mavlite
               Log.printf("UPLINK: MavLite #76 Command_Long: Command=%d  confirm=%d param_count=%d ", ap76_command, ap76_confirm, par_cnt);  
               for ( int i = 0; i < par_cnt ; i++) {
                  Log.printf("param%d=%.5f", i+1,  ap76_param[i]); 
               }
               Log.println();
              #endif  

              return true;      
            }                  
          default:
            #if defined Debug_Mavlite
              Log.printf("Mavlite Uplink: Unknown Message ID #%d ignored  ", mt_msg_id);
              PrintMavLiteUplink();
            #endif
            return false;   
        } // end of switch
      }  
    #endif 
    return false;        
    }

    //===================================================================  
    #if defined Support_MavLite
    
    bool FrSkyPort::UnchunkMavLitePayload() {   
          
      if (mt_seq == 0) {            // first chunk 
        for (int i = 6 ; i <= 8 ; i++, mt_idx++) {
          #if defined Debug_Mavlite_Chunking
            Log.printf("\ti=%d  mt_idx=%d frbuf[i]=%2X \n",i, mt_idx, frbuf[i]);
          #endif  
          mt_payload[mt_idx] = frbuf[i];
          if (mt_idx >= mt_paylth) {
            return true;
          }        
        }
        return false;
      } else {           // rest of the chunks
        for (int i = 4 ; i <= 8 ; i++, mt_idx++) {
          #if defined Debug_Mavlite_Chunking
            Log.printf("\ti=%d  mt_idx=%d frbuf[i]=%2X \n",i, mt_idx, frbuf[i]);
          #endif  
          mt_payload[mt_idx] = frbuf[i];
          if (mt_idx >= mt_paylth) {
            mt_payload[mt_idx] = 0x00; // terminate string      
            return true; 
          }                
         }
       return false;
      }
     return false;
    }
    #endif

    //===================================================================   
    void FrSkyPort::WriteCrc() {
      uint8_t byte;
      byte = 0xFF-crcout;

      FrSkyPort::SafeWrite(byte, true);

      crcout = 0;          // crcout reset
    }
    //===================================================================  

    uint16_t FrSkyPort::FirstEmptyMt20Row() {
      uint16_t i = 0;
      while (mt20[i].inuse == 1) {   // find empty mt20 uplink row
      //  Log.printf("i=%d  mt20[i].inuse=%d\n", i, mt20[i].inuse);

        if (millis() - mt20[i].millis > 5000) { // expire the row if no reply from FC in 5 seconds ? bad param_id
          mt20[i].inuse = 0;
          Log.printf("param_id %s not received back from FC. Timed out.\n", mt20[i].param_id);
        }
        i++; 
        if ( i >= mt20_rows-1) {
          mt20_buf_full_gear++;
          if ( (mt20_buf_full_gear == 0) || (mt20_buf_full_gear%1000 == 0)) {
            Log.println("mt20 uplink buffer full");  // Report every so often
          }
          return 0xffff;
        }
      }
      return i;
    }    
    //=================================================================== 
    void FrSkyPort::PushMessage(uint16_t msg_id, uint8_t sub_id) {    // Downlink to GCS
      switch(msg_id) {

        // MavLite below
        case 0x16:                   // msg_id 0x16 MavLite PARAM_VALUE ( #22 )
          FrSkyPort::Push_Param_Val_016(msg_id);
          break; 
          
        case 0x4d:                   // msg_id 0x4d MavLite COMMAND_ACK ( #77 )
          FrSkyPort::Push_Command_Ack_04d(msg_id);
          break;           

        // Passthrough below
        case 0x800:                  // msg_id 0x800 Lat & Lon
          if (sub_id == 0) {
            FrSkyPort::Push_Lat_800(msg_id);
          }
          if (sub_id == 1) {
            FrSkyPort::Push_Lon_800(msg_id);
          }
          break;            
        case 0x5000:                 // msg_id 0x5000 Status Text            
            FrSkyPort::Push_Text_Chunks_5000(msg_id);
            break;
        
        case 0x5001:                // msg_id 0x5001 AP Status
          FrSkyPort::Push_AP_status_5001(msg_id);
          break; 

        case 0x5002:                // msg_id 0x5002 GPS Status
          FrSkyPort::Push_GPS_status_5002(msg_id);
          break; 
          
        case 0x5003:                //msg_id 0x5003 Batt 1
          FrSkyPort::Push_Bat1_5003(msg_id);
          break; 
                    
        case 0x5004:                // msg_id 0x5004 Home
          FrSkyPort::Push_Home_5004(msg_id);
          break; 

        case 0x5005:                // msg_id 0x5005 Velocity and yaw
          FrSkyPort::Push_VelYaw_5005(msg_id);
          break; 

        case 0x5006:                // msg_id 0x5006 Attitude and range
          FrSkyPort::Push_Atti_5006(msg_id);
          break; 
      
        case 0x5007:                // msg_id 0x5007 Parameters 
          FrSkyPort::Push_Parameters_5007(msg_id, sub_id);
          break; 
      
        case 0x5008:                // msg_id 0x5008 Batt 2
          FrSkyPort::Push_Bat2_5008(msg_id);
          break; 

        case 0x5009:                // msg_id 0x5009 Waypoints/Missions 
          FrSkyPort::Push_WayPoint_5009(msg_id);
          break;       

        case 0x500A:                // msg_id 0x500A RPM
          FrSkyPort::Push_RPM_500A(msg_id);
          break;

        case 0x500B:                // msg_id 0x500B Terrain
          FrSkyPort::Push_Terrain_500B(msg_id);
          break;

        case 0x50F1:                // msg_id 0x50F1 Servo_Raw            
          FrSkyPort::Push_Servo_Raw_50F1(msg_id);
          break;      

        case 0x50F2:                // msg_id 0x50F2 VFR HUD          
          FrSkyPort::Push_VFR_Hud_50F2(msg_id);
          break;    

        case 0x50F3:                // msg_id 0x50F3 Wind Estimate      
       //   FrSkyPort::Push_Wind_Estimate_50F3(msg_id);  // not presently implemented
          break; 
        case 0xF101:                // msg_id 0xF101 RSSI      
          FrSkyPort::Push_Rssi_F101(msg_id);      
          break;       
        default:
          Log.print("Warning, msg_id "); Log.print(msg_id, HEX); Log.println(" unknown");
          break;       
      }            
    }
    //=================================================================== 
    void FrSkyPort::PushToEmptyRow(uint16_t msg_id, uint8_t sub_id) {  // Downlink to GCS
      sb_row = 0;
      while (sb[sb_row].inuse) {   // find empty F.Port row 
        sb_row++; 
        if (sb_row >= sb_rows-1) {
          if ( (sb_buf_full_gear == 0) || (sb_buf_full_gear%40000 == 0)) {
            Log.println("FrPort scheduler buffer full. Check FrPort Downlink to Taranis/Horus");  // Report every so often
          }
          sb_buf_full_gear++;
          return;     
        }
      }
      sb_unsent++;

      // The push
      sb[sb_row].millis = millis();
      sb[sb_row].inuse = 1;                // 0=free for use, 1=occupied
      sb[sb_row].msg_id = msg_id;
      sb[sb_row].sub_id = sub_id;
  
      if(FrSkyPort::msg_class_now(msg_id) == passthru) {
          sb[sb_row].payload.passthru = pt_payload;  
      } else
      if(FrSkyPort::msg_class_now(msg_id)== mavlite) {
          sb[sb_row].payload.mavlite = mtf_payload;  
      }
      
      #if defined Debug_SITL_Input
        if (msg_id == 0x5004) {
          Log.print("ZZZZZZZZZZZZZZZZZZZ"); 
        }   
               
        Log.print(sb_unsent); 
        Log.printf("\tPush row= %3d", sb_row );
        Log.print("  msg_id=0x"); Log.print(msg_id, HEX);
        if (msg_id < 0x1000) Log.print(" ");
        Log.println(); 
      #endif           
        
      #if (defined Frs_Debug_Scheduler) || (defined MavLite_Debug_Scheduler) 
  
        uint16_t msgid_filter;  
        #if (defined MavLite_Debug_Scheduler)
          msgid_filter = 0x100;
        #else
          msgid_filter = 0xffff; // high values
        #endif
    
        if (msg_id < msgid_filter) {
          Log.print(sb_unsent); 
          Log.printf("\tPush row= %3d", sb_row );
          Log.print("  msg_id=0x"); Log.print(msg_id, HEX);
          if (msg_id < 0x1000) Log.print(" ");
          Log.printf("  sub_id= %2d", sub_id);
          pb_rx=false;
          FrSkyPort::PrintPayload(FrSkyPort::msg_class_now(msg_id), downlink);
          Log.println();      
        }
      #endif
    }
    //=================================================================== 
 
    void FrSkyPort::Push_Param_Val_016(uint16_t msg_id) {   //  0x16 MavLite PARAM_VALUE ( #22 ) Downlink to GCS
    #if defined Support_MavLite  
 
      mt_msg_id = msg_id;                                // 0x16  #22 
      mt_paylth = 4 +strlen(ap22_param_id);              // payload = value + param = 17
      memset (&mt_payload[0],0x00,32);                   // clear payload
      // value to expanded payload
      byte *b = (byte *)&ap22_param_value;               // cast value to bytes
      mt_payload[0] = b[0];
      mt_payload[1] = b[1];
      mt_payload[2] = b[2];
      mt_payload[3] = b[3];

      for (int i=0; i<16 ; i++) {
        mt_payload[i+4] = ap22_param_id[i];
      }

      mt_totlth = 2 + mt_paylth;                         // msg_id + lth + payload
    
      #if (defined Debug_Mavlite)
          Log.printf("Push: MavLiteParamValue16 msg_id=0x%X lth=%d param_value=%.3f param_id=%s \n"
            , mt_msg_id, mt_paylth, ap22_param_value, ap22_param_id );           
          Log.print("Expanded Msg=");
          for (int i = 0 ; i < mt_totlth ; i++) {
            if ( (i == 3) || ( (i > 3) && (!((i+2)%5)) ) ) Log.print("|");
            Printbyte(mt_payload[i], false, '>');
          }  
          Log.print("|\t|");
          for (int i = 0 ; i < mt_totlth ; i++) {
            if ((mt_payload[i] >31) && (mt_payload[i]<127)) Log.write(mt_payload[i]);
          }
          Log.println("|");
        #endif
      
      FrSkyPort::ChunkMavLitePayload(msg_id, mt_paylth);
      
    #endif
    }
    
    //===================================================================         

    void FrSkyPort::Push_Command_Ack_04d(uint16_t msg_id) {  //  0x4d MavLite COMMAND_ACK ( #77 ) Downlink to GCS 
    #if defined Support_MavLite  
 
      mt_msg_id = msg_id;                          // 0x4d #77
      mt_paylth = 1;                               // payload = 1 byte = result
      memset (&mt_payload[0],0x00,32);             // clear payload field
      mt_payload[0] = ap77_result;

      mt_totlth = 2 + mt_paylth;                         // msg_id + lth + payload
          
      #if (defined Debug_Mavlite)
          Log.printf("PUSH: Command_Ack_04d msg_id=0x%X lth=%d result=%d  ", mt_msg_id, mt_paylth, ap77_result);           
          Log.print("Expanded Msg=");
          for (int i = 0 ; i < mt_totlth ; i++) {
            if ( (i == 3) || ( (i > 3) && (!((i+2)%5)) ) ) Log.print("|");
            Printbyte(mt_payload[i], false, '>');
          }  

          Log.println();
        #endif
      
      FrSkyPort::ChunkMavLitePayload(msg_id, mt_paylth);
     #endif
     }
     
    //===================================================================     
    #if defined Support_MavLite  // Downlink to GCS
    
    void FrSkyPort::ChunkMavLitePayload(uint8_t msg_id, uint8_t tlth) {
      mt_chunk = 0; 
      mt_idx = 0;   

      while (mt_idx <= tlth) {    
      //  Log.printf("mt_idx=%d  tlth=%d   mt_chunk=%d\n", mt_idx, tlth, mt_chunk     ); 
        if (mt_chunk == 0) {

          //======= message to frame (chunk)
          // payload starting position  msgid, lth - then 20B payload, then crcout
    
          // first chunk 
          mtf_payload.seq = mt_chunk;      FrSkyPort::crcStepOut(mtf_payload.lth);
          mtf_payload.lth = mt_paylth;     FrSkyPort::crcStepOut(mtf_payload.lth);     // idx = 1          
          mtf_payload.msg_id = msg_id;     FrSkyPort::crcStepOut(mtf_payload.msg_id);  // idx = 0

         //===========================================

         for (int i = 3 ; i < 6 ; i++) {  
            mtf_payload.raw[i] = mt_payload[mt_idx];  
            FrSkyPort::crcStepOut(mtf_payload.raw[i]); 
            #if defined Debug_Mavlite_Chunking
              Log.printf("\ti=%d mt_idx=%d mtf_payload.raw[i]=0x%X\t|", i, mt_idx, mtf_payload.raw[i]); 
              if ((mtf_payload.raw[i] > 31) && (mtf_payload.raw[i] < 127)) 
                Log.write(mtf_payload.raw[i]);
                else Log.print(" ");
              Log.println("|");  
            #endif
        
            mt_idx++;      
          } 
      
          //_______________________________________________________
          // rest of the frames / chunks
        } else {           
          mtf_payload.seq = mt_chunk;  // changed to passthru style crc calc FrSkyPort::crcStepOut(mtf_payload.seq); 
          //_______________________________________________________
          for (int i = 1 ; i < 6 ; i++) { 
          
            if (mt_idx >= tlth) {  // break out of the for loop
              mtf_payload.raw[i] = crcout;     // no 1s complement? and append the crcout
      //       Log.printf("mt_idx=%d  mt_payload.raw[mt_idx]=0x%X\n", mt_idx, mt_payload.raw[mt_idx] );
              mt_idx++;  // gets me out of the while loop
              break; // short chunk  - break out of for loop
            }

            mtf_payload.raw[i] = mt_payload[mt_idx];
            FrSkyPort::crcStepOut(mtf_payload.raw[i]); 
            #if defined Debug_Mavlite_Chunking
              Log.printf("\ti=%d mt_idx=%d mtf_payload.raw[i]=0x%X\t|", i, mt_idx, mtf_payload.raw[i]); 
                if ((mtf_payload.raw[i] > 31) && (mtf_payload.raw[i] < 127)) 
                Log.write(mtf_payload.raw[i]);
                else Log.print(" ");
              Log.println("|");
            #endif
             
            mtf_payload.raw[i] = mt_payload[mt_idx];   
            mt_idx++;                                    
          }  // end of for
          //_________________________________________________________ 
        }   // end of else - chunks > 0

        FrSkyPort::PushToEmptyRow(msg_id, 0);  

        #if defined Frs_Debug_All || defined Debug_Mavlite
          Log.printf("PUSH: MavLite to GCS #%d 0x%x:  chunk=%d  ", msg_id, msg_id, mtf_payload.seq);
          pb_rx = false;
          FrSkyPort::PrintPayload(mavlite, downlink); 
          Log.println();
        #endif
      
        mt_chunk++;
        mtf_payload = {};  // clear payload struct
      } // end of while loop

       crcout = 0;
       printcrcout = 0;
 
    }
    #endif   
    //===================================================================   
    void FrSkyPort::Push_Lat_800(uint16_t msg_id) {       // 0x800
      pt_gps_status = ap24_fixtype < 3 ? ap24_fixtype : 3;                   //  0 - 3 
      if (pt_gps_status < 3) return;
      if ((px4_flight_stack) || (pitlab_flight_stack)) {
        pt_lat = Abs(ap24_lat) / 100 * 6;  // ap_lat * 60 / 1000
        if (ap24_lat<0) 
          ms2bits = 1;
        else ms2bits = 0;    
      } else {
        pt_lat = Abs(ap33_lat) / 100 * 6;  // ap_lat * 60 / 1000
        if (ap33_lat<0) 
          ms2bits = 1;
        else ms2bits = 0;
      }
      pt_payload = 0;
      bit32Pack(pt_lat, 0, 30);
      bit32Pack(ms2bits, 30, 2);
          
      #if defined Frs_Debug_All || defined Frs_Debug_LatLon
        PrintFrPeriod(0); 
        Log.print("Passthru out LatLon 0x800: ");
        Log.print(" ap33_lat="); Log.print((float)ap33_lat / 1E7, 7); 
        Log.print(" pt_lat="); Log.print(pt_lat);  
        Log.print(" pt_payload="); Log.print(pt_payload); Log.print(" ");
       FrSkyPort::PrintPayload(passthru, downlink);
        int32_t r_lat = (bit32Extract(pt_payload,0,30) * 100 / 6);
        Log.print(" lat unpacked="); Log.println(r_lat );    
      #endif

     FrSkyPort::PushToEmptyRow(msg_id, 0);        
    }
    //===================================================================  
    void FrSkyPort::Push_Lon_800(uint16_t msg_id) {      // 0x800
      pt_gps_status = ap24_fixtype < 3 ? ap24_fixtype : 3;                   //  0 - 3 
      if (pt_gps_status < 3) return;
      if ((px4_flight_stack) || (pitlab_flight_stack)) {
        pt_lon = Abs(ap24_lon) / 100 * 6;  // ap_lon * 60 / 1000
        if (ap24_lon<0) {
          ms2bits = 3;
        }
        else {
          ms2bits = 2;    
        }
      } else {
        pt_lon = Abs(ap33_lon) / 100 * 6;  // ap_lon * 60 / 1000
        if (ap33_lon<0) { 
          ms2bits = 3;
        }
        else {
          ms2bits = 2;
        }
      }
      pt_payload = 0;
      bit32Pack(pt_lon, 0, 30);
      bit32Pack(ms2bits, 30, 2);
          
      #if defined Frs_Debug_All || defined Frs_Debug_LatLon
       PrintFrPeriod(0); 
        Log.print("Passthru out LatLon 0x800: ");  
        Log.print(" ap33_lon="); Log.print((float)ap33_lon / 1E7, 7);     
        Log.print(" pt_lon="); Log.print(pt_lon); 
        Log.print(" pt_payload="); Log.print(pt_payload); Log.print(" ");
        FrSkyPort::PrintPayload(passthru, downlink);
        int32_t r_lon = (bit32Extract(pt_payload,0,30) * 100 / 6);
        Log.print(" lon unpacked="); Log.println(r_lon );  
      #endif

       FrSkyPort::PushToEmptyRow(msg_id, 1); 
    }
    //===================================================================  
    void FrSkyPort::Push_Text_Chunks_5000(uint16_t msg_id) {

      // status text  char[50] no null,  ap-text char[60]

      for (int i=0; i<50 ; i++) {       // Get text len
        if (ap_text[i]==0) {            // end of text
          len=i;
          break;
        }
      }
  
      ap_text[len+1]=0x00;
      ap_text[len+2]=0x00;  // mark the end of text chunk +
      ap_text[len+3]=0x00;
      ap_text[len+4]=0x00;
          
      ap_txtlth = len;
  
      // look for simple-mode status change messages       
      if (strcmp (ap_text,"SIMPLE mode on") == 0)
        ap_simple = true;
      else if (strcmp (ap_text,"SIMPLE mode off") == 0)
        ap_simple = false;

      pt_severity = ap_severity;
      pt_txtlth = ap_txtlth;
      memcpy(pt_text, ap_text, pt_txtlth+4);   // plus rest of last chunk at least
      pt_simple = ap_simple;

      #if defined Frs_Debug_All || defined Frs_Debug_StatusText
        PrintFrPeriod(0); 
        Log.print("Passthru out AP_Text 0x5000: ");  
        Log.print(" pt_severity="); Log.print(pt_severity);
        Log.print(" "); Log.print(MavSeverity(pt_severity)); 
        Log.print(" Text= ");  Log.print(" |"); Log.print(pt_text); Log.println("| ");
      #endif

      pt_chunk_idx = 0;

      while (pt_chunk_idx <= (pt_txtlth)) {                 // send multiple 4 byte (32b) chunks
    
        pt_chunk_num = (pt_chunk_idx / 4) + 1;
    
        pt_chunk[0] = pt_text[pt_chunk_idx];
        pt_chunk[1] = pt_text[pt_chunk_idx+1];
        pt_chunk[2] = pt_text[pt_chunk_idx+2];
        pt_chunk[3] = pt_text[pt_chunk_idx+3];
    
        pt_payload = 0;
        bit32Pack(pt_chunk[0], 24, 7);
        bit32Pack(pt_chunk[1], 16, 7);
        bit32Pack(pt_chunk[2], 8, 7);    
        bit32Pack(pt_chunk[3], 0, 7);  
    
        #if defined Frs_Debug_All || defined Frs_Debug_StatusText
          PrintFrPeriod(0); 
          Log.print(" pt_chunk_num="); Log.print(pt_chunk_num); 
          Log.print(" pt_txtlth="); Log.print(pt_txtlth); 
          Log.print(" pt_chunk_idx="); Log.print(pt_chunk_idx); 
          Log.print(" "); 
          strncpy(pt_chunk_print,pt_chunk, 4);
          pt_chunk_print[4] = 0x00;
          Log.print(" |"); Log.print(pt_chunk_print); Log.print("| ");
          Log.print(" pt_payload="); Log.print(pt_payload); Log.print(" ");
          FrSkyPort::PrintPayload(passthru, downlink);
          Log.println();
        #endif  

        if (pt_chunk_idx+4 > (pt_txtlth)) {

          bit32Pack((pt_severity & 0x1), 7, 1);            // ls bit of severity
          bit32Pack(((pt_severity & 0x2) >> 1), 15, 1);    // mid bit of severity
          bit32Pack(((pt_severity & 0x4) >> 2) , 23, 1);   // ms bit of severity                
          bit32Pack(0, 31, 1);     // filler
      
          #if defined Frs_Debug_All || defined Frs_Debug_StatusText
            PrintFrPeriod(0); 
            Log.print(" pt_chunk_num="); Log.print(pt_chunk_num); 
            Log.print(" pt_severity="); Log.print(pt_severity);
            Log.print(" "); Log.print(MavSeverity(pt_severity)); 
            bool lsb = (pt_severity & 0x1);
            bool sb = (pt_severity & 0x2) >> 1;
            bool msb = (pt_severity & 0x4) >> 2;
            Log.print(" ls bit="); Log.print(lsb); 
            Log.print(" mid bit="); Log.print(sb); 
            Log.print(" ms bit="); Log.print(msb); 
            Log.print(" pt_payload="); Log.print(pt_payload); Log.print(" ");
            FrSkyPort::PrintPayload(passthru, downlink);
            Log.println(); Log.println();
         #endif 
         }

       FrSkyPort::PushToEmptyRow(msg_id, pt_chunk_num); 

       #if defined Send_status_Text_3_Times 
        FrSkyPort::PushToEmptyRow(msg_id, pt_chunk_num); 
        FrSkyPort::PushToEmptyRow(msg_id, pt_chunk_num); 
       #endif 
    
       pt_chunk_idx +=4;
     }
  
      pt_chunk_idx = 0;
   
    }

    //===================================================================   
    void FrSkyPort::Push_AP_status_5001(uint16_t msg_id) {
      if (ap_type == 6) return;      // If GCS heartbeat ignore it  -  yaapu  - ejs also handled at #0 read
      pt_payload = 0;
     // pt_simple = ap_simple;         // Derived from "ALR SIMPLE mode on/off" text messages
      pt_simple = 0;                   // stops repeated 'simple mode enabled' and flight mode messages
      pt_armed = ap_base_mode >> 7;  
      pt_land_complete = pt_armed;
  
      if (px4_flight_stack) {
        pt_flight_mode = PX4FlightModeNum(px4_main_mode, px4_sub_mode);
      } else 
      if (pitlab_flight_stack) {
        pt_flight_mode = ap_custom_mode;
      } else { //  APM Flight Stack
        pt_flight_mode = ap_custom_mode + 1; // AP_CONTROL_MODE_LIMIT - ls 5 bits
      }
  
      pt_fence_present = ap_fence_enabled?1:0;
      pt_fence_breached = ap_breach_status;
      // [-100,100] scaled to [-63,63] encoded in 7 bits, MSB is sign +6bits
      int8_t ap74_scaled_throttle = THR_To_63(ap74_throt);
      pt_throt = (uint8_t)abs(ap74_scaled_throttle);
      if (ap74_scaled_throttle < 0) {
        pt_throt |= 0x1<<6;
      }
      pt_imu_temp = ap26_temp;
      /*
      if (ap26_temp < 0)          // @rotorman suggestion
        pt_imu_temp = 0;
      else
        if (ap26_temp > 255)
          pt_imu_temp = 255;
       else
         pt_imu_temp = (uint8_t)ap26_temp;
       */
         
      bit32Pack(pt_flight_mode, 0, 5);      // Flight mode   0-32 - 5 bits
      bit32Pack(pt_simple ,5, 2);           // Simple/super simple mode flags
      bit32Pack(pt_land_complete ,7, 1);    // Landed flag
      bit32Pack(pt_armed ,8, 1);            // Armed
      bit32Pack(pt_bat_fs ,9, 1);           // Battery failsafe flag
      bit32Pack(pt_ekf_fs ,10, 2);          // EKF failsafe flag
      bit32Pack(pt_fs ,12, 1);              // Ardupilot uses this bit to signal a generic failsafe, hardcoded = 0
      bit32Pack(pt_fence_present ,13, 1);   // Fence enabled
      bit32Pack(pt_fence_breached ,14, 1);  // Fence breached
      bit32Pack(pt_throt, 19, 7);            // Throttle
      bit32Pack(pt_imu_temp, 26, 6);        // imu temperature in cdegC

      #if defined Frs_Debug_All || defined Frs_Debug_APStatus
        PrintFrPeriod(0); 
        Log.print("Passthru out AP_status 0x5001: ");   
        Log.print(" pt_flight_mode="); Log.print(pt_flight_mode);
        Log.print(" pt_simple="); Log.print(pt_simple);
        Log.print(" pt_land_complete="); Log.print(pt_land_complete);
        Log.print(" pt_armed="); Log.print(pt_armed);
        Log.print(" pt_bat_fs="); Log.print(pt_bat_fs);
        Log.print(" pt_ekf_fs="); Log.print(pt_ekf_fs);
        Log.print(" pt_fs="); Log.print(pt_fs);
        Log.print(" pt_fence_present="); Log.print(pt_fence_present);
        Log.print(" pt_fence_breached="); Log.print(pt_fence_breached);
        Log.print(" pt_imu_temp="); Log.print(pt_imu_temp);
        Log.print(" pt_throt="); Log.print(pt_throt);
        Log.print(" pt_payload="); Log.print(pt_payload); Log.print(" ");
        FrSkyPort::PrintPayload(passthru, downlink);
        Log.println();
      #endif

      FrSkyPort::PushToEmptyRow(msg_id, 0);       

    }
    //===================================================================   
    void FrSkyPort::Push_GPS_status_5002(uint16_t msg_id) {
      pt_payload = 0;
if (ap24_sat_visible > 15) {                // @rotorman 2021/01/18
  if (ap24_sat_visible == 255)
    pt_numsats = 0; // for special case 255 == unknown satellite count
  else
    pt_numsats = 15; // limit to 15 due to only 4 bits available
}  else 
     pt_numsats = ap24_sat_visible;
  
      bit32Pack(pt_numsats ,0, 4); 
          
      pt_gps_status = ap24_fixtype & 0x03;            // patch by @rotorman 2021/01/18
      pt_gps_adv_status = (ap24_fixtype >> 2) & 0x03; // patch by @rotorman 2021/01/18 
          
      pt_amsl = ap24_amsl / 100;  // dm
      if (pt_amsl > 127000) // upper bound +12.7 km
        pt_amsl = 127000;
      if (pt_amsl < -127000) // lower bound -12.7 km
        pt_amsl = -127000;
      
      if (ap24_eph > 1270)
         pt_hdop = 1270;   // unknown state 0xFFFF from MAVLink is also relayed as 127 @rotorman 2021/01/25
       else
         pt_hdop = ap24_eph / 10;   
          
      bit32Pack(pt_gps_status ,4, 2);       // part a, 2 bits
      bit32Pack(pt_gps_adv_status ,14, 2);  // part b, 2 bits
          
      #if defined Frs_Debug_All || defined Frs_Debug_GPS_status
        PrintFrPeriod(0); 
        Log.print("Passthru out GPS Status 0x5002: ");   
        Log.print(" pt_numsats="); Log.print(pt_numsats);
        Log.print(" pt_gps_status="); Log.print(pt_gps_status);
        Log.print(" pt_gps_adv_status="); Log.print(pt_gps_adv_status);
        Log.print(" pt_amsl="); Log.print(pt_amsl);
        Log.print(" pt_hdop="); Log.print(pt_hdop);
      #endif
          
      pt_amsl = prep_number(pt_amsl,2,2);                        
      pt_hdop = prep_number(pt_hdop,2,1);
          
      #if defined Frs_Debug_All || defined Frs_Debug_GPS_status
        Log.print(" After prep: pt_amsl="); Log.print(pt_amsl);
        Log.print(" pt_hdop="); Log.print(pt_hdop); 
        Log.print(" pt_payload="); Log.print(pt_payload); Log.print(" ");
        FrSkyPort::PrintPayload(passthru, downlink);
        Log.println(); 
      #endif     
              
      bit32Pack(pt_hdop ,6, 8);
      bit32Pack(pt_amsl ,22, 9);
      if (pt_amsl < 0) {
        bit32Pack(0, 31,1);   // 1=negative     // @rotorman  2021/01/18
      } else {
        bit32Pack(0, 31,0);       
      }

      FrSkyPort::PushToEmptyRow(msg_id, 0);  
    }
    //===================================================================   
    void FrSkyPort::Push_Bat1_5003(uint16_t msg_id) {   //  Into F.Port table from #1 SYS_status only
      pt_payload = 0;
  
      // pt_bat1_mAh is populated at #147 depending on battery id.  Into FrPort table from #1 SYS_status only.
      //pt_bat1_mAh = Total_mAh1();  // If record type #147 is not sent and good
  
      #if defined Frs_Debug_All || defined Debug_Batteries
        PrintFrPeriod(0); 
        Log.print("Passthru out Bat1 0x5003: ");   
        Log.print(" pt_bat1_volts="); Log.print((float)pt_bat1_volts / 10, 2);// print whole V float
        Log.print(" pt_bat1_amps="); Log.print((float)pt_bat1_amps / 10, 1);  // print whole As float
        Log.print(" pt_bat1_mAh="); Log.print(pt_bat1_mAh);
        Log.print(" pt_payload="); Log.print(pt_payload); Log.print(" ");
        FrSkyPort::PrintPayload(passthru, downlink);
        Log.println();               
      #endif
          
      bit32Pack(pt_bat1_volts ,0, 9);
      uint16_t w_bat1_amps = prep_number(roundf(pt_bat1_amps * 0.1F),2,1);    // NOTE, (0.1F) rounds to whole Amps for dA   
      bit32Pack(w_bat1_amps,9, 8);
      bit32Pack(pt_bat1_mAh,17, 15);
      //Log.printf("mantissa:%d  10exponent:%d mutiplier:%d \n", bit32Extract(pt_payload,10,7), bit32Extract(pt_payload,9,1), TenToPwr(bit32Extract(pt_payload,9,1)) );
      FrSkyPort::PushToEmptyRow(msg_id, 0);  
                       
    }
    //===================================================================
       
    void FrSkyPort::Push_Home_5004(uint16_t msg_id) {
      pt_payload = 0;
    
      lon1=hom.lon/180*PI;  // degrees to radians
      lat1=hom.lat/180*PI;
      lon2=cur.lon/180*PI;
      lat2=cur.lat/180*PI;

      //Calculate azimuth bearing of craft from home
      a=atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1));
      az=a*180/PI;  // radians to degrees
      if (az<0) az=360+az;

      pt_home_angle = Add360(az, -180);                  // Is now the angle from the craft to home in degrees
  
      pt_home_arrow = pt_home_angle * 0.3333;            // Units of 3 degrees for Frsky

      // Calculate the distance from home to craft
      dLat = (lat2-lat1);
      dLon = (lon2-lon1);
      a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
      c = 2* asin(sqrt(a));    // proportion of Earth's radius
      dis = 6371000 * c;       // radius of the Earth is 6371km

      if ((homGood) || (ap242_homGood)) {
        //Log.printf("hom.lat:%3.7f  hom.lon:%3.7f  hom.alt:%.0f  hom.hdg:%.0F\n", hom.lat, hom.lon, hom.alt, hom.hdg);
        pt_home_dist = (int)dis;        
      }
      else {
        pt_home_dist = 0;
      }
        pt_home_alt = ap33_alt_ag / 100;    // mm->dm
        
      #if defined Frs_Debug_All || defined Frs_Debug_Home
        PrintFrPeriod(0); 
        Log.print("Passthru out Home 0x5004: ");
        //Log.print("msg_id=");  Log.print(msg_id, HEX);       
        Log.print("pt_home_dist=");  Log.print(pt_home_dist);
        Log.print(" pt_home_alt=");  Log.print((float)(pt_home_alt/10), 1);
        Log.print(" az=");  Log.print(az);
        Log.print(" pt_home_angle="); Log.print(pt_home_angle);  
        Log.print(" pt_home_arrow="); Log.print(pt_home_arrow);         // units of 3 deg       
      #endif
      
      pt_home_dist = prep_number(roundf(pt_home_dist), 3, 2);
      bit32Pack(pt_home_dist ,0, 12);
      pt_home_alt = prep_number(roundf(pt_home_alt), 3, 2);
      bit32Pack(pt_home_alt ,12, 12);
      if (pt_home_alt < 0)
        bit32Pack(1,24, 1);
      else  
        bit32Pack(0,24, 1);
      bit32Pack(pt_home_arrow,25, 7);
      
      #if defined Frs_Debug_All || defined Frs_Debug_Home
        Log.print(" pt_payload="); Log.print(pt_payload); Log.print(" ");
        FrSkyPort::PrintPayload(passthru, downlink);
        Log.println();       
      #endif
      
      FrSkyPort::PushToEmptyRow(msg_id, 0);  

    }

    //===================================================================   
    
    void FrSkyPort::Push_VelYaw_5005(uint16_t msg_id) {
      pt_payload = 0;
  
      pt_vy = ap74_climb * 10;   // from #74   m/s to dm/s;
      pt_vx = ap74_grd_spd * 10;  // from #74  m/s to dm/s

      //pt_yaw = (float)ap33_gps_hdg / 10;  // (degrees*100) -> (degrees*10)
      pt_yaw = ap74_hdg * 10;              // degrees -> (degrees*10) or use ap_yaw (-rad, +rad)
  
      #if defined Frs_Debug_All || defined Frs_Debug_VelYaw
        PrintFrPeriod(0); 
        Log.print("Passthru out VelYaw 0x5005:");  
        Log.print(" pt_vy=");  Log.print(pt_vy,2);       
        Log.print(" pt_vx=");  Log.print(pt_vx,2);
        Log.print(" pt_yaw="); Log.print(pt_yaw/10,0);
     
      #endif
      if (pt_vy<0)
        bit32Pack(1, 8, 1);
      else
        bit32Pack(0, 8, 1);
      pt_vy = prep_number(roundf(pt_vy), 2, 1);  // Vertical velocity
      bit32Pack(pt_vy, 0, 8);   

      pt_vx = prep_number(roundf(pt_vx), 2, 1);  // Horizontal velocity
      bit32Pack(pt_vx, 9, 8);    
      pt_yaw = pt_yaw * 0.5f;                   // Unit = 0.2 deg
      bit32Pack(pt_yaw ,17, 11);  

     #if defined Frs_Debug_All || defined Frs_Debug_VelYaw
       Log.print(" After prep:"); \
       Log.print(" pt_vy=");  Log.print((int)pt_vy);          
       Log.print(" pt_vx=");  Log.print((int)pt_vx);  
       Log.print(" pt_yaw="); Log.print((int)pt_yaw);  
       Log.print(" pt_payload="); Log.print(pt_payload); Log.print(" ");
       FrSkyPort::PrintPayload(passthru, downlink);
       Log.println();                 
     #endif

     FrSkyPort::PushToEmptyRow(msg_id, 0);  
    
    }
    //=================================================================== 
      
    void FrSkyPort::Push_Atti_5006(uint16_t msg_id) {
      pt_payload = 0;
  
      pt_roll = (ap_roll * 5) + 900;             //  -- pt_roll units = [0,1800] <==> [-180,180]
      pt_pitch = (ap_pitch * 5) + 450;           //  -- pt_pitch units = [0,900] <==> [-90,90]
      pt_range = roundf(ap_range*100);   
      bit32Pack(pt_roll, 0, 11);
      bit32Pack(pt_pitch, 11, 10); 
      bit32Pack(prep_number(pt_range,3,1), 21, 11);
      #if defined Frs_Debug_All || defined Frs_Debug_AttiRange
        PrintFrPeriod(0); 
        Log.print("Passthru out Attitude 0x5006: ");         
        Log.print("pt_roll=");  Log.print(pt_roll);
        Log.print(" pt_pitch=");  Log.print(pt_pitch);
        Log.print(" pt_range="); Log.print(pt_range);
        Log.print(" Payload="); Log.println(pt_payload);  
      #endif

      FrSkyPort::PushToEmptyRow(msg_id, 0);   
     
    }
    //===================================================================   
    
    void FrSkyPort::Push_Parameters_5007(uint16_t msg_id, uint8_t sub_id) {
    
      switch(sub_id) {
        case 1:                                    // Frame type
          pt_param_id = 1;
          pt_frame_type = ap_type;
      
          pt_payload = 0;
          bit32Pack(pt_frame_type, 0, 24);
          bit32Pack(pt_param_id, 24, 4);

          #if defined Frs_Debug_All || defined Frs_Debug_Params
            PrintFrPeriod(0);  
            Log.print("Passthru out Params 0x5007: ");   
            Log.print(" pt_param_id="); Log.print(pt_param_id);
            Log.print(" pt_frame_type="); Log.print(pt_frame_type);  
            Log.print(" pt_payload="); Log.print(pt_payload);  Log.print(" "); 
            FrSkyPort::PrintPayload(passthru, downlink);
            Log.println();                
          #endif
      
          FrSkyPort::PushToEmptyRow(msg_id, sub_id);
          break;    
       
        case 4:    // Battery pack 1 capacity
          pt_param_id = sub_id;
          #if (Battery_mAh_Source == 2)    // Local
            pt_bat1_capacity = bat1_capacity;
          #elif  (Battery_mAh_Source == 1) //  FC
            pt_bat1_capacity = ap_bat1_capacity;
          #endif 

          pt_payload = 0;
          bit32Pack(pt_bat1_capacity, 0, 24);
          bit32Pack(pt_param_id, 24, 4);

          #if defined Frs_Debug_All || defined Frs_Debug_Params || defined Debug_Batteries
            PrintFrPeriod(0);       
            Log.print("Passthru out Params 0x5007: ");   
            Log.print(" pt_param_id="); Log.print(pt_param_id);
            Log.print(" pt_bat1_capacity="); Log.print(pt_bat1_capacity);  
            Log.print(" pt_payload="); Log.print(pt_payload);  Log.print(" "); 
            FrSkyPort::PrintPayload(passthru, downlink);
            Log.println();                   
          #endif

          FrSkyPort::PushToEmptyRow(msg_id, sub_id); 
          break; 
      
        case 5:                 // Battery pack 2 capacity
          pt_param_id = sub_id;
          #if (Battery_mAh_Source == 2)    // Local
            pt_bat2_capacity = bat2_capacity;
          #elif  (Battery_mAh_Source == 1) //  FC
            pt_bat2_capacity = ap_bat2_capacity;
          #endif  

          pt_payload = 0;
          bit32Pack(pt_bat2_capacity, 0, 24);
          bit32Pack(pt_param_id, 24, 4);
      
          #if defined Frs_Debug_All || defined Frs_Debug_Params || defined Debug_Batteries
            PrintFrPeriod(0);  
            Log.print("Passthru out Params 0x5007: ");   
            Log.print(" pt_param_id="); Log.print(pt_param_id);
            Log.print(" pt_bat2_capacity="); Log.print(pt_bat2_capacity); 
            Log.print(" pt_payload="); Log.print(pt_payload);  Log.print(" "); 
            FrSkyPort::PrintPayload(passthru, downlink);
            Log.println();           
          #endif
      
          FrSkyPort::PushToEmptyRow(msg_id, sub_id);
          break; 
    
         case 6:               // Number of waypoints in mission                       
          pt_param_id = sub_id;
          pt_mission_count = ap_mission_count;

          pt_payload = 0;
          bit32Pack(pt_mission_count, 0, 24);
          bit32Pack(pt_param_id, 24, 4);

          FrSkyPort::PushToEmptyRow(msg_id, sub_id);        
      
          #if defined Frs_Debug_All || defined Frs_Debug_Params || defined Debug_Batteries
            PrintFrPeriod(0); 
            Log.print("Passthru out Params 0x5007: ");   
            Log.print(" pt_param_id="); Log.print(pt_param_id);
            Log.print(" pt_mission_count="); Log.println(pt_mission_count);           
          #endif
 
          pt_paramsSent = true;          // get this done early on and then regularly thereafter

          break;
      }    
    }
    //===================================================================   
    
    void FrSkyPort::Push_Bat2_5008(uint16_t msg_id) {
       pt_payload = 0;
   
       pt_bat2_volts = ap_voltage_battery2 / 100;         // Were mV, now dV  - V * 10
       pt_bat2_amps = ap_current_battery2 ;               // Remain       dA  - A * 10   
   
      // pt_bat2_mAh is populated at #147 depending on battery id
      //pt_bat2_mAh = Total_mAh2();  // If record type #147 is not sent and good
  
      #if defined Frs_Debug_All || defined Debug_Batteries
        PrintFrPeriod(0);  
        Log.print("Passthru out Bat2 0x5008: ");   
        Log.print(" pt_bat2_volts="); Log.print(pt_bat2_volts);
        Log.print(" pt_bat2_amps="); Log.print(pt_bat2_amps);
        Log.print(" pt_bat2_mAh="); Log.print(pt_bat2_mAh);
        Log.print(" pt_payload="); Log.print(pt_payload);  Log.print(" "); 
        FrSkyPort::PrintPayload(passthru, downlink);
        Log.println();                  
      #endif        
          
      bit32Pack(pt_bat2_volts ,0, 9);
      pt_bat2_amps = prep_number(roundf(pt_bat2_amps * 0.1F),2,1);          
      bit32Pack(pt_bat2_amps,9, 8);
      bit32Pack(pt_bat2_mAh,17, 15);      

      FrSkyPort::PushToEmptyRow(msg_id, 1);           
    }


    //===================================================================   

    void FrSkyPort::Push_WayPoint_5009(uint16_t msg_id) {
      pt_payload = 0;
  
      pt_ms_seq = ap_ms_seq;                                      // Current WP seq number, wp[0] = wp1, from regular #42
  
      pt_ms_dist = ap_wp_dist;                                        // Distance to next WP  

      pt_ms_xtrack = ap_xtrack_error;                                 // Cross track error in metres from #62
      pt_ms_target_bearing = ap_target_bearing;                       // Direction of next WP
      pt_ms_cog = ap24_cog * 0.01;                                      // COG in degrees from #24
      int32_t angle = (int32_t)wrap_360(pt_ms_target_bearing - pt_ms_cog);
      int32_t arrowStep = 360 / 8; 
      pt_ms_offset = ((angle + (arrowStep/2)) / arrowStep) % 8;       // Next WP bearing offset from COG

      /*
   
    0 - up
    1 - up-right
    2 - right
    3 - down-right
    4 - down
    5 - down - left
    6 - left
    7 - up - left
 
       */
      #if defined Frs_Debug_All || defined Frs_Debug_Missions
        PrintFrPeriod(0);  
        Log.print("Passthru out RC 0x5009: ");   
        Log.print(" pt_ms_seq="); Log.print(pt_ms_seq);
        Log.print(" pt_ms_dist="); Log.print(pt_ms_dist);
        Log.print(" pt_ms_xtrack="); Log.print(pt_ms_xtrack, 3);
        Log.print(" pt_ms_target_bearing="); Log.print(pt_ms_target_bearing, 0);
        Log.print(" pt_ms_cog="); Log.print(pt_ms_cog, 0);  
        Log.print(" pt_ms_offset="); Log.print(pt_ms_offset);
        Log.print(" pt_payload="); Log.print(pt_payload);  Log.print(" "); 
        FrSkyPort::PrintPayload(passthru, downlink);         
        Log.println();      
      #endif

      bit32Pack(pt_ms_seq, 0, 10);    //  WP number

      pt_ms_dist = prep_number(roundf(pt_ms_dist), 3, 2);       //  number, digits, power
      bit32Pack(pt_ms_dist, 10, 12);    

      pt_ms_xtrack = prep_number(roundf(pt_ms_xtrack), 1, 1);  
      bit32Pack(pt_ms_xtrack, 22, 6); 

      bit32Pack(pt_ms_offset, 29, 3);  

      FrSkyPort::PushToEmptyRow(msg_id, 1);  
        
    }

    //===================================================================

    void FrSkyPort::Push_RPM_500A(uint16_t msg_id) {
      pt_payload = 0;
      pt_rpm1 = (int16_t)roundf(ap_rpm1  * 0.1);
      pt_rpm2 = (int16_t)roundf(ap_rpm2 * 0.1);

      bit32Pack(pt_rpm1, 0, 16);
      bit32Pack(pt_rpm2, 16, 16);
      
      //pt_payload = pt_rpm1 | (pt_rpm2 << 16);  //alex

      FrSkyPort::PushToEmptyRow(msg_id, 1);

      #if defined Frs_Debug_All || defined Frs_Debug_RPM
        PrintFrPeriod(0);
        Log.print("Passthru out RC 0x500A: ");
        Log.print(" pt_rpm1="); Log.print(pt_rpm1);
        Log.print(" pt_rpm2="); Log.print(pt_rpm2);
        Log.print(" pt_payload="); Log.print(pt_payload);  Log.print(" ");
        FrSkyPort::PrintPayload(passthru, downlink);
        Log.println();
      #endif
    }

    //===================================================================

    void FrSkyPort::Push_Terrain_500B(uint16_t msg_id) {
      pt_payload = 0;
      pt_height_above_terrain = prep_number(roundf(ap136_current_height*10), 3, 2);
      pt_terrain_unhealthy = ap_terrain_spacing == 0 ? 1 : 0;

      bit32Pack(pt_height_above_terrain, 0, 13);
      bit32Pack(pt_terrain_unhealthy, 13, 1);

      //pt_payload = pt_height_above_terrain | (pt_terrain_unhealthy << 13); // alex

      FrSkyPort::PushToEmptyRow(msg_id, 1);

      #if defined Frs_Debug_All || defined Frs_Debug_Terrain
        PrintFrPeriod(0);
        Log.print("Passthru out RC 0x500B: ");
        Log.print(" ap_current_height="); Log.print(ap136_current_height);
        Log.print(" ap_terrain_spacing="); Log.print(ap_terrain_spacing);
        Log.print(" pt_height_above_terrain="); Log.print(pt_height_above_terrain);
        Log.print(" pt_terrain_unhealthy="); Log.print(pt_terrain_unhealthy);
        Log.print(" pt_payload="); Log.print(pt_payload);  Log.print(" ");
        FrSkyPort::PrintPayload(passthru, downlink);
        Log.println();
      #endif
    }

    //===================================================================  
    
    void FrSkyPort::Push_Servo_Raw_50F1(uint16_t msg_id) {
    uint8_t sv_chcnt = 8;
      pt_payload = 0;
  
      if (sv_count+4 > sv_chcnt) { // 4 channels at a time
        sv_count = 0;
        return;
      } 

      uint8_t  chunk = sv_count / 4; 

      pt_sv[1] = PWM_To_63(ap65_chan_raw[sv_count]);     // PWM 1000 to 2000 -> 6bit 0 to 63
      pt_sv[2] = PWM_To_63(ap65_chan_raw[sv_count+1]);    
      pt_sv[3] = PWM_To_63(ap65_chan_raw[sv_count+2]); 
      pt_sv[4] = PWM_To_63(ap65_chan_raw[sv_count+3]); 

      bit32Pack(chunk, 0, 4);                // chunk number, 0 = chans 1-4, 1=chans 5-8, 2 = chans 9-12, 3 = chans 13 -16 .....
      bit32Pack(Abs(pt_sv[1]) ,4, 6);        // fragment 1 
      if (pt_sv[1] < 0)
        bit32Pack(1, 10, 1);                 // neg
      else 
        bit32Pack(0, 10, 1);                 // pos          
      bit32Pack(Abs(pt_sv[2]), 11, 6);      // fragment 2 
      if (pt_sv[2] < 0) 
        bit32Pack(1, 17, 1);                 // neg
      else 
        bit32Pack(0, 17, 1);                 // pos   
      bit32Pack(Abs(pt_sv[3]), 18, 6);       // fragment 3
      if (pt_sv[3] < 0)
        bit32Pack(1, 24, 1);                 // neg
      else 
        bit32Pack(0, 24, 1);                 // pos      
      bit32Pack(Abs(pt_sv[4]), 25, 6);       // fragment 4 
      if (pt_sv[4] < 0)
        bit32Pack(1, 31, 1);                 // neg
      else 
        bit32Pack(0, 31, 1);                 // pos  
        
      uint8_t sv_num = sv_count % 4;

      FrSkyPort::PushToEmptyRow(msg_id, sv_num + 1);  

      #if defined Frs_Debug_All || defined Frs_Debug_Servo
        PrintFrPeriod(0);  
        Log.print("Passthru out Servo_Raw 0x50F1: ");  
        Log.print(" sv_chcnt="); Log.print(sv_chcnt); 
        Log.print(" sv_count="); Log.print(sv_count); 
        Log.print(" chunk="); Log.print(chunk);
        Log.print(" pt_sv1="); Log.print(pt_sv[1]);
        Log.print(" pt_sv2="); Log.print(pt_sv[2]);
        Log.print(" pt_sv3="); Log.print(pt_sv[3]);   
        Log.print(" pt_sv4="); Log.print(pt_sv[4]); 
        Log.print(" pt_payload="); Log.print(pt_payload);  Log.print(" "); 
        FrSkyPort::PrintPayload(passthru, downlink);
        Log.println();             
      #endif

      sv_count += 4; 
    }
    //===================================================================   
    
    void FrSkyPort::Push_VFR_Hud_50F2(uint16_t msg_id) {
      pt_payload = 0;
  
      pt_air_spd = ap74_air_spd * 10;      // from #74  m/s to dm/s
      pt_throt = ap74_throt;               // 0 - 100%
      pt_bar_alt = ap74_amsl * 10;         // m to dm

      #if defined Frs_Debug_All || defined Frs_Debug_Hud
        PrintFrPeriod(0);  
        Log.print("Passthru out Hud 0x50F2: ");   
        Log.print(" pt_air_spd="); Log.print(pt_air_spd);
        Log.print(" pt_throt=");   Log.print(pt_throt);
        Log.print(" pt_bar_alt="); Log.print(pt_bar_alt);
        Log.print(" pt_payload="); Log.print(pt_payload);  Log.print(" "); 
        FrSkyPort::PrintPayload(passthru, downlink);
        Log.println();             
      #endif
  
      pt_air_spd = prep_number(roundf(pt_air_spd), 2, 1);  
      bit32Pack(pt_air_spd, 0, 8);    

      bit32Pack(pt_throt, 8, 7);

      pt_bar_alt =  prep_number(roundf(pt_bar_alt), 3, 2);
      bit32Pack(pt_bar_alt, 15, 12);
      if (pt_bar_alt < 0)
        bit32Pack(1, 27, 1);  
      else
       bit32Pack(0, 27, 1); 
    
      FrSkyPort::PushToEmptyRow(msg_id, 1); 
        
    }
    //===================================================================  
    void FrSkyPort::Push_Wind_Estimate_50F3(uint16_t msg_id) {
      pt_payload = 0;
    }
    //===================================================================
               
    void FrSkyPort::Push_Rssi_F101(uint16_t msg_id) {          // msg_id 0xF101 RSSI tell LUA script in Taranis we are connected
      pt_payload = 0;
  
      if (rssiGood)
        pt_rssi = ap_rssi;            // always %
      else
        pt_rssi = 254;     // We may have a connection but don't yet know how strong. Prevents spurious "Telemetry lost" announcement
        
      #ifdef RSSI_Override   // dummy rssi override for debugging
        pt_rssi = RSSI_Override;
      #endif

      if(pt_rssi < 1){    // Patch from hasi123 
        pt_rssi = 69;
      }

      bit32Pack(pt_rssi ,0, 32);

      #if defined Frs_Debug_All || defined Frs_Debug_Rssi
        PrintFrPeriod(0);    
        Log.print("Passthru out Rssi 0xF101: ");   
        Log.print(" pt_rssi="); Log.print(pt_rssi);
        Log.print(" pt_payload="); Log.print(pt_payload);  Log.print(" "); 
        FrSkyPort::PrintPayload(passthru, downlink);
        Log.println();             
      #endif

      FrSkyPort::PushToEmptyRow(msg_id, 1); 
    }
         
    //===================================================================  
    
    uint16_t FrSkyPort::MatchWaitingParamRequests(char * paramid) {
      for (int i = 0 ; i < mt20_rows ; i++ ) {  // try to find match on waiting param requests
    //    Log.printf("MatchWaiting. i=%d  mt20_rows=%d  inuse=%d param_id=%s paramid=%s \n", i, mt20_rows, mt20[i].inuse, mt20[i].param_id,  ap22_param_id);

        bool paramsequal = (strcmp (mt20[i].param_id, paramid) == 0);
        if ( (mt20[i].inuse) && paramsequal ) {
          return i;             
         }
      }
      return 0xffff;  // this mean no match  
    }  
    //===================================================================  

    void FrSkyPort::PrintPayload(msg_class_t msg_class, telem_direction_t telem_direction)  {
      uint8_t *bytes = 0;
      uint8_t sz = 0;
      
      char in_or_out;
      if (telem_direction == uplink) {
        in_or_out = '<';
      } else
      if (telem_direction == downlink) {  
        in_or_out = '>';
      } else {
        in_or_out = ' ';
      }

      bool dbg_mavlite = false;
      #if defined Debug_Mavlite
        dbg_mavlite = true;
      #endif
      
      if ( (msg_class == passthru) && (!dbg_mavlite) ) {
        Log.print(" passthru payload 0x10 ");
        bytes = (uint8_t*)&pt_payload;         // cast to bytes
        sz = 4;   
      } else
      if (msg_class == mavlite) {
        Log.print(" mavlite payload 0x32 ");   
        bytes = (uint8_t*)&mtf_payload;       // cast to bytes
        sz = 6;
      } 
      
      if ( (msg_class == passthru) && (dbg_mavlite) )  return;
       
      for (int i = 0 ; i < sz ; i++) {
        Printbyte(bytes[i], 0, in_or_out);   
      }
       
      Log.print("\t");
      for (int i = 0 ; i <sz ; i++) {
         if ((bytes[i] > 31) && (bytes[i] < 127)) {
          Log.write(bytes[i]);  
         } else {
           Log.print('.');  
         }
      }
      Log.print("\t");
    } 
    //===================================================================  

    void FrSkyPort::PrintMavLiteUplink() {
  
      //  if (frbuf[3] == 0) Log.println();  // seq == 0    
              
        Printbyte(frbuf[0], false, '<');  // 0x7E
        Printbyte(frbuf[1], false, '<');  // 0x0D
        Printbyte(frbuf[2], false, '<');  // 0x30 
        Log.print("  ");
        Printbyte(frbuf[3], false, '<');  // seq
        Log.print("  ");       
        for (int i = 4 ; i <= 8 ; i++ ) {
          Printbyte(frbuf[i], false, '<');
        }
        Log.print(" CRC=");
        Printbyte(frbuf[9], false, '<');  // ?

        Log.print("\t");
        
        Printbyte(frbuf[3], false, ' ');  // seq

        if (frbuf[3] == 0)  {    // if seq == 0
          Log.print("  ");
          Printbyte(frbuf[4], false, ' ');   // length
          Printbyte(frbuf[5], false, ' ');   // msg_id 
          Log.print(" [");
          for (int i = 6 ; i <= 8 ; i++ ) {  
            if ((frbuf[i] > 31) && (frbuf[i] < 127)) Log.write(frbuf[i]);   // print ascii
          }
        } else {                  // seq > 0
          Log.print("[");
          for (int i = 4 ; i <= 8 ; i++ ) {  
           if ((frbuf[i] > 31) && (frbuf[i] < 127)) Log.write(frbuf[i]);   // print ascii
          }       
        }
        Log.println("] ");

    }
    
    //===================================================================      
    void  FrSkyPort::Print_PWM_Channels(int16_t *ch, uint16_t num_of_channels) {
      for (int i = 0 ; i < num_of_channels ; i++ ) {
        Log.printf("%2d:%4d  ", i+1, *(ch+i)); 
      }
      Log.printf("  rssi:%d\n", FrSkyPort::pwm_rssi );
    }
    //===================================================================      

   void FrSkyPort::PrintBuffer(uint8_t *buf, uint8_t length) {
    //Log.printf("length=%d\t", length);
    for (int i = 0 ; i < length ; i++) {  
      if (*(buf+i) <= 0xf) Log.print("0");
        Log.print(*(buf+i),HEX);
        Log.write(" ");
      }
      Log.println();
  }
   
    //===================================================================
    void FrSkyPort::CheckForTimeouts() {
      
     if ((millis() - serGood_millis) > (FrSkyPort::timeout_secs * 1000)) {
       FrSkyPort::serGood = false;
       FrSkyPort::frGood = false; 
       FrSkyPort::pwmGood = false;           
     }    

     if ((millis() - frGood_millis) > (FrSkyPort::timeout_secs * 1000)) {
       FrSkyPort::frGood = false;
       FrSkyPort::pwmGood = false;  
     }
     
     if ((millis() - pwmGood_millis) > (FrSkyPort::timeout_secs * 1000)) {
       FrSkyPort::pwmGood = false;
     }

     
     FrSkyPort::ServiceStatusLed(); 
     FrSkyPort::ReportOnlineStatus();
    }   
    //===================================================================     

    void FrSkyPort::ServiceStatusLed() {
      if (fpStatusLed == 99) return;
      
      if (FrSkyPort::pwmGood) {
          if (InvertfpLed) {
           FrSkyPort::fpLedState = LOW;
          } else {
           FrSkyPort::fpLedState = HIGH;
          }
          digitalWrite(fpStatusLed, fpLedState); 
      }
        else {
          if (serGood) {
            FrSkyPort::BlinkFpLed(500);
          }
        }
      digitalWrite(fpStatusLed, fpLedState); 
    }
    //===================================================================   
    void FrSkyPort::BlinkFpLed(uint32_t period) {
         if (millis() - FrSkyPort::fpLed_millis >= period) {    // blink period
            FrSkyPort::fpLed_millis = millis();
            if (FrSkyPort::fpLedState == LOW) {
              FrSkyPort::fpLedState = HIGH; }   
            else {
              FrSkyPort::fpLedState = LOW;  } 
          }
    }  
    //===================================================================  
    
    void FrSkyPort::ReportOnlineStatus() {
  
       if (FrSkyPort::frGood != FrSkyPort::frPrev) {  // report on change of status
         FrSkyPort::frPrev = FrSkyPort::frGood;
         if (frGood) {
           Log.println("FrPort read good!");
           LogScreenPrintln("FrPort read ok");         
         } else {
          Log.println("FrPort read timeout!");
          LogScreenPrintln("FrPort timeout");         
         }
       }
       
       if (FrSkyPort::pwmGood != FrSkyPort::pwmPrev) {  // report on change of status
         FrSkyPort::pwmPrev = FrSkyPort::pwmGood;
         if (pwmGood) {
           Log.println("RC PWM good");
           LogScreenPrintln("RC PWM good");         
         } else {
          Log.println("RC PWM timeout");
          LogScreenPrintln("RC PWM timeout");         
         }
       }
    } 
    
    //======================================================
    pol_t FrSkyPort::getPolarity(uint8_t pin) {
      uint32_t pw_hi = 0;
      uint32_t pw_lo = 0;   
      
      for (int i = 0; i < 1000; i++) {
        pw_lo += (digitalRead(pin) == LOW); 
        pw_hi += (digitalRead(pin) == HIGH);         
        delayMicroseconds(500); // so test for 0.5 seconds
      }  

      //Log.printf("hi:%d  lo:%d\n", pw_hi, pw_lo);  
      if ( ( (pw_lo == 1000) && (pw_hi == 0) ) || ((pw_lo == 0) && (pw_hi == 1000) ) ) 
      {
        return no_traffic;
      }
      if (pw_hi > pw_lo) {
        return idle_high;
      } else {
        return idle_low;        
      }
    }   
    //===================================================================     

    uint32_t FrSkyPort::getBaud(uint8_t s_pin, pol_t pl) {
      Log.print("autoBaud - sensing pin "); Log.println(s_pin);
      uint8_t i = 0;
      uint8_t col = 0;
      pinMode(s_pin, INPUT);       
      digitalWrite (s_pin, HIGH); // pull up enabled for noise reduction ?

      uint32_t gb_baud = getConsistent(s_pin, pl);
      while (gb_baud == 0) {
        if(ftgetBaud) {
          ftgetBaud = false;
        }
        i++;
        if ((i % 5) == 0) {
          Log.print(".");
          col++; 
        }
        if (col > 60) {
          Log.println(); 
          Log.print("No telemetry found on pin "); Log.println(s_pin);
          col = 0;
          i = 0;
        }
        gb_baud = getConsistent(s_pin, pl);
      } 
      if (!ftgetBaud) {
        Log.println();
      }

      Log.print("autoBaud: "); Log.print(gb_baud);  Log.println(" b/s");
      LogScreenPrintln("autoBaud: " + String(gb_baud));
      
      return(gb_baud);  
    }
    //===================================================================   
    uint32_t FrSkyPort::getConsistent(uint8_t s_pin, pol_t pl) {
      uint32_t t_baud[5];

      while (true) {  
        t_baud[0] = SenseUart(s_pin, pl);
        delay(10);
        t_baud[1] = SenseUart(s_pin, pl);
        delay(10);
        t_baud[2] = SenseUart(s_pin, pl);
        delay(10);
        t_baud[3] = SenseUart(s_pin, pl);
        delay(10);
        t_baud[4] = SenseUart(s_pin, pl);
        #if defined Debug_All || defined Debug_Baud
          Log.print("  t_baud[0]="); Log.print(t_baud[0]);
          Log.print("  t_baud[1]="); Log.print(t_baud[1]);
          Log.print("  t_baud[2]="); Log.print(t_baud[2]);
          Log.print("  t_baud[3]="); Log.println(t_baud[3]);
        #endif  
        if (t_baud[0] == t_baud[1]) {
          if (t_baud[1] == t_baud[2]) {
            if (t_baud[2] == t_baud[3]) { 
              if (t_baud[3] == t_baud[4]) {   
                #if defined Debug_All || defined Debug_Baud    
                  Log.print("Consistent baud found="); Log.println(t_baud[3]); 
                #endif   
                return t_baud[3]; 
              }          
            }
          }
        }
      }
    }
    //===================================================================   
    uint32_t FrSkyPort::SenseUart(uint8_t  s_pin, pol_t pl) {

    uint32_t pw = 999999;  //  Pulse width in uS
    uint32_t min_pw = 999999;
    uint32_t su_baud = 0;
    const uint32_t su_timeout = 5000; // uS !  Default timeout 1000mS!

      #if defined Debug_All || defined Debug_Baud
        Log.printf("s_pin:%d  rxInvert:%d\n", s_pin, frInvert);  
      #endif  

      if (pl == idle_low) {       
        while(digitalRead(s_pin) == 0){ };  // idle_low, wait for high bit (low pulse) to start
      } else {
        while(digitalRead(s_pin) == 1){ };  // idle_high, wait for high bit (high pulse) to start  
      }

      for (int i = 0; i < 10; i++) {

      if (pl == idle_low) {                //  Returns the length of the pulse in uS
          pw = pulseIn(s_pin,HIGH, su_timeout);     
        } else {
          pw = pulseIn(s_pin,LOW, su_timeout);    
        }    

        if (pw !=0) {
          min_pw = (pw < min_pw) ? pw : min_pw;  // Choose the lowest
          //Log.printf("i:%d  pw:%d  min_pw:%d\n", i, pw, min_pw);      
        } 
      } 
      #if defined Debug_All || defined Debug_Baud
        Log.printf("pw:%d  min_pw:%d\n", pw, min_pw);
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
    //===================================================================
    String FrSkyPort::frportName(frport_t frp) {
      if (frp == f_none) {
        return "f_none";   
      } else
      if (frp == f_port1) {
        return "f_port1";   
      } else     
       if (frp == f_port2) {
        return "f_port2";   
      } else        
       if (frp == s_port) {
        return "s_port";   
      } else    
       if (frp == f_auto) {
        return "f_auto";   
      }
      return "f_none";             
    }
    //===================================================================   
    void FrSkyPort::WriteSBUS(uint8_t *buf) {
    #if defined Support_SBUS_Out      
      uint32_t byteDuration = 0;
      uint8_t i = 0;
      while (i < 25) {   //  22 bytes plus digi byte = 16 channels plus ch 17 digi channel               
        // Sending one byte of data takes 11bits in 8E1 which = 110us at 100000 baud, but we 
        // need one stopbit longer so we start the sending the next byte after 120us not 110
        byteDuration += micros();
        if (byteDuration >= 124){
          byteDuration = 0;
          sbusSerial.write(buf[i]); // write one byte
          i++; 
        }
      }
    #endif
    }



    
#endif  // end of FrSky port support


      
           
