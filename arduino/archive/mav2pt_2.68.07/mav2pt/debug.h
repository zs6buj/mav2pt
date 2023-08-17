//=================================================================================================   
//=============================     D E B U G G I N G   O P T I O N S   ===========================
//=================================================================================================

//#define inhibit_SPort     // Use me to send debug messages only, out of GPIO1/TX0 on ESP32_Variant 3, DL V3 internal ESP32
//#define Mav_Debug_All
//#define Frs_Debug_All
//#define Mav_Debug_RingBuff

//#define Debug_Air_Mode
//#define Debug_Relay_Mode

//#define Mav_Debug_Params
//#define Debug_BT    
//#define Debug_FC_Down         // traffic down from FC to Ring Buffer
//#define Debug_FC_Up           // traffic up to FC from GCS
//#define Debug_GCS_Down        // traffic from RB to GCS
//#define Debug_GCS_Up          // traffic up from GCS to FC

//#define Debug_Read_UDP_GCS  

//#define Debug_sendUDP_GCS

//#define Debug_Read_UDP_FC  
//#define Debug_sendUDP_FC  

//#define Mav_Debug_Servo
//#define Frs_Debug_Servo
//#define Mav_Debug_Rssi        // #109 > #65 > #35
//#define Frs_Debug_Rssi        // 0xF101
//#define Mav_Debug_RC           
//#define Frs_Debug_RC

//#define Frs_Debug_Params       //0x5007
//#define Frs_Debug_APStatus    // 0x5001
//#define Mav_Debug_SysStatus   // #1 && battery
//#define Debug_Batteries       // 0x5003

//#define Frs_Debug_Home        // 0x5004

//#define Mav_Debug_GPS_Raw     // #24
//#define Mav_Debug_GPS_Int     // #33
//#define Frs_Debug_LatLon      // 0x800
//#define Frs_Debug_VelYaw      // 0x5005
//#define Frs_Debug_GPS_status  // 0x5002
//#define Mav_Debug_Scaled_IMU
//#define Mav_Debug_Raw_IMU
//#define Mav_Debug_Hud         // #74
//#define Frs_Debug_Hud         // 0x50F2
//#define Mav_Debug_Scaled_Pressure
//#define Mav_Debug_Attitude    // #30
//#define Mav_Debug_Range       // #173
//#define Frs_Debug_AttiRange   // 0x5006
//#define Mav_Debug_Terrain     // #136
//#define Frs_Debug_Terrain     // 0x500B
//#define Mav_Debug_Fence     // #162
//#define Frs_Debug_Fence     // 0x500B
//#define Mav_Debug_StatusText  // #253  
//#define Frs_Debug_StatusText  // 0x5000
//#define Mav_Debug_Missions
//#define Frs_Debug_Missions   
//#define Mav_Debug_System_Time   
//#define Decode_Non_Essential_Mav 
//#define Debug_Radio_Status  
//#define Debug_GCS_Unknown
//#define Debug_Param_Request_Read
//#define Mav_Show_Unknown_Msgs             
//#define Mav_Print_All_Msgid

//#define DEBUG_EEPROM
//#define DEBUG_WEB_SETTINGS

//#define Mav_Debug_RPM
//#define Frs_Debug_RPM
//#define Debug_SD   
//#define Debug_WiFi
//#define Debug_Loop_Period

//#define Mav_Debug_Commands

//#define Debug_SRAM



//#define Mav_Debug_FC_Heartbeat
//#define Mav_Debug_GCS_Heartbeat
//#define Debug_Our_FC_Heartbeat
//#define Debug_Param_Request_Read  // #20
//#define Debug_Param_Request_List  // #21
//#define Mav_Debug_Params
//#define Frs_Debug_Payload
//#define Debug_FrPort_Serial_Loop
//#define Debug_FrPort_Switching
//#define Frs_Debug_Period
//#define Support_SBUS_Out 
//#define Debug_Read_TCP
//#define Debug_Read_UDP
//#define Debug_sendTCP
//#define Debug_sendUDP
//#define Debug_Inject_Delay
//#define MavLite_Debug_Scheduler
//#define Debug_Mavlite 
//#define Debug_Mavlite_Chunking
//#define Debug_Mavlite_SPort

//#define Mav_List_Params       // Use this to test uplink to Flight Controller 

//#define Debug_FrPort_Stream 
//#define Debug_FrPort_Stream_Out
//#define Debug_FrPort_Safe_Read
//#define Debug_FPort_Buffer 
//#define CRC_Test_Case
//#define Debug_CRC
//#define Frs_Debug_Scheduler // - this debugger affects the performance of the scheduler when activated
//#define Derive_PWM
//#define Debug_PWM_Channels
//#define Debug_Baud 
//#define Debug_FrSPort_Loop_Period
//#define Mav_Debug_Home_Position

//#define Debug_SITL_Input
//#define Slowdown_SITL_Input - for Ubuntu 20.04, sitl telemetry 
