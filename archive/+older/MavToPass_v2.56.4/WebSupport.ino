//================================================================================================= 
//================================================================================================= 
//
//                                      W E B   S U P P O R T  
// 
//================================================================================================= 
//================================================================================================= 
#if defined webSupport

 String styleLogin =
    "<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
    "input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
    "#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
    "#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
    "form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
    ".btn{background:#3498db;color:#fff;cursor:pointer} .big{ width: 1em; height: 1em;}</style>";
   
 String styleSettings =
    "<style>"
    "input{background:#f1f1f1;border:1;margin:8px auto;font-size:14px}"
    "body{background:#3498db;font-family:arial;font-size:10px;color:black}"
    "#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
    "form{background:#fff;max-width:400px;margin:30px auto;padding:30px;border-radius:10px;text-align:left;font-size:16px}"
    ".big{ width: 1em; height: 1em;} .bold {font-weight: bold;} "
    "</style>";

 String otaIndex = styleLogin +
    "<style>form{max-width:450px;margin:75px auto;padding:30px;border-radius:10px;text-align:centre}</style>"
    "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
    "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
    "<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
    "<label id='file-input' for='file'>   Choose file...</label>"
    "<input type='submit' class=btn value='Update'>"
    "<br><br>"
    "<div id='prg'></div>"
    "<br><div id='prgbar'><div id='bar'></div></div><br></form>"
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
    "}"
    "}, false);"
    "return xhr;"
    "},"
    "success:function(d, s) {"
    "console.log('success!') "
    "},"
    "error: function (a, b, c) {"
    "}"
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
  server.on("/settingsReturnIndex", handleSettingsReturn); 
  server.on("/otaIndex", handleOtaPage); 
  
  server.begin();
  
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
    loginPage += "<input name=userid  placeholder='User ID' size='10'> ";
    loginPage += "<input name=pwd placeholder=Password type=Password> <br> <br>";
    loginPage += "<input type='radio' class='big' name='_nextFn' value='ota' checked > Update Firmware &nbsp &nbsp";
    loginPage += "<input type='radio' class='big' name='_nextFn' value='set' > Settings <br> <br>"; 
    loginPage += "<input type=submit onclick=check(this.form) class=btn value=Login></form>";
    loginPage += "<script>";
    loginPage += "function check(form) {";
    sprintf(temp, "if(form.userid.value=='admin' && form.pwd.value=='%s')", webPassword);
    loginPage += temp;
    loginPage += "{{if(form._nextFn.value=='ota'){window.open('/otaIndex')}}";
    loginPage += "{if(form._nextFn.value=='set'){window.open('/settingsIndex')}}}";
    loginPage += "else";
    loginPage += "{alert('Error Password or Username')}";
    loginPage += "}";
    loginPage += "</script>";
  }
  //=================================================================================
  void ComposeSettingsPage() {
   
  settingsPage  = styleSettings;
  settingsPage += "<!DOCTYPE html><html><body><h>Mavlink To Passthrough</h><form action='/settingsReturnIndex' ";  
  settingsPage += "autocomplete='on'> <center> <b><h3>MavToPassthrough Translator Setup</h3> </b></center> <style>text-align:left</style>";
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
  sprintf(temp, "<input type='radio' class='big' name='_fc_io' value='SD' %s> SD <br> ", set.fc_io3);
  settingsPage += temp;    
  settingsPage += "GCS IO: &nbsp";
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
  settingsPage += " <br> GCS SD: &nbsp";
  sprintf(temp, "<input type='radio' class='big' name='_gs_sd' value='OFF' %s> OFF  &nbsp &nbsp ", set.gs_sd0);
  settingsPage += temp; 
  sprintf(temp, "<input type='radio' class='big' name='_gs_sd' value='ON' %s> ON <br>", set.gs_sd1);
  settingsPage += temp;      
  settingsPage += "WiFi Mode: &nbsp &nbsp ";
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
  sprintf(temp, "Slave Connect To: <input type='text' name='_btSlaveConnectTo' value='%s' size='20' maxlength='20'> <br><br><center>", set.btSlaveConnectTo);
  settingsPage += temp;
  settingsPage += "<b><input type='submit' class='bold' value='Save & Reboot'> </b><br><br>";
  settingsPage += "<p><font size='1' color='black'><strong>";
//  settingsPage += pgm_name.substring(0, pgm_name.lastIndexOf('.'));
  settingsPage += pgm_name + ".  Compliled for "; 
  #if defined ESP32 
    settingsPage += "ESP32";
  #elif defined ESP8266
    settingsPage += "ESP8266";
  #endif   
  settingsPage += "</strong></p></center> </form> </body>";
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
    EEPROMReadString(140, set.btSlaveConnectTo);  // 140 thru 160 

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
      Debug.print("SlaveConnect To Name = "); Debug.println(set.btSlaveConnectTo);
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
      EEPROMWriteString(140, set.btSlaveConnectTo);   // 140 thru 160   
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
        Debug.print("Slave Connect To Name = "); Debug.println(set.btSlaveConnectTo); 
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
  String_char(set.btSlaveConnectTo, server.arg("_btSlaveConnectTo"));

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
        Debug.print("Slave Connect To Name = "); Debug.println(set.btSlaveConnectTo);  
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
    set.trmode3 = "checked";  
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
    //    String s = "<a href='/'> Update Success. Rebooting........  Back to login screen</a>";
    //    server.send(200, "text/html", s);
        Debug.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        delay(5000);
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
  strcpy(set.btSlaveConnectTo, BT_SlaveConnectTo);                

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
      Debug.print("Slave Connect To Name = "); Debug.println(set.btSlaveConnectTo); 
      Debug.println(); 
  #endif    
}

//=================================================================================
