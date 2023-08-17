
// see bottom of tab for ESP32 special case
uint32_t GetBaud(uint8_t rxPin) {
  
  pinMode(rxPin, INPUT);       
  digitalWrite (rxPin, HIGH); // pull up enabled for noise reduction ?
  
  uint32_t gb_baud = GetConsistent(rxPin);
  while (GetConsistent(rxPin) == 0) {
    if(ftGetBaud) {
      ftGetBaud = false;
      Debug.println("Waiting for telemetry"); 
      OledDisplayln("Waiting for telem");
    }
    Debug.print("."); 
    delay(4000);
    gb_baud = GetConsistent(rxPin);
  } 
  if (!ftGetBaud) {
    Debug.println();
  }

  Debug.print("Telem found at "); Debug.print(gb_baud);  Debug.println(" b/s");
  OledDisplayln("Telem found at " + String(gb_baud));

  return(gb_baud);
}

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

uint32_t SenseUart(uint8_t  rxPin) {

uint32_t pw = 999999;  //  Pulse width in uS
uint32_t min_pw = 999999;
uint32_t su_baud = 0;



#if defined Debug_All || defined Debug_Baud
//  Debug.print("rxPin ");  Debug.println(rxPin);
#endif  


 while(digitalRead(rxPin) == 1){} // wait for low bit to start
 
  for (int i = 0; i < 10; i++) {
    pw = pulseIn(rxPin,LOW);               // Returns the length of the pulse in microseconds
    if (pw !=0) {
      min_pw = (pw < min_pw) ? pw : min_pw;  // Choose the lowest
    }
  }
 
  #if defined Debug_All || defined Debug_Baud
 //   Debug.print("pw="); Debug.print(pw); Debug.print("  min_pw="); Debug.println(min_pw);
  #endif

  switch(min_pw) {   
    case 3 ... 11:     
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
     su_baud = 0;            
 }

 return su_baud;
} 
/* ESP32 SPECIAL CASE
 *koobest commented on Apr 22 • 

You should enable the register UART_AUTOBAUD_EN, and then, the sender needs to send a binary sequence such as 01010101. The receiver will detect the shortest pulse. The minimum duration is stored in UART_LOWPULSE_REG and UART_HIGHPULSE_REG.

baud_rate = 80000000/min(UART_LOWPULSE_REG, UART_HIGHPULSE_REG);
Here is an example:

uint32_t uart_baud_detect(uart_port_t uart_num)
{
    int low_period = 0;
    int high_period = 0;
    uint32_t intena_reg = UART[uart_num]->int_ena.val;
    //Disable the interruput.
    UART[uart_num]->int_ena.val = 0;
    UART[uart_num]->int_clr.val = ~0;
    //Filter
    UART[uart_num]->auto_baud.glitch_filt = 4;
    //Clear the previous result
    UART[uart_num]->auto_baud.en = 0;
    UART[uart_num]->auto_baud.en = 1;
    while(UART[uart_num]->rxd_cnt.edge_cnt < 100) {
        ets_delay_us(10);
    }
    low_period = UART[uart_num]->lowpulse.min_cnt;
    high_period = UART[uart_num]->highpulse.min_cnt;
    // disable the baudrate detection
    UART[uart_num]->auto_baud.en = 0;
    //Reset the fifo;
    uart_reset_rx_fifo(uart_num);
    UART[uart_num]->int_ena.val = intena_reg;
    //Set the clock divider reg
    //UART[uart_num]->clk_div.div_int = (low_period > high_period) ? high_period : low_period;

    //Return the divider. baud = APB / divider;
    return (low_period > high_period) ? high_period : low_period;;
}

 */
