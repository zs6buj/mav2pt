

uint32_t Get_Volt_Average1(uint16_t mV)  {
  
  if (bat1.avg_mV < 1) bat1.avg_mV = mV;  // Initialise first time

  bat1.avg_mV = (bat1.avg_mV * 0.9) + (mV * 0.1);  // moving average
  Accum_Volts1(mV);  
  return bat1.avg_mV;
}
  
uint32_t Get_Current_Average1(uint16_t dA)  {

  if (bat1.avg_dA < 1) bat1.avg_dA = dA;  // Initialise first time

  bat1.avg_dA = (bat1.avg_dA * 0.9) + (dA * 0.1);  // moving average

  Accum_mAh1(dA);  
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
    
  float hrs = (float)(period / 3600000.0f);

  bat1.mAh = dAs * 100 * hrs;  //  dA to mA    Tiny mAh consumed this tiny period di/dt
  bat1.tot_mAh += bat1.mAh;   //   Add them all in
}

float Total_mAh1() {
  return bat1.tot_mAh;
}

float Total_mWh1() {                                     // Total energy consumed bat1
  return bat1.tot_mAh * (bat1.tot_volts / bat1.samples);
}
//***********************************************************
uint32_t Get_Volt_Average2(uint16_t mV)  {
  
  if (bat2.avg_mV == 0) bat2.avg_mV = mV;  // Initialise first time

  bat2.avg_mV = (bat2.avg_mV * 0.9) + (mV * 0.1);  // moving average
  Accum_Volts2(mV);  
  return bat2.avg_mV;
}
  
uint32_t Get_Current_Average2(uint16_t dA)  {

  if (bat2.avg_dA == 0) bat2.avg_dA = dA;  // Initialise first time

  bat2.avg_dA = (bat2.avg_dA * 0.9) + (dA * 0.1);  // moving average

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
    
  float hrs = (float)(period / 3600000.0f);

  bat2.mAh = dAs * 100 * hrs;  //  dA to mA    Tiny mAh consumed this tiny period di/dt
  bat2.tot_mAh += bat2.mAh;   //   Add them all in
}

float Total_mAh2() {
  return bat2.tot_mAh;
}

float Total_mWh2() {                                     // Total energy consumed bat1
  return bat2.tot_mAh * (bat2.tot_volts / bat2.samples);
}
