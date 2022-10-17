//////////////////////////////////////////////
/////   GPS - LoRa - AVR128 - Firmware   /////
//////////////////////////////////////////////

//*** Libraries ***//
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <permaDefs.h>
#include <time.h>
#include <TimeLib.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SPIMemory.h>
#include <elapsedMillis.h>
#include <avr/sleep.h>
#include <MCP79412RTC.h>
#include <DFRobot_LIS2DW12.h>

//*** Library Declarations ***//
elapsedMillis mTime;
TinyGPSPlus gps;
SPIFlash flash(FCS);
MCP79412RTC rtc(false);
DFRobot_LIS2DW12_I2C acce;

//#####################################################################################################//

//*** Variables ***//

// General Variables //
uint16_t cnt;                     //** Total data points collected

// Flash Addresses // 
uint32_t wAdd = 0;                //** Last written to flash address
uint32_t rAdd = 0;                //** Last read from flash address

// GPS Control Variables //
int gpsFrequency = 60;            //(Minutes)>>> GPS Frequency in minutes *** USER CONFIG ***
int gpsTimeout = 60;              //(Seconds)>>> GPS Timesout after 'x' seconds *** USER CONFIG ***
int gpsHdop = 5;                  //(N/A)>>> GPS HODP Parameter *** USER CONFIG ***

// GPS Storage Variables //
float lat;                        //** Storing last known Latitude
float lng;                        //** Storing last known Longitude

// Accelerometer Variables //
int act_treshold = 120;           // Activity threshold 0-255 *** USER CONFIG *** 
int act_gpsFrequency = 15;        // Activity mode GPS Frequency *** USER CONFIG ***
int act_duration = 60;            // Activity mode duration in minutes *** USER CONFIG ***
time_t act_start;                 // activity mode start time
time_t act_end;                   // activity mode end time

// Time Variables // 
time_t strtTime;
time_t act_end_time;
time_t next_gps_wakeup;
time_t next_ping_wakeup;
time_t schedule_user_provided; 

// Booleans and Flags //
bool act_int_triggered = false;   //** Flag to identify activity trigger
bool rtc_int_triggered = false;   //** Flag to identify rtc timer trigger 
bool mortality = false;           //** Flag to identify mortality status
bool act_mode = false;            //** Activity mode status - is device in activity mode or not
bool activity_enabled = false;    //>>> Enables or diables activity mode
bool wipe_memory = false;         //>>> Wipe memory during reset/Activation
bool scheduled = false;           //>>> Flag to enable scheduling mode

// Radio Variables //
int radioFrequency = 1;           //Frequency of Pings in minutes *** USER CONFIG ***
int rcv_duration = 5;             // Receive Window Duration in seconds *** USER CONFIG ***
time_t scheduled_wake_start;      // Last Schedule Start Time 
time_t scheduled_wake_end;        // Last Schedule End Time
int sch_duration = 5;             // Schedule Window Duration in mins *** USER CONFIG ***
int sch_rpt_duration = 10;        // Schedule repeat time in days *** USER CONFIG ***

//#####################################################################################################//

//*** Functions ***//

// Function 1 : Record and Store GPS data
void recGPS(){
  mTime = 0;
  digitalWrite(GPS_PIN, HIGH);
  Serial.println(gpsTimeout*1000);
  while (mTime <= (unsigned)gpsTimeout*1000)
  {
    while (Serial1.available())
    {
      if (!gps.encode(Serial1.read()))
      {
        if (!gps.location.isValid())
        {
          Serial.println(F("Acquiring"));
        }else{
          Serial.println(gps.location.isUpdated());
          Serial.print(F("Location Age:"));
          Serial.println(gps.location.age());
          Serial.print(F("Time Age:"));
          Serial.println(gps.time.age());
          Serial.print(F("Date Age:"));
          Serial.println(gps.date.age());
          Serial.print(F("Satellites:"));
          Serial.println(gps.satellites.value());
          Serial.print(F("HDOP:"));
          Serial.println(gps.hdop.hdop());
        }       
      }      
    }
    if (gps.hdop.hdop() < (double)gpsHdop && gps.location.age() < 1000 && gps.time.age() < 1000 && mTime > 3000)
    {
      break;
    }  
  }   

  digitalWrite(GPS_PIN, LOW);
  data dat;

  if (gps.location.age() < 60000)
  {
    //pack data into struct
    lat = gps.location.lat();
    lng = gps.location.lng();
    dat.lat = gps.location.lat();
    dat.lng = gps.location.lng();
  }else{
    // pack data into struct with lat long = 0
    dat.lat = 0;
    dat.lng = 0;
  }
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    dat.datetime = (uint32_t)now();
    dat.locktime = mTime/1000;
    dat.hdop = gps.hdop.hdop();
    
    Serial.println(dat.datetime);
    Serial.println(dat.lat);
    Serial.println(dat.lng);
    Serial.println(dat.locktime);
    Serial.println(dat.hdop);


  if (flash.powerUp())
  {
    Serial.println(F("Powered Up"));
    delay(500);
    Serial.println((int)sizeof(dat));
    wAdd = flash.getAddress(sizeof(dat));
    Serial.println(wAdd);
    if (flash.writeAnything(wAdd, dat))
    {
      Serial.println(F("Write Successful"));
      cnt = cnt + 1;
    }else
    {
      Serial.println(F("Write Failed"));
      Serial.println(flash.error(VERBOSE));
    }     
  }else
  {
    Serial.println(F("Power Up Failed"));
  }   
  flash.powerDown();

}

// Function 1-a : Secondary GPS Function used only in activation sequence
void acqGPS(){
  digitalWrite(GPS_PIN, HIGH);
      do{ 
        while (Serial1.available() > 0)
        {
          if (gps.encode(Serial1.read()))
          {
            if (!gps.location.isValid())
            {
              Serial.println(F("Not Valid"));
            }else{
              Serial.println(gps.location.isUpdated());
              Serial.print("Location Age:");
              Serial.println(gps.location.age());
              Serial.print("Time Age:");
              Serial.println(gps.time.age());
              Serial.print("Date Age:");
              Serial.println(gps.date.age());
              Serial.print("Satellites:");
              Serial.println(gps.satellites.value());
              Serial.print("HDOP:");
              Serial.println(gps.hdop.hdop());
            }
          }
        }
      }while(!gps.location.isValid());
    if (gps.location.age() < 60000)
    {
      //pack data into struct
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
    if (gps.time.isValid())
    {
      setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
      time_t n = now();
      strtTime = n;
      Serial.print(F("START TIME")); Serial.println(strtTime);
    }    
    digitalWrite(GPS_PIN, LOW);
}

// Function 2 : Read data from flash and send it through radio
void read_send(){ 
  data dat;

  if (flash.powerUp())
  {
    if (flash.readAnything(rAdd, dat))
    {
      // dat.id = tag;
      Serial.println(F("Read Successful"));
      Serial.println(dat.datetime);
      Serial.println(dat.hdop);
      Serial.println(dat.lat);
      Serial.println(dat.lng);
      Serial.println(dat.locktime);
      Serial.println(dat.act);
    }else
    {
      Serial.println(F("Read Failed"));
    }    
  }
      LoRa.idle();
      LoRa.beginPacket();
      LoRa.write((uint8_t*)&dat, sizeof(dat));
      LoRa.endPacket();
      LoRa.sleep();
}

// Function 3 : RTC Timer Interrupt Service Routine
void risr(){
  rtc_int_triggered = true;
  detachInterrupt(RINT);
  detachInterrupt(AINT1);
}

// Function 4 : Activity Detection Interrupt Service Routine
void aisr(){
  act_int_triggered = true;
  mortality = false;
  detachInterrupt(AINT1);
}

// Function 5 : Pinger - Takes several inputs and sends ping
void Ping(float x, float y, uint16_t a, uint16_t c, byte d, bool m){

  ping p;

  p.devtyp = d;
  p.ta = a;
  p.la = x;
  p.ln = y;
  p.cnt = c;
  p.mortality = m;

  Serial.print(F("Size")); Serial.println((int)sizeof(p));
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&p, sizeof(p));
  LoRa.endPacket();
  LoRa.sleep();
  
}

// Function 6 : Look for and process inbound transmission right after ping.
void receive(unsigned int rcv_time){
  Serial.println(F("Receiving"));
  LoRa.idle();
  mTime = 0;
  int x = 0;
  do
  {  
    x = LoRa.parsePacket();
    if (x)
    {
      Serial.println(x);
    }
    
    
    if (x == 3)
    { 
      Serial.print(F("int : ")); Serial.println(x);
      struct request{
      uint16_t tag;
      byte request;
      }r;
      while (LoRa.available())
      {
        Serial.println(F("Reading in"));
        LoRa.readBytes((uint8_t*)&r, sizeof(r));
      }
      Serial.println(r.tag);
      Serial.println(r.request);
      if (r.tag == tag && r.request == (byte)82)
      {
        
        do
        {
          Serial.println("Init Stream");
          read_send();
          rAdd = rAdd + 16;
          Serial.println(rAdd);
        
        } while (rAdd <= wAdd);

        delay(1000);
        struct resp{
        uint16_t tag;
        byte res;
        }r;
        r.res = (byte) 68;
        r.tag = tag;

        LoRa.idle();
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&r, sizeof(r));
        LoRa.endPacket();
        LoRa.sleep();
      }            
    }

    if (x == 21)
    {
      setttings set;

      while (LoRa.available())
      {
        Serial.println(F("Incoming Settings"));
        LoRa.readBytes((uint8_t*)&set, sizeof(set));
      }

      schedule_user_provided = set.pingTime;
      act_treshold = set.act_trsh;
      act_gpsFrequency = set.act_gps_frq;
      act_duration = set.act_duration;
      gpsFrequency = set.gpsFrq;
      gpsTimeout = set.gpsTout;
      gpsHdop = set.hdop;
      radioFrequency = set.radioFrq;
      rcv_duration = set.rcv_dur;
      sch_duration = set.sch_dur;
      sch_rpt_duration = set.sch_rpt_dur;
      activity_enabled = set.act_enabled;
      scheduled = set.sch_enabled;

      Serial.println(set.pingTime);
      Serial.println(set.act_trsh);
      Serial.println(set.act_gps_frq);
      Serial.println(set.act_duration);
      Serial.println(set.gpsFrq);
      Serial.println(set.gpsTout);
      Serial.println(set.hdop);
      Serial.println(set.radioFrq);
      Serial.println(set.rcv_dur);
      Serial.println(set.sch_dur);
      Serial.println(set.sch_rpt_dur);
      Serial.println(set.act_enabled);
      Serial.println(set.sch_enabled);
      delay(100);

      if (activity_enabled == true)
      {
        attachInterrupt(AINT1, aisr, CHANGE);        
        Serial.print(F("ACTIVITY On"));
      }else{
        detachInterrupt(AINT1);
        if (act_mode == true)
        {
          act_mode = false;
        }        
        Serial.print(F("ACTIVITY OFF"));
      }

      if (scheduled == true)
      {
        digitalWrite(RTC_PIN, HIGH);
        rtc.alarmPolarity(HIGH);
        rtc.setAlarm(1, schedule_user_provided);
        rtc.enableAlarm(1, ALM_MATCH_DATETIME);
        scheduled_wake_start = schedule_user_provided;
        scheduled_wake_end = scheduled_wake_start + sch_duration*60;
      }
      resPing r;
        r.resp = (byte)83;
        r.tag = tag;

        LoRa.idle();
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&r, sizeof(r));
        LoRa.endPacket();
        LoRa.sleep();
    }
  }while(mTime <= rcv_time);
  LoRa.sleep();
  delay(50);
}

// Function 7 : First activation ping - process active/sleep/wipe modes
void activationPing(){

  reqPing px1;
  resPing rs1;

  int x;

  px1.tag = tag;
  px1.request = (byte)73;


  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&px1, sizeof(px1));
  LoRa.endPacket();

  mTime = 0;
  while (mTime < 30000)
  {
    x = LoRa.parsePacket();
    if (x)
    {
      Serial.println(F("Incoming"));
      Serial.println(x);
    }
    
    if (x == 3)
    {
      while (LoRa.available())
      {
        Serial.println(F("Message"));
        LoRa.readBytes((uint8_t*)&rs1, sizeof(rs1));
      }
      break;      
    }     
  } 
  LoRa.sleep();

  if (rs1.tag == tag && rs1.resp == (byte)70)
  {
    Serial.print(F("System Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
    Serial.print(F("System Initialising"));
     /// Begin GPS and Acquire Lock ////
    acqGPS();
    
    wipe_memory = true;

    px1.request = (byte)106;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();
  }

  if (rs1.tag == tag && rs1.resp == (byte)71)
  {
    Serial.print(F("System Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
     /// Begin GPS and Acquire Lock ////
    acqGPS();

    wipe_memory = false;

    px1.request = (byte)105;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();

  }

  if (rs1.tag == tag && rs1.resp == (byte)115){
        Serial.print(F("Indefinite Sleep"));
        EEPROM.put(1, false);
        Serial.println(EEPROM.read(1));
        delay(50);  
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();      
        sleep_cpu();
  }

  Serial.println(EEPROM.read(1));

  if (EEPROM.read(1) == false){
    Serial.println(F("SLEEP1"));
    delay(50);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
  }else{
    Serial.println(F("Reset"));
    Serial.print(F("System Re - Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
     /// Begin GPS and Acquire Lock ////
    acqGPS();

    wipe_memory = false;

    px1.request = (byte)105;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();
  }
    
}

// Function 8 : Mortality Check
void mortalityCheck(bool m){
  if (m == true)
  {
    if (act_end_time > act_end_time + 86400)
    {
      mortality = true; 
    }    
  }  
}

// Setup : Initialize Device
void setup(){
  Serial.begin(115200);
  pinMode(RTC_PIN, OUTPUT);
  Wire.swapModule(&TWI1);
  Wire.usePullups();
  Wire.begin();
  //***********************************************//
  if(!acce.begin()){
     Serial.println("Communication failed, check the connection and I2C address setting when using I2C communication.");
     delay(1000);
  }else{
  Serial.print("chip id : ");
  Serial.println(acce.getID(),HEX);
  }
  acce.softReset();
  acce.setRange(DFRobot_LIS2DW12::e4_g);
  acce.setFilterPath(DFRobot_LIS2DW12::eLPF);
  acce.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
  acce.setWakeUpDur(/*dur = */2);
  acce.setWakeUpThreshold(/*threshold = */0.3);
  acce.setPowerMode(DFRobot_LIS2DW12::eContLowPwrLowNoise1_12bit);
  acce.setActMode(DFRobot_LIS2DW12::eDetectAct);
  acce.setInt1Event(DFRobot_LIS2DW12::eWakeUp);
  acce.setDataRate(DFRobot_LIS2DW12::eRate_200hz);
  if (activity_enabled)
  {
    attachInterrupt(AINT1, aisr, CHANGE);
  }else{
    detachInterrupt(AINT1);
  }  
  //***********************************************//

  if (wipe_memory == true)
  {
    Serial.println(F("WIPING FLASH"));
    if(flash.eraseChip()){
    Serial.println(F("Memory Wiped"));  
    }else
    {
      Serial.println(flash.error(VERBOSE));
    }
  }else{
    rAdd = flash.getAddress(16);
    wAdd = flash.getAddress(16);
  }    
  if(flash.powerDown()){
    Serial.println("Powered Down");
    digitalWrite(1, HIGH);
  }else{
    Serial.println(flash.error(VERBOSE));
  }

  //***********************************************//
  
  digitalWrite(RTC_PIN, HIGH);
  rtc.set(strtTime);
  Serial.println(rtc.get());
  delay(100);
  rtc.alarmPolarity(HIGH);
  rtc.setAlarm(0, strtTime + 60*gpsFrequency);
  if (scheduled == true)
  {
    rtc.setAlarm(1, scheduled_wake_start);
  }else{
    rtc.setAlarm(1, strtTime + 60*radioFrequency);
    next_ping_wakeup = strtTime + 60*radioFrequency;
  }
  
  next_gps_wakeup = strtTime + 60*gpsFrequency;
  Serial.println(next_gps_wakeup);
  rtc.enableAlarm(0, ALM_MATCH_DATETIME);
  rtc.enableAlarm(1, ALM_MATCH_DATETIME);
  digitalWrite(RTC_PIN, LOW);
  delay(100);
  //***********************************************//
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
}

// Loop : Main Loop
void loop(){
    
}

