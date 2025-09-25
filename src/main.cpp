//////////////////////////////////////////////
/////   GPS - LoRa - AVR128 - Firmware   /////
//////////////////////////////////////////////

//*** Libraries ***//
#include <Arduino.h>
#include <SPI.h>
#include <permaDefs.h>
#include <TimeLib.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SPIMemory.h>
#include <elapsedMillis.h>
#include <avr/sleep.h>

//*** Library Declarations ***//
elapsedMillis mTime;
TinyGPSPlus gps;
SPIFlash flash(FCS);

//*** Variables ***//

// General Variables //
uint16_t cnt = 0;                     // Total data points collected
uint16_t pingSecondCounter = 0;       // Counts Seconds Until Ping
uint16_t gpsSecondCounter = 0;        // Counts Seconds Until GPS Event
uint16_t pingCounterTarget;           // Target number of seconds before ping
uint16_t gpsCounterTarget;            // Target number of seconds before GPS wake

// Flash Addresses //
uint32_t wAdd = 0;                    // Last written to flash address
uint32_t rAdd = 0;                    // Last read from flash address

// GPS Control Variables //
int gpsFrequency = 60;                // GPS Frequency in minutes
int gpsTimeout = 120;                 // GPS Timeout in seconds
int gpsHdop = 5;                      // GPS HDOP Parameter

// GPS Storage Variables //
float lat = 0.0;                      // Storing last known Latitude
float lng = 0.0;                      // Storing last known Longitude

// Radio Variables //
int radioFrequency = 1;               // Frequency of Pings in minutes
bool mortality = false;               // Mortality status flag

//*** Functions ***//

// Function 1: Record and Store GPS data
void recGPS() {
    mTime = 0;
    digitalWrite(GPS_PIN, HIGH);
    Serial.println("GPS acquiring fix...");
    
    while (mTime <= (unsigned)gpsTimeout * 1000) {
        while (Serial1.available()) {
            if (gps.encode(Serial1.read())) {
                if (gps.location.isValid()) {
                    Serial.print("Location Age: ");
                    Serial.println(gps.location.age());
                    Serial.print("Satellites: ");
                    Serial.println(gps.satellites.value());
                    Serial.print("HDOP: ");
                    Serial.println(gps.hdop.hdop());
                }
            }
        }
        
        // Check if we have a good fix
        if (gps.hdop.hdop() < (double)gpsHdop && 
            gps.location.age() < 1000 && 
            gps.time.age() < 1000 && 
            mTime > 3000) {
            break;
        }
    }
    
    digitalWrite(GPS_PIN, LOW);
    
    // Create data structure
    data dat;
    
    // Store GPS data if valid
    if (gps.location.age() < 60000 && gps.location.isValid()) {
        lat = gps.location.lat();
        lng = gps.location.lng();
        dat.lat = lat;
        dat.lng = lng;
        Serial.print("GPS Fix: ");
        Serial.print(lat, 6);
        Serial.print(", ");
        Serial.println(lng, 6);
    } else {
        dat.lat = 0;
        dat.lng = 0;
        Serial.println("GPS fix failed - storing zeros");
    }
    
    // Set timestamp
    if (gps.time.isValid() && gps.date.isValid()) {
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 
               gps.date.day(), gps.date.month(), gps.date.year());
        dat.datetime = (uint32_t)now();
    } else {
        dat.datetime = (uint32_t)now(); // Use system time if GPS time invalid
    }
    
    dat.locktime = mTime / 1000;
    dat.hdop = (byte)gps.hdop.hdop();
    dat.act = false; // No activity mode in simplified version
    
    Serial.print("GPS lock time: ");
    Serial.print(dat.locktime);
    Serial.println(" seconds");
    
    // Store to flash memory
    if (flash.powerUp()) {
        Serial.println("Flash powered up");
        delay(100);
        
        wAdd = flash.getAddress(sizeof(dat));
        Serial.print("Writing to address: ");
        Serial.println(wAdd);
        
        if (flash.writeAnything(wAdd, dat)) {
            Serial.println("Data written to flash successfully");
            cnt++;
            
            Serial.println("Stored GPS data:");
            Serial.print("Datetime: "); Serial.println(dat.datetime);
            Serial.print("Lat: "); Serial.println(dat.lat, 6);
            Serial.print("Lng: "); Serial.println(dat.lng, 6);
            Serial.print("Lock time: "); Serial.println(dat.locktime);
            Serial.print("HDOP: "); Serial.println(dat.hdop);
        } else {
            Serial.println("Flash write failed");
            Serial.println(flash.error(VERBOSE));
        }
    } else {
        Serial.println("Flash power up failed");
    }
    
    flash.powerDown();
}

// Function 2: Send ping with GPS data
void sendPing(float x, float y, uint16_t tag_id, uint16_t count, uint8_t devType, bool mortality_flag) {
    ping p;
    
    p.devtyp = devType;
    p.ta = tag_id;
    p.la = x;
    p.ln = y;
    p.cnt = count;
    p.mortality = mortality_flag;
    
    Serial.println("Sending ping...");
    Serial.print("Lat: "); Serial.println(p.la, 6);
    Serial.print("Lng: "); Serial.println(p.ln, 6);
    Serial.print("Count: "); Serial.println(p.cnt);
    
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&p, sizeof(p));
    LoRa.endPacket();
    LoRa.sleep();
    
    Serial.println("Ping sent");
}

// Function 3: Read stored data from flash and send via LoRa
void readAndSendData() {
    data dat;
    
    if (flash.powerUp()) {
        if (flash.readAnything(rAdd, dat)) {
            Serial.println("Read successful from flash:");
            Serial.print("Datetime: "); Serial.println(dat.datetime);
            Serial.print("Lat: "); Serial.println(dat.lat, 6);
            Serial.print("Lng: "); Serial.println(dat.lng, 6);
            Serial.print("Lock time: "); Serial.println(dat.locktime);
            Serial.print("HDOP: "); Serial.println(dat.hdop);
            
            // Send data via LoRa
            LoRa.idle();
            LoRa.beginPacket();
            LoRa.write((uint8_t*)&dat, sizeof(dat));
            LoRa.endPacket();
            LoRa.sleep();
            
            Serial.println("Data sent via LoRa");
        } else {
            Serial.println("Flash read failed");
        }
    }
    flash.powerDown();
}

// Function 4: Listen for incoming commands
void receiveCommands(unsigned int rcv_time) {
    Serial.println("Listening for commands...");
    LoRa.idle();
    mTime = 0;
    
    while (mTime <= rcv_time) {
        int packetSize = LoRa.parsePacket();
        
        if (packetSize > 0) {
            Serial.print("Received packet size: ");
            Serial.println(packetSize);
            
            // Handle data request (3 bytes) - Send all stored data
            if (packetSize == 3) {
                struct request {
                    uint16_t tag;
                    byte request;
                } r;
                
                while (LoRa.available()) {
                    LoRa.readBytes((uint8_t*)&r, sizeof(r));
                }
                
                Serial.print("Request from tag: "); Serial.println(r.tag);
                Serial.print("Request type: "); Serial.println(r.request);
                
                // Send all data if request is for our tag
                if (r.tag == tag && r.request == (byte)82) {
                    Serial.println("Sending all stored data...");
                    
                    uint32_t addr = 0;
                    while (addr <= wAdd) {
                        data dat;
                        
                        if (flash.powerUp()) {
                            if (flash.readAnything(addr, dat)) {
                                LoRa.idle();
                                LoRa.beginPacket();
                                LoRa.write((uint8_t*)&dat, sizeof(dat));
                                LoRa.endPacket();
                                LoRa.sleep();
                                
                                delay(50); // Small delay between packets
                            }
                            flash.powerDown();
                        }
                        addr += sizeof(data);
                    }
                    
                    // Send completion response
                    delay(1000);
                    struct resp {
                        uint16_t tag;
                        byte res;
                    } response;
                    
                    response.res = (byte)68; // Data transfer complete
                    response.tag = tag;
                    
                    LoRa.idle();
                    LoRa.beginPacket();
                    LoRa.write((uint8_t*)&response, sizeof(response));
                    LoRa.endPacket();
                    LoRa.sleep();
                    
                    Serial.println("Data transfer complete");
                }
            }
            
            // Handle settings update (7 bytes)
            else if (packetSize == 7) {
                setttings settings;
                
                if (LoRa.available()) {
                    LoRa.readBytes((uint8_t*)&settings, sizeof(settings));
                    
                    // Update configuration
                    gpsFrequency = settings.gpsFrq;
                    gpsTimeout = settings.gpsTout;
                    gpsHdop = settings.hdop;
                    radioFrequency = settings.radioFrq;
                    
                    Serial.println("Settings updated:");
                    Serial.print("GPS Freq: "); Serial.println(gpsFrequency);
                    Serial.print("GPS Timeout: "); Serial.println(gpsTimeout);
                    Serial.print("GPS HDOP: "); Serial.println(gpsHdop);
                    Serial.print("Radio Freq: "); Serial.println(radioFrequency);
                    
                    // Send acknowledgment
                    resPing response;
                    response.resp = (byte)83; // Settings received confirmation
                    response.tag = tag;
                    
                    LoRa.idle();
                    LoRa.beginPacket();
                    LoRa.write((uint8_t*)&response, sizeof(response));
                    LoRa.endPacket();
                    LoRa.sleep();
                    
                    // Update timing targets
                    pingCounterTarget = radioFrequency * 60;
                    gpsCounterTarget = gpsFrequency * 60;
                }
            }
        }
    }
    
    LoRa.sleep();
    Serial.println("Receive window closed");
}

// Function 4: RTC Timer Interrupt Service Routine
void RTC_init(void) {
    while (RTC.STATUS > 0) {
        ; // Wait for all registers to be synchronized
    }
    
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    // 32.768kHz Internal Oscillator
    RTC.PITINTCTRL = RTC_PI_bm;           // PIT Interrupt enabled
    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm; // Enable PIT counter (1Hz)
}

// Function 5: Initialize RTC for timing
ISR(RTC_PIT_vect) {
    RTC.PITINTFLAGS = RTC_PI_bm;          // Clear interrupt flag
    pingSecondCounter++;
    gpsSecondCounter++;
}


void initGPS() {
    digitalWrite(GPS_PIN, HIGH);
    delay(1000); // Allow GPS to power up
    
    Serial.println("Acquiring initial GPS fix...");
    
    // Wait for first valid fix
    while (!gps.location.isValid() || !gps.time.isValid()) {
        while (Serial1.available() > 0) {
            if (gps.encode(Serial1.read())) {
                if (gps.location.isValid()) {
                    Serial.print("Initial GPS fix acquired: ");
                    Serial.print(gps.location.lat(), 6);
                    Serial.print(", ");
                    Serial.println(gps.location.lng(), 6);
                    
                    lat = gps.location.lat();
                    lng = gps.location.lng();
                    
                    if (gps.time.isValid() && gps.date.isValid()) {
                        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
                               gps.date.day(), gps.date.month(), gps.date.year());
                        Serial.println("System time synchronized with GPS");
                    }
                }
            }
        }
    }
    
    digitalWrite(GPS_PIN, LOW);
}

//*** Setup Function ***//
void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);
    
    // Initialize pins
    pinMode(GPS_PIN, OUTPUT);
    digitalWrite(GPS_PIN, LOW);
    
    Serial.println("Initializing GPS-LoRa firmware...");
    
    // Initialize RTC timer
    RTC_init();
    
    // Initialize LoRa
    LoRa.setPins(LCS, LRST, LDIO0);
    if (!LoRa.begin(867E6)) {
        Serial.println("LoRa initialization failed!");
        while (1);
    }
    
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setSpreadingFactor(12);
    LoRa.sleep();
    
    // Initialize Flash Memory
    if (flash.powerUp()) {
        Serial.println("Flash powered up");
    }
    
    if (!flash.begin()) {
        Serial.println("Flash initialization failed!");
        Serial.println(flash.error(VERBOSE));
        while (1);
    }
    
    Serial.print("Flash Manufacturer ID: ");
    Serial.println(flash.getManID());
    
    // Get current write address (where to continue writing)
    wAdd = flash.getAddress(sizeof(data));
    rAdd = 0; // Start reading from beginning
    
    if (flash.powerDown()) {
        Serial.println("Flash powered down");
    }
    
    Serial.println("Flash memory initialized");
    
    // Get initial GPS fix
    initGPS();
    
    // Set timing targets
    pingCounterTarget = radioFrequency * 60;
    gpsCounterTarget = gpsFrequency * 60;
    
    Serial.println("System ready - entering sleep mode");
    Serial.flush();
    
    // Configure sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
}

//*** Main Loop ***//
void loop() {
    Serial.begin(115200);
    Serial.println("System wake up");
    
    bool gpsEvent = false;
    bool pingEvent = false;
    
    // Check what woke us up
    if (gpsSecondCounter >= gpsCounterTarget) {
        gpsEvent = true;
        gpsSecondCounter = 0;
    }
    
    if (pingSecondCounter >= pingCounterTarget) {
        pingEvent = true;
        pingSecondCounter = 0;
    }
    
    // Handle GPS event
    if (gpsEvent) {
        Serial.println("GPS event triggered");
        Serial1.begin(9600);
        SPI.begin();
        
        recGPS();
        
        // Adjust next GPS timing based on actual GPS acquisition time
        if ((mTime / 1000) < (gpsFrequency * 60)) {
            gpsCounterTarget = (gpsFrequency * 60) - (mTime / 1000);
        } else {
            gpsCounterTarget = gpsFrequency * 60;
        }
        
        Serial1.end();
        SPI.end();
    }
    
    // Handle ping event
    if (pingEvent) {
        Serial.println("Ping event triggered");
        Serial1.begin(9600);
        SPI.begin();
        
        sendPing(lat, lng, tag, cnt, devType, mortality);
        receiveCommands(4000); // 4 second receive window
        
        // Adjust next ping timing
        if ((mTime / 1000) < (radioFrequency * 60)) {
            pingCounterTarget = (radioFrequency * 60) - (mTime / 1000);
        } else {
            pingCounterTarget = radioFrequency * 60;
        }
        
        Serial1.end();
        SPI.end();
    }
    
    Serial.print("Next GPS in: "); Serial.print(gpsCounterTarget); Serial.println("s");
    Serial.print("Next ping in: "); Serial.print(pingCounterTarget); Serial.println("s");
    
    Serial.println("Returning to sleep");
    Serial.flush();
    Serial.end();
    
    delay(100);
    sleep_cpu();
}