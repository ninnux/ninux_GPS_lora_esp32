//Public Licenze GPL 3.0


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  600        /* Time ESP32 will go to sleep (in seconds) */


#include <Wire.h>
#include <SSD1306Wire.h>

#include "WiFi.h""
//#include "images.h"

#include <TinyGPS++.h>
//#include "driver/rtc_io.h"

TinyGPSPlus gps;

static const int RXPin = 12, TXPin = 13;
static const uint32_t GPSBaud = 9600;
 
#define SDA      4
#define SCL     15
#define RSTOLED  16   //RST must be set by software
#define LED  25

SSD1306Wire display(0x3c, SDA, SCL, RSTOLED);

#include "config.h"
//make a confi.h file with following lines:
//static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}


void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


char mydata[40];
unsigned char mydata2[40];
static osjob_t sendjob;


// Pin mapping

const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

char latitude[10];
char longitude[10];
char howmanybytes[10];

unsigned int GPSbytes=0;

void displayInfoOnDisplay(){
  if (gps.location.isValid())
  {
     display.drawString(0, 0, "LAT:");
     sprintf(latitude,"%f",gps.location.lat());
     display.drawString(30, 0, latitude);
     sprintf(longitude,"%f",gps.location.lng());
     display.drawString(0, 10, "LON:");
     display.drawString(30, 10, longitude);
     sprintf(howmanybytes,"%d",GPSbytes);
     display.drawString(00, 40, "Valid bytes:");
     display.drawString(70, 40, howmanybytes);
     display.display();
     sleep(4);
     display.clear();
 
  }
  else
  {
     display.drawString(0, 0, "LAT:");
     display.drawString(30, 0, "INVALID");
     sprintf(latitude,"INVALID");
     display.drawString(0, 10, "LON:");
     display.drawString(30, 10, "INVALID");
     sprintf(longitude,"INVALID");
     sprintf(howmanybytes,"%d",GPSbytes);
     display.drawString(00, 40, "Unvalid bytes:");
     display.drawString(70, 40, howmanybytes);
     display.display();
     sleep(1);
     display.clear();
  
  }


  
}


void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}


void GPSStatus(){

     //while (!gps.location.isValid()){
        while (Serial2.available() > 0 ){
          if (gps.encode(Serial2.read()) and !gps.location.isValid()){
            //displayInfo();
            displayInfoOnDisplay();
            GPSbytes++;
          }
        }
        if (millis() > 5000 && gps.charsProcessed() < 10)
        {
          Serial.println(F("No GPS detected: check wiring."));
          while(true);
        }
        
     //} 
     
      
      //displayInfo();
      if(gps.location.isValid()){
        display.drawString(0, 0, "GPS fix");
        display.display();
        sleep(2);
        display.clear();
        displayInfoOnDisplay();
      }
  
}




void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            display.drawString(0, 0, "JOINED");
            display.display();
            sleep(5);
            display.clear();
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            display.drawString(0, 0, "Message sent... go to bed");
            display.display();
            sleep(10);
            display.clear();
            //sleep(30);
            esp_deep_sleep_start();
            //rtc_gpio_hold_en(GPIO_NUM_2);
            //rtc_gpio_hold_en(GPIO_NUM_21);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){



  
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        display.drawString(0, 0, "Sending message.");
        display.display();
        display.clear();
        memset(mydata,'\0',sizeof(mydata));
        sprintf(mydata,"lat:%s,lon:%s",latitude,longitude);
        //sprintf(mydata,"pippopippo");
        memcpy(mydata2,mydata,sizeof(mydata));
        LMIC_setTxData2(1, mydata2, sizeof(mydata2)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    //rtc_gpio_init(GPIO_NUM_2);
    //rtc_gpio_init(GPIO_NUM_21);
    //rtc_gpio_hold_dis(GPIO_NUM_2);
    Serial.begin(9600);
    Serial.println(F("Starting"));

    Serial2.begin(9600, SERIAL_8N1, RXPin, TXPin);
  

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    display.init();
    
    display.drawString(0, 0, "Booting. JOIN");
//    bool pippo=rtc_gpio_is_valid_gpio(GPIO_NUM_2);
//    if( pippo ){
//    display.drawString(0, 40, "pippo" );
//    }else{
//    display.drawString(0, 40, "pluto" );
//    }
    display.display();
    sleep(5);
    display.clear();
    sleep(5);
    GPSStatus();
    
    
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
