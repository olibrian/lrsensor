/* lrsensor
 * Get GPS Position and send it to a MQTT Broker
 * Hardware: SIM808_v2; Arudio compatible board
 * Olivier Brian
 */

//Include library
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

//set variables
#define SIM808_RX 2
#define SIM808_TX 3
#define SIM808_RST 4

//gprs_apn = "gprs.swisscom.ch";
int Powerkey = 9; // PowerKey to enable 808 Modul
// this is a large buffer for replies
char replybuffer[255]; //Buffer for 808 replies 

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;



//Inital software serial
SoftwareSerial sim808SS = SoftwareSerial(SIM808_TX, SIM808_RX);  //Arduino
SoftwareSerial *sim808Serial = &sim808SS; // SIM808

Adafruit_FONA sim808 = Adafruit_FONA(SIM808_RST);

void setup() {
  // Start sim808
  pinMode(Powerkey, OUTPUT);   // initialize the digital pin as an output.  
  power();                     //power on the sim808 or power down the sim808
  
  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("SIM808 basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  sim808Serial->begin(4800);
  if (! sim808.begin(*sim808Serial)) {
    Serial.println(F("Couldn't find SIM808 :-("));
    while (1);
  }
  type = sim808.type();
  Serial.println(F("SIM(=( is OK"));
  
  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = sim808.getIMEI(imei);
  if (imeiLen > 0)
    Serial.print("Module IMEI: "); Serial.println(imei);
 

  // turn GPS on
  if (!sim808.enableGPS(true))
     Serial.println(F("Failed to turn on"));
         
}

void loop() {
        // check GPS fix
        int8_t stat;
        stat = sim808.GPSstatus();
        if (stat < 0)
          Serial.println(F("Failed to get GPS stat"));
        if (stat == 0) Serial.println(F("GPS off"));
        if (stat == 1) Serial.println(F("No fix"));
        if (stat == 2) Serial.println(F("2D fix"));
        if (stat == 3) Serial.println(F("3D fix"));

        if (stat > 2){
            // check for GPS location
            char gpsdata[120];
            sim808.getGPS(0, gpsdata, 120);
               //Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
            Serial.println(gpsdata);
        }
        delay(500);
}


void power(void)
{
  digitalWrite(Powerkey, LOW); 
  delay(1000);               // wait for 1 second
  digitalWrite(Powerkey, HIGH);
  Serial.println("Power Up the 808");
}
