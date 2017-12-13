/* lrsensor
   Get GPS Position and send it to a MQTT Broker
   Hardware: SIM808_v2; Arudio compatible board
   Olivier Brian
*/

//Include library
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"

//set variables
#define SIM808_RX 2
#define SIM808_TX 3
#define SIM808_RST 4

#define GPRS_APN "gprs.swisscom.ch"

#define MQTT_SERVER      "io.adafruit.com"
#define MQTT_SERVERPORT  1883
#define MQTT_USERNAME  "obrian"
#define MQTT_KEY       "63e391a67f2e401eb988cd200774a2aa"

int Powerkey = 9; // PowerKey to enable 808 Modul
// this is a large buffer for replies
char replybuffer[255]; //Buffer for 808 replies

char gpsdata[120];


uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
// How many transmission failures in a row we're willing to be ok with before reset
uint8_t txfailures = 0;
#define MAXTXFAILURES 3

uint8_t type;

//Inital software serial
SoftwareSerial sim808SS = SoftwareSerial(SIM808_TX, SIM808_RX);  //Arduino
SoftwareSerial *sim808Serial = &sim808SS; // SIM808

Adafruit_FONA sim808 = Adafruit_FONA(SIM808_RST);

// Setup the MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&sim808, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

/****************************** Feeds ***************************************/

// Setup feeds for publishing.
Adafruit_MQTT_Publish geolocation = Adafruit_MQTT_Publish(&mqtt, MQTT_USERNAME "/feeds/geolocation");

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
  Serial.println(F("SIM is OK"));

  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = sim808.getIMEI(imei);
  if (imeiLen > 0)
    Serial.print("Module IMEI: "); Serial.println(imei);

  // Connect GPRS
  while (! FONAconnect(F(GPRS_APN), F(""), F(""))) {
    Serial.println("Retrying FONA");
  }

  Serial.println(F("Connected to Cellular!"));

  // Turn GPS on
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

  if (stat > 2) {
    // check for GPS location
    sim808.getGPS(0, gpsdata, 120);
    //Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
    Serial.println(gpsdata);
  }

  // Connect MQTT
  MQTT_connect();

  if (! geolocation.publish(gpsdata)) {
    Serial.println(F("Failed"));
    txfailures++;
  } else {
    Serial.println(F("MQTT send OK!"));
    txfailures = 0;
  }
  delay(1000);  // wait for 1 second
}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

void power(void)
{
  digitalWrite(Powerkey, LOW);
  delay(1000);               // wait for 1 second
  digitalWrite(Powerkey, HIGH);
  Serial.println("Power Up the 808");
}
