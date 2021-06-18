/***

  SHENKAR - SMART SYSTEMS - IMM System
  By: Moran Michal and Ido
  DATE: June-2021
  Version: 3.2.4
  Final
 **/
//#include <ESP8266WiFi.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"


/* WiFi Access Point ***/

#define WLAN_SSID        "********"
#define WLAN_PASS        "********"        

/* Adafruit.io Setup ***/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "iznor"
#define AIO_KEY         "aio_Cpgb17LUjnJirkoBokycHOVIz9mK"


/* Global State (you don't need to change this!) */


// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/** Feeds ***/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity2");
Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure");
Adafruit_MQTT_Publish light_level = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/shenkar_my_light_level");


// Setup for subscribing.
Adafruit_MQTT_Subscribe sub_temp = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Subscribe sub_humid = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Humidity2");
Adafruit_MQTT_Subscribe sub_pressure = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Pressure");
Adafruit_MQTT_Subscribe sub_light_level = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/shenkar_my_light_level");


/* Sketch Code **/

#include "DHT.h"

#define DHTPIN 4     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

//Constants
#define redLed 13       // redLed pin at Arduino pin 14
#define orangeLed 14    // orangeLed pin at Arduino pin 14
#define greenLed 25     // greenLed pin at Arduino pin 14
#define blueLed 27      // blueLed pin at Arduino pin 14

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

//DHT init
DHT dht(DHTPIN, DHTTYPE);

int count = 0;

//Declerations of pressure sensor variables

float cf = 19.5; // caliberation factor

int ffs1 = 32; // FlexiForce sensor is connected analog pin A0 of arduino or mega.

int ffsdata = 0;
float vout;

//End of decleration of pressure sensor

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void setup() {
  count++;
  Serial.begin(115200);

  pinMode(ffs1, INPUT);     //FlexiForce sensor pin 32 mode

  delay(3000);
  Serial.println(F("Starting..."));
  delay(3000);
  Serial.println(F("\n\n##################################"));
  Serial.println(F("IMM Smart Storage System"));
  Serial.println(F("-->  by Moran, Michal & Ido -- "));
  Serial.println(F("##################################"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  Serial.println();
  // SETUP ALL THE SUBSRCIPTIONS HERE
  Serial.println(F("DHTxx test!"));

  //Led pin Modes
  pinMode(redLed, OUTPUT);    // Set redLed - 13 pin as an output
  pinMode(blueLed, OUTPUT);   // Set blueLed - 27 pin as an output
  pinMode(greenLed, OUTPUT);   // Set greenLed - 25 pin as an output
  pinMode(orangeLed, OUTPUT);  // Set orangeLed - 14 pin as an output
  dht.begin();

  //Led testing
  Serial.println("Testing Red... ");
  digitalWrite(redLed, LOW);
  delay(3000);
  digitalWrite(redLed, HIGH);
  delay(1500);
  digitalWrite(redLed, LOW);
  Serial.println("Done Testing Red... ");
  Serial.println("Testing Blue... ");
  digitalWrite(blueLed, LOW);
  delay(3000);
  digitalWrite(blueLed, HIGH);
  delay(1500);
  digitalWrite(blueLed, LOW);
  Serial.println("Done Testing Blue... ");
  Serial.println("Testing Green... ");
  digitalWrite(greenLed, LOW);
  delay(3000);
  digitalWrite(greenLed, HIGH);
  delay(1500);
  digitalWrite(greenLed, LOW);
  Serial.println("Done Testing Green... ");
  Serial.println("Testing Orange... ");
  digitalWrite(orangeLed, LOW);
  delay(3000);
  digitalWrite(orangeLed, HIGH);
  delay(1500);
  digitalWrite(orangeLed, LOW);
  Serial.println("Done Testing Orange... ");
  Serial.println("Done Led Test");

  mqtt.subscribe(&sub_pressure);
}




void loop() {
  // We must keep this for now
  MQTT_connect();

  Adafruit_MQTT_Subscribe *subscription;
  unsigned char* current_pressure = sub_pressure.lastread;

  char* cPressure = (char*)current_pressure;
  unsigned long a = atol(cPressure);
  unsigned int intPressure = (unsigned int)a;

  ffsdata = analogRead(ffs1);
  intPressure = (ffsdata * 5.0) / 1023.0;
  intPressure = intPressure * cf;

  //Cloud reading loop
  while ((subscription = mqtt.readSubscription(2000))) {
    if (subscription == &sub_pressure) {
      Serial.print(F("Current Data From Cloud: "));
      Serial.println(cPressure);
      if (intPressure < 80) {
        digitalWrite(redLed, HIGH);
        digitalWrite(greenLed, LOW);
      }
      else {
        digitalWrite(redLed, LOW);
        digitalWrite(greenLed, HIGH);
      }
    }

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    // Compute heat index in Fahrenheit (the default)
    float hif = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
    Serial.print(f);
    Serial.print(F("°F  Heat index: "));
    Serial.print(hic);
    Serial.print(F("°C "));
    Serial.print(hif);
    Serial.println(F("°F"));
    humidity.publish(h);

    //Tempratue check
    Serial.print(F("Current temprature Data From Cloud: "));
    Serial.println(t);
    if (t < 24.5) {
      temperature.publish(t);
      digitalWrite(blueLed, HIGH);
      if (intPressure < 80) digitalWrite(redLed, HIGH);
      else digitalWrite(redLed, LOW);
    }
    else {
      temperature.publish(t);
      digitalWrite(redLed, HIGH);
      digitalWrite(blueLed, LOW);
    }

    //Humidity check
    Serial.print(F("Current humuidity Data From Cloud: "));
    Serial.println(h);
    if (h > 80) {
      digitalWrite(redLed, HIGH);
      digitalWrite(orangeLed, LOW);
    }
    else {
      digitalWrite(orangeLed, HIGH);
      if ((intPressure < 80) || (t > 24.5)) digitalWrite(redLed, HIGH);
      else digitalWrite(redLed, LOW);
    }
  }

  ffsdata = analogRead(ffs1);
  vout = (ffsdata * 5.0) / 1023.0;
  vout = vout * cf;
  Serial.print("Flexi Force sensor: ");
  Serial.print(vout, 3);
  Serial.println("");
  pressure.publish(vout);
  delay(5000);

}

// ping the server to keep the mqtt connection alive
// NOT required if you are publishing once every KEEPALIVE seconds
/*
  if(! mqtt.ping()) {
  mqtt.disconnect();
  }
*/


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}
