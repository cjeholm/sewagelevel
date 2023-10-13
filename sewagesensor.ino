//  ESP32 Sewage Level Sensor
//  -------------------------
//  This code is written for the M5Stack ATOM Lite ESP32
//  using the Time-of-Flight Distance Ranging Sensor Unit VL53L0X.
//  Measurements are posted to MQTT
//  by Conny Holm 2021

#include "Adafruit_VL53L0X.h"   // ToF sensor
#include "M5Atom.h"
#include "WiFi.h"
#include "PubSubClient.h"       // MQTT
#include "ArduinoJson.h"        // For JSON serialization

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */ 
#define TIME_TO_SLEEP 8 * 3600 /* Time ESP32 will go to sleep (in seconds) */
#define LOX1_ADDRESS 29 /* I2C adress of ToF sensor */
#define max_conn_attempts 10

String device_name = "SewageLaser01";

int ToF_measure;
int ToF_status;
int connection_attempts = 0;

char ssid[] = "my-wifi";     //  your network SSID (name)
char pass[] = "my-password";  // your network password
int WL_status = WL_IDLE_STATUS;     // the WiFi radio's status

char mqtt_server[] = "192.168.0.100";	// IP of your MQTT server
char mqtt_ID[] = "SewageLaser01";
char mqtt_topic[] = "home/sewagetank/levelsensor";
int mqtt_port = 1883;

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config = Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE; // Setting Long Range mode

WiFiClient wClient;
PubSubClient mqttClient(wClient);



void setup() {
   M5.begin(true, false, true);
   M5.dis.drawpix(0, 0xfFFFF00);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  
  Serial.println("VL53L0X Laser Time of Flight sensor");
    if (!lox1.begin(LOX1_ADDRESS, false, &Wire, sensor_config)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }


  // attempt to connect to WiFi network:
  while (WL_status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    // WiFi.hostname(device_name);    
    WL_status = WiFi.begin(ssid, pass);

    connection_attempts += 1; // sleep again if connection was not made
    if (connection_attempts >= max_conn_attempts)
      {
          esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
          esp_deep_sleep_start();
      }

    // wait 500 + 500 milliseconds for connection:
    M5.dis.drawpix(0, 0xfFF0000); 
    delay(500);
    M5.dis.drawpix(0, 0xf000000); 
    delay(500);
  }  


// Set the MQTT server to the server stated above ^
  mqttClient.setServer(mqtt_server, mqtt_port);  
  mqttClient.setKeepAlive(600); 
 
  // Attempt to connect to the server with the ID "mqtt_ID"
  if (mqttClient.connect(mqtt_ID)) 
  {
    Serial.print("MQTT connected to "); Serial.println(mqtt_server);
    // Establish the subscribe event
    // mqttClient.setCallback(subscribeReceive);
  } 
  else 
  {
    Serial.println("MQTT server connection failed...");
  }
  

// Main stuff

  // Blue blink if wifi is connected
  if (WL_status == WL_CONNECTED) {
    M5.dis.drawpix(0, 0xf0000FF); 
    delay(200);
  }
  else {
    M5.dis.drawpix(0, 0xfFF0000); 
    delay(200);
  }
  M5.dis.drawpix(0, 0xf000000); 
  delay(200);


  // Do a measurement
  VL53L0X_RangingMeasurementData_t measure;
  lox1.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  // int ToF_measure;
  // int ToF_status;

  ToF_measure = measure.RangeMilliMeter;
  // ToF_status = measure.RangeStatus;
    

//  Serial.println("");
//  Serial.print("Status: "); Serial.println(ToF_status);
//  Serial.print("Measure: "); Serial.println(ToF_measure);

  if (ToF_status != 4) {
    M5.dis.drawpix(0, 0xf00FF00);
  } else {
    M5.dis.drawpix(0, 0xfFF0000);    
  }

/*
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    M5.dis.drawpix(0, 0xf00FF00);
  } else {
    Serial.println(" out of range "); Serial.println(measure.RangeMilliMeter);
    M5.dis.drawpix(0, 0xfFF0000);
  }
*/

    // you're connected now, so print out the wifi data:
  Serial.println("");
  Serial.print("IP   "); Serial.println(WiFi.localIP());
  Serial.print("SSID "); Serial.println(WiFi.SSID());
//  Serial.print("RSSI "); Serial.println(WiFi.RSSI());



  // Serialize sensor and RSSI data to JSON and publish to MQTT
  DynamicJsonDocument doc(128);

  doc["sensor"] = "VL53L0X_ToF";   
  doc["measure"] = ToF_measure;
  doc["status"]  = ToF_status;
  doc["RSSI"]    = WiFi.RSSI();
  doc["retries"] = connection_attempts;
  

  char payload[128];  
  serializeJson(doc, payload);
  // char payload;
  // dtostrf(doc, 1, 0, payload);
  mqttClient.publish(mqtt_topic, payload);
  Serial.println(mqtt_topic);
  Serial.println(payload);


/*
  // MQTT publishing for tank sensor
  char distString[4];
  char statusString[1];
  dtostrf(ToF_measure, 1, 0, distString);
  dtostrf(ToF_status, 1, 0, statusString);
  mqttClient.publish("home/sewagetank/laser_measure", distString);
  Serial.print("home/sewagetank/laser_measure: "); Serial.println(ToF_measure);
  mqttClient.publish("home/sewagetank/laser_status", statusString);
  Serial.print("home/sewagetank/laser_status: "); Serial.println(ToF_status);




  // MQTT pubishing for RSSI
  char rssiString[4];
  dtostrf(WiFi.RSSI(), 1, 0, rssiString);
  mqttClient.publish("home/sewagetank/laser_RSSI", rssiString);
  Serial.print("home/sewagetank/laser_RSSI: "); Serial.println(rssiString);

*/  

  delay(200);
  M5.dis.drawpix(0, 0xf000000);  // turn off led before sleep
  // M5.dis.drawpix(0, 0xf666666);  // turn on led before sleep to keep power bank on
  delay(200);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();

}


void loop() {
  // Nothing in loop, sleeping and rebooting instead.
}

