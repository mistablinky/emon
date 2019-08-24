#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <sys/time.h>
#include <Adafruit_MCP3008.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "EmonLib.h"

// Secrets
const char* ssid_1      = "wifi_1_ssid";
const char* password_1  = "wifi_1_pw";
const char* ssid_2      = "wifi_2_ssid";
const char* password_2  = "wifi_2_pw";
const char* ssid_3      = "wifi_3_ssid";
const char* password_3  = "wifi_3_pw";
const char fingerprint[] PROGMEM = "emoncms_fingerprint";
const char* node      = "emoncms_node_name";
const char* apikey    = "emoncms_api_key";

//const char* server    = "https://emoncms.org/input";
const char* server    = "emoncms.org";
const int httpsPort   = 443;

ESP8266WiFiMulti wifiMulti;
boolean connectioWasAlive = true;
boolean flash = true;

// Time settings
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600; // still hard coded Sommerzeit
time_t now;

// Hardware settings
#define ONBOARD_LED_PIN_RED      0
#define ONBOARD_LED_PIN_BLUE     2
#define TEMPERATURE_PIN          5    // Dallas DS18B20 digital temperature sensor (one wire protocol)
#define MCP_CS_PIN              16    // Chip select pin for the MCP3008
//#define CURRENT_GRID_IN_PIN   36    // UEETEK SCT-013-000 current sensor on ADC1 (0-100A:0-50mA)
//#define CURRENT_GRID_OUT_PIN  39    // UEETEK SCT-013-000 current sensor on ADC1 (0-100A:0-50mA)


// External 8-Channel 10-Bit A/D Converter
#define MCP_CHANNELS_USED        5
Adafruit_MCP3008 adc;
uint16_t adc_raw[8];

// ADC calibration (MCP2008, 10bit)
#define V_REF                  3.3                          // MCP3008 reference voltage
float V_BatteryCalibration  =  (V_REF/1024) * 10.0;         // Voltage divider 100K/10K (33V max, 1=32mV)
float V_SolarCalibration    =  (V_REF/1024) * (100.0/4.7);  // Voltage divider 100K/4K7 (70V max, 1=68mV)
//float I_SolarCalibration  =  (V_REF/1024) * (1/0.045);    // ACS711EX Current Sensor 1A=45mV (73A max, 1=71mA)
float I_SolarCalibration    =  (V_REF/1024) * (1/0.045);    // ACS711EX Current Sensor 1A=45mV (73A max, 1=71mA)
float I_SolarOffset         =  -36.7;                       // Because ACS711EX quiescent output voltage is V_REF/2

// Create Emon Library instance
EnergyMonitor CGI;
EnergyMonitor CGO;
EnergyMonitor CS;

// Temperature sensor settings
OneWire oneWire(TEMPERATURE_PIN);     // oneWire instance to communicate with the DS18B20
DallasTemperature sensors(&oneWire);  // Pass oneWire reference to Dallas Temperature

// Timing and measurement settings
#define PUBLISHING_INTERVAL       5000  // milli seconds
#define MEASUREMENT_BUFFER_SIZE   100   // maximum count of readings between two publishings
uint8_t bufferPointer;
uint16_t measurementInterval = PUBLISHING_INTERVAL/MEASUREMENT_BUFFER_SIZE;   // milli seconds
unsigned long lastMeasurement;
unsigned long lastPublishing;

// Value Storage
unsigned long timestamp;            // UNIX timestamp

float I_GridIn_Buffer[MEASUREMENT_BUFFER_SIZE];
float I_GridOut_Buffer[MEASUREMENT_BUFFER_SIZE];
float V_Solar_Buffer[MEASUREMENT_BUFFER_SIZE];
float I_Solar_Buffer[MEASUREMENT_BUFFER_SIZE];
float V_Battery_Buffer[MEASUREMENT_BUFFER_SIZE];
float T_Battery_Buffer[MEASUREMENT_BUFFER_SIZE];

float V_GridIn       = 230.0;       // VAC
float I_GridIn;                     // IRMS
float P_GridIn;                     // Power

float V_GridOut      = 230.0;       // VAC
float I_GridOut;                    // IRMS
float P_GridOut;                    // Power

float V_Solar;                      // dc voltage (0-36,6V)
float I_Solar;                      // direct current (0-8,47A)
float P_Solar;                      // Power

float V_Battery;                    // DC (10-15VDC)
float T_Battery;                    // Temperature in °C


// Error handling
boolean error;

// ============================================================================================ //

void setup() {

  error = false;

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Booting up EmonCMS client \"" + String(node) + "\"");
  Serial.println("------------------------------------------");

  pinMode(ONBOARD_LED_PIN_RED, OUTPUT);
  pinMode(ONBOARD_LED_PIN_BLUE, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN_RED, HIGH);
  digitalWrite(ONBOARD_LED_PIN_BLUE, HIGH);

//  CGI.current(CURRENT_GRID_IN_PIN, 111.1);
//  CGO.current(CURRENT_GRID_OUT_PIN, 111.1);
//  CS.current(CURRENT_SOLAR_PIN, 111.1);
  
  // prepare WIFI connection
  WiFi.hostname(node);
  wifiMulti.addAP(ssid_1, password_1);
  wifiMulti.addAP(ssid_2, password_2);
  //wifiMulti.addAP(ssid_3, password_3);

  // Initialize A/D Converter
  adc.begin(MCP_CS_PIN);

  // Initialize temperature sensor
  sensors.begin();

}
 
// ============================================================================================ //
 
void loop() {

//  if (error) {
//    ESP.restart();
//  }

  monitorWiFi();

  if (connectioWasAlive) {

    if (millis()-lastMeasurement >= measurementInterval) {
  
      digitalWrite(ONBOARD_LED_PIN_RED, LOW);
  
      //Serial.print(".");
  
      Serial.print("Sample rate (ms): " + String(millis()-lastMeasurement));
      
      // reset measurement timer
      lastMeasurement = millis();
  
      Serial.print("\n[" + String(bufferPointer) + "]\t");
  
      // Measurements
      for (int chan=0; chan<MCP_CHANNELS_USED; chan++) {
        adc_raw[chan] = adc.readADC(chan);
        Serial.print(adc_raw[chan]); Serial.print("\t");
      }
      
      double  V_BatteryRaw = adc_raw[0];  // MCP8003 CH0
      V_Battery_Buffer[bufferPointer] = V_BatteryRaw * V_BatteryCalibration;
      //Serial.println("\nV_BatteryRaw * V_BatteryCalibration: " + String(V_BatteryRaw) + " * " + String(V_BatteryCalibration));
      //Serial.println(V_REF*adc_raw[0]/1024);
      //Serial.println("V_Battery_Buffer[" + String(bufferPointer) + "]: " + String(V_Battery_Buffer[bufferPointer]));
  
      double  V_SolarRaw = adc_raw[3];    // MCP8003 CH1
      V_Solar_Buffer[bufferPointer]   = V_SolarRaw * V_SolarCalibration;
      Serial.println("V_SolarRaw: " + String(V_SolarRaw));
      Serial.println("V_Solar_Buffer[" + String(bufferPointer) + "]: " + String(V_Solar_Buffer[bufferPointer]));
  
      double  I_SolarRaw = adc_raw[2];    // MCP8003 CH2
      I_Solar_Buffer[bufferPointer]   = I_SolarRaw * I_SolarCalibration + I_SolarOffset;
      //Serial.println("I_Solar_Buffer[" + String(bufferPointer) + "]: " + String(I_Solar_Buffer[bufferPointer]));
      
      //I_GridIn_Buffer[bufferPointer]  = adc1_get_raw(ADC1_CHANNEL_0);
      //I_GridIn_Buffer[bufferPointer]  = CGI.calcIrms(1480);
      //Serial.println("I_GridIn_Buffer[" + String(bufferPointer) + "]: " + String(I_GridIn_Buffer[bufferPointer]));
      
      //I_GridOut_Buffer[bufferPointer] = CGO.calcIrms(1480);
      //Serial.println("I_GridOut_Buffer[" + String(bufferPointer) + "]: " + String(I_GridOut_Buffer[bufferPointer]));
  
      bufferPointer++;
  
      digitalWrite(ONBOARD_LED_PIN_RED, HIGH);
  
      if (millis()-lastPublishing >= PUBLISHING_INTERVAL) {
  
        //Serial.println("[" + String(bufferPointer) + " Readings]");
  
        Serial.println("\n------- START PUBLISHING -------");
        Serial.println("millis()-lastPublishing: " + String(millis()-lastPublishing));
  
        // reset measurement timer
        lastPublishing = millis();
  
        //Serial.println("measurementInterval: " + String(measurementInterval));
       
    
        if ((WiFi.status() == WL_CONNECTED)) { //Check the current connection status
  
          digitalWrite(ONBOARD_LED_PIN_RED, HIGH);
        
          // read temperature sensor value
          sensors.requestTemperatures();
          T_Battery = sensors.getTempCByIndex(0);
    
          // calculate mean of sensor readings
          V_Battery = 0;
          V_Solar   = 0;
          I_Solar   = 0;
          I_GridIn  = 0;
          I_GridOut = 0;
          for (uint8_t i=0; i<bufferPointer; i++){
            V_Battery += V_Battery_Buffer[i];
            V_Solar   += V_Solar_Buffer[i];
            I_Solar   += I_Solar_Buffer[i];
            I_GridIn  += I_GridIn_Buffer[i];
            I_GridOut += I_GridOut_Buffer[i];
          }
          V_Battery /= bufferPointer;
          V_Solar   /= bufferPointer;
          I_Solar   /= bufferPointer;
          I_GridIn  /= bufferPointer;
          I_GridOut /= bufferPointer;
  
          P_GridIn  = I_GridIn * V_GridIn;
          P_GridOut = I_GridOut * V_GridOut;
  
    
          // read current UTC time and store in UNIX format (POSIX)
          time(&now);
          timestamp = now;
    
          Serial.println();
          Serial.println();
          Serial.print("UTC Time:  " + String(ctime(&now)));
          Serial.println("UNIX time: " + String(timestamp));
          Serial.println();
          Serial.println("V_GridIn: " + String(V_GridIn));
          Serial.println("I_GridIn: " + String(I_GridIn));
          Serial.println("P_GridIn: " + String(P_GridIn));
          Serial.println();
          Serial.println("V_GridOut: " + String(V_GridOut));
          Serial.println("I_GridOut: " + String(I_GridOut));
          Serial.println("P_GridOut: " + String(P_GridOut));
          Serial.println();
          Serial.println("V_Solar: " + String(V_Solar));
          Serial.println("I_Solar: " + String(I_Solar));
          Serial.println("P_Solar: " + String(P_Solar));
          Serial.println();
          Serial.println("V_Battery: " + String(V_Battery));
          Serial.println("T_Battery: "  + String(T_Battery));
          Serial.println();
          
          // build up json url
          //String url = String(server) + "/input/post?time=" + String(timestamp) + "&node=" + String(node) + "&fulljson={";
          String url = "/input/post?time=" + String(timestamp) + "&node=" + String(node) + "&fulljson={";
          url = url + "\"I_GridIn\":"  + I_GridIn  + ",";
          url = url + "\"I_GridOut\":" + I_GridOut + ",";
          url = url + "\"I_Solar\":"   + I_Solar   + ",";
          url = url + "\"V_Solar\":"   + V_Solar   + ",";
          url = url + "\"V_Battery\":" + V_Battery + ",";
          url = url + "\"P_GridIn\":"  + P_GridIn  + ",";
          url = url + "\"P_GridOut\":" + P_GridOut + ",";
          url = url + "\"T_Battery\":"  + T_Battery   + "}";
          url = url + "&apikey="          + String(apikey);
        
  
          // Use WiFiClientSecure class to create TLS connection
          WiFiClientSecure client;
          client.setFingerprint(fingerprint);

          monitorWiFi();
          Serial.print("Connecting to " + String(server) + "... ");
          if (!client.connect(server, httpsPort)) {
            Serial.println("failed");
            digitalWrite(ONBOARD_LED_PIN_RED, LOW);
            error = true;
            return;
          } else Serial.println("OK");
          
          Serial.println(String("GET ") + url);
          //Make the request
          digitalWrite(ONBOARD_LED_PIN_BLUE, LOW);
          client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + server + "\r\n" +
                 "User-Agent: BuildFailureDetectorESP8266\r\n" +
                 "Connection: close\r\n\r\n");
        
          String line = client.readStringUntil('\n');
          if (line.lastIndexOf("OK") != -1) {
            Serial.print("Publishing on emoncms.org successfull!");
          } else {
            Serial.print("Publishing on emoncms.org has failed");
            digitalWrite(ONBOARD_LED_PIN_RED, LOW);
            error = true;
          }
          Serial.println(" (" + line + ")");
        }
         digitalWrite(ONBOARD_LED_PIN_BLUE, HIGH);
  
        // reset buffer pointer
        bufferPointer = 0;
  
        Serial.println("------- END PUBLISHING -------");
        
      } // end publishinginterval
  
    } // end measurement interval

  } // end connectioWasAlive = true
}

void monitorWiFi() {
  // wenn WiFi-Verbindung nicht steht
  if (wifiMulti.run() != WL_CONNECTED) {
    if (connectioWasAlive == true) {
      connectioWasAlive = false;
      Serial.println();
      Serial.println();
      Serial.print("Looking for WiFi ");
    }
    digitalWrite(ONBOARD_LED_PIN_RED, !flash);
    Serial.print(".");
    delay(500);
    flash = !flash;
  }
  // wenn ansonsten WiFi-Verbindung gerade neu etabliert wurde
  // dann Zeit neu synchronisieren
  else if (connectioWasAlive == false) {
    connectioWasAlive = true;
    Serial.printf(" connected to %s\n", WiFi.SSID().c_str());

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    //init and get the time
    Serial.print("Get current time from " + String(ntpServer) + " ... ");
    configTime(0, 0, ntpServer);
    delay(100);
    time(&now);
    Serial.print(ctime(&now));

    // reset publishing and measurement timer and buffer pointer
    Serial.println("Going to send data every " + String(PUBLISHING_INTERVAL) + " milli seconds to " + String(server));
    lastPublishing = millis();
    lastMeasurement = millis();
    bufferPointer = 0;

  }
}
