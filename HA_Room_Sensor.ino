#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <FS.h> 
#include <ArduinoJson.h> 
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "WiFiManager.h"          //https://github.com/tzapu/WiFiManager
#include <Adafruit_BMP085.h>
#include "DHTesp.h"
#include <PubSubClient.h>

DHTesp dht;
Adafruit_BMP085 bmp;
WiFiClient espClient;
PubSubClient client(espClient);

/*
 *  Define the pins the sensors are connected to
 */
#define PIR_PIN 13
#define DHT_PIN 2
#define LDR_PIN A0
// The bmp180 is on the I2C bus

/*
    Setting up the Global Variables for our sensor readings
*/
// Used to set the JSON Buffer size
const int BUFFER_SIZE = 400;

// Needed for Sensor Data
bool MotionSensor;
float Temperature;
float TemperatureF;
float Pressure;
float LightReading;
float humidity;

// Timer
const unsigned long Minutes = 1 * 60 * 1000UL;
static unsigned long lastSampleTime = 0 - Minutes;  // initialize such that a reading is due the first time through

// Settings
char mqtt_server[20];
char mqtt_port[6];
char mqtt_user[8];
char mqtt_passwd[10];
char update_server[20];
char Sensor_Name[20];
bool shouldSaveConfig = false;

//holding stuff for checks

char message_buff[100];
bool MQTT_Status;
bool MQTT_Message;
float LState;



ESP8266WebServer server(80);


/*
 * 
 * MQTT Post, reconnect, and Callback Functions
 * 
 */

void mqtt_post(){

  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
  
  JsonObject& root = jsonBuffer.createObject();
  root[F("temperature")] = Temperature;
  root[F("temperatureF")] = TemperatureF;
  root[F("humidity")] = humidity;
  root[F("motion")] = MotionSensor;
  root[F("ldr")] = LightReading;
  root[F("heatIndex")] = calculateHeatIndex();
  root[F("barometric_pressure")] = Pressure;
  
  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  MQTT_Message = client.publish(Sensor_Name, buffer, true);
  
}


void reconnect() {
 
  while (!client.connected()) {

    if (client.connect(Sensor_Name, mqtt_user, mqtt_passwd)) {
      MQTT_Status = true;

    } else {

      MQTT_Status = false;

      delay(5000);
    }
  }
}

/*
 * 
 * Wifi Functions to Setup, connect, and Reconnect
 * 
 * 
 */

 void saveConfigCallback () {
   shouldSaveConfig = true;
}

void WiFiConfig(){

    //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json

  if (SPIFFS.begin()) {
    
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
         size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());

        if (json.success()) {
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_passwd, json["mqtt_passwd"]);
          strcpy(update_server, json["update_server"]);
          strcpy(Sensor_Name, json["Sensor_Name"]);
          
        } else {
          
        }
      }
    }
  } else {
  
  }
 



  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("MQTT User", "mqtt user", mqtt_server, 8);
  WiFiManagerParameter custom_mqtt_passwd("MQTT Password", "mqtt passwd", mqtt_server, 10);
  WiFiManagerParameter custom_update_server("Userver", "Update Server", update_server, 40);
  WiFiManagerParameter custom_Sensor_Name("Sensor_Name", "Sensor Name", Sensor_Name, 40);
  
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_passwd);
  wifiManager.addParameter(&custom_update_server);
  wifiManager.addParameter(&custom_Sensor_Name);
  
  //reset settings - for testing
  //wifiManager.resetSettings();

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect()) {
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi


  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_passwd, custom_mqtt_passwd.getValue());
  strcpy(update_server, custom_update_server.getValue());
  strcpy(Sensor_Name, custom_Sensor_Name.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {

    //DynamicJsonBuffer jsonBuffer;
    StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_passwd"] = mqtt_passwd;
    json["update_server"] = update_server;
    json["Sensor_Name"] = Sensor_Name;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {

    }

 
    json.printTo(configFile);
    configFile.close();
    //end save
  }

}


/*
 * 
 *  Sensor Functions. Collect Data and Calculate heat index
 * 
 * 
 */
float calculateHeatIndex() {
  float heatIndex= 0;
  if (Temperature >= 80) {
    heatIndex = -42.379 + 2.04901523*Temperature + 10.14333127*humidity;
    heatIndex = heatIndex - .22475541*Temperature*humidity - .00683783*Temperature*Temperature;
    heatIndex = heatIndex - .05481717*humidity*humidity + .00122874*Temperature*Temperature*humidity;
    heatIndex = heatIndex + .00085282*Temperature*humidity*humidity - .00000199*Temperature*Temperature*humidity*humidity;
  } else {
     heatIndex = 0.5 * (Temperature + 61.0 + ((Temperature - 68.0)*1.2) + (humidity * 0.094));
  }

  if (humidity < 13 && 80 <= Temperature <= 112) {
     float adjustment = ((13-humidity)/4) * sqrt((17-abs(Temperature-95.))/17);
     heatIndex = heatIndex - adjustment;
  }

  return heatIndex;
}

void Sensor_Data() {
  
unsigned long time_now = millis();

/*
 *  Pull LDR Reading and report only if the change is greater than 20
 */
delay(10);
    int sensorValue = analogRead(LDR_PIN);
    LightReading = sensorValue * (5.0 / 1023.0);
    if ( LightReading < LState - 20) {
        LState = LightReading;
        mqtt_post();
    } else if ( LightReading > LState + 20) {
      LState = LightReading;
        mqtt_post();
    }

/*
 *  Pull Motion Reading and report only on state change
 */
   long state = digitalRead(PIR_PIN);
    
    if (state == HIGH && MotionSensor != true) {
      MotionSensor = true;
      mqtt_post();
       } else if (state == LOW && MotionSensor != false) {
      MotionSensor = false;
      mqtt_post();
      }  

/*      
 *       The other sensors only need to be read once a min and reported
 */
if (time_now - lastSampleTime >= Minutes){
   lastSampleTime += Minutes;
   Temperature = bmp.readTemperature();
   TemperatureF = (Temperature * 1.8) + 32;
   Pressure = bmp.readPressure();
   delay(dht.getMinimumSamplingPeriod());
   humidity = dht.getHumidity();
   mqtt_post();
}
   
  }

/*
 * 
 * Web Server Functions handling the index, /reset, and file not found 
 * 
 * 
 */
void handleRoot() {
  char HTML_CODE[2000];
  float HeatIndex = calculateHeatIndex();
  snprintf  (HTML_CODE, 2000,
                    "<html><head> <title>Room Sensor</title></head>\
                    <body><h1 style=\"text-align: center;\">Room Sensor</h1><p>&nbsp;</p>\
                    <table align=\"center\" border=\"1\" cellpadding=\"1\" cellspacing=\"1\" style=\"width:500px;\">\
                    <caption><B>Readings</b></caption><tbody>\
                    <tr><td>Motion</td><td>%s</td></tr>\
                    <tr><td>Light reading</td><td>%1.2f</td></tr>\
                    <tr><td>Temperature</td><td>%2.2f C</td></tr>\
                    <tr><td>Temperature in F</td><td>%2.2f F</td></tr>\
                    <tr><td>Humidity</td><td>%2.2f %</td></tr>\
                    <tr><td>Heat Index</td><td>%2.2f %</td></tr>\
                    <tr><td>Barometric Pressure</td><td>%4.2f Pa</td></tr>\
                    <tr><td>MQTT Status</td><td>%s</td></tr>\
                    <tr><td>Last MQTT Message</td><td>%s</td></tr>\
                    </tbody></table>\
                    <table align=\"center\" border=\"1\" cellpadding=\"1\" cellspacing=\"1\" style=\"width:500px;\">\
                    <caption><b>Settings</b></caption><tbody>\
                    <tr><td>Sensor Name</td><td>%s</td></tr>\
                    <tr><td>MQTT Server</td><td>%s</td></tr>\
                    <tr><td>MQTT Port</td><td>%s</td></tr>\
                    <tr><td>Update Server</td><td>%s</td></tr>\
                    </tbody></table>\
                    <p><center><h3><a href=\"/reset\">Reset Config</a></p></h3><center></body></html>",
                    MotionSensor ? "true" : "false", LightReading, Temperature, TemperatureF, humidity, HeatIndex, Pressure, MQTT_Status  ? "Connected" : "Connection Failed", MQTT_Message ? "Success" : "Failed", Sensor_Name, mqtt_server, mqtt_port, update_server
                    );
                    
  server.send(200, "text/html", HTML_CODE);
}


void handleReset() {
  char HTML_CODE[400];
  if (server.hasArg("con")) {
  snprintf  (HTML_CODE, 400,
              "  <html><head> <title>Room Sensor</title></head>\
             <body><h1 style=\"text-align: center;\">Room Sensor</h1>\
             <center><h1>Resetting Config!!!</h1></center><br><br>\
             <center><h1>Wait 1 min then reboot the device</h1></center>"
             );
             server.send(200, "text/html", HTML_CODE);
             delay(1000);
             WiFi.disconnect();
  } else {  
  snprintf  (HTML_CODE, 400,
             "<html><head> <title>Room Sensor</title></head>\
             <body><h1 style=\"text-align: center;\">Room Sensor</h1>\
             <p>Are you sure you want to reset the config?</p><BR>\
             <a href=\"/reset?con=Yes\">Yes!</a><br><BR><BR><BR><a href=\"/\">No!</a>"
             );
             server.send(200, "text/html", HTML_CODE);
  }           
  
}

/*
 * 
 * Initial Setup call
 * 
 * 
 */

void setup() {
  WiFiConfig();
  server.on("/", handleRoot);
  server.on ( "/reset", handleReset);
  server.onNotFound(handleRoot);
  server.begin();
  bmp.begin();
  //Start MQTT Communications
  client.setServer(mqtt_server, atoi(mqtt_port));    // Configure MQTT connexion
  // Set pin for dht sensor and set the Motion sensor pin for input
  dht.setup(DHT_PIN);
  pinMode(PIR_PIN, INPUT);   // declare sensor as input

}

/*
 * 
 * 
 *  Main loop
 * 
 * 
 */
void loop() {
  // Check MQTT Connection and reconnect if needed
    if (!client.connected()) {
    reconnect();
  }
  // Pull Sensor Data and post to MQTT
  Sensor_Data();
  // Handle webpage requests
  server.handleClient();
}
