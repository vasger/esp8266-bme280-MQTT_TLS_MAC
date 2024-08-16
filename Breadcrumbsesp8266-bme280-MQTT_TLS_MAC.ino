/*
 This sketch publishes temperature humidity and barometric pressure data from a bme280
 device to a MQTT topic + TLS.
 
 This sketch goes in deep sleep mode once the data has been sent to the MQTT
 topic and wakes up periodically (configure SLEEP_DELAY_IN_SECONDS accordingly).
 
 Hookup guide:
 
 - connect D0 pin to RST pin in order to enable the ESP8266 to wake up periodically
   (only after uploading the sketch!)
 - connect the Vout of your voltage divider to NodeMCU A0
 - connect the bme280:
      BME280 GND --> NodeMCU GND
      BME280 3.3V --> NodeMCU 3V3
      BME280 SDA --> NodeMCU D3
      BME280 SCL --> NodeMCU D2
      !bme.begin(0x76)
     
 This sketch is the result of the compilation of some diyers and arduinoers. Thanks to
 them!
 And of course it is in public domain.

 Enjoy!
 
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SLEEP_DELAY_IN_SECONDS  60

Adafruit_BME280 bme;

float tempC = 1;    // Temperature in Celsius degrees
float humidity = 1; // Relative Humidity in %
float pressHpa = 1; // Barometric Pressure in hectopascals
float level = 1;    // Battery voltage in volts
char temperatureString[6];
char humidityString[6];
char pressHpaString[6];
char voltageString[6];
String responseJson;
/*
Put you wi-fi credentials here
*/
const char* ssid = "Wifi-network"; // WiFi network SSID
const char* password = "Wifi-network-password"; // WiFi network password

 


const char* mqtt_server = "mqtt.server.xxx"; // MQTT broker URL
const char* mqtt_username = "mqtt_username";      // MQTT client user name, if needed
const char* mqtt_password = "mqtt_password";      // MQTT client password, if needed


const String TEMP_STATUS_TOPIC = "sensors/"+WiFi.macAddress()+"/test/temperature";
const String HUMIDITY_STATUS_TOPIC = "sensors/"+WiFi.macAddress()+"/test/humidity";
const String PRESSURE_STATUS_TOPIC = "sensors/"+WiFi.macAddress()+"/test/pressure";
const String VOLTAGE_STATUS_TOPIC = "sensors/"+WiFi.macAddress()+"/test/voltage";



// WiFiClient espClient;
WiFiClientSecure espClient;         
PubSubClient client(espClient);

// our gpio
#define ESP8266_LED (0)


void setup() {

  // setup serial port
  Serial.begin(115200);

  // connecting to the bme280 device
  if(!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    
    while (1)
    {
        yield();
        delay(2000);
    }
  }
  delay(500);
}

void setup_wifi()   // Connecting to the WiFi network
{
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Connecting to WiFi network ");
    Serial.print(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.print(".");
      }

    Serial.println(" connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
}

void callback(char* topic, byte* payload, unsigned int length) // Reading received topic
{
  Serial.print("Reveived topic [");
  Serial.print(topic);
  Serial.print("] ");
  
  for (int i = 0; i < length; i++)
    {
      Serial.print((char)payload[i]);
    }
  Serial.println();
}

void connect_mqtt() //Connecting to the MQTT broker
{
  while (!client.connected())
    {
    Serial.print("Connecting to MQTT broker at ");
    Serial.print(mqtt_server);
    Serial.print("...");

    // Attempt to connect
    if (client.connect(("ESP8266Client-"+WiFi.macAddress()).c_str(), mqtt_username, mqtt_password))
      {
        Serial.println(" connected!");
  
        // Once connected, publish an announcement...
        Serial.println("Sending topic(s)...");
        //client.publish("sensor/external", responseJson.c_str(), true);
        client.publish(TEMP_STATUS_TOPIC.c_str(), dtostrf(tempC, 2, 2, temperatureString));
        client.publish(HUMIDITY_STATUS_TOPIC.c_str(), dtostrf(humidity, 2, 2, humidityString));
        client.publish(PRESSURE_STATUS_TOPIC.c_str(), dtostrf(pressHpa, 2, 2, pressHpaString));

        client.publish(VOLTAGE_STATUS_TOPIC.c_str(), dtostrf(level/100, 2, 2, voltageString));
        // "Proj/webtech/"WiFi.macAddress()"/Time"
      }
    else
      {
        Serial.print("Connection failed, rc=");
        Serial.print(client.state());
        Serial.println(" will retry in 5 seconds");
  
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
}

void loop()
{
    // Reading temperature    
    tempC = bme.readTemperature();

    // Reading barometric pressure
    pressHpa = bme.readPressure() / 100.0;

    // Reading humidity
    humidity = bme.readHumidity();

    // Reading voltage
    float rawLevel = analogRead(A0);
    level = map(rawLevel, 98, 192, 200, 400); // You need to adjust these values according to the voltage divider you install

    Serial.println("");
    Serial.println("Temperature:");
    printValueAndUnits(tempC, " °C");

    Serial.println("Barometric Pressure:");
    printValueAndUnits(pressHpa, " hPa");

    Serial.println("Relative Humidity:");
    printValueAndUnits(humidity, " %");

    Serial.println("Voltage: ");
    printValueAndUnits(level/100, " V");

    responseJson = "";
    responseJson += "{";
    responseJson +=     "\"temperature\":" + String(tempC) + ",";
    responseJson +=     "\"humidity\":" + String(humidity) + ",";
    responseJson +=     "\"pressure\":" + String(pressHpa) + ",";
    responseJson +=     "\"voltage\":" + String(level/100);
    responseJson += "}";
    setup_wifi();
    
    client.setServer(mqtt_server, 8883);
    espClient.setInsecure();

    client.setCallback(callback);
    connect_mqtt();

    delay(100);
    
    Serial.println("Closing connection to MQTT broker...");
    client.disconnect();

    delay(100);

    Serial.print("Closing connection to WiFi network ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.disconnect();

    delay(100);

    Serial.print("Going to sleep for ");
    Serial.print(SLEEP_DELAY_IN_SECONDS);
    Serial.println(" seconds...");


    ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);

    delay(100);
}  
  
void printValueAndUnits(float value, String units)
{
    Serial.print("   ");
    Serial.print(value);
    Serial.print(" ");
    Serial.println(units);
    Serial.println("");
}
