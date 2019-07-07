/*
  Read both CCS811 and BME280 sensors at same time
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 6th, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Works with SparkFun combo board https://www.sparkfun.com/products/14241
  Enjoy this code? Buy a board and help support SparkFun! 
  
  Let's read both sensors at the same time!
  Outputs CO2, TVOC, temperature, humidty, and pressure.
  
  Hardware Connections (Breakoutboard to Arduino):
  Attach a Qwiic Shield to your RedBoard, Photon, or ESP32.
  Plug the Qwiic Air Quality Combo board into one of the connectors
  Serial.print it out at 9600 baud to serial monitor.

*/

#include <Wire.h>
#include "SparkFunBME280.h" //Library for BME280 from library manager or https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
#include "SparkFunCCS811.h"
#define CCS811_ADDR 0x5B
CCS811 myCCS811(CCS811_ADDR);
BME280 myBME280; //Global sensor object for BME280

#include <WiFi.h>
#include <WiFiMulti.h>
WiFiMulti WiFiMulti;

#include <PubSubClient.h>
const char* mqtt_server = "192.168.86.184";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

//Global variables obtained from the two sensors
unsigned int tVOC = 0;
unsigned int CO2 = 0;
float tempf = 0;
float humidity = 0;
float pressureInHg = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("CCS811+BME280 Read Example");

  delay(100);

// We start by connecting to a WiFi network
    WiFiMulti.addAP(".......", "...........");

    Serial.println();
    Serial.println();
    Serial.print("Waiting for WiFi... ");

    while(WiFiMulti.run() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    Serial.println("");
    Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  
  Wire.begin();//initialize I2C bus
  
  CCS811Core::status returnCode = myCCS811.begin();
  if (returnCode != CCS811Core::SENSOR_SUCCESS)
  {
    Serial.println("Problem with CCS811");
    printDriverError(returnCode);
  }
  else
  {
    Serial.println("CCS811 online");
  }

  //Setup the BME280 for basic readings
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;
  myBME280.settings.runMode = 3; //  3, Normal mode
  myBME280.settings.tStandby = 0; //  0, 0.5ms
  myBME280.settings.filter = 0; //  0, filter off
  myBME280.settings.tempOverSample = 1;
  myBME280.settings.pressOverSample = 1;
  myBME280.settings.humidOverSample = 1;

  delay(10); //Give BME280 time to come on
  //Calling .begin() causes the settings to be loaded
  byte id = myBME280.begin(); //Returns ID of 0x60 if successful
  if (id != 0x60)
  {
    Serial.println("Problem with BME280");
  }
  else
  {
    Serial.println("BME280 online");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "temp-1";
//    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected to mqtt broker");
      // Once connected, publish an announcement...
      client.publish("greenhouse/devices/online", clientId.c_str());
      // ... and resubscribe
//      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  if (myCCS811.dataAvailable()) //Check to see if CCS811 has new data (it's the slowest sensor)
  {
    myCCS811.readAlgorithmResults(); //Read latest from CCS811 and update tVOC and CO2 variables
    //getWeather(); //Get latest humidity/pressure/temp data from BME280
    printData(); //Pretty print all the data
  }
  else if (myCCS811.checkForStatusError()) //Check to see if CCS811 has thrown an error
  {
    Serial.println(myCCS811.getErrorRegister()); //Prints whatever CSS811 error flags are detected
  }

  delay(2000); //Wait for next reading
}

//Print CO2, TVOC, Humidity, Pressure and Temp
void printData()
{
  
  String CO2 = String(myCCS811.getCO2());
  Serial.print(" CO2[");
  Serial.print(myCCS811.getCO2());
  Serial.print("]ppm");
  client.publish("greenhouse/env/CO2/ppm", CO2.c_str());

  String tvoc = String(myCCS811.getTVOC());
  Serial.print(" TVOC[");
  Serial.print(tvoc);
  Serial.print("]ppb");
  client.publish("greenhouse/env/TVOC/ppb", tvoc.c_str());

  String tempC = String(myBME280.readTempC());
  Serial.print(" temp[");
  Serial.print(tempC);
  Serial.print("]C");
  client.publish("greenhouse/env/temp/C", tempC.c_str());

  String tempF = String(myBME280.readTempF());
  Serial.print(" temp[");
  Serial.print(tempF);
  Serial.print("]F");
  client.publish("greenhouse/env/temp/F", tempF.c_str());

  String pressurePa = String(myBME280.readFloatPressure());
  Serial.print(" pressure[");
  Serial.print(pressurePa);
  Serial.print("]Pa");
  client.publish("greenhouse/env/pressure/Pa", pressurePa.c_str());

  String pressureInHg = String((myBME280.readFloatPressure() * 0.0002953));
  Serial.print(" pressure[");
  Serial.print(pressureInHg);
  Serial.print("]InHg");
  client.publish("greenhouse/env/pressure/InHg", pressureInHg.c_str());

  String altitudeM = String(myBME280.readFloatAltitudeMeters());
  Serial.print("altitude[");
  Serial.print(altitudeM);
  Serial.print("]m");
  client.publish("greenhouse/env/altitude/m",altitudeM.c_str());

  String altitudeF = String(myBME280.readFloatAltitudeFeet());
  Serial.print("altitude[");
  Serial.print(altitudeF);
  Serial.print("]ft");
  client.publish("greenhouse/env/altitude/ft",altitudeF.c_str());

  String humidity = String(myBME280.readFloatHumidity());
  Serial.print(" humidity[");
  Serial.print(humidity);
  Serial.print("]%");
  client.publish("greenhouse/env/humidity/percent",humidity.c_str());

  Serial.println("");
  delay(5000);
}

void printDriverError( CCS811Core::status errorCode )
{
  switch ( errorCode )
  {
    case CCS811Core::SENSOR_SUCCESS:
      Serial.print("SUCCESS");
      break;
    case CCS811Core::SENSOR_ID_ERROR:
      Serial.print("ID_ERROR");
      break;
    case CCS811Core::SENSOR_I2C_ERROR:
      Serial.print("I2C_ERROR");
      break;
    case CCS811Core::SENSOR_INTERNAL_ERROR:
      Serial.print("INTERNAL_ERROR");
      break;
    case CCS811Core::SENSOR_GENERIC_ERROR:
      Serial.print("GENERIC_ERROR");
      break;
    default:
      Serial.print("Unspecified error.");
  }
}
