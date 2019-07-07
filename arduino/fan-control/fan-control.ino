/*
 * Uno with wifly shield
 * 
 * Thanks and references:
 * wifly shield - 
 * Thanks to area515 makerspace and hackrack
 * https://github.com/dpslwk/WiFly
 * https://learn.sparkfun.com/tutorials/wifly-shield-hookup-guide/all
 * 
 * pubsubclient - 
 * https://github.com/knolleary/pubsubclient
 * https://pubsubclient.knolleary.net/
 * 
 * pwm 25khz fan control
 * pwm25kHzBegin
 * author - dlloyd
 * https://forum.arduino.cc/index.php?topic=415167.0
 * 
 * 
 */

#include <SPI.h>
#include <WiFly.h>
//#include "Credentials.h"
#include <PubSubClient.h>

word VentPin = 3;

WiFlyClient wiflyClient;
PubSubClient client(wiflyClient);

void pwm25kHzBegin() {
  TCCR2A = 0;                               // TC2 Control Register A
  TCCR2B = 0;                               // TC2 Control Register B
  TIMSK2 = 0;                               // TC2 Interrupt Mask Register
  TIFR2 = 0;                                // TC2 Interrupt Flag Register
  TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  // OC2B cleared/set on match when up/down counting, fast PWM
  TCCR2B |= (1 << WGM22) | (1 << CS21);     // prescaler 8
  OCR2A = 79;                               // TOP overflow value (Hz)
  OCR2B = 0;
}

//  pwmDuty(19); // 25% (range = 0-79 = 1.25-100%)
//  pwmDuty(39); // 50% (range = 0-79 = 1.25-100%)
//  pwmDuty(59); // 75% (range = 0-79 = 1.25-100%)
void pwmDuty(byte ocrb) {
  OCR2B = ocrb;                             // PWM Width (duty)
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char request[length];
  int percentFanRequest;
  for (int i=0;i<length;i++) {
    request[i] = (char)payload[i];
  }
  
  percentFanRequest = atoi(request);
  Serial.println(percentFanRequest);
  controlFanSpeed(percentFanRequest);
  client.publish("greenhouse/fan/response",request);
  
}



void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("greenhouse/device/online","arduinoClient");
      // ... and resubscribe
      client.subscribe("greenhouse/fan/request");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

IPAddress server(192,168,86,184);

void setup() {
  
  Serial.begin(9600);
  pinMode(VentPin, OUTPUT);
  pwm25kHzBegin();
  
  WiFly.begin();

  Serial.print("WIFI status: ");
  Serial.println(WiFly.ip());
  
  Serial.println("Wifi connection established");

  // this is the mqtt server
  client.setServer(server,1883);
  client.setCallback(callback);
}

void loop() {

  if(Serial.available()){
    int fanPercent=Serial.parseInt();
    Serial.print("setting fan percent to ");
    Serial.println(fanPercent);
    controlFanSpeed(fanPercent); 
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void controlFanSpeed (int fanSpeedPercent) {
  Serial.print("Fan Speed: ");
  Serial.print(fanSpeedPercent);
  Serial.println("%");  
  Serial.print("pwm value = ");
  int pwmval = fanSpeedPercent * .79;
  Serial.println(pwmval);
  pwmDuty(pwmval);
}
