/********************************************************************
  Example using AqualaboSensors library to communicate
  with C4E controller using a half-duplex RS485 transceiver.

  This example is tested against an Sensor C4E.

  Depended Library: ModbusFP

  created 26 July 2020
  by Author:: Andres Sabas @ Electronic Cats

  This code is beerware; if you see me (or any other Electronic Cats
  member) at the local, and you've found our code helpful,
  please buy us a round!
  Distributed as-is; no warranty is given.

*********************************************************************/
#include <AqualaboSensors.h>

aqualaboModbusSensorsClass aqualaboModbusSensors;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  
  while (!Serial);
  Serial.println("Modbus Sensor C4E");
  
  aqualaboModbusSensors.begin(30, 30, 21, Serial2 );
  
  Serial.println("Init sensor");
  aqualaboModbusSensors.initSensor();
  Serial.println("Ready... ");
}

void loop() {
  
  aqualaboModbusSensors.read();
  Serial.print("Temperature: ");
  Serial.println(aqualaboModbusSensors.sensorC4E.temperature, 2);
  Serial.print("Conductivity: ");
  Serial.println(aqualaboModbusSensors.sensorC4E.conductivity, 2);
  
  delay(1000);
}