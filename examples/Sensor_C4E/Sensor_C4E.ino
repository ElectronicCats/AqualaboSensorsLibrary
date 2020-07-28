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

/*!
  We're using a MAX485-compatible RS485 Transceiver.
  Rx/Tx is hooked up to the hardware serial port at 'Serial2'.
  The Data Enable and Receiver Enable pins are hooked up as follows:
*/

// instantiate aqualaboModbusSensors object
aqualaboModbusSensorsClass aqualaboModbusSensors;

void setup() {
  Serial.begin(9600);

  // Modbus communication runs at 9600 baud
  Serial2.begin(9600);
  
  while (!Serial);
  Serial.println("Modbus Sensor C4E");
  
  //Modbus slave ID 1, pin De, Pin Re, Serial
  aqualaboModbusSensors.begin(30, 30, 21, Serial2 );
  
  // Setups the sensor configuration paramenters 
  Serial.println("Init sensor");
  aqualaboModbusSensors.initSensor();
  
  Serial.println("Ready... ");
}

void loop() {
  
  // Reads the sensor data
  aqualaboModbusSensors.read();

  // Variable: stores measured temperature in Celsius degrees
  Serial.print("Temperature: ");
  Serial.println(aqualaboModbusSensors.sensorC4E.temperature, 2);
  
  // Variable: stores measured conductivity in μS/cm
  Serial.print("Conductivity: ");
  Serial.println(aqualaboModbusSensors.sensorC4E.conductivity, 2);
  
  delay(1000);
}