#include <AqualaboSensors.h>

aqualaboModbusSensorsClass aqualaboModbusSensors;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Modbus Sensor C4E");
  
  aqualaboModbusSensors.begin(30, Serial2, 21, 30 );
  aqualaboModbusSensors.initSensor();
}

void loop() {
  
  aqualaboModbusSensors.read();
  Serial.print("Temperature: ");
  Serial.println(aqualaboModbusSensors.sensorC4E.temperature, 2);
  Serial.print("Conductivity: ");
  Serial.println(aqualaboModbusSensors.sensorC4E.conductivity, 2);
  
  delay(1000);
}