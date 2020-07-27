#include <AqualaboSensors.h>

aqualaboModbusSensorsClass aqualaboModbusSensors;

void setup() {
  /*
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  */
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