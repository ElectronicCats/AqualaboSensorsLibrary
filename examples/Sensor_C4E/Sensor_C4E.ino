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

  // Setup the sensor configuration paramenters
  Serial.println("Init sensor");
  aqualaboModbusSensors.initSensor();

  /*
   * Temperature parameter
    The sensor is calibrated ex works, meaning that no calibration is required
    before initial startup. During operation the sensor should be calibrated if the
    measured values begin to drift.

    Rinse the sensor in clean water and dry it with a soft cloth or an absorbent
    paper before each calibration.

    For this process it is advisable to use a reference temperature sensor.

    To exit the calibration without considering anything please insert 'Q' to Quit
    and press Enter.
    1. Insert the first calibration standard value you will use (offset) and press Enter.
    0*C is recommended (Sensor fully immersed in an ice/water bath)
    Example: 0.350
  */

  //aqualaboModbusSensors.calibrationProcess_C4E(TEMPERATURE);

  /*
    Conductivity parameter
    0. Introduction:

    This is a two-point calibration method. At the end of the process the results
    of the calibration will be stored in the FLASH memory of the sensor for
    future uses.

    The sensor is calibrated ex works, meaning that no calibration is required
    before initial startup. During operation the sensor should be calibrated if the
    measured values begin to drift.

    Rinse the sensor in clean water and dry it with a soft cloth or an absorbent
    paper before each calibration.

    With this process only one range will be calibrated, if desired, carry out
    this process for each range to be calibrated a maximum of 4 times.

    To exit the calibration without considering anything please insert 'Q' to Quit
    and press Enter.
    1. Insert the range to be calibrated from the 4 available ranges and press Enter:
    1 for Range 1 (0 / 200 uS/cm)
    2 for Range 2 (0 / 2000 uS/cm)
    3 for Range 3 (0 / 20 mS/cm)
    4 for Range 4 (0 / 200 mS/cm)
    Example: 2 */
  
  //aqualaboModbusSensors.calibrationProcess_C4E(CONDUCTIVITY);

  Serial.println("Ready... ");
}

void loop() {

  // Reads the sensor data
  aqualaboModbusSensors.read_C4E();

  // Variable: stores measured temperature in Celsius degrees
  Serial.print("Temperature: ");
  Serial.println(aqualaboModbusSensors.sensorC4E.temperature, 2);

  // Variable: stores measured conductivity in μS/cm
  Serial.print("Conductivity: ");
  Serial.println(aqualaboModbusSensors.sensorC4E.conductivity, 2);

  delay(1000);
}