/*! \file AqualaboModbusSensors.h
	\brief Library for managing the modbus sensors in Smart Water Xtreme.
	This library is not compatible con Smart water version.

	Copyright (C) 2019 Libelium Comunicaciones Distribuidas S.L.
	http://www.libelium.com

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 2.1 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Version:			3.2
	Design:				David Gascón
	Implementation:		Victor Boria
*/


#ifndef AqualaboModbusSensor_h
#define ModbusSensor_h

/*****************************************************************************
* Includes
******************************************************************************/
#include <inttypes.h>
#include <Arduino.h>
/////////////////////////////////////////////////////////////////////////////
// Include the neccesary libraries.
/////////////////////////////////////////////////////////////////////////////
#include "ModbusMasterFP.h"
/******************************************************************************
 * Definitions & Declarations
 ******************************************************************************/

//! DEBUG MODE
/*! 0: No debug mode enabled
 * 	1: debug mode enabled for error output messages
 * 	2: debug mode enabled for both error and ok messages
 */
#define DEBUG_XTR_MODBUS		0

#define PRINT_XTR_MODBUS(str)		USB.print(F("[XTR-MODBUS] ")); USB.print(str);
#define PRINT_XTR_MODBUS_VAL(val)	USB.print(val, BIN);

#define PRINTLN_XTR_MODBUS(str)		USB.print(F("[XTR-MODBUS] ")); USB.println(str);
#define PRINTLN_XTR_MODBUS_VAL(val)	USB.println(val, BIN);


// Some sensor Registers
#define ADDRESS_REG					0xA3
#define WAITING_TIME_REG			0xA4
#define MEASUREMENTS_REG			0x53
#define EXT_MEASUREMENTS_REG		0x89
#define RESET_REG					0x02
#define TEMP_MEAS_STATUS			0x64
#define OPER_NAME_REG				0x027E
#define DATE_TEMP_REG				0x0286
#define SERIAL_NUMBER_REG			0x0D10
#define TEMP_TYPE_CON				0xA5
#define TURB_TYPE_CON				0xA6
#define AVRG_PARA_REG				0xAA
#define TURB_MEAS_STATUS			0x65
#define NEW_MEAS_REG				0x01
#define TEMP_COEF_LIST_REG			0x014C
#define RESTORE_CALIB_REG			0x02
#define COMP_TEMP_REG				0x5D
#define COMP_VAL_1_REG				0x5F
#define COMP_VAL_2_REG				0x61
#define PARAM1_MEAS_TYPE_CONFIG_REG	0xA6
#define PARAM2_MEAS_TYPE_CONFIG_REG	0xA7
#define PARAM3_MEAS_TYPE_CONFIG_REG	0xA8
#define PARAM4_MEAS_TYPE_CONFIG_REG	0xA9

#define MEAS_TYPE_COMP_TEMP_BIT		0x10
#define MEAS_TYPE_COMP_VAL_1_BIT	0x20
#define MEAS_TYPE_COMP_VAL_2_BIT	0x40


//NTU sensor standard calibration registers
#define CALIB_STANDARD_TEMP_1	0x0200 //Temp offset
#define CALIB_STANDARD_TEMP_2 	0x0202 //Temp slope
#define CALIB_STANDARD_1		0x0204
#define CALIB_STANDARD_2		0x0206
#define CALIB_STANDARD_3		0x0208
#define CALIB_STANDARD_4		0x020A
#define CALIB_STANDARD_5		0x020C
#define CALIB_STANDARD_6		0x020E
#define CALIB_STANDARD_7		0x0210
#define CALIB_STANDARD_8		0x0212
#define CALIB_STANDARD_9		0x0214
#define CALIB_STANDARD_10		0x0216

#define DONT_RETURN_AVG_TO_1	0
#define RETURN_AVG_TO_1			1
#define RETURN_AVG_TO_1_AND_STOP_ELECTRONIC_ZERO	2

//Parameters
#define PARAMETER_1 	1
#define PARAMETER_2 	2
#define PARAMETER_3		3
#define PARAMETER_4		4
#define TEMPERATURE		5

#define COMPENSATES_1		1
#define COMPENSATES_2		2
#define COMPENSATES_3		3
#define COMPENSATES_TEMP	5

//OPTOD COMPENSATION
#define COMPENSATE_OXYGEN		1 //LIKE PARAMETER_1
#define EXTERNAL_ATM_PRESSURE	1 //LIKE COMPENSATES_1
#define EXTERNAL_SALINITY		2 //LIKE COMPENSATES_2
#define EXTERNAL_TEMP			5 //LIKE COMPENSATES_TEMP

//PHEHT COMPENSATION
#define COMPENSATE_PH			1 		//LIKE PARAMETER_1

//NTU COMPENSATION
#define COMPENSATE_TURBIDITY	1 //LIKE PARAMETER_1

//CTZN COMPENSATION
#define COMPENSATE_CONDUCTIVITY	1 //LIKE PARAMETER_1
#define EXTERNAL_ALPHA			1 //LIKE COMPENSATES_1

//SAC
#define UV_254			1
#define VIS_530			2


#define PH				2
#define REDOX			3
#define CONDUCTIVITY	2
#define NTU_TURBIDITY	2
#define SLUDGE_BLANKET	2
#define FAU_TURBIDITY	4


#define OXYGEN			2
#define PH				2
#define REDOX			3
#define CONDUCTIVITY	2
#define NTU_TURBIDITY	2
#define SLUDGE_BLANKET	2
#define FAU_TURBIDITY	4

//Ranges for some measures
#define RANGE_AUTOMATIC 0
#define RANGE_1 		1
#define RANGE_2 		2
#define RANGE_3 		3
#define RANGE_4 		4


#define STEP_1 		1
#define STEP_2 		2
#define STEP_3 		3
#define STEP_4 		4
#define STEP_3B		10

// This address will be configured as a first step
#define DEFAULT_ADDRESS			40
#define DEFAULT_WAITING_TIME	500 // in ms


// Supported sensors
#define _5TE				1
#define GS3					2
#define VP4					3
#define MPS6				4
#define SO411				5
#define SI411				6
#define _5TM				7
#define SF421				8
#define C4E					9
#define CTZN				10
#define MES5				11
#define OPTOD				12
#define PHEHT				13
#define NTU					14
#define SAC					17

#define EUREKA_PH			0
#define EUREKA_ORP			1
#define EUREKA_DPTH			2
#define EUREKA_COND			3
#define EUREKA_HDO			4
#define EUREKA_CHL			5
#define EUREKA_NH4			6
#define EUREKA_NO3			7
#define EUREKA_CL			8


#define C21_LOW_POWER_MODE 0
#define C21_NORMAL_POWER_MODE 1
#define C21_DISTANCE_IN_M 0
#define C21_DISTANCE_IN_FT 1
#define C21_TEMPERATURE_IN_C 0
#define C21_TEMPERATURE_IN_F 1


/*******************************************************************************
 * Smart Agriculture Xtreme Structs
 ******************************************************************************/
/*!
 * \struct weatherStationVector
 * \brief Struct to store data of the GILL GMX weather stations
 */

struct weatherStationVector
{
	//! Variable: stores measured wind direction in degrees
	uint16_t windDirection;

	//! Variable: stores measured corrected wind direction in degrees
	uint16_t correctedWindDirection;

	//! Variable: stores measured average wind direction in degrees
	uint16_t avgWindDirection;

	//! Variable: stores measured wind speed in m/s
	float windSpeed;

	//! Variable: stores measured average wind speed in m/s
	float avgWindSpeed;

	//! Variable: stores measured average wind gust direction in degrees
	uint16_t avgWindGustDirection;

	//! Variable: stores measured wind gust speed in m/s
	float avgWindGustSpeed;

	//! Variable: stores measured average corrected wind direction in degrees
	uint16_t avgCorrectedWindDirection;

	//! Variable: stores measured wind sensor status
	char windSensorStatus[5];

	//! Variable: stores measured precipitation total in mm
	float precipTotal;

	//! Variable: stores measured precipitation intensity in mm
	float precipIntensity;

	//! Variable: stores measured precipitation status
	uint8_t precipStatus;

	//! Variable: stores measured compass in degrees
	uint16_t compass;

	//! Variable: stores measured x tilt
	float xTilt;

	//! Variable: stores measured y tilt
	float yTilt;

	//! Variable: stores measured z orientation
	float zOrient;

	//! Variable: stores measured timestamp
	char timestamp[22];

	//! Variable: stores measured supply voltage
	float supplyVoltage;

	//! Variable: stores measured status
	char status[5];

	//! Variable: stores measured solar radiation in W/m²
	uint16_t solarRadiation;

	//! Variable: stores measured sunshine hours
	float sunshineHours;

	//! Variable: stores measured sunrise time in h:min
	char sunriseTime[6];

	//! Variable: stores measured solar noon time in h:min
	char solarNoonTime[6];

	//! Variable: stores measured sunset time in h:min
	char sunsetTime[6];

	//! Variable: stores measured position of the sun in degrees:degrees
	char sunPosition[8];

	//! Variable: stores measured twilight civil time in h:min
	char twilightCivil[6];

	//! Variable: stores measured twilight nautical time in h:min
	char twilightNautical[6];

	//! Variable: stores measured twilight astronomical time in h:min
	char twilightAstronom[6];

	//! Variable: stores measured pressure in hPa
	float pressure;

	//! Variable: stores measured pressure at sea level in hPa
	float pressureSeaLevel;

	//! Variable: stores measured pressure at station level in hPa
	float pressureStation;

	//! Variable: stores measured relative humidity in %
	uint16_t relativeHumidity;

	//! Variable: stores measured  temperature in degrees celsius
	float temperature;

	//! Variable: stores measured dew point in degrees celsius
	float dewpoint;

	//! Variable: stores measured absolute humidity in gm-3
	float absoluteHumidity;

	//! Variable: stores measured air density in Kgm-3
	float airDensity;

	//! Variable: stores measured wet bulb temperature in degrees celsius
	float wetBulbTemperature;

	//! Variable: stores measured wind chill in celsius degrees
	float windChill;

	//! Variable: stores measured heat index in celsius degrees
	uint16_t heatIndex;


	#ifdef GMX_GPS_OPTION

		//! Variable: stores measured  gps corrected speed in m/s
		float gpsCorrectedSpeed;

		//! Variable: stores measured  gps average corrected speed in m/s
		float gpsAvgCorrectedSpeed;

		//! Variable: stores measured gps corrected gust speed in m/s
		float gpsCorrectedGustSpeed;

		//! Variable: stores measured gps corrected gust direction in degrees
		uint16_t gpsCorrectedGustDirection;

		//! Variable: stores measured gps location
		char gpsLocation[29];

		//! Variable: stores measured gps heading in degrees
		uint16_t gpsHeading;

		//! Variable: stores measured gps speed in m/s
		float gpsSpeed;

		//! Variable: stores measured gps status
		char gpsStatus[5];

	#endif
};

/*!
 * \struct sensor5TEVector
 * \brief Struct to store data of the 5TE sensor
 */

struct sensor5TEVector
{

	//! Variable: stores measured dielectric permittivity (dimensionless) in float type
	float dielectricPermittivity;

	//! Variable: stores measured electrical conductivity in dS/m in float type
	float electricalConductivity;

	//! Variable: stores measured temperature in degrees Celsius in float type
	float temperature;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};


/*!
 * \struct sensor5TMVector
 * \brief Struct to store data of the 5TM sensor
 */

struct sensor5TMVector
{

	//! Variable: stores measured dielectric permittivity (dimensionless) in float type
	float dielectricPermittivity;

	//! Variable: stores measured temperature in degrees Celsius in float type
	float temperature;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};

/*!
 * \struct sensorGS3Vector
 * \brief Struct to store data of the GS3 sensor
 */

struct sensorGS3Vector
{

	//! Variable: stores measured dielectric permittivity (dimensionless) in float type
	float dielectricPermittivity;

	//! Variable: stores measured electrical conductivity in μS/cm in float type
	float electricalConductivity;

	//! Variable: stores measured temperature in degrees Celsius in float type
	float temperature;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};

/*!
 * \struct sensorVP4Vector
 * \brief Struct to store data of the VP4 sensor
 */

struct sensorVP4Vector
{
	//! Variable: stores measured Vapor Pressure in kPa in float type
	float vaporPressure;

	//! Variable: stores measured temperature in degrees Celsius in float type
	float temperature;

	//! Variable: stores measured Relative Humidity in %RH in float type
	float relativeHumidity;

	//! Variable: stores measured Atmospheric Pressure in kPa in float type
	float atmosphericPressure;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};

/*!
 * \struct sensorMPS6Vector
 * \brief Struct to store data of the MPS6 sensor
 */

struct sensorMPS6Vector
{
	//! Variable: stores measured Water Potential in kPa in float type (it's always a negative value)
	float waterPotential;

	//! Variable: stores measured temperature in degrees Celsius in float type
	float temperature;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};

/*!
 * \struct sensorSO411Vector
 * \brief Struct to store data of the SO411 sensor
 */

struct sensorSO411Vector
{
	//! Variable: stores measured calibrated oxygen in % in float type
	float calibratedOxygen;

	//! Variable: stores measured sensor body temperature in degrees Celsius in float type
	float bodyTemperature;

	//! Variable: stores measured sensor millivolts in float type
	float milliVolts;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};


/*!
 * \struct sensorSI411Vector
 * \brief Struct to store data of the SI411 sensor
 */

struct sensorSI411Vector
{
	//! Variable: stores measured Target Temperature in degrees Celsius in float type
	float targetTemperature;

	//! Variable: stores measured sensor body temperature in degrees Celsius in float type
	float sensorBodyTemperature;

	//! Variable: stores measured target millivolts in float type
	float targetMilliVolts;

	//Sensor serial number variable
	char sensorSerialNumber[14];

  //! Variable: stores correctedTemperature in degrees Celsius in float type
  float targetTemperatureCorrected;
};


/*!
 * \struct sensorSI4B1Vector
 * \brief Struct to store data of the SI411 sensor
 */

struct sensorSI4B1Vector
{
	//! Variable: stores measured Target Temperature in degrees Celsius in float type
	float targetTemperature;

	//! Variable: stores measured sensor body temperature in degrees Celsius in float type
	float sensorBodyTemperature;

	//! Variable: stores measured target millivolts in float type
	float targetMilliVolts;

	//Sensor serial number variable
	char sensorSerialNumber[14];

  //! Variable: stores correctedTemperature in degrees Celsius in float type
  float targetTemperatureCorrected;
};


/*!
 * \struct sensorSF421Vector
 * \brief Struct to store data of the SF421 sensor
 */

struct sensorSF421Vector
{
	//! Variable: stores measured Bud Temperature in degrees Celsius in float type
	float budTemperature;

	//! Variable: stores measured Leaf Temperature in degrees Celsius in float type
	float leafTemperature;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};

/*!
 * \struct sensorSF421Vector
 * \brief Struct to store data of the SF421 sensor
 */

struct DatasolMETVector
{
	//! Variable: stores measured radiation in W/m2
	uint16_t radiation;

	//! Variable: stores measured semi-cell 1 radiation in W/m2
	uint16_t semicell1Radiation;

	//! Variable: stores measured semi-cell 2 radiation in W/m2
	uint16_t semicell2Radiation;

	//! Variable: stores measured envirnoment temperature in degrees Celsius
	float environmentTemperature;

	//! Variable: stores measured solar panel temperature in degrees Celsius
	float panelTemperature;

	//! Variable: stores measured wind speed in m/s
	float windSpeed;

	//! Variable: stores necessary cleaning notice
	bool necessaryCleaningNotice;

	//! Variable: stores Peak Sun Hours (PSH)
	float peakSunHours;

	//Sensor serial number variable
	char sensorSerialNumber[11];
};

/*!
 * \struct sensorTEROS12Vector
 * \brief Struct to store data of the TEROS12 sensor
 */

struct sensorTEROS12Vector
{

	//! Variable: stores measured dielectric permittivity (dimensionless) in float type
	float calibratedCountsVWC;

	//! Variable: stores measured electrical conductivity in μS/cm in float type
	float electricalConductivity;

	//! Variable: stores measured temperature in degrees Celsius in float type
	float temperature;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};


/*!
 * \struct sensor5TMVector
 * \brief Struct to store data of the 5TM sensor
 */

struct sensorTEROS11Vector
{

  //! Variable: stores measured dielectric permittivity (dimensionless) in float type
  float calibratedCountsVWC;

	//! Variable: stores measured temperature in degrees Celsius in float type
	float temperature;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};




/*!
 * \struct VegaPulsC21Vector
 * \brief Struct to store data of the Vegapuls C 21 sensor
 */

struct VegaPulsC21Vector
{

	//! Variable: stores measured stage in float type
	float stage;

	//! Variable: stores measured distance in float type
	float distance;

	//! Variable: stores measured temperature in float type
	float temperature;

	//! Variable: stores measured status in float type
	float status;

	//! Variable: stores stage reference in float type
	float stageReference;

	//! Variable: stores stage reference in float type
	uint8_t powerOperationMode;

	//! Variable: stores stage reference in float type
	uint8_t distanceUnit;

	//! Variable: stores stage reference in float type
	uint8_t temperatureUnit;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};

/*******************************************************************************
 * Smart Water Xtreme Structs
 ******************************************************************************/


/*!
 * \struct sensorOPTODVector
 * \brief Struct to store data of the OPTOD sensor
 */

struct sensorOPTODVector
{
	//! Variable: stores measured temperature in Celsius degrees
	float temperature;

	//! Variable: stores measured oxygen in saturation percentage
	float oxygenSAT;

	//! Variable: stores measured oxygen in mg/L
	float oxygenMGL;

	//! Variable: stores measured oxygen in ppm
	float oxygenPPM;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};




/*!
 * \struct sensorPHEHTVector
 * \brief Struct to store data of the PHEHT sensor
 */

struct sensorPHEHTVector
{
	//! Variable: stores measured temperature in Celsius degrees
	float temperature;

	//! Variable: stores measured pH
	float pH;

	//! Variable: stores measured redox in mV
	float redox;

	//! Variable: stores measured pH in mV
	float pHMV;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};


/*!
 * \struct sensorPHEHTVector
 * \brief Struct to store data of the PHEHT sensor
 */

struct sensorSACVector
{
	//! Variable: stores measured temperature in Celsius degrees
	float temperature;

	//! Variable: stores measured sac
	float sac;

	//! Variable: stores measured cod in mV
	float cod;

	//! Variable: stores measured pH in mV
	float bod;
	
	//! Variable: stores measured cod in mV
	float cot;

	//! Variable: stores measured pH in mV
	float uvComp;
	
	//! Variable: stores measured cod in mV
	float grComp;

	//! Variable: stores measured pH in mV
	float uvTran;
	
	//! Variable: stores measured cod in mV
	float grTran;

	//! Variable: stores measured pH in mV
	float turb;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};

/*!
 * \struct sensorC4EVector
 * \brief Struct to store data of the C4E sensor
 */

struct sensorC4EVector
{
	//! Variable: stores measured temperature in Celsius degrees
	float temperature;

	//! Variable: stores measured conductivity in μS/cm
	float conductivity;

	//! Variable: stores measured salinity in ppt
	float salinity;

	//! Variable: stores measured total dissolved solids in ppm
	float totalDissolvedSolids;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};




/*!
 * \struct sensorNTUVector
 * \brief Struct to store data of the NTU sensor
 */

struct sensorNTUVector
{
	//! Variable: stores measured temperature in Celsius degrees
	float temperature;

	//! Variable: stores measured turbidity in NTU
	float turbidityNTU;

	//! Variable: stores measured turbidity in mg/l
	float turbidityMGL;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};





/*!
 * \struct sensorCTZNVector
 * \brief Struct to store data of the CTZN sensor
 */

struct sensorCTZNVector
{
	//! Variable: stores measured temperature in Celsius degrees
	float temperature;

	//! Variable: stores measured conductivity in mS/cm
	float conductivity;

	//! Variable: stores measured salinity in ppt
	float salinity;

	//! Variable: stores measured conductivity not compensated with temperature in mS/cm
	float conductivityNotCompensated;

	//Sensor serial number variable
	char sensorSerialNumber[14];
 };



/*!
 * \struct sensorMES5Vector
 * \brief Struct to store data of the MES5 sensor
 */

struct sensorMES5Vector
{
	//! Variable: stores measured temperature in Celsius degrees
	float temperature;

	//! Variable: stores measured sludge blanket in %
	float sludgeBlanket;

	//! Variable: stores measured supended solids in g/L
	float suspendedSolids;

	//! Variable: stores measured turbidity in FAU
	float turbidityFAU;

	//Sensor serial number variable
	char sensorSerialNumber[14];
};


/*!
 * \struct sensorEurekaVector
 * \brief Struct to store data of the Eureka sensor
 */
struct sensorEurekaVector
{
	// sensor variables
	float ph;
	float orp;
	float depth;
	float spCond;
	float chl;
	float nh4;
	float no3;
	float cl;
	float hdo;
	float temperature;
	char ME_date[10];
	char ME_time[10];

	char pressure[10];

	char model[20];

	char version[7];

	//Sensor serial number variable
	char sensorSerialNumber[9];


};

/*******************************************************************************
 * AqualaboSensor Classes
 ******************************************************************************/

/*!
 * \class AqualaboWaterXtr
 * \brief class for Aqualabo sensors in Water Xtreme board
 */
class aqualaboModbusSensorsClass
{
	public:
		//! Constructor
		aqualaboModbusSensorsClass();

		//Modbus
		void begin(uint8_t slave, Stream&);
        void begin(uint8_t slave, int dePin, int rePin, Stream&);
		uint8_t initSensor();
		uint8_t initSensor(uint8_t range);
		uint8_t initSensor(uint8_t range, uint8_t avg);
		uint8_t calibrate(uint8_t sensor, uint8_t parameter, uint8_t step, float value);
		void fillOperatorsName(char* name);
		void fillCalibrationDate(char* date);
		uint8_t restoreToFactoryCalibration(uint8_t parameter);
		uint8_t resetTemporaryCalibrationData(uint8_t returnAvgTo1);
		uint8_t searchAddress(uint8_t _sensorAddr);
		uint8_t changeAddress(uint8_t _sensorAddr);

		//Menu assisted calibration functions
		void exitCalibration();
		void exitCalibrationAndStopElectronicZero();
		void printLine();
		void printBigLine();
		uint8_t getData(char* input, uint8_t inputLength);
		boolean getDate(char* input, uint8_t inputLength, int numBytes);
		void serialClean();
		bool find( uint8_t* buffer, uint16_t length, char* pattern);

		//Parameter compensation with external values
		uint8_t enableCompensation(uint8_t compensatedParam, uint8_t extParamWithWhichCompensate, uint8_t enablingState);
		uint8_t setCompensationValue(uint8_t extParamWithWhichCompensate, float value);

	protected:
		//Parameter compensation with external values
		uint8_t compensationTemp = 0x80;
		uint8_t compensation1 = 0x80;
		uint8_t compensation2 = 0x80;
		uint8_t saveCompensationValue(uint8_t paramNumber, uint8_t _compensationTemp, uint8_t _compensation1, uint8_t _compensation2);
};

/*!
 * \class Aqualabo OPTOD
 * \brief class for OPTOD sensor
 */
class Aqualabo_OPTOD: public aqualaboModbusSensorsClass
{
	public:
		// constructor
		Aqualabo_OPTOD(uint8_t _socket);

		sensorOPTODVector sensorOPTOD;

		uint8_t ON();
		void OFF();
		uint8_t read();
		uint8_t readSerialNumber();
		void calibrationProcess(uint8_t parameter);

	private:
	  Stream& _modbusport; 
	  ModbusMasterFP sensor;                                           ///< reference to serial port object
      uint8_t  _u8MBSlave;
      int _dePin, _rePin;

      bool _transmisionBegun;
      unsigned long _baudrate;
      uint16_t _config;
};




/*!
 * \class Aqualabo PHEHT
 * \brief class for PHEHT sensor
 */
class Aqualabo_PHEHT: public aqualaboModbusSensorsClass
{
	public:
		// constructor
		Aqualabo_PHEHT(uint8_t _socket);

		sensorPHEHTVector sensorPHEHT;

		uint8_t ON();
		void OFF();
		uint8_t read();
		uint8_t readSerialNumber();
		void calibrationProcess(uint8_t parameter);

	private:
	  Stream& _modbusport; 
	  ModbusMasterFP sensor;                                           ///< reference to serial port object
      uint8_t  _u8MBSlave;
      int _dePin, _rePin;

      bool _transmisionBegun;
      unsigned long _baudrate;
      uint16_t _config;
};


/*!
 * \class Aqualabo PHEHT
 * \brief class for PHEHT sensor
 */
class Aqualabo_SAC: public aqualaboModbusSensorsClass
{
	public:
		// constructor
		Aqualabo_SAC(uint8_t _socket);

		sensorSACVector sensorSAC;
		uint8_t socket;

		uint8_t ON();
		void OFF();
		uint8_t read();
		uint8_t readSerialNumber();
		void calibrationProcess(uint8_t parameter);

	private:
	  Stream& _modbusport; 
	  ModbusMasterFP sensor;                                           ///< reference to serial port object
      uint8_t  _u8MBSlave;
      int _dePin, _rePin;

      bool _transmisionBegun;
      unsigned long _baudrate;
      uint16_t _config;
		
};



/*!
 * \class Aqualabo C4E
 * \brief class for C4E sensor
 */
class Aqualabo_C4E: public aqualaboModbusSensorsClass
{
	public:
		// constructor
		Aqualabo_C4E(uint8_t _socket);

		sensorC4EVector sensorC4E;

		uint8_t ON();
		void OFF();
		uint8_t read();
		uint8_t readSerialNumber();
		void calibrationProcess(uint8_t parameter);

	private:
	  Stream& _modbusport; 
	  ModbusMasterFP sensor;                                           ///< reference to serial port object
      uint8_t  _u8MBSlave;
      int _dePin, _rePin;

      bool _transmisionBegun;
      unsigned long _baudrate;
      uint16_t _config;
};



/*!
 * \class Aqualabo NTU
 * \brief class for NTU sensor
 */
class Aqualabo_NTU: public aqualaboModbusSensorsClass
{
	public:
		// constructor
		Aqualabo_NTU(uint8_t _socket);

		sensorNTUVector sensorNTU;

		uint8_t ON();
		void OFF();
		uint8_t read();
		uint8_t readSerialNumber();
		void calibrationProcess(uint8_t parameter);

	private:
	  Stream& _modbusport; 
	  ModbusMasterFP sensor;                                           ///< reference to serial port object
      uint8_t  _u8MBSlave;
      int _dePin, _rePin;

      bool _transmisionBegun;
      unsigned long _baudrate;
      uint16_t _config;

};


/*!
 * \class Aqualabo CTZN
 * \brief class for CTZN sensor
 */
class Aqualabo_CTZN: public aqualaboModbusSensorsClass
{
	public:
		// constructor
		Aqualabo_CTZN(uint8_t _socket);

		sensorCTZNVector sensorCTZN;

		uint8_t ON();
		void OFF();
		uint8_t read();
		uint8_t readSerialNumber();
		void calibrationProcess(uint8_t parameter);

	private:
	  Stream& _modbusport; 
	  ModbusMasterFP sensor;                                           ///< reference to serial port object
      uint8_t  _u8MBSlave;
      int _dePin, _rePin;

      bool _transmisionBegun;
      unsigned long _baudrate;
      uint16_t _config;
};


/*!
 * \class Aqualabo MES5
 * \brief class for MES5 sensor
 */
class Aqualabo_MES5: public aqualaboModbusSensorsClass
{
	public:
		// constructor
		Aqualabo_MES5(uint8_t _socket);

		sensorMES5Vector sensorMES5;

		uint8_t ON();
		void OFF();
		uint8_t read();
		uint8_t readSerialNumber();
		void calibrationProcess(uint8_t parameter);

	private:
	  Stream& _modbusport; 
	  ModbusMasterFP sensor;                                           ///< reference to serial port object
      uint8_t  _u8MBSlave;
      int _dePin, _rePin;

      bool _transmisionBegun;
      unsigned long _baudrate;
      uint16_t _config;
};


/*!
 * \class Eureka_Manta
 * \brief class for Eureka Manta 2
 */
class Eureka_Manta
{
	public:
		// constructor
		Eureka_Manta();

		sensorEurekaVector sensorEureka;

		uint8_t ON();
		void OFF();
		uint8_t read();
		uint8_t sendCommand(char* str);
		void configureSensor();
		uint8_t saveConfig();
		uint8_t getBarometricPressure();
		uint8_t setBarometricPressure(float pressure);


	private:
		uint8_t socket;


};


// object to manage Modbus sensors in Xtreme boards
extern aqualaboModbusSensorsClass aqualaboModbusSensors;

#endif
