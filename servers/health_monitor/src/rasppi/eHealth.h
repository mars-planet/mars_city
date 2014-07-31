/*
 *  eHealth sensor platform for Arduino and Raspberry from Cooking-hacks.
 *
 *  Description: "The e-Health Sensor Shield allows Arduino and Raspberry Pi 
 *  users to perform biometric and medical applications by using 9 different 
 *  sensors: Pulse and Oxygen in Blood Sensor (SPO2), Airflow Sensor (Breathing),
 *  Body Temperature, Electrocardiogram Sensor (ECG), Glucometer, Galvanic Skin
 *  Response Sensor (GSR - Sweating), Blood Pressure (Sphygmomanometer) and 
 *  Patient Position (Accelerometer)."
 *
 *  Copyright (C) 2012 Libelium Comunicaciones Distribuidas S.L.
 *  http://www.libelium.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Version 2.0
 *  Author: Luis Martín & Ahmad Saad & Anartz Nuin
 */


// Ensure this library description is only included once
#ifndef eHealthClass_h
#define eHealthClass_h

#include "arduPi.h"
#include <math.h>

#ifdef SWIG
%rename(ar_Digivalue) DigivalueNS;
#endif

struct DigivalueNS
{
	enum Value {
		LOW = 0,
		HIGH = 1,
		RISING = 2,
		FALLING = 3,
		BOTH = 4
	};
};
typedef DigivalueNS::Value ar_Digivalue;

//wrappers for arduino functions
/*
#define ar_attachInterrupt attachInterrupt
#define ar_detachInterrupt detachInterrupt
#define ar_digitalWrite digitalWrite
#define ar_digitalRead digitalRead
#define ar_analogRead analogRead
#define ar_delay delay
#define ar_delayMicroseconds delayMicroseconds
#define ar_millis millis
*/

void ar_delay(long millis);
void ar_delayMicroseconds(long micros);
long ar_millis();

//!Struct to store data of the glucometer.
struct glucoseData {
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minutes;
	uint8_t glucose;
	uint8_t meridian;
};

//!Struct to store data of the blood pressure sensor.
struct bloodPressureData {
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minutes;
	uint8_t systolic;
	uint8_t diastolic;
	uint8_t pulse;
};

// Library interface description
class eHealthClass {
	
	public: 
  
	//***************************************************************
	// Constructor of the class										*
	//***************************************************************
  
		//! Class constructor.
		eHealthClass(void);
		 
	//***************************************************************
	// Public Methods												*
	//***************************************************************

		//! Initializes the position sensor and configure some values.
		/*!
		\param void
		\return void
		*/
		static void initPositionSensor(void);
		 		
		//! Initializes the BloodPressureSensor sensor and configure some values
		/*!
		\param void
		\return void
		*/
		static void readBloodPressureSensor(void);

		//! Initializes the pulsioximeter sensor and configure some values.
		/*!
		\param void
		\return void
		*/
		static void initPulsioximeter(int pulsioxSkipReadings);

		//! Sets a loop to read from the pulsioximeter sensor.
		/*!
		\param void
		\return void
		*/
		static void readPulsioximeterLoop();

		//! It reads a value from pulsioximeter sensor.
		/*!
		\param void
		\return void
		*/
		static void readPulsioximeter(void);

		//! sets up the device for the next reading.
		/*!
		\param void
		\return void
		*/
		static void setupPulsioximeterForNextReading();

		//! Returns the corporal temperature.
		/*!
		\param void
		\return float : The corporal temperature value.   
		*/
		static float getTemperature( void );
		
		//! Returns the oxygen saturation in blood in percent.
		/*!
		\param void
		\return int : The oxygen saturation value. Normal values betwen 95-99%  
		*/
		static int getOxygenSaturation(void);

		//! Returns the heart beats per minute. 
		/*!
		\param void
		\return int : The beats per minute. 
		*/
		static int getBPM(void);
 
		//! Returns the value of skin conductance. 
		/*!
		\param void
		\return float : The skin conductance value.  
		*/
		static float getSkinConductance(void);
		
		//! Returns the value of skin resistance. 
		/*!
		\param void
		\return float : The skin resistance value.  
		*/
		static float getSkinResistance (void);
		
		//! Returns the value of skin conductance in voltage. 
		/*!
		\param void
		\return float : The skin conductance value in voltage (0-5v).  
		*/
		static float getSkinConductanceVoltage(void);

		//! Returns an analogic value to represent the Electrocardiography.
		/*!
		\param void
		\return float : The analogic value (0-5V).  
		*/
		static float getECG(void);

		//! Returns an analogic value to represent the Electromyography.
		/*!
		\param void
		\return float : The analogic value (0-5V).  
		*/
		static int getEMG(void);

		//! Returns the body position.
		/*!
		\param void   
		\return uint8_t : the position of the pacient.
		 *		1 == Supine position.
		 *		2 == Left lateral decubitus.
		 *		3 == Rigth lateral decubitus.
		 *		4 == Prone position.
		 *		5 == Stand or sit position
		 */
		static uint8_t getBodyPosition(void);

		//! Returns the body 3-axis acceleration.
		/*!
		\param void
		\return uint8_t : the 3-axis acceleration of the pacient.
		*/
		static float* getBodyAcceleration(void);

		//! Returns the  value of the systolic pressure.
		/*!
		\param void   
		\return int : The systolic pressure.
		*/
		static int getSystolicPressure(int i);

		//! Returns the  value of the diastolic pressure.
		/*!
		\param void   
		\return int : The diastolic pressure.
		*/
		static int getDiastolicPressure(int i);

		//! Returns an analogic value to represent the air flow.
		/*!
		\param void   
		\return int : The value (0-1023) read from the analogic in.  
		*/
		static int getAirFlow(void);

		//! Prints the current body position 
		/*!
		\param uint8_t position : the current body position.   
		\return void  
		*/
		static void printPosition(uint8_t position);

		//!  Prints air flow wave form in the serial monitor
		/*!
		\param int air : analogic value to print. 
		\return void 
		*/
		static void airFlowWave(int air);

		//!  Read the values stored in the glucometer. 
		/*!
		\param void
		\return void
		*/
		static void readGlucometer(void);

		//!Returns the number of data stored in the glucometer.
		/*!
		\param void
		\return int : length of data 
		*/
		static int getGlucometerLength(void);

		//!Returns the number of data stored in the blood pressure sensor.
		/*!
		\param void
		\return int : length of data 
		*/
		static int getBloodPressureLength(void);

		//!  Returns the library version 
		/*!
		\param void
		\return int : The library version. 
		*/
		static int version(void);

		//! Convert month variable from numeric to character.
		/*!
		 \param int month in numerical format.
		 \return String with the month characters (January, February...).
		 */
		static const char* numberToMonth(int month);

		//!Vector to store the glucometer measures and dates.
		static glucoseData glucoseDataVector[8];

		//!Vector to store the blood pressure measures and dates.
		static bloodPressureData bloodPressureDataVector[8];

	private:

	//***************************************************************
	// Private Methods												*
	//***************************************************************
		//! Initialize the MMA8452 registers 
		static void initMMA8452(char fsr, char dataRate);

		//! Sets the MMA8452 to standby mode. It must be in standby to change most register settings.
		static void MMA8452Standby();
		
		//! Sets the MMA8452 to active mode. Needs to be in this mode to output data.
		static void MMA8452Active();

		//! Read i registers sequentially, starting at address into the dest byte array
		static void readRegisters(char address, int i, char * dest);

		//! Read a single byte from address and return it as a byte.
		static char readRegister(char address);

		//! Writes a single byte (data) into address.
		static void writeRegister(unsigned char address, unsigned char data);

		//! This function will read the p/l source register and
		//! print what direction the sensor is now facing. 
		static void portraitLandscapeHandler();

		//! Assigns a value depending on body position.
		static void bodyPosition(void);
		
		//! Converts from 7 segments to number.
		static uint8_t segToNumber(uint8_t A,
							uint8_t B,
							uint8_t C,
							uint8_t D,
							uint8_t E,
							uint8_t F,
							uint8_t G );

		//! Assigns a value depending on body position.
		static char swap(char _data);

	//***************************************************************
	// Private Variables											*
	//***************************************************************

		//! It stores the systolic pressure value
		static int systolic;

		//! It stores the diastolic pressure value
		static int diastolic;

		//! Sets the maximum number of readings to skip.
		static int pulsioxSkipReadings;

		//! stores reading count.
		static int pulsioxReadingCount;

		//! It stores the  beats per minute value.
		static int BPM;
		
		//! It stores blood oxigen saturation value.
		static int SPO2;
		
		//! It stores current body position.
		static uint8_t bodyPos;

		//! x/y/z accel register data store here.
		static char data[6];

		//! Stores the 12-bit signed value.
		static int accelCount[3];

		//! Stores the real accel value in g's.
		static float accel[3];

		//! Stores the body position in vector value. 
		static uint8_t position[];

		//!It stores the number of data of the glucometer.
		static uint8_t length;
};

//extern eHealthClass eHealth;

#endif

