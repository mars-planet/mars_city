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


// include this library's description file
#include "eHealth.h"


void ar_attachInterrupt(int p, void (*f)(), ar_Digivalue m){
	attachInterrupt(p, f, static_cast<Digivalue>(m));
}

void ar_detachInterrupt(int p){
	detachInterrupt(p);
}

void ar_digitalWrite(int pin, int value){
	digitalWrite(pin, value);
}

int ar_digitalRead(int pin){
	return digitalRead(pin);
}

int ar_analogRead(int pin){
	return analogRead(pin);
}

void ar_delay(long millis){
	delay(millis);
}

void ar_delayMicroseconds(long micros){
	delayMicroseconds(micros);
}

long ar_millis(){
	return millis();
}


//***************************************************************
// Accelerometer Variables and definitions						*
//***************************************************************

	
	//! Breakout board defaults to 1, set to 0 if SA0 jumper is set
	#define SA0 1  
		#if SA0
			#define MMA8452_ADDRESS 0x1D  //! SA0 is high, 0x1C if low
		#else
			#define MMA8452_ADDRESS 0x1C  
		#endif

	#define	int1Pin 2
	#define int2Pin 3

	//! Set the scale below either 2, 4 or 8.
	const byte scale = 2;

	//! Set the output data rate below. Value should be between 0 and 7.
	//! 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
	const byte dataRate = 0;

//***************************************************************
// static members
//***************************************************************

	//!Vector to store the glucometer measures and dates.
	glucoseData eHealthClass::glucoseDataVector[8];

	//!Vector to store the blood pressure measures and dates.
	bloodPressureData eHealthClass::bloodPressureDataVector[8];

	int eHealthClass::systolic;

	//! It stores the diastolic pressure value
	int eHealthClass::diastolic;

	//! Sets the maximum number of readings to skip.
	int eHealthClass::pulsioxSkipReadings;

	//! stores reading count.
	int eHealthClass::pulsioxReadingCount;

	//! It stores the  beats per minute value.
	int eHealthClass::BPM;

	//! It stores blood oxigen saturation value.
	int eHealthClass::SPO2;

	//! It stores current body position.
	uint8_t eHealthClass::bodyPos;

	//! x/y/z accel register data store here.
	char eHealthClass::data[6];

	//! Stores the 12-bit signed value.
	int eHealthClass::accelCount[3];

	//! Stores the real accel value in g's.
	float eHealthClass::accel[3];

	//! Stores the body position in vector value.
	uint8_t eHealthClass::position[3];

	//!It stores the number of data of the glucometer.
	uint8_t eHealthClass::length;
//***************************************************************
// Constructor of the class										*
//***************************************************************

	//! Function that handles the creation and setup of instances
	eHealthClass::eHealthClass(void) { /*void constructor*/ }
	

//***************************************************************
// Public Methods												*
//***************************************************************

	//!******************************************************************************
	//!	Name:	initPositionSensor()												*
	//!	Description: Initializes the position sensor and configure some values.		*
	//!	Param : void																*
	//!	Returns: void																*
	//!	Example: eHealth.initPositionSensor();										*
	//!******************************************************************************

	void eHealthClass::initPositionSensor(void)
	{
		Wire.begin();
		byte c;
		/* Set up the interrupt pins, they're set as active high, push-pull */
		pinMode(int1Pin, INPUT);
		digitalWrite(int1Pin, LOW);
		pinMode(int2Pin, INPUT);
		digitalWrite(int2Pin, LOW);
		/* Read the WHO_AM_I register, this is a good test of communication */
		c = readRegister(0x0D);  // Read WHO_AM_I register
		if (c == 0x2A)	{ // WHO_AM_I should always be 0x2A
			initMMA8452(scale, dataRate);  // init the accelerometer if communication is good
			printf("MMA8452Q is online...\n");
		} else {
			printf("Could not connect to MMA8452Q: 0x%X\n",c);
			//while (1);  // Loop forever if communication doesn't happen
		}
	}


	//!******************************************************************************
	//!	Name:	readBloodPressureSensor()											*
	//!	Description: Initializes the BloodPressureSensor sensor.					*
	//!	Param : void																*
	//!	Returns: void																*
	//!	Example: eHealth.initBloodPressureSensor();									*
	//!******************************************************************************

	void eHealthClass::readBloodPressureSensor(void)
	{	
		unsigned char _data=0;
		int ia=0;
		eHealthClass::length=0;

		Serial.begin(19200);
	 	Serial.write(0xAA);
		delayMicroseconds(1);
	 	Serial.write(0x55);
		delayMicroseconds(1);
		Serial.write(0x88);
	 	delay(2500);
		Serial.print("\n");

		if (Serial.available() > 0) { // The protocol sends the measures 

			for (int i = 0; i<4; i++){ // Read four dummy data	
				Serial.read();
			}
			
 			while(_data != 0xD1){
  				if (ia==0){
   				_data = Serial.read();
  				}

  			eHealthClass::bloodPressureDataVector[length].year = swap(_data);
  			eHealthClass::bloodPressureDataVector[length].month = swap(Serial.read());
  			eHealthClass::bloodPressureDataVector[length].day = swap(Serial.read());
  			eHealthClass::bloodPressureDataVector[length].hour = swap(Serial.read());
  			eHealthClass::bloodPressureDataVector[length].minutes = swap(Serial.read());
  			eHealthClass::bloodPressureDataVector[length].systolic = swap(Serial.read());
  			eHealthClass::bloodPressureDataVector[length].diastolic = swap(Serial.read());
  			eHealthClass::bloodPressureDataVector[length].pulse = swap(Serial.read());
  			eHealthClass::length++;
			ia=1;
			for (int i = 0; i<4; i++){ // CheckSum 1
				Serial.read();
			}
			
			_data = Serial.read();
			}


 			for (int i = 0; i<3; i++){ // CheckSum 2
  				Serial.read();
 			}	
			
		}
	}

	//!******************************************************************************
	//!		Name:	initPulsioximeter()												*
	//!		Description: Initializes the pulsioximeter sensor.						*
	//!		Param : void															*
	//!		Returns: void															*
	//!		Example: eHealth.initPulsioximeter();									*
	//!******************************************************************************

	void eHealthClass::initPulsioximeter(int pulsioxSkipReadings)
	{
		// Configuring digital pins like INPUTS
		pinMode(13, INPUT);		pinMode(12, INPUT);
		pinMode(11, INPUT);		pinMode(10, INPUT);
		pinMode( 9, INPUT);		pinMode( 8, INPUT);
		pinMode( 7, INPUT);		pinMode( 6, INPUT);
		// attach a PinChange Interrupt to our pin on the rising edge
		eHealthClass::pulsioxSkipReadings = pulsioxSkipReadings;
		eHealthClass::pulsioxReadingCount = 0;
		attachInterrupt(6, eHealthClass::readPulsioximeterLoop, RISING);
	}

	void eHealthClass::setupPulsioximeterForNextReading(){
		digitalWrite(2, HIGH);
	}
	//!******************************************************************************
	//!		Name:	getTemperature()												*
	//!		Description: Returns the corporal temperature.							*
	//!		Param : void															*
	//!		Returns: float with the corporal temperature value.						*
	//!		Example: float temperature = eHealth.getTemperature();					*
	//!******************************************************************************

	float eHealthClass::getTemperature(void)
	{	
		//Local variables
		float Temperature=-1.0; //Corporal Temperature
		float Resistance;  //Resistance of sensor.
		float ganancia=5.0;
		float Vcc=3.3;
		float RefTension=3.0; // Voltage Reference of Wheatstone bridge.
		float Ra=4645.0; //Wheatstone bridge resistance.
		float Rc=4725.0; //Wheatstone bridge resistance.
		float Rb=814.0; //Wheatstone bridge resistance.
		int sensorValue = analogRead(3);

		float voltage2=((float)sensorValue*Vcc)/1023; // binary to voltage conversion  

		// Wheatstone bridge output voltage.
		voltage2=voltage2/ganancia;
		// Resistance sensor calculate  
		float aux=(voltage2/RefTension)+Rb/(Rb+Ra);
		Resistance=Rc*aux/(1-aux);    
		if (Resistance >=1822.8) {
			// if temperature between 25ºC and 29.9ºC. R(tª)=6638.20457*(0.95768)^t
			Temperature=log(Resistance/6638.20457)/log(0.95768);  
		} else {
			if (Resistance >=1477.1){
					// if temperature between 30ºC and 34.9ºC. R(tª)=6403.49306*(0.95883)^t
					Temperature=log(Resistance/6403.49306)/log(0.95883);  
			} else {
				if (Resistance >=1204.8){
					// if temperature between 35ºC and 39.9ºC. R(tª)=6118.01620*(0.96008)^t
					Temperature=log(Resistance/6118.01620)/log(0.96008); 
				}
				else{
					if (Resistance >=988.1){
						// if temperature between 40ºC and 44.9ºC. R(tª)=5859.06368*(0.96112)^t
						Temperature=log(Resistance/5859.06368)/log(0.96112); 
					}
					else {
						if (Resistance >=811.7){
							// if temperature between 45ºC and 50ºC. R(tª)=5575.94572*(0.96218)^t
							Temperature=log(Resistance/5575.94572)/log(0.96218); 
						}
					}
				}
			}  
		}

		return Temperature;
	}

	//!******************************************************************************
	//!		Name:	getOxygenSaturation()											*
	//!		Description: Returns the oxygen saturation in blood in percent.			*
	//!		Param : void															*
	//!		Returns: int with the oxygen saturation value							*
	//!		Example: int SPO2 = eHealth.getOxygenSaturation();						*
	//!******************************************************************************

	int eHealthClass::getOxygenSaturation(void)
	{
		return SPO2;
	}

	//!******************************************************************************
	//!		Name:	getBPM()														*
	//!		Description: Returns the heart beats per minute.						*
	//!		Param : void															*
	//!		Returns: int with the beats per minute									*
	//!		Example: int BPM = eHealth.getBPM();									*
	//!******************************************************************************

	int eHealthClass::getBPM(void)
	{
		return BPM;
	}


	//!******************************************************************************
	//!		Name:	getSkinConductance()											*
	//!		Description: Returns the value of skin conductance.						*
	//!		Param : void															*
	//!		Returns: float with the value of skin conductance						*
	//!		Example: float conductance = eHealth.getSkinConductance();				*
	//!******************************************************************************

	float eHealthClass::getSkinConductance(void)
	{
		// Local variable declaration.   
		// float resistance;
		float conductance;
		delay(1);
		
		// Read an analogic value from analogic2 pin.
		float sensorValue = analogRead(2);
		float voltage = sensorValue*5.0/1023;

		conductance = 2*((voltage - 0.5) / 100000);

		// Conductance calculation
		// resistance = 1 / conductance;
		conductance = conductance * 1000000;
		delay(1);
		
		if (conductance > 1.0) 	return conductance;
		else return -1.0;
	}


	//!******************************************************************************
	//!		Name:	getSkinResistance()												*
	//!		Description: Returns the value of skin resistance.						*
	//!		Param : void															*
	//!		Returns: float with the value of skin resistance						*
	//!		Example: float resistance = eHealth.getSkinResistance();				*
	//!******************************************************************************

	float eHealthClass::getSkinResistance(void)
	{	
		// Local variable declaration.   
		float resistance;
		float conductance;
	
		// Read an analogic value from analogic2 pin.
		float sensorValue = analogRead(2);
		float voltage = (sensorValue * 5.0) / 1023; 
	
		delay(2);
		conductance = 2*((voltage - 0.5) / 100000);
   
		//Conductance calcultacion
		resistance = 1 / conductance;
		delay(2);
	
		if (resistance > 1.0 ) return resistance;
		else return -1.0;
	}

	
	//!******************************************************************************
	//!		Name:	getSkinConductanceVoltage()										*
	//!		Description: Returns the skin conductance value in voltage .			*
	//!		Param : void															*
	//!		Returns: float with the skin conductance value in voltage 				*
	//!		Example: float volt = eHealth.getSkinConductanceVoltage();				*
	//!******************************************************************************

	float eHealthClass::getSkinConductanceVoltage(void)
	{
		delay(2);
	
		//Read analogic value from analogic2 pin.
		int sensorValue = analogRead(2);
	
		//Convert the readed value to voltage.
		float voltage = ( sensorValue * 5.0 ) / 1023;

		delay(2);
		return voltage;
	}


	//!******************************************************************************
	//!		Name:	getECG()														*
	//!		Description: Returns an analogic value to represent the ECG.			*
	//!		Param : void															*
	//!		Returns: float with the ECG value in voltage			 				*
	//!		Example: float volt = eHealth.getECG();									*
	//!******************************************************************************

	float eHealthClass::getECG(void)
	{
		float analog0;
		// Read from analogic in. 
		analog0=analogRead(0);
		// binary to voltage conversion
		return analog0 = (float)analog0 * 5 / 1023.0;   
	}
	

	//!******************************************************************************
	//!		Name:	getEMG()														*
	//!		Description: Returns an analogic value to represent the EMG.			*
	//!		Param : void															*
	//!		Returns: float with the EMG value in voltage			 				*
	//!		Example: float volt = eHealth.getEMG();									*
	//!******************************************************************************

	int eHealthClass::getEMG(void)
	{
		int analog0;
		// Read from analogic in. 
		analog0=analogRead(0);
		// binary to voltage conversion
		return analog0;   
	}
	

	//!******************************************************************************
	//!		Name:	getBodyPosition()												*
	//!		Description: Returns the current body position.							*
	//!		Param : void															*
	//!		Returns: uint8_t with the the position of the pacient.	 				*
	//!		Example: uint8_t position = eHealth.getBodyPosition();					*
	//!******************************************************************************

	uint8_t eHealthClass::getBodyPosition(void)
	{
		static byte source;
		// eHealthClass::accel will be modified inside this function
		eHealthClass::getBodyAcceleration();
		
		/* If int2 goes high, either p/l has changed or there's been a single/double tap */
		if (digitalRead(int2Pin)) {
			source = readRegister(0x0C);  // Read the interrupt source reg.
			
			if ((source & 0x10)==0x10)  // If the p/l bit is set, go check those registers
				portraitLandscapeHandler();      

			delay(50); // Delay here for a little printing visibility, make it longer, or delete it
		}

		delay(100);
  
		return bodyPos; 
	}

	//!******************************************************************************
	//!		Name:	getBodyAcceleration()												*
	//!		Description: Returns the current body 3-axis acceleration.							*
	//!		Param : void															*
	//!		Returns: float[] with the 3-axis acceleration of the pacient.	 				*
	//!		Example: float[] accel = eHealth.getBodyAcceleration();					*
	//!******************************************************************************

	float *eHealthClass::getBodyAcceleration(void)
	{
		/* If int1 goes high, all data registers have new data */
		if (digitalRead(int1Pin)) {// Interrupt pin, should probably attach to interrupt function
			readRegisters(0x01, 6, &data[0]);  // Read the six data registers into data array.

			/* For loop to calculate 12-bit ADC and g value for each axis */
			for (int i=0; i<6; i+=2) {
				accelCount[i/2] = ((data[i] << 8) | data[i+1]) >> 4;  // Turn the MSB and LSB into a 12-bit value

					if (data[i] > 0x7F) {
						accelCount[i/2] = ~accelCount[i/2] + 1;
						accelCount[i/2] *= -1;  // Transform into negative 2's complement #
					}

				accel[i/2] = (float) accelCount[i/2]/((1<<12)/(2*scale));  // get actual g value, this depends on scale being set
			}
		}

		return accel;
	}

	//!******************************************************************************
	//!		Name:	getSystolicPressure()											*
	//!		Description: Returns the  value of the systolic pressure.				*
	//!		Param : void															*
	//!		Returns: int with the systolic pressure.								*
	//!		Example: int systolic = eHealth.getSystolicPressure();					*
	//!******************************************************************************

	int eHealthClass::getSystolicPressure(int i)
	{
		return eHealthClass::bloodPressureDataVector[i].systolic;
	}


	//!******************************************************************************
	//!		Name:	getDiastolicPressure()											*
	//!		Description: Returns the  value of the diastolic pressure.				*
	//!		Param : void															*
	//!		Returns: int with the diastolic pressure.								*
	//!		Example: int diastolic = eHealth.getDiastolicPressure();				*
	//!******************************************************************************

	int eHealthClass::getDiastolicPressure(int i)
	{
		return eHealthClass::bloodPressureDataVector[i].diastolic;
	}


	//!******************************************************************************
	//!		Name:	getAirFlow()													*
	//!		Description: Returns an analogic value to represent the air flow.		*
	//!		Param : void															*
	//!		Returns: int with the airFlow value (0-1023).							*
	//!		Example: int airFlow = eHealth.getAirFlow();							*
	//!******************************************************************************

	int eHealthClass::getAirFlow(void)
	{
		int airFlow = analogRead(1);
		return airFlow; 
	}


	//!******************************************************************************
	//!		Name:	printPosition()													*
	//!		Description: Returns an analogic value to represent the air flow.		*
	//!		Param : uint8_t position : the current body position. 					*
	//!		Returns: void															*
	//!		Example: eHealth.printPosition(position);								*
	//!******************************************************************************

	void eHealthClass::printPosition( uint8_t position )
	{
		if (position == 1) {
			printf("Supine position\n");
		} else if (position == 2) {
			printf("Left lateral decubitus\n");
		} else if (position == 3) {
			printf("Rigth lateral decubitus\n");
		} else if (position == 4) {
			printf("Prone position\n");
		} else if (position == 5) {
			printf("Stand or sit position\n");
		} else  {
			printf("non-defined position\n");
		}
	}

	void eHealthClass::readPulsioximeterLoop(){

		eHealthClass::pulsioxReadingCount ++;
		if (eHealthClass::pulsioxReadingCount == eHealthClass::pulsioxSkipReadings) { //Get only of one pulsioxSkipMeasuresCount measures to reduce the latency
			eHealthClass::readPulsioximeter();
			eHealthClass::pulsioxReadingCount = 0;
		}
	}
	//!******************************************************************************
	//!		Name:	readPulsioximeter()												*
	//!		Description: It reads a value from pulsioximeter sensor.				*
	//!		Param : void										 					*
	//!		Returns: void															*
	//!		Example: readPulsioximeter();											*
	//!******************************************************************************

	void eHealthClass::readPulsioximeter(void)
	   {
	      uint8_t digito[6];

	      uint8_t A = 0;
	      uint8_t B = 0;
	      uint8_t C = 0;
	      uint8_t D = 0;
	      uint8_t E = 0;
	      uint8_t F = 0;
	      uint8_t G = 0;

	      for (int i = 0; i<6 ; i++) { // read all the led's of the module
	         A = !digitalRead(13);
	         B = !digitalRead(12);
	         C = !digitalRead(11);
	         D = !digitalRead(10);
	         E = !digitalRead(9);
	         F = !digitalRead(8);
	         G = !digitalRead(7);

	         digito[i] = segToNumber(A, B, C ,D ,E, F,G);
	         delayMicroseconds(2350); //2800 microseconds

	      }

	         SPO2 = 10 * digito[4] + digito[3];
	         BPM  = 100 * digito[2] + 10 * digito[1] + digito[0];
	   }

	//!******************************************************************************
	//!		Name: airflowWave()														*
	//!		Description: It prints air flow wave form in the serial monitor			*
	//!		Param : int air with the analogic value									*
	//!		Returns: void															*
	//!		Example: eHealth.airflowWave();											*
	//!******************************************************************************

	void eHealthClass::airFlowWave(int air)
	{
		for (int i=0; i < (air / 5) ; i ++) {  
				printf("..");  
		}

		printf("..");
		printf("\n");
		delay(25);
	}


	//!******************************************************************************
	//!		Name: readGlucometer()													*
	//!		Description: It reads the data stored in the glucometer					*
	//!		Param : void															*
	//!		Returns: void															*
	//!		Example: eHealth.readGlucometer();										*
	//!******************************************************************************
	
	void eHealthClass::readGlucometer(void)
	{
		
		// Configuring digital pins like INPUTS
		pinMode(5, OUTPUT);
		digitalWrite(5, HIGH);
		delay(100);
		Serial.begin(1200);
		delay(100);

		Serial.print("U"); // Start communication command. 
		delay(1000); // Wait while receiving data.

		Serial.print("\n");
		if (Serial.available() > 0) {
			eHealthClass::length = Serial.read();// The protocol sends the number of measures
			
			if (Serial.available() > 0) Serial.read(); // Read one dummy data

			for (int i = 0; i<length; i++) { // The protocol sends data in this order
				if (Serial.available() > 0) eHealthClass::glucoseDataVector[i].year = Serial.read();
				if (Serial.available() > 0) eHealthClass::glucoseDataVector[i].month = Serial.read();
				if (Serial.available() > 0) eHealthClass::glucoseDataVector[i].day = Serial.read();
				if (Serial.available() > 0) eHealthClass::glucoseDataVector[i].hour = Serial.read();
				if (Serial.available() > 0) eHealthClass::glucoseDataVector[i].minutes = Serial.read();

				if (Serial.available() > 0)Serial.read(); // Byte of separation must be 0x00.

				if (Serial.available() > 0) eHealthClass::glucoseDataVector[i].glucose = Serial.read();
				if (Serial.available() > 0) eHealthClass::glucoseDataVector[i].meridian = Serial.read();

				if (Serial.available() > 0) Serial.read(); // CheckSum 1
				if (Serial.available() > 0) Serial.read(); // CheckSum 2			
			}
		}
		digitalWrite(5, LOW);
	}

	//!******************************************************************************
	//!		Name: getGlucometerLength()												*
	//!		Description: it returns the number of data stored in the glucometer		*
	//!		Param : void															*
	//!		Returns: uint8_t with length											*
	//!		Example: int length = eHealth.getGlucometerLength();					*
	//!******************************************************************************

	int eHealthClass::getGlucometerLength(void)
	{
		return length;
	}

	//!******************************************************************************
	//!		Name: getBloodPressureLength()											*
	//!		Description: it returns the number of data stored in					*
	//!		the blood pressure sensor												*
	//!		Param : void															*
	//!		Returns: uint8_t with length											*
	//!		Example: int length = eHealth.getBloodPressureLength();					*
	//!******************************************************************************

	int eHealthClass::getBloodPressureLength(void)
	{
		return eHealthClass::length;
	}


	//!******************************************************************************
	//!		Name: numberToMonth()													*
	//!		Description: Convert month variable from numeric to character.			*
	//!		Param : int month in numerical format									*
	//!		Returns: String with the month characters (January, February...).		*
	//!		Example: Serial.print(eHealth.numberToMonth(month));					*
	//!******************************************************************************
	
	const char * eHealthClass::numberToMonth(int month)
	{
		if (month == 1)  return "January"; 
		else if (month == 2)  return "February";
		else if (month == 3)  return "March";
		else if (month == 4)  return "April";
		else if (month == 5)  return "May";
		else if (month == 6)  return "June";
		else if (month == 7)  return "July";
		else if (month == 8)  return "August";
		else if (month == 9)  return "September";
		else if (month == 10) return "October";
		else if (month == 11) return "November";
		else return "December";
	}

	
	//!******************************************************************************
	//!		Name:	version()														*
	//!		Description: It check the version of the library						*
	//!		Param : void															*
	//!		Returns: void															*
	//!		Example: eHealth.version();												*
	//!******************************************************************************

	int eHealthClass::version(void)
	{
		return 2.0;
	}


//***************************************************************
// Private Methods												*
//***************************************************************

	//! This function will read the p/l source register and
	//!	print what direction the sensor is now facing */
	
	void eHealthClass::portraitLandscapeHandler()
	{
		byte pl = readRegister(0x10);  // Reads the PL_STATUS register
		
		switch((pl&0x06)>>1)  // Check on the LAPO[1:0] bits
		{
			case 0:
				position[0] = 0;
			break;
			
			case 1:
				position[0] = 1;
			break;
			
			case 2:
				position[0] = 2;
			break;
			
			case 3:
				position[0] = 3;
			break;
		}
		
		if (pl&0x01)  // Check the BAFRO bit
			position[1] = 0;
		else
			position[1] = 1;
		if (pl&0x40)  // Check the LO bit
			position[2] = 0;
		else 
			position[2] = 1;
 
		bodyPosition();  
	}

/*******************************************************************************************************/

	//! Initialize the MMA8452 registers.

	void eHealthClass::initMMA8452(char fsr, char dataRate)
	{
		MMA8452Standby();  // Must be in standby to change registers
  
		/* Set up the full scale range to 2, 4, or 8g. */
		if ((fsr==2)||(fsr==4)||(fsr==8))
			writeRegister(0x0E, fsr >> 2);  
		else
			writeRegister(0x0E, 0);
			
		/* Setup the 3 data rate bits, from 0 to 7 */
		writeRegister(0x2A, readRegister(0x2A) & ~(0x38));
		
		if (dataRate <= 7)
			writeRegister(0x2A, readRegister(0x2A) | (dataRate << 3));
			
		/* Set up portrait/landscap registers */
		writeRegister(0x11, 0x40);  // Enable P/L
		writeRegister(0x13, 0x14);  // 29deg z-lock, 
		writeRegister(0x14, 0x84);  // 45deg thresh, 14deg hyst
		writeRegister(0x12, 0x05);  // debounce counter at 100ms
		
		/* Set up single and double tap */
		writeRegister(0x21, 0x7F);  // enable single/double taps on all axes
		writeRegister(0x23, 0x20);  // x thresh at 2g
		writeRegister(0x24, 0x20);  // y thresh at 2g
		writeRegister(0x25, 0x8);  // z thresh at .5g
		writeRegister(0x26, 0x30);  // 60ms time limit, the min/max here is very dependent on output data rate
		writeRegister(0x27, 0x28);  // 200ms between taps min
		writeRegister(0x28, 0xFF);  // 1.275s (max value) between taps max
		
		/* Set up interrupt 1 and 2 */
		writeRegister(0x2C, 0x02);  // Active high, push-pull
		writeRegister(0x2D, 0x19);  // DRDY int enabled, P/L enabled
		writeRegister(0x2E, 0x01);  // DRDY on INT1, P/L on INT2
		
		MMA8452Active();  // Set to active to start reading
	}

/*******************************************************************************************************/

	//! Sets the MMA8452 to standby mode. It must be in standby to change most register settings.
	
	void eHealthClass::MMA8452Standby()
	{
		byte c = readRegister(0x2A);
		writeRegister(0x2A, c & ~(0x01));
	}

/*******************************************************************************************************/

	//! Sets the MMA8452 to active mode. Needs to be in this mode to output data
	
	void eHealthClass::MMA8452Active()
	{
		byte c = readRegister(0x2A);
		writeRegister(0x2A, c | 0x01);
	}

/*******************************************************************************************************/	

	//! Read i registers sequentially, starting at address into the dest byte array.
	void eHealthClass::readRegisters(char address, int i, char * dest)
	{
		Wire.beginTransmission(MMA8452_ADDRESS);
		Wire.read_rs(&address,dest,i);
	}

/*******************************************************************************************************/

	//! Read a single byte from address and return it as a byte.
	
	char eHealthClass::readRegister(char address)
	{
		char dest[1];
		Wire.beginTransmission(MMA8452_ADDRESS);
		Wire.read_rs(&address,dest,1);
		return dest[0];
	}

/*******************************************************************************************************/

	//! Writes a single byte (data) into address 
	void eHealthClass::writeRegister(unsigned char address, unsigned char data)
	{
		char transmit[2];
		transmit[0] = address;
		transmit[1] = data;
		Wire.write(transmit,2);
	}

/*******************************************************************************************************/

	//! Assigns a value depending on body position.

	void eHealthClass::bodyPosition( void )
	{  
		if (( position[0] == 0 ) && (position[1] == 1) && (position [2] == 0)) {
			bodyPos = 1;
		} else if (( position[0] == 1 ) && (position[1] == 1) && (position [2] == 0)) {
			bodyPos = 1;
		} else if (( position[0] == 3 ) && (position[1] == 1) && (position [2] == 0)) {
			bodyPos = 1;
		} else if (( position[0] == 2 ) && (position[1] == 0) && (position [2] == 0)) {
			bodyPos = 1;
		} else if (( position[0] == 2 ) && (position[1] == 1) && (position [2] == 1)) {
			bodyPos = 1;
		} else if (( position[0] == 2 ) && (position[1] == 1) && (position [2] == 0)) {
			bodyPos = 1; 
			
		} else if (( position[0] == 0 ) && (position[1] == 1) && (position [2] == 1)) {
			bodyPos = 2;
		} else if (( position[0] == 0 ) && (position[1] == 0) && (position [2] == 1)) {
			bodyPos = 2;
			
		} else if (( position[0] == 1 ) && (position[1] == 1) && (position [2] == 1)) {
			bodyPos = 3;
		} else if (( position[0] == 1 ) && (position[1] == 0) && (position [2] == 1)) {
			bodyPos = 3;
		  
		} else if (( position[0] == 1 ) && (position[1] == 0) && (position [2] == 0)) {
			bodyPos = 4;
		} else if (( position[0] == 3 ) && (position[1] == 0) && (position [2] == 0)) {
			bodyPos = 4;
			
		} else if (( position[0] == 3 ) && (position[1] == 0) && (position [2] == 1)) {
			bodyPos = 5;
		} else if (( position[0] == 3 ) && (position[1] == 1) && (position [2] == 1)) {
			bodyPos = 5;
		} else if (( position[0] == 2 ) && (position[1] == 0) && (position [2] == 1)) {
			bodyPos = 5;
		} else  { 
			bodyPos = 6;
		}
	}

/*******************************************************************************************************/

	//! Converts from 7 segments to number.

	uint8_t eHealthClass::segToNumber(uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint8_t E, uint8_t F, uint8_t G )
	{
		if ((A == 1) && (B == 1) && (C == 1) && (D == 0) && (E == 1) && (F == 1) && (G == 1)) {
			return 0;
	   
		} else if ((A == 0) && (B == 1) && (C == 0) && (D == 0) && (E == 1) && (F == 0) && (G == 0)) {  
			return 1;
		
		} else if ((A == 1) && (B == 1) && (C == 0) && (D == 1) && (E == 0) && (F == 1) && (G == 1)) { 
			return 2;
		
		} else if ((A == 1) && (B == 1) && (C == 0) && (D == 1) && (E == 1) && (F == 0) && (G == 1)) { 
			return 3;
		
		} else if ((A == 0) && (B == 1) && (C == 1) && (D == 1) && (E == 1) && (F == 0) && (G == 0)) { 
			return 4;
		
		} else if ((A == 1) && (B == 0) && (C == 1) && (D == 1) && (E == 1) && (F == 0) && (G == 1)) { 
			return 5;
		
		} else if ((A == 1) && (B == 0) && (C == 1) && (D == 1) && (E == 1) && (F == 1) && (G == 1)) { 
			return 6;
		
		} else if ((A == 1) && (B == 1) && (C == 0) && (D == 0) && (E == 1) && (F == 0) && (G == 0)) {
			return 7;  
		
		} else if ((A == 1) && (B == 1) && (C == 1) && (D == 1) && (E == 1) && (F == 1) && (G == 1)) { 
			return 8;
		
		} else if ((A == 1) && (B == 1) && (C == 1) && (D == 1) && (E == 1) && (F == 0) && (G == 1)) { 
			return 9;
			
		} else  {
			return 0;
		}
	}

/*******************************************************************************************************/

	//! Swap data for blood pressure mesure
	
	char eHealthClass::swap(char _data)
	{
		char highBits = (_data & 0xF0) / 16; 
 		char lowBits =  (_data & 0x0F) * 16; 
  		return ~(highBits + lowBits);
	}

/*******************************************************************************************************/
//***************************************************************
// Preinstantiate Objects										*
//***************************************************************
// eHealthClass eHealth = eHealthClass();

void readPulsioximeterForwarder(void* context) {
	static_cast<eHealthClass*>(context)->readPulsioximeterLoop();
}


