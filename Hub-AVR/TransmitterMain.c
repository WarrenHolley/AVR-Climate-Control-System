/*  Climate Control System - Hub Node - AVR Implementation.
    Author   Warren Holley
    Version  0.1.0
    Date     March 14,2017
    Purpose
      This component is a central hub for sensing and communication.
      A simple climate control system for use in a small apartment or room, controlled by an
      ATMega 168. Uses the 'ATMega168 UART ASK Wireless Library' I also published.
      Consists of a few small subsystems: Heating, Humidifying and a Fan for circulation.
      Designed for easy modification or expansion.
      Currently only runs in Simplex communication.
    Notes
      Works with standard cheap 'RF' modules. (ASK modules, ~$2-$5/e)
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h> //For the ADC oversampling
#include <util/delay.h>

//Boolean implementation.
typedef int bool; 
#define true 1
#define false 0

//Config:
#define UserDebugMode 	true //Pushes human-readable text to the through UART.
#define VRef 		5.00 //ADC Voltage Reference.
#define TemperaturePin	PC5  //Pin 28
#define HumidityPin 	PC4  //Pin 27

//Atmosphere and Tolerances (Default selected for book preservation/archival)
#define TargetTemp     21.0 //*C
#define TargetHumidity 40.0 //%

//Tolerances
#define ToleranceTemp      3.0 //*C
#define ToleranceHumidity  10.0 //%

//Timing Values. TODO: Upgrade to PID system.
#define TimeBetween 5 //Time, in seconds, between testing conditions at the hub.

//Time, in seconds for each module to be activated for.
// System uses non-blocking power switching. These values are fail-safes.
// If comms or Hub fails, don't want powered items on permentantly.
#define TimeOnHeater     20
#define TimeOnHumidifier 15
#define TimeOnFan        30

//ID values of nodes. Must match IDs assigned to leaf nodes.
#define HubNode  	0
#define HumidifierNode  1
#define HeatNode 	2
#define FanNode  	3

//Included Modules:
#include "RFTransceiver.h"

//Main Interfaces:
void initADC(uint8_t); 		//Initializes ADC on port given.
double getAnalog(uint8_t); 	//Reinitialize ADC, returns real voltage read.

void powerNode(uint8_t, uint8_t); 	//Sends packet to arg1 to turn on system for arg2 time.

double getTemp(); 		 //Returns, in *C, the temperature.
double getHumidity(float); //Returns, in %, the True Humidity of the environment.
				 // Requests the temperature as to minimize reads to the ADC.

//-----------------ADC SECTION----------------------
//Initializer. Unnedded for user calling.
void initADC(uint8_t ADCPort) {
	ADCSRA &= ~(1 << ADEN); //Disable ADC
	ADMUX &= 0; 		//Clear settings, in order to switch sources
	for (int i = 0; i < 128; i++) //As ADMUX stored in a buffer, need to wait for 
		; // Clock-Divider cycles to allow reset. (Doubled here for certainty)

	//Activate, configure
	ADMUX |= (0x0f & ADCPort); //Select port.
	ADMUX |= (0 << REFS1) | (1 << REFS0); //Refer to AVcc w/ external cap to AREF
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (0 <<ADPS0); //64x-Clock Division
	ADCSRA |= (1 << ADEN); //Enable ADC.
	
	//Enable ADC Sleep mode.
	set_sleep_mode(SLEEP_MODE_ADC);
	ADCSRA |= (1 << ADIE); //Enable interrupt for ADC. Used for oversampling.
}

// To make sure the ADC doesn't interrupt anything else.
EMPTY_INTERRUPT(ADC_vect);

// Oversample the ADC. Returns the average reading.
// Reduces risk of noise in reads.
uint16_t getADC_overSample16x() {
	uint16_t overSample = 0;
	for (int i = 0; i < 16; i++) { //Sum 16 reads.
		sleep_mode();
		overSample += ADC;
	}
	overSample = overSample >> 4; 	//Divide by 16 to get average of the oversample.
	overSample = overSample >> 2; 	//Map 10-bit value to 8-bit.
	return overSample; //Returning 2B value as it's immediately casted to double anyways.
}

//Returns double value of input port voltage.
double getAnalog(uint8_t ADCPort)
{
	//As system grabs all three sensors each cycle, needs reinitialization for each.
	initADC(ADCPort);
	double returnVal = (double) getADC_overSample16x() * VRef / (double)256;
	return returnVal;
}


//------------------Sensor Fetch & Process-----------------------

// Returns float of temperature in *C
//  Designed for TMP36-GZ. Accepts 2.7-5.5V.
//  Voltage output linear to the atmospheric temperature.
double getTemp() {
	double temp=100*getAnalog(TemperaturePin) -50;
	return temp;//returnVal;
}

// Returns float of True Humidity in %.
//  Designed for HIH-4030. Accepts 4-5.8V.
//  Argument-given temp as to minimize ADC reads.
double getHumidity(float temperature){
	/*Calculations:
	By Datasheet: 	Vo  = VRef*(0.0062*Rh+0.16)	    Rh: Relative Humidity in %
		Thus:	Rh  = Vo/(0.0062*VRef) - 25.81
	  	Return	Th = Rh / (1.0546 - 0.00216*Temp)   Th: True Humidity in % */
	float Vo = getAnalog(HumidityPin);
	float Rh = -25.81 + Vo / (0.0062*VRef);
	float Th = Rh / (1.0546 - 0.00216 * temperature);
	return Th;
}

//----------------------Transmitter----------------------


void powerNode(uint8_t ID, uint8_t onTime){
	SecTransmitPacket(ID,onTime);
	_delay_ms(50);	//Allow time between transmissions of receiver-formatted packets
			//to minimize packet loss
}

//----------------------MAIN-----------------------
int main(){
	initADC(PC5); 			//Initialize ADC (Not neccessarily needed.)
	InitTransmitter(1000); 		//Initialize Serial Comms
	sei(); 				//Activate Interupts.

	double temperature, humidity;

	bool TurnHeaterOn, TurnHeaterOff;
	bool TurnHumidifierOn, TurnHumidifierOff;

	while(1) {
		//Reset variables.
		TurnHeaterOn 	= false;
		TurnHeaterOff 	= false;
		TurnHumidifierOn  = false;
		TurnHumidifierOff = false;
		
		temperature = getTemp();
		_delay_ms(25); //Required delay to reset ADC.
		humidity = getHumidity(temperature);

		//Deactivate if exceeds target+tolerance
		//Active if below target-tolerance
		if (temperature > TargetTemp + ToleranceTemp)
			TurnHeaterOff = true;
		else if (temperature < TargetTemp - ToleranceTemp)
			TurnHeaterOn = true;

		if (humidity > TargetHumidity + ToleranceHumidity)
			TurnHumidifierOff = true;
		else if (humidity < TargetHumidity - ToleranceHumidity)
			TurnHumidifierOn = true;

		if (UserDebugMode) { //Debug output regarded as noise by receivers.
			debugPrintString("Temp: ",10);
			printDouble(temperature);
			debugPrintString("*C\r\n",5);
			debugPrintString("Humidity: ",11);
			printDouble(humidity);
			debugPrintString("%\r\n\n",4);
		}

		if (UserDebugMode)  { //Let user know what is happening this cycle
			if (TurnHeaterOn)
				debugPrintString("Activating Heater\r\n",30);
			if (TurnHeaterOff)
				debugPrintString("Dectivating Heater\r\n",30);
			if (TurnHumidifierOn)
				debugPrintString("Activating Humidifier\r\n",30);
			if (TurnHumidifierOff)
				debugPrintString("Deactivating Humidifier\r\n",30);
			if (TurnHeaterOn || TurnHumidifierOn)
				debugPrintString("Activating Fan\r\n",20);
			if (TurnHumidifierOn || TurnHeaterOn || TurnHeaterOff || TurnHumidifierOff)
				printNewLine(); //Leave extra line for readability.
		}
		

		//Power Signals:
		
		if (TurnHeaterOn)
			powerNode(HeatNode, TimeOnHeater);
		else if (TurnHeaterOff)
			powerNode(HeatNode, 0);

		if (TurnHumidifierOn)
			powerNode(HumidifierNode, TimeOnHumidifier);
		else if (TurnHumidifierOff)
			powerNode(HumidifierNode, 0);

		if (TurnHumidifierOn || TurnHeaterOn)
			powerNode(FanNode,TimeOnFan);

		if (TurnHumidifierOn || TurnHeaterOn || TurnHeaterOff || TurnHumidifierOff)
			printNewLine(); //Such that the serial monitor only prints a new line after all
					// the transmissions.
		
		// Don't turn the fan off, as the fan should run a bit longer than 
		//  the other systems to allow for circulation.
		// As the hub is currently time-memoryless, does not know if non-fan 
		// systems are currently on or off at any given time.
		// Auto-shutdown (TimeOnFan constant) is left to handle it.
				
		_delay_ms(TimeBetween*1000); 
		// Not exactly X seconds, as there are transmission
		// and waiting delays, but not currently worth implementing
		// a timer or timer-interrupt for a few percent accuracy.
	}
	
	return 0;
}
