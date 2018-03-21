/*  Climate Control System - Leaf Node - AVR Implementation.
    Author   Warren Holley
    Version  0.1.0
    Date     March 14,2017
    Purpose
      This component is a leaf module for power switching to external devices.
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
#include <util/delay.h>
#include <stdlib.h>

//Boolean implementation.
typedef int bool; 
#define true 1
#define false 0

//Config:
#define UserDebugMode 	true 	//Pushes human-readable text to the through UART.
#define MyID		{1,3} 	//Default: Humidifier, fan.
#define MyIDLength	2
#define RelayPort	PORTB	//Relay I/O Port
#define RelayPin	1	//First Pin for Relay I/O (Active high). Counts up for each ID value.

#define MaxOnLength 	60 	//Failsafe Hub override.

//ID values of nodes. Must match set given to Hub Node.
#define HubNode  	0
#define HumidifierNode  1
#define HeatNode 	2
#define FanNode  	3

//Included Modules:
#include "RFTransceiver.h"

//Global Variables:
uint8_t onTimeLength[MyIDLength]; 
//in Half-Seconds. 8-bit as to set absolute maximum to 128 seconds ontime

//------------------------------------------------------
static inline void initTimer1(void) {
	//Configures the interrupt to activate just about twice per second.
	TCCR1B |= (0<<CS12)|(1<<CS11)|(0<<CS10);
	TIMSK1 |= (1 << OCIE1A);
}

// Interrupt function.
// Twice per second, check for timeout, turn off.
//  Otherwise, decrement time counter.
ISR(TIMER1_COMPA_vect) { 
	PORTD ^= (1<<4); // Toggle Debug LED;
	
	for (int i = 0; i < MyIDLength; i++) {
		//Decrement onTimeLength, or turn relay off if 0.
		if (onTimeLength[i] > 0)
			onTimeLength[i]--;
		else if ( 1 & ( RelayPort >> (RelayPin+i))) { //If device is on
			RelayPort &= ~(1 << (RelayPin+i)); //Turn off the relay.
		
			if (UserDebugMode){ //Alarm User to info.
				debugPrintString("Timeout Turning off device ",40);
				printUInt(i);
				printNewLine();
			}
		}
	}
}

//------------------------------------------------------
//Returns the pin offset of the pin mapped to the pin of the packet.
uint8_t pinMap(uint8_t packetID){
	uint8_t myID[] = MyID; //Set of IDs that the device will respond to.
	
	for (int i = 0; i < MyIDLength; i++)
		if (packetID == myID[i])
			return i;
	
	return 10; //Should never hit this. Sentinel value.
}

//-----------------------------------------------------

int main(){
	uint8_t myID[] = MyID; //Set of IDs that the device will respond to.
	
	//Setup. Initializes each pin, (Default: PB1,PB2...)
	for (int i = 0; i < MyIDLength; i++) {	
		DDRB |= (1 << (RelayPin+i)); 		//Sets the pins to output
		RelayPort &= ~(1<<(RelayPin+i)); 	//Default off.
		onTimeLength[i] = 0; 			//Initialize on-Time counter.
	}

	//Optional Debug LED. Toggles for each interrupt
	DDRD |= (1<<4);
	PORTD |= (1<<4);
	
	//Initialize Interupt timer
	initTimer1();
	
	//Initialize Serial Comms as receiver receiver
	//with output debugging if requested by user.
	InitReceiver(1000,UserDebugMode,myID,MyIDLength); 
	
	sei(); //Activate interrupts.
	uint8_t* receivedPacket;

	while(1) {
		//Fetch personal packet
		receivedPacket = ReceivePersonalPacket();
		uint8_t packetID = receivedPacket[0];				
		uint8_t onTime = receivedPacket[1];
		uint8_t pinOffset = pinMap(packetID);
		uint8_t pinByte = (1 << (RelayPin + pinOffset));
		free(receivedPacket);

		if (pinOffset == 10)
			continue; //Something went wrong with getting the offset. Reset loop.
		
		if (onTime > MaxOnLength) //Fail-safe checking.
			onTime = MaxOnLength; 
		
		//Double as to interrupt value maps to ~half-seconds. Store
		onTimeLength[pinOffset] = onTime*2; 

		if (onTime != 0) //Receives a 0 for turn-off. Otherwise, turn on.
			RelayPort |= pinByte; // Turn on the relay;
		else
			RelayPort &= ~pinByte; //Turn off.

		if (UserDebugMode){ //Alarm User to info.
			debugPrintString("Turning Device ",20);
			printUInt(pinOffset);
			debugPrintString(" on for ",20);
			printUInt(onTime);
			debugPrintString(" seconds\r\n",20);
		}
	}
	return 0;
}
