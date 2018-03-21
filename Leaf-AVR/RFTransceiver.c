/*  ATMega168 UART for ASK Wireless Modules.
    Author   Warren Holley
    Version  0.1.5
    Date     March 14,2017
    Purpose
      For transmission and reception of UART over noisy wireless systems.
      Enpackets data with timing, checksumming, and target-device bytes to minimize packet loss.
      Intended for use with standard cheap 'RF' modules. (ASK modules, ~$2-$5/e)
*/
#include <stdlib.h> //For Memory Allocation
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "RFTransceiver.h"

typedef int bool; //Boolean implementation.
#define true 1
#define false 0

//Setup Config:
#define WIRELESS_BAUD 1000 	//In bits/s

uint8_t* ChipIDSet;
uint8_t  ChipIDSize;

// Initialize USART in UART format.
// Implemented and tested for ATMega168a.
// TODO: Test with remaining ATMega*8a line.

void InitTransmitter (uint16_t dataBaudRate) {
	//Calclate USART Baud Rate Register value.
	uint16_t UBRR_rate = F_CPU/(16*dataBaudRate) - 1;
	//Set Baud Rate
	UBRR0H = (unsigned char) (UBRR_rate >> 8); 
	UBRR0L = (unsigned char) UBRR_rate;
	//Enable Transmitter and/or Receiver
	UCSR0B = (1<<TXEN0);
	// Frame Format: 8 bits of data, 1 stop bit
	UCSR0C = (1<<UCSZ00) | (1 << UCSZ01);

	//TODO. Upgrade for setable frame format range.
	//Defaulting to 8/1 as that is what's needed for the wireless component of the major
	// Climate Control system that this firmware was built for.
}

void InitReceiver (uint16_t dataBaudRate, bool transmitBool, uint8_t* receiverID, uint8_t IDSize) {

	//ID Parsing.
	if (ChipIDSet) //If reinitializing, free previously malloc'd data
		free(ChipIDSet);
	ChipIDSet = malloc(IDSize);
	ChipIDSize = IDSize;
	//Deep copy, in case of parent thread freeing array.
	for (int i = 0; i < IDSize; i++)
		ChipIDSet[i] = receiverID[i];
	
	//Calclate USART Baud Rate Register value.
	uint16_t UBRR_rate = F_CPU/(16*dataBaudRate) - 1;
	//Set Baud Rate
	UBRR0H = (unsigned char) (UBRR_rate >> 8); 
	UBRR0L = (unsigned char) UBRR_rate;
	//Enable Receiver (and transmitter, if flagged)
	UCSR0B = (transmitBool<<TXEN0)|(1<<RXEN0);
	// Frame Format: 8 bits of data, 1 stop bit
	UCSR0C = (1<<UCSZ00) | (1 << UCSZ01);

	//TODO. Upgrade for setable frame format range.
	//Defaulting to 8/1 as that is what's needed for the wireless component of the major
	// Climate Control system that this firmware was built for.
}

//Transmits the raw byte given as an argument.
//Is blocking until the output buffer is clear.
void TransmitByte (uint8_t dataByte) {
	while (!( UCSR0A & (1<<UDRE0)))
		; //Delay until data buffer is empty
	//Push data to transmit buffer
	UDR0 = dataByte;
}

// Enpacket, transmit 3 4-byte packets.
// Packet Format: [Timing][ID(5),Packet(3)][Data][Checksum]
// [Checksum] = [ID,Packet] XOR [Data]
void SecTransmitPacket(uint8_t ID, uint8_t inputByte) {
	
	uint8_t ID_PacketByte = (ID<<3) | 0; //0 for packet# init.

	for (int i = 0; i < 3; i++){ //Transmit 3 packets
		TransmitByte(0b10101010);
		TransmitByte(ID_PacketByte+i); //=0,1,2
		TransmitByte(inputByte);
		TransmitByte(inputByte^(ID_PacketByte+i));
	}
}

// Returns the UART Received byte when available.
// Blocking until something is received.
uint8_t ReceiveByte() {
	while (! (UCSR0A & (1 << RXC0)))
		; //Wait for Receive Complete Flag to clear
	return UDR0;
}

// Receives, verifies, and truncates a received byte.
// Returns a 3-byte uint8 block: {ID,Packet#,Data}
// MALLOC! Be sure to free pointer after use.
uint8_t* SecReceiveDataPacket(){
	//Receipt Format: [Timing][ID(5),Packet(3)][Data][Checksum]
	uint8_t buffer[4];
	for (int i = 0; i < 4; i++)
		buffer[i] = ReceiveByte(); //Init: Fill buffer.
	
	//If timing buffer incorrect, or checksum value incorrect, shift, fill buffer.
	while (buffer[0] != 0b10101010  || (buffer[1]^buffer[2]) != buffer[3]){ 
	 	for (int i = 0; i < 3; i++)  		// Shift buffer. Could improve perf by using
			buffer[i] = buffer[i+1];  	//  modulo-circle array.
		buffer[3] = ReceiveByte();
	}
	//At this point timing and checksum values accurate.
	uint8_t* returnBlock = malloc(3);
	returnBlock[0] = buffer[1] >> 3;  //ID Value
	returnBlock[1] = buffer[1] & 0x7; //Packet#
	returnBlock[2] = buffer[2];	  //Data

	return returnBlock;
}


// Monitors receiver for datagrams intended for the device's ID.
// Attempts to receive the 3-block datagram, parse their values, and return the majority value.
// Uses basic Repetition Code for voting.
// Returns a two-byte array: [ID Target][Data Block]
uint8_t* ReceivePersonalPacket(){
	
	bool didVote[3]; 	//Two arrays, boolean if there was a viable packet, and if
	uint8_t dataVotes[3];	// what the packets value was.
	for (int i = 0; i < 3; i++){
		didVote[i] = false; //Clear arrays.
		dataVotes[i] = 0;
	}

	//Init: Wait for data packet for this individual device.
	uint8_t tempReceiverID = 0; //
	uint8_t* recData = SecReceiveDataPacket();
	while(tempReceiverID == 0){
		for (int i = 0; i < ChipIDSize; i++)
			if (recData[0] == ChipIDSet[i])
				tempReceiverID = ChipIDSet[i]; // Then look for this packet ID.
		
		if (tempReceiverID == 0) { //If it's still 0, wait for next packet.
			free(recData);
			recData = SecReceiveDataPacket();
		}
	}
	
	//Vote Collation: if packet received, vote on it's value.
	// recData[1] is the packet#. As no packet repetition, are guarenteed to be in order.
	// Lost or dropped packets are ignored.
	if (recData[1] == 0){
		dataVotes[0] = recData[2];
		didVote[0] = true;
		//Fetch next block
		free(recData);
		recData = SecReceiveDataPacket();
	}
	if (recData[1] == 1 && recData[0] == tempReceiverID){
		dataVotes[1] = recData[2];
		didVote[1] = true;
		free(recData);
		recData = SecReceiveDataPacket();
	}
	if (recData[1] == 2 && recData[0] == tempReceiverID){
		dataVotes[2] = recData[2];
		didVote[2] = true;
	}// End Vote Collation.
	free(recData);
	
	//As transmissions are 3*4 bytes transmitted immediately, if ID changes, then 
	//following blocks were lost. Edge case where two transmissions A,B, both received.
	// If A2,A3,B1 dropped, would consider A1,B2 single data block.

	//Possible fix: more than 3-count packet counting, with offset check. 
	// Eg, Packet 3 not in same package as Packet 4. Disregard 3.
	
	
	//If three packets, return more common value.
	//If only one or two packets, assume the first is correct. 
	// (TODO: Maybe upgrade to 5+-block datagram, -require- at least 2 votes)
	// If no common value in 3, return first vote.

	//Edge Case: Corruption to values that make no sense.
	// Eg: 0x05 -> 0x85. Fix: Bound, or picking minimum of available values,
	//  as don't want to run mains power continuously.
	// Could also implement some FECC, but would require a hell of a lot long datagrams.
	// Maybe Manchester Encoding? Would fix a few issues, but only double size.
	//  Need to test if the decoding and FECC algs would fit in such a tiny device.


	//Return Format: [ID Target][Data]
	uint8_t* returnArray = malloc(2);
	returnArray[0] = tempReceiverID;
		
	if (didVote[0]){
		if (didVote[1] && didVote[2]){
			if (dataVotes[0] == dataVotes[1])
				returnArray[1] =  dataVotes[0];	   //If ABC, A=B,Ret A;
			if (dataVotes[1] == dataVotes[2])
				returnArray[1] =  dataVotes[1];	   //If ABC, B=C,Ret C;
		}
		returnArray[1] = dataVotes[0];//AB||AC,ret A.
	}
	else if (didVote[1])//B||BC, ret B.
		returnArray[1] = dataVotes[1];
	else // Only C, ret C.
		returnArray[1] = dataVotes[2];	
	
	return returnArray;
	
}


// Parses Double value into integer and fractional value.
// Prints these values to the screen. Rounds down to hundreths.
void printDouble(double number) {
	uint8_t intPart, fracPart;
	double doubleIntPart;

	if (number < 0){ //If negative, note as such, invert.
		TransmitByte('-');
		number *= -1;
	}
	//Translate to uint values.
	fracPart = (uint8_t) (modf(number, &doubleIntPart)*100);
	intPart = (uint8_t) doubleIntPart;

	printUInt(intPart);
	TransmitByte('.');	
	printUInt(fracPart);
}


// Steps through Decimal notation of the uint8 value, printing individual num-characters.
void printUInt (uint8_t value){
	uint16_t divisor=10;
	uint8_t outputValue;
	while (value/divisor > 0)
		divisor*=10;
	for (divisor/=10; divisor >= 1; divisor/=10) {
		outputValue='0'+(value/divisor)%10;
		TransmitByte(outputValue);
	}
}

// For sanity's sake.
// Suggest using sprintf for more advanced strings, but this is simple enough for most.
// 'length' argument is only for substrings or for arrays with no null terminator.
// Does not impact performance to overstate the actual string length.
void debugPrintString(char* string, int length){
	for (int i = 0; i < length && string[i] != '\0'; i++)
		TransmitByte(string[i]);
}

//To reduce headaches.
void printNewLine () {
	
	TransmitByte('\r'); //As some terminals (And windows) don't cooperate with only \n.
	TransmitByte('\n');
}

//Debug example transmitter.
//Counts & Transmits 0->99. Repeats
void debugTransmit(uint8_t ChipID){
	InitTransmitter(1000);
	for (uint8_t i = 0; i < 100; i = (i+1)%100){
		SecTransmitPacket(ChipID, i);
		printNewLine();
		_delay_ms(1000); //Increment & print once per second (or about, with IO delays)
	}
}	
// Debug receiving script.
// Transmits character string values of received uint 
void debugReceive(){
	uint8_t ID[3] = {1,2,3}; //Recieve all packets
	InitReceiver (1000, true, ID, 3);

	uint8_t* packet;
	while(1) {
		packet=ReceivePersonalPacket();
		debugPrintString("Received: Packet for ",50);
		printUInt(packet[0]);
		debugPrintString(" with value: ",50);
		printUInt(packet[1]);
		printNewLine();
		free(packet);
	}
}




