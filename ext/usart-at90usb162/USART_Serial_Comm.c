/*
 * USART_Serial_Comm.c
 *
 * Created: 03/01/2012 16:29:59
 *  Author: prego
 */ 

#include <avr/io.h>
#include <avr/delay.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void USART_Init (){
	// Turn on the Tx and Rx circuitry
    UCSR1B |= (1 << RXEN1) | (1 << TXEN1);

	// Use 8-bit character sizes
	UCSR1C |= (1 << UCSZ10) | (1 << UCSZ11);

	// Set the baud rate
	UBRR1H = (BAUD_PRESCALE >> 8);
	UBRR1L = BAUD_PRESCALE;
}

void USART_Transmit (unsigned char data){
	// Wait for empty transmit buffer from USART
	while ( !(UCSR1A & (1 << UDRE1)));

	// Transmit the data
	UDR1 = data;
}

unsigned char USART_Receive (){
	// Wait for data to be received
	while ( !(UCSR1A & (1 << RXC1)));

	// Get and return received data from buffer
	return UDR1;
}

int main(void)
{
	USART_Init();
	char DataTransfer;
	while (1){
		DataTransfer = USART_Receive();
		USART_Transmit(DataTransfer);
	}
}
