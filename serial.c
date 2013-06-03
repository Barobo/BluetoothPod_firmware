#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "serial.h"
#include "timer.h"
#include "twi.h"
volatile int g_serialBufferN = 0;
volatile int g_serialBufferIndex = 0;
volatile uint8_t g_serialBuffer[SERIAL_BUFFER_SIZE];

volatile int g_serialBufferOutN = 0;
volatile int g_serialBufferOutIndex = 0;
volatile uint8_t g_serialBufferOut[SERIAL_BUFFER_SIZE];

volatile uint8_t* g_commandBuf = NULL;
volatile int g_commandSize = 0;
volatile int g_commandSource = 0;

#define QUERIEDADDRESSES_SIZE 64
uint16_t* g_queriedAddresses;
char** g_queriedIDs;
volatile uint8_t g_numQueriedAddresses;

void USARTInit(void)
{
	// Initialize the serial port 
	// configure baud rate register UBRR1 with double speed mode (U2X1=1)
	// according to formula UBRR1 = (fosc/(8*baud)) - 1
	// use UBRR1 = (fosc/(16*baud)) - 1 for normal speed mode (U2X1=0)
	// For 115.2kbps UBRR1=16.36. If we use UBRR1=16, we will have a baud rate
	// of 117647, about 2% faster than 115200, but sufficiently close
	UBRR0 = 34;
	//UBRR1 = 16;
	
	UCSR0A = 1<<U2X0; // disable double speed mode
	//UCSR1A = (1<<U2X1); // enable double speed mode
	
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); // enable RX, TX, and RX interrupt on USART1

} // USARTInit()

/* Interrupt for receiving a byte */
ISR(USART_RX_vect)
{
  uint8_t sreg = SREG;
  uint8_t data;
  //static char buf[16];
  data = UDR0;
  g_serialBuffer[(g_serialBufferIndex + g_serialBufferN)%SERIAL_BUFFER_SIZE] = data;
  g_serialBufferN++;
  //serialWriteByte(data);
  SREG = sreg;
  return;
}

unsigned char serialReadByteDIRECT(void)
{
	// get byte received in USART1
	while( !(UCSR0A & (1<<RXC0)) ); // wait for received byte
	return UDR0; // return received byte
} // serialReadByte()

int serialReadByte(void)
{
  uint8_t data;
  if(g_serialBufferN == 0) {
    return -1;
  }
  data = g_serialBuffer[g_serialBufferIndex];
  g_serialBufferIndex = (g_serialBufferIndex+1)%SERIAL_BUFFER_SIZE;
  g_serialBufferN--;
  return data;
}

void serialWriteByte(unsigned char txdata)
{
	// send data byte via USART1
	while( !(UCSR0A & (1<<UDRE0)) );
	UDR0 = txdata; // copy data byte to data register
} // serialWriteByte()

void serialWriteBytes(uint8_t* data, int datasize)
{
  int i;
  for(i = 0; i < datasize; i++) {
    serialWriteByte(data[i]);
  }
}

void serialWriteString(const char* str)
{
  int i;
  for(i = 0; str[i] != '\0'; i++) {
    serialWriteByte(str[i]);
  }
  serialWriteByte(0);
}

int serialAvailable()
{
  return g_serialBufferN;
}

volatile int serialBufferIndex = 0;
static void serialTimeoutHandler(void)
{
  serialBufferIndex = 0;
  g_serialBufferN = 0;
  g_serialBufferIndex = 0;
  //flash_led(1);
  //setRGB(g_rgb[0], ~g_rgb[1], g_rgb[2]);
}

void serialHandler()
{
  int byte;
  static uint8_t buf[256]; /* global 256 bytes */
  static timer_t timer;
  timer.interval_millis = 200;
  timer.mode = TIMER_INTERVAL;
  timer.callback = serialTimeoutHandler;
  /* Process incoming data */
  while(serialAvailable() > 0) {
    byte = serialReadByte();
    // DEBUG
    //serialWriteString("Received:");
    //serialWriteByte(byte);
    //serialWriteByte(serialBufferIndex);
    if(byte < 0) {
      serialBufferIndex = 0;
      return;
    }
    /* Start a timeout */
    if(serialBufferIndex == 0) {
      //flash_led(1);
      timerStart(&timer);
    }
    buf[serialBufferIndex] = (uint8_t)byte;
    serialBufferIndex++; /* "serialBufferIndex" is now the number of bytes that have been received */
    if(serialBufferIndex > 1) {
      if(buf[1] == serialBufferIndex) {
        /* Found end of stream. Now parse */
        /* Put the data on the I2C bus */
        //serialWriteString("twi send\n");
        //delay(500);
        //serialWriteByte(0x80);
        //TWISend(0x01, buf, buf[1]);
        //TWIWait();
        memcpy(g_twi_send_prebuf, buf, buf[1]);
        g_twi_send_prebuf_n = serialBufferIndex;
        serialBufferIndex = 0;
        return;
      }
    }
  }
  /* Process outgoing data */
  while(g_serialBufferOutN > 0) {
    serialWriteByte(g_serialBufferOut[g_serialBufferOutIndex % SERIAL_BUFFER_SIZE]);
    g_serialBufferOutIndex = (g_serialBufferOutIndex+1) % SERIAL_BUFFER_SIZE;
    g_serialBufferOutN--;
  }
}

