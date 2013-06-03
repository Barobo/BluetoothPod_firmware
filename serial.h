#ifndef _USART_H_
#define _USART_H_
#include <stdint.h>

#define SERIAL_BUFFER_SIZE 256 

/* Serial Communications Protocol:
   Computer initiates command:
   [0] 1 byte command header
   [1] 1 byte total message size
   [2] 1 byte zigbee destination address high byte
   [3] 1 byte zigbee destination address low byte
   [4] 1 byte zigbee destination endpoint
   [5] x bytes Mobot protocol message

   Mobot checks destination address. If it is itself, sends data back to computer:
   1 byte header
   1 byte total message size
   2 bytes zigbee source address
   1 byte zigbee source endpoint
   x bytes Mobot protocol message
  
   If not itself, sends zigbee message to destination, waits for response, then
   responds to computer using above protocol. 
   */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum cmdSource_e
{
  CMD_SOURCE_NULL,
  CMD_SOURCE_SERIAL,
  CMD_SOURCE_ZIGBEE
}cmdSource_t;

extern volatile int g_serialBufferN;
extern volatile int g_serialBufferIndex;

extern volatile int g_serialBufferOutN;
extern volatile int g_serialBufferOutIndex;
extern volatile uint8_t g_serialBufferOut[SERIAL_BUFFER_SIZE];

void USARTInit(void);
unsigned char serialReadByteDIRECT(void);
int serialReadByte(void);
void serialWriteByte(unsigned char txdata);
void serialWriteBytes(uint8_t* data, int datasize);
//#define debugWriteString(x) serialWriteString(x)
#define debugWriteString(x) 
int debugPrintf(const char *format, ...);
void serialWriteString(const char* str);
int serialAvailable();
void serialHandler();
uint8_t* processSerialCommand(const uint8_t* data, int size, cmdSource_t cmdSource);
uint8_t* formatSerialMessage(const uint8_t *data, int datasize, uint16_t srcAddress, uint8_t srcEndpoint);
uint8_t* formatMessage(const uint8_t *data, int datasize);

#ifdef __cplusplus
}
#endif

#endif
