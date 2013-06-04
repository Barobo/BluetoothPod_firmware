#include "twi.h"
#include <stdio.h>
#include "serial.h"

volatile enum twi_state_e g_twi_state;
volatile enum twi_operation_e g_twi_operation = TWI_OP_NONE;
volatile int g_twi_return_status = 0;

volatile uint8_t g_twi_busy = 0;
volatile uint8_t g_twi_slaveaddr;
const uint8_t* g_twi_send_buffer;
volatile uint8_t g_twi_send_size = 0;
volatile uint8_t *g_twi_mr_recv_buffer;
volatile uint8_t g_twi_mr_recv_index = 0;
volatile uint8_t g_twi_mr_recv_size = 0;
volatile uint8_t g_twi_sr_recv_buffer[128];
volatile uint8_t g_twi_sr_recv_index = 0;

volatile uint8_t g_twi_send_prebuf[128];
volatile uint8_t g_twi_send_prebuf_n = 0;

ISR(TWI_vect)
{
  uint8_t sreg = SREG;
  int i;
  static uint8_t bytes_sent;
  //sprintf(buf, "TW_STATUS: 0x%02x\n", TW_STATUS);
  //_delay_ms(10);
  //serialWriteString(buf);
  //serialWriteByte(TW_STATUS);
  switch(TW_STATUS)
  {
    /* MASTER TRANSMITTER STATES */
    case TW_START:
      if(g_twi_operation == TWI_OP_NONE) {
        g_twi_busy = 0;
        goto twi_reset_state_vars;
      }
      g_twi_busy = 1;
      g_twi_return_status = -1;
      bytes_sent = 0;
      /* Start has been sent. Must load slave address and read/write now */
      switch(g_twi_operation) {
        case TWI_OP_SEND:
        case TWI_OP_SEND_RECV:
          TWDR = (g_twi_slaveaddr<<1);
          break;
        default:
          TWDR = (g_twi_slaveaddr<<1) | (0x01);
          break;
      }
      TWCR &= ~(1<<TWSTA | 1<<TWSTO);
      TWCR |= (1<<TWINT);
      break;
    case TW_REP_START:
      if(g_twi_operation == TWI_OP_NONE) {
        g_twi_busy = 0;
        goto twi_reset_state_vars;
      }
      /* Repeated start has been sent. Send sla+r */
      TWDR = (g_twi_slaveaddr<<1) | 1;
      TWCR &= ~(1<<TWSTA | 1<<TWSTO);
      TWCR |= (1<<TWINT);
      break;
    case TW_MT_SLA_ACK:
    case TW_MT_DATA_ACK:  
      /* Check to see if all of the data has been sent */
      if(bytes_sent >= g_twi_send_size) {
        if(g_twi_operation == TWI_OP_SEND_RECV) {
          /* Issue repeated start */
          TWCR |= (1<<TWSTA | 1<<TWINT);
          TWCR &= ~(1<<TWSTO);
        } else {
          /* Issue stop */
          TWCR &= ~(1<<TWSTA);
          TWCR |= (1<<TWSTO | 1<<TWINT);
          g_twi_return_status = 0;
          goto twi_reset_state_vars;
        }
      } else {
        /* Not all data has been sent yet. Send another byte */
        TWDR = g_twi_send_buffer[bytes_sent];
        TWCR &= ~(1<<TWSTA | 1<<TWSTO);
        TWCR |= 1<<TWINT;
        bytes_sent++;
      }
      break;
    case TW_MT_ARB_LOST:
      TWCR &= ~(1<<TWSTO);
      TWCR |= (1<<TWSTA) | (1<<TWINT);
      break;

    /* MASTER RECEIVER STATES */
    case TW_MR_SLA_ACK:
      g_twi_mr_recv_index = 0;
      TWCR &= ~(1<<TWSTA | 1<<TWSTO);
      TWCR |= (1<<TWINT) | (1<<TWEA);
      break;

    case TW_MR_DATA_ACK:
      /* Read data byte */
      g_twi_mr_recv_buffer[g_twi_mr_recv_index] = TWDR;
      g_twi_mr_recv_index++;
      /* Have we received all of the data? */
      if( (g_twi_mr_recv_index+1) >= g_twi_mr_recv_size ) {
        /* Make next byte return NACK */
        TWCR &= ~(1<<TWSTA | 1<<TWSTO | 1<<TWEA);
        TWCR |= 1<<TWINT;
      } else {
        TWCR &= ~(1<<TWSTA | 1<<TWSTO);
        TWCR |= (1<<TWINT) | (1<<TWEA);
      }
      break;

    case TW_MR_DATA_NACK:
      /* Read last byte */
      g_twi_mr_recv_buffer[g_twi_mr_recv_index] = TWDR;
      g_twi_mr_recv_index++;
      /* Send stop */
      TWCR &= ~(1<<TWSTA);
      TWCR |= (1<<TWSTO | 1<<TWINT);
      /* Data is ready */
      g_twi_return_status = 0;
      goto twi_reset_state_vars;
      break;
   
    /* SLAVE RECEIVER STATES */
    case TW_SR_SLA_ACK:
      g_twi_busy = 1;
      g_twi_sr_recv_index = 0;
      /* Respond with ACK */
      TWCR &= ~(1<<TWSTO);
      TWCR |= (1<<TWINT) | (1<<TWEA);
      break;
    case TW_SR_ARB_LOST_SLA_ACK:
      TWCR &= ~(1<<TWSTO | 1<<TWEA);
      TWCR |= 1<<TWINT;
      break;

    case TW_SR_DATA_ACK:
      /* Receive a byte */
      g_twi_sr_recv_buffer[g_twi_sr_recv_index] = TWDR;
      //serialWriteByte(TWDR);
      g_twi_sr_recv_index++;
      /* Respond with ACK */
      TWCR &= ~(1<<TWSTO);
      TWCR |= (1<<TWINT) | (1<<TWEA);
      break;
    case TW_SR_STOP:
      /* Received whole packet. Reset state vars and forward to state handler */
      /*
      sprintf(buf, 
          "%d Received packet containing data: 0x%02x\n", 
          g_twi_sr_recv_index, g_twi_sr_recv_buffer[0]);
      serialWriteString(buf);
      */
      /* No need to send zigbee portion of message */
      for(i = 5; i < g_twi_sr_recv_index-1; i++) {
        g_serialBufferOut[(g_serialBufferOutN + g_serialBufferOutIndex)%SERIAL_BUFFER_SIZE] = 
          g_twi_sr_recv_buffer[i];
        g_serialBufferOutN++;
      }
      g_twi_return_status = 0;
      TWCR &= ~(1<<TWSTA | 1<<TWSTO);
      TWCR |= 1<<TWINT | 1<<TWEA;
      goto twi_reset_state_vars;
      break;
    case TW_BUS_ERROR:
      TWCR &= ~(1<<TWSTA);
      TWCR |= (1<<TWSTO)|(1<<TWINT);
      goto twi_reset_state_vars;

    case TW_NO_INFO:
      break;

    default:
      TWCR |= 1<<TWINT;
      goto twi_reset_state_vars;
  }
  SREG = sreg;
  return;

twi_reset_state_vars:
  TWAR = (0x02) << 1;
  TWCR &= ~(1<<TWSTA | 1<<TWSTO);
  TWCR |= (1<<TWEA | 1<<TWEN);
  bytes_sent = 0;
  g_twi_state = TWI_IDLE;
  g_twi_operation = TWI_OP_NONE;
  g_twi_slaveaddr = 0;
  g_twi_send_size = 0;
  g_twi_mr_recv_size = 0;
  g_twi_sr_recv_index = 0;
  g_twi_busy = 0;
  SREG = sreg;
  return;
}

void TWIInit(void)
{
	// Initialize Two-Wire Interface (I2C)
	TWSR = 0x00;  // clear TWI status reg, prescaler=0
	//TWBR = 0x48;  // use 0x0C (12) for SCL freq = 400kHz or
	TWBR = 18;  // use (3) for SCL freq = 400kHz or
					// use (18) for SCL freq = 100kHz
	/* 
	Formula for SCL freq:
	freq_SCL = freq_CPU / (16 + (2*TWBR)*(4^TWPS))
	where TWBR is TWI bit rate and TWPS is TWI prescaler
	From p.384 in mega128RFA1 datasheet
	*/

  /* Set up the address to 0x02 */
  TWAR = (0x02) << 1;

	TWCR = (1<<TWEA) | (1<<TWEN) | (1<<TWIE);  // enable Ack and TWI and interrupt
} // TWIInit()

int TWISend(uint8_t addr, const uint8_t *buf, uint8_t size)
{
  if(g_twi_busy) {
    return -1;
  }
  g_twi_send_buffer = buf;
  g_twi_send_size = size;
  g_twi_slaveaddr = addr;
  g_twi_operation = TWI_OP_SEND;
  g_twi_busy = 1;
  /* Emit start */
	TWCR |= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  return 0;
}

int TWIRecv(uint8_t addr, uint8_t *buf, uint8_t size)
{
  if(g_twi_busy) {
    return -1;
  }
  g_twi_mr_recv_buffer = buf;
  g_twi_mr_recv_size = size;
  g_twi_mr_recv_index = 0;
  g_twi_slaveaddr = addr;
  g_twi_operation = TWI_OP_RECV;
  g_twi_busy = 1;
  /* Emit start */
	TWCR |= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  return 0;
}

int TWISendRecv(uint8_t addr, 
    const uint8_t *sendBuf,
    uint8_t sendSize,
    uint8_t *recvBuf,
    uint8_t recvSize)
{
  if(g_twi_busy) {
    return -1;
  }
  g_twi_slaveaddr = addr;
  g_twi_send_buffer = sendBuf;
  g_twi_send_size = sendSize;
  g_twi_mr_recv_buffer = recvBuf;
  g_twi_mr_recv_index = 0;
  g_twi_mr_recv_size = recvSize;
  g_twi_operation = TWI_OP_SEND_RECV;
  g_twi_return_status = -1;
  g_twi_busy = 1;
  /* Emit start */
	TWCR |= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  return 0;
}

int TWIWait()
{
  while(g_twi_busy) {
    asm("nop");
  }
  return g_twi_return_status;
}

void TWIHandler()
{
  if(g_twi_send_prebuf_n) {
    TWISend(0x01, g_twi_send_prebuf, g_twi_send_prebuf_n);
    g_twi_send_prebuf_n = 0;
  }
}
