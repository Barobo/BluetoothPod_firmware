#ifndef _TWI_H_
#define _TWI_H_
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>

enum twi_state_e {
  TWI_IDLE,
  TWI_START_DONE,
  TWI_SLA_RW_DONE,
  TWI_DATA_DONE,
};

enum twi_operation_e {
  TWI_OP_SEND,
  TWI_OP_RECV,
  TWI_OP_SEND_RECV,
  TWI_OP_NONE,
};

extern volatile enum twi_state_e g_twi_state;
volatile enum twi_operation_e g_twi_operation;
extern volatile int g_twi_return_status;

extern volatile uint8_t g_twi_send_prebuf[128];
extern volatile uint8_t g_twi_send_prebuf_n;

extern volatile uint8_t* DDR_REGS[14];
extern volatile uint8_t* PORT_REGS[14];
extern volatile uint8_t* PIN_REGS[14];
extern uint8_t PINS[14];

extern volatile uint8_t* PWM_COM_REGS[14];
extern uint8_t PWM_COM0_PIN[14];
extern uint8_t PWM_COM1_PIN[14];
extern volatile uint8_t* PWM_OCR_REGS[14];

void TWIInit(void);
int TWISend(uint8_t addr, const uint8_t *buf, uint8_t size);
int TWIRecv(uint8_t addr, uint8_t *buf, uint8_t size);
int TWISendRecv(uint8_t addr, 
    const uint8_t *sendBuf,
    uint8_t sendSize,
    uint8_t *recvBuf,
    uint8_t recvSize);
int TWIWait();
void TWIHandler();
void processTWIMessage();

void setPinMode(int pin, int mode);
void digitalWritePin(int pin, int value);
void analogWritePin(int pin, uint8_t value);
void analogReadPin(int pin);
void analogSetRef(int ref);
#endif
