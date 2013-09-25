
/*
 * dof.c
 *
 * Created: 7/15/2012 11:37:36 PM
 *  Author: Benny Brown
 */ 

#include "twi.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "serial.h"
#include "twi.h"

volatile unsigned long g_millis;

volatile uint8_t* DDR_REGS[14];
volatile uint8_t* PORT_REGS[14];
volatile uint8_t* PIN_REGS[14];
uint8_t PINS[14];

volatile uint8_t* PWM_COM_REGS[14];
volatile uint8_t* PWM_COMB_REGS[14];
uint8_t PWM_COM0_PIN[14];
uint8_t PWM_COM1_PIN[14];
volatile uint8_t* PWM_OCR_REGS[14];


ISR(TIMER1_COMPA_vect)
{
  uint8_t sreg = SREG;
  g_millis++;
  SREG = sreg;
}

void AVRInit(void)
{
  /* Set the clock prescalar to 1 */
  CLKPR = (1<<CLKPCE);
  //CLKPR = (1<<CLKPS0);
  CLKPR = 0;

  cli();

#if 0
  /* Use Timer 1 as a millisecond timer */
  TCNT1 = 0;
  //TIMSK1_struct.ocie1a = 1; // Enable Timer1 CTC interrupt
  TIMSK1 |= (1<<OCIE1A);
  TCCR1B |= (1<<WGM12) | (1<<CS12); // Enable CTC mode, 256 prescalar, start timer
  OCR1A = (F_CPU / (256000) );
  ICR1 = (F_CPU / (256000) );
#endif

  /* Set PC0 to input */
  DDRC &= ~(1<<0);
  PORTC &= ~(1<<0);

  /* Initialize PWMs, but don't start them */
  /* Initialize all PWMS to 8-bit Fast PWM */
  TCCR0A |= ((1<<WGM00)|(1<<WGM01));
  TCCR1A |= (1<<WGM10);
  TCCR1B |= (1<<WGM12);
  TCCR2A |= ((1<<WGM20)|(1<<WGM21));
  /* DEBUG 
  DDRD |= (1<<3);
  PORTD &= ~(1<<3);
  TCCR2A |= (1<<COM2B1);
  TCCR2B |= (1<<CS22);
  OCR2B = 255;
  */

  /* Initialize ADC to default setting */
  ADMUX = 0x40;

  sei();

} // AVRInit

/* Mapping:
   IO0 <-> PD0
   IO1 <-> PD1
   ...     ...
   IO7 <-> PD7
   IO8 <-> PB0
   IO9 <-> PB1
   IO10<-> PB2
   IO11<-> PB3
   IO12<-> PB4
   IO13<-> PB5
   */

/* PWM Mapping:
   IO3 <-> OC2B
   IO5 <-> OC0B
   IO6 <-> OC0A
   IO9 <-> OC1A
   IO10<-> OC1B
   IO11<-> OC2A
   */

void PortsInit(void)
{
  int i;
  for(i = 0; i < 8; i++) {
    PORT_REGS[i] = &PORTD;
    PIN_REGS[i] = &PIND;
    PINS[i] = i;
    DDR_REGS[i] = &DDRD;
  }
  for(i = 8; i < 14; i++) {
    PORT_REGS[i] = &PORTB;
    PIN_REGS[i] = &PINB;
    PINS[i] = i-8;
    DDR_REGS[i] = &DDRB;
  }

  PWM_COM_REGS[3] = &TCCR2A;
  PWM_COM_REGS[5] = &TCCR0A;
  PWM_COM_REGS[6] = &TCCR0A;
  PWM_COM_REGS[9] = &TCCR1A;
  PWM_COM_REGS[10] = &TCCR1A;
  PWM_COM_REGS[11] = &TCCR2A;

  PWM_COMB_REGS[3] = &TCCR2B;
  PWM_COMB_REGS[5] = &TCCR0B;
  PWM_COMB_REGS[6] = &TCCR0B;
  PWM_COMB_REGS[9] = &TCCR1B;
  PWM_COMB_REGS[10] = &TCCR1B;
  PWM_COMB_REGS[11] = &TCCR2B;

  PWM_COM0_PIN[3] = 4;
  PWM_COM1_PIN[3] = 5;

  PWM_COM0_PIN[5] = 4;
  PWM_COM1_PIN[5] = 5;

  PWM_COM0_PIN[6] = 6;
  PWM_COM1_PIN[6] = 7;

  PWM_COM0_PIN[9] = 6;
  PWM_COM1_PIN[9] = 7;

  PWM_COM0_PIN[10] = 4;
  PWM_COM1_PIN[10] = 5;

  PWM_COM0_PIN[11] = 6;
  PWM_COM1_PIN[11] = 7;

  PWM_OCR_REGS[3] = &OCR2B;
  PWM_OCR_REGS[5] = &OCR0B;
  PWM_OCR_REGS[6] = &OCR0A;
  PWM_OCR_REGS[9] = &OCR1AL;
  PWM_OCR_REGS[10] = &OCR1BL;
  PWM_OCR_REGS[11] = &OCR2A;
}

#pragma GCC optimize ("O0")
/* Do not optimize this function. If the compiler optimizes this function, it
 * doesn't know that g_millis is constantly changing and loops forever.*/
void delay(unsigned long ms)
{
  unsigned long starttime, endtime;
  starttime = g_millis;
  endtime = starttime + ms;
  while(g_millis <= endtime);
}
#pragma GCC optimize ("02")

uint16_t getADC(uint8_t adc)
{
  uint16_t value = 0;
  /* Initialize ADC */
  /* Set prescalar to 128, conversion frequency of 125kHz */
  //ADCSRC = 10<<ADSUT0;

  // Select the right channel
  ADMUX &= 0xE0;
  ADMUX |= adc;
  ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN) | (1<<ADSC); // Set prescalar, enable
  // Wait for the conversion to finish 
  while(ADCSRA & (1<<ADSC));
  // Compose the return value
  //value = ADC;
  value = ADCL;
  value |= (ADCH & 0x03) << 8;
  //ADCSRA = 0;
  return value;

  /* Set TWI pins to input, PC4 and PC5, to input */
  DDRC &= ~(1<<4 | 1<<5);
  PORTC &= ~(1<<4 | 1<<5);
}

int main(void)
{
  uint8_t buf[20]; // send 0x30 0x8 0x0 0x0 0x1 0x30 0x3 0x0
  buf[0] = 0x6c;
  buf[1] = 0x08;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x01;
  buf[5] = 0x6c;
  buf[6] = 0x03;
  buf[7] = 0x00;
  PortsInit();
	AVRInit();
	USARTInit();
  TWIInit();
  while(1) {
    serialHandler();
    //TWISend(0x01, buf, buf[1]);
    //TWIWait();
    //delay(3000);
    TWIHandler();
  }
} // main()

