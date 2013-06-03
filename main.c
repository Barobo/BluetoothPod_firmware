
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

  /* Use Timer 1 as a millisecond timer */
  TCNT1 = 0;
  //TIMSK1_struct.ocie1a = 1; // Enable Timer1 CTC interrupt
  TIMSK1 |= (1<<OCIE1A);
  TCCR1B |= (1<<WGM12) | (1<<CS12); // Enable CTC mode, 256 prescalar, start timer
  OCR1A = (F_CPU / (256000) );
  ICR1 = (F_CPU / (256000) );

  sei();

} // AVRInit

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

