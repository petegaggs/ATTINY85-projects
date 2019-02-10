/*
 * Noise generator for ATTINT85
 * Uses 32 bit LFSR, output on PB0 (pin 5 of IC)
 */

#include <avr/io.h> 

uint32_t lfsr = 1; //32 bit LFSR, must be non-zero to start

void setup() {
  pinMode(0,OUTPUT); // this pins gives out the noise
  // Set up timer 1 to generate interrupt at 31.25kHz
  TCCR1 = _BV(CS10);
  TIMSK = _BV(TOIE1);
}

void loop() {
}

SIGNAL(TIMER1_OVF_vect) {
  // set or clear noise pin PB0
  unsigned lsb = lfsr & 1;
  if (lsb) {
    PORTB |= 1;
  }
  else {
    PORTB &= ~1;
  }
  // advance LFSR
  lfsr >>= 1;
  if (lsb) {
    lfsr ^= 0xA3000000u;
  }
}

