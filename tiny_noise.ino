/*
 * Noise generator for ATTINT85, by Peter Gaggs
 * Uses 32 bit LFSR, output on PB0 (pin 5 of IC)
 * MIT License
 * Copyright (c) 2019 petegaggs
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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

