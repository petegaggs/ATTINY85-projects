/*
 * Noise generator for ATTINT85, by Peter Gaggs
 * Uses 32 bit LFSR, output by PWM on OCR1A (pin 6 of IC)
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
#define PWM_PIN 1 //OC1A pin 6 of IC
#define NOISE_PWM OCR1A

uint32_t lfsr = 1; //32 bit LFSR, must be non-zero to start

void setup() {
  pinMode(PWM_PIN,OUTPUT); // this pins gives out the noise
  PLLCSR = _BV(PLLE); // enable PLL
  delay(10); //wait a bit to allow PLL to lock
  while (PLLCSR & _BV(PLOCK) == 0){
    // ensure PLL is locked
  }
  PLLCSR |= _BV(PCKE); // set PCK to 64MHz
  // Set up timer 1 to generate interrupt at 31.25kHz
  // configure PWM timer 1 channel A
  TCCR1 = _BV(CS10)| _BV(COM1A1) | _BV(PWM1A);
  TIMSK = _BV(TOIE1);
}

void loop() {
}

SIGNAL(TIMER1_OVF_vect) {
  // output lower 8 bits of lfsr on pwm
  NOISE_PWM = lfsr & 0xFF;
  unsigned lsb = lfsr & 1;
  // advance LFSR
  lfsr >>= 1;
  if (lsb) {
    lfsr ^= 0xA3000000u;
  }
}

