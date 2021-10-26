/*
 * LFO generator for ATTINY85
 * By Peter Gaggs
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
 
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define PWM_PIN 1 //OC1A pin 6 of IC
#define LFO_FREQ_PIN A2 //pin 3 of IC
#define LFO_WAVE_PIN A3 //pin 2 of IC
#define ISR_TEST_PIN 0 //pin 5 of IC
#define RANGE_SEL_PIN 2 //pin 7 of IC
//#define ISR_TEST_BUILD //test the ISR timing
#define LFO_PWM OCR1A

// LFO stuff
bool triPhase = true; // rising or falling phase for triangle wave
uint32_t lfoPhaccu;   // phase accumulator
uint32_t lfoTword_m;  // dds tuning word m
uint8_t lfoCnt = 0;      // top 8 bits of accum is index into table
uint8_t lastLfoCnt = 0;
uint8_t pwmSet; // whole part of pwm
uint8_t ditherByte; // random dither
uint8_t pwmFrac; // fractional part of pwm
uint32_t lfsr = 1; //32 bit LFSR, must be non-zero to startfloat lfoControlVoltage;
enum lfoWaveTypes {
  RAMP,
  SAW,
  TRI,
  SQR,
  SAMPLE_AND_HOLD,
  STEPSUP,
  STEPSDOWN
};
lfoWaveTypes lfoWaveform;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  #ifdef ISR_TEST_BUILD
  pinMode(ISR_TEST_PIN, OUTPUT);
  #endif
  pinMode(RANGE_SEL_PIN, INPUT);
  digitalWrite(RANGE_SEL_PIN, HIGH); // enable internal pullup
  PLLCSR = _BV(PLLE); // enable PLL
  delay(10); //wait a bit to allow PLL to lock
  while (PLLCSR & _BV(PLOCK) == 0){
    // ensure PLL is locked
  }
  PLLCSR |= _BV(PCKE); // set PCK to 64MHz
  // configure PWM timer 1 channel A
  TCCR1 = _BV(CS10)| _BV(COM1A1) | _BV(PWM1A);
  // Set up timer 0 to generate interrupt at 31.25kHz
  noInterrupts();
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;
  OCR0A = 255;
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS00);
  TIMSK = (1 << OCIE0A);
  interrupts();
}

void getLfoParams() {
  // read ADC to calculate the required DDS tuning word, range 0.01Hz and 10Hz approx
  uint32_t tempVal = analogRead(LFO_FREQ_PIN);
  if (digitalRead(RANGE_SEL_PIN) == 1) {
    // high range, around 1Hz to 31Hz
    if (lfoWaveform == TRI) {
      lfoTword_m = (tempVal << 13) + 262144;
    } else {
      lfoTword_m = (tempVal << 12) + 131072;
    }
  } else {
    // low range, around 0.007Hz to 2Hz
    if (lfoWaveform == TRI) {
      lfoTword_m = (tempVal << 9) + 2048;
    } else {
      lfoTword_m = (tempVal << 8) + 1024;
    }    
  }
  // read ADC to get the LFO wave type
  int waveType = analogRead(LFO_WAVE_PIN) >> 7;
  switch (waveType) {
    case 0:
      lfoWaveform = RAMP;
      break;
    case 1:
      lfoWaveform = SAW;
      break;
    case 2:
      lfoWaveform = TRI;
      break;
    case 3:
      lfoWaveform = SQR;
      break;
    case 4:
      lfoWaveform = SAMPLE_AND_HOLD;
      break;
    case 5:
      lfoWaveform = STEPSUP;
      break;
    case 6:
      lfoWaveform = STEPSDOWN;
      break;
    case 7:
      lfoWaveform = RAMP; // reserved
      break;
    default:
      lfoWaveform = RAMP; // reserved
      break;    
  }
}

void loop() {
  getLfoParams();
}

ISR(TIMER0_COMPA_vect) {
  #ifdef ISR_TEST_BUILD
  PORTB |= 0x01; // set PB0 high
  #endif
  // LFSR, for dither
  unsigned lsb = lfsr & 1;
  lfsr >>= 1;
  if (lsb) {
    lfsr ^= 0xA3000000u;
  }
  // handle LFO DDS
  lfoPhaccu += lfoTword_m; // increment phase accumulator
  lfoCnt = lfoPhaccu >> 24;  // use upper 8 bits for phase accu as frequency information
  ditherByte = lfsr & 0xFF; // random dither
  pwmFrac = (lfoPhaccu >> 16) & 0xFF; // fractional part of 16 bit value
  switch (lfoWaveform) {
    case RAMP:
      pwmSet = lfoCnt; // whole part
      if ((pwmFrac > ditherByte) && (pwmSet < 255)) {
        pwmSet += 1;
      }     
      break;
    case SAW:
      pwmSet = 255 - lfoCnt; // whole part
      // note dither is done in reverse for this waveform
      if ((pwmFrac > ditherByte) && (pwmSet > 0)) {
        pwmSet -= 1;
      }
      break;
    case TRI:
      if (lfoCnt < lastLfoCnt) {
        triPhase = not(triPhase); // reverse direction
      }
      if (triPhase) {
        // rising (same as ramp)
        pwmSet = lfoCnt; // whole part
        if ((pwmFrac > ditherByte) && (pwmSet < 255)) {
          pwmSet += 1;
        }     
      } else {
        // falling (same as saw)
        pwmSet = 255 - lfoCnt; // whole part
        // note dither is done in reverse for this waveform
        if ((pwmFrac > ditherByte) && (pwmSet > 0)) {
          pwmSet -= 1;
        }
      }
      break;
    case SQR:
      if (lfoCnt & 0x80) {
        pwmSet = 255;
      } else {
        pwmSet = 0;
      }
      break;
    case SAMPLE_AND_HOLD:
      if (lfoCnt < lastLfoCnt) {
        pwmSet = ditherByte; // random
      }
      break;
    case STEPSUP:
      pwmSet = lfoCnt & 0xE0;
      break;
    case STEPSDOWN:
      pwmSet = (255 - lfoCnt) & 0xE0;
      break;
    default:
      break;
  }
  LFO_PWM = pwmSet;
  lastLfoCnt = lfoCnt;
  #ifdef ISR_TEST_BUILD
  PORTB &= ~0x01; // set PB0 low
  #endif
}
