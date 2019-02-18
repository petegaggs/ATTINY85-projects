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
#define LFO_RESET_PIN 0 //pin 4 of IC. Optional. Leave disconnected if not used

#define LFO_PWM OCR1A

// table of 256 sine values / one sine period / stored in flash memory
const char sineTable[] PROGMEM = {
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124
};

// LFO stuff
uint32_t lfoPhaccu;   // phase accumulator
uint32_t lfoTword_m;  // dds tuning word m
uint8_t lfoCnt;      // top 8 bits of accum is index into table
bool lfoReset = false;

float lfoControlVoltage;
enum lfoWaveTypes {
  RAMP,
  SAW,
  TRI,
  SINE,
  SQR
};
lfoWaveTypes lfoWaveform;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LFO_RESET_PIN, INPUT);
  digitalWrite(LFO_RESET_PIN, HIGH); // enable internal pullup
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
  PCMSK = (1 << PCINT0); // enable pin change interrupt on PB0
  GIMSK = (1 << PCIE); // enable pin change interrupts
  interrupts();
}

void getLfoParams() {
  // read ADC to calculate the required DDS tuning word, log scale between 0.01Hz and 10Hz approx
  float lfoControlVoltage = float(analogRead(LFO_FREQ_PIN)) * float(10)/float(1024); //gives 10 octaves range 0.01Hz to 10Hz
  lfoTword_m = float(1369) * pow(2.0, lfoControlVoltage); //1369 sets the lowest frequency to 0.01Hz
  // read ADC to get the LFO wave type
  int adcVal = analogRead(LFO_WAVE_PIN);
  if (adcVal < 128) {
    lfoWaveform = RAMP;
  } else if (adcVal < 384) {
    lfoWaveform = SAW;
  } else if (adcVal < 640) {
    lfoWaveform = TRI;
  } else if (adcVal < 896) {
    lfoWaveform = SINE;
  } else {
    lfoWaveform = SQR;
  }
}

void loop() {
  getLfoParams();
}

ISR(PCINT0_vect) {
  if (digitalRead(0)) {
    lfoReset = true;
  }
}

ISR(TIMER0_COMPA_vect) {
  // handle LFO DDS
  if (lfoReset) {
    lfoPhaccu = 0; // reset the lfo
    lfoReset = false;
  } else {
    lfoPhaccu += lfoTword_m; // increment phase accumulator
  }
  lfoCnt = lfoPhaccu >> 24;  // use upper 8 bits for phase accu as frequency information
  switch (lfoWaveform) {
    case RAMP:
      LFO_PWM = lfoCnt;
      break;
    case SAW:
      LFO_PWM = 255 - lfoCnt;
      break;
    case TRI:
      if (lfoCnt & 0x80) {
        LFO_PWM = 254 - ((lfoCnt & 0x7F) << 1); //ramp down
      } else {
        LFO_PWM = lfoCnt << 1; //ramp up
      }
      break;
    case SINE:
      // sine wave from table
      LFO_PWM = pgm_read_byte_near(sineTable + lfoCnt);
      break;
    case SQR:
      if (lfoCnt & 0x80) {
        LFO_PWM = 255;
      } else {
        LFO_PWM = 0;
      }
      break;
    default:
      break;
  }

}




