/*
 * Envelope generator for ATTINY85
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

#define ADS_BUILD // only needs 3 ADCs, no need to blow fuses
//#define ADSR_BUILD // note! need to blow fuse RSTDISBL
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define GATE_PIN 0 // pin 5 of IC
#define PWM_PIN 1 //OC1A pin 6 of IC
#define ENV_ATTACK_PIN A1 //pin 7 of IC
#define ENV_DECAY_PIN A2 //pin 3 of IC
#define ENV_SUSTAIN_PIN A3 //pin 2 of IC
#define ENV_RELEASE_PIN A0 // pin 1 of IC: (optional), only for ADSR build, need to blow fuse RSTDISBL
#define ENV_PWM OCR1A

// table of exponential rising waveform for envelope gen
// this one is shorter than the origninal (3.5xTCs instead of 5) and kludged to get to full scale
const char expTable[] PROGMEM = {
0,4,7,11,14,17,21,24,27,30,34,37,40,43,46,49,52,55,57,60,63,66,68,71,74,76,79,81,84,86,88,91,93,96,98,100,102,104,107,
109,111,113,115,117,119,121,123,125,127,128,130,132,134,136,137,139,141,142,144,146,147,149,150,152,153,155,156,158,159,161,
162,163,165,166,167,169,170,171,172,174,175,176,177,178,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,
197,198,199,200,200,201,202,203,204,205,205,206,207,208,208,209,210,211,211,212,213,213,214,215,215,216,217,217,218,219,219,
220,220,221,221,222,223,223,224,224,225,225,226,226,227,227,228,228,229,229,230,230,231,231,231,232,232,233,233,233,234,234,
235,235,235,236,236,237,237,237,238,238,238,239,239,239,240,240,240,241,241,241,241,242,242,242,243,243,243,243,244,244,244,
244,245,245,245,245,246,246,246,246,247,247,247,247,247,248,248,248,248,249,249,249,249,249,249,250,250,250,250,250,251,251,
251,251,251,251,252,252,252,252,252,252,252,253,253,253,253,253,253,253,254,254,254,254,254,254,254,254,254,255,255,255,255
};

// envelope stuff
uint32_t envPhaccu;   // phase accumulator
uint32_t envAttackTword;  // dds tuning word attack stage
uint32_t envReleaseTword;  // dds tuning word release stage
uint32_t envDecayTword;  // dds tuning word decay stage
uint8_t envSustainControl; // envelope sustain control 0-255
uint8_t envCnt;      // top 8 bits of accum is index into table
uint8_t lastEnvCnt;
uint8_t envCurrentLevel; // the current level of envelope
uint8_t envStoredLevel; // the level that the envelope was at start of release stage
uint8_t envMultFactor; // multiplication factor to account for release starting before attack complete and visa versa
float envControlVoltage;


enum envStates {
  WAIT,
  START_ATTACK,
  ATTACK,
  START_DECAY,
  DECAY,
  SUSTAIN,
  START_RELEASE,
  RELEASE
};
envStates envState;


void setup() {
  pinMode(GATE_PIN, INPUT);
  digitalWrite(GATE_PIN, HIGH); // enable pull up
  pinMode(PWM_PIN, OUTPUT);
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
  envState = WAIT;
}

void getEnvParams() {
  float envControlVoltage;
  // read ADC to calculate the required DDS tuning word, log scale between 1ms and 10s approx
  envControlVoltage = float(1023 - analogRead(ENV_ATTACK_PIN)) * 13/1024; //gives 13 octaves range 1ms to 10s
  envAttackTword = 13690 * pow(2.0, envControlVoltage); //13690 sets the longest rise time to 10s
  envControlVoltage = float(1023 - analogRead(ENV_DECAY_PIN)) * 13/1024; //gives 13 octaves range 1ms to 10s
  envDecayTword = 13690 * pow(2.0, envControlVoltage); //13690 sets the longest rise time to 10s
  envSustainControl = analogRead(ENV_SUSTAIN_PIN) >> 2; //0 to 255 level for sustain control
  #ifdef ADSR_BUILD
  envControlVoltage = float(1023 - analogRead(ENV_RELEASE_PIN)) * 13/1024; //gives 13 octaves range 1ms to 10s
  envReleaseTword = 13690 * pow(2.0, envControlVoltage); //13690 sets the longest rise time to 10s
  #endif
  #ifdef ADS_BUILD
  envReleaseTword = envDecayTword;
  #endif
}


void loop() {
  getEnvParams();
}

ISR(PCINT0_vect) {
  if (digitalRead(0)) {
    if (envState != ATTACK) {
      envState = START_ATTACK;
    } 
  } else {
      envState = START_RELEASE;
  }
}

ISR(TIMER0_COMPA_vect) {
  // handle Envelope DDS
  switch (envState) {
    case WAIT:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      envCurrentLevel = 0;
      ENV_PWM = 0;
      break;
    case START_ATTACK:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      envMultFactor = 255 - envCurrentLevel;
      envStoredLevel = envCurrentLevel;
      envState = ATTACK;
      break;
    case ATTACK:
      envPhaccu += envAttackTword; // increment phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt < lastEnvCnt) {
        envState = START_DECAY; // end of attack stage when counter wraps
      } else {
        envCurrentLevel = ((envMultFactor * pgm_read_byte_near(expTable + envCnt)) >> 8) + envStoredLevel;
        ENV_PWM = envCurrentLevel;
        lastEnvCnt = envCnt;
      }
      break;
    case START_DECAY:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      envMultFactor = 255 - envSustainControl;
      envState = DECAY;
      break;
    case DECAY:
      envPhaccu += envDecayTword; // increment phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt < lastEnvCnt) {
        envState = SUSTAIN; // end of release stage when counter wraps
      } else {
        envCurrentLevel = 255 - ((envMultFactor * pgm_read_byte_near(expTable + envCnt)) >> 8);
        ENV_PWM = envCurrentLevel;
        lastEnvCnt = envCnt;
      }
      break;
    case SUSTAIN:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      envCurrentLevel = envSustainControl;
      ENV_PWM = envCurrentLevel;
      break;
    case START_RELEASE:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      envMultFactor = envCurrentLevel;
      envStoredLevel = envCurrentLevel;
      envState = RELEASE;
      break;
    case RELEASE:
      envPhaccu += envReleaseTword; // increment phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt < lastEnvCnt) {
        envState = WAIT; // end of release stage when counter wraps
      } else {
        envCurrentLevel = envStoredLevel - ((envMultFactor * pgm_read_byte_near(expTable + envCnt)) >> 8);
        ENV_PWM = envCurrentLevel;
        lastEnvCnt = envCnt;
      }
      break;
    default:
      break;
  }
}





