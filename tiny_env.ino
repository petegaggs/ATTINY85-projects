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
#define ISR_TEST_BUILD // use a GPIO to measure ISR time
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define GATE_PIN 0 // pin 5 of IC
#define PWM_PIN 1 //OC1A pin 6 of IC
#define ENV_ATTACK_PIN A1 //pin 7 of IC
#define ENV_DECAY_PIN A2 //pin 3 of IC
#ifdef ISR_TEST_BUILD
#define ENV_SUSTAIN_PIN A2 // same pin as decay control
#else
#define ENV_SUSTAIN_PIN A3 //pin 2 of IC
#endif
#define ISR_TEST_PIN 3 //pin 2 of IC, replaces sustain control for ISR test build, note this is PB3
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

// 16-bit version of exponential waveform in 2 tables, one for msbyte, one for lsbyte
//const char expTableHighByte[] PROGMEM = {
//0,3,7,10,
//13,17,20,23,27,30,33,36,39,42,45,48,51,54,57,60,62,65,68,70,73,76,78,81,83,86,88,90,93,95,97,99,102,104,106,
//108,110,112,114,116,118,120,122,124,126,128,130,131,133,135,137,138,140,142,143,145,147,148,150,151,153,154,
//156,157,159,160,161,163,164,166,167,168,169,171,172,173,174,176,177,178,179,180,181,182,183,185,186,187,188,
//189,190,191,192,193,194,194,195,196,197,198,199,200,201,202,202,203,204,205,206,206,207,208,209,209,210,211,
//211,212,213,213,214,215,215,216,217,217,218,219,219,220,220,221,221,222,223,223,224,224,225,225,226,226,227,
//227,228,228,229,229,230,230,230,231,231,232,232,233,233,233,234,234,234,235,235,236,236,236,237,237,237,238,
//238,238,239,239,239,240,240,240,241,241,241,241,242,242,242,243,243,243,243,244,244,244,244,245,245,245,245,
//246,246,246,246,246,247,247,247,247,248,248,248,248,248,249,249,249,249,249,249,250,250,250,250,250,250,
//251,251,251,251,251,251,252,252,252,252,252,252,252,253,253,253,253,253,253,253,253,254,254,254,254,254,254,254,254
//};

//const char expTableLowByte[] PROGMEM = {
//0,145,23,144,253,95,180,254,61,112,152,181,
//200,207,204,191,168,134,90,37,230,157,75,239,139,29,166,39,159,14,117,211,41,120,190,252,50,
//97,136,168,192,209,218,221,217,206,188,163,132,94,49,255,198,135,66,246,165,78,242,143,39,185,70,
//206,80,205,69,184,37,142,242,81,171,1,81,158,230,41,104,162,217,11,57,99,137,170,200,226,249,11,26,
//37,44,48,48,45,38,28,15,254,234,211,185,156,123,88,49,8,219,172,122,69,14,211,150,86,20,207,136,
//62,242,163,82,254,168,80,245,152,57,216,117,15,168,62,211,101,245,132,16,155,35,170,47,178,51,179,
//49,173,40,160,24,141,1,116,228,84,194,46,153,2,106,209,54,153,252,93,189,27,120,212,47,136,225,56,
//141,226,54,136,217,42,121,199,20,95,170,244,61,133,204,18,87,155,222,32,97,162,225,32,94,155,215,
//18,77,135,192,248,47,102,156,209,5,57,108,159,208,1,50,97,144,191,237,26,70,114,158,200,242,28,69,
//110,149,189,228,10,48,85,122,158,194,229,8,42,76,110,143,175,207,239,14,45,75,105,135,164,193,221
//};  
// table as 16-bit values

const uint16_t expTable16[] PROGMEM = {
0,913,1815,
2704,3581,4447,5300,6142,6973,7792,8600,9397,10184,10959,11724,12479,13224,
13958,14682,15397,16102,16797,17483,18159,18827,19485,20134,20775,21407,22030,22645,23251,23849,
24440,25022,25596,26162,26721,27272,27816,28352,28881,29402,29917,30425,30926,31420,31907,32388,
32862,33329,33791,34246,34695,35138,35574,36005,36430,36850,37263,37671,38073,38470,38862,
39248,39629,40005,40376,40741,41102,41458,41809,42155,42497,42833,43166,43494,43817,44136,44450,
44761,45067,45369,45667,45961,46250,46536,46818,47097,47371,47642,47909,48172,48432,48688,48941,
49190,49436,49679,49918,50154,50387,50617,50844,51067,51288,51505,51720,51931,52140,52346,52549,
52750,52947,53142,53334,53524,53711,53896,54078,54258,54435,54610,54782,54952,55120,55285,55448,
55609,55768,55925,56079,56232,56382,56531,56677,56821,56964,57104,57243,57379,57514,57647,57778,
57907,58035,58161,58285,58408,58528,58648,58765,58881,58996,59108,59220,59330,59438,59545,59650,
59754,59857,59958,60057,60156,60253,60349,60443,60536,60628,60719,60808,60897,60984,61069,61154,
61238,61320,61401,61482,61561,61639,61716,61791,61866,61940,62013,62085,62156,62226,62295,62363,
62430,62496,62561,62626,62689,62752,62814,62875,62935,62994,63053,63111,63168,63224,63279,63334,
63388,63441,63493,63545,63596,63647,63696,63745,63794,63841,63888,63935,63981,64026,64070,64114,
64158,64200,64242,64284,64325,64366,64405,64445,64484,64522,64560,64597,64634,64670,64706,64741,
64776,64810,64844,64878,64911,64943,64975,65007,65038,65069,65099,65129,65159,65188,65217,65245
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
uint16_t tableVal;
// dither
uint32_t lfsr = 1; //32 bit LFSR, must be non-zero to start

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
  #ifdef ISR_TEST_BUILD
  pinMode(ISR_TEST_PIN, OUTPUT); //PB3
  #endif
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
  OCR0A = 3; //31250 / (3+1) = 7812.5Hz timer
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS02); // prescaler divide by 256 (8MHz/256=31250Hz)
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
  envAttackTword = 3422 * pow(2.0, envControlVoltage); //13690 sets the longest rise time to 10s
  envControlVoltage = float(1023 - analogRead(ENV_DECAY_PIN)) * 13/1024; //gives 13 octaves range 1ms to 10s
  envDecayTword = 3422 * pow(2.0, envControlVoltage); //13690 sets the longest rise time to 10s
  envSustainControl = analogRead(ENV_SUSTAIN_PIN) >> 2; //0 to 255 level for sustain control
  #ifdef ADSR_BUILD
  envControlVoltage = float(1023 - analogRead(ENV_RELEASE_PIN)) * 13/1024; //gives 13 octaves range 1ms to 10s
  envReleaseTword = 3422 * pow(2.0, envControlVoltage); //13690 sets the longest rise time to 10s
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
  #ifdef ISR_TEST_BUILD
  PORTB |= 0x8; // set PB3 high
  #endif
  // LFSR for dither
  unsigned lsb = lfsr & 1;
  // advance LFSR
  lfsr >>= 1;
  if (lsb) {
    lfsr ^= 0xA3000000u;
  }
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
        tableVal = pgm_read_word_near(expTable16 + envCnt);
        //envCurrentLevel = ((envMultFactor * pgm_read_byte_near(expTable + envCnt)) >> 8) + envStoredLevel;
        envCurrentLevel = ((envMultFactor * (tableVal >> 8)) >> 8) + envStoredLevel;
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
        tableVal = pgm_read_word_near(expTable16 + envCnt);
        //envCurrentLevel = 255 - ((envMultFactor * pgm_read_byte_near(expTable + envCnt)) >> 8);
        envCurrentLevel = 255 - ((envMultFactor * (tableVal >> 8)) >> 8);
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
        tableVal = pgm_read_word_near(expTable16 + envCnt);
        //envCurrentLevel = envStoredLevel - ((envMultFactor * pgm_read_byte_near(expTable + envCnt)) >> 8);
        envCurrentLevel = envStoredLevel - ((envMultFactor * (tableVal >> 8)) >> 8);
        ENV_PWM = envCurrentLevel;
        lastEnvCnt = envCnt;
      }
      break;
    default:
      break;
  }
  #ifdef ISR_TEST_BUILD
  PORTB &= ~0x8; // clear PB3
  #endif
}
