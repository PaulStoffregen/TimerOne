/*
 *  Interrupt and PWM utilities for 16 bit Timer1 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jérôme Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified April 2012 by Paul Stoffregen - portable to other AVR chips, use inline functions
 *  Modified again, June 2014 by Paul Stoffregen - support Teensy 3.x & even more AVR chips
 *  Modified July 2017 by Stoyko Dimitrov - added support for ATTiny85 except for the PWM functionality
 *  
 *  Modified October 2023 by David Caldwell - added support for UNO-R4 boards except for the PWM functionality
 *  
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of Creative Commons Attribution 3.0 United States License. 
 *  To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0/us/ 
 *  or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
 */

#ifndef TimerOne_h_
#define TimerOne_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "config/known_16bit_timers.h"
#if defined (__AVR_ATtiny85__)
#define TIMER1_RESOLUTION 256UL  // Timer1 is 8 bit
#elif defined(__AVR__)
#define TIMER1_RESOLUTION 65536UL  // Timer1 is 16 bit
#else
#define TIMER1_RESOLUTION 65536UL  // assume 16 bits for non-AVR chips
#endif

#if defined (ARDUINO_UNOR4_MINIMA) || defined (ARDUINO_UNOR4_WIFI)
#include "IRQManager.h"

#define AGT1_RESOLUTION 0xFFFF

typedef enum agt_opmode_t {
  TIMER,
  PULSE_OUTPUT,
  EVENT_COUNTER,
  PULSE_WIDTH,
  PULSE_PERIOD
};

typedef enum agt_cntsrc_t {
  PCLKB,
  PCLKB_8,
  PCLKB_2 = 3,
  AGTLCLK,
  AGT0_UF,
  AGTSCLK
};
#endif

// Placing nearly all the code in this .h file allows the functions to be
// inlined by the compiler.  In the very common case with constant values
// the compiler will perform all calculations and simply write constants
// to the hardware registers (for example, setPeriod).

class TimerOne {
#if defined (ARDUINO_UNOR4_MINIMA) || defined (ARDUINO_UNOR4_WIFI)
	// Adding support for R4 UNO boards with RA4M1 chip
private:
  static agt_cntsrc_t srcbits;
  static uint8_t divbits;
  static uint16_t reloadSetting;
  static uint8_t eventLinkIndex;

  static void internalCallback();

public:

  static void (*isrCallback)();
  static void isrDefaultUnused(){};
  

  //****************************
  //  Configuration
  //****************************
  void initialize(unsigned long microseconds = 1000000) __attribute__((always_inline)) {

    // enable the timer in Module Stop Control Register D
    R_MSTP->MSTPCRD &= ~(1 << R_MSTP_MSTPCRD_MSTPD2_Pos);
    //  Make sure timer is stopped while we adjust registers.
    R_AGT1->AGTCR = 0;

    // We're using R_AGT1, but all the positions and bitmasks are defined as R_AGT0
    // set mode register 1
    //(-) (TCK[2:0]) (TEDGPL) (TMOD[2:0])
    //  Use TIMER mode with the LOCO clock (best we can do since Arduino doesn't have crystal for SOSC)
    R_AGT1->AGTMR1 = (AGTLCLK << R_AGT0_AGTMR1_TCK_Pos) | (TIMER << R_AGT0_AGTMR1_TMOD_Pos);
    // mode register 2
    // (LPM) (----) (CKS[2:0])
    R_AGT1->AGTMR2 = 0;
    // AGT I/O Control Register
    // (TIOGT[1:0]) (TIPF[1:0]) (-) (TOE) (-) (TEDGSEL)
    R_AGT1->AGTIOC = 0;
    // Event Pin Select Register
    // (-----) (EEPS) (--)
    R_AGT1->AGTISR = 0;
    // AGT Compare Match Function Select Register
    // (-) (TOPOLB) (TOEB) (TCMEB) (-) (TOPOLA) (TOEA) (TCMEA)
    R_AGT1->AGTCMSR = 0;
    // AGT Pin Select Register
    // (---) (TIES) (--) (SEL[1:0])
    R_AGT1->AGTIOSEL = 0;
    // AGT1_AGTI is the underflow event
    //  The event code is 0x21

    // Use IRQManager to attach a handler.  
    timer_cfg_t base;
    base.channel = 1;
    base.cycle_end_irq = FSP_INVALID_VECTOR;
    TimerIrqCfg_t iCfg;
    iCfg.base_cfg = &base;
    // to satisfy IRQManager agt_ext_cfg just has to not be null
    agt_extended_cfg_t fake;
    iCfg.agt_ext_cfg = &fake;
    iCfg.gpt_ext_cfg = nullptr;

    __disable_irq();
    if (IRQManager::getInstance().addTimerOverflow(iCfg, internalCallback)) {
      // Serial.println("Attached Interrupt.");
      eventLinkIndex = iCfg.base_cfg->cycle_end_irq;
      R_BSP_IrqDisable((IRQn_Type)eventLinkIndex);
      R_BSP_IrqStatusClear((IRQn_Type)eventLinkIndex);
      NVIC_SetPriority((IRQn_Type)eventLinkIndex, 14);
      R_BSP_IrqEnable((IRQn_Type)eventLinkIndex);
      // Serial.println(eventLinkIndex);
      setPeriod(microseconds);
    }
    __enable_irq();
  }

  void setPeriod(unsigned long microseconds) __attribute__((always_inline)) {

    // for smal periods we can use PCLKB instead of LOCO
    divbits = 0;  // No divider bits on PCLKB
    // PCLKB is running at SYSCLK/2 or 24MHz or 24 ticks per microsecond
    unsigned long ticks = (24 * microseconds);
    if (ticks < AGT1_RESOLUTION) {
      srcbits = PCLKB;
      reloadSetting = ticks;
    } else if (ticks < AGT1_RESOLUTION * 2) {
      srcbits = PCLKB_2;
      reloadSetting = ticks / 2;
    } else if (ticks < AGT1_RESOLUTION * 8) {
      srcbits = PCLKB_8;
      reloadSetting = ticks / 8;
    } else {
      //  Period is too long for PCLKB, use AGTLCLK (LOCO)
      // LOCO is 32.768KHz  is (1/32768) = 30.518us/tick
      srcbits = AGTLCLK;
      // recalculate ticks at new clock speed
      ticks = microseconds / (1000000.0 / 32768.0);
      if (ticks < AGT1_RESOLUTION) {
        divbits = 0;
        reloadSetting = ticks;
      } else if (ticks < AGT1_RESOLUTION * 2) {
        divbits = 1;
        reloadSetting = ticks / 2;
      } else if (ticks < AGT1_RESOLUTION * 4) {
        divbits = 2;
        reloadSetting = ticks / 4;
      } else if (ticks < AGT1_RESOLUTION * 8) {
        divbits = 3;
        reloadSetting = ticks / 8;
      } else if (ticks < AGT1_RESOLUTION * 16) {
        divbits = 4;
        reloadSetting = ticks / 16;
      } else if (ticks < AGT1_RESOLUTION * 32) {
        divbits = 5;
        reloadSetting = ticks / 32;
      } else if (ticks < AGT1_RESOLUTION * 64) {
        divbits = 6;
        reloadSetting = ticks / 64;
      } else if (ticks < AGT1_RESOLUTION * 128) {
        divbits = 7;
        reloadSetting = ticks / 128;
      }
    }
    //  Use TIMER mode with the LOCO clock (best we can do since Arduino doesn't have crystal for SOSC)
    R_AGT1->AGTMR1 = (srcbits << R_AGT0_AGTMR1_TCK_Pos) | (TIMER << R_AGT0_AGTMR1_TMOD_Pos);
    R_AGT1->AGTMR2 = divbits;
    R_AGT1->AGT = reloadSetting;
    start();
  }
  

  //****************************
  //  Run Control
  //****************************	
  void start() __attribute__((always_inline)) {
    resume();
  }
  void stop() __attribute__((always_inline)) {
    R_AGT1->AGTCR = 0;
  }
  void restart() __attribute__((always_inline)) {
    start();
  }
  void resume() __attribute__((always_inline)) {
    R_AGT1->AGTCR = 1;
  }

  //****************************
  //  PWM outputs
  //****************************
	//Not implemented yet for UNO-R4
	//TO DO
	
  //****************************
  //  Interrupt Function
  //****************************
  void attachInterrupt(void (*isr)()) __attribute__((always_inline)) {
    isrCallback = isr;
  }
  void attachInterrupt(void (*isr)(), unsigned long microseconds) __attribute__((always_inline)) {
    if (microseconds > 0) setPeriod(microseconds);
    attachInterrupt(isr);
  }
  void detachInterrupt() __attribute__((always_inline)) {
    isrCallback = isrDefaultUnused;
  }
	
#elif defined (__AVR_ATtiny85__)
  public:
    //****************************
    //  Configuration
    //****************************
    void initialize(unsigned long microseconds=1000000) __attribute__((always_inline)) {
	TCCR1 = _BV(CTC1);              //clear timer1 when it matches the value in OCR1C
	TIMSK |= _BV(OCIE1A);           //enable interrupt when OCR1A matches the timer value
	setPeriod(microseconds);
    }
    void setPeriod(unsigned long microseconds) __attribute__((always_inline)) {		
	const unsigned long cycles = microseconds * ratio;
	if (cycles < TIMER1_RESOLUTION) {
		clockSelectBits = _BV(CS10);
		pwmPeriod = cycles;
	} else
	if (cycles < TIMER1_RESOLUTION * 2UL) {
		clockSelectBits = _BV(CS11);
		pwmPeriod = cycles / 2;
	} else
	if (cycles < TIMER1_RESOLUTION * 4UL) {
		clockSelectBits = _BV(CS11) | _BV(CS10);
		pwmPeriod = cycles / 4;
	} else
	if (cycles < TIMER1_RESOLUTION * 8UL) {
		clockSelectBits = _BV(CS12);
		pwmPeriod = cycles / 8;
	} else
	if (cycles < TIMER1_RESOLUTION * 16UL) {
		clockSelectBits = _BV(CS12) | _BV(CS10);
		pwmPeriod = cycles / 16;
	} else
	if (cycles < TIMER1_RESOLUTION * 32UL) {
		clockSelectBits = _BV(CS12) | _BV(CS11);
		pwmPeriod = cycles / 32;
	} else
	if (cycles < TIMER1_RESOLUTION * 64UL) {
		clockSelectBits = _BV(CS12) | _BV(CS11) | _BV(CS10);
		pwmPeriod = cycles / 64UL;
	} else
	if (cycles < TIMER1_RESOLUTION * 128UL) {
		clockSelectBits = _BV(CS13);
		pwmPeriod = cycles / 128;
	} else
	if (cycles < TIMER1_RESOLUTION * 256UL) {
		clockSelectBits = _BV(CS13) | _BV(CS10);
		pwmPeriod = cycles / 256;
	} else
	if (cycles < TIMER1_RESOLUTION * 512UL) {
		clockSelectBits = _BV(CS13) | _BV(CS11);
		pwmPeriod = cycles / 512;
	} else
	if (cycles < TIMER1_RESOLUTION * 1024UL) {
		clockSelectBits = _BV(CS13) | _BV(CS11) | _BV(CS10);
		pwmPeriod = cycles / 1024;
	} else
	if (cycles < TIMER1_RESOLUTION * 2048UL) {
		clockSelectBits = _BV(CS13) | _BV(CS12);
		pwmPeriod = cycles / 2048;
	} else
	if (cycles < TIMER1_RESOLUTION * 4096UL) {
		clockSelectBits = _BV(CS13) | _BV(CS12) | _BV(CS10);
		pwmPeriod = cycles / 4096;
	} else
	if (cycles < TIMER1_RESOLUTION * 8192UL) {
		clockSelectBits = _BV(CS13) | _BV(CS12) | _BV(CS11);
		pwmPeriod = cycles / 8192;
	} else
	if (cycles < TIMER1_RESOLUTION * 16384UL) {
		clockSelectBits = _BV(CS13) | _BV(CS12) | _BV(CS11)  | _BV(CS10);
		pwmPeriod = cycles / 16384;
	} else {
		clockSelectBits = _BV(CS13) | _BV(CS12) | _BV(CS11)  | _BV(CS10);
		pwmPeriod = TIMER1_RESOLUTION - 1;
	}
	OCR1A = pwmPeriod;
	OCR1C = pwmPeriod;
	TCCR1 = _BV(CTC1) | clockSelectBits;
    }
	
    //****************************
    //  Run Control
    //****************************	
    void start() __attribute__((always_inline)) {
	TCCR1 = 0;
	TCNT1 = 0;		
	resume();
    }
    void stop() __attribute__((always_inline)) {
	TCCR1 = _BV(CTC1);
    }
    void restart() __attribute__((always_inline)) {
	start();
    }
    void resume() __attribute__((always_inline)) {
	TCCR1 = _BV(CTC1) | clockSelectBits;
    }
	
    //****************************
    //  PWM outputs
    //****************************
	//Not implemented yet for ATTiny85
	//TO DO
	
    //****************************
    //  Interrupt Function
    //****************************
    void attachInterrupt(void (*isr)()) __attribute__((always_inline)) {
	isrCallback = isr;
	TIMSK |= _BV(OCIE1A);
    }
    void attachInterrupt(void (*isr)(), unsigned long microseconds) __attribute__((always_inline)) {
	if(microseconds > 0) setPeriod(microseconds);
	attachInterrupt(isr);
    }
    void detachInterrupt() __attribute__((always_inline)) {
	//TIMSK = 0; // Timer 0 and Timer 1 both use TIMSK register so setting it to 0 will override settings for Timer1 as well
	TIMSK &= ~_BV(OCIE1A);
    }
    static void (*isrCallback)();
    static void isrDefaultUnused();

  private:
    static unsigned short pwmPeriod;
    static unsigned char clockSelectBits;
    static const byte ratio = (F_CPU)/ ( 1000000 );
	
#elif defined(__AVR__)

#if defined (__AVR_ATmega8__)
  //in some io definitions for older microcontrollers TIMSK is used instead of TIMSK1
  #define TIMSK1 TIMSK
#endif
	
  public:
    //****************************
    //  Configuration
    //****************************
    void initialize(unsigned long microseconds=1000000) __attribute__((always_inline)) {
	TCCR1B = _BV(WGM13);        // set mode as phase and frequency correct pwm, stop the timer
	TCCR1A = 0;                 // clear control register A 
	setPeriod(microseconds);
    }
    void setPeriod(unsigned long microseconds) __attribute__((always_inline)) {
	const unsigned long cycles = ((F_CPU/100000 * microseconds) / 20);
	if (cycles < TIMER1_RESOLUTION) {
		clockSelectBits = _BV(CS10);
		pwmPeriod = cycles;
	} else
	if (cycles < TIMER1_RESOLUTION * 8) {
		clockSelectBits = _BV(CS11);
		pwmPeriod = cycles / 8;
	} else
	if (cycles < TIMER1_RESOLUTION * 64) {
		clockSelectBits = _BV(CS11) | _BV(CS10);
		pwmPeriod = cycles / 64;
	} else
	if (cycles < TIMER1_RESOLUTION * 256) {
		clockSelectBits = _BV(CS12);
		pwmPeriod = cycles / 256;
	} else
	if (cycles < TIMER1_RESOLUTION * 1024) {
		clockSelectBits = _BV(CS12) | _BV(CS10);
		pwmPeriod = cycles / 1024;
	} else {
		clockSelectBits = _BV(CS12) | _BV(CS10);
		pwmPeriod = TIMER1_RESOLUTION - 1;
	}
	ICR1 = pwmPeriod;
	TCCR1B = _BV(WGM13) | clockSelectBits;
    }

    //****************************
    //  Run Control
    //****************************
    void start() __attribute__((always_inline)) {
	TCCR1B = 0;
	TCNT1 = 0;		// TODO: does this cause an undesired interrupt?
	resume();
    }
    void stop() __attribute__((always_inline)) {
	TCCR1B = _BV(WGM13);
    }
    void restart() __attribute__((always_inline)) {
	start();
    }
    void resume() __attribute__((always_inline)) {
	TCCR1B = _BV(WGM13) | clockSelectBits;
    }

    //****************************
    //  PWM outputs
    //****************************
    void setPwmDuty(char pin, unsigned int duty) __attribute__((always_inline)) {
	unsigned long dutyCycle = pwmPeriod;
	dutyCycle *= duty;
	dutyCycle >>= 10;
	if (pin == TIMER1_A_PIN) OCR1A = dutyCycle;
	#ifdef TIMER1_B_PIN
	else if (pin == TIMER1_B_PIN) OCR1B = dutyCycle;
	#endif
	#ifdef TIMER1_C_PIN
	else if (pin == TIMER1_C_PIN) OCR1C = dutyCycle;
	#endif
    }
    void pwm(char pin, unsigned int duty) __attribute__((always_inline)) {
	if (pin == TIMER1_A_PIN) { pinMode(TIMER1_A_PIN, OUTPUT); TCCR1A |= _BV(COM1A1); }
	#ifdef TIMER1_B_PIN
	else if (pin == TIMER1_B_PIN) { pinMode(TIMER1_B_PIN, OUTPUT); TCCR1A |= _BV(COM1B1); }
	#endif
	#ifdef TIMER1_C_PIN
	else if (pin == TIMER1_C_PIN) { pinMode(TIMER1_C_PIN, OUTPUT); TCCR1A |= _BV(COM1C1); }
	#endif
	setPwmDuty(pin, duty);
	TCCR1B = _BV(WGM13) | clockSelectBits;
    }
    void pwm(char pin, unsigned int duty, unsigned long microseconds) __attribute__((always_inline)) {
	if (microseconds > 0) setPeriod(microseconds);
	pwm(pin, duty);
    }
    void disablePwm(char pin) __attribute__((always_inline)) {
	if (pin == TIMER1_A_PIN) TCCR1A &= ~_BV(COM1A1);
	#ifdef TIMER1_B_PIN
	else if (pin == TIMER1_B_PIN) TCCR1A &= ~_BV(COM1B1);
	#endif
	#ifdef TIMER1_C_PIN
	else if (pin == TIMER1_C_PIN) TCCR1A &= ~_BV(COM1C1);
	#endif
    }

    //****************************
    //  Interrupt Function
    //****************************
	
    void attachInterrupt(void (*isr)()) __attribute__((always_inline)) {
	isrCallback = isr;
	TIMSK1 = _BV(TOIE1);
    }
    void attachInterrupt(void (*isr)(), unsigned long microseconds) __attribute__((always_inline)) {
	if(microseconds > 0) setPeriod(microseconds);
	attachInterrupt(isr);
    }
    void detachInterrupt() __attribute__((always_inline)) {
	TIMSK1 = 0;
    }
    static void (*isrCallback)();
    static void isrDefaultUnused();

  private:
    // properties
    static unsigned short pwmPeriod;
    static unsigned char clockSelectBits;






#elif defined(__arm__) && defined(TEENSYDUINO) && (defined(KINETISK) || defined(KINETISL))

#if defined(KINETISK)
#define F_TIMER F_BUS
#elif defined(KINETISL)
#define F_TIMER (F_PLL/2)
#endif

// Use only 15 bit resolution.  From K66 reference manual, 45.5.7 page 1200:
//   The CPWM pulse width (duty cycle) is determined by 2 x (CnV - CNTIN) and the
//   period is determined by 2 x (MOD - CNTIN). See the following figure. MOD must be
//   kept in the range of 0x0001 to 0x7FFF because values outside this range can produce
//   ambiguous results.
#undef TIMER1_RESOLUTION
#define TIMER1_RESOLUTION 32768

  public:
    //****************************
    //  Configuration
    //****************************
    void initialize(unsigned long microseconds=1000000) __attribute__((always_inline)) {
	setPeriod(microseconds);
    }
    void setPeriod(unsigned long microseconds) __attribute__((always_inline)) {
	const unsigned long cycles = (F_TIMER / 2000000) * microseconds;
  // A much faster if-else
  // This is like a binary serch tree and no more than 3 conditions are evaluated.
  // I haven't checked if this becomes significantly longer ASM than the simple ladder.
  // It looks very similar to the ladder tho: same # of if's and else's
 
  /*
  // This code does not work properly in all cases :(
  // https://github.com/PaulStoffregen/TimerOne/issues/17 
  if (cycles < TIMER1_RESOLUTION * 16) {
    if (cycles < TIMER1_RESOLUTION * 4) {
      if (cycles < TIMER1_RESOLUTION) {
        clockSelectBits = 0;
        pwmPeriod = cycles;
      }else{
        clockSelectBits = 1;
        pwmPeriod = cycles >> 1;
      }
    }else{
      if (cycles < TIMER1_RESOLUTION * 8) {
        clockSelectBits = 3;
        pwmPeriod = cycles >> 3;
      }else{
        clockSelectBits = 4;
        pwmPeriod = cycles >> 4;
      }
    }
  }else{
    if (cycles > TIMER1_RESOLUTION * 64) {
      if (cycles > TIMER1_RESOLUTION * 128) {
        clockSelectBits = 7;
        pwmPeriod = TIMER1_RESOLUTION - 1;
      }else{
        clockSelectBits = 7;
        pwmPeriod = cycles >> 7;
      }
    }
    else{
      if (cycles > TIMER1_RESOLUTION * 32) {
        clockSelectBits = 6;
        pwmPeriod = cycles >> 6;
      }else{
        clockSelectBits = 5;
        pwmPeriod = cycles >> 5;
      }
    }
  }
  */
	if (cycles < TIMER1_RESOLUTION) {
		clockSelectBits = 0;
		pwmPeriod = cycles;
	} else
	if (cycles < TIMER1_RESOLUTION * 2) {
		clockSelectBits = 1;
		pwmPeriod = cycles >> 1;
	} else
	if (cycles < TIMER1_RESOLUTION * 4) {
		clockSelectBits = 2;
		pwmPeriod = cycles >> 2;
	} else
	if (cycles < TIMER1_RESOLUTION * 8) {
		clockSelectBits = 3;
		pwmPeriod = cycles >> 3;
	} else
	if (cycles < TIMER1_RESOLUTION * 16) {
		clockSelectBits = 4;
		pwmPeriod = cycles >> 4;
	} else
	if (cycles < TIMER1_RESOLUTION * 32) {
		clockSelectBits = 5;
		pwmPeriod = cycles >> 5;
	} else
	if (cycles < TIMER1_RESOLUTION * 64) {
		clockSelectBits = 6;
		pwmPeriod = cycles >> 6;
	} else
	if (cycles < TIMER1_RESOLUTION * 128) {
		clockSelectBits = 7;
		pwmPeriod = cycles >> 7;
	} else {
		clockSelectBits = 7;
		pwmPeriod = TIMER1_RESOLUTION - 1;
	}

	uint32_t sc = FTM1_SC;
	FTM1_SC = 0;
	FTM1_MOD = pwmPeriod;
	FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_CPWMS | clockSelectBits | (sc & FTM_SC_TOIE);
    }

    //****************************
    //  Run Control
    //****************************
    void start() __attribute__((always_inline)) {
	stop();
	FTM1_CNT = 0;
	resume();
    }
    void stop() __attribute__((always_inline)) {
	FTM1_SC = FTM1_SC & (FTM_SC_TOIE | FTM_SC_CPWMS | FTM_SC_PS(7));
    }
    void restart() __attribute__((always_inline)) {
	start();
    }
    void resume() __attribute__((always_inline)) {
	FTM1_SC = (FTM1_SC & (FTM_SC_TOIE | FTM_SC_PS(7))) | FTM_SC_CPWMS | FTM_SC_CLKS(1);
    }

    //****************************
    //  PWM outputs
    //****************************
    void setPwmDuty(char pin, unsigned int duty) __attribute__((always_inline)) {
	unsigned long dutyCycle = pwmPeriod;
	dutyCycle *= duty;
	dutyCycle >>= 10;
	if (pin == TIMER1_A_PIN) {
		FTM1_C0V = dutyCycle;
	} else if (pin == TIMER1_B_PIN) {
		FTM1_C1V = dutyCycle;
	}
    }
    void pwm(char pin, unsigned int duty) __attribute__((always_inline)) {
	setPwmDuty(pin, duty);
	if (pin == TIMER1_A_PIN) {
		*portConfigRegister(TIMER1_A_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
	} else if (pin == TIMER1_B_PIN) {
		*portConfigRegister(TIMER1_B_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
	}
    }
    void pwm(char pin, unsigned int duty, unsigned long microseconds) __attribute__((always_inline)) {
	if (microseconds > 0) setPeriod(microseconds);
	pwm(pin, duty);
    }
    void disablePwm(char pin) __attribute__((always_inline)) {
	if (pin == TIMER1_A_PIN) {
		*portConfigRegister(TIMER1_A_PIN) = 0;
	} else if (pin == TIMER1_B_PIN) {
		*portConfigRegister(TIMER1_B_PIN) = 0;
	}
    }

    //****************************
    //  Interrupt Function
    //****************************
    void attachInterrupt(void (*isr)()) __attribute__((always_inline)) {
	isrCallback = isr;
	FTM1_SC |= FTM_SC_TOIE;
	NVIC_ENABLE_IRQ(IRQ_FTM1);
    }
    void attachInterrupt(void (*isr)(), unsigned long microseconds) __attribute__((always_inline)) {
	if(microseconds > 0) setPeriod(microseconds);
	attachInterrupt(isr);
    }
    void detachInterrupt() __attribute__((always_inline)) {
	FTM1_SC &= ~FTM_SC_TOIE;
	NVIC_DISABLE_IRQ(IRQ_FTM1);
    }
    static void (*isrCallback)();
    static void isrDefaultUnused();

  private:
    // properties
    static unsigned short pwmPeriod;
    static unsigned char clockSelectBits;

#undef F_TIMER

#elif defined(__arm__) && defined(TEENSYDUINO) && defined(__IMXRT1062__)

  public:
    //****************************
    //  Configuration
    //****************************
    void initialize(unsigned long microseconds=1000000) __attribute__((always_inline)) {
	setPeriod(microseconds);
    }
    void setPeriod(unsigned long microseconds) __attribute__((always_inline)) {
	uint32_t period = (float)F_BUS_ACTUAL * (float)microseconds * 0.0000005f;
	uint32_t prescale = 0;
	while (period > 32767) {
		period = period >> 1;
		if (++prescale > 7) {
			prescale = 7;	// when F_BUS is 150 MHz, longest
			period = 32767; // period is 55922 us (~17.9 Hz)
			break;
		}
	}
	//Serial.printf("setPeriod, period=%u, prescale=%u\n", period, prescale);
	FLEXPWM1_FCTRL0 |= FLEXPWM_FCTRL0_FLVL(8); // logic high = fault
	FLEXPWM1_FSTS0 = 0x0008; // clear fault status
	FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(8);
	FLEXPWM1_SM3CTRL2 = FLEXPWM_SMCTRL2_INDEP;
	FLEXPWM1_SM3CTRL = FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(prescale);
	FLEXPWM1_SM3INIT = -period;
	FLEXPWM1_SM3VAL0 = 0;
	FLEXPWM1_SM3VAL1 = period;
	FLEXPWM1_SM3VAL2 = 0;
	FLEXPWM1_SM3VAL3 = 0;
	FLEXPWM1_SM3VAL4 = 0;
	FLEXPWM1_SM3VAL5 = 0;
	FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(8) | FLEXPWM_MCTRL_RUN(8);
	pwmPeriod = period;
    }
    //****************************
    //  Run Control
    //****************************
    void start() __attribute__((always_inline)) {
	stop();
	// TODO: how to force counter back to zero?
	resume();
    }
    void stop() __attribute__((always_inline)) {
	FLEXPWM1_MCTRL &= ~FLEXPWM_MCTRL_RUN(8);
    }
    void restart() __attribute__((always_inline)) {
	start();
    }
    void resume() __attribute__((always_inline)) {
	FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_RUN(8);
    }

    //****************************
    //  PWM outputs
    //****************************
    void setPwmDuty(char pin, unsigned int duty) __attribute__((always_inline)) {
	if (duty > 1023) duty = 1023;
	int dutyCycle = (pwmPeriod * duty) >> 10;
	//Serial.printf("setPwmDuty, period=%u\n", dutyCycle);
	if (pin == TIMER1_A_PIN) {
		FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(8);
		FLEXPWM1_SM3VAL5 = dutyCycle;
		FLEXPWM1_SM3VAL4 = -dutyCycle;
		FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(8);
	} else if (pin == TIMER1_B_PIN) {
		FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(8);
		FLEXPWM1_SM3VAL3 = dutyCycle;
		FLEXPWM1_SM3VAL2 = -dutyCycle;
		FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(8);
	}
    }
    void pwm(char pin, unsigned int duty) __attribute__((always_inline)) {
	setPwmDuty(pin, duty);
	if (pin == TIMER1_A_PIN) {
		FLEXPWM1_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(8);
		IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 6; // pin 7 FLEXPWM1_PWM3_B
	} else if (pin == TIMER1_B_PIN) {
		FLEXPWM1_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(8);
		IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00 = 6; // pin 8 FLEXPWM1_PWM3_A
	}
    }
    void pwm(char pin, unsigned int duty, unsigned long microseconds) __attribute__((always_inline)) {
	if (microseconds > 0) setPeriod(microseconds);
	pwm(pin, duty);
    }
    void disablePwm(char pin) __attribute__((always_inline)) {
	if (pin == TIMER1_A_PIN) {
		IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 5; // pin 7 FLEXPWM1_PWM3_B
		FLEXPWM1_OUTEN &= ~FLEXPWM_OUTEN_PWMB_EN(8);
	} else if (pin == TIMER1_B_PIN) {
		IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00 = 5; // pin 8 FLEXPWM1_PWM3_A
		FLEXPWM1_OUTEN &= ~FLEXPWM_OUTEN_PWMA_EN(8);
	}
    }
    //****************************
    //  Interrupt Function
    //****************************
    void attachInterrupt(void (*f)()) __attribute__((always_inline)) {
	isrCallback = f;
	attachInterruptVector(IRQ_FLEXPWM1_3, &isr);
	FLEXPWM1_SM3STS = FLEXPWM_SMSTS_RF;
	FLEXPWM1_SM3INTEN = FLEXPWM_SMINTEN_RIE;
	NVIC_ENABLE_IRQ(IRQ_FLEXPWM1_3);
    }
    void attachInterrupt(void (*f)(), unsigned long microseconds) __attribute__((always_inline)) {
	if(microseconds > 0) setPeriod(microseconds);
	attachInterrupt(f);
    }
    void detachInterrupt() __attribute__((always_inline)) {
	NVIC_DISABLE_IRQ(IRQ_FLEXPWM1_3);
	FLEXPWM1_SM3INTEN = 0;
    }
    static void isr(void);
    static void (*isrCallback)();
    static void isrDefaultUnused();

  private:
    // properties
    static unsigned short pwmPeriod;
    static unsigned char clockSelectBits;

#endif
};

extern TimerOne Timer1;

#endif

