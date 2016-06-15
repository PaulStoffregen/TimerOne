// This program is for testing purposes.
// All setPeriods() have been commented out.
// By selectively uncommenting, and changing arguments you can test
// various scenarios.
// 
// Contact @arrow- (ananya95@gmail.com or @jukedude on pjrc forum)

#include <TimerOne.h>

void setup(void)
{
  pinMode(13, OUTPUT);
  Timer1.initialize(20000);
  Timer1.attachInterrupt(blinkLED);
  Serial.begin(115200);
}

// The interrupt will blink the LED, and
// mainloop will print FTM1 configuration
unsigned long vv[] = {120000, 2000};

unsigned long last_event_micros = 0;
volatile uint8_t lstate = 0, wflag = 0;

void blinkLED(void)
{
  lstate = !lstate;
  wflag = 1;
  digitalWrite(13, lstate);
  /*
  if (lstate)
    FTM1_MOD = 65535;
  else
    FTM1_MOD = 324;
  */
  //Timer1.setPeriod(vv[lstate], 1); // this is a frequent update
}

void loop(void)
{
  if (wflag == 1){
    Serial.print(micros() - last_event_micros); // slight errors (upto 10us) acceptable
    last_event_micros = micros();
    Serial.print(" ");
    Serial.print(FTM1_MOD);
    Serial.print(" ");
    Serial.println(FTM1_SC & 0x7);
    wflag = 0;

    //Timer1.setPeriod(11650, 1); // this is a frequent update
  }
  /*
  delay(4000);
  Timer1.setPeriod(80000); // this is not a frequent update -- because of above delay.
                           // the printout value will be wrong but you can verify from
                           // MOD*2*PS/F_BUS (in sec) == setPeriod's argument (in usec)
  */
}
