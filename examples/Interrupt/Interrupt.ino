#include <TimerOne.h>

const int led = LED_BUILTIN;  // the pin with a LED


// The interrupt will blink the LED, and keep
// track of how many times it has blinked.
int ledState = LOW;
volatile unsigned long blinkCount = 0; // use volatile for shared variables

void blinkLED(void)
{
  if (ledState == LOW) {
    ledState = HIGH;
    blinkCount = blinkCount + 1;  // increase when LED turns on
  } else {
    ledState = LOW;
  }
  digitalWrite(led, ledState);
}



void setup(void)
{
  pinMode(led, OUTPUT);
  //Timer1.initialize(150000);
  //Timer1.attachInterrupt(blinkLED); // blinkLED to run every 0.15 seconds
  //Timer1.initialize(150000).attachInterrupt(blinkLED); //alternative way to enable the timer in one line
  Timer1.setFrequency(6.66).attachInterrupt(blinkLED); //alternative way to enable the timer by frequency
  Serial.begin(9600);
}



// The main program will print the blink count
// to the Arduino Serial Monitor
void loop(void)
{
  unsigned long blinkCopy;  // holds a copy of the blinkCount

  // to read a variable which the interrupt code writes, we
  // must temporarily disable interrupts, to be sure it will
  // not change while we are reading.  To minimize the time
  // with interrupts off, just quickly make a copy, and then
  // use the copy while allowing the interrupt to keep working.
  //noInterrupts();
  //blinkCopy = blinkCount;
  //interrupts();

  Serial.print("blinkCount = ");
  Serial.println(blinkCopy);
  delay(100);

}
