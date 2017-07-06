#include <TimerOne.h>
#define ledPin 4
int ledState = LOW;

void setup() {
  pinMode(ledPin, OUTPUT);
  Timer1.initialize(500000); //The led will blink in a half second time interval
  Timer1.attachInterrupt(blinkLed);
}

void loop() {
}

void blinkLed(){
  ledState = !ledState;
  digitalWrite(ledPin, ledState);
}


