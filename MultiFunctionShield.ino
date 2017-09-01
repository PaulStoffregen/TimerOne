//This project is made by Kutay Yavuz.
//https://github.com/kutaygs/Multi-Function-Shield-Arduino
//Please contact me to be contributor(kutayyavuz03@hotmail.com).

#include <TimerOne.h>
#include <Wire.h>
#include <MultiFuncShield.h>
enum CountDownModeValues
{
 COUNTING_STOPPED,
 COUNTING
};
byte countDownMode = COUNTING_STOPPED;
byte tenths = 0;
char seconds = 0;
char minutes = 0;
void setup() {
 // put your setup code here, to run once:
 Timer1.initialize();
 MFS.initialize(&Timer1);
 MFS.write(0);

 Serial.begin(9600);
}
void loop() {
 byte btn = MFS.getButton();

 switch (countDownMode)
 {
 case COUNTING_STOPPED:
 if (btn == BUTTON_1_SHORT_RELEASE && (minutes + seconds) > 0)
 {
 countDownMode = COUNTING;
 }
 else if (btn == BUTTON_1_LONG_PRESSED)
 {
 tenths = 0;
 seconds = 0;
 minutes = 0;
 MFS.write(minutes*100 + seconds);
 }
 else if (btn == BUTTON_2_PRESSED || btn == BUTTON_2_LONG_PRESSED)
 {
 minutes++;
 if (minutes > 60)
 {
 minutes = 0;
 }
 MFS.write(minutes*100 + seconds);
 }
 else if (btn == BUTTON_3_PRESSED || btn == BUTTON_3_LONG_PRESSED)
 {
 seconds += 10;
 if (seconds >= 60)
 {
 seconds = 0;
 }
 MFS.write(minutes*100 + seconds);
 }
 break;

 case COUNTING:
 if (btn == BUTTON_1_SHORT_RELEASE || btn == BUTTON_1_LONG_RELEASE)
 {
 countDownMode = COUNTING_STOPPED;
 }
 else
 {
 tenths++;

 if (tenths == 10)
 {
 tenths = 0;
 seconds--;

 if (seconds < 0 && minutes > 0)
 {
 seconds = 59;
 minutes--;
 }

 if (minutes == 0 && seconds == 0)
 {
 MFS.beep(50, 50, 3); 
 countDownMode = COUNTING_STOPPED;
 }

 MFS.write(minutes*100 + seconds);
 }
 delay(100);
 }
 break;
 }
}
