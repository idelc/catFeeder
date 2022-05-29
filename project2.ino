#include "ESP8266TimerInterrupt.h"  // Timer libraries
#include "ESP8266_ISR_Timer.hpp"   

#include <SPI.h>                    // RFID libraries
#include <MFRC522.h>

#include <Stepper.h>                // Stepper library

// Timer definitions (Prescalers)
#define USING_TIM_DIV1                false           // for shortest and most accurate timer
#define USING_TIM_DIV16               false           // for medium time and medium accurate timer
#define USING_TIM_DIV256              true            // for longest timer but least accurate. Default

// RFID definitions
#define RST_PIN 2
#define SS_PIN  15

// Stepper definitions
#define stepsPerRev 64


// Init ESP8266 only and only Timer 1
ESP8266Timer ITimer; 
// Timer interval (in microseconds)
bool setInterval(unsigned long interval, timer_callback callback);
// Timer interval (in microseconds)
bool attachInterruptInterval(unsigned long interval, timer_callback callback);

void IRAM_ATTR TimerHandler()
{
  // Doing something here inside ISR
}

// Init RFID
MFRC522 mfrc522(SS_PIN, RST_PIN); 

// Init Stepper Motor
Stepper lidMotor(stepsPerRev, 16, 5, 4, 0);

void setup()
{ 
// Timer Init
  // Set Interval in microsecs
  /*
  if (!ITimer.attachInterruptInterval(timerGCD * 1000, TimerHandler))
  {
    exit(1);
  }
  */
// RFID Init
  // SPI.begin();
  // mfrc522.PCD_Init();
  // delay(4);
  // mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max); // max read distance
// Stepper Motor Init
// lidMotor.setSpeed(30); // arbitrary speed, 30 rpm
}  

void loop(){

}
