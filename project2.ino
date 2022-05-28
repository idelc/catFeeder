#include "ESP8266TimerInterrupt.h"
#include "ESP8266_ISR_Timer.hpp"   

#define USING_TIM_DIV1                false           // for shortest and most accurate timer
#define USING_TIM_DIV16               false           // for medium time and medium accurate timer
#define USING_TIM_DIV256              true            // for longest timer but least accurate. Default

// Init ESP8266 only and only Timer 1
ESP8266Timer ITimer;

// interval (in microseconds)
bool setInterval(unsigned long interval, timer_callback callback);

// interval (in microseconds)
bool attachInterruptInterval(unsigned long interval, timer_callback callback);

bool led = true;
void IRAM_ATTR TimerHandler()
{
  // Doing something here inside ISR
  if(led){
      digitalWrite(LED_BUILTIN, HIGH);
  }
  else{
      digitalWrite(LED_BUILTIN, LOW);
  }
  led = !led;
}

#define TIMER_INTERVAL_MS        1000

unsigned long lastMillis = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Interval in microsecs
  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_MS * 1000, TimerHandler))
  {
    lastMillis = millis();
    Serial.print(F("Starting  ITimer OK, millis() = ")); Serial.println(lastMillis);
  }
  else
    Serial.println(F("Can't set ITimer correctly. Select another freq. or interval"));
}  

void loop(){

}

// #include <SPI.h>
// #include <MFRC522.h>

// #define RST_PIN 2
// #define SS_PIN  15

// MFRC522 mfrc522(SS_PIN, RST_PIN);

// void setup(){
//     Serial.begin(9600);
//     while(!Serial);
//     SPI.begin();
//     mfrc522.PCD_Init();
//     delay(4);
//     mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
//     mfrc522.PCD_DumpVersionToSerial();
//     Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
// }

// int gain = 0;
// void loop(){
//     if(!mfrc522.PICC_IsNewCardPresent()){
//         return;
//     }
//     if(!mfrc522.PICC_ReadCardSerial()){
//         return;
//     }
//     mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
//     gain = mfrc522.PCD_GetAntennaGain();
//     Serial.print("Antena Gain: ");
//     Serial.println(gain);
//     delay(1000);
// }


// #include <Stepper.h>

// const int stepsPerRev = 64;

// Stepper lidMotor(stepsPerRev, 16, 5, 4, 0);

// void setup(){
//     lidMotor.setSpeed(30);
// }

// void loop(){
//     lidMotor.step(32);
//     delay(2000);
//     lidMotor.step(-32);
//     delay(2000;)
// }